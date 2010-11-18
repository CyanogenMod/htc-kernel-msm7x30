/* linux/arch/arm/mach-msm/devices.c
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/irqs.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/board.h>
#ifdef CONFIG_MSM_RMT_STORAGE_SERVER
#include "smd_private.h"
#endif
#include <asm/mach/flash.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/mmc.h>
#include <asm/setup.h>

#include "devices.h"
#include "clock.h"
#include "proc_comm.h"
#include <mach/msm_hsusb.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb_hw.h>
#ifdef CONFIG_USB_FUNCTION
#include <linux/usb/mass_storage_function.h>
#endif
#ifdef CONFIG_PMIC8058
#include <linux/mfd/pmic8058.h>
#endif

static char *df_serialno = "000000000000";
static char *board_sn;
int usb_phy_error;

#define HSUSB_API_INIT_PHY_PROC	2
#define HSUSB_API_PROG		0x30000064
#define HSUSB_API_VERS MSM_RPC_VERS(1, 1)

#define MFG_GPIO_TABLE_MAX_SIZE        0x400
static unsigned char mfg_gpio_table[MFG_GPIO_TABLE_MAX_SIZE];

static void internal_phy_reset(void)
{
	struct msm_rpc_endpoint *usb_ep;
	int rc;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
	} req;

	printk(KERN_INFO "msm_hsusb_phy_reset\n");

	usb_ep = msm_rpc_connect(HSUSB_API_PROG, HSUSB_API_VERS, 0);
	if (IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(usb_ep));
		return;
	}
	rc = msm_rpc_call(usb_ep, HSUSB_API_INIT_PHY_PROC,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

	msm_rpc_close(usb_ep);
}

static void *usb_base;
#define MSM_USB_BASE              ((unsigned)usb_base)
static unsigned ulpi_read(void __iomem *usb_base, unsigned reg)
{
	unsigned timeout = 100000;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_read: timeout %08x\n",
			readl(USB_ULPI_VIEWPORT));
		return 0xffffffff;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(void __iomem *usb_base, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_write: timeout\n");
		return -1;
	}

	return 0;
}

#define CLKRGM_APPS_RESET_USBH      37
#define CLKRGM_APPS_RESET_USB_PHY   34
static void msm_hsusb_apps_reset_link(int reset)
{
	int ret;
	unsigned usb_id = CLKRGM_APPS_RESET_USBH;

	if (reset)
		ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT,
				&usb_id, NULL);
	else
		ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
				&usb_id, NULL);
	if (ret)
		printk(KERN_INFO "%s: Cannot set reset to %d (%d)\n",
			__func__, reset, ret);
}

static void msm_hsusb_apps_reset_phy(void)
{
	int ret;
	unsigned usb_phy_id = CLKRGM_APPS_RESET_USB_PHY;

	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT,
			&usb_phy_id, NULL);
	if (ret) {
		printk(KERN_INFO "%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
	msleep(1);
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
			&usb_phy_id, NULL);
	if (ret) {
		printk(KERN_INFO "%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
}

#define ULPI_VERIFY_MAX_LOOP_COUNT  3
static int msm_hsusb_phy_verify_access(void __iomem *usb_base)
{
	int temp;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		if (ulpi_read(usb_base, ULPI_DEBUG) != (unsigned)-1)
			break;
		msm_hsusb_apps_reset_phy();
	}

	if (temp == ULPI_VERIFY_MAX_LOOP_COUNT) {
		pr_err("%s: ulpi read failed for %d times\n",
				__func__, ULPI_VERIFY_MAX_LOOP_COUNT);
		return -1;
	}

	return 0;
}

static unsigned msm_hsusb_ulpi_read_with_reset(void __iomem *usb_base, unsigned reg)
{
	int temp;
	unsigned res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = ulpi_read(usb_base, reg);
		if (res != -1)
			return res;
		msm_hsusb_apps_reset_phy();
	}

	pr_err("%s: ulpi read failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);

	return -1;
}

static int msm_hsusb_ulpi_write_with_reset(void __iomem *usb_base,
		unsigned val, unsigned reg)
{
	int temp;
	int res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = ulpi_write(usb_base, val, reg);
		if (!res)
			return 0;
		msm_hsusb_apps_reset_phy();
	}

	pr_err("%s: ulpi write failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);
	return -1;
}

static int msm_hsusb_phy_caliberate(void __iomem *usb_base)
{
	int ret;
	unsigned res;

	ret = msm_hsusb_phy_verify_access(usb_base);
	if (ret)
		return -ETIMEDOUT;

	res = msm_hsusb_ulpi_read_with_reset(usb_base, ULPI_FUNC_CTRL_CLR);
	if (res == -1)
		return -ETIMEDOUT;

	res = msm_hsusb_ulpi_write_with_reset(usb_base,
			res | ULPI_SUSPENDM,
			ULPI_FUNC_CTRL_CLR);
	if (res)
		return -ETIMEDOUT;

	msm_hsusb_apps_reset_phy();

	return msm_hsusb_phy_verify_access(usb_base);
}

#define USB_LINK_RESET_TIMEOUT      (msecs_to_jiffies(10))
void msm_hsusb_8x50_phy_reset(void)
{
	u32 temp;
	unsigned long timeout;
	printk(KERN_INFO "msm_hsusb_phy_reset\n");
	usb_base = ioremap(MSM_HSUSB_PHYS, 4096);

	msm_hsusb_apps_reset_link(1);
	msm_hsusb_apps_reset_phy();
	msm_hsusb_apps_reset_link(0);

	/* select ULPI phy */
	temp = (readl(USB_PORTSC) & ~PORTSC_PTS);
	writel(temp | PORTSC_PTS_ULPI, USB_PORTSC);

	if (msm_hsusb_phy_caliberate(usb_base)) {
		usb_phy_error = 1;
		return;
	}

	/* soft reset phy */
	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	while (readl(USB_USBCMD) & USBCMD_RESET) {
		if (time_after(jiffies, timeout)) {
			pr_err("usb link reset timeout\n");
			break;
		}
		msleep(1);
	}
	usb_phy_error = 0;

	return;
}

/* adjust eye diagram, disable vbusvalid interrupts */
static int hsusb_phy_init_seq[] = { 0x1D, 0x0D, 0x1D, 0x10, -1 };

#ifdef CONFIG_USB_FUNCTION
static char *usb_functions[] = {
#if defined(CONFIG_USB_FUNCTION_MASS_STORAGE) || defined(CONFIG_USB_FUNCTION_UMS)
	"usb_mass_storage",
#endif
#if defined(CONFIG_USB_FUNCTION_ADB)
	"adb",
#endif
#if defined(CONFIG_USB_FUNCTION_DIAG)
	"diag",
#endif
#if defined(CONFIG_USB_FUNCTION_MODEM)
	"serial",
#endif
#if defined(CONFIG_USB_FUNCTION_PROJECTOR)
	"projector",
#endif
#if defined(CONFIG_USB_FUNCTION_MTP_TUNNEL)
	"mtp_tunnel",
#endif
#if defined(CONFIG_USB_FUNCTION_ETHER)
	"ether",
#endif
#if defined(CONFIG_USB_FUNCTION_MODEM)
	"modem",
	"nmea",
#endif
};

static struct msm_hsusb_product usb_products[] = {
	{
		.product_id	= 0x0ff9,
		.functions	= 0x00000001, /* usb_mass_storage */
	},
	{
		.product_id	= 0x0c02,
		.functions	= 0x00000003, /* usb_mass_storage + adb */
	},
	{
		.product_id	= 0x0c03,
		.functions	= 0x00000101, /* modem + mass_storage */
	},
	{
		.product_id	= 0x0c04,
		.functions	= 0x00000103, /* modem + adb + mass_storage */
	},
	{
		.product_id = 0x0c05,
		.functions	= 0x00000021, /* Projector + mass_storage */
	},
	{
		.product_id = 0x0c06,
		.functions	= 0x00000023, /* Projector + adb + mass_storage */
	},
	{
		.product_id	= 0x0c07,
		.functions	= 0x0000000B, /* diag + adb + mass_storage */
	},
	{
		.product_id = 0x0c08,
		.functions	= 0x00000009, /* diag + mass_storage */
	},
	{
		.product_id = 0x0c88,
		.functions	= 0x0000010B, /* adb + mass_storage + diag + modem */
	},
	{
	       .product_id = 0x0c89,
	       .functions      = 0x00000019, /* serial + diag + mass_storage */
	},
	{
	       .product_id = 0x0c8a,
	       .functions      = 0x0000001B, /* serial + diag + adb + mass_storage */
	},
	{
		.product_id = 0x0c93,
		.functions	= 0x00000080, /* mtp */
	},
	{
		.product_id = 0x0FFE,
		.functions	= 0x00000004, /* internet sharing */
	},
};
#endif

struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_reset = internal_phy_reset,
	.phy_init_seq = hsusb_phy_init_seq,
#ifdef CONFIG_USB_FUNCTION
	.vendor_id = 0x0bb4,
	.product_id = 0x0c02,
	.version = 0x0100,
	.product_name = "Android Phone",
	.manufacturer_name = "HTC",

	.functions = usb_functions,
	.num_functions = ARRAY_SIZE(usb_functions),
	.products = usb_products,
	.num_products = ARRAY_SIZE(usb_products),
#endif
};

#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns = 1,
	.buf_size = 16384,
	.vendor = "HTC     ",
	.product = "Android Phone   ",
	.release = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
		},
};
#endif

static struct resource resources_hsusb[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
#ifdef CONFIG_ARCH_MSM7X30
	{
		.name	= "vbus_on",
		.start	= PM8058_IRQ_CHGVAL,
		.end	= PM8058_IRQ_CHGVAL,
		.flags	= IORESOURCE_IRQ,
	},
#endif
};

struct platform_device msm_device_hsusb = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb),
	.resource	= resources_hsusb,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
		.platform_data = &msm_hsusb_pdata,
	},
};

void __init msm_add_usb_devices(void (*phy_reset) (void), void (*phy_shutdown) (void))
{
	/* setup */
	msm_hsusb_pdata.phy_reset = phy_reset;
	msm_hsusb_pdata.phy_shutdown = phy_shutdown;

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_device_register(&msm_device_hsusb);
#ifdef CONFIG_USB_FUNCTION
	platform_device_register(&usb_mass_storage_device);
#endif
}

#ifdef CONFIG_USB_FUNCTION
void __init msm_set_ums_device_id(int id)
{
	usb_mass_storage_device.id = id;
}

void __init msm_add_usb_id_pin_function(void (*config_usb_id_gpios)(bool enable))
{
	/* setup */
	msm_hsusb_pdata.config_usb_id_gpios = config_usb_id_gpios;

}
void __init msm_enable_car_kit_detect(bool enable)
{
	msm_hsusb_pdata.enable_car_kit_detect = enable;
}

void __init msm_init_ums_lun(int lun_num)
{
	if (lun_num > 0)
		mass_storage_pdata.nluns = lun_num;
}

void __init msm_register_usb_phy_init_seq(int *int_seq)
{
	if (int_seq)
		msm_hsusb_pdata.phy_init_seq = int_seq;
}

void __init msm_register_uart_usb_switch(void (*usb_uart_switch) (int))
{
	if (usb_uart_switch)
		msm_hsusb_pdata.usb_uart_switch = usb_uart_switch;
}

void __init msm_add_usb_id_pin_gpio(int usb_id_pin_io)
{
	msm_hsusb_pdata.usb_id_pin_gpio = usb_id_pin_io;
}

void __init msm_hsusb_set_product(struct msm_hsusb_product *product,
			int num_products) {
	msm_hsusb_pdata.products = product;
	msm_hsusb_pdata.num_products = num_products;
}
#endif

static struct resource resources_uart1[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart2[] = {
	{
		.start	= INT_UART2,
		.end	= INT_UART2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2_PHYS,
		.end	= MSM_UART2_PHYS + MSM_UART2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart3[] = {
	{
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART3_PHYS,
		.end	= MSM_UART3_PHYS + MSM_UART3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart1 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart1),
	.resource	= resources_uart1,
};

struct platform_device msm_device_uart2 = {
	.name	= "msm_serial",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(resources_uart2),
	.resource	= resources_uart2,
};

struct platform_device msm_device_uart3 = {
	.name	= "msm_serial",
	.id	= 2,
	.num_resources	= ARRAY_SIZE(resources_uart3),
	.resource	= resources_uart3,
};

static struct resource msm_uart1_dm_resources[] = {
	{
		.start = MSM_UART1DM_PHYS,
		.end   = MSM_UART1DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART1DM_IRQ,
		.end   = INT_UART1DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART1DM_RX,
		.end   = INT_UART1DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART1_TX_CHAN,
		.end   = DMOV_HSUART1_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART1_TX_CRCI,
		.end   = DMOV_HSUART1_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm1_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm1 = {
	.name = "msm_serial_hs",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart1_dm_resources),
	.resource = msm_uart1_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart2_dm_resources[] = {
	{
		.start = MSM_UART2DM_PHYS,
		.end   = MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART2DM_IRQ,
		.end   = INT_UART2DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART2DM_RX,
		.end   = INT_UART2DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART2_TX_CHAN,
		.end   = DMOV_HSUART2_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART2_TX_CRCI,
		.end   = DMOV_HSUART2_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm2_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm2 = {
	.name = "msm_serial_hs",
	.id = 1,
	.num_resources = ARRAY_SIZE(msm_uart2_dm_resources),
	.resource = msm_uart2_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm2_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

#ifdef CONFIG_ARCH_MSM7X30
static struct resource resources_i2c_2[] = {
	{
		.start	= MSM_I2C_2_PHYS,
		.end	= MSM_I2C_2_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C_2,
		.end	= INT_PWB_I2C_2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c_2 = {
	.name		= "msm_i2c",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_i2c_2),
	.resource	= resources_i2c_2,
};
#endif

static struct resource resources_i2c[] = {
	{
		.start	= MSM_I2C_PHYS,
		.end	= MSM_I2C_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C,
		.end	= INT_PWB_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c = {
	.name		= "msm_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_i2c),
	.resource	= resources_i2c,
};

#ifdef CONFIG_ARCH_MSM7X30
static struct resource resources_qup[] = {
	{
		.name   = "qup_phys_addr",
		.start	= MSM_QUP_PHYS,
		.end	= MSM_QUP_PHYS + MSM_QUP_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI_QUP_I2C_PHYS,
		.end	= MSM_GSBI_QUP_I2C_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "qup_in_intr",
		.start	= INT_PWB_QUP_IN,
		.end	= INT_PWB_QUP_IN,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "qup_out_intr",
		.start	= INT_PWB_QUP_OUT,
		.end	= INT_PWB_QUP_OUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "qup_err_intr",
		.start	= INT_PWB_QUP_ERR,
		.end	= INT_PWB_QUP_ERR,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device qup_device_i2c = {
	.name		= "qup_i2c",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_qup),
	.resource	= resources_qup,
};
#endif
#ifdef CONFIG_SPI_QSD_NEW
static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA8000000,
		.end	= 0xA8000000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

struct platform_device qsdnew_device_spi = {
	.name		= "spi_qsd_new",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};
#endif

#ifdef CONFIG_I2C_SSBI
#define MSM_SSBI6_PHYS	0xAD900000
static struct resource msm_ssbi6_resources[] = {
	{
		.name   = "ssbi_base",
		.start	= MSM_SSBI6_PHYS,
		.end	= MSM_SSBI6_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi6 = {
	.name		= "i2c_ssbi",
	.id		= 6,
	.num_resources	= ARRAY_SIZE(msm_ssbi6_resources),
	.resource	= msm_ssbi6_resources,
};

#define MSM_SSBI7_PHYS  0xAC800000
static struct resource msm_ssbi7_resources[] = {
	{
		.name   = "ssbi_base",
		.start  = MSM_SSBI7_PHYS,
		.end    = MSM_SSBI7_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi7 = {
	.name		= "i2c_ssbi",
	.id		= 7,
	.num_resources	= ARRAY_SIZE(msm_ssbi7_resources),
	.resource	= msm_ssbi7_resources,
};
#endif /* CONFIG_I2C_SSBI */

#if defined(CONFIG_ARCH_MSM7X30)
#define GPIO_I2C_CLK 70
#define GPIO_I2C_DAT 71
#elif defined(CONFIG_ARCH_QSD8X50)
#define GPIO_I2C_CLK 95
#define GPIO_I2C_DAT 96
#else
#define GPIO_I2C_CLK 60
#define GPIO_I2C_DAT 61
#endif

void msm_i2c_gpio_init(void)
{
	gpio_request(GPIO_I2C_CLK, "i2c_clk");
	gpio_request(GPIO_I2C_DAT, "i2c_data");
}

void msm_set_i2c_mux(bool gpio, int *gpio_clk, int *gpio_dat, int clk_str, int dat_str)
{
	unsigned id;
	if (gpio) {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		*gpio_clk = GPIO_I2C_CLK;
		*gpio_dat = GPIO_I2C_DAT;
	} else {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 1, GPIO_INPUT,
				   GPIO_NO_PULL, clk_str);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT , 1, GPIO_INPUT,
				   GPIO_NO_PULL, dat_str);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

struct flash_platform_data msm_nand_data = {
	.parts		= NULL,
	.nr_parts	= 0,
};

static struct resource resources_nand[] = {
	[0] = {
		.start	= 7,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_nand = {
	.name		= "msm_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_nand),
	.resource	= resources_nand,
	.dev		= {
		.platform_data	= &msm_nand_data,
	},
};

struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.id	= -1,
};

static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_PHYS,
		.end	= MSM_SDC1_PHYS + MSM_SDC1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC1_0,
		.end	= INT_SDC1_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
	{
		.start	= INT_SDC1_1,
		.end	= INT_SDC1_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc2[] = {
	{
		.start	= MSM_SDC2_PHYS,
		.end	= MSM_SDC2_PHYS + MSM_SDC2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC2_0,
		.end	= INT_SDC2_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC2_1,
		.end	= INT_SDC2_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc3[] = {
	{
		.start	= MSM_SDC3_PHYS,
		.end	= MSM_SDC3_PHYS + MSM_SDC3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC3_0,
		.end	= INT_SDC3_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC3_1,
		.end	= INT_SDC3_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc4[] = {
	{
		.start	= MSM_SDC4_PHYS,
		.end	= MSM_SDC4_PHYS + MSM_SDC4_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC4_0,
		.end	= INT_SDC4_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC4_1,
		.end	= INT_SDC4_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_sdc1 = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_sdc1),
	.resource	= resources_sdc1,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc2 = {
	.name		= "msm_sdcc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_sdc2),
	.resource	= resources_sdc2,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc3 = {
	.name		= "msm_sdcc",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(resources_sdc3),
	.resource	= resources_sdc3,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc4 = {
	.name		= "msm_sdcc",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_sdc4),
	.resource	= resources_sdc4,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags)
{
	struct platform_device	*pdev;
	struct resource *res;

	if (controller < 1 || controller > 4)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "status_irq");
	if (!res)
		return -EINVAL;
	else if (stat_irq) {
		res->start = res->end = stat_irq;
		res->flags &= ~IORESOURCE_DISABLED;
		res->flags |= stat_irq_flags;
	}

#ifdef CONFIG_MMC_SUPPORT_EXTERNEL_DRIVER
	if (plat->use_ext_sdiodrv)
		pdev->name = plat->ext_sdiodrv_name;
#endif

	return platform_device_register(pdev);
}

#ifdef CONFIG_MSM_RMT_STORAGE_SERVER
#define RAMFS_INFO_MAGICNUMBER		0x654D4D43
#define RAMFS_INFO_VERSION		0x00000001
#define RAMFS_MODEMSTORAGE_ID		0x4D454653

static void __init msm_register_device(struct platform_device *pdev, void *data)
{
	int ret;

	pdev->dev.platform_data = data;

	ret = platform_device_register(pdev);
	if (ret)
		dev_err(&pdev->dev,
			  "%s: platform_device_register() failed = %d\n",
			  __func__, ret);
}

static struct resource rmt_storage_resources[] = {
       {
		.flags  = IORESOURCE_MEM,
       },
};

static struct platform_device rmt_storage_device = {
       .name           = "rmt_storage",
       .id             = -1,
       .num_resources  = ARRAY_SIZE(rmt_storage_resources),
       .resource       = rmt_storage_resources,
};

int __init rmt_storage_add_ramfs(void)
{
	struct shared_ramfs_table *ramfs_table;
	struct shared_ramfs_entry *ramfs_entry;
	int index;

	ramfs_table = smem_alloc(SMEM_SEFS_INFO,
			sizeof(struct shared_ramfs_table));

	if (!ramfs_table) {
		printk(KERN_WARNING "%s: No RAMFS table in SMEM\n", __func__);
		return -ENOENT;
	}

	if ((ramfs_table->magic_id != (u32) RAMFS_INFO_MAGICNUMBER) ||
		(ramfs_table->version != (u32) RAMFS_INFO_VERSION)) {
		printk(KERN_WARNING "%s: Magic / Version mismatch:, "
		       "magic_id=%#x, format_version=%#x\n", __func__,
		       ramfs_table->magic_id, ramfs_table->version);
		return -ENOENT;
	}

	for (index = 0; index < ramfs_table->entries; index++) {
		ramfs_entry = &ramfs_table->ramfs_entry[index];

		/* Find a match for the Modem Storage RAMFS area */
		if (ramfs_entry->client_id == (u32) RAMFS_MODEMSTORAGE_ID) {
			printk(KERN_INFO "%s: RAMFS Info (from SMEM): "
				"Baseaddr = 0x%08x, Size = 0x%08x\n", __func__,
				ramfs_entry->base_addr, ramfs_entry->size);

			rmt_storage_resources[0].start = ramfs_entry->base_addr;
			rmt_storage_resources[0].end = ramfs_entry->base_addr +
							ramfs_entry->size - 1;
			msm_register_device(&rmt_storage_device, ramfs_entry);
			return 0;
		}
	}
	return -ENOENT;
}
#endif

static struct resource resources_mddi0[] = {
	{
		.start	= MSM_PMDH_PHYS,
		.end	= MSM_PMDH_PHYS + MSM_PMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_PRI,
		.end	= INT_MDDI_PRI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource resources_mddi1[] = {
	{
		.start	= MSM_EMDH_PHYS,
		.end	= MSM_EMDH_PHYS + MSM_EMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_EXT,
		.end	= INT_MDDI_EXT,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mddi0 = {
	.name = "msm_mddi",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mddi0),
	.resource = resources_mddi0,
	.dev            = {
		.coherent_dma_mask      = 0xffffffff,
	},
};

struct platform_device msm_device_mddi1 = {
	.name = "msm_mddi",
	.id = 1,
	.num_resources = ARRAY_SIZE(resources_mddi1),
	.resource = resources_mddi1,
	.dev            = {
		.coherent_dma_mask      = 0xffffffff,
	}
};

static struct resource resources_mdp[] = {
	{
		.start	= MSM_MDP_PHYS,
		.end	= MSM_MDP_PHYS + MSM_MDP_SIZE - 1,
		.name	= "mdp",
		.flags	= IORESOURCE_MEM
	},
	{
		.start	= INT_MDP,
		.end	= INT_MDP,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mdp = {
	.name = "msm_mdp",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mdp),
	.resource = resources_mdp,
};

#if defined(CONFIG_ARCH_MSM7X30)
static struct resource msm_vidc_720p_resources[] = {
	{
		.start	= 0xA3B00000,
		.end	= 0xA3B00000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MFC720,
		.end	= INT_MFC720,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_vidc_720p = {
	.name = "msm_vidc",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_vidc_720p_resources),
	.resource = msm_vidc_720p_resources,
};
#endif

static struct resource resources_tssc[] = {
#if defined(CONFIG_ARCH_MSM7225)
	{
		.start	= MSM_TSSC_PHYS,
		.end	= MSM_TSSC_PHYS + MSM_TSSC_SIZE - 1,
		.name	= "tssc",
		.flags	= IORESOURCE_MEM,
	},
#endif
	{
		.start	= INT_TCHSCRN1,
		.end	= INT_TCHSCRN1,
		.name	= "tssc1",
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
	{
		.start	= INT_TCHSCRN2,
		.end	= INT_TCHSCRN2,
		.name	= "tssc2",
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
};

struct platform_device msm_device_touchscreen = {
	.name = "msm_touchscreen",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_tssc),
	.resource = resources_tssc,
};

#if defined(CONFIG_ARCH_QSD8X50)
static struct resource resources_spi[] = {
	{
		.start	= MSM_SPI_PHYS,
		.end	= MSM_SPI_PHYS + MSM_SPI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.name	= "irq_in",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.name	= "irq_out",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.name	= "irq_err",
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_spi = {
	.name		= "msm_spi",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_spi),
	.resource	= resources_spi,
};
#endif

#ifdef CONFIG_MSM_ROTATOR
static struct resource resources_msm_rotator[] = {
	{
		.start	= MSM_ROTATOR_PHYS,
		.end	= MSM_ROTATOR_PHYS + MSM_ROTATOR_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_ROTATOR,
		.end	= INT_ROTATOR,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_rotator_device = {
	.name		= "msm_rotator",
	.id		= 0,
	.num_resources  = ARRAY_SIZE(resources_msm_rotator),
	.resource       = resources_msm_rotator,
};
#endif

#ifdef CONFIG_MSM_SSBI
static struct resource resources_ssbi_pmic[] = {
	{
		.start	= MSM_PMIC_SSBI_PHYS,
		.end	= MSM_PMIC_SSBI_PHYS + MSM_PMIC_SSBI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi_pmic = {
	.name		= "msm_ssbi",
	.id		= -1,
	.resource	= resources_ssbi_pmic,
	.num_resources	= ARRAY_SIZE(resources_ssbi_pmic),
};
#endif


#define CLOCK(clk_name, clk_id, clk_dev, clk_flags, clk_arch) {	\
	.name = clk_name, \
	.id = clk_id, \
	.flags = (clk_flags) | ((clk_arch) & CLKFLAG_ARCH_ALL), \
	.dev = clk_dev, \
	}

#define CLK_ALL(name, id, dev, flags) \
		CLOCK(name, id, dev, flags, CLKFLAG_ARCH_ALL)
#define CLK_7X00A(name, id, dev, flags) \
		CLOCK(name, id, dev, flags, CLKFLAG_ARCH_MSM7X00A)
#define CLK_8X50(name, id, dev, flags) \
		CLOCK(name, id, dev, flags, CLKFLAG_ARCH_QSD8X50)

#define OFF CLKFLAG_AUTO_OFF
#define MINMAX (CLKFLAG_USE_MIN_TO_SET | CLKFLAG_USE_MAX_TO_SET)
#define USE_MIN (CLKFLAG_USE_MIN_TO_SET | CLKFLAG_SHARED)
#define DEFER CLKFLAG_DEFER

struct clk msm_clocks[] = {
#ifndef CONFIG_ARCH_MSM7X30
	CLK_ALL("adm_clk", ADM_CLK, NULL, DEFER),
	CLK_ALL("adsp_clk", ADSP_CLK, NULL, 0),
	CLK_ALL("ebi1_clk", EBI1_CLK, NULL, USE_MIN),
	CLK_ALL("ebi2_clk", EBI2_CLK, NULL, 0),
	CLK_ALL("ecodec_clk", ECODEC_CLK, NULL, 0),
	CLK_ALL("mddi_clk", EMDH_CLK, &msm_device_mddi1.dev, OFF),
	CLK_ALL("gp_clk", GP_CLK, NULL, 0),
	CLK_ALL("grp_clk", GRP_CLK, NULL, OFF),
	CLK_ALL("i2c_clk", I2C_CLK, &msm_device_i2c.dev, 0),
	CLK_ALL("icodec_rx_clk", ICODEC_RX_CLK, NULL, 0),
	CLK_ALL("icodec_tx_clk", ICODEC_TX_CLK, NULL, 0),
	CLK_ALL("imem_clk", IMEM_CLK, NULL, OFF),
	CLK_ALL("mdc_clk", MDC_CLK, NULL, 0),
	CLK_ALL("mdp_clk", MDP_CLK, &msm_device_mdp.dev, 0),
	CLK_ALL("pbus_clk", PBUS_CLK, NULL, 0),
	CLK_ALL("pcm_clk", PCM_CLK, NULL, 0),
#ifdef CONFIG_MACH_SUPERSONIC
	CLK_ALL("mddi_clk", PMDH_CLK, &msm_device_mddi0.dev, OFF),
#else
	CLK_ALL("mddi_clk", PMDH_CLK, &msm_device_mddi0.dev, OFF | MINMAX),
#endif
	CLK_ALL("sdac_clk", SDAC_CLK, NULL, OFF),
	CLK_ALL("sdc_clk", SDC1_CLK, &msm_device_sdc1.dev, OFF),
	CLK_ALL("sdc_pclk", SDC1_PCLK, &msm_device_sdc1.dev, OFF),
	CLK_ALL("sdc_clk", SDC2_CLK, &msm_device_sdc2.dev, OFF),
	CLK_ALL("sdc_pclk", SDC2_PCLK, &msm_device_sdc2.dev, OFF),
	CLK_ALL("sdc_clk", SDC3_CLK, &msm_device_sdc3.dev, OFF),
	CLK_ALL("sdc_pclk", SDC3_PCLK, &msm_device_sdc3.dev, OFF),
	CLK_ALL("sdc_clk", SDC4_CLK, &msm_device_sdc4.dev, OFF),
	CLK_ALL("sdc_pclk", SDC4_PCLK, &msm_device_sdc4.dev, OFF),
	CLK_ALL("tsif_clk", TSIF_CLK, NULL, 0),
	CLK_ALL("tsif_ref_clk", TSIF_REF_CLK, NULL, 0),
	CLK_ALL("tv_dac_clk", TV_DAC_CLK, NULL, 0),
	CLK_ALL("tv_enc_clk", TV_ENC_CLK, NULL, 0),
	CLK_ALL("uart_clk", UART1_CLK, &msm_device_uart1.dev, OFF),
	CLK_ALL("uart_clk", UART2_CLK, &msm_device_uart2.dev, OFF),
	CLK_ALL("uart_clk", UART3_CLK, &msm_device_uart3.dev, OFF),
	CLK_ALL("uartdm_clk", UART1DM_CLK, &msm_device_uart_dm1.dev, OFF),
	CLK_ALL("uartdm_clk", UART2DM_CLK, &msm_device_uart_dm2.dev, OFF),
	CLK_ALL("usb_hs_clk", USB_HS_CLK, &msm_device_hsusb.dev, OFF),
	CLK_ALL("usb_hs_pclk", USB_HS_PCLK, &msm_device_hsusb.dev, OFF),
	CLK_ALL("usb_otg_clk", USB_OTG_CLK, NULL, 0),
	CLK_ALL("vdc_clk", VDC_CLK, NULL, OFF | MINMAX),
	CLK_ALL("vfe_clk", VFE_CLK, NULL, OFF),
	CLK_ALL("vfe_mdc_clk", VFE_MDC_CLK, NULL, OFF),
#ifdef CONFIG_ARCH_MSM7227
	CLK_ALL("grp_pclk", GRP_PCLK, NULL, OFF),
#endif
	CLK_ALL("spi_clk", SPI_CLK, NULL, 0),
	CLK_ALL("usb_phy_clk", USB_PHY_CLK, NULL, USE_MIN),
	CLK_8X50("lcdc_pclk_clk", LCDC_PCLK, &msm_device_mdp.dev, 0),
	CLK_8X50("lcdc_pad_pclk_clk", LCDC_PAD_PCLK, &msm_device_mdp.dev, 0),
	CLK_8X50("mdp_vsync_clk", MDP_VSYNC_CLK, &msm_device_mdp.dev, 0),

	CLOCK(NULL, 0, NULL, 0, 0),
#else /* 7x30 clock tbl */
	CLK_PCOM("adm_clk", ADM_CLK, NULL, DEFER),
	CLK_PCOM("adsp_clk", ADSP_CLK, NULL, 0),
	CLK_PCOM("cam_m_clk", CAM_M_CLK, NULL, 0),
	CLK_PCOM("camif_pad_pclk", CAMIF_PAD_P_CLK, NULL, OFF),
	CLK_PCOM("codec_ssbi_clk", CODEC_SSBI_CLK, NULL, 0),
	CLK_PCOM("ebi1_clk", EBI1_CLK, NULL, USE_MIN),
	CLK_PCOM("ecodec_clk", ECODEC_CLK, NULL, 0),
	CLK_PCOM("emdh_clk", EMDH_CLK, NULL, OFF | MINMAX),
	CLK_PCOM("emdh_pclk", EMDH_P_CLK, NULL, OFF),
	CLK_PCOM("gp_clk", GP_CLK, NULL, 0),
	CLK_PCOM("grp_2d_clk", GRP_2D_CLK, NULL, 0),
	CLK_PCOM("grp_2d_pclk", GRP_2D_P_CLK, NULL, 0),
	CLK_PCOM("grp_clk", GRP_3D_CLK,	NULL, 0),
	CLK_PCOM("grp_pclk", GRP_3D_P_CLK, NULL, 0),
	CLK_7X30S("grp_src_clk", GRP_3D_SRC_CLK, GRP_3D_CLK, NULL, 0),
	CLK_PCOM("hdmi_clk", HDMI_CLK, NULL, 0),
	CLK_PCOM("i2c_clk", I2C_CLK, &msm_device_i2c.dev, 0),
	CLK_PCOM("i2c_clk", I2C_2_CLK, &msm_device_i2c_2.dev, 0),
	CLK_PCOM("imem_clk", IMEM_CLK, NULL, OFF),
	CLK_PCOM("jpeg_clk", JPEG_CLK, NULL, OFF),
	CLK_PCOM("jpeg_pclk", JPEG_P_CLK, NULL, OFF),
	CLK_PCOM("lpa_codec_clk", LPA_CODEC_CLK, NULL, 0),
	CLK_PCOM("lpa_core_clk", LPA_CORE_CLK, NULL, 0),
	CLK_PCOM("lpa_pclk", LPA_P_CLK, NULL, 0),
	CLK_PCOM("mdc_clk", MDC_CLK, NULL, 0),
	CLK_PCOM("mddi_clk", PMDH_CLK, NULL, OFF | MINMAX),
	CLK_PCOM("mddi_pclk", PMDH_P_CLK, NULL, 0),
	CLK_PCOM("mdp_clk", MDP_CLK, NULL, OFF),
	CLK_PCOM("mdp_pclk", MDP_P_CLK, NULL, 0),
	/*Original is mdp_lcdc_pclk_clk and mdp_lcdc_pad_pclk_clk*/
	CLK_PCOM("lcdc_pclk_clk", MDP_LCDC_PCLK_CLK, NULL, OFF),
	CLK_PCOM("lcdc_pad_pclk_clk", MDP_LCDC_PAD_PCLK_CLK, NULL, OFF),
	CLK_PCOM("mdp_vsync_clk", MDP_VSYNC_CLK, NULL, 0),
	CLK_PCOM("mfc_clk", MFC_CLK, NULL, 0),
	CLK_PCOM("mfc_div2_clk", MFC_DIV2_CLK, NULL, 0),
	CLK_PCOM("mfc_pclk", MFC_P_CLK, NULL, 0),
	CLK_PCOM("mi2s_codec_rx_m_clk", MI2S_CODEC_RX_M_CLK, NULL, 0),
	CLK_PCOM("mi2s_codec_rx_s_clk", MI2S_CODEC_RX_S_CLK, NULL, 0),
	CLK_PCOM("mi2s_codec_tx_m_clk", MI2S_CODEC_TX_M_CLK, NULL, 0),
	CLK_PCOM("mi2s_codec_tx_s_clk", MI2S_CODEC_TX_S_CLK, NULL, 0),
	CLK_PCOM("pbus_clk", PBUS_CLK, NULL, USE_MIN),
	CLK_PCOM("pcm_clk", PCM_CLK, NULL, 0),
	CLK_PCOM("qup_clk", QUP_I2C_CLK, &qup_device_i2c.dev, 0),
	CLK_PCOM("rotator_clk", AXI_ROTATOR_CLK, NULL, 0),
	CLK_PCOM("rotator_imem_clk", ROTATOR_IMEM_CLK, NULL, OFF),
	CLK_PCOM("rotator_pclk", ROTATOR_P_CLK, NULL, OFF),
	CLK_PCOM("sdac_clk", SDAC_CLK, NULL, OFF),
	CLK_PCOM("sdc_clk", SDC1_CLK, &msm_device_sdc1.dev, OFF),
	CLK_PCOM("sdc_pclk", SDC1_P_CLK, &msm_device_sdc1.dev, OFF),
	CLK_PCOM("sdc_clk", SDC2_CLK, &msm_device_sdc2.dev, OFF),
	CLK_PCOM("sdc_pclk", SDC2_P_CLK, &msm_device_sdc2.dev, OFF),
	CLK_PCOM("sdc_clk", SDC3_CLK, &msm_device_sdc3.dev, OFF),
	CLK_PCOM("sdc_pclk", SDC3_P_CLK, &msm_device_sdc3.dev, OFF),
	CLK_PCOM("sdc_clk", SDC4_CLK, &msm_device_sdc4.dev, OFF),
	CLK_PCOM("sdc_pclk", SDC4_P_CLK, &msm_device_sdc4.dev, OFF),
	CLK_PCOM("spi_clk", SPI_CLK, NULL, 0),
	CLK_PCOM("spi_pclk", SPI_P_CLK, NULL, OFF),
	CLK_7X30S("tv_src_clk", TV_CLK, TV_ENC_CLK, NULL, 0),
	CLK_PCOM("tv_dac_clk", TV_DAC_CLK, NULL, 0),
	CLK_PCOM("tv_enc_clk", TV_ENC_CLK, NULL, 0),
	CLK_PCOM("uart_clk", UART1_CLK, &msm_device_uart1.dev, OFF),
	CLK_PCOM("uart_clk", UART2_CLK, &msm_device_uart2.dev, 0),
	CLK_PCOM("uart_clk", UART3_CLK, &msm_device_uart3.dev, OFF),
	CLK_PCOM("uartdm_clk", UART1DM_CLK, &msm_device_uart_dm1.dev, OFF),
	CLK_PCOM("uartdm_clk", UART2DM_CLK, &msm_device_uart_dm2.dev, 0),
	CLK_PCOM("usb_hs_clk", USB_HS_CLK, NULL, OFF),
	CLK_PCOM("usb_hs_pclk", USB_HS_P_CLK, NULL, OFF),
	/* Now we can't close these usb clocks at the beginning change OFF to 0 temporarily */
	CLK_PCOM("usb_hs_core_clk", USB_HS_CORE_CLK, NULL, 0),
	CLK_PCOM("usb_hs2_clk", USB_HS2_CLK, NULL, 0),
	CLK_PCOM("usb_hs2_pclk", USB_HS2_P_CLK, NULL, 0),
	CLK_PCOM("usb_hs2_core_clk", USB_HS2_CORE_CLK, NULL, 0),
	CLK_PCOM("usb_hs3_clk", USB_HS3_CLK, NULL, 0),
	CLK_PCOM("usb_hs3_pclk", USB_HS3_P_CLK, NULL, 0),
	CLK_PCOM("usb_hs3_core_clk", USB_HS3_CORE_CLK, NULL, 0),
	CLK_PCOM("vdc_clk", VDC_CLK, NULL, OFF | MINMAX),
	CLK_PCOM("vfe_camif_clk", VFE_CAMIF_CLK, NULL, OFF),
	CLK_PCOM("vfe_clk", VFE_CLK, NULL, 0),
	CLK_PCOM("vfe_mdc_clk", VFE_MDC_CLK, NULL, OFF),
	CLK_PCOM("vfe_pclk", VFE_P_CLK, NULL, OFF),
	CLK_PCOM("vpe_clk", VPE_CLK, NULL, 0),

	/* 7x30 v2 hardware only. */
	CLK_PCOM("csi_clk", CSI0_CLK, NULL, 0),
	CLK_PCOM("csi_pclk", CSI0_P_CLK, NULL, 0),
	CLK_PCOM("csi_vfe_clk", CSI0_VFE_CLK, NULL, 0),

	CLOCK(NULL, 0, NULL, 0, 0),
#endif
};

static int mfg_mode;
int __init board_mfg_mode_init(char *s)
{
	if (!strcmp(s, "normal"))
		mfg_mode = 0;
	else if (!strcmp(s, "factory2"))
		mfg_mode = 1;
	else if (!strcmp(s, "recovery"))
		mfg_mode = 2;
	else if (!strcmp(s, "charge"))
		mfg_mode = 3;
	else if (!strcmp(s, "power_test"))
		mfg_mode = 4;
	else if (!strcmp(s, "offmode_charging"))
		mfg_mode = 5;

	return 1;
}
__setup("androidboot.mode=", board_mfg_mode_init);


int board_mfg_mode(void)
{
	return mfg_mode;
}

EXPORT_SYMBOL(board_mfg_mode);

static int __init board_serialno_setup(char *serialno)
{
	char *str;

	/* use default serial number when mode is factory2 */
	if (board_mfg_mode() == 1 || !strlen(serialno))
		str = df_serialno;
	else
		str = serialno;
#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.serial_number = str;
#endif
	board_sn = str;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

char *board_serialno(void)
{
	return board_sn;
}

#define ATAG_SKUID 0x4d534D73
int __init parse_tag_skuid(const struct tag *tags)
{
	int skuid = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_SKUID) {
			printk(KERN_DEBUG "find the skuid tag\n");
			find = 1;
			break;
		}
	}

	if (find)
		skuid = t->u.revision.rev;
	printk(KERN_DEBUG "parse_tag_skuid: hwid = 0x%x\n", skuid);
	return skuid;
}
__tagtable(ATAG_SKUID, parse_tag_skuid);

#define ATAG_HERO_PANEL_TYPE 0x4d534D74
int panel_type;
int __init tag_panel_parsing(const struct tag *tags)
{
	panel_type = tags->u.revision.rev;

	printk(KERN_DEBUG "%s: panel type = %d\n", __func__,
		panel_type);

	return panel_type;
}
__tagtable(ATAG_HERO_PANEL_TYPE, tag_panel_parsing);

#define ATAG_ENGINEERID 0x4d534D75
unsigned engineer_id;
int __init parse_tag_engineerid(const struct tag *tags)
{
	int engineerid = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_ENGINEERID) {
			printk(KERN_DEBUG "find the engineer tag\n");
			find = 1;
			break;
		}
	}

	if (find) {
		engineer_id = t->u.revision.rev;
		engineerid = t->u.revision.rev;
	}
	printk(KERN_DEBUG "parse_tag_engineerid: 0x%x\n", engineerid);
	return engineerid;
}
__tagtable(ATAG_ENGINEERID, parse_tag_engineerid);

#define ATAG_MFG_GPIO_TABLE 0x59504551
int __init parse_tag_mfg_gpio_table(const struct tag *tags)
{
       unsigned char *dptr = (unsigned char *)(&tags->u);
       __u32 size;

       size = min((__u32)(tags->hdr.size - 2) * sizeof(__u32), (__u32)MFG_GPIO_TABLE_MAX_SIZE);
       memcpy(mfg_gpio_table, dptr, size);
       return 0;
}
__tagtable(ATAG_MFG_GPIO_TABLE, parse_tag_mfg_gpio_table);

char * board_get_mfg_sleep_gpio_table(void)
{
        return mfg_gpio_table;
}
EXPORT_SYMBOL(board_get_mfg_sleep_gpio_table);

static char *emmc_tag;
static int __init board_set_emmc_tag(char *get_hboot_emmc)
{
	if (strlen(get_hboot_emmc))
		emmc_tag = get_hboot_emmc;
	else
		emmc_tag = NULL;
	return 1;
}
__setup("androidboot.emmc=", board_set_emmc_tag);

int board_emmc_boot(void)
{
	if (emmc_tag) {
		if (!strcmp(emmc_tag, "true"))
			return 1;
	}

	return 0;
}

#define ATAG_MEMSIZE 0x5441001e
unsigned memory_size;
int __init parse_tag_memsize(const struct tag *tags)
{
	int mem_size = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_MEMSIZE) {
			printk(KERN_DEBUG "find the memsize tag\n");
			find = 1;
			break;
		}
	}

	if (find) {
		memory_size = t->u.revision.rev;
		mem_size = t->u.revision.rev;
	}
	printk(KERN_DEBUG "parse_tag_memsize: %d\n", memory_size);
	return mem_size;
}
__tagtable(ATAG_MEMSIZE, parse_tag_memsize);

