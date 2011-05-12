/* linux/driver/spi/spi_qsd.c 
 *
 * Copyright (C) 2009 Solomon Chiu <solomon_chiu@htc.com>
 *
 * 	This is a temporary solution to substitute Qualcomm's SPI.
 *	Should be replaced by formal SPI driver in the future.
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <asm/gpio.h>
#include <mach/msm_iomap.h>

#define SPI_CONFIG              (0x00000000)
#define SPI_IO_CONTROL          (0x00000004)
#define SPI_OPERATIONAL         (0x00000030)
#define SPI_ERROR_FLAGS_EN      (0x00000038)
#define SPI_ERROR_FLAGS         (0x00000034)
#define SPI_OUTPUT_FIFO         (0x00000100)

#ifdef CONFIG_ARCH_MSM7X30

struct spi_device        *spidev;

int qspi_send_16bit(unsigned char id, unsigned data)
{
	unsigned char buffer[2];
	int tmp;
	tmp = (id<<13 | data)<<16;

	buffer[0] = tmp >> 24;
	buffer[1] = (tmp & 0x00FF0000) >> 16;
	spi_write(spidev,buffer, 2);
	return 0;
}

int qspi_send_9bit(struct spi_msg *msg)
{
	int tmp = 0;
	spidev->bits_per_word = 9;
	tmp = (0x0 <<8 | msg->cmd)<<23;
	msg->buffer[0] = tmp >> 24;
	msg->buffer[1] = (tmp & 0x00FF0000) >> 16;

	if(msg->len != 0) {
		int i = 0, j;
		for(j = 2; i < msg->len; i++, j+=2){
			tmp &= 0x00000000;
			tmp = (0x1<<8 | *(msg->data+i))<<23;
			msg->buffer[j] = tmp >> 24;
			msg->buffer[j+1] = (tmp & 0x00FF0000) >> 16;
}
}

	spi_read_write_lock(spidev, msg, NULL, 2, 1);

	return 0;
}


int qspi_send(unsigned char id, unsigned data)
{
	unsigned char buffer[2];
	int tmp;
	tmp = (0x7000 | id<<9 | data)<<16;

	spidev->bits_per_word = 16;

	buffer[0] = tmp >> 24;
	buffer[1] = (tmp & 0x00FF0000) >> 16;

	spi_write(spidev,buffer,2);
	return 0;
	}


static int msm_spi_probe(struct spi_device *spi)
{
	printk(" %s \n", __func__);
	spidev = spi;
	return 0 ;
}

static int msm_spi_remove(struct spi_device *pdev)
{
	spidev = NULL;
	return 0;
	}


static struct spi_driver spi_qsd = {
	.driver = {
		.name  = "spi_qsd",
		.owner = THIS_MODULE,
	},
	.probe         = msm_spi_probe,
	.remove        = msm_spi_remove,
};


static int __init spi_qsd_init(void)
{
	int rc;
	rc = spi_register_driver(&spi_qsd);
	return rc;
}
module_init(spi_qsd_init);

static void __exit spi_qsd_exit(void)
{
	spi_unregister_driver(&spi_qsd);
}
module_exit(spi_qsd_exit);

#else

void __iomem *spi_base;
struct clk *spi_clk ;

int qspi_send_16bit(unsigned char id, unsigned data)
{
        unsigned err ;

        /* bit-5: OUTPUT_FIFO_NOT_EMPTY */
	clk_enable(spi_clk);
        while( readl(spi_base+SPI_OPERATIONAL) & (1<<5) )
{
                if( (err=readl(spi_base+SPI_ERROR_FLAGS)) )
	{
                        printk("\rERROR:  SPI_ERROR_FLAGS=%d\r", err);
                        return -1;
		}
	}

	writel( (id<<13 | data)<<16, spi_base+SPI_OUTPUT_FIFO );/*AUO*/
        udelay(1000);
	clk_disable(spi_clk);

	return 0;
}

int qspi_send_9bit(unsigned char id, unsigned data)
{
        unsigned err ;

        /* bit-5: OUTPUT_FIFO_NOT_EMPTY */
	clk_enable(spi_clk);
        while( readl(spi_base+SPI_OPERATIONAL) & (1<<5) )
{
                if( (err=readl(spi_base+SPI_ERROR_FLAGS)) )
{
                        printk("\rERROR:  SPI_ERROR_FLAGS=%d\r", err);
                        return -1;
	}
}

	writel( ((id<<8) | data)<<23, spi_base+SPI_OUTPUT_FIFO);/*sharp*/

        udelay(1000);
	clk_disable(spi_clk);
	return 0;
}


int qspi_send(unsigned char id, unsigned data)
{
        unsigned err ;

        /* bit-5: OUTPUT_FIFO_NOT_EMPTY */
	clk_enable(spi_clk);
        while( readl(spi_base+SPI_OPERATIONAL) & (1<<5) )
{
                if( (err=readl(spi_base+SPI_ERROR_FLAGS)) )
{
                        printk("\rERROR:  SPI_ERROR_FLAGS=%d\r", err);
                        return -1;
	}
}

        writel( (0x7000 | id<<9 | data)<<16, spi_base+SPI_OUTPUT_FIFO );
        udelay(100);
	clk_disable(spi_clk);
	return 0;
}

static int msm_spi_probe(struct platform_device *pdev)
{
	int rc ;
	struct spi_platform_data *pdata = pdev->dev.platform_data;

	spi_base=ioremap(0xA1200000, 4096);
	if(!spi_base)
		return -1;

        spi_clk = clk_get(&pdev->dev, "spi_clk");
        if (IS_ERR(spi_clk)) {
		dev_err(&pdev->dev, "%s: unable to get spi_clk\n", __func__);
                rc = PTR_ERR(spi_clk);
		goto err_probe_clk_get;
	}
        rc = clk_enable(spi_clk);
	if (rc) {
		dev_err(&pdev->dev, "%s: unable to enable spi_clk\n",
			__func__);
		goto err_probe_clk_enable;
	}

	if(pdata == NULL)
		clk_set_rate(spi_clk, 4800000);
	else
		clk_set_rate(spi_clk, pdata->clk_rate);

	printk(KERN_DEBUG "spi clk = 0x%ld\n", clk_get_rate(spi_clk));
	printk("spi: SPI_CONFIG=%x\n", readl(spi_base+SPI_CONFIG));
	printk("spi: SPI_IO_CONTROL=%x\n", readl(spi_base+SPI_IO_CONTROL));
	printk("spi: SPI_OPERATIONAL=%x\n", readl(spi_base+SPI_OPERATIONAL));
	printk("spi: SPI_ERROR_FLAGS_EN=%x\n", readl(spi_base+SPI_ERROR_FLAGS_EN));
	printk("spi: SPI_ERROR_FLAGS=%x\n", readl(spi_base+SPI_ERROR_FLAGS));
	printk("-%s()\n", __FUNCTION__);
	clk_disable(spi_clk);

	return 0;

err_probe_clk_get:
err_probe_clk_enable:
	return -1 ;
}

static int __devexit msm_spi_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver msm_spi_driver = {
	.probe          = msm_spi_probe,
	.driver		= {
		.name	= "spi_qsd",
		.owner	= THIS_MODULE,
	},
	.remove		= __exit_p(msm_spi_remove),
};

static int __init msm_spi_init(void)
{
	return platform_driver_register(&msm_spi_driver);
}

fs_initcall(msm_spi_init);
#endif
