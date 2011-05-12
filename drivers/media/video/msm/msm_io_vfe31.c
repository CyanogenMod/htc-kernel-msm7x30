/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include <mach/clk.h>

#define CAMIF_CFG_RMSK             0x1fffff
#define CAM_SEL_BMSK               0x2
#define CAM_PCLK_SRC_SEL_BMSK      0x60000
#define CAM_PCLK_INVERT_BMSK       0x80000
#define CAM_PAD_REG_SW_RESET_BMSK  0x100000

#define EXT_CAM_HSYNC_POL_SEL_BMSK 0x10000
#define EXT_CAM_VSYNC_POL_SEL_BMSK 0x8000
#define MDDI_CLK_CHICKEN_BIT_BMSK  0x80

#define CAM_SEL_SHFT               0x1
#define CAM_PCLK_SRC_SEL_SHFT      0x11
#define CAM_PCLK_INVERT_SHFT       0x13
#define CAM_PAD_REG_SW_RESET_SHFT  0x14

#define EXT_CAM_HSYNC_POL_SEL_SHFT 0x10
#define EXT_CAM_VSYNC_POL_SEL_SHFT 0xF
#define MDDI_CLK_CHICKEN_BIT_SHFT  0x7
/* MIPI	CSI	controller registers */
#define	MIPI_PHY_CONTROL			0x00000000
#define	MIPI_PROTOCOL_CONTROL		0x00000004
#define	MIPI_INTERRUPT_STATUS		0x00000008
#define	MIPI_INTERRUPT_MASK			0x0000000C
#define	MIPI_CAMERA_CNTL			0x00000024
#define	MIPI_CALIBRATION_CONTROL	0x00000018
#define	MIPI_PHY_D0_CONTROL2		0x00000038
#define	MIPI_PHY_D1_CONTROL2		0x0000003C
#define	MIPI_PHY_D2_CONTROL2		0x00000040
#define	MIPI_PHY_D3_CONTROL2		0x00000044
#define	MIPI_PHY_CL_CONTROL			0x00000048
#define	MIPI_PHY_D0_CONTROL			0x00000034
#define	MIPI_PHY_D1_CONTROL			0x00000020
#define	MIPI_PHY_D2_CONTROL			0x0000002C
#define	MIPI_PHY_D3_CONTROL			0x00000030
#define	MIPI_PROTOCOL_CONTROL_SW_RST_BMSK			0x8000000
#define	MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK	0x200000
#define	MIPI_PROTOCOL_CONTROL_DATA_FORMAT_BMSK			0x180000
#define	MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK			0x40000
#define	MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK			0x20000
#define	MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT		0x16
#define	MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT	0x15
#define	MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT		0x14
#define	MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT	0x7
#define	MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT			0x13
#define	MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT			0x1e
#define	MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_D1_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D1_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D1_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D1_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_D2_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D2_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D2_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D2_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_D3_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D3_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D3_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D3_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT			0x18
#define	MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT				0x2
#define	MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT				0x1c
#define	MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT		0x9
#define	MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT	0x8

#define MIPI_IMASK_ERROR_OCCUR                0xF01FFFC0
#define MIPI_IMASK_CLK_ULPM_ENTRY             (0x00000001<<0)
#define MIPI_IMASK_CLK_ULPM_EXIT              (0x00000001<<1)
#define MIPI_IMASK_DATA_ULPM_ENTRY            (0x00000001<<2)
#define MIPI_IMASK_DATA_ULPM_EXIT             (0x00000001<<3)
#define MIPI_IMASK_CLK_START                  (0x00000001<<4)
#define MIPI_IMASK_CLK_STOP                   (0x00000001<<5)
#define MIPI_IMASK_ERR_SOT                    (0x00000001<<6)
#define MIPI_IMASK_ERR_SOT_SYNC               (0x00000001<<7)
#define MIPI_IMASK_CLK_CTL_ERROR              (0x00000001<<8)
#define MIPI_IMASK_DATA_CTL_ERROR             (0x00000001<<9)
#define MIPI_IMASK_CLK_CMM_ERROR              (0x00000001<<10)
#define MIPI_IMASK_DATA_CMM_ERROR             (0x00000001<<11)
#define MIPI_IMASK_DL0_SYNC_ERROR             (0x00000001<<12)
#define MIPI_IMASK_DL1_SYNC_ERROR             (0x00000001<<13)
#define MIPI_IMASK_DL2_SYNC_ERROR             (0x00000001<<14)
#define MIPI_IMASK_DL3_SYNC_ERROR             (0x00000001<<15)
#define MIPI_IMASK_ECC_ERROR                  (0x00000001<<16)
#define MIPI_IMASK_CRC_ERROR                  (0x00000001<<17)
#define MIPI_IMASK_FRAME_SYNC_ERROR           (0x00000001<<18)
#define MIPI_IMASK_ID_ERROR                   (0x00000001<<19)
#define MIPI_IMASK_EOT_ERROR                  (0x00000001<<20)
#define MIPI_IMASK_SW_RST_DONE                (0x00000001<<21)
#define MIPI_IMASK_SHORT_PACKET_CAPTURE_DONE  (0x00000001<<22)
#define MIPI_IMASK_CAL_DONE                   (0x00000001<<23)
#define MIPI_IMASK_DL0_FIFO_OVERFLOW          (0x00000001<<28)
#define MIPI_IMASK_DL1_FIFO_OVERFLOW          (0x00000001<<29)
#define MIPI_IMASK_DL2_FIFO_OVERFLOW          (0x00000001<<30)
#define MIPI_IMASK_DL3_FIFO_OVERFLOW          (0x00000001<<31)

static struct clk *camio_vfe_mdc_clk;
static struct clk *camio_mdc_clk;
static struct clk *camio_vfe_clk;
static struct clk *camio_vfe_camif_clk;
static struct clk *camio_vfe_pbdg_clk;
static struct clk *camio_cam_m_clk;
static struct clk *camio_camif_pad_pbdg_clk;
static struct clk *camio_csi_clk;
static struct clk *camio_csi_pclk;
static struct clk *camio_csi_vfe_clk;
static struct msm_camera_io_ext camio_ext;

static struct resource *camifpadio, *csiio;
void __iomem *camifpadbase, *csibase;

void msm_io_w(u32 data, void __iomem *addr)
{
	/*CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));*/
	writel((data), (addr));
}

void msm_io_w_mb(u32 data, void __iomem *addr)
{
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	wmb();
	writel((data), (addr));
	wmb();
}

u32 msm_io_r(void __iomem *addr)
{
	uint32_t data = readl(addr);
	/*CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));*/
	return data;
}

u32 msm_io_r_mb(void __iomem *addr)
{
	uint32_t data;
	rmb();
	data = readl(addr);
	rmb();
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	return data;
}

void msm_io_memcpy_toio(void __iomem *dest_addr,
	void __iomem *src_addr, u32 len)
{
	int i;
	u32 *d = (u32 *) dest_addr;
	u32 *s = (u32 *) src_addr;
	/* memcpy_toio does not work. Use writel for now */
	for (i = 0; i < len; i++)
		writel(*s++, d++);
}

void msm_io_dump(void __iomem *addr, int size)
{
	char line_str[128], *p_str;
	int i;
	u32 *p = (u32 *) addr;
	u32 data;
	CDBG("%s: %p %d\n", __func__, addr, size);
	line_str[0] = '\0';
	p_str = line_str;
	for (i = 0; i < size/4; i++) {
		if (i % 4 == 0) {
			sprintf(p_str, "%08x: ", (u32) p);
			p_str += 10;
		}
		data = readl(p++);
		sprintf(p_str, "%08x ", data);
		p_str += 9;
		if ((i + 1) % 4 == 0) {
			CDBG("%s\n", line_str);
			line_str[0] = '\0';
			p_str = line_str;
		}
	}
	if (line_str[0] != '\0')
		CDBG("%s\n", line_str);

}

void msm_io_memcpy(void __iomem *dest_addr, void __iomem *src_addr, u32 len)
{
	CDBG("%s: %p %p %d\n", __func__, dest_addr, src_addr, len);
	msm_io_memcpy_toio(dest_addr, src_addr, len / 4);
	msm_io_dump(dest_addr, len);
}


int msm_camio_clk_enable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		camio_vfe_mdc_clk =
		clk = clk_get(NULL, "vfe_mdc_clk");
		break;

	case CAMIO_MDC_CLK:
		camio_mdc_clk =
		clk = clk_get(NULL, "mdc_clk");
		break;

	case CAMIO_VFE_CLK:
		camio_vfe_clk =
		clk = clk_get(NULL, "vfe_clk");
		msm_camio_clk_rate_set_2(clk, 122880000);
		break;

	case CAMIO_VFE_CLK_FOR_MIPI_2_LANE:
		camio_vfe_clk =
		clk = clk_get(NULL, "vfe_clk");
		msm_camio_clk_rate_set_2(clk, 153600000);
		break;

	case CAMIO_VFE_CAMIF_CLK:
		camio_vfe_camif_clk =
		clk = clk_get(NULL, "vfe_camif_clk");
		break;

	case CAMIO_VFE_PBDG_CLK:
		camio_vfe_pbdg_clk =
		clk = clk_get(NULL, "vfe_pclk");
		break;

	case CAMIO_CAM_MCLK_CLK:
		camio_cam_m_clk =
		clk = clk_get(NULL, "cam_m_clk");
		msm_camio_clk_rate_set_2(clk, 24000000);
		break;

	case CAMIO_CAMIF_PAD_PBDG_CLK:
		camio_camif_pad_pbdg_clk =
		clk = clk_get(NULL, "camif_pad_pclk");
		break;

	case CAMIO_CSI0_CLK:
		camio_csi_clk =
		clk = clk_get(NULL, "csi_clk");
		msm_camio_clk_rate_set_2(clk, 153600000);
		break;
	case CAMIO_CSI0_VFE_CLK:
		camio_csi_vfe_clk =
		clk = clk_get(NULL, "csi_vfe_clk");
		break;
	case CAMIO_CSI0_PCLK:
		camio_csi_pclk =
		clk = clk_get(NULL, "csi_pclk");
		break;
	default:
		break;
	}

	if (clk && !IS_ERR(clk))
		clk_enable(clk);
	else
		rc = -1;
	return rc;
}

int msm_camio_clk_disable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk;
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk;
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk;
		break;

	case CAMIO_VFE_CAMIF_CLK:
		clk = camio_vfe_camif_clk;
		break;

	case CAMIO_VFE_PBDG_CLK:
		clk = camio_vfe_pbdg_clk;
		break;

	case CAMIO_CAM_MCLK_CLK:
		clk = camio_cam_m_clk;
		break;

	case CAMIO_CAMIF_PAD_PBDG_CLK:
		clk = camio_camif_pad_pbdg_clk;
		break;
	case CAMIO_CSI0_CLK:
		clk = camio_csi_clk;
		break;
	case CAMIO_CSI0_VFE_CLK:
		clk = camio_csi_vfe_clk;
		break;
	case CAMIO_CSI0_PCLK:
		clk = camio_csi_pclk;
		break;
	default:
		break;
	}

	if (clk && !IS_ERR(clk)) {
		clk_disable(clk);
		clk_put(clk);
	} else
		rc = -1;

	return rc;
}

void msm_camio_clk_rate_set(int rate)
{
	struct clk *clk = camio_cam_m_clk;
	clk_set_rate(clk, rate);
}

void msm_camio_clk_rate_set_2(struct clk *clk, int rate)
{
	clk_set_rate(clk, rate);
}

static struct clk *jpeg_clk;
static struct clk *jpeg_pclk;

int msm_camio_jpeg_clk_enable(void)
{
	/* MP*fps*(1 + %blanking)
	   2MP: 24MHz  ------ 2 x 10 x 1.2
	   3MP: 36MHz  ------ 3 x 10 x 1.2
	   5MP: 60MHz  ------ 5 x 10 x 1.2
	   8MP: 96MHz  ------ 8 x 10 x 1.2
	  12MP: 144MHz ------12 x 10 x 1.2
	 */
	int rc = -1;
	u32 rate = 144000000;

	if (jpeg_clk  == NULL) {
		jpeg_clk  = clk_get(NULL, "jpeg_clk");
		if (jpeg_clk  == NULL) {
			pr_err("%s:%d] fail rc = %d\n", __func__, __LINE__,
				rc);
			goto fail;
		}
	}

	rc = clk_set_min_rate(jpeg_clk, rate);
	if (rc) {
		pr_err("%s:%d] fail rc = %d\n", __func__, __LINE__, rc);
		goto fail;
	}

	rc = clk_enable(jpeg_clk);
	if (rc) {
		pr_err("%s:%d] fail rc = %d\n", __func__, __LINE__, rc);
		goto fail;
	}

	if (jpeg_pclk == NULL) {
		jpeg_pclk = clk_get(NULL, "jpeg_pclk");
		if (jpeg_pclk == NULL) {
			pr_err("%s:%d] fail rc = %d\n", __func__, __LINE__,
				rc);
			goto fail;
		}
	}

	rc = clk_enable(jpeg_pclk);
	if (rc) {
		pr_err("%s:%d] fail rc = %d\n", __func__, __LINE__, rc);
		goto fail;
	}

	/*rc = pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ,
		"msm_gemini", MSM_SYSTEM_BUS_RATE);
	if (rc) {
		GMN_PR_ERR("request AXI bus QOS fails. rc = %d\n", rc);
		goto fail;
	}*/

	return rc;

fail:
	pr_err("%s:%d] fail rc = %d\n", __func__, __LINE__, rc);
	return rc;
}

int msm_camio_jpeg_clk_disable(void)
{
	clk_disable(jpeg_clk);
	clk_put(jpeg_clk);
	jpeg_clk = NULL;

	clk_disable(jpeg_pclk);
	clk_put(jpeg_pclk);
	jpeg_pclk = NULL;

	//pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ, "msm_gemini");
	return 0;
}


static irqreturn_t msm_io_csi_irq(int irq_num, void *data)
{
	uint32_t irq;
	irq = msm_io_r(csibase + MIPI_INTERRUPT_STATUS);
	CDBG("%s MIPI_INTERRUPT_STATUS = 0x%x\n", __func__, irq);

	if (irq & MIPI_IMASK_ERROR_OCCUR) {
		if (irq & MIPI_IMASK_ERR_SOT)
			pr_info("[CAM]msm_io_csi_irq: SOT error\n");
		if (irq & MIPI_IMASK_ERR_SOT_SYNC)
			pr_info("[CAM]msm_io_csi_irq: SOT SYNC error\n");
		if (irq & MIPI_IMASK_CLK_CTL_ERROR)
			pr_info("[CAM]msm_io_csi_irq: Clock lane ULPM mode sequence or command error\n");
		if (irq & MIPI_IMASK_DATA_CTL_ERROR)
			pr_info("[CAM]msm_io_csi_irq: Data lane ULPM mode sequence or command error\n");
#if 0
		if (irq & MIPI_IMASK_CLK_CMM_ERROR) /* defeatured */
			pr_err("[CAM]msm_io_csi_irq: Common mode error detected by PHY CLK lane\n");
		if (irq & MIPI_IMASK_DATA_CMM_ERROR) /* defeatured */
			pr_err("[CAM]msm_io_csi_irq: Common mode error detected by PHY data lane\n");
#endif
		if (irq & MIPI_IMASK_DL0_SYNC_ERROR)
			pr_info("[CAM]msm_io_csi_irq: An error occured while synchronizing data " \
				"from PHY to VFE clock domain on data lane 0\n");
		if (irq & MIPI_IMASK_DL1_SYNC_ERROR)
			pr_info("[CAM]msm_io_csi_irq: An error occured while synchronizing data " \
				"from PHY to VFE clock domain on data lane 1\n");
		if (irq & MIPI_IMASK_DL2_SYNC_ERROR)
			pr_info("[CAM]msm_io_csi_irq: An error occured while synchronizing data " \
				"from PHY to VFE clock domain on data lane 2\n");
		if (irq & MIPI_IMASK_DL3_SYNC_ERROR)
			pr_info("[CAM]msm_io_csi_irq: An error occured while synchronizing data " \
				"from PHY to VFE clock domain on data lane 3\n");
		if (irq & MIPI_IMASK_ECC_ERROR)
			pr_info("[CAM]msm_io_csi_irq: ECC error\n");
		if (irq & MIPI_IMASK_CRC_ERROR)
			pr_info("[CAM]msm_io_csi_irq: CRC error\n");
		if (irq & MIPI_IMASK_FRAME_SYNC_ERROR)
			pr_info("[CAM]msm_io_csi_irq: FS not paired with FE\n");
		if (irq & MIPI_IMASK_ID_ERROR)
			pr_info("[CAM]msm_io_csi_irq: Long packet ID not defined\n");
		if (irq & MIPI_IMASK_EOT_ERROR)
			pr_info("[CAM]msm_io_csi_irq: The received data is less than the value indicated by WC\n");
		if (irq & MIPI_IMASK_DL0_FIFO_OVERFLOW)
			pr_info("[CAM]msm_io_csi_irq: Data lane 0 FIFO overflow\n");
		if (irq & MIPI_IMASK_DL1_FIFO_OVERFLOW)
			pr_info("[CAM]msm_io_csi_irq: Data lane 1 FIFO overflow\n");
		if (irq & MIPI_IMASK_DL2_FIFO_OVERFLOW)
			pr_info("[CAM]msm_io_csi_irq: Data lane 2 FIFO overflow\n");
		if (irq & MIPI_IMASK_DL3_FIFO_OVERFLOW)
			pr_info("[CAM]msm_io_csi_irq: Data lane 3 FIFO overflow\n");
	}

	msm_io_w(irq, csibase + MIPI_INTERRUPT_STATUS);
	return IRQ_HANDLED;
}
int msm_camio_enable(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;

	camio_ext = camdev->ioext;

	camdev->camera_gpio_on();

	msm_camio_clk_enable(CAMIO_VFE_PBDG_CLK);
	msm_camio_clk_enable(CAMIO_CAMIF_PAD_PBDG_CLK);
	msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
	if (!sinfo->csi_if) {
		msm_camio_clk_enable(CAMIO_VFE_CLK);
		camifpadio = request_mem_region(camio_ext.camifpadphy,
			camio_ext.camifpadsz, pdev->name);
		if (!camifpadio) {
			rc = -EBUSY;
			goto common_fail;
		}
		camifpadbase = ioremap(camio_ext.camifpadphy,
			camio_ext.camifpadsz);
		if (!camifpadbase) {
			rc = -ENOMEM;
			goto parallel_busy;
		}
		msm_camio_clk_enable(CAMIO_VFE_CAMIF_CLK);
	} else {
		uint32_t val;
		msm_camio_clk_enable(CAMIO_VFE_CLK_FOR_MIPI_2_LANE);
		csiio = request_mem_region(camio_ext.csiphy,
			camio_ext.csisz, pdev->name);
		if (!csiio) {
			rc = -EBUSY;
			goto common_fail;
		}
		csibase = ioremap(camio_ext.csiphy,
			camio_ext.csisz);
		if (!csibase) {
			rc = -ENOMEM;
			goto csi_busy;
		}
		rc = request_irq(camio_ext.csiirq, msm_io_csi_irq,
			IRQF_TRIGGER_RISING, "csi", 0);
		if (rc < 0)
			goto csi_irq_fail;
		/* enable required clocks for CSI */
		msm_camio_clk_enable(CAMIO_CSI0_PCLK);
		msm_camio_clk_enable(CAMIO_CSI0_VFE_CLK);
		msm_camio_clk_enable(CAMIO_CSI0_CLK);

		val = (20 << MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
			(0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
			(0x0 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
			(0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
		CDBG("%s MIPI_PHY_D0_CONTROL2 val=0x%x\n", __func__, val);
		msm_io_w(val, csibase + MIPI_PHY_D0_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D1_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D2_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D3_CONTROL2);

		val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
			(0x0 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
		CDBG("%s MIPI_PHY_CL_CONTROL val=0x%x\n", __func__, val);
		msm_io_w(val, csibase + MIPI_PHY_CL_CONTROL);
	}
	return 0;

parallel_busy:
	release_mem_region(camio_ext.camifpadphy, camio_ext.camifpadsz);
	goto common_fail;
csi_irq_fail:
	iounmap(csibase);
csi_busy:
	release_mem_region(camio_ext.csiphy, camio_ext.csisz);
common_fail:
	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
	msm_camio_clk_disable(CAMIO_CAMIF_PAD_PBDG_CLK);
	msm_camio_clk_disable(CAMIO_VFE_PBDG_CLK);
	camdev->camera_gpio_off();
	return rc;
}

void msm_camio_disable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;

	if (!sinfo->csi_if) {
		msm_camio_clk_disable(CAMIO_VFE_CAMIF_CLK);
		iounmap(camifpadbase);
		release_mem_region(camio_ext.camifpadphy, camio_ext.camifpadsz);
	} else {
		uint32_t val;

		val = (20 << MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
			(0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
			(0x0 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
			(0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
		CDBG("%s MIPI_PHY_D0_CONTROL2 val=0x%x\n", __func__, val);
		msm_io_w(val, csibase + MIPI_PHY_D0_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D1_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D2_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D3_CONTROL2);

		val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
			(0x0 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
		CDBG("%s MIPI_PHY_CL_CONTROL val=0x%x\n", __func__, val);
		msm_io_w(val, csibase + MIPI_PHY_CL_CONTROL);
		msleep(10);

		free_irq(camio_ext.csiirq, 0);
		msm_camio_clk_disable(CAMIO_CSI0_PCLK);
		msm_camio_clk_disable(CAMIO_CSI0_VFE_CLK);
		msm_camio_clk_disable(CAMIO_CSI0_CLK);
		iounmap(csibase);
		release_mem_region(camio_ext.csiphy, camio_ext.csisz);
	}
	CDBG("disable clocks\n");

	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_CAMIF_PAD_PBDG_CLK);
	msm_camio_clk_disable(CAMIO_VFE_PBDG_CLK);
}

void msm_camio_camif_pad_reg_reset(void)
{
	uint32_t reg;

	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_INTERNAL);
	msleep(10);

	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	reg |= 0x3;
	msm_io_w(reg, camifpadbase);
	msleep(10);

	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	reg |= 0x10;
	msm_io_w(reg, camifpadbase);
	msleep(10);

	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	/* Need to be uninverted*/
	reg &= 0x03;
	msm_io_w(reg, camifpadbase);
	msleep(10);
}

void msm_camio_vfe_blk_reset(void)
{
	return;


}

void msm_camio_camif_pad_reg_reset_2(void)
{
	uint32_t reg;
	uint32_t mask, value;
	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 1 << CAM_PAD_REG_SW_RESET_SHFT;
	msm_io_w((reg & (~mask)) | (value & mask), camifpadbase);
	mdelay(10);
	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 0 << CAM_PAD_REG_SW_RESET_SHFT;
	msm_io_w((reg & (~mask)) | (value & mask), camifpadbase);
	mdelay(10);
}

void msm_camio_clk_sel(enum msm_camio_clk_src_type srctype)
{
	struct clk *clk = NULL;

	clk = camio_vfe_clk;

	if (clk != NULL) {
		switch (srctype) {
		case MSM_CAMIO_CLK_SRC_INTERNAL:
			clk_set_flags(clk, 0x00000100 << 1);
			break;

		case MSM_CAMIO_CLK_SRC_EXTERNAL:
			clk_set_flags(clk, 0x00000100);
			break;

		default:
			break;
		}
	}
}
int msm_camio_probe_on(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camdev->camera_gpio_on();

	return msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
}

int msm_camio_probe_off(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camdev->camera_gpio_off();
	return msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
}

int msm_camio_read_camif_status(void)
{
	return msm_io_r(camifpadbase + 0x4);
}
int msm_camio_csi_config(struct msm_camera_csi_params *csi_params)
{
	int rc = 0;
	uint32_t val = 0;

	CDBG("msm_camio_csi_config \n");

	/* SOT_ECC_EN enable error correction for SYNC (data-lane) */
	msm_io_w(0x4, csibase + MIPI_PHY_CONTROL);

	/* SW_RST to the CSI core */
	msm_io_w(MIPI_PROTOCOL_CONTROL_SW_RST_BMSK,
		csibase + MIPI_PROTOCOL_CONTROL);

	/* PROTOCOL CONTROL */
	val = MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK |
		MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK |
		MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK;
	val |= (uint32_t)(csi_params->data_format) <<
		MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT;
	val |= csi_params->dpcm_scheme <<
		MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT;
	CDBG("%s MIPI_PROTOCOL_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_PROTOCOL_CONTROL);

	/* SW CAL EN */
	val = (0x1 << MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT) |
		(0x1 <<
		MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT) |
		(0x1 << MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT) |
		(0x1 << MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT);
	CDBG("%s MIPI_CALIBRATION_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_CALIBRATION_CONTROL);

	/* settle_cnt is very sensitive to speed!
	increase this value to run at higher speeds */
	val = (csi_params->settle_cnt <<
			MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
		(0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
		(0x1 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
		(0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
	CDBG("%s MIPI_PHY_D0_CONTROL2 val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_PHY_D0_CONTROL2);
	msm_io_w(val, csibase + MIPI_PHY_D1_CONTROL2);
	msm_io_w(val, csibase + MIPI_PHY_D2_CONTROL2);
	msm_io_w(val, csibase + MIPI_PHY_D3_CONTROL2);


	val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
		(0x1 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
	CDBG("%s MIPI_PHY_CL_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_PHY_CL_CONTROL);

	val = 0 << MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT;
	msm_io_w(val, csibase + MIPI_PHY_D0_CONTROL);

	val = (0x1 << MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT) |
		(0x1 << MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT);
	CDBG("%s MIPI_PHY_D1_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_PHY_D1_CONTROL);

	msm_io_w(0x00000000, csibase + MIPI_PHY_D2_CONTROL);
	msm_io_w(0x00000000, csibase + MIPI_PHY_D3_CONTROL);

	/* halcyon only supports 1 or 2 lane */
	switch (csi_params->lane_cnt) {
	case 1:
		msm_io_w(csi_params->lane_assign << 8 | 0x4,
			csibase + MIPI_CAMERA_CNTL);
		break;
	case 2:
		msm_io_w(csi_params->lane_assign << 8 | 0x5,
			csibase + MIPI_CAMERA_CNTL);
		break;
	case 3:
		msm_io_w(csi_params->lane_assign << 8 | 0x6,
			csibase + MIPI_CAMERA_CNTL);
		break;
	case 4:
		msm_io_w(csi_params->lane_assign << 8 | 0x7,
			csibase + MIPI_CAMERA_CNTL);
		break;
	}

	/* mask out ID_ERROR[19], DATA_CMM_ERR[11]
	and CLK_CMM_ERR[10] - de-featured */
	msm_io_w(0xFFF7F3FF, csibase + MIPI_INTERRUPT_MASK);
	/*clear IRQ bits*/
	msm_io_w(0xFFF7F3FF, csibase + MIPI_INTERRUPT_STATUS);

	return rc;
}
