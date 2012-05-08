/*
 * XG20, XG21, XG40, XG42 frame buffer device
 * for Linux kernels  2.5.x, 2.6.x
 * Base on TW's sis fbdev code.
 */

/* #include <linux/config.h> */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <linux/selection.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/vt_kern.h>
#include <linux/capability.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/proc_fs.h>

#ifndef XGIFB_PAN
#define XGIFB_PAN
#endif

#include <linux/io.h>
#ifdef CONFIG_MTRR
#include <asm/mtrr.h>
#endif

#include "XGIfb.h"
#include "vgatypes.h"
#include "XGI_main.h"
#include "vb_init.h"
#include "vb_util.h"
#include "vb_setmode.h"

#define Index_CR_GPIO_Reg1 0x48
#define Index_CR_GPIO_Reg2 0x49
#define Index_CR_GPIO_Reg3 0x4a

#define GPIOG_EN    (1<<6)
#define GPIOG_WRITE (1<<6)
#define GPIOG_READ  (1<<1)

#define XGIFB_ROM_SIZE	65536

/* -------------------- Macro definitions ---------------------------- */

#undef XGIFBDEBUG

#ifdef XGIFBDEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#ifdef XGIFBDEBUG
static void dumpVGAReg(void)
{
	u8 i, reg;

	xgifb_reg_set(XGISR, 0x05, 0x86);
	/*
	xgifb_reg_set(XGISR, 0x08, 0x4f);
	xgifb_reg_set(XGISR, 0x0f, 0x20);
	xgifb_reg_set(XGISR, 0x11, 0x4f);
	xgifb_reg_set(XGISR, 0x13, 0x45);
	xgifb_reg_set(XGISR, 0x14, 0x51);
	xgifb_reg_set(XGISR, 0x1e, 0x41);
	xgifb_reg_set(XGISR, 0x1f, 0x0);
	xgifb_reg_set(XGISR, 0x20, 0xa1);
	xgifb_reg_set(XGISR, 0x22, 0xfb);
	xgifb_reg_set(XGISR, 0x26, 0x22);
	xgifb_reg_set(XGISR, 0x3e, 0x07);
	*/

	/* xgifb_reg_set(XGICR, 0x19, 0x00); */
	/* xgifb_reg_set(XGICR, 0x1a, 0x3C); */
	/* xgifb_reg_set(XGICR, 0x22, 0xff); */
	/* xgifb_reg_set(XGICR, 0x3D, 0x10); */

	/* xgifb_reg_set(XGICR, 0x4a, 0xf3); */

	/* xgifb_reg_set(XGICR, 0x57, 0x0); */
	/* xgifb_reg_set(XGICR, 0x7a, 0x2c); */

	/* xgifb_reg_set(XGICR, 0x82, 0xcc); */
	/* xgifb_reg_set(XGICR, 0x8c, 0x0); */
	/*
	xgifb_reg_set(XGICR, 0x99, 0x1);
	xgifb_reg_set(XGICR, 0x41, 0x40);
	*/

	for (i = 0; i < 0x4f; i++) {
		reg = xgifb_reg_get(XGISR, i);
		printk("\no 3c4 %x", i);
		printk("\ni 3c5 => %x", reg);
	}

	for (i = 0; i < 0xF0; i++) {
		reg = xgifb_reg_get(XGICR, i);
		printk("\no 3d4 %x", i);
		printk("\ni 3d5 => %x", reg);
	}
	/*
	xgifb_reg_set(XGIPART1,0x2F,1);
	for (i=1; i < 0x50; i++) {
		reg = xgifb_reg_get(XGIPART1, i);
		printk("\no d004 %x", i);
		printk("\ni d005 => %x", reg);
	}

	for (i=0; i < 0x50; i++) {
		 reg = xgifb_reg_get(XGIPART2, i);
		 printk("\no d010 %x", i);
		 printk("\ni d011 => %x", reg);
	}
	for (i=0; i < 0x50; i++) {
		reg = xgifb_reg_get(XGIPART3, i);
		printk("\no d012 %x",i);
		printk("\ni d013 => %x",reg);
	}
	for (i=0; i < 0x50; i++) {
		reg = xgifb_reg_get(XGIPART4, i);
		printk("\no d014 %x",i);
		printk("\ni d015 => %x",reg);
	}
	*/
}
#else
static inline void dumpVGAReg(void)
{
}
#endif

/* data for XGI components */
struct video_info xgi_video_info;

#if 1
#define DEBUGPRN(x)
#else
#define DEBUGPRN(x) printk(KERN_INFO x "\n");
#endif

/* --------------- Hardware Access Routines -------------------------- */

static int XGIfb_mode_rate_to_dclock(struct vb_device_info *XGI_Pr,
		struct xgi_hw_device_info *HwDeviceExtension,
		unsigned char modeno, unsigned char rateindex)
{
	unsigned short ModeNo = modeno;
	unsigned short ModeIdIndex = 0, ClockIndex = 0;
	unsigned short RefreshRateTableIndex = 0;

	/* unsigned long  temp = 0; */
	int Clock;
	XGI_Pr->ROMAddr = HwDeviceExtension->pjVirtualRomBase;
	InitTo330Pointer(HwDeviceExtension->jChipType, XGI_Pr);

	RefreshRateTableIndex = XGI_GetRatePtrCRT2(HwDeviceExtension, ModeNo,
			ModeIdIndex, XGI_Pr);

	/*
	temp = XGI_SearchModeID(ModeNo , &ModeIdIndex,  XGI_Pr) ;
	if (!temp) {
		printk(KERN_ERR "Could not find mode %x\n", ModeNo);
		return 65000;
	}

	RefreshRateTableIndex = XGI_Pr->EModeIDTable[ModeIdIndex].REFindex;
	RefreshRateTableIndex += (rateindex - 1);

	*/
	ClockIndex = XGI_Pr->RefIndex[RefreshRateTableIndex].Ext_CRTVCLK;

	Clock = XGI_Pr->VCLKData[ClockIndex].CLOCK * 1000;

	return Clock;
}

static int XGIfb_mode_rate_to_ddata(struct vb_device_info *XGI_Pr,
		struct xgi_hw_device_info *HwDeviceExtension,
		unsigned char modeno, unsigned char rateindex,
		u32 *left_margin, u32 *right_margin, u32 *upper_margin,
		u32 *lower_margin, u32 *hsync_len, u32 *vsync_len, u32 *sync,
		u32 *vmode)
{
	unsigned short ModeNo = modeno;
	unsigned short ModeIdIndex = 0, index = 0;
	unsigned short RefreshRateTableIndex = 0;

	unsigned short VRE, VBE, VRS, VBS, VDE, VT;
	unsigned short HRE, HBE, HRS, HBS, HDE, HT;
	unsigned char sr_data, cr_data, cr_data2;
	unsigned long cr_data3;
	int A, B, C, D, E, F, temp, j;
	XGI_Pr->ROMAddr = HwDeviceExtension->pjVirtualRomBase;
	InitTo330Pointer(HwDeviceExtension->jChipType, XGI_Pr);
	RefreshRateTableIndex = XGI_GetRatePtrCRT2(HwDeviceExtension, ModeNo,
			ModeIdIndex, XGI_Pr);
	/*
	temp = XGI_SearchModeID(ModeNo, &ModeIdIndex, XGI_Pr);
	if (!temp)
		return 0;

	RefreshRateTableIndex = XGI_Pr->EModeIDTable[ModeIdIndex].REFindex;
	RefreshRateTableIndex += (rateindex - 1);
	*/
	index = XGI_Pr->RefIndex[RefreshRateTableIndex].Ext_CRT1CRTC;

	sr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[5];

	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[0];

	/* Horizontal total */
	HT = (cr_data & 0xff) | ((unsigned short) (sr_data & 0x03) << 8);
	A = HT + 5;

	/*
	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[1];

	Horizontal display enable end
	HDE = (cr_data & 0xff) | ((unsigned short) (sr_data & 0x0C) << 6);
	*/
	HDE = (XGI_Pr->RefIndex[RefreshRateTableIndex].XRes >> 3) - 1;
	E = HDE + 1;

	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[3];

	/* Horizontal retrace (=sync) start */
	HRS = (cr_data & 0xff) | ((unsigned short) (sr_data & 0xC0) << 2);
	F = HRS - E - 3;

	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[1];

	/* Horizontal blank start */
	HBS = (cr_data & 0xff) | ((unsigned short) (sr_data & 0x30) << 4);

	sr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[6];

	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[2];

	cr_data2 = XGI_Pr->XGINEWUB_CRT1Table[index].CR[4];

	/* Horizontal blank end */
	HBE = (cr_data & 0x1f) | ((unsigned short) (cr_data2 & 0x80) >> 2)
			| ((unsigned short) (sr_data & 0x03) << 6);

	/* Horizontal retrace (=sync) end */
	HRE = (cr_data2 & 0x1f) | ((sr_data & 0x04) << 3);

	temp = HBE - ((E - 1) & 255);
	B = (temp > 0) ? temp : (temp + 256);

	temp = HRE - ((E + F + 3) & 63);
	C = (temp > 0) ? temp : (temp + 64);

	D = B - F - C;

	*left_margin = D * 8;
	*right_margin = F * 8;
	*hsync_len = C * 8;

	sr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[14];

	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[8];

	cr_data2 = XGI_Pr->XGINEWUB_CRT1Table[index].CR[9];

	/* Vertical total */
	VT = (cr_data & 0xFF) | ((unsigned short) (cr_data2 & 0x01) << 8)
			| ((unsigned short) (cr_data2 & 0x20) << 4)
			| ((unsigned short) (sr_data & 0x01) << 10);
	A = VT + 2;

	/* cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[10]; */

	/* Vertical display enable end */
	/*
	VDE = (cr_data & 0xff) |
		((unsigned short) (cr_data2 & 0x02) << 7) |
		((unsigned short) (cr_data2 & 0x40) << 3) |
		((unsigned short) (sr_data & 0x02) << 9);
	*/
	VDE = XGI_Pr->RefIndex[RefreshRateTableIndex].YRes - 1;
	E = VDE + 1;

	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[10];

	/* Vertical retrace (=sync) start */
	VRS = (cr_data & 0xff) | ((unsigned short) (cr_data2 & 0x04) << 6)
			| ((unsigned short) (cr_data2 & 0x80) << 2)
			| ((unsigned short) (sr_data & 0x08) << 7);
	F = VRS + 1 - E;

	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[12];

	cr_data3 = (XGI_Pr->XGINEWUB_CRT1Table[index].CR[14] & 0x80) << 5;

	/* Vertical blank start */
	VBS = (cr_data & 0xff) | ((unsigned short) (cr_data2 & 0x08) << 5)
			| ((unsigned short) (cr_data3 & 0x20) << 4)
			| ((unsigned short) (sr_data & 0x04) << 8);

	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[13];

	/* Vertical blank end */
	VBE = (cr_data & 0xff) | ((unsigned short) (sr_data & 0x10) << 4);
	temp = VBE - ((E - 1) & 511);
	B = (temp > 0) ? temp : (temp + 512);

	cr_data = XGI_Pr->XGINEWUB_CRT1Table[index].CR[11];

	/* Vertical retrace (=sync) end */
	VRE = (cr_data & 0x0f) | ((sr_data & 0x20) >> 1);
	temp = VRE - ((E + F - 1) & 31);
	C = (temp > 0) ? temp : (temp + 32);

	D = B - F - C;

	*upper_margin = D;
	*lower_margin = F;
	*vsync_len = C;

	if (XGI_Pr->RefIndex[RefreshRateTableIndex].Ext_InfoFlag & 0x8000)
		*sync &= ~FB_SYNC_VERT_HIGH_ACT;
	else
		*sync |= FB_SYNC_VERT_HIGH_ACT;

	if (XGI_Pr->RefIndex[RefreshRateTableIndex].Ext_InfoFlag & 0x4000)
		*sync &= ~FB_SYNC_HOR_HIGH_ACT;
	else
		*sync |= FB_SYNC_HOR_HIGH_ACT;

	*vmode = FB_VMODE_NONINTERLACED;
	if (XGI_Pr->RefIndex[RefreshRateTableIndex].Ext_InfoFlag & 0x0080)
		*vmode = FB_VMODE_INTERLACED;
	else {
		j = 0;
		while (XGI_Pr->EModeIDTable[j].Ext_ModeID != 0xff) {
			if (XGI_Pr->EModeIDTable[j].Ext_ModeID ==
			    XGI_Pr->RefIndex[RefreshRateTableIndex].ModeID) {
				if (XGI_Pr->EModeIDTable[j].Ext_ModeFlag &
				    DoubleScanMode) {
					*vmode = FB_VMODE_DOUBLE;
				}
				break;
			}
			j++;
		}
	}

	return 1;
}

static void XGIRegInit(struct vb_device_info *XGI_Pr, unsigned long BaseAddr)
{
	XGI_Pr->RelIO = BaseAddr;
	XGI_Pr->P3c4 = BaseAddr + 0x14;
	XGI_Pr->P3d4 = BaseAddr + 0x24;
	XGI_Pr->P3c0 = BaseAddr + 0x10;
	XGI_Pr->P3ce = BaseAddr + 0x1e;
	XGI_Pr->P3c2 = BaseAddr + 0x12;
	XGI_Pr->P3ca = BaseAddr + 0x1a;
	XGI_Pr->P3c6 = BaseAddr + 0x16;
	XGI_Pr->P3c7 = BaseAddr + 0x17;
	XGI_Pr->P3c8 = BaseAddr + 0x18;
	XGI_Pr->P3c9 = BaseAddr + 0x19;
	XGI_Pr->P3da = BaseAddr + 0x2A;
	/* Digital video interface registers (LCD) */
	XGI_Pr->Part1Port = BaseAddr + XGI_CRT2_PORT_04;
	/* 301 TV Encoder registers */
	XGI_Pr->Part2Port = BaseAddr + XGI_CRT2_PORT_10;
	/* 301 Macrovision registers */
	XGI_Pr->Part3Port = BaseAddr + XGI_CRT2_PORT_12;
	/* 301 VGA2 (and LCD) registers */
	XGI_Pr->Part4Port = BaseAddr + XGI_CRT2_PORT_14;
	/* 301 palette address port registers */
	XGI_Pr->Part5Port = BaseAddr + XGI_CRT2_PORT_14 + 2;

}

/* ------------ Interface for init & mode switching code ------------- */

static unsigned char XGIfb_query_VGA_config_space(
		struct xgi_hw_device_info *pXGIhw_ext, unsigned long offset,
		unsigned long set, unsigned long *value)
{
	static struct pci_dev *pdev = NULL;
	static unsigned char init = 0, valid_pdev = 0;

	if (!set)
		DPRINTK("XGIfb: Get VGA offset 0x%lx\n", offset);
	else
		DPRINTK("XGIfb: Set offset 0x%lx to 0x%lx\n", offset, *value);

	if (!init) {
		init = 1;
		pdev = pci_get_device(PCI_VENDOR_ID_XG, xgi_video_info.chip_id,
				pdev);
		if (pdev) {
			valid_pdev = 1;
			pci_dev_put(pdev);
		}
	}

	if (!valid_pdev) {
		printk(KERN_DEBUG "XGIfb: Can't find XGI %d VGA device.\n",
				xgi_video_info.chip_id);
		return 0;
	}

	if (set == 0)
		pci_read_config_dword(pdev, offset, (u32 *) value);
	else
		pci_write_config_dword(pdev, offset, (u32)(*value));

	return 1;
}

/* ------------------ Internal helper routines ----------------- */

static int XGIfb_GetXG21DefaultLVDSModeIdx(void)
{

	int found_mode = 0;
	int XGIfb_mode_idx = 0;

	found_mode = 0;
	while ((XGIbios_mode[XGIfb_mode_idx].mode_no != 0)
			&& (XGIbios_mode[XGIfb_mode_idx].xres
					<= XGI21_LCDCapList[0].LVDSHDE)) {
		if ((XGIbios_mode[XGIfb_mode_idx].xres
				== XGI21_LCDCapList[0].LVDSHDE)
				&& (XGIbios_mode[XGIfb_mode_idx].yres
						== XGI21_LCDCapList[0].LVDSVDE)
				&& (XGIbios_mode[XGIfb_mode_idx].bpp == 8)) {
			XGIfb_mode_no = XGIbios_mode[XGIfb_mode_idx].mode_no;
			found_mode = 1;
			break;
		}
		XGIfb_mode_idx++;
	}
	if (!found_mode)
		XGIfb_mode_idx = 0;

	return XGIfb_mode_idx;
}

static void XGIfb_search_mode(const char *name)
{
	int i = 0, j = 0, l;

	if (name == NULL) {
		printk(KERN_ERR "XGIfb: Internal error, using default mode.\n");
		xgifb_mode_idx = DEFAULT_MODE;
		if ((xgi_video_info.chip == XG21)
				&& ((xgi_video_info.disp_state & DISPTYPE_DISP2)
						== DISPTYPE_LCD)) {
			xgifb_mode_idx = XGIfb_GetXG21DefaultLVDSModeIdx();
		}
		return;
	}

	if (!strcmp(name, XGIbios_mode[MODE_INDEX_NONE].name)) {
		printk(KERN_ERR "XGIfb: Mode 'none' not supported anymore. Using default.\n");
		xgifb_mode_idx = DEFAULT_MODE;
		if ((xgi_video_info.chip == XG21)
				&& ((xgi_video_info.disp_state & DISPTYPE_DISP2)
						== DISPTYPE_LCD)) {
			xgifb_mode_idx = XGIfb_GetXG21DefaultLVDSModeIdx();
		}
		return;
	}

	while (XGIbios_mode[i].mode_no != 0) {
		l = min(strlen(name), strlen(XGIbios_mode[i].name));
		if (!strncmp(name, XGIbios_mode[i].name, l)) {
			xgifb_mode_idx = i;
			j = 1;
			break;
		}
		i++;
	}
	if (!j)
		printk(KERN_INFO "XGIfb: Invalid mode '%s'\n", name);
}

static void XGIfb_search_vesamode(unsigned int vesamode)
{
	int i = 0, j = 0;

	if (vesamode == 0) {

		printk(KERN_ERR "XGIfb: Mode 'none' not supported anymore. Using default.\n");
		xgifb_mode_idx = DEFAULT_MODE;
		if ((xgi_video_info.chip == XG21)
				&& ((xgi_video_info.disp_state & DISPTYPE_DISP2)
						== DISPTYPE_LCD)) {
			xgifb_mode_idx = XGIfb_GetXG21DefaultLVDSModeIdx();
		}
		return;
	}

	vesamode &= 0x1dff; /* Clean VESA mode number from other flags */

	while (XGIbios_mode[i].mode_no != 0) {
		if ((XGIbios_mode[i].vesa_mode_no_1 == vesamode) ||
		    (XGIbios_mode[i].vesa_mode_no_2 == vesamode)) {
			xgifb_mode_idx = i;
			j = 1;
			break;
		}
		i++;
	}
	if (!j)
		printk(KERN_INFO "XGIfb: Invalid VESA mode 0x%x'\n", vesamode);
}

static int XGIfb_GetXG21LVDSData(void)
{
	u8 tmp;
	unsigned char *pData;
	int i, j, k;

	tmp = xgifb_reg_get(XGISR, 0x1e);
	xgifb_reg_set(XGISR, 0x1e, tmp | 4);

	pData = xgi_video_info.mmio_vbase + 0x20000;
	if ((pData[0x0] == 0x55) &&
	    (pData[0x1] == 0xAA) &&
	    (pData[0x65] & 0x1)) {
		i = pData[0x316] | (pData[0x317] << 8);
		j = pData[i - 1];
		if (j == 0xff)
			j = 1;

		k = 0;
		do {
			XGI21_LCDCapList[k].LVDS_Capability = pData[i]
					| (pData[i + 1] << 8);
			XGI21_LCDCapList[k].LVDSHT = pData[i + 2] | (pData[i
					+ 3] << 8);
			XGI21_LCDCapList[k].LVDSVT = pData[i + 4] | (pData[i
					+ 5] << 8);
			XGI21_LCDCapList[k].LVDSHDE = pData[i + 6] | (pData[i
					+ 7] << 8);
			XGI21_LCDCapList[k].LVDSVDE = pData[i + 8] | (pData[i
					+ 9] << 8);
			XGI21_LCDCapList[k].LVDSHFP = pData[i + 10] | (pData[i
					+ 11] << 8);
			XGI21_LCDCapList[k].LVDSVFP = pData[i + 12] | (pData[i
					+ 13] << 8);
			XGI21_LCDCapList[k].LVDSHSYNC = pData[i + 14]
					| (pData[i + 15] << 8);
			XGI21_LCDCapList[k].LVDSVSYNC = pData[i + 16]
					| (pData[i + 17] << 8);
			XGI21_LCDCapList[k].VCLKData1 = pData[i + 18];
			XGI21_LCDCapList[k].VCLKData2 = pData[i + 19];
			XGI21_LCDCapList[k].PSC_S1 = pData[i + 20];
			XGI21_LCDCapList[k].PSC_S2 = pData[i + 21];
			XGI21_LCDCapList[k].PSC_S3 = pData[i + 22];
			XGI21_LCDCapList[k].PSC_S4 = pData[i + 23];
			XGI21_LCDCapList[k].PSC_S5 = pData[i + 24];
			i += 25;
			j--;
			k++;
		} while ((j > 0) && (k < (sizeof(XGI21_LCDCapList)
				/ sizeof(struct XGI21_LVDSCapStruct))));
		return 1;
	}
	return 0;
}

static int XGIfb_validate_mode(int myindex)
{
	u16 xres, yres;

	if (xgi_video_info.chip == XG21) {
		if ((xgi_video_info.disp_state & DISPTYPE_DISP2)
				== DISPTYPE_LCD) {
			xres = XGI21_LCDCapList[0].LVDSHDE;
			yres = XGI21_LCDCapList[0].LVDSVDE;
			if (XGIbios_mode[myindex].xres > xres)
				return -1;
			if (XGIbios_mode[myindex].yres > yres)
				return -1;
			if ((XGIbios_mode[myindex].xres < xres) &&
			    (XGIbios_mode[myindex].yres < yres)) {
				if (XGIbios_mode[myindex].bpp > 8)
					return -1;
			}

		}
		return myindex;

	}

	/* FIXME: for now, all is valid on XG27 */
	if (xgi_video_info.chip == XG27)
		return myindex;

	if (!(XGIbios_mode[myindex].chipset & MD_XGI315))
		return -1;

	switch (xgi_video_info.disp_state & DISPTYPE_DISP2) {
	case DISPTYPE_LCD:
		switch (XGIhw_ext.ulCRT2LCDType) {
		case LCD_640x480:
			xres = 640;
			yres = 480;
			break;
		case LCD_800x600:
			xres = 800;
			yres = 600;
			break;
		case LCD_1024x600:
			xres = 1024;
			yres = 600;
			break;
		case LCD_1024x768:
			xres = 1024;
			yres = 768;
			break;
		case LCD_1152x768:
			xres = 1152;
			yres = 768;
			break;
		case LCD_1280x960:
			xres = 1280;
			yres = 960;
			break;
		case LCD_1280x768:
			xres = 1280;
			yres = 768;
			break;
		case LCD_1280x1024:
			xres = 1280;
			yres = 1024;
			break;
		case LCD_1400x1050:
			xres = 1400;
			yres = 1050;
			break;
		case LCD_1600x1200:
			xres = 1600;
			yres = 1200;
			break;
		/* case LCD_320x480: */ /* TW: FSTN */
			/*
			xres =  320;
			yres =  480;
			break;
			*/
		default:
			xres = 0;
			yres = 0;
			break;
		}
		if (XGIbios_mode[myindex].xres > xres)
			return -1;
		if (XGIbios_mode[myindex].yres > yres)
			return -1;
		if ((XGIhw_ext.ulExternalChip == 0x01) || /* LVDS */
		    (XGIhw_ext.ulExternalChip == 0x05)) { /* LVDS+Chrontel */
			switch (XGIbios_mode[myindex].xres) {
			case 512:
				if (XGIbios_mode[myindex].yres != 512)
					return -1;
				if (XGIhw_ext.ulCRT2LCDType == LCD_1024x600)
					return -1;
				break;
			case 640:
				if ((XGIbios_mode[myindex].yres != 400)
						&& (XGIbios_mode[myindex].yres
								!= 480))
					return -1;
				break;
			case 800:
				if (XGIbios_mode[myindex].yres != 600)
					return -1;
				break;
			case 1024:
				if ((XGIbios_mode[myindex].yres != 600) &&
				    (XGIbios_mode[myindex].yres != 768))
					return -1;
				if ((XGIbios_mode[myindex].yres == 600) &&
				    (XGIhw_ext.ulCRT2LCDType != LCD_1024x600))
					return -1;
				break;
			case 1152:
				if ((XGIbios_mode[myindex].yres) != 768)
					return -1;
				if (XGIhw_ext.ulCRT2LCDType != LCD_1152x768)
					return -1;
				break;
			case 1280:
				if ((XGIbios_mode[myindex].yres != 768) &&
				    (XGIbios_mode[myindex].yres != 1024))
					return -1;
				if ((XGIbios_mode[myindex].yres == 768) &&
				    (XGIhw_ext.ulCRT2LCDType != LCD_1280x768))
					return -1;
				break;
			case 1400:
				if (XGIbios_mode[myindex].yres != 1050)
					return -1;
				break;
			case 1600:
				if (XGIbios_mode[myindex].yres != 1200)
					return -1;
				break;
			default:
				return -1;
			}
		} else {
			switch (XGIbios_mode[myindex].xres) {
			case 512:
				if (XGIbios_mode[myindex].yres != 512)
					return -1;
				break;
			case 640:
				if ((XGIbios_mode[myindex].yres != 400) &&
				    (XGIbios_mode[myindex].yres != 480))
					return -1;
				break;
			case 800:
				if (XGIbios_mode[myindex].yres != 600)
					return -1;
				break;
			case 1024:
				if (XGIbios_mode[myindex].yres != 768)
					return -1;
				break;
			case 1280:
				if ((XGIbios_mode[myindex].yres != 960) &&
				    (XGIbios_mode[myindex].yres != 1024))
					return -1;
				if (XGIbios_mode[myindex].yres == 960) {
					if (XGIhw_ext.ulCRT2LCDType ==
					    LCD_1400x1050)
						return -1;
				}
				break;
			case 1400:
				if (XGIbios_mode[myindex].yres != 1050)
					return -1;
				break;
			case 1600:
				if (XGIbios_mode[myindex].yres != 1200)
					return -1;
				break;
			default:
				return -1;
			}
		}
		break;
	case DISPTYPE_TV:
		switch (XGIbios_mode[myindex].xres) {
		case 512:
		case 640:
		case 800:
			break;
		case 720:
			if (xgi_video_info.TV_type == TVMODE_NTSC) {
				if (XGIbios_mode[myindex].yres != 480)
					return -1;
			} else if (xgi_video_info.TV_type == TVMODE_PAL) {
				if (XGIbios_mode[myindex].yres != 576)
					return -1;
			}
			/*  TW: LVDS/CHRONTEL does not support 720 */
			if (xgi_video_info.hasVB == HASVB_LVDS_CHRONTEL ||
			    xgi_video_info.hasVB == HASVB_CHRONTEL) {
				return -1;
			}
			break;
		case 1024:
			if (xgi_video_info.TV_type == TVMODE_NTSC) {
				if (XGIbios_mode[myindex].bpp == 32)
					return -1;
			}
			break;
		default:
			return -1;
		}
		break;
	case DISPTYPE_CRT2:
		if (XGIbios_mode[myindex].xres > 1280)
			return -1;
		break;
	}
	return myindex;

}

static void XGIfb_search_crt2type(const char *name)
{
	int i = 0;

	if (name == NULL)
		return;

	while (XGI_crt2type[i].type_no != -1) {
		if (!strcmp(name, XGI_crt2type[i].name)) {
			XGIfb_crt2type = XGI_crt2type[i].type_no;
			XGIfb_tvplug = XGI_crt2type[i].tvplug_no;
			break;
		}
		i++;
	}
	if (XGIfb_crt2type < 0)
		printk(KERN_INFO "XGIfb: Invalid CRT2 type: %s\n", name);
}

static u8 XGIfb_search_refresh_rate(unsigned int rate)
{
	u16 xres, yres;
	int i = 0;

	xres = XGIbios_mode[xgifb_mode_idx].xres;
	yres = XGIbios_mode[xgifb_mode_idx].yres;

	XGIfb_rate_idx = 0;
	while ((XGIfb_vrate[i].idx != 0) && (XGIfb_vrate[i].xres <= xres)) {
		if ((XGIfb_vrate[i].xres == xres) &&
		    (XGIfb_vrate[i].yres == yres)) {
			if (XGIfb_vrate[i].refresh == rate) {
				XGIfb_rate_idx = XGIfb_vrate[i].idx;
				break;
			} else if (XGIfb_vrate[i].refresh > rate) {
				if ((XGIfb_vrate[i].refresh - rate) <= 3) {
					DPRINTK("XGIfb: Adjusting rate from %d up to %d\n",
						rate, XGIfb_vrate[i].refresh);
					XGIfb_rate_idx = XGIfb_vrate[i].idx;
					xgi_video_info.refresh_rate =
						XGIfb_vrate[i].refresh;
				} else if (((rate - XGIfb_vrate[i - 1].refresh)
						<= 2) && (XGIfb_vrate[i].idx
						!= 1)) {
					DPRINTK("XGIfb: Adjusting rate from %d down to %d\n",
						rate, XGIfb_vrate[i-1].refresh);
					XGIfb_rate_idx = XGIfb_vrate[i - 1].idx;
					xgi_video_info.refresh_rate =
						XGIfb_vrate[i - 1].refresh;
				}
				break;
			} else if ((rate - XGIfb_vrate[i].refresh) <= 2) {
				DPRINTK("XGIfb: Adjusting rate from %d down to %d\n",
					rate, XGIfb_vrate[i].refresh);
				XGIfb_rate_idx = XGIfb_vrate[i].idx;
				break;
			}
		}
		i++;
	}
	if (XGIfb_rate_idx > 0) {
		return XGIfb_rate_idx;
	} else {
		printk(KERN_INFO "XGIfb: Unsupported rate %d for %dx%d\n",
		       rate, xres, yres);
		return 0;
	}
}

static void XGIfb_search_tvstd(const char *name)
{
	int i = 0;

	if (name == NULL)
		return;

	while (XGI_tvtype[i].type_no != -1) {
		if (!strcmp(name, XGI_tvtype[i].name)) {
			XGIfb_tvmode = XGI_tvtype[i].type_no;
			break;
		}
		i++;
	}
}

/* ----------- FBDev related routines for all series ----------- */

static void XGIfb_bpp_to_var(struct fb_var_screeninfo *var)
{
	switch (var->bits_per_pixel) {
	case 8:
		var->red.offset = var->green.offset = var->blue.offset = 0;
		var->red.length = var->green.length = var->blue.length = 6;
		xgi_video_info.video_cmap_len = 256;
		break;
	case 16:
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
		xgi_video_info.video_cmap_len = 16;
		break;
	case 32:
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		xgi_video_info.video_cmap_len = 16;
		break;
	}
}

/* --------------------- SetMode routines ------------------------- */

static void XGIfb_pre_setmode(void)
{
	u8 cr30 = 0, cr31 = 0;

	cr31 = xgifb_reg_get(XGICR, 0x31);
	cr31 &= ~0x60;

	switch (xgi_video_info.disp_state & DISPTYPE_DISP2) {
	case DISPTYPE_CRT2:
		cr30 = (XGI_VB_OUTPUT_CRT2 | XGI_SIMULTANEOUS_VIEW_ENABLE);
		cr31 |= XGI_DRIVER_MODE;
		break;
	case DISPTYPE_LCD:
		cr30 = (XGI_VB_OUTPUT_LCD | XGI_SIMULTANEOUS_VIEW_ENABLE);
		cr31 |= XGI_DRIVER_MODE;
		break;
	case DISPTYPE_TV:
		if (xgi_video_info.TV_type == TVMODE_HIVISION)
			cr30 = (XGI_VB_OUTPUT_HIVISION
					| XGI_SIMULTANEOUS_VIEW_ENABLE);
		else if (xgi_video_info.TV_plug == TVPLUG_SVIDEO)
			cr30 = (XGI_VB_OUTPUT_SVIDEO
					| XGI_SIMULTANEOUS_VIEW_ENABLE);
		else if (xgi_video_info.TV_plug == TVPLUG_COMPOSITE)
			cr30 = (XGI_VB_OUTPUT_COMPOSITE
					| XGI_SIMULTANEOUS_VIEW_ENABLE);
		else if (xgi_video_info.TV_plug == TVPLUG_SCART)
			cr30 = (XGI_VB_OUTPUT_SCART
					| XGI_SIMULTANEOUS_VIEW_ENABLE);
		cr31 |= XGI_DRIVER_MODE;

		if (XGIfb_tvmode == 1 || xgi_video_info.TV_type == TVMODE_PAL)
			cr31 |= 0x01;
		else
			cr31 &= ~0x01;
		break;
	default: /* disable CRT2 */
		cr30 = 0x00;
		cr31 |= (XGI_DRIVER_MODE | XGI_VB_OUTPUT_DISABLE);
	}

	xgifb_reg_set(XGICR, IND_XGI_SCRATCH_REG_CR30, cr30);
	xgifb_reg_set(XGICR, IND_XGI_SCRATCH_REG_CR31, cr31);
	xgifb_reg_set(XGICR, IND_XGI_SCRATCH_REG_CR33, (XGIfb_rate_idx & 0x0F));
}

static void XGIfb_post_setmode(void)
{
	u8 reg;
	unsigned char doit = 1;
	/*
	xgifb_reg_set(XGISR,IND_XGI_PASSWORD,XGI_PASSWORD);
	xgifb_reg_set(XGICR, 0x13, 0x00);
	xgifb_reg_and_or(XGISR,0x0E, 0xF0, 0x01);
	*test*
	*/
	if (xgi_video_info.video_bpp == 8) {
		/* TW: We can't switch off CRT1 on LVDS/Chrontel
		 * in 8bpp Modes */
		if ((xgi_video_info.hasVB == HASVB_LVDS) ||
		    (xgi_video_info.hasVB == HASVB_LVDS_CHRONTEL)) {
			doit = 0;
		}
		/* TW: We can't switch off CRT1 on 301B-DH
		 * in 8bpp Modes if using LCD */
		if (xgi_video_info.disp_state & DISPTYPE_LCD)
			doit = 0;
	}

	/* TW: We can't switch off CRT1 if bridge is in slave mode */
	if (xgi_video_info.hasVB != HASVB_NONE) {
		reg = xgifb_reg_get(XGIPART1, 0x00);

		if ((reg & 0x50) == 0x10)
			doit = 0;

	} else {
		XGIfb_crt1off = 0;
	}

	reg = xgifb_reg_get(XGICR, 0x17);
	if ((XGIfb_crt1off) && (doit))
		reg &= ~0x80;
	else
		reg |= 0x80;
	xgifb_reg_set(XGICR, 0x17, reg);

	xgifb_reg_and(XGISR, IND_XGI_RAMDAC_CONTROL, ~0x04);

	if ((xgi_video_info.disp_state & DISPTYPE_TV) && (xgi_video_info.hasVB
			== HASVB_301)) {

		reg = xgifb_reg_get(XGIPART4, 0x01);

		if (reg < 0xB0) { /* Set filter for XGI301 */
			switch (xgi_video_info.video_width) {
			case 320:
				filter_tb = (xgi_video_info.TV_type ==
					     TVMODE_NTSC) ? 4 : 12;
				break;
			case 640:
				filter_tb = (xgi_video_info.TV_type ==
					     TVMODE_NTSC) ? 5 : 13;
				break;
			case 720:
				filter_tb = (xgi_video_info.TV_type ==
					     TVMODE_NTSC) ? 6 : 14;
				break;
			case 800:
				filter_tb = (xgi_video_info.TV_type ==
					     TVMODE_NTSC) ? 7 : 15;
				break;
			default:
				filter = -1;
				break;
			}
			xgifb_reg_or(XGIPART1, XGIfb_CRT2_write_enable, 0x01);

			if (xgi_video_info.TV_type == TVMODE_NTSC) {

				xgifb_reg_and(XGIPART2, 0x3a, 0x1f);

				if (xgi_video_info.TV_plug == TVPLUG_SVIDEO) {

					xgifb_reg_and(XGIPART2, 0x30, 0xdf);

				} else if (xgi_video_info.TV_plug
						== TVPLUG_COMPOSITE) {

					xgifb_reg_or(XGIPART2, 0x30, 0x20);

					switch (xgi_video_info.video_width) {
					case 640:
						xgifb_reg_set(XGIPART2,
							      0x35,
							      0xEB);
						xgifb_reg_set(XGIPART2,
							      0x36,
							      0x04);
						xgifb_reg_set(XGIPART2,
							      0x37,
							      0x25);
						xgifb_reg_set(XGIPART2,
							      0x38,
							      0x18);
						break;
					case 720:
						xgifb_reg_set(XGIPART2,
							      0x35,
							      0xEE);
						xgifb_reg_set(XGIPART2,
							      0x36,
							      0x0C);
						xgifb_reg_set(XGIPART2,
							      0x37,
							      0x22);
						xgifb_reg_set(XGIPART2,
							      0x38,
							      0x08);
						break;
					case 800:
						xgifb_reg_set(XGIPART2,
							      0x35,
							      0xEB);
						xgifb_reg_set(XGIPART2,
							      0x36,
							      0x15);
						xgifb_reg_set(XGIPART2,
							      0x37,
							      0x25);
						xgifb_reg_set(XGIPART2,
							      0x38,
							      0xF6);
						break;
					}
				}

			} else if (xgi_video_info.TV_type == TVMODE_PAL) {

				xgifb_reg_and(XGIPART2, 0x3A, 0x1F);

				if (xgi_video_info.TV_plug == TVPLUG_SVIDEO) {

					xgifb_reg_and(XGIPART2, 0x30, 0xDF);

				} else if (xgi_video_info.TV_plug
						== TVPLUG_COMPOSITE) {

					xgifb_reg_or(XGIPART2, 0x30, 0x20);

					switch (xgi_video_info.video_width) {
					case 640:
						xgifb_reg_set(XGIPART2,
							      0x35,
							      0xF1);
						xgifb_reg_set(XGIPART2,
							      0x36,
							      0xF7);
						xgifb_reg_set(XGIPART2,
							      0x37,
							      0x1F);
						xgifb_reg_set(XGIPART2,
							      0x38,
							      0x32);
						break;
					case 720:
						xgifb_reg_set(XGIPART2,
							      0x35,
							      0xF3);
						xgifb_reg_set(XGIPART2,
							      0x36,
							      0x00);
						xgifb_reg_set(XGIPART2,
							      0x37,
							      0x1D);
						xgifb_reg_set(XGIPART2,
							      0x38,
							      0x20);
						break;
					case 800:
						xgifb_reg_set(XGIPART2,
							      0x35,
							      0xFC);
						xgifb_reg_set(XGIPART2,
							      0x36,
							      0xFB);
						xgifb_reg_set(XGIPART2,
							      0x37,
							      0x14);
						xgifb_reg_set(XGIPART2,
							      0x38,
							      0x2A);
						break;
					}
				}
			}

			if ((filter >= 0) && (filter <= 7)) {
				DPRINTK("FilterTable[%d]-%d: %02x %02x %02x %02x\n",
					filter_tb, filter,
					XGI_TV_filter[filter_tb].
						filter[filter][0],
					XGI_TV_filter[filter_tb].
						filter[filter][1],
					XGI_TV_filter[filter_tb].
						filter[filter][2],
					XGI_TV_filter[filter_tb].
						filter[filter][3]
				);
				xgifb_reg_set(
					XGIPART2,
					0x35,
					(XGI_TV_filter[filter_tb].
						filter[filter][0]));
				xgifb_reg_set(
					XGIPART2,
					0x36,
					(XGI_TV_filter[filter_tb].
						filter[filter][1]));
				xgifb_reg_set(
					XGIPART2,
					0x37,
					(XGI_TV_filter[filter_tb].
						filter[filter][2]));
				xgifb_reg_set(
					XGIPART2,
					0x38,
					(XGI_TV_filter[filter_tb].
						filter[filter][3]));
			}
		}
	}
}

static int XGIfb_do_set_var(struct fb_var_screeninfo *var, int isactive,
		struct fb_info *info)
{

	unsigned int htotal = var->left_margin + var->xres + var->right_margin
			+ var->hsync_len;
	unsigned int vtotal = var->upper_margin + var->yres + var->lower_margin
			+ var->vsync_len;
#if defined(__powerpc__)
	u8 sr_data, cr_data;
#endif
	unsigned int drate = 0, hrate = 0;
	int found_mode = 0;
	int old_mode;
	/* unsigned char reg, reg1; */

	DEBUGPRN("Inside do_set_var");
	/* printk(KERN_DEBUG "XGIfb:var->yres=%d, var->upper_margin=%d, var->lower_margin=%d, var->vsync_len=%d\n", var->yres, var->upper_margin, var->lower_margin, var->vsync_len); */

	info->var.xres_virtual = var->xres_virtual;
	info->var.yres_virtual = var->yres_virtual;
	info->var.bits_per_pixel = var->bits_per_pixel;

	if ((var->vmode & FB_VMODE_MASK) == FB_VMODE_NONINTERLACED)
		vtotal <<= 1;
	else if ((var->vmode & FB_VMODE_MASK) == FB_VMODE_DOUBLE)
		vtotal <<= 2;
	else if ((var->vmode & FB_VMODE_MASK) == FB_VMODE_INTERLACED) {
		/* vtotal <<= 1; */
		/* var->yres <<= 1; */
	}

	if (!htotal || !vtotal) {
		DPRINTK("XGIfb: Invalid 'var' information\n");
		return -EINVAL;
	} printk(KERN_DEBUG "XGIfb: var->pixclock=%d, htotal=%d, vtotal=%d\n",
			var->pixclock, htotal, vtotal);

	if (var->pixclock && htotal && vtotal) {
		drate = 1000000000 / var->pixclock;
		hrate = (drate * 1000) / htotal;
		xgi_video_info.refresh_rate = (unsigned int) (hrate * 2
				/ vtotal);
	} else {
		xgi_video_info.refresh_rate = 60;
	}

	printk(KERN_DEBUG "XGIfb: Change mode to %dx%dx%d-%dHz\n",
	       var->xres,
	       var->yres,
	       var->bits_per_pixel,
	       xgi_video_info.refresh_rate);

	old_mode = xgifb_mode_idx;
	xgifb_mode_idx = 0;

	while ((XGIbios_mode[xgifb_mode_idx].mode_no != 0)
			&& (XGIbios_mode[xgifb_mode_idx].xres <= var->xres)) {
		if ((XGIbios_mode[xgifb_mode_idx].xres == var->xres)
				&& (XGIbios_mode[xgifb_mode_idx].yres
						== var->yres)
				&& (XGIbios_mode[xgifb_mode_idx].bpp
						== var->bits_per_pixel)) {
			XGIfb_mode_no = XGIbios_mode[xgifb_mode_idx].mode_no;
			found_mode = 1;
			break;
		}
		xgifb_mode_idx++;
	}

	if (found_mode)
		xgifb_mode_idx = XGIfb_validate_mode(xgifb_mode_idx);
	else
		xgifb_mode_idx = -1;

	if (xgifb_mode_idx < 0) {
		printk(KERN_ERR "XGIfb: Mode %dx%dx%d not supported\n",
		       var->xres, var->yres, var->bits_per_pixel);
		xgifb_mode_idx = old_mode;
		return -EINVAL;
	}

	if (XGIfb_search_refresh_rate(xgi_video_info.refresh_rate) == 0) {
		XGIfb_rate_idx = XGIbios_mode[xgifb_mode_idx].rate_idx;
		xgi_video_info.refresh_rate = 60;
	}

	if (isactive) {

		XGIfb_pre_setmode();
		if (XGISetModeNew(&XGIhw_ext, XGIfb_mode_no) == 0) {
			printk(KERN_ERR "XGIfb: Setting mode[0x%x] failed\n",
			       XGIfb_mode_no);
			return -EINVAL;
		}
		info->fix.line_length = ((info->var.xres_virtual
				* info->var.bits_per_pixel) >> 6);

		xgifb_reg_set(XGISR, IND_XGI_PASSWORD, XGI_PASSWORD);

		xgifb_reg_set(XGICR, 0x13, (info->fix.line_length & 0x00ff));
		xgifb_reg_set(XGISR,
			      0x0E,
			      (info->fix.line_length & 0xff00) >> 8);

		XGIfb_post_setmode();

		DPRINTK("XGIfb: Set new mode: %dx%dx%d-%d\n",
				XGIbios_mode[xgifb_mode_idx].xres,
				XGIbios_mode[xgifb_mode_idx].yres,
				XGIbios_mode[xgifb_mode_idx].bpp,
				xgi_video_info.refresh_rate);

		xgi_video_info.video_bpp = XGIbios_mode[xgifb_mode_idx].bpp;
		xgi_video_info.video_vwidth = info->var.xres_virtual;
		xgi_video_info.video_width = XGIbios_mode[xgifb_mode_idx].xres;
		xgi_video_info.video_vheight = info->var.yres_virtual;
		xgi_video_info.video_height = XGIbios_mode[xgifb_mode_idx].yres;
		xgi_video_info.org_x = xgi_video_info.org_y = 0;
		xgi_video_info.video_linelength = info->var.xres_virtual
				* (xgi_video_info.video_bpp >> 3);
		switch (xgi_video_info.video_bpp) {
		case 8:
			xgi_video_info.DstColor = 0x0000;
			xgi_video_info.XGI310_AccelDepth = 0x00000000;
			xgi_video_info.video_cmap_len = 256;
#if defined(__powerpc__)
			cr_data = xgifb_reg_get(XGICR, 0x4D);
			xgifb_reg_set(XGICR, 0x4D, (cr_data & 0xE0));
#endif
			break;
		case 16:
			xgi_video_info.DstColor = 0x8000;
			xgi_video_info.XGI310_AccelDepth = 0x00010000;
#if defined(__powerpc__)
			cr_data = xgifb_reg_get(XGICR, 0x4D);
			xgifb_reg_set(XGICR, 0x4D, ((cr_data & 0xE0) | 0x0B));
#endif
			xgi_video_info.video_cmap_len = 16;
			break;
		case 32:
			xgi_video_info.DstColor = 0xC000;
			xgi_video_info.XGI310_AccelDepth = 0x00020000;
			xgi_video_info.video_cmap_len = 16;
#if defined(__powerpc__)
			cr_data = xgifb_reg_get(XGICR, 0x4D);
			xgifb_reg_set(XGICR, 0x4D, ((cr_data & 0xE0) | 0x15));
#endif
			break;
		default:
			xgi_video_info.video_cmap_len = 16;
			printk(KERN_ERR "XGIfb: Unsupported depth %d",
			       xgi_video_info.video_bpp);
			break;
		}
	}
	XGIfb_bpp_to_var(var); /*update ARGB info*/
	DEBUGPRN("End of do_set_var");

	dumpVGAReg();
	return 0;
}

#ifdef XGIFB_PAN
static int XGIfb_pan_var(struct fb_var_screeninfo *var)
{
	unsigned int base;

	/* printk("Inside pan_var"); */

	if (var->xoffset > (var->xres_virtual - var->xres)) {
		/* printk("Pan: xo: %d xv %d xr %d\n",
			var->xoffset, var->xres_virtual, var->xres); */
		return -EINVAL;
	}
	if (var->yoffset > (var->yres_virtual - var->yres)) {
		/* printk("Pan: yo: %d yv %d yr %d\n",
			var->yoffset, var->yres_virtual, var->yres); */
		return -EINVAL;
	}
	base = var->yoffset * var->xres_virtual + var->xoffset;

	/* calculate base bpp dep. */
	switch (var->bits_per_pixel) {
	case 16:
		base >>= 1;
		break;
	case 32:
		break;
	case 8:
	default:
		base >>= 2;
		break;
	}

	xgifb_reg_set(XGISR, IND_XGI_PASSWORD, XGI_PASSWORD);

	xgifb_reg_set(XGICR, 0x0D, base & 0xFF);
	xgifb_reg_set(XGICR, 0x0C, (base >> 8) & 0xFF);
	xgifb_reg_set(XGISR, 0x0D, (base >> 16) & 0xFF);
	xgifb_reg_set(XGISR, 0x37, (base >> 24) & 0x03);
	xgifb_reg_and_or(XGISR, 0x37, 0xDF, (base >> 21) & 0x04);

	if (xgi_video_info.disp_state & DISPTYPE_DISP2) {
		xgifb_reg_or(XGIPART1, XGIfb_CRT2_write_enable, 0x01);
		xgifb_reg_set(XGIPART1, 0x06, (base & 0xFF));
		xgifb_reg_set(XGIPART1, 0x05, ((base >> 8) & 0xFF));
		xgifb_reg_set(XGIPART1, 0x04, ((base >> 16) & 0xFF));
		xgifb_reg_and_or(XGIPART1,
				 0x02,
				 0x7F,
				 ((base >> 24) & 0x01) << 7);
	}
	/* printk("End of pan_var"); */
	return 0;
}
#endif

static int XGIfb_open(struct fb_info *info, int user)
{
	return 0;
}

static int XGIfb_release(struct fb_info *info, int user)
{
	return 0;
}

static int XGIfb_get_cmap_len(const struct fb_var_screeninfo *var)
{
	int rc = 16;

	switch (var->bits_per_pixel) {
	case 8:
		rc = 256;
		break;
	case 16:
		rc = 16;
		break;
	case 32:
		rc = 16;
		break;
	}
	return rc;
}

static int XGIfb_setcolreg(unsigned regno, unsigned red, unsigned green,
		unsigned blue, unsigned transp, struct fb_info *info)
{
	if (regno >= XGIfb_get_cmap_len(&info->var))
		return 1;

	switch (info->var.bits_per_pixel) {
	case 8:
		outb(regno, XGIDACA);
		outb((red >> 10), XGIDACD);
		outb((green >> 10), XGIDACD);
		outb((blue >> 10), XGIDACD);
		if (xgi_video_info.disp_state & DISPTYPE_DISP2) {
			outb(regno, XGIDAC2A);
			outb((red >> 8), XGIDAC2D);
			outb((green >> 8), XGIDAC2D);
			outb((blue >> 8), XGIDAC2D);
		}
		break;
	case 16:
		((u32 *) (info->pseudo_palette))[regno] = ((red & 0xf800))
				| ((green & 0xfc00) >> 5) | ((blue & 0xf800)
				>> 11);
		break;
	case 32:
		red >>= 8;
		green >>= 8;
		blue >>= 8;
		((u32 *) (info->pseudo_palette))[regno] = (red << 16) | (green
				<< 8) | (blue);
		break;
	}
	return 0;
}

/* ----------- FBDev related routines for all series ---------- */

static int XGIfb_get_fix(struct fb_fix_screeninfo *fix, int con,
		struct fb_info *info)
{
	DEBUGPRN("inside get_fix");
	memset(fix, 0, sizeof(struct fb_fix_screeninfo));

	strcpy(fix->id, myid);

	fix->smem_start = xgi_video_info.video_base;

	fix->smem_len = xgi_video_info.video_size;

	fix->type = video_type;
	fix->type_aux = 0;
	if (xgi_video_info.video_bpp == 8)
		fix->visual = FB_VISUAL_PSEUDOCOLOR;
	else
		fix->visual = FB_VISUAL_DIRECTCOLOR;
	fix->xpanstep = 0;
#ifdef XGIFB_PAN
	if (XGIfb_ypan)
		fix->ypanstep = 1;
#endif
	fix->ywrapstep = 0;
	fix->line_length = xgi_video_info.video_linelength;
	fix->mmio_start = xgi_video_info.mmio_base;
	fix->mmio_len = xgi_video_info.mmio_size;
	fix->accel = FB_ACCEL_XGI_XABRE;

	DEBUGPRN("end of get_fix");
	return 0;
}

static int XGIfb_set_par(struct fb_info *info)
{
	int err;

	/* printk("XGIfb: inside set_par\n"); */
	err = XGIfb_do_set_var(&info->var, 1, info);
	if (err)
		return err;
	XGIfb_get_fix(&info->fix, -1, info);
	/* printk("XGIfb: end of set_par\n"); */
	return 0;
}

static int XGIfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	unsigned int htotal = var->left_margin + var->xres + var->right_margin
			+ var->hsync_len;
	unsigned int vtotal = 0;
	unsigned int drate = 0, hrate = 0;
	int found_mode = 0;
	int refresh_rate, search_idx;

	DEBUGPRN("Inside check_var");

	if ((var->vmode & FB_VMODE_MASK) == FB_VMODE_NONINTERLACED) {
		vtotal = var->upper_margin + var->yres + var->lower_margin
				+ var->vsync_len;
		vtotal <<= 1;
	} else if ((var->vmode & FB_VMODE_MASK) == FB_VMODE_DOUBLE) {
		vtotal = var->upper_margin + var->yres + var->lower_margin
				+ var->vsync_len;
		vtotal <<= 2;
	} else if ((var->vmode & FB_VMODE_MASK) == FB_VMODE_INTERLACED) {
		vtotal = var->upper_margin + (var->yres / 2)
				+ var->lower_margin + var->vsync_len;
	} else
		vtotal = var->upper_margin + var->yres + var->lower_margin
				+ var->vsync_len;

	if (!(htotal) || !(vtotal))
		XGIFAIL("XGIfb: no valid timing data");

	if (var->pixclock && htotal && vtotal) {
		drate = 1000000000 / var->pixclock;
		hrate = (drate * 1000) / htotal;
		xgi_video_info.refresh_rate =
			(unsigned int) (hrate * 2 / vtotal);
		printk(KERN_DEBUG
			"%s: pixclock = %d ,htotal=%d, vtotal=%d\n"
			"%s: drate=%d, hrate=%d, refresh_rate=%d\n",
			__func__, var->pixclock, htotal, vtotal,
			__func__, drate, hrate, xgi_video_info.refresh_rate);
	} else {
		xgi_video_info.refresh_rate = 60;
	}

	/*
	if ((var->pixclock) && (htotal)) {
		drate = 1E12 / var->pixclock;
		hrate = drate / htotal;
		refresh_rate = (unsigned int) (hrate / vtotal * 2 + 0.5);
	} else {
		refresh_rate = 60;
	}
	*/
	/* TW: Calculation wrong for 1024x600 - force it to 60Hz */
	if ((var->xres == 1024) && (var->yres == 600))
		refresh_rate = 60;

	search_idx = 0;
	while ((XGIbios_mode[search_idx].mode_no != 0) &&
		(XGIbios_mode[search_idx].xres <= var->xres)) {
		if ((XGIbios_mode[search_idx].xres == var->xres) &&
			(XGIbios_mode[search_idx].yres == var->yres) &&
			(XGIbios_mode[search_idx].bpp == var->bits_per_pixel)) {
			if (XGIfb_validate_mode(search_idx) > 0) {
				found_mode = 1;
				break;
			}
		}
		search_idx++;
	}

	if (!found_mode) {

		printk(KERN_ERR "XGIfb: %dx%dx%d is no valid mode\n",
			var->xres, var->yres, var->bits_per_pixel);
		search_idx = 0;
		while (XGIbios_mode[search_idx].mode_no != 0) {
			if ((var->xres <= XGIbios_mode[search_idx].xres) &&
			    (var->yres <= XGIbios_mode[search_idx].yres) &&
			    (var->bits_per_pixel ==
			     XGIbios_mode[search_idx].bpp)) {
				if (XGIfb_validate_mode(search_idx) > 0) {
					found_mode = 1;
					break;
				}
			}
			search_idx++;
		}
		if (found_mode) {
			var->xres = XGIbios_mode[search_idx].xres;
			var->yres = XGIbios_mode[search_idx].yres;
			printk(KERN_DEBUG "XGIfb: Adapted to mode %dx%dx%d\n",
				var->xres, var->yres, var->bits_per_pixel);

		} else {
			printk(KERN_ERR "XGIfb: Failed to find similar mode to %dx%dx%d\n",
				var->xres, var->yres, var->bits_per_pixel);
			return -EINVAL;
		}
	}

	/* TW: TODO: Check the refresh rate */

	/* Adapt RGB settings */
	XGIfb_bpp_to_var(var);

	/* Sanity check for offsets */
	if (var->xoffset < 0)
		var->xoffset = 0;
	if (var->yoffset < 0)
		var->yoffset = 0;

	if (!XGIfb_ypan) {
		if (var->xres != var->xres_virtual)
			var->xres_virtual = var->xres;
		if (var->yres != var->yres_virtual)
			var->yres_virtual = var->yres;
	} /* else { */
		/* TW: Now patch yres_virtual if we use panning */
		/* May I do this? */
		/* var->yres_virtual = xgi_video_info.heapstart /
			(var->xres * (var->bits_per_pixel >> 3)); */
		/* if (var->yres_virtual <= var->yres) { */
		/* TW: Paranoia check */
		/* var->yres_virtual = var->yres; */
		/* } */
	/* } */

	/* Truncate offsets to maximum if too high */
	if (var->xoffset > var->xres_virtual - var->xres)
		var->xoffset = var->xres_virtual - var->xres - 1;

	if (var->yoffset > var->yres_virtual - var->yres)
		var->yoffset = var->yres_virtual - var->yres - 1;

	/* Set everything else to 0 */
	var->red.msb_right =
	var->green.msb_right =
	var->blue.msb_right =
	var->transp.offset = var->transp.length = var->transp.msb_right = 0;

	DEBUGPRN("end of check_var");
	return 0;
}

#ifdef XGIFB_PAN
static int XGIfb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	int err;

	/* printk("\nInside pan_display:\n"); */

	if (var->xoffset > (var->xres_virtual - var->xres))
		return -EINVAL;
	if (var->yoffset > (var->yres_virtual - var->yres))
		return -EINVAL;

	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset < 0 || var->yoffset >= info->var.yres_virtual
				|| var->xoffset)
			return -EINVAL;
	} else {
		if (var->xoffset + info->var.xres > info->var.xres_virtual
				|| var->yoffset + info->var.yres
						> info->var.yres_virtual)
			return -EINVAL;
	}
	err = XGIfb_pan_var(var);
	if (err < 0)
		return err;

	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;
	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

	/* printk("End of pan_display\n"); */
	return 0;
}
#endif

static int XGIfb_blank(int blank, struct fb_info *info)
{
	u8 reg;

	reg = xgifb_reg_get(XGICR, 0x17);

	if (blank > 0)
		reg &= 0x7f;
	else
		reg |= 0x80;

	xgifb_reg_set(XGICR, 0x17, reg);
	xgifb_reg_set(XGISR, 0x00, 0x01); /* Synchronous Reset */
	xgifb_reg_set(XGISR, 0x00, 0x03); /* End Reset */
	return 0;
}

static struct fb_ops XGIfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = XGIfb_open,
	.fb_release = XGIfb_release,
	.fb_check_var = XGIfb_check_var,
	.fb_set_par = XGIfb_set_par,
	.fb_setcolreg = XGIfb_setcolreg,
#ifdef XGIFB_PAN
	.fb_pan_display = XGIfb_pan_display,
#endif
	.fb_blank = XGIfb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	/* .fb_mmap = XGIfb_mmap, */
};

/* ---------------- Chip generation dependent routines ---------------- */

/* for XGI 315/550/650/740/330 */

static int XGIfb_get_dram_size(void)
{

	u8 ChannelNum, tmp;
	u8 reg = 0;

	/* xorg driver sets 32MB * 1 channel */
	if (xgi_video_info.chip == XG27)
		xgifb_reg_set(XGISR, IND_XGI_DRAM_SIZE, 0x51);

	reg = xgifb_reg_get(XGISR, IND_XGI_DRAM_SIZE);
	switch ((reg & XGI_DRAM_SIZE_MASK) >> 4) {
	case XGI_DRAM_SIZE_1MB:
		xgi_video_info.video_size = 0x100000;
		break;
	case XGI_DRAM_SIZE_2MB:
		xgi_video_info.video_size = 0x200000;
		break;
	case XGI_DRAM_SIZE_4MB:
		xgi_video_info.video_size = 0x400000;
		break;
	case XGI_DRAM_SIZE_8MB:
		xgi_video_info.video_size = 0x800000;
		break;
	case XGI_DRAM_SIZE_16MB:
		xgi_video_info.video_size = 0x1000000;
		break;
	case XGI_DRAM_SIZE_32MB:
		xgi_video_info.video_size = 0x2000000;
		break;
	case XGI_DRAM_SIZE_64MB:
		xgi_video_info.video_size = 0x4000000;
		break;
	case XGI_DRAM_SIZE_128MB:
		xgi_video_info.video_size = 0x8000000;
		break;
	case XGI_DRAM_SIZE_256MB:
		xgi_video_info.video_size = 0x10000000;
		break;
	default:
		return -1;
	}

	tmp = (reg & 0x0c) >> 2;
	switch (xgi_video_info.chip) {
	case XG20:
	case XG21:
	case XG27:
		ChannelNum = 1;
		break;

	case XG42:
		if (reg & 0x04)
			ChannelNum = 2;
		else
			ChannelNum = 1;
		break;

	case XG45:
		if (tmp == 1)
			ChannelNum = 2;
		else if (tmp == 2)
			ChannelNum = 3;
		else if (tmp == 3)
			ChannelNum = 4;
		else
			ChannelNum = 1;
		break;

	case XG40:
	default:
		if (tmp == 2)
			ChannelNum = 2;
		else if (tmp == 3)
			ChannelNum = 3;
		else
			ChannelNum = 1;
		break;
	}

	xgi_video_info.video_size = xgi_video_info.video_size * ChannelNum;
	/* PLiad fixed for benchmarking and fb set */
	/* xgi_video_info.video_size = 0x200000; */ /* 1024x768x16 */
	/* xgi_video_info.video_size = 0x1000000; */ /* benchmark */

	printk("XGIfb: SR14=%x DramSzie %x ChannelNum %x\n",
	       reg,
	       xgi_video_info.video_size, ChannelNum);
	return 0;

}

static void XGIfb_detect_VB(void)
{
	u8 cr32, temp = 0;

	xgi_video_info.TV_plug = xgi_video_info.TV_type = 0;

	switch (xgi_video_info.hasVB) {
	case HASVB_LVDS_CHRONTEL:
	case HASVB_CHRONTEL:
		break;
	case HASVB_301:
	case HASVB_302:
		/* XGI_Sense30x(); */ /* Yi-Lin TV Sense? */
		break;
	}

	cr32 = xgifb_reg_get(XGICR, IND_XGI_SCRATCH_REG_CR32);

	if ((cr32 & XGI_CRT1) && !XGIfb_crt1off)
		XGIfb_crt1off = 0;
	else {
		if (cr32 & 0x5F)
			XGIfb_crt1off = 1;
		else
			XGIfb_crt1off = 0;
	}

	if (XGIfb_crt2type != -1)
		/* TW: Override with option */
		xgi_video_info.disp_state = XGIfb_crt2type;
	else if (cr32 & XGI_VB_TV)
		xgi_video_info.disp_state = DISPTYPE_TV;
	else if (cr32 & XGI_VB_LCD)
		xgi_video_info.disp_state = DISPTYPE_LCD;
	else if (cr32 & XGI_VB_CRT2)
		xgi_video_info.disp_state = DISPTYPE_CRT2;
	else
		xgi_video_info.disp_state = 0;

	if (XGIfb_tvplug != -1)
		/* PR/TW: Override with option */
		xgi_video_info.TV_plug = XGIfb_tvplug;
	else if (cr32 & XGI_VB_HIVISION) {
		xgi_video_info.TV_type = TVMODE_HIVISION;
		xgi_video_info.TV_plug = TVPLUG_SVIDEO;
	} else if (cr32 & XGI_VB_SVIDEO)
		xgi_video_info.TV_plug = TVPLUG_SVIDEO;
	else if (cr32 & XGI_VB_COMPOSITE)
		xgi_video_info.TV_plug = TVPLUG_COMPOSITE;
	else if (cr32 & XGI_VB_SCART)
		xgi_video_info.TV_plug = TVPLUG_SCART;

	if (xgi_video_info.TV_type == 0) {
		temp = xgifb_reg_get(XGICR, 0x38);
		if (temp & 0x10)
			xgi_video_info.TV_type = TVMODE_PAL;
		else
			xgi_video_info.TV_type = TVMODE_NTSC;
	}

	/* TW: Copy forceCRT1 option to CRT1off if option is given */
	if (XGIfb_forcecrt1 != -1) {
		if (XGIfb_forcecrt1)
			XGIfb_crt1off = 0;
		else
			XGIfb_crt1off = 1;
	}
}

static int XGIfb_has_VB(void)
{
	u8 vb_chipid;

	vb_chipid = xgifb_reg_get(XGIPART4, 0x00);
	switch (vb_chipid) {
	case 0x01:
		xgi_video_info.hasVB = HASVB_301;
		break;
	case 0x02:
		xgi_video_info.hasVB = HASVB_302;
		break;
	default:
		xgi_video_info.hasVB = HASVB_NONE;
		return 0;
	}
	return 1;
}

static void XGIfb_get_VB_type(void)
{
	u8 reg;

	if (!XGIfb_has_VB()) {
		reg = xgifb_reg_get(XGICR, IND_XGI_SCRATCH_REG_CR37);
		switch ((reg & XGI_EXTERNAL_CHIP_MASK) >> 1) {
		case XGI310_EXTERNAL_CHIP_LVDS:
			xgi_video_info.hasVB = HASVB_LVDS;
			break;
		case XGI310_EXTERNAL_CHIP_LVDS_CHRONTEL:
			xgi_video_info.hasVB = HASVB_LVDS_CHRONTEL;
			break;
		default:
			break;
		}
	}
}

XGIINITSTATIC int __init XGIfb_setup(char *options)
{
	char *this_opt;

	xgi_video_info.refresh_rate = 0;

	printk(KERN_INFO "XGIfb: Options %s\n", options);

	if (!options || !*options)
		return 0;

	while ((this_opt = strsep(&options, ",")) != NULL) {

		if (!*this_opt)
			continue;

		if (!strncmp(this_opt, "mode:", 5)) {
			XGIfb_search_mode(this_opt + 5);
		} else if (!strncmp(this_opt, "vesa:", 5)) {
			XGIfb_search_vesamode(simple_strtoul(
						this_opt + 5, NULL, 0));
		} else if (!strncmp(this_opt, "mode:", 5)) {
			XGIfb_search_mode(this_opt + 5);
		} else if (!strncmp(this_opt, "vesa:", 5)) {
			XGIfb_search_vesamode(simple_strtoul(
						this_opt + 5, NULL, 0));
		} else if (!strncmp(this_opt, "vrate:", 6)) {
			xgi_video_info.refresh_rate = simple_strtoul(
						this_opt + 6, NULL, 0);
		} else if (!strncmp(this_opt, "rate:", 5)) {
			xgi_video_info.refresh_rate = simple_strtoul(
						this_opt + 5, NULL, 0);
		} else if (!strncmp(this_opt, "off", 3)) {
			XGIfb_off = 1;
		} else if (!strncmp(this_opt, "crt1off", 7)) {
			XGIfb_crt1off = 1;
		} else if (!strncmp(this_opt, "filter:", 7)) {
			filter = (int)simple_strtoul(this_opt + 7, NULL, 0);
		} else if (!strncmp(this_opt, "forcecrt2type:", 14)) {
			XGIfb_search_crt2type(this_opt + 14);
		} else if (!strncmp(this_opt, "forcecrt1:", 10)) {
			XGIfb_forcecrt1 = (int)simple_strtoul(
						this_opt + 10, NULL, 0);
		} else if (!strncmp(this_opt, "tvmode:", 7)) {
			XGIfb_search_tvstd(this_opt + 7);
		} else if (!strncmp(this_opt, "tvstandard:", 11)) {
			XGIfb_search_tvstd(this_opt + 7);
		} else if (!strncmp(this_opt, "dstn", 4)) {
			enable_dstn = 1;
			/* TW: DSTN overrules forcecrt2type */
			XGIfb_crt2type = DISPTYPE_LCD;
		} else if (!strncmp(this_opt, "pdc:", 4)) {
			XGIfb_pdc = simple_strtoul(this_opt + 4, NULL, 0);
			if (XGIfb_pdc & ~0x3c) {
				printk(KERN_INFO "XGIfb: Illegal pdc parameter\n");
				XGIfb_pdc = 0;
			}
		} else if (!strncmp(this_opt, "noypan", 6)) {
			XGIfb_ypan = 0;
		} else if (!strncmp(this_opt, "userom:", 7)) {
			XGIfb_userom = (int)simple_strtoul(
						this_opt + 7, NULL, 0);
			/* } else if (!strncmp(this_opt, "useoem:", 7)) { */
			/* XGIfb_useoem = (int)simple_strtoul(
						this_opt + 7, NULL, 0); */
		} else {
			XGIfb_search_mode(this_opt);
			/* printk(KERN_INFO "XGIfb: Invalid option %s\n",
				  this_opt); */
		}

		/* TW: Panning only with acceleration */
		XGIfb_ypan = 0;

	}
	printk("\nxgifb: outa xgifb_setup 3450");
	return 0;
}

static unsigned char *xgifb_copy_rom(struct pci_dev *dev)
{
	void __iomem *rom_address;
	unsigned char *rom_copy;
	size_t rom_size;

	rom_address = pci_map_rom(dev, &rom_size);
	if (rom_address == NULL)
		return NULL;

	rom_copy = vzalloc(XGIFB_ROM_SIZE);
	if (rom_copy == NULL)
		goto done;

	rom_size = min_t(size_t, rom_size, XGIFB_ROM_SIZE);
	memcpy_fromio(rom_copy, rom_address, rom_size);

done:
	pci_unmap_rom(dev, rom_address);
	return rom_copy;
}

static int __devinit xgifb_probe(struct pci_dev *pdev,
		const struct pci_device_id *ent)
{
	u8 reg, reg1;
	u8 CR48, CR38;
	int ret;

	if (XGIfb_off)
		return -ENXIO;

	XGIfb_registered = 0;

	memset(&XGIhw_ext, 0, sizeof(struct xgi_hw_device_info));
	fb_info = framebuffer_alloc(sizeof(struct fb_info), &pdev->dev);
	if (!fb_info)
		return -ENOMEM;

	xgi_video_info.chip_id = pdev->device;
	pci_read_config_byte(pdev,
			     PCI_REVISION_ID,
			     &xgi_video_info.revision_id);
	XGIhw_ext.jChipRevision = xgi_video_info.revision_id;

	xgi_video_info.pcibus = pdev->bus->number;
	xgi_video_info.pcislot = PCI_SLOT(pdev->devfn);
	xgi_video_info.pcifunc = PCI_FUNC(pdev->devfn);
	xgi_video_info.subsysvendor = pdev->subsystem_vendor;
	xgi_video_info.subsysdevice = pdev->subsystem_device;

	xgi_video_info.video_base = pci_resource_start(pdev, 0);
	xgi_video_info.mmio_base = pci_resource_start(pdev, 1);
	xgi_video_info.mmio_size = pci_resource_len(pdev, 1);
	xgi_video_info.vga_base = pci_resource_start(pdev, 2) + 0x30;
	XGIhw_ext.pjIOAddress = (unsigned char *)xgi_video_info.vga_base;
	/* XGI_Pr.RelIO  = ioremap(pci_resource_start(pdev, 2), 128) + 0x30; */
	printk("XGIfb: Relocate IO address: %lx [%08lx]\n",
	       (unsigned long)pci_resource_start(pdev, 2), XGI_Pr.RelIO);

	if (pci_enable_device(pdev)) {
		ret = -EIO;
		goto error;
	}

	XGIRegInit(&XGI_Pr, (unsigned long)XGIhw_ext.pjIOAddress);

	xgifb_reg_set(XGISR, IND_XGI_PASSWORD, XGI_PASSWORD);
	reg1 = xgifb_reg_get(XGISR, IND_XGI_PASSWORD);

	if (reg1 != 0xa1) { /*I/O error */
		printk("\nXGIfb: I/O error!!!");
		ret = -EIO;
		goto error;
	}

	switch (xgi_video_info.chip_id) {
	case PCI_DEVICE_ID_XG_20:
		xgifb_reg_or(XGICR, Index_CR_GPIO_Reg3, GPIOG_EN);
		CR48 = xgifb_reg_get(XGICR, Index_CR_GPIO_Reg1);
		if (CR48&GPIOG_READ)
			xgi_video_info.chip = XG21;
		else
			xgi_video_info.chip = XG20;
		XGIfb_CRT2_write_enable = IND_XGI_CRT2_WRITE_ENABLE_315;
		break;
	case PCI_DEVICE_ID_XG_40:
		xgi_video_info.chip = XG40;
		XGIfb_CRT2_write_enable = IND_XGI_CRT2_WRITE_ENABLE_315;
		break;
	case PCI_DEVICE_ID_XG_41:
		xgi_video_info.chip = XG41;
		XGIfb_CRT2_write_enable = IND_XGI_CRT2_WRITE_ENABLE_315;
		break;
	case PCI_DEVICE_ID_XG_42:
		xgi_video_info.chip = XG42;
		XGIfb_CRT2_write_enable = IND_XGI_CRT2_WRITE_ENABLE_315;
		break;
	case PCI_DEVICE_ID_XG_27:
		xgi_video_info.chip = XG27;
		XGIfb_CRT2_write_enable = IND_XGI_CRT2_WRITE_ENABLE_315;
		break;
	default:
		ret = -ENODEV;
		goto error;
	}

	printk("XGIfb:chipid = %x\n", xgi_video_info.chip);
	XGIhw_ext.jChipType = xgi_video_info.chip;

	if ((xgi_video_info.chip == XG21) || (XGIfb_userom)) {
		XGIhw_ext.pjVirtualRomBase = xgifb_copy_rom(pdev);
		if (XGIhw_ext.pjVirtualRomBase)
			printk(KERN_INFO "XGIfb: Video ROM found and mapped to %p\n",
			       XGIhw_ext.pjVirtualRomBase);
		else
			printk(KERN_INFO "XGIfb: Video ROM not found\n");
	} else {
		XGIhw_ext.pjVirtualRomBase = NULL;
		printk(KERN_INFO "XGIfb: Video ROM usage disabled\n");
	}
	XGIhw_ext.pQueryVGAConfigSpace = &XGIfb_query_VGA_config_space;

	if (XGIfb_get_dram_size()) {
		printk(KERN_INFO "XGIfb: Fatal error: Unable to determine RAM size.\n");
		ret = -ENODEV;
		goto error;
	}

	if ((xgifb_mode_idx < 0) ||
	    ((XGIbios_mode[xgifb_mode_idx].mode_no) != 0xFF)) {
		/* Enable PCI_LINEAR_ADDRESSING and MMIO_ENABLE  */
		xgifb_reg_or(XGISR,
			     IND_XGI_PCI_ADDRESS_SET,
			     (XGI_PCI_ADDR_ENABLE | XGI_MEM_MAP_IO_ENABLE));
		/* Enable 2D accelerator engine */
		xgifb_reg_or(XGISR, IND_XGI_MODULE_ENABLE, XGI_ENABLE_2D);
	}

	XGIhw_ext.ulVideoMemorySize = xgi_video_info.video_size;

	if (!request_mem_region(xgi_video_info.video_base,
				xgi_video_info.video_size,
				"XGIfb FB")) {
		printk("unable request memory size %x",
		       xgi_video_info.video_size);
		printk(KERN_ERR "XGIfb: Fatal error: Unable to reserve frame buffer memory\n");
		printk(KERN_ERR "XGIfb: Is there another framebuffer driver active?\n");
		ret = -ENODEV;
		goto error;
	}

	if (!request_mem_region(xgi_video_info.mmio_base,
				xgi_video_info.mmio_size,
				"XGIfb MMIO")) {
		printk(KERN_ERR "XGIfb: Fatal error: Unable to reserve MMIO region\n");
		ret = -ENODEV;
		goto error_0;
	}

	xgi_video_info.video_vbase = XGIhw_ext.pjVideoMemoryAddress =
	ioremap(xgi_video_info.video_base, xgi_video_info.video_size);
	xgi_video_info.mmio_vbase = ioremap(xgi_video_info.mmio_base,
					    xgi_video_info.mmio_size);

	printk(KERN_INFO "XGIfb: Framebuffer at 0x%lx, mapped to 0x%p, size %dk\n",
	       xgi_video_info.video_base,
	       xgi_video_info.video_vbase,
	       xgi_video_info.video_size / 1024);

	printk(KERN_INFO "XGIfb: MMIO at 0x%lx, mapped to 0x%p, size %ldk\n",
	       xgi_video_info.mmio_base, xgi_video_info.mmio_vbase,
	       xgi_video_info.mmio_size / 1024);
	printk("XGIfb: XGIInitNew() ...");
	if (XGIInitNew(&XGIhw_ext))
		printk("OK\n");
	else
		printk("Fail\n");

	xgi_video_info.mtrr = (unsigned int) 0;

	if ((xgifb_mode_idx < 0) ||
	    ((XGIbios_mode[xgifb_mode_idx].mode_no) != 0xFF)) {
		xgi_video_info.hasVB = HASVB_NONE;
		if ((xgi_video_info.chip == XG20) ||
		    (xgi_video_info.chip == XG27)) {
			xgi_video_info.hasVB = HASVB_NONE;
		} else if (xgi_video_info.chip == XG21) {
			CR38 = xgifb_reg_get(XGICR, 0x38);
			if ((CR38&0xE0) == 0xC0) {
				xgi_video_info.disp_state = DISPTYPE_LCD;
				if (!XGIfb_GetXG21LVDSData()) {
					int m;
					for (m = 0; m < sizeof(XGI21_LCDCapList)/sizeof(struct XGI21_LVDSCapStruct); m++) {
						if ((XGI21_LCDCapList[m].LVDSHDE == XGIbios_mode[xgifb_mode_idx].xres) &&
						    (XGI21_LCDCapList[m].LVDSVDE == XGIbios_mode[xgifb_mode_idx].yres)) {
							xgifb_reg_set(XGI_Pr.P3d4, 0x36, m);
						}
					}
				}
			} else if ((CR38&0xE0) == 0x60) {
				xgi_video_info.hasVB = HASVB_CHRONTEL;
			} else {
				xgi_video_info.hasVB = HASVB_NONE;
			}
		} else {
			XGIfb_get_VB_type();
		}

		XGIhw_ext.ujVBChipID = VB_CHIP_UNKNOWN;

		XGIhw_ext.ulExternalChip = 0;

		switch (xgi_video_info.hasVB) {
		case HASVB_301:
			reg = xgifb_reg_get(XGIPART4, 0x01);
			if (reg >= 0xE0) {
				XGIhw_ext.ujVBChipID = VB_CHIP_302LV;
				printk(KERN_INFO "XGIfb: XGI302LV bridge detected (revision 0x%02x)\n", reg);
			} else if (reg >= 0xD0) {
				XGIhw_ext.ujVBChipID = VB_CHIP_301LV;
				printk(KERN_INFO "XGIfb: XGI301LV bridge detected (revision 0x%02x)\n", reg);
			}
			/* else if (reg >= 0xB0) {
				XGIhw_ext.ujVBChipID = VB_CHIP_301B;
				reg1 = xgifb_reg_get(XGIPART4, 0x23);
				printk("XGIfb: XGI301B bridge detected\n");
			} */
			else {
				XGIhw_ext.ujVBChipID = VB_CHIP_301;
				printk("XGIfb: XGI301 bridge detected\n");
			}
			break;
		case HASVB_302:
			reg = xgifb_reg_get(XGIPART4, 0x01);
			if (reg >= 0xE0) {
				XGIhw_ext.ujVBChipID = VB_CHIP_302LV;
				printk(KERN_INFO "XGIfb: XGI302LV bridge detected (revision 0x%02x)\n", reg);
			} else if (reg >= 0xD0) {
				XGIhw_ext.ujVBChipID = VB_CHIP_301LV;
				printk(KERN_INFO "XGIfb: XGI302LV bridge detected (revision 0x%02x)\n", reg);
			} else if (reg >= 0xB0) {
				reg1 = xgifb_reg_get(XGIPART4, 0x23);

				XGIhw_ext.ujVBChipID = VB_CHIP_302B;

			} else {
				XGIhw_ext.ujVBChipID = VB_CHIP_302;
				printk(KERN_INFO "XGIfb: XGI302 bridge detected\n");
			}
			break;
		case HASVB_LVDS:
			XGIhw_ext.ulExternalChip = 0x1;
			printk(KERN_INFO "XGIfb: LVDS transmitter detected\n");
			break;
		case HASVB_TRUMPION:
			XGIhw_ext.ulExternalChip = 0x2;
			printk(KERN_INFO "XGIfb: Trumpion Zurac LVDS scaler detected\n");
			break;
		case HASVB_CHRONTEL:
			XGIhw_ext.ulExternalChip = 0x4;
			printk(KERN_INFO "XGIfb: Chrontel TV encoder detected\n");
			break;
		case HASVB_LVDS_CHRONTEL:
			XGIhw_ext.ulExternalChip = 0x5;
			printk(KERN_INFO "XGIfb: LVDS transmitter and Chrontel TV encoder detected\n");
			break;
		default:
			printk(KERN_INFO "XGIfb: No or unknown bridge type detected\n");
			break;
		}

		if (xgi_video_info.hasVB != HASVB_NONE)
			XGIfb_detect_VB();

		if (xgi_video_info.disp_state & DISPTYPE_DISP2) {
			if (XGIfb_crt1off)
				xgi_video_info.disp_state |= DISPMODE_SINGLE;
			else
				xgi_video_info.disp_state |= (DISPMODE_MIRROR |
							      DISPTYPE_CRT1);
		} else {
			xgi_video_info.disp_state = DISPMODE_SINGLE |
						    DISPTYPE_CRT1;
		}

		if (xgi_video_info.disp_state & DISPTYPE_LCD) {
			if (!enable_dstn) {
				reg = xgifb_reg_get(XGICR, IND_XGI_LCD_PANEL);
				reg &= 0x0f;
				XGIhw_ext.ulCRT2LCDType = XGI310paneltype[reg];

			} else {
				/* TW: FSTN/DSTN */
				XGIhw_ext.ulCRT2LCDType = LCD_320x480;
			}
		}

		XGIfb_detectedpdc = 0;

		XGIfb_detectedlcda = 0xff;

		/* TW: Try to find about LCDA */

		if ((XGIhw_ext.ujVBChipID == VB_CHIP_302B) ||
				(XGIhw_ext.ujVBChipID == VB_CHIP_301LV) ||
				(XGIhw_ext.ujVBChipID == VB_CHIP_302LV)) {
			int tmp;
			tmp = xgifb_reg_get(XGICR, 0x34);
			if (tmp <= 0x13) {
				/* Currently on LCDA?
				 *(Some BIOSes leave CR38) */
				tmp = xgifb_reg_get(XGICR, 0x38);
				if ((tmp & 0x03) == 0x03) {
					/* XGI_Pr.XGI_UseLCDA = 1; */
				} else {
					/* Currently on LCDA?
					 *(Some newer BIOSes set D0 in CR35) */
					tmp = xgifb_reg_get(XGICR, 0x35);
					if (tmp & 0x01) {
						/* XGI_Pr.XGI_UseLCDA = 1; */
					} else {
						tmp = xgifb_reg_get(XGICR,
								    0x30);
						if (tmp & 0x20) {
							tmp = xgifb_reg_get(
								XGIPART1, 0x13);
							if (tmp & 0x04) {
								/* XGI_Pr.XGI_UseLCDA = 1; */
							}
						}
					}
				}
			}

		}

		if (xgifb_mode_idx >= 0)
			xgifb_mode_idx = XGIfb_validate_mode(xgifb_mode_idx);

		if (xgifb_mode_idx < 0) {
			switch (xgi_video_info.disp_state & DISPTYPE_DISP2) {
			case DISPTYPE_LCD:
				xgifb_mode_idx = DEFAULT_LCDMODE;
				if (xgi_video_info.chip == XG21)
					xgifb_mode_idx =
					    XGIfb_GetXG21DefaultLVDSModeIdx();
				break;
			case DISPTYPE_TV:
				xgifb_mode_idx = DEFAULT_TVMODE;
				break;
			default:
				xgifb_mode_idx = DEFAULT_MODE;
				break;
			}
		}

		XGIfb_mode_no = XGIbios_mode[xgifb_mode_idx].mode_no;

		/* yilin set default refresh rate */
		if (xgi_video_info.refresh_rate == 0)
			xgi_video_info.refresh_rate = 60;
		if (XGIfb_search_refresh_rate(
				xgi_video_info.refresh_rate) == 0) {
			XGIfb_rate_idx = XGIbios_mode[xgifb_mode_idx].rate_idx;
			xgi_video_info.refresh_rate = 60;
		}

		xgi_video_info.video_bpp = XGIbios_mode[xgifb_mode_idx].bpp;
		xgi_video_info.video_vwidth =
			xgi_video_info.video_width =
				XGIbios_mode[xgifb_mode_idx].xres;
		xgi_video_info.video_vheight =
			xgi_video_info.video_height =
				XGIbios_mode[xgifb_mode_idx].yres;
		xgi_video_info.org_x = xgi_video_info.org_y = 0;
		xgi_video_info.video_linelength =
			xgi_video_info.video_width *
			(xgi_video_info.video_bpp >> 3);
		switch (xgi_video_info.video_bpp) {
		case 8:
			xgi_video_info.DstColor = 0x0000;
			xgi_video_info.XGI310_AccelDepth = 0x00000000;
			xgi_video_info.video_cmap_len = 256;
			break;
		case 16:
			xgi_video_info.DstColor = 0x8000;
			xgi_video_info.XGI310_AccelDepth = 0x00010000;
			xgi_video_info.video_cmap_len = 16;
			break;
		case 32:
			xgi_video_info.DstColor = 0xC000;
			xgi_video_info.XGI310_AccelDepth = 0x00020000;
			xgi_video_info.video_cmap_len = 16;
			break;
		default:
			xgi_video_info.video_cmap_len = 16;
			printk(KERN_INFO "XGIfb: Unsupported depth %d",
			       xgi_video_info.video_bpp);
			break;
		}

		printk(KERN_INFO "XGIfb: Default mode is %dx%dx%d (%dHz)\n",
		       xgi_video_info.video_width,
		       xgi_video_info.video_height,
		       xgi_video_info.video_bpp,
		       xgi_video_info.refresh_rate);

		default_var.xres =
			default_var.xres_virtual =
				xgi_video_info.video_width;
		default_var.yres =
			default_var.yres_virtual =
				xgi_video_info.video_height;
		default_var.bits_per_pixel = xgi_video_info.video_bpp;

		XGIfb_bpp_to_var(&default_var);

		default_var.pixclock = (u32) (1000000000 /
				XGIfb_mode_rate_to_dclock(&XGI_Pr, &XGIhw_ext,
						XGIfb_mode_no, XGIfb_rate_idx));

		if (XGIfb_mode_rate_to_ddata(&XGI_Pr, &XGIhw_ext,
			XGIfb_mode_no, XGIfb_rate_idx,
			&default_var.left_margin, &default_var.right_margin,
			&default_var.upper_margin, &default_var.lower_margin,
			&default_var.hsync_len, &default_var.vsync_len,
			&default_var.sync, &default_var.vmode)) {

			if ((default_var.vmode & FB_VMODE_MASK) ==
			    FB_VMODE_INTERLACED) {
				default_var.yres <<= 1;
				default_var.yres_virtual <<= 1;
			} else if ((default_var.vmode & FB_VMODE_MASK) ==
				   FB_VMODE_DOUBLE) {
				default_var.pixclock >>= 1;
				default_var.yres >>= 1;
				default_var.yres_virtual >>= 1;
			}

		}

		fb_info->flags = FBINFO_FLAG_DEFAULT;
		fb_info->var = default_var;
		fb_info->fix = XGIfb_fix;
		fb_info->par = &xgi_video_info;
		fb_info->screen_base = xgi_video_info.video_vbase;
		fb_info->fbops = &XGIfb_ops;
		XGIfb_get_fix(&fb_info->fix, -1, fb_info);
		fb_info->pseudo_palette = pseudo_palette;

		fb_alloc_cmap(&fb_info->cmap, 256 , 0);

#ifdef CONFIG_MTRR
		xgi_video_info.mtrr = mtrr_add(
			(unsigned int) xgi_video_info.video_base,
			(unsigned int) xgi_video_info.video_size,
			MTRR_TYPE_WRCOMB, 1);
		if (xgi_video_info.mtrr)
			printk(KERN_INFO "XGIfb: Added MTRRs\n");
#endif

		if (register_framebuffer(fb_info) < 0) {
			ret = -EINVAL;
			goto error_1;
		}

		XGIfb_registered = 1;

		printk(KERN_INFO "fb%d: %s frame buffer device, Version %d.%d.%02d\n",
		       fb_info->node, myid, VER_MAJOR, VER_MINOR, VER_LEVEL);

	}

	dumpVGAReg();

	return 0;

error_1:
	iounmap(xgi_video_info.mmio_vbase);
	iounmap(xgi_video_info.video_vbase);
	release_mem_region(xgi_video_info.mmio_base, xgi_video_info.mmio_size);
error_0:
	release_mem_region(xgi_video_info.video_base,
			   xgi_video_info.video_size);
error:
	vfree(XGIhw_ext.pjVirtualRomBase);
	framebuffer_release(fb_info);
	return ret;
}

/*****************************************************/
/*                PCI DEVICE HANDLING                */
/*****************************************************/

static void __devexit xgifb_remove(struct pci_dev *pdev)
{
	unregister_framebuffer(fb_info);
	iounmap(xgi_video_info.mmio_vbase);
	iounmap(xgi_video_info.video_vbase);
	release_mem_region(xgi_video_info.mmio_base, xgi_video_info.mmio_size);
	release_mem_region(xgi_video_info.video_base,
			   xgi_video_info.video_size);
	vfree(XGIhw_ext.pjVirtualRomBase);
	framebuffer_release(fb_info);
	pci_set_drvdata(pdev, NULL);
}

static struct pci_driver xgifb_driver = {
	.name = "xgifb",
	.id_table = xgifb_pci_table,
	.probe = xgifb_probe,
	.remove = __devexit_p(xgifb_remove)
};

XGIINITSTATIC int __init xgifb_init(void)
{
	char *option = NULL;

	if (fb_get_options("xgifb", &option))
		return -ENODEV;
	XGIfb_setup(option);

	return pci_register_driver(&xgifb_driver);
}

#ifndef MODULE
module_init(xgifb_init);
#endif

/*****************************************************/
/*                      MODULE                       */
/*****************************************************/

#ifdef MODULE

static char *mode = NULL;
static int vesa = 0;
static unsigned int rate = 0;
static unsigned int mem = 0;
static char *forcecrt2type = NULL;
static int forcecrt1 = -1;
static int pdc = -1;
static int pdc1 = -1;
static int noypan = -1;
static int userom = -1;
static int useoem = -1;
static char *tvstandard = NULL;
static int nocrt2rate = 0;
static int scalelcd = -1;
static char *specialtiming = NULL;
static int lvdshl = -1;
static int tvxposoffset = 0, tvyposoffset = 0;
#if !defined(__i386__) && !defined(__x86_64__)
static int resetcard = 0;
static int videoram = 0;
#endif

MODULE_DESCRIPTION("Z7 Z9 Z9S Z11 framebuffer device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("XGITECH , Others");

module_param(mem, int, 0);
module_param(noypan, int, 0);
module_param(userom, int, 0);
module_param(useoem, int, 0);
module_param(mode, charp, 0);
module_param(vesa, int, 0);
module_param(rate, int, 0);
module_param(forcecrt1, int, 0);
module_param(forcecrt2type, charp, 0);
module_param(scalelcd, int, 0);
module_param(pdc, int, 0);
module_param(pdc1, int, 0);
module_param(specialtiming, charp, 0);
module_param(lvdshl, int, 0);
module_param(tvstandard, charp, 0);
module_param(tvxposoffset, int, 0);
module_param(tvyposoffset, int, 0);
module_param(filter, int, 0);
module_param(nocrt2rate, int, 0);
#if !defined(__i386__) && !defined(__x86_64__)
module_param(resetcard, int, 0);
module_param(videoram, int, 0);
#endif

MODULE_PARM_DESC(noypan,
		"\nIf set to anything other than 0, y-panning will be disabled and scrolling\n"
		"will be performed by redrawing the screen. (default: 0)\n");

MODULE_PARM_DESC(mode,
		"\nSelects the desired default display mode in the format XxYxDepth,\n"
		"eg. 1024x768x16. Other formats supported include XxY-Depth and\n"
		"XxY-Depth@Rate. If the parameter is only one (decimal or hexadecimal)\n"
		"number, it will be interpreted as a VESA mode number. (default: 800x600x8)\n");

MODULE_PARM_DESC(vesa,
		"\nSelects the desired default display mode by VESA defined mode number, eg.\n"
		"0x117 (default: 0x0103)\n");

MODULE_PARM_DESC(rate,
		"\nSelects the desired vertical refresh rate for CRT1 (external VGA) in Hz.\n"
		"If the mode is specified in the format XxY-Depth@Rate, this parameter\n"
		"will be ignored (default: 60)\n");

MODULE_PARM_DESC(forcecrt1,
		"\nNormally, the driver autodetects whether or not CRT1 (external VGA) is\n"
		"connected. With this option, the detection can be overridden (1=CRT1 ON,\n"
		"0=CRT1 OFF) (default: [autodetected])\n");

MODULE_PARM_DESC(forcecrt2type,
		"\nIf this option is omitted, the driver autodetects CRT2 output devices, such as\n"
		"LCD, TV or secondary VGA. With this option, this autodetection can be\n"
		"overridden. Possible parameters are LCD, TV, VGA or NONE. NONE disables CRT2.\n"
		"On systems with a SiS video bridge, parameters SVIDEO, COMPOSITE or SCART can\n"
		"be used instead of TV to override the TV detection. Furthermore, on systems\n"
		"with a SiS video bridge, SVIDEO+COMPOSITE, HIVISION, YPBPR480I, YPBPR480P,\n"
		"YPBPR720P and YPBPR1080I are understood. However, whether or not these work\n"
		"depends on the very hardware in use. (default: [autodetected])\n");

MODULE_PARM_DESC(scalelcd,
		"\nSetting this to 1 will force the driver to scale the LCD image to the panel's\n"
		"native resolution. Setting it to 0 will disable scaling; LVDS panels will\n"
		"show black bars around the image, TMDS panels will probably do the scaling\n"
		"themselves. Default: 1 on LVDS panels, 0 on TMDS panels\n");

MODULE_PARM_DESC(pdc,
		"\nThis is for manually selecting the LCD panel delay compensation. The driver\n"
		"should detect this correctly in most cases; however, sometimes this is not\n"
		"possible. If you see 'small waves' on the LCD, try setting this to 4, 32 or 24\n"
		"on a 300 series chipset; 6 on a 315 series chipset. If the problem persists,\n"
		"try other values (on 300 series: between 4 and 60 in steps of 4; on 315 series:\n"
		"any value from 0 to 31). (default: autodetected, if LCD is active during start)\n");

MODULE_PARM_DESC(pdc1,
		"\nThis is same as pdc, but for LCD-via CRT1. Hence, this is for the 315/330\n"
		"series only. (default: autodetected if LCD is in LCD-via-CRT1 mode during\n"
		"startup) - Note: currently, this has no effect because LCD-via-CRT1 is not\n"
		"implemented yet.\n");

MODULE_PARM_DESC(specialtiming,
		"\nPlease refer to documentation for more information on this option.\n");

MODULE_PARM_DESC(lvdshl,
		"\nPlease refer to documentation for more information on this option.\n");

MODULE_PARM_DESC(tvstandard,
		"\nThis allows overriding the BIOS default for the TV standard. Valid choices are\n"
		"pal, ntsc, palm and paln. (default: [auto; pal or ntsc only])\n");

MODULE_PARM_DESC(tvxposoffset,
		"\nRelocate TV output horizontally. Possible parameters: -32 through 32.\n"
		"Default: 0\n");

MODULE_PARM_DESC(tvyposoffset,
		"\nRelocate TV output vertically. Possible parameters: -32 through 32.\n"
		"Default: 0\n");

MODULE_PARM_DESC(filter,
		"\nSelects TV flicker filter type (only for systems with a SiS301 video bridge).\n"
		"(Possible values 0-7, default: [no filter])\n");

MODULE_PARM_DESC(nocrt2rate,
		"\nSetting this to 1 will force the driver to use the default refresh rate for\n"
		"CRT2 if CRT2 type is VGA. (default: 0, use same rate as CRT1)\n");

static int __init xgifb_init_module(void)
{
	printk("\nXGIfb_init_module");
	if (mode)
		XGIfb_search_mode(mode);
	else if (vesa != -1)
		XGIfb_search_vesamode(vesa);

	return xgifb_init();
}

static void __exit xgifb_remove_module(void)
{
	pci_unregister_driver(&xgifb_driver);
	printk(KERN_DEBUG "xgifb: Module unloaded\n");
}

module_init(xgifb_init_module);
module_exit(xgifb_remove_module);

#endif	/*  /MODULE  */
