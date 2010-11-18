/* drivers/input/touchscreen/ntrig.c - NTRIG_G41 Touch driver
 *
 * Copyright (C) 2010 HTC Corporation.
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include <linux/ntrig_g41.h>
#include <linux/ntrig_g41_fw.h>
#include <linux/spi/spi.h>
#include <linux/stat.h>


#define SWAP16(x) \
    ((u16)( \
    (((u16)(x) & (u16) 0x00ffU) << 8) | \
    (((u16)(x) & (u16) 0xff00U) >> 8)))

int CNT_DFU = 1;

struct ntg_tsData {
	struct input_dev 	*input_dev;
	struct spi_device 	*spiDev;
	struct workqueue_struct *ntg_wq;
	struct work_struct 		ntg_work;
	struct ntg_touch_event *touchEvnt;
	uint16_t  abs_x_min;
	uint16_t  abs_x_max;
	uint16_t  abs_y_min;
	uint16_t  abs_y_max;
	uint8_t abs_pressure_min;
	uint8_t abs_pressure_max;
	uint8_t abs_width_min;
	uint8_t abs_width_max;
};
static struct ntg_tsData *private_ts;

static char *spiTxbuf, *spiRxbuf;

static unsigned char CalcChecksum(unsigned char *PacketData, u16 PackLen)
{
	int i;
	unsigned char  sum = 0, ret;

	for (i = 0 ; i < PackLen-1; i++)
		sum += PacketData[i];
	ret = 0-sum;
	return ret;
}

static ssize_t spiDuplex_Lock(struct spi_device *dev, char *txbuf,
							char *rxbuf, size_t len)
{
	struct spi_msg msg;
	int err = 0;
	if (!dev)
		return 0;
	memset(msg.buffer, 0, sizeof(unsigned char)*128);
	memcpy(msg.buffer, txbuf, 128);
	err = spi_read_write_lock(dev, &msg, rxbuf,  len, 2);

	if (len > 128) {
		char extrabuf[8] = {0};
		memcpy(msg.buffer, (txbuf+128), sizeof(char)*8);
		err = spi_read_write_lock(dev, &msg, (char *)&extrabuf, (len-128), 2);
		memcpy((rxbuf+128), (char *)&extrabuf, sizeof(char)*8);
	}
	return err;
}
/*
#if 0
static int frame_parser(char *buf, struct ntg_touch_event *touEvt)
{
	int i = 0, nIndex = 0;
	struct ntg_tsData *ts = private_ts;
	while (1) {
		if (buf[i] != 0xFF) {
			if (buf[i] == 0xA5 && buf[i+1] == 0x5A &&
				buf[i+2] == 0xE7 && buf[i+3] == 0x7E) {
				printk(KERN_INFO "[ts]Preamble OK!\n");
				 nIndex = i+4;
				if ((NTRIG_PACKET_SIZE-nIndex) <= 3) {
					printk(KERN_INFO "\n");
					spiDuplex_Lock(ts->spiDev,
						spiTxbuf+NTRIG_PACKET_SIZE,
						spiRxbuf+NTRIG_PACKET_SIZE, 128);
					break;
				} else {
					printk(KERN_INFO "\n");
					if (buf[nIndex+1] >
						(NTRIG_PACKET_SIZE-nIndex)) {
						spiDuplex_Lock (ts->spiDev, spiTxbuf+NTRIG_PACKET_SIZE,
								spiRxbuf+NTRIG_PACKET_SIZE,
								(buf[nIndex+1]-NTRIG_PACKET_SIZE-nIndex+1));
						break;
					}
				}
			} else {
				printk(KERN_INFO "[ts-np]%x%x%x%x",
								buf[i], buf[i+1], buf[i+2], buf[i+3]);
				return -1;
			}
		} else {
			i++;
		}
	}
	touEvt->header.type = buf[nIndex];
	memcpy(&touEvt->header.length, &buf[nIndex+1], 2);
	touEvt->header.flag = buf[nIndex+3];
	touEvt->header.channel = buf[nIndex+4];
	touEvt->header.function = buf[nIndex+5];
	printk(KERN_INFO "[ts_parser]type:%02x;length:%02x;channel:%02x;function:%02x\n",
			touEvt->header.type, touEvt->header.length,
			touEvt->header.channel, touEvt->header.function);

	if (touEvt->header.type == 0x02 || touEvt->header.type == 0x01) {
		switch (touEvt->header.channel) {
		case 0x01:
			break;
		case 0x02:
			touEvt->stouch.repID = buf[nIndex+6];
			touEvt->stouch.bit = buf[nIndex+7];
			memcpy(&touEvt->stouch.posX, &buf[nIndex + 8], 2);
			memcpy(&touEvt->stouch.posY, &buf[nIndex+10], 2);
			memcpy(&touEvt->stouch.posDx, &buf[nIndex+12], 2);
			memcpy(&touEvt->stouch.posDy, &buf[nIndex+14], 2);
			printk(KERN_INFO "[ts_parser](STouch)pos_x:%d,pos_y:%d,posDx:%d,posDy:%d\n",
					touEvt->stouch.posX, touEvt->stouch.posY,
					touEvt->stouch.posDx, touEvt->stouch.posDy);
			break;
		case 0x03:
			touEvt->ptouch.repID = buf[nIndex+6];
			touEvt->ptouch.bit = buf[nIndex+7];
			memcpy(&touEvt->ptouch.posX, &buf[nIndex + 8], 2);
			memcpy(&touEvt->ptouch.posY, &buf[nIndex + 10], 2);
			memcpy(&touEvt->ptouch.tip, &buf[nIndex + 12], 2);
			memcpy(&touEvt->ptouch.unused, &buf[nIndex + 14], 2);
			printk(KERN_INFO "[ts_parser](Pen)pos_x:%d,pos_y:%d,tip:%d,unused:%d\n",
					touEvt->ptouch.posX, touEvt->ptouch.posY,
					touEvt->ptouch.tip, touEvt->ptouch.unused);
			break;
		case 0x10:
			touEvt->version.repId  = buf[nIndex+6];
			switch (touEvt->version.repId) {
			case 0x0C:
				memcpy(&touEvt->version.value, &buf[nIndex+8], 4);
				touEvt->version.reserved_0 = buf[nIndex+7];
				memcpy(&touEvt->version.value, &buf[nIndex+8], 4);
				memcpy(&touEvt->version.reserved_1, &buf[nIndex+12], 2);
				printk(KERN_INFO "[ts_parser](GetVer)repID:%x,value:%04x\n",
					touEvt->version.repId, touEvt->version.value);
				break;
			case 0x0B:
				memcpy(&touEvt->version.value, &buf[nIndex+7], 3);
				printk(KERN_INFO "[ts_parser](Start-Cal)repID:%x,value:%04x\n",
						touEvt->version.repId, touEvt->version.value);
				break;
			case 0x11:
				memcpy(&touEvt->version.value, &buf[nIndex+7], 3);
				printk(KERN_INFO "[ts_parser](GetCal-ing)repID:%x,value:%04x\n",
						touEvt->version.repId, touEvt->version.value);
				break;
			}
			break;
			}
	}
	return 0;
}
#endif
*/
static int ntrg_frame_parser(char *buf, struct ntg_touch_event *touEvt)
{
	int i = 0, nIndex = 0;

	while (1) {
		if (buf[i] != 0xFF) {
			if (buf[i] == 0xA5 && buf[i+1] == 0x5A &&
				buf[i+2] == 0xE7 && buf[i+3] == 0x7E) {
				 nIndex = i+4;

				 touEvt->header.type = buf[nIndex];
				 memcpy(&touEvt->header.length, &buf[nIndex + 1], 2);
				 touEvt->header.flag = buf[nIndex + 3];
				 touEvt->header.channel = buf[nIndex + 4];
				 touEvt->header.function = buf[nIndex + 5];
				 if (touEvt->header.type == 0x02 || touEvt->header.type == 0x01) {
					switch (touEvt->header.channel) {
					case 0x01:
						/*buf into api
						NtgProcessReport(buf[nIndex+6], 94, touEvt->mtouch, sizeof(ntg_touch_Mtouch));
						*/break;
					case 0x02:
						touEvt->stouch.repID = buf[nIndex+6];
						touEvt->stouch.bit = buf[nIndex+7];
						memcpy(&touEvt->stouch.posX, &buf[nIndex + 8], 2);
						memcpy(&touEvt->stouch.posY, &buf[nIndex + 10], 2);
						memcpy(&touEvt->stouch.posDx, &buf[nIndex + 12], 2);
						memcpy(&touEvt->stouch.posDy, &buf[nIndex + 14], 2);
						/*touEvt->stouch.posX = 7200 -touEvt->stouch.posX;
						printk(KERN_INFO "[ts_parser](STouch)pos_x:%d,pos_y:%d,posDx:%d,posDy:%d\n",
							touEvt->stouch.posX, touEvt->stouch.posY,
							touEvt->stouch.posDx, touEvt->stouch.posDy);
						*/break;
					case 0x03:
						touEvt->ptouch.repID = buf[nIndex + 6];
						touEvt->ptouch.bit = buf[nIndex + 7];
						memcpy(&touEvt->ptouch.posX, &buf[nIndex + 8], 2);
						memcpy(&touEvt->ptouch.posY, &buf[nIndex + 10], 2);
						memcpy(&touEvt->ptouch.tip, &buf[nIndex + 12], 2);
						memcpy(&touEvt->ptouch.unused, &buf[nIndex + 14], 2);
						/*touEvt->ptouch.posX = 7200 -touEvt->ptouch.posX;
						printk(KERN_INFO "[ts_parser](Pen)pos_x:%d,pos_y:%d,tip:%d,unused:%d\n",
							touEvt->ptouch.posX, touEvt->ptouch.posY,
							touEvt->ptouch.tip, touEvt->ptouch.unused);
						*/break;
					case 0x10:
						touEvt->version.repId  = buf[nIndex+6];
						switch (touEvt->version.repId) {
						case 0x0C:
							memcpy(&touEvt->version.value, &buf[nIndex + 8], 4);
							touEvt->version.reserved_0 = buf[nIndex + 7];
							memcpy(&touEvt->version.value, &buf[nIndex + 8], 4);
							memcpy(&touEvt->version.reserved_1, &buf[nIndex + 12], 2);
							printk(KERN_INFO "[ts_parser](GetVer)repID:%x,value:%04x\n", touEvt->version.repId, touEvt->version.value);
							break;
						case 0x0B:
							memcpy(&touEvt->version.value, &buf[nIndex + 7], 3);
							printk(KERN_INFO "[ts_parser](Start-Cal)repID:%x,value:%04x\n",
									touEvt->version.repId, touEvt->version.value);
							break;
						case 0x11:
							memcpy(&touEvt->version.value, &buf[nIndex + 7], 3);
							printk(KERN_INFO "[ts_parser](GetCal-ing)repID:%x,value:%04x\n",
									touEvt->version.repId, touEvt->version.value);
							break;
						}
						break;
					case 0x20:
						switch (touEvt->header.function) {
						case 0x61:
							if ((buf[nIndex+6] == 0x7E && buf[nIndex+11] == 0x81) ||
								(buf[nIndex+6] == 0x7E && buf[nIndex+11] == 0xC1)) {
								touEvt->DfuStCmd.dfuHeader.startByte = buf[nIndex+6];
								touEvt->DfuStCmd.dfuHeader.msgType = buf[nIndex+11];
								touEvt->DfuStCmd.dfuHeader.msgCode = buf[nIndex + 13];

								switch (touEvt->DfuStCmd.dfuHeader.msgCode) {
								case DFU_CMD_MSG_CODE_START:
									printk(KERN_INFO "[ts_Dfu_MSG_CODE_START]\n");
								break;
								case DFU_CMD_MSG_CODE_BLOCK_HEADER:
									printk(KERN_INFO "[ts_Dfu_MSG_BLOCK_HEADER]\n");
								break;
								case DFU_CMD_MSG_CODE_DATA_FRAME:
									CNT_DFU++;
									if (CNT_DFU > 0xbb0 || CNT_DFU < 5)
										printk(KERN_INFO "[ts_Dfu_MSG_DATA_FRAME_%.4x]\n", CNT_DFU);
								break;
								case DFU_CMD_MSG_CODE_DATA_COMPLETE:
									printk(KERN_INFO "[ts_Dfu_MSG_DATA_COMPLETE]\n");
								break;
								case DFU_CMD_MSG_CODE_LOAD_FW:
									printk(KERN_INFO "[ts_Dfu_MSG_LOAD_FW]\n");
								break;
								case DFU_CMD_MSG_CODE_TEST_FW:
									printk(KERN_INFO "[ts_Dfu_MSG_TEST_FW]\n");
								break;
								case DFU_CMD_MSG_CODE_SET_SELF_VALID:
									printk(KERN_INFO "[ts_Dfu_MSG_SET_SELF_VALID]\n");
								break;
								}
								touEvt->DfuStCmd.dfuHeader.returnCode = *((long *)&buf[nIndex+14]);
								if (touEvt->DfuStCmd.dfuHeader.returnCode == 1)
									printk(KERN_INFO "[ts_DFU](Resp) FAILE_MsgCode= %.2x_RTcode=%.4lx\n",
										touEvt->DfuStCmd.dfuHeader.msgCode, touEvt->DfuStCmd.dfuHeader.returnCode);
							} else {
								printk(KERN_INFO "[ts_Dfu] protocol error,%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x\n", buf[nIndex+6], buf[nIndex+7], buf[nIndex+8], buf[nIndex+9],
										buf[nIndex+10], buf[nIndex+11], buf[nIndex+12], buf[nIndex+13]);
							}
							break;
						}
						break;
					}
				}
				break;
			} else {
				return -1;
			}
		} else {
			i++;
		}
	}
	return 0;
}


static ssize_t ntg_GetFw(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	int  ret, err;
	struct ntg_tsData *ts_data;

	ts_data = private_ts;

	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);

	memcpy(spiTxbuf, &ntgFW_getfw, sizeof(ntgFW_getfw));
	memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);
	printk(KERN_INFO "[[ts-GetFw]GetFw test send\n");
	err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
						sizeof(char)*NTRIG_PACKET_SIZE);
	sprintf(buf, "[ts-GetFw]Rx=%.2x%.2x%.2x%.2x\n",
				spiRxbuf[19], spiRxbuf[18], spiRxbuf[17], spiRxbuf[16]);

	mdelay(100);

	ret	= strlen(buf)+1;

	return ret;
}
DEVICE_ATTR(ts_fw, 0444, ntg_GetFw , NULL);

static ssize_t ntg_GoToBl(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret, err;
	struct ntg_tsData *ts_data;
	ts_data = private_ts;
	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);

	memcpy(spiTxbuf, &ntgFW_bl, sizeof(ntgFW_bl));
	memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);
	printk(KERN_INFO "[ts-D_Go2BL]\n");

	err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
						sizeof(char)*NTRIG_PACKET_SIZE);
	ret   = strlen(buf)+1;
	return ret;
}
DEVICE_ATTR(ts_bl, 0444 , ntg_GoToBl, NULL);

static ssize_t ntg_DfuStart(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret, err;
	long startByte = 0;
	struct ntg_tsData *ts_data;
	ts_data = private_ts;
	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);

	memcpy((char *)&startByte, &ntgFW[0], 4);
	memcpy(&ntgFW_start[28], (char *)&startByte, 4);

	printk(KERN_INFO "[ts-D_Start]\n");
	ntgFW_start[33] = CalcChecksum((unsigned char *)&ntgFW_start[14], 20);

/*	int i;
	for (i = 0; i < 4; i++) {
		printk(KERN_INFO "[ts]%.2x%.2x%.2x%.2x\\%.2x%.2x%.2x%.2x\n",
				ntgFW_start[i*8], ntgFW_start[i*8+1], ntgFW_start[i*8+2], ntgFW_start[i*8+3],
				ntgFW_start[i*8+4], ntgFW_start[i*8+5], ntgFW_start[i*8+6], ntgFW_start[i*8+7]);
	}
	printk(KERN_INFO "[ts]%.2x%.2x\n", ntgFW_start[32], ntgFW_start[33]);
*/

	memcpy(spiTxbuf, &ntgFW_start, sizeof(ntgFW_start));
	memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);

	err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
						sizeof(char)*NTRIG_PACKET_SIZE);

	ret  = strlen(buf)+1;

	return ret;
}
DEVICE_ATTR(ts_Start, 0444 , ntg_DfuStart, NULL);

static ssize_t ntg_DfuSendBlock(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret, err;
	struct ntg_tsData *ts_data;
	ts_data = private_ts;
	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);
	printk(KERN_INFO "[ts-dfuSendBlock]\n");

	memcpy(&ntgFW_blk[28], &ntgFW[12], 4);
	memcpy(&ntgFW_blk[32], &ntgFW[16], 4);

	ntgFW_blk[37] =  CalcChecksum((unsigned char *)(&(ntgFW_blk[14])), 24);

	memcpy(spiTxbuf, &ntgFW_blk, sizeof(ntgFW_blk));
	memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);
/*	int i;
	for (i = 0; i < 4; i++) {
		printk(KERN_INFO "[ts]%.2x%.2x%.2x%.2x\\%.2x%.2x%.2x%.2x\n",
				ntgFW_blk[i*8], ntgFW_blk[i*8+1], ntgFW_blk[i*8+2], ntgFW_blk[i*8+3],
				ntgFW_blk[i*8+4], ntgFW_blk[i*8+5], ntgFW_blk[i*8+6], ntgFW_blk[i*8+7]);
	}
	printk(KERN_INFO "[ts]%.2x%.2x%.2x%.2x%.2x%.2x\n", ntgFW_blk[32], ntgFW_blk[33],
				ntgFW_blk[34], ntgFW_blk[35], ntgFW_blk[36], ntgFW_blk[37]);
*/
	err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
						sizeof(char)*NTRIG_PACKET_SIZE);

	ret   = strlen(buf)+1;
	return ret;
}
DEVICE_ATTR(ts_Blk, 0444  , ntg_DfuSendBlock, NULL);

static ssize_t ntg_DfuSendData(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0, err, i, BinFileOffset;
	short curLen, CmdOffset, tmpLen;
	u32 BinFilePackCnt;

	struct ntg_tsData *ts_data;
	ts_data = private_ts;

/*	char Data[136] = {0xFF, 0xFF, 0xFF, 0xFF,
					0xA5, 0x5A, 0xE7, 0x7E,
					0x02, 0xFF, 0x00, 0x00,
					0x02, 0x61, 0x7E, 0x00,
					0x00, 0xFF, 0xFF, 0x01,
					0x07, 0x12, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,};
*/

	BinFilePackCnt = 0;
	tmpLen = 0, curLen = 0, BinFileOffset = 21;

	memcpy((char *)&BinFilePackCnt, &ntgFW[8], 4);

	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);

	for (i = 0; i < BinFilePackCnt ; i++) {
		CmdOffset = 28;
		memcpy((char *)&((ntgFW_Data[CmdOffset])), (char *)(&(ntgFW[BinFileOffset])), 2);
		BinFileOffset += 2;
		CmdOffset += 2;
		memcpy((char *)&((ntgFW_Data[CmdOffset])), (char *)(&(ntgFW[BinFileOffset])), 4);
		BinFileOffset += 4;
		CmdOffset += 4;
		memcpy((char *)&((ntgFW_Data[CmdOffset])), (char *)(&(ntgFW[BinFileOffset])), 2);
		CmdOffset += 2;
		memcpy((char *)(&curLen), (char *)(&(ntgFW[BinFileOffset])), 2);

		tmpLen = curLen+21+6;
		memcpy((char *)&ntgFW_Data[17], &tmpLen, 2);
		SWAP16(curLen);

		BinFileOffset += 2;
		memcpy((char *)&((ntgFW_Data[CmdOffset])), (char *)(&(ntgFW[BinFileOffset])), curLen);
		BinFileOffset += curLen;
		CmdOffset += curLen;

		memcpy((char *)&((ntgFW_Data[CmdOffset])), (char *)(&(ntgFW[BinFileOffset])), 4);
		BinFileOffset += 4;
		CmdOffset += 4;

		ntgFW_Data[CmdOffset] = CalcChecksum((unsigned char *)(&(ntgFW_Data[14])), CmdOffset-14+1);
		CmdOffset++;
		SWAP16(CmdOffset);
		memcpy(&ntgFW_Data[9], (char *)&CmdOffset, 2);
		SWAP16(CmdOffset);
		memcpy(spiTxbuf, &ntgFW_Data, CmdOffset);

		memset(spiRxbuf, 0, NTRIG_PACKET_SIZE*2);
		err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf, NTRIG_PACKET_SIZE);

		mdelay(30);
	}

	ret   = strlen(buf)+1;

	return ret;

}
DEVICE_ATTR(ts_Data, 0444  , ntg_DfuSendData, NULL);

static ssize_t ntg_DataCmp(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret, err;
	long len;
	short slen;

	struct ntg_tsData *ts_data;
	ts_data = private_ts;

	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);
	memcpy((char *)&len, &ntgFW[8], 4);
	slen = (short)len;
	memcpy(&ntgFW_cmp[28], &slen, 2);
	ntgFW_cmp[31] = CalcChecksum((unsigned char *)&ntgFW_cmp[14], 18);
/*	int i;
	for (i = 0; i < 4; i++) {
		printk(KERN_INFO "[ts]%.2x%.2x%.2x%.2x\\%.2x%.2x%.2x%.2x\n",
				ntgFW_cmp[i*8], ntgFW_cmp[i*8+1], ntgFW_cmp[i*8+2], ntgFW_cmp[i*8+3],
				ntgFW_cmp[i*8+4], ntgFW_cmp[i*8+5], ntgFW_cmp[i*8+6], ntgFW_cmp[i*8+7]);
	}
*/
	memcpy(spiTxbuf, &ntgFW_cmp, sizeof(ntgFW_cmp));
	memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);

	printk(KERN_INFO "[ts-DataCmp]DataCmp send\n");
	err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
						sizeof(char)*NTRIG_PACKET_SIZE);
	ret   = strlen(buf)+1;

	return ret;

}
DEVICE_ATTR(ts_cmp, 0444  , ntg_DataCmp, NULL);

static ssize_t ntg_Loadfw(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret, err;
	struct ntg_tsData *ts_data;

	ts_data = private_ts;
	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);

	memcpy(spiTxbuf, &ntgFW_load, sizeof(char)*30);
	memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);

	printk(KERN_INFO "[ts-LoadFw]ts_LoadFw send\n");
/*	int i;
	for (i = 0; i <7 ; i++) {
		printk(KERN_INFO "[ts]%.2x%.2x%.2x%.2x\n",
				ntgFW_load[i*4], ntgFW_load[i*4+1], ntgFW_load[i*4+2], ntgFW_load[i*4+3]);
	}
	printk(KERN_INFO "[ts]%.2x%.2x\n", ntgFW_load[28], ntgFW_load[29]);
*/	err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
						sizeof(char)*NTRIG_PACKET_SIZE);
	ret   = strlen(buf)+1;
	return ret;
}
DEVICE_ATTR(ts_load, 0444  , ntg_Loadfw, NULL);

static ssize_t ntg_fwtest(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret, err;
	struct ntg_tsData *ts_data;
	ts_data = private_ts;
	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);
	printk(KERN_INFO "[ts-dfu_FWtest]FWtest send\n");
/*	char fwtest[33] = {0xFF, 0xFF, 0xFF, 0xFF,
					0xA5, 0x5A, 0xE7, 0x7E,
					0x02, 0x19, 0x00, 0x00,
					0x02, 0x61, 0x7E, 0x00,
					0x00, 0x13, 0x00, 0x01,
					0x07, 0x17, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0xFF
					};
	fwtest[32] = CalcChecksum(&fwtest[14], 19);
*/

	memcpy(spiTxbuf, &ntgFW_fwtest, sizeof(ntgFW_fwtest));
	memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE*2);
/*	int i;
	for (i = 0; i <8 ; i++) {
		printk(KERN_INFO "[ts]%.2x%.2x%.2x%.2x\n",
				ntgFW_fwtest[i*4], ntgFW_fwtest[i*4+1], ntgFW_fwtest[i*4+2], ntgFW_fwtest[i*4+3]);
	}
	printk(KERN_INFO "[ts]%.2x\n", ntgFW_fwtest[32]);
*/
	err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
						sizeof(char)*NTRIG_PACKET_SIZE);
	ret   = strlen(buf)+1;

	return ret;

}
DEVICE_ATTR(ts_fwtest, 0444  , ntg_fwtest, NULL);


static ssize_t ntg_valid(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret, err;
	struct ntg_tsData *ts_data;

	ts_data = private_ts;
	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);
	printk(KERN_INFO "[ts-dfu_valid]VALID test send\n");
/*
	char valid[29] = {0xFF, 0xFF, 0xFF, 0xFF,
					0xA5, 0x5A, 0xE7, 0x7E,
					0x02, 0x16, 0x00, 0x00,
					0x02, 0x61, 0x7E, 0x00,
					0x00, 0x15, 0x00, 0x01,
					0x07, 0x19, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x4C
					};
	valid[28] = CalcChecksum(&valid[14], 15);
*/
	memcpy(spiTxbuf, &ntgFW_valid, sizeof(ntgFW_valid));
	memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);

/*	int i;
	for (i = 0; i < 7; i++) {
		printk(KERN_INFO "[ts]%.2x%.2x%.2x%.2x\n",
				ntgFW_valid[i*4], ntgFW_valid[i*4+1], ntgFW_valid[i*4+2], ntgFW_valid[i*4+3]);
	}printk(KERN_INFO "[ts]%.2x\n", ntgFW_valid[28]);
*/
	err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
						sizeof(char)*NTRIG_PACKET_SIZE);
	ret   = strlen(buf)+1;
	return ret;

}
DEVICE_ATTR(ts_val, 0444  , ntg_valid, NULL);

static ssize_t ntg_SetCal(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret, err, i;
	struct ntg_tsData *ts_data;
	ts_data = private_ts;
	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);

	for (i = 0; i < 5; i++) {
		memcpy(spiTxbuf, &ntgFW_setCal, sizeof(ntgFW_setCal));
		memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);

		printk(KERN_INFO "[ts-SetCal-%d]SetCal test send\n", i);
		err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
								sizeof(char)*NTRIG_PACKET_SIZE);

		mdelay(50);
	}
	ret = sprintf(buf, "[Calibrating! wait 1 mins!!]\n");
	ret = strlen(buf)+1;
	return ret;
}
DEVICE_ATTR(ts_SetCal, 0444, ntg_SetCal, NULL);

static ssize_t ntgDFU(struct device *dev, struct device_attribute *attr,
						char *buf)
{

	printk(KERN_INFO "[ts]DFU_BL\n");
	ntg_GoToBl(dev, attr, buf);
	mdelay(1000);
	printk(KERN_INFO "[ts]DFU_START...wait 5 secs\n");
	ntg_DfuStart(dev, attr, buf);
	mdelay(5000);
	printk(KERN_INFO "[ts]DFU_BLK\n");
	ntg_DfuSendBlock(dev, attr, buf);
	mdelay(1000);
	printk(KERN_INFO "[ts]DFU_DATA.. wait! this take time\n");
	ntg_DfuSendData(dev, attr, buf);
	printk(KERN_INFO "[ts]DFU_COMPELE!\n");
	ntg_DataCmp(dev, attr, buf);
	printk(KERN_INFO "[ts]DFU_LOAD FW!... wait 15 sec\n");
	ntg_Loadfw(dev, attr, buf);
	mdelay(15000);
	printk(KERN_INFO "[ts]DFU_TEST FW\n");
	ntg_fwtest(dev, attr, buf);
	printk(KERN_INFO "[ts]FW_VALID\n[ts]Next Set Calibration... this take about 1 mins and DON'T TOUCH SCREEN\n");
	mdelay(10);
	ntg_valid(dev, attr, buf);
	printk(KERN_INFO "[ts]Calibrating !!... wait 1 min\n");
	ntg_SetCal(dev, attr, buf);
	mdelay(60000);
	printk(KERN_INFO "[ts]Calibration Finish !!\n");

	return 0;
}
DEVICE_ATTR(ts_DFU, 0444  , ntgDFU, NULL);

static ssize_t ntg_GetCal(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret, err, i;
	struct ntg_tsData *ts_data;

	ts_data = private_ts;

	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE*2);

	for (i = 0; i < 5; i++) {
		memcpy(spiTxbuf, &ntgFW_getCal, sizeof(ntgFW_getCal));
		memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);
		printk(KERN_INFO "[[ts-GetCal -%d]GetCal test send\n", i);
		err = spiDuplex_Lock(ts_data->spiDev, spiTxbuf, spiRxbuf,
				sizeof(char)*NTRIG_PACKET_SIZE);
		sprintf(buf, "[ts-GetCal-%d]tx:%x%x%x%x\\%x%x%x%x\n", i,
					spiRxbuf[8], spiRxbuf[9], spiRxbuf[10], spiRxbuf[11],
					spiRxbuf[12], spiRxbuf[13], spiRxbuf[14], spiRxbuf[15]);
		mdelay(100);

		ret = strlen(buf)+1;
	}
	return ret;
}
DEVICE_ATTR(ts_GetCal, 0444,  ntg_GetCal, NULL);

static int ntg_spi_D_read(struct spi_device *dev, struct ntg_touch_event *Event)
{
	int ret = 0;
	long *preamble ;

	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x0, sizeof(char)*NTRIG_PACKET_SIZE);

	preamble = (long *)spiTxbuf;

	spiDuplex_Lock(dev, spiTxbuf, spiRxbuf, NTRIG_PACKET_SIZE);

	ret = ntrg_frame_parser(spiRxbuf , Event) ;
/*
	printk(KERN_INFO "[ts_D_read]:%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
			spiRxbuf[0], spiRxbuf[1], spiRxbuf[2], spiRxbuf[3],
			spiRxbuf[4], spiRxbuf[5], spiRxbuf[6], spiRxbuf[7],
			spiRxbuf[8], spiRxbuf[9], spiRxbuf[10], spiRxbuf[11],
			spiRxbuf[12], spiRxbuf[13], spiRxbuf[14], spiRxbuf[15],
			spiRxbuf[16], spiRxbuf[17], spiRxbuf[18], spiRxbuf[19],
			spiRxbuf[20], spiRxbuf[21], spiRxbuf[22], spiRxbuf[23]);
*/
	return ret;
}

static void ntrig_ts_work(struct work_struct *work)
{
	int ret;
	struct ntg_tsData *ntg_ts;
	ntg_ts = container_of(work, struct ntg_tsData, ntg_work);
	ret = ntg_spi_D_read(ntg_ts->spiDev, ntg_ts->touchEvnt);
	if (ntg_ts->touchEvnt->header.type == 0x01
			&& ntg_ts->touchEvnt->header.channel == 0x02) {
		if (ntg_ts->touchEvnt->stouch.bit & 0x02) {
			if (ntg_ts->touchEvnt->stouch.posX	> 4500 &&
				ntg_ts->touchEvnt->stouch.posX < 6500 &&
				ntg_ts->touchEvnt->stouch.posY > 9100) {
				input_report_key(ntg_ts->input_dev, KEY_BACK, 1);
				input_report_key(ntg_ts->input_dev, KEY_BACK, 0);
			} else {
				input_report_abs(ntg_ts->input_dev, ABS_MT_TOUCH_MAJOR, 25);
				input_report_abs(ntg_ts->input_dev, ABS_MT_WIDTH_MAJOR,
					(ntg_ts->touchEvnt->stouch.posDy + ntg_ts->touchEvnt->stouch.posDy)/168);
				input_report_abs(ntg_ts->input_dev, ABS_MT_POSITION_X,
								ntg_ts->touchEvnt->stouch.posX);
				input_report_abs(ntg_ts->input_dev, ABS_MT_POSITION_Y,
								ntg_ts->touchEvnt->stouch.posY);
			}
		} else {
			input_report_abs(ntg_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		}
		input_mt_sync(ntg_ts->input_dev);
	} else if (ntg_ts->touchEvnt->header.type == 0x01
		&& ntg_ts->touchEvnt->header.channel == 0x03) {
		if (ntg_ts->touchEvnt->ptouch.bit & 0x02) {
			input_report_abs(ntg_ts->input_dev, ABS_MT_TOUCH_MAJOR, 25);
			input_report_abs(ntg_ts->input_dev, ABS_MT_WIDTH_MAJOR, 10);
			input_report_abs(ntg_ts->input_dev, ABS_MT_POSITION_X,
						ntg_ts->touchEvnt->ptouch.posX);
			input_report_abs(ntg_ts->input_dev, ABS_MT_POSITION_Y,
						ntg_ts->touchEvnt->ptouch.posY);
		} else {
			input_report_abs(ntg_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		}
		input_mt_sync(ntg_ts->input_dev);
	} /*else if (ntg_ts->touchEvnt->header.type == 0x01
		&& ntg_ts->touchEvnt->header.channel == 0x03) {
	}*/
		input_sync(ntg_ts->input_dev);
		memset(ntg_ts->touchEvnt, 0x00, sizeof(struct ntg_touch_event));
		enable_irq(ntg_ts->spiDev->irq);
}

static irqreturn_t ntrig_ts_irq_handler(int irq, void *dev_id)
{
	struct ntg_tsData *ts;
	ts = dev_id;

	disable_irq_nosync(ts->spiDev->irq);
	queue_work(ts->ntg_wq, &ts->ntg_work);
	return IRQ_HANDLED;
}
static int ntrig_ts_probe(struct spi_device *dev)
{
	int ret, i, err;
	struct ntg_tsData *ntg_ts;
	struct ntrig_spi_platform_data *pdata;

	spiTxbuf = kzalloc(sizeof(char)*(2*NTRIG_PACKET_SIZE), GFP_KERNEL);
	spiRxbuf = kzalloc(sizeof(char)*(2*NTRIG_PACKET_SIZE), GFP_KERNEL);
	pdata = dev->dev.platform_data;

	ret = 0;
	for (i = 0; i < 10 ; i++) {
		if (ret == 0) {
			gpio_set_value(pdata->spi_enable, 1);
			break;
		}
		mdelay(10);
	}
	ntg_ts = kzalloc(sizeof(struct ntg_tsData), GFP_KERNEL);
	if (ntg_ts == NULL) {
		printk(KERN_INFO "%s:allocate ntg_tsdata failed \n", __func__);
		ret = -ENODEV;
		goto err_alloc_data_failed;
	}

	ntg_ts->ntg_wq = create_singlethread_workqueue("ntrig_wq");
	if (!ntg_ts->ntg_wq) {
		printk(KERN_INFO "%s:allocate ntg_tsdata failed \n", __func__);
		ret = -ENOMEM;
		goto err_create_wq_failed;
	}
	INIT_WORK(&ntg_ts->ntg_work, ntrig_ts_work);
	ntg_ts->spiDev = dev;
	if (!ntg_ts->spiDev)
		printk(KERN_INFO "[ts]:dev pointer give failure!\n");

	dev_set_drvdata(&dev->dev, ntg_ts);
	ntg_ts->touchEvnt = kzalloc(sizeof(struct ntg_touch_event), GFP_KERNEL);
	ntg_ts->abs_x_max = pdata->abs_x_max;
	ntg_ts->abs_x_min =  pdata->abs_x_min;
	ntg_ts->abs_y_max = pdata->abs_y_max;
	ntg_ts->abs_y_min = pdata->abs_y_min;
	ntg_ts->abs_pressure_max = pdata->abs_pressure_max;
	ntg_ts->abs_pressure_min = pdata->abs_pressure_min;
	ntg_ts->abs_width_max = pdata->abs_width_max;
	ntg_ts->abs_width_min = pdata->abs_width_min;

	ntg_ts->input_dev = input_allocate_device();
	if (ntg_ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&ntg_ts->spiDev->dev,
			"Failed to allocate input device\n");
		printk(KERN_INFO "[ts] input_allocate_device failed\n");
		goto err_input_dev_alloc_failed;
	}

	ntg_ts->input_dev->name = "ntrig-G41-touchscreen";
	set_bit(EV_SYN, ntg_ts->input_dev->evbit);
	set_bit(EV_KEY, ntg_ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ntg_ts->input_dev->keybit);
	set_bit(BTN_2, ntg_ts->input_dev->keybit);
	set_bit(EV_ABS, ntg_ts->input_dev->evbit);
	set_bit(KEY_BACK, ntg_ts->input_dev->keybit);
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_POSITION_X,
					ntg_ts->abs_x_min, ntg_ts->abs_x_max, 0, 0);
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_POSITION_Y,
					ntg_ts->abs_y_min, ntg_ts->abs_y_max, 0, 0);
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_TOUCH_MAJOR,
					ntg_ts->abs_pressure_min, ntg_ts->abs_pressure_max, 0, 0);
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_WIDTH_MAJOR,
					ntg_ts->abs_width_min, ntg_ts->abs_width_max, 0, 0);
	ret = input_register_device(ntg_ts->input_dev);
	if (ret) {
		dev_err(&ntg_ts->spiDev->dev,
			"ntrig_ts_probe: Unable to register %s input device\n",
					ntg_ts->input_dev->name);
		printk(KERN_INFO "[ts] input_register_device failed\n");
		goto err_input_register_device_failed;
	}

	ret = request_irq(dev->irq, ntrig_ts_irq_handler, IRQF_TRIGGER_HIGH,
					"ntrig_irq_g41", ntg_ts);
	if (ret < 0) {
		dev_err(&ntg_ts->spiDev->dev, "request_irq_failed");
		printk(KERN_INFO "[ts] ********* request_irq failed, %d\n",
			ret);
	}
/*	//track Lib
	if(!NtrTrackingInit())
		printk(KERN_INFO "[ts] Ntring Tracking Lib Failure\n");
	else
		printk(KERN_INFO "[ts] Ntring Tracking Lib OK\n");
*/
	private_ts = ntg_ts;

	err = device_create_file(&(dev->dev), &dev_attr_ts_fw);
	err = device_create_file(&(dev->dev), &dev_attr_ts_SetCal);
	err = device_create_file(&(dev->dev), &dev_attr_ts_GetCal);

	err = device_create_file(&(dev->dev), &dev_attr_ts_bl);
	err = device_create_file(&(dev->dev), &dev_attr_ts_Start);
	err = device_create_file(&(dev->dev), &dev_attr_ts_Blk);
	err = device_create_file(&(dev->dev), &dev_attr_ts_Data);
	err = device_create_file(&(dev->dev), &dev_attr_ts_cmp);
	err = device_create_file(&(dev->dev), &dev_attr_ts_load);
	err = device_create_file(&(dev->dev), &dev_attr_ts_fwtest);
	err = device_create_file(&(dev->dev), &dev_attr_ts_val);
	err = device_create_file(&(dev->dev), &dev_attr_ts_DFU);
	return 0;

err_input_register_device_failed:
	input_free_device(ntg_ts->input_dev);

err_input_dev_alloc_failed:
	destroy_workqueue(ntg_ts->ntg_wq);

err_create_wq_failed:
	kfree(ntg_ts);

err_alloc_data_failed:
	return ret;
}

static int __exit ntrig_ts_remove(struct spi_device *dev)
{
	struct ntg_tsData *ts;

	kfree(spiTxbuf);
	kfree(spiRxbuf);

	ts = private_ts;

/*	NtgTrackingDestory();*/
	destroy_workqueue(ts->ntg_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	device_remove_file(&(dev->dev), &dev_attr_ts_fw);
	device_remove_file(&(dev->dev), &dev_attr_ts_SetCal);
	device_remove_file(&(dev->dev), &dev_attr_ts_GetCal);

	device_remove_file(&(dev->dev), &dev_attr_ts_bl);
	device_remove_file(&(dev->dev), &dev_attr_ts_Start);
	device_remove_file(&(dev->dev), &dev_attr_ts_Blk);
	device_remove_file(&(dev->dev), &dev_attr_ts_Data);
	device_remove_file(&(dev->dev), &dev_attr_ts_cmp);
	device_remove_file(&(dev->dev), &dev_attr_ts_fwtest);
	device_remove_file(&(dev->dev), &dev_attr_ts_val);

	device_remove_file(&(dev->dev), &dev_attr_ts_DFU);
	return 0;
}

static int ntrig_ts_suspend(struct spi_device *dev, pm_message_t mesg)
{
	int ret = 0;
	struct ntg_tsData *ts;
	printk(KERN_INFO "[ts]N-trig suspend \n");
	ts = private_ts;

	disable_irq(ts->spiDev->irq);

	ret = cancel_work_sync(&ts->ntg_work);
	if (ret)
		enable_irq(ts->spiDev->irq);
	return ret;
}

static int ntrig_ts_resume(struct spi_device *dev)
{
	struct ntg_tsData *ts ;
	int ret = 0;

	printk(KERN_INFO "[ts]N-trig resume\n");

	ts = private_ts;
	enable_irq(ts->spiDev->irq);
	return ret;
}

static const struct spi_device_id ntrig_ts_spi_id[] = {
	{NTRIG_G41_NAME, 0},
	{}
};

static struct spi_driver ntrig_ts_driver = {
	.id_table = ntrig_ts_spi_id,
	.probe = ntrig_ts_probe,
	.suspend = ntrig_ts_suspend,
	.resume = ntrig_ts_resume,
	.remove = __exit_p(ntrig_ts_remove),
	.driver = {
		.name = NTRIG_G41_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ntrig_ts_init(void)
{
	int ret = 0;
	ret = spi_register_driver(&ntrig_ts_driver);
	return ret;
}

static void __exit ntrig_ts_exit(void)
{
	spi_unregister_driver(&ntrig_ts_driver);
}


module_init(ntrig_ts_init);
module_exit(ntrig_ts_exit);

MODULE_DESCRIPTION("N-trig SPI TOUCHSCREEN");
MODULE_LICENSE("GPL");


