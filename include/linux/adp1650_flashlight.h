/*
 * Definitions for adp1650 led flash chip.
 */

#ifndef LEDFLASH_ADP1650_H
#define LEDFLASH_ADP1650_H

#include <linux/ioctl.h>

#define MAX_FAILURE_COUNT 		3

/* ADP1650 register address */
#define INFO_REG			(uint8_t) 0x00
#define TIMER_REG			(uint8_t) 0x02
#define CURRENT_SET_REG 		(uint8_t) 0x03
#define OUTPUT_MODE_REG 		(uint8_t) 0x04
#define FAULT_INFO_REG			(uint8_t) 0x05
#define INPUT_CTRL_REG			(uint8_t) 0x06
#define AD_MOD_REG			(uint8_t) 0x07
#define AD_ADC_REG			(uint8_t) 0x08
#define BATT_LOW_REG			(uint8_t) 0x09

/* VREF and timer register */
#define TIMER_200MS			(uint8_t) 0x01
#define TIMER_600MS                     (uint8_t) 0x05
#define TIMER_1600MS			(uint8_t) 0x0f

/* Current set register */
#define CUR_FL_700MA			(uint8_t) 0x40

#define CUR_TOR_25MA			(uint8_t) 0x00
#define CUR_TOR_50MA			(uint8_t) 0x01
#define CUR_TOR_75MA			(uint8_t) 0x02
#define CUR_TOR_100MA			(uint8_t) 0x03
#define CUR_TOR_125MA			(uint8_t) 0x04
#define CUR_TOR_150MA			(uint8_t) 0x05
#define CUR_TOR_175MA			(uint8_t) 0x06
#define CUR_TOR_200MA			(uint8_t) 0x07

/* Output mode register */
#define OUTPUT_MODE_DEF 		(uint8_t) 0xa0
#define OUTPUT_MODE_ASSIST		(uint8_t) 0x0a
#define OUTPUT_MODE_FLASH		(uint8_t) 0x0b

struct adp1650_platform_data {
	int reset;
	int intr;
};

#endif

