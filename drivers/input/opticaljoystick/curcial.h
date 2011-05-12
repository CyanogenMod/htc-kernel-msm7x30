/* drivers/input/opticaljoystick/curcial.h
 *
 * Copyright (C) 2009 HTC Corporation.
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
#ifndef _CURCIAL_H
#define _CURCIAL_H

#include <linux/input.h>

typedef enum {
	OJ_KEY_RIGHT = KEY_RIGHT,
	OJ_KEY_LEFT = KEY_LEFT,
	OJ_KEY_UP = KEY_UP,
	OJ_KEY_DOWN = KEY_DOWN,
/*	OJ_KEY_CLICK = KEY_REPLY,*/
	OJ_KEY_NONE
}OJKeyEvt_T;

typedef enum {
	OJ_TOUCH_NONE_EVT = 0,
	OJ_TOUCH_PRESS_EVT,
	OJ_TOUCH_RELEASE_EVT,
	OJ_TOUCH_CLICK_EVT
}OJTouchEvt_T;

typedef struct {
	int8_t 	deltaX;
	int8_t 	deltaY;
	int8_t	shtHi;
	int8_t	shtLo;
	uint8_t squal;
	uint16_t	key;
}OJData_T;

enum {
  OJ_QUEUE_01 = 0,
  OJ_QUEUE_02,
  OJ_QUEUE_03,
  OJ_QUEUE_04,
  OJ_QUEUE_05,
  OJ_QUEUE_MAX
};

extern OJTouchEvt_T 	OJ_SoftClick_Event(OJData_T* OJData);
extern OJTouchEvt_T		gTouchEvt;
extern uint8_t  gSqRatio;
extern uint8_t  gdeltamod;
extern uint8_t	gPressBufCnt;
extern uint8_t  softclick;
extern uint8_t  gDeltaU;
extern uint8_t  gDeltaL;
#endif
