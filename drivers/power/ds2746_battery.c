/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Copyright (c) 2010 High Tech Computer Corporation

Module Name:

		ds2746_battery.c

Abstract:

		This module implements the power algorithm, including below concepts:
		1. Charging function control.
		2. Charging full condition.
		3. Recharge control.
		4. Battery capacity maintainance.
		5. Battery full capacity calibration.

Original Auther:

		Andy.YS Wang  June-01-2010
---------------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/android_alarm.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/ds2746_battery.h>
#include <linux/ds2746_battery_config.h>
#include <linux/ds2746_param.h>
#include <linux/wrapper_types.h>
#include <linux/smb329.h>
#include <mach/htc_battery.h>
#include <asm/mach-types.h>
#include "../../arch/arm/mach-msm/proc_comm.h"
#include <linux/i2c.h>  					/* for i2c_adapter, i2c_client define*/
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <mach/board.h>

struct ds2746_device_info {

		struct device *dev;
		struct device *w1_dev;
		struct workqueue_struct *monitor_wqueue;
		struct work_struct monitor_work;
		/* lock to protect the battery info */
		struct mutex lock;
		/* DS2746 data, valid after calling ds2746_battery_read_status() */
		unsigned long update_time;	/* jiffies when data read */
		struct alarm alarm;
		struct wake_lock work_wake_lock;
		u8 slow_poll;
		ktime_t last_poll;
};
static struct wake_lock vbus_wake_lock;

/*========================================================================================

HTC power algorithm helper member and functions

========================================================================================*/

static struct poweralg_type poweralg = {0};
static struct poweralg_config_type config = {0};
static struct poweralg_config_type debug_config = {0};
BOOL is_need_battery_id_detection = TRUE;
/* workaround to get device_info */
static struct ds2746_device_info *g_di_ptr = NULL;

static int g_first_update_charger_ctl = 1;

#define FAST_POLL	(1 * 30)
#define SLOW_POLL	(10 * 60)
#define PREDIC_POLL	20

#define HTC_BATTERY_I2C_DEBUG_ENABLE		0
#define HTC_BATTERY_DS2746_DEBUG_ENABLE 	1

/* DS2746 I2C BUS*/
#define DS2746_I2C_BUS_ID   	0
#define DS2746_I2C_SLAVE_ADDR   0x26

/* safety timer */
static UINT32 delta_time_sec = 0;
static UINT32 chg_en_time_sec = 0;
static UINT32 super_chg_on_time_sec = 0;

/*========================================================================================

IC dependent defines

========================================================================================*/

/* DS2746 I2C register address*/
#define DS2746_STATUS_REG	0x01
#define DS2746_AUX0_MSB		0x08
#define DS2746_AUX0_LSB 	0x09
#define DS2746_AUX1_MSB 	0x0A
#define DS2746_AUX1_LSB 	0x0B
#define DS2746_VOLT_MSB 	0x0C
#define DS2746_VOLT_LSB 	0x0D
#define DS2746_CURRENT_MSB	0x0E
#define DS2746_CURRENT_LSB	0x0F
#define DS2746_ACR_MSB  	0x10
#define DS2746_ACR_LSB  	0x11

/* DS2746 I2C I/O*/
static struct i2c_adapter *i2c2 = NULL;
static struct i2c_client *ds2746_i2c = NULL;

int ds2746_i2c_write_u8(u8 value, u8 reg)
{
	int ret;
	u8 buf[2];
	struct i2c_msg *msg;
	struct i2c_msg xfer_msg[1];

	/* [MSG1] fill the register address data and fill the data Tx buffer */
	msg = &xfer_msg[0];
	msg->addr = ds2746_i2c->addr;
	msg->len = 2;
	msg->flags = 0; 		/* Read the register value */
	msg->buf = buf;

	buf[0] = reg;
	buf[1] = value;

	ret = i2c_transfer(ds2746_i2c->adapter, xfer_msg, 1);
	if (ret <= 0){
		printk(DRIVER_ZONE "[%s] fail.\n", __func__);
	}

#if HTC_BATTERY_I2C_DEBUG_ENABLE
	printk(DRIVER_ZONE "[%s] ds2746[0x%x]<-0x%x.\n", __func__, reg, value);
#endif

	return ret;
}

int ds2746_i2c_read_u8(u8 *value, u8 reg)
{
	int ret;
	struct i2c_msg *msg;
	struct i2c_msg xfer_msg[2];

	/* [MSG1] fill the register address data */
	msg = &xfer_msg[0];
	msg->addr = ds2746_i2c->addr;
	msg->len = 1;
	msg->flags = 0; 		/* Read the register value */
	msg->buf = &reg;
	/* [MSG2] fill the data rx buffer */
	msg = &xfer_msg[1];
	msg->addr = ds2746_i2c->addr;
	msg->len = 1;
	msg->flags = I2C_M_RD;  /* Read the register value */
	msg->buf = value;

	ret = i2c_transfer(ds2746_i2c->adapter, xfer_msg, 2);
	if (ret <= 0){
		printk(DRIVER_ZONE "[%s] fail.\n", __func__);
	}

#if HTC_BATTERY_I2C_DEBUG_ENABLE
	printk(DRIVER_ZONE "[%s] ds2746[0x%x]=0x%x.\n", __func__, reg, *value);
#endif

	return ret;
}

static void ds2746_i2c_exit(void)
{
	if (ds2746_i2c != NULL){
		kfree(ds2746_i2c);
		ds2746_i2c = NULL;
	}

	if (i2c2 != NULL){
		i2c_put_adapter(i2c2);
		i2c2 = NULL;
	}
}

static int ds2746_i2c_init(void)
{
	i2c2 = i2c_get_adapter(DS2746_I2C_BUS_ID);
	ds2746_i2c = kzalloc(sizeof(*ds2746_i2c), GFP_KERNEL);

	if (i2c2 == NULL || ds2746_i2c == NULL){
		printk(DRIVER_ZONE "[%s] fail (0x%x, 0x%x).\n",
			__func__,
			(int) i2c2,
			(int) ds2746_i2c);
		ds2746_i2c_exit();
		return -ENOMEM;
	}

	ds2746_i2c->adapter = i2c2;
	ds2746_i2c->addr = DS2746_I2C_SLAVE_ADDR;

	return 0;
}

/*========================================================================================

	HTC supporting MFG testing member and functions

=========================================================================================*/

static BOOL b_is_charge_off_by_bounding = FALSE;
static void bounding_fullly_charged_level(int upperbd)
{
	static int pingpong = 1;
	int lowerbd;
	int current_level;
	b_is_charge_off_by_bounding = FALSE;
	if (upperbd <= 0)
		return; /* doesn't activated this function */
	lowerbd = upperbd - 5; /* 5% range */

	if (lowerbd < 0)
		lowerbd = 0;
	current_level = CEILING(poweralg.capacity_01p, 10);

	if (pingpong == 1 && upperbd <= current_level) {
		printk(DRIVER_ZONE "MFG: lowerbd=%d, upperbd=%d, current=%d, pingpong:1->0 turn off\n", lowerbd, upperbd, current_level);
		b_is_charge_off_by_bounding = TRUE;
		pingpong = 0;
	} else if (pingpong == 0 && lowerbd < current_level) {
		printk(DRIVER_ZONE "MFG: lowerbd=%d, upperbd=%d, current=%d, toward 0, turn off\n", lowerbd, upperbd, current_level);
		b_is_charge_off_by_bounding = TRUE;
	} else if (pingpong == 0 && current_level <= lowerbd) {
		printk(DRIVER_ZONE "MFG: lowerbd=%d, upperbd=%d, current=%d, pingpong:0->1 turn on\n", lowerbd, upperbd, current_level);
		pingpong = 1;
	} else {
		printk(DRIVER_ZONE "MFG: lowerbd=%d, upperbd=%d, current=%d, toward %d, turn on\n", lowerbd, upperbd, current_level, pingpong);
	}

}

static BOOL is_charge_off_by_bounding_condition(void)
{
	return b_is_charge_off_by_bounding;
}

void calibrate_id_ohm(struct battery_type *battery)
{
	if (!poweralg.charging_source || !poweralg.charging_enable){
		battery->id_ohm += 500; 		/* If device is in discharge mode, Rid=Rid_1 + 0.5Kohm*/
	}
	else if (poweralg.charging_source == 2 && battery->current_mA >= 400 && battery->id_ohm >= 1500){
		battery->id_ohm -= 1500;		/* If device is in charge mode and ISET=1 (charge current is <800mA), Rid=Rid_1 - 1.5Kohm*/
	}
	else if (battery->id_ohm >= 700){
		battery->id_ohm -= 700; 		/* If device is in charge mode and ISET=0 (charge current is <400mA), Rid=Rid_1 - 0.7Kohm*/
	}
}

static BOOL is_charging_avaiable(void)
{
	if (poweralg.is_superchg_software_charger_timeout) return FALSE;
	if (poweralg.is_software_charger_timeout) return FALSE;
	if (!poweralg.protect_flags.is_charging_enable_available)return FALSE;
	if (!poweralg.is_cable_in) return FALSE;
	if (poweralg.charge_state == CHARGE_STATE_PENDING) return FALSE;
	if (poweralg.charge_state == CHARGE_STATE_FULL_PENDING)	return FALSE;
	if (poweralg.charge_state == CHARGE_STATE_PREDICTION) return FALSE;
	if (is_charge_off_by_bounding_condition()) return FALSE;
	return TRUE; /* CHARGE_STATE_UNKNOWN, SET_LED_BATTERY_CHARGING is available to be charged by default*/
}

static BOOL is_high_current_charging_avaialable(void)
{
	if (!poweralg.protect_flags.is_charging_high_current_avaialble)	return FALSE;
	if (!poweralg.is_china_ac_in) return FALSE;
	if (poweralg.charge_state == CHARGE_STATE_UNKNOWN) return FALSE;
	return TRUE;
}

static BOOL is_super_current_charging_avaialable(void)
{
	if (!poweralg.is_super_ac) return FALSE;
	return TRUE;
}
static BOOL is_set_min_taper_current(void)
{
	if ((config.min_taper_current_ma > 0) &&
		(config.min_taper_current_mv > 0) &&
		(poweralg.battery.current_mA < config.min_taper_current_ma) &&
		(config.min_taper_current_mv < poweralg.battery.voltage_mV))
		return TRUE;

	return FALSE;
}

static void update_next_charge_state(void)
{
	static UINT32 count_charging_full_condition;
	static UINT32 count_charge_over_load;
	int next_charge_state;
	int i;

	/*  unknown -> prediction -> unknown -> discharge/charging/pending
	charging -> full-wait-stable -> full-charging -> full-pending
	full-pending -> full-charging -> charging
	*(cable in group) -> discharge, charge-pending, dead
	*(cable out group), full-wait-stable, charge-pending, dead -> charging*/

	for (i = 0; i < 25; i++) /* maximun 25 times state transition to prevent from busy loop; ideally the transition time shall be less than 5 times.*/
	{
		/*printk(DRIVER_ZONE "%s.run[%d]: poweralg.charge_state=%d\n",
			__func__, i,poweralg.charge_state);*/
		next_charge_state = poweralg.charge_state;

		/* 0. enter prediction state or not*/
		if (poweralg.charge_state == CHARGE_STATE_UNKNOWN){
			if (poweralg.battery.is_power_on_reset || config.debug_always_predict){
				if (poweralg.protect_flags.is_battery_dead){
					/* keep poweralg.charge_state unchanged, set capacity to 0% directly*/
					printk(DRIVER_ZONE " dead battery, \
						p=0%%\n");
					poweralg.capacity_01p = 0;
					battery_capacity_update(&poweralg.battery, poweralg.capacity_01p);

					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;
				}
				else{
					/* battery replaced, recalculate capacity based on battery voltage*/
					printk(DRIVER_ZONE " start predict discharge...\n");
					next_charge_state = CHARGE_STATE_PREDICTION;
				}

				config.debug_always_predict = FALSE;
			}
		}

		if (next_charge_state == poweralg.charge_state){
			/*---------------------------------------------------------------------------------------------------*/
			/* 1. cable in group*/
			if (poweralg.charge_state == CHARGE_STATE_UNKNOWN ||
				poweralg.charge_state == CHARGE_STATE_CHARGING ||
				poweralg.charge_state == CHARGE_STATE_PENDING ||
				poweralg.charge_state == CHARGE_STATE_FULL_WAIT_STABLE ||
				poweralg.charge_state == CHARGE_STATE_FULL_CHARGING ||
				poweralg.charge_state == CHARGE_STATE_FULL_RECHARGING ||
				poweralg.charge_state == CHARGE_STATE_FULL_PENDING){
				if (!poweralg.is_cable_in){
					next_charge_state = CHARGE_STATE_DISCHARGE;
				}
				else if (!poweralg.protect_flags.is_charging_enable_available){
					next_charge_state = CHARGE_STATE_PENDING;
				}
			}

			/*---------------------------------------------------------------------------------------------------*/
			/* 2. cable out group*/
			if (poweralg.charge_state == CHARGE_STATE_UNKNOWN ||
				poweralg.charge_state == CHARGE_STATE_DISCHARGE){
				if (poweralg.is_cable_in){
					next_charge_state = CHARGE_STATE_CHARGING;
				}
			}
		}

		/*---------------------------------------------------------------------------------------------------*/
		/* 3. state handler/transition, if the charge state is not changed due to cable/protect flags*/
		if (next_charge_state == poweralg.charge_state){
			switch (poweralg.charge_state){
				case CHARGE_STATE_PREDICTION:
					{
						UINT32 end_time_ms = BAHW_MyGetMSecs();

						if (end_time_ms - poweralg.state_start_time_ms >=
							config.predict_timeout_sec * 1000){

							printk(DRIVER_ZONE "predict done [%d->%d]\n", poweralg.state_start_time_ms,
								end_time_ms);
							next_charge_state = CHARGE_STATE_UNKNOWN;
						}
					}
					break;
				case CHARGE_STATE_CHARGING:
					if (!poweralg.battery.is_power_on_reset){
						/* -> full-charging, pending, dead*/
						if (poweralg.capacity_01p > 990){
							/* only ever charge-full, the capacity can be larger than 99.0%*/
							next_charge_state = CHARGE_STATE_FULL_CHARGING; // MATT: should be FULL_RECHARGING ?
						}
						else if (poweralg.battery.voltage_mV >= config.full_charging_mv &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_charging_ma){
							/* meet charge full terminate condition, check again*/
							next_charge_state = CHARGE_STATE_FULL_WAIT_STABLE;
						}
					}

					if (poweralg.battery.current_mA <= 0){
						/* count_charge_over_load is 5 as max*/
						if (count_charge_over_load < 5)
							count_charge_over_load++;
						else
							poweralg.is_charge_over_load = TRUE;
					}
					else{
						count_charge_over_load = 0;
						poweralg.is_charge_over_load = FALSE;
					}

					/* is_software_charger_timeout: only triggered when AC adapter in*/
					if (config.software_charger_timeout_sec && poweralg.is_china_ac_in){
						/* software charger timer is enabled; for AC charge only*/
						UINT32 end_time_ms = BAHW_MyGetMSecs();

						if (end_time_ms - poweralg.state_start_time_ms >=
							config.software_charger_timeout_sec * 1000){

							printk(DRIVER_ZONE "software charger timer timeout [%d->%d]\n",
								poweralg.state_start_time_ms,
								end_time_ms);
							poweralg.is_software_charger_timeout = TRUE;
						}
					}
					/* is_superchg_software_charger_timeout: only triggered when superAC adapter in*/
#if 0
					if (config.superchg_software_charger_timeout_sec && poweralg.is_super_ac
						&& FALSE==poweralg.is_superchg_software_charger_timeout){
						/* software charger timer is enabled; for superAC charge*/
						UINT32 end_time_ms = BAHW_MyGetMSecs();

						if (end_time_ms - poweralg.state_start_time_ms >=
							config.superchg_software_charger_timeout_sec * 1000){

							printk(DRIVER_ZONE "superchg software charger timer timeout [%d->%d]\n",
								poweralg.state_start_time_ms,
								end_time_ms);
							poweralg.is_superchg_software_charger_timeout = TRUE;
						}
					}
#endif
					/* Software should also toggle MCHG_EN within 4 hrs
						to prevent charger HW safety timer expired. */
#if 0
					if (config.charger_hw_safety_timer_watchdog_sec && poweralg.is_cable_in){
						UINT32 end_time_ms = BAHW_MyGetMSecs();
						if (end_time_ms - poweralg.last_charger_enable_toggled_time_ms >=
							config.charger_hw_safety_timer_watchdog_sec * 1000){
							poweralg.last_charger_enable_toggled_time_ms = BAHW_MyGetMSecs();
							poweralg.is_need_toggle_charger = TRUE;
							printk(DRIVER_ZONE "need software toggle charger [%d->%d]\n",
								poweralg.last_charger_enable_toggled_time_ms,
								end_time_ms);

						}
					}
#endif
					break;
				case CHARGE_STATE_FULL_WAIT_STABLE:
					{
						/* -> full-charging, pending, dead*/
						if (poweralg.battery.voltage_mV >= config.full_charging_mv &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_charging_ma){

							count_charging_full_condition++;
						}
						else{
							count_charging_full_condition = 0;
							next_charge_state = CHARGE_STATE_CHARGING;
						}

						if (count_charging_full_condition >= 3){

							poweralg.capacity_01p = 1000;
							battery_capacity_update(&poweralg.battery, poweralg.capacity_01p);

							next_charge_state = CHARGE_STATE_FULL_CHARGING;
						}
					}
					break;
				case CHARGE_STATE_FULL_CHARGING:
					{
						/* -> full-pending, charging*/
						UINT32 end_time_ms = BAHW_MyGetMSecs();

						if (poweralg.battery.voltage_mV < config.voltage_exit_full_mv){
							if (poweralg.capacity_01p > 990)
								poweralg.capacity_01p = 990;
							next_charge_state = CHARGE_STATE_CHARGING;
						}
						else if (config.full_pending_ma != 0 &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_pending_ma){

							printk(DRIVER_ZONE " charge-full pending(%dmA)(%d:%d)\n",
								poweralg.battery.current_mA,
								poweralg.state_start_time_ms,
								end_time_ms);

							next_charge_state = CHARGE_STATE_FULL_PENDING;
						}
						else if (end_time_ms - poweralg.state_start_time_ms >=
							config.full_charging_timeout_sec * 1000){

							printk(DRIVER_ZONE " charge-full (expect:%dsec)(%d:%d)\n",
								config.full_charging_timeout_sec,
								poweralg.state_start_time_ms,
								end_time_ms);
							next_charge_state = CHARGE_STATE_FULL_PENDING;
						}
					}
					break;
				case CHARGE_STATE_FULL_PENDING:
					if ((poweralg.battery.voltage_mV >= 0 &&
						poweralg.battery.voltage_mV < config.voltage_recharge_mv) ||
						(poweralg.battery.RARC_01p >= 0 &&
						poweralg.battery.RARC_01p <= config.capacity_recharge_p * 10)){
						/* -> full-recharging*/
						next_charge_state = CHARGE_STATE_FULL_RECHARGING;
					}
					break;
				case CHARGE_STATE_FULL_RECHARGING:
					{
						if (poweralg.battery.voltage_mV < config.voltage_exit_full_mv){
							if (poweralg.capacity_01p > 990)
								poweralg.capacity_01p = 990;
							next_charge_state = CHARGE_STATE_CHARGING;
						}
						else if (poweralg.battery.voltage_mV >= config.full_charging_mv &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_charging_ma){
							/* meet charge full terminate condition, check again*/
							next_charge_state = CHARGE_STATE_FULL_CHARGING;
						}
					}
					break;
				case CHARGE_STATE_PENDING:
				case CHARGE_STATE_DISCHARGE:
					{
						UINT32 end_time_ms = BAHW_MyGetMSecs();

						if (!poweralg.is_voltage_stable){
							if (end_time_ms - poweralg.state_start_time_ms >=
								config.wait_votlage_statble_sec * 1000){

								printk(DRIVER_ZONE " voltage stable\n");
								poweralg.is_voltage_stable = TRUE;
							}
						}
					}

					if (poweralg.is_cable_in &&
						poweralg.protect_flags.is_charging_enable_available){
						/* -> charging*/
						next_charge_state = CHARGE_STATE_CHARGING;
					}
					break;
			}
		}

		/*---------------------------------------------------------------------------------------------------*/
		/* 4. state transition*/
		if (next_charge_state != poweralg.charge_state){
			/* state exit*/
			switch (poweralg.charge_state){
				case CHARGE_STATE_UNKNOWN:
					poweralg.capacity_01p = poweralg.battery.RARC_01p;
					if (poweralg.capacity_01p > 990)
						poweralg.capacity_01p = 990;
					if (poweralg.capacity_01p < 0)
						poweralg.capacity_01p = 0;

					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;
					break;
				case CHARGE_STATE_PREDICTION:
					battery_param_update(&poweralg.battery,
						&poweralg.protect_flags);

					poweralg.capacity_01p = poweralg.battery.KADC_01p;
					if (poweralg.capacity_01p > 990)
						poweralg.capacity_01p = 990;
					if (poweralg.capacity_01p < 0)
						poweralg.capacity_01p = 0;
					battery_capacity_update(&poweralg.battery,
						poweralg.capacity_01p);

					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;
					break;
			}

			/* state init*/
			poweralg.state_start_time_ms = BAHW_MyGetMSecs();

			switch (next_charge_state){
				case CHARGE_STATE_DISCHARGE:
				case CHARGE_STATE_PENDING:
					/*! star_lee 20100426 - always set ACR=FULL when discharge starts and ACR>FULL*/
					if (poweralg.battery.RARC_01p > 1000)
						battery_capacity_update(&poweralg.battery, 1000);

					poweralg.is_need_calibrate_at_49p = TRUE;
					poweralg.is_need_calibrate_at_14p = TRUE;
					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;
					poweralg.is_voltage_stable = FALSE;

					break;
				case CHARGE_STATE_CHARGING:
					poweralg.is_need_toggle_charger = FALSE;
					poweralg.last_charger_enable_toggled_time_ms = BAHW_MyGetMSecs();
					poweralg.is_software_charger_timeout = FALSE;   /* reset software charger timer every time when charging re-starts*/
					poweralg.is_charge_over_load = FALSE;
					count_charge_over_load = 0;
					poweralg.battery.charge_full_real_mAh = poweralg.battery.charge_full_design_mAh;
					battery_capacity_update(&poweralg.battery, poweralg.capacity_01p);
					break;
				case CHARGE_STATE_FULL_WAIT_STABLE:
					/* set to 0 first; the cournter will be add to 1 soon in CHARGE_STATE_FULL_WAIT_STABLE state handler*/
					count_charging_full_condition = 0;
					break;
			}

			printk(DRIVER_ZONE " state change(%d->%d), full count=%d, over load count=%d [%d]\n",
				poweralg.charge_state,
				next_charge_state,
				count_charging_full_condition,
				count_charge_over_load,
				poweralg.state_start_time_ms);

			poweralg.charge_state = next_charge_state;
			continue;
		}

		break;
	}
}

static void __update_capacity(void)
{
	INT32 next_capacity_01p;

	if (poweralg.charge_state == CHARGE_STATE_PREDICTION ||
		poweralg.charge_state == CHARGE_STATE_UNKNOWN){

		/*! star_lee 20100429 - return 99%~25% when in prediction mode*/
		poweralg.capacity_01p = max(min(990, poweralg.battery.KADC_01p), 250);
		printk(DRIVER_ZONE "fake percentage (%d) during prediction.\n",
			poweralg.capacity_01p);
	}
	else if (poweralg.charge_state == CHARGE_STATE_FULL_CHARGING ||
		poweralg.charge_state == CHARGE_STATE_FULL_RECHARGING ||
		poweralg.charge_state == CHARGE_STATE_FULL_PENDING){

		poweralg.capacity_01p = 1000;
	}
	else if (!is_charging_avaiable() && poweralg.is_voltage_stable){
		/* If battery voltage is critical-low, decrease level by 3 */
		if (poweralg.battery.voltage_mV < 3300) {
			poweralg.capacity_01p -= 30; /* cap% = cap% - 3% */
			if (poweralg.capacity_01p < 0)
				poweralg.capacity_01p = 0;
			battery_capacity_update(&poweralg.battery, poweralg.capacity_01p);
		}
		/* DISCHARGE ALG: capacity is based on KADC/RARC; only do this after cable in 3 minutes later*/
		else if (poweralg.battery.KADC_01p <= 0){
			if (poweralg.capacity_01p > 0)
				poweralg.capacity_01p -= 10;
			if (poweralg.capacity_01p > 0){
				/* capacity is still not 0 when KADC is 0; record capacity for next boot time*/
				battery_capacity_update(&poweralg.battery, poweralg.capacity_01p);
			}
		}
		else{
			if ((config.enable_weight_percentage) && (poweralg.capacity_01p <150 ||
				poweralg.battery.RARC_01p> poweralg.battery.KADC_01p)){
				INT32 w_kadc;
				INT32 w_rarc;
				INT32 Padc;
				INT32 Pw;
				if (0 <= poweralg.pdata->batt_param->padc)
					Padc = poweralg.pdata->batt_param->padc[poweralg.battery.id_index];
				else
					Padc = 200; /* default */
				if (0 <= poweralg.pdata->batt_param->pw)
					Pw = poweralg.pdata->batt_param->pw[poweralg.battery.id_index];
				else
					Pw = 5;	/* default */
/* 500=<W_KADC<=1000*/
/* w_kadc is a linear function of the abs difference between kadc and rarc
    w_kadc = f(|rarc-kadc|)
    let u = |rarc-kadc|
    f(u) is linear function ( Padc is the offset from x-axis, pw is the slope)
	=> w_kadc is proportinal to u. and  w_kadc > 0
	=> Padc (offset) must >= 0
	=> Pw (slope) >= 0 */
#define W_KADC(RARC, Percentage) Padc+(INT32)abs(RARC-Percentage)*Pw
				/*! star_lee 20100426 - W_KADC must be larger or equal to 0*/
				w_kadc = min(max(W_KADC(poweralg.battery.RARC_01p, poweralg.battery.KADC_01p), 0), 1000);
				w_rarc = 1000 - w_kadc;
				next_capacity_01p = (w_kadc * poweralg.battery.KADC_01p + w_rarc * poweralg.battery.RARC_01p)/1000;
			}
			else{
				next_capacity_01p = poweralg.battery.RARC_01p;
			}

			if (next_capacity_01p > 1000)
				next_capacity_01p = 1000;
			if (next_capacity_01p < 0)
				next_capacity_01p = 0;

			if (next_capacity_01p < poweralg.capacity_01p){
				poweralg.capacity_01p -= min(10, poweralg.capacity_01p-next_capacity_01p);
			}
		}

		if (config.enable_full_calibration){
			if (poweralg.is_need_calibrate_at_49p &&
				poweralg.capacity_01p <= 500 &&
				poweralg.fst_discharge_capacity_01p >= 600){

				poweralg.battery.charge_full_real_mAh = (poweralg.fst_discharge_acr_mAh-poweralg.battery.charge_counter_mAh)*1000/
					(poweralg.fst_discharge_capacity_01p-poweralg.capacity_01p);

				battery_capacity_update(&poweralg.battery, poweralg.capacity_01p);

				poweralg.is_need_calibrate_at_49p = FALSE;
				poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
				poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;

				printk(DRIVER_ZONE " 1.full calibrate: full=%d\n",
					poweralg.battery.charge_full_real_mAh);
			}
			else if (poweralg.is_need_calibrate_at_14p &&
				poweralg.capacity_01p <= 150 &&
				poweralg.fst_discharge_capacity_01p >= 250){
				poweralg.battery.charge_full_real_mAh = (poweralg.fst_discharge_acr_mAh-poweralg.battery.charge_counter_mAh)*1000/
					(poweralg.fst_discharge_capacity_01p - poweralg.capacity_01p);

				battery_capacity_update(&poweralg.battery, poweralg.capacity_01p);

				poweralg.is_need_calibrate_at_14p = FALSE;
				poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
				poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;

				printk(DRIVER_ZONE " 2.full calibrate: full=%d\n",
					poweralg.battery.charge_full_real_mAh);
			}
		}
	}
	else{
		/* CHARGE ALG: capacity is always based on ACR
		1. plus 1% as max when charge, if the orignal capacity is <= 99%, the result is no more than 99%
		2. minus 1% as max when discharge, not less than 0%*/
		next_capacity_01p = poweralg.battery.RARC_01p;

		if (next_capacity_01p > 1000)
			next_capacity_01p = 1000;
		if (next_capacity_01p < 0)
			next_capacity_01p = 0;

		if (next_capacity_01p > poweralg.capacity_01p){
			/* charge case*/
			next_capacity_01p = poweralg.capacity_01p + min(next_capacity_01p - poweralg.capacity_01p, 10);
			if (poweralg.capacity_01p > 990)
				poweralg.capacity_01p = next_capacity_01p;
			else
				poweralg.capacity_01p = min(next_capacity_01p, 990);
		}
		else if (next_capacity_01p < poweralg.capacity_01p){
			/* discharge case*/
			poweralg.capacity_01p -= min(poweralg.capacity_01p - next_capacity_01p, 10);
			if (poweralg.capacity_01p < 0)
				poweralg.capacity_01p = 0;
		}
	}
}

/*========================================================================================

HTC power algorithm implemetation

========================================================================================*/

int get_state_check_interval_min_sec(void)
{
	/*the minimal check interval of each states in seconds
	reserve for change polling rate
	UINT32 elapse_time_ms = BAHW_MyGetMSecs() - poweralg.state_start_time_ms;
	   switch (poweralg.charge_state)
	   {
	   case CHARGE_STATE_FULL_WAIT_STABLE:
		   //! star_lee 20100429 - takes 30 seconds(10 seconds*3 times) to confirm charge full condition
		   return 10;
	   case CHARGE_STATE_PREDICTION:
		   return min(config.predict_timeout_sec, max((INT32)(config.predict_timeout_sec - elapse_time_ms/1000), (INT32)1));
	   default:
		   if ( BAHW_IsChargeSourceIn() )  return config.polling_time_in_charging_sec;
		   else 						   return config.polling_time_in_discharging_sec;
	   }
	*/
	return 0;
}

BOOL do_power_alg(BOOL is_event_triggered)
{
	/* is_event_triggered - TRUE: handle event only, do not update capacity; FALSE; always update capacity*/
	static BOOL s_bFirstEntry = TRUE;
	static UINT32 s_pre_time_ms;
	static INT32 s_level;

	UINT32 now_time_ms = BAHW_MyGetMSecs();
#if !HTC_BATTERY_DS2746_DEBUG_ENABLE
	BOOL show_debug_message = FALSE;
#endif
	/*printk(DRIVER_ZONE "%s(%d) {\n",__func__, is_event_triggered);*/
	/*------------------------------------------------------
	1 get battery data and update charge state*/
	if (!battery_param_update(&poweralg.battery, &poweralg.protect_flags)){
		printk(DRIVER_ZONE "battery_param_update fail, please retry next time.\n");
		return FALSE;
	}

	update_next_charge_state();

	/*-----------------------------------------------------
	2 calculate battery capacity (predict if necessary)*/
	if (s_bFirstEntry || now_time_ms - s_pre_time_ms > 10000 || !is_event_triggered){
		/* DO not update capacity when plug/unplug cable less than 10 seconds*/
		__update_capacity();

		s_bFirstEntry = FALSE;
		s_pre_time_ms = now_time_ms;
	}

	if (config.debug_disable_shutdown){
		if (poweralg.capacity_01p <= 0){
			poweralg.capacity_01p = 1;
		}
	}

	s_level = CEILING(poweralg.capacity_01p, 10);
	if (CEILING(poweralg.last_capacity_01p, 10) != s_level ||
		poweralg.battery.last_temp_01c != poweralg.battery.temp_01c) {

		poweralg.battery.last_temp_01c = poweralg.battery.temp_01c;
		poweralg.last_capacity_01p = poweralg.capacity_01p;
		ds2746_blocking_notify(DS2746_LEVEL_UPDATE, &s_level);
#if !HTC_BATTERY_DS2746_DEBUG_ENABLE
		show_debug_message = TRUE;
#endif
	}

	bounding_fullly_charged_level(config.full_level);

	/* is_superchg_software_charger_timeout: only triggered when superAC adapter in*/
	if (config.superchg_software_charger_timeout_sec && poweralg.is_super_ac
		&& FALSE==poweralg.is_superchg_software_charger_timeout){
		super_chg_on_time_sec += delta_time_sec;
		if (config.superchg_software_charger_timeout_sec <= super_chg_on_time_sec){
			printk(DRIVER_ZONE "superchg charger on timer timeout: %u sec\n",
				super_chg_on_time_sec);
			poweralg.is_superchg_software_charger_timeout = TRUE;
		}
	}
	/*------------------------------------------------------
	3 charging function change*/
	if (is_charging_avaiable()){
		/* Software should also toggle MCHG_EN within 4 hrs
			to prevent charger HW safety timer expired. */
		if (config.charger_hw_safety_timer_watchdog_sec){
			chg_en_time_sec += delta_time_sec;
			if (config.charger_hw_safety_timer_watchdog_sec <= chg_en_time_sec) {
				printk(DRIVER_ZONE "need software toggle charger: lasts %d sec\n", chg_en_time_sec);
				chg_en_time_sec = 0;
				ds2746_charger_control(DISABLE);
				udelay(200);
			}
		}

		if (is_high_current_charging_avaialable()){
			if (is_super_current_charging_avaialable()) {
				ds2746_charger_control(ENABLE_SUPER_CHG);
			} else {
				ds2746_charger_control(ENABLE_FAST_CHG);
			}
		} else {
			ds2746_charger_control(ENABLE_SLOW_CHG);
		}

		/* EXPRESS only: control charger IC BQ24170 minimum taper current */
		// TODO: use a state variable here: set when only state changes
		if ((config.min_taper_current_ma > 0)) {
			if (is_set_min_taper_current()) {
				ds2746_charger_control(ENABLE_MIN_TAPER);
			} else {
				ds2746_charger_control(DISABLE_MIN_TAPER);
			}
		}
	} else {
		ds2746_charger_control(DISABLE);
		chg_en_time_sec = 0;
		super_chg_on_time_sec = 0;
		poweralg.is_need_toggle_charger = FALSE;
	}

	if (config.debug_disable_hw_timer && poweralg.is_charge_over_load){
		ds2746_charger_control(DISABLE);
		printk(DRIVER_ZONE "Toggle charger due to HW disable charger.\n");
	}

	/*------------------------------------------------------
	 4 debug messages and update os battery status*/

	/*powerlog_to_file(&poweralg);
	update_os_batt_status(&poweralg);*/

#if HTC_BATTERY_DS2746_DEBUG_ENABLE
	printk(DRIVER_ZONE "S=%d P=%d chg=%d cable=%d%d%d flg=%d%d%d dbg=%d%d%d%d fst_dischg=%d/%d [%u]\n",
		poweralg.charge_state,
		poweralg.capacity_01p,
		poweralg.charging_enable,
		poweralg.is_cable_in,
		poweralg.is_china_ac_in,
		poweralg.is_super_ac,
		poweralg.protect_flags.is_charging_enable_available,
		poweralg.protect_flags.is_charging_high_current_avaialble,
		poweralg.protect_flags.is_battery_dead,
		config.debug_disable_shutdown,
		config.debug_fake_room_temp,
		config.debug_disable_hw_timer,
		config.debug_always_predict,
		poweralg.fst_discharge_capacity_01p,
		poweralg.fst_discharge_acr_mAh,
		BAHW_MyGetMSecs());
#else
	if (show_debug_message == TRUE)
		printk(DRIVER_ZONE "P=%d V=%d T=%d I=%d ACR=%d/%d KADC=%d charger=%d%d \n",
			poweralg.capacity_01p,
			poweralg.battery.voltage_mV,
			poweralg.battery.temp_01c,
			poweralg.battery.current_mA,
			poweralg.battery.charge_counter_mAh,
			poweralg.battery.charge_full_real_mAh,
			poweralg.battery.KADC_01p,
			poweralg.charging_source,
			poweralg.charging_enable);
#endif

	/*printk(DRIVER_ZONE "};\n");*/
	return TRUE;
}

void power_alg_init(struct poweralg_config_type *debug_config)
{
	/*-------------------------------------------------------------
	1. setup default poweralg data*/
	poweralg.charge_state = CHARGE_STATE_UNKNOWN;
	poweralg.capacity_01p = 990;
	poweralg.last_capacity_01p = poweralg.capacity_01p;
	poweralg.fst_discharge_capacity_01p = 0;
	poweralg.fst_discharge_acr_mAh = 0;
	poweralg.is_need_calibrate_at_49p = TRUE;
	poweralg.is_need_calibrate_at_14p = TRUE;
	poweralg.is_charge_over_load = FALSE;
	poweralg.is_cable_in = FALSE;
	poweralg.is_china_ac_in = FALSE;
	poweralg.is_super_ac = FALSE;
	poweralg.is_voltage_stable = FALSE;
	poweralg.is_software_charger_timeout = FALSE;
	poweralg.is_superchg_software_charger_timeout = FALSE;
	poweralg.is_need_toggle_charger = FALSE;
	poweralg.last_charger_enable_toggled_time_ms = 0;
	poweralg.state_start_time_ms = 0;

	if(get_cable_status() == CONNECT_TYPE_USB) {
		poweralg.is_cable_in = TRUE;
		poweralg.charging_source = CONNECT_TYPE_USB;
		ds2746_charger_control(ENABLE_SLOW_CHG);
	}
	else if (get_cable_status() == CONNECT_TYPE_AC) {
		poweralg.is_cable_in = TRUE;
		poweralg.is_china_ac_in = TRUE;
		poweralg.charging_source = CONNECT_TYPE_AC;
		ds2746_charger_control(ENABLE_FAST_CHG);
	}
	else if (get_cable_status() == CONNECT_TYPE_9V_AC) {
		poweralg.is_cable_in = TRUE;
		poweralg.is_china_ac_in = TRUE;
		poweralg.is_super_ac = TRUE;
		poweralg.charging_source = CONNECT_TYPE_9V_AC;
		ds2746_charger_control(ENABLE_SUPER_CHG);
	} else {
		poweralg.charging_source = CONNECT_TYPE_NONE;
		ds2746_charger_control(DISABLE);
	}
	/*-------------------------------------------------------------
	2. setup default config flags (board dependent)*/
	if (poweralg.pdata && poweralg.pdata->func_poweralg_config_init)
		poweralg.pdata->func_poweralg_config_init(&config);
	else
		poweralg_config_init(&config);

	if (debug_config){
		config.debug_disable_shutdown = debug_config->debug_disable_shutdown;
		config.debug_fake_room_temp = debug_config->debug_fake_room_temp;
		config.debug_disable_hw_timer = debug_config->debug_disable_hw_timer;
		config.debug_always_predict = debug_config->debug_always_predict;
	}

	/* if ( BAHW_IsTestMode() )
	 {
		 config.debug_disable_shutdown = TRUE;
		 config.debug_fake_room_temp   = TRUE;
		 config.debug_disable_hw_timer = TRUE;
	 }*/

	/*-------------------------------------------------------------
	3. setup default protect flags*/
	poweralg.protect_flags.is_charging_enable_available = TRUE;
	poweralg.protect_flags.is_battery_dead = FALSE;
	poweralg.protect_flags.is_charging_high_current_avaialble = FALSE;
	poweralg.protect_flags.is_fake_room_temp = config.debug_fake_room_temp;
	poweralg.protect_flags.func_update_charging_protect_flag = NULL;

	/*-------------------------------------------------------------
	4. setup default battery structure*/
	battery_param_init(&poweralg.battery);

	/*pr_info("power alg inited with board name <%s>\n", HTC_BATT_BOARD_NAME);*/
}

void power_alg_preinit(void)
{
	/* make sure cable and battery is in when off mode charging*/
}

static BLOCKING_NOTIFIER_HEAD(ds2746_notifier_list);
int ds2746_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ds2746_notifier_list, nb);
}

int ds2746_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ds2746_notifier_list, nb);
}


int ds2746_blocking_notify(unsigned long val, void *v)
{
	int chg_ctl;

	if (val == DS2746_CHARGING_CONTROL){
		chg_ctl = *(int *) v;
		if (machine_is_passionc()){
			if (chg_ctl <= 2){
				gpio_direction_output(22, !(!!chg_ctl));/*PNC*/
				set_charger_ctrl(chg_ctl);
			}
			return 0;
		}else if (poweralg.battery.id_index != BATTERY_ID_UNKNOWN && (TOGGLE_CHARGER == chg_ctl || ENABLE_MIN_TAPER == chg_ctl || DISABLE_MIN_TAPER == chg_ctl)) {
			if (0 == poweralg.charging_enable)
				return 0;
		} else if (poweralg.battery.id_index != BATTERY_ID_UNKNOWN && poweralg.charge_state != CHARGE_STATE_PREDICTION) {
			/* only notify at changes */
			if (g_first_update_charger_ctl == 1) {
				printk(DRIVER_ZONE "first update charger control forcely.\n");
				g_first_update_charger_ctl = 0;
				poweralg.charging_enable = chg_ctl;
			} else if (poweralg.charging_enable == chg_ctl)
				return 0;
			else
				poweralg.charging_enable = chg_ctl;
		} else {
			if (poweralg.charging_enable == DISABLE) {
				return 0;
			} else {
				poweralg.charging_enable = DISABLE;
				v = DISABLE;
			}
			printk(DRIVER_ZONE "Charging disable due to Unknown battery\n");
		}
	}
	return blocking_notifier_call_chain(&ds2746_notifier_list, val, v);
}


int ds2746_get_battery_info(struct battery_info_reply *batt_info)
{
	batt_info->batt_id = poweralg.battery.id_index; /*Mbat ID*/
	batt_info->batt_vol = poweralg.battery.voltage_mV; /*VMbat*/
	batt_info->batt_temp = poweralg.battery.temp_01c; /*Temperature*/
	batt_info->batt_current = poweralg.battery.current_mA; /*Current*/
	batt_info->level = CEILING(poweralg.capacity_01p, 10); /*last_show%*/
	batt_info->charging_source = poweralg.charging_source;
	batt_info->charging_enabled = poweralg.charging_enable;
	batt_info->full_bat = poweralg.battery.charge_full_real_mAh;
	return 0;
}
ssize_t htc_battery_show_attr(struct device_attribute *attr, char *buf)
{
	int len = 0;
	if (!strcmp(attr->attr.name, "batt_attr_text")){
		len += scnprintf(buf +
				len,
				PAGE_SIZE -
				len,
				"Percentage(%%): %d;\n"
				"KADC(%%): %d;\n"
				"RARC(%%): %d;\n"
				"V_MBAT(mV): %d;\n"
				"Battery_ID: %d;\n"
				"pd_M: %d;\n"
				"Current(mA): %d;\n"
				"Temp: %d;\n"
				"Charging_source: %d;\n"
				"ACR(mAh): %d;\n"
				"FULL(mAh): %d;\n"
				"1st_dis_percentage(%%): %d;\n"
				"1st_dis_ACR: %d;\n",
				CEILING(poweralg.capacity_01p, 10),
				CEILING(poweralg.battery.KADC_01p, 10),
				CEILING(poweralg.battery.RARC_01p, 10),
				poweralg.battery.voltage_mV,
				poweralg.battery.id_index,
				poweralg.battery.pd_m,
				poweralg.battery.current_mA,
				CEILING(poweralg.battery.temp_01c, 10),
				poweralg.charging_source,
				poweralg.battery.charge_counter_mAh,
				poweralg.battery.charge_full_real_mAh,
				CEILING(poweralg.fst_discharge_capacity_01p, 10),
				poweralg.fst_discharge_acr_mAh
		);
	}
	return len;
}

static void ds2746_program_alarm(struct ds2746_device_info *di, int seconds)
{
	ktime_t low_interval = ktime_set(seconds, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	next = ktime_add(di->last_poll, low_interval);

	delta_time_sec = seconds;
	alarm_start_range(&di->alarm, next, ktime_add(next, slack));
}

static int cable_status_handler_func(struct notifier_block *nfb,
	unsigned long action, void *param)
{
	u32 cable_type = (u32) action;
	/* When the cable plug out, reset all the related flag,
	Let algorithm machine to judge latest state */
	if (cable_type == CONNECT_TYPE_NONE) {
		poweralg.is_cable_in = 0;
		poweralg.is_china_ac_in = 0;
		poweralg.is_super_ac = 0;
		poweralg.charging_source = cable_type;
		chg_en_time_sec = super_chg_on_time_sec = delta_time_sec = 0;
		if (TRUE == poweralg.is_superchg_software_charger_timeout) {
			poweralg.is_superchg_software_charger_timeout = FALSE;	/* reset */
			printk(DRIVER_ZONE "reset superchg software timer\n");
		}
		ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &poweralg.charging_source);
	} else if (cable_type == CONNECT_TYPE_USB) {
		poweralg.is_cable_in = 1;
		poweralg.is_china_ac_in = 0;
		poweralg.is_super_ac = 0;
		poweralg.charging_source = cable_type;
		chg_en_time_sec = super_chg_on_time_sec = delta_time_sec = 0;
		//ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &poweralg.charging_source);
		if (g_di_ptr) {
			alarm_try_to_cancel(&g_di_ptr->alarm);
			ds2746_program_alarm(g_di_ptr, 0);
		}
		else {
			printk(DRIVER_ZONE "charger in but no di ptr.\n");
		}
	} else if (cable_type == CONNECT_TYPE_AC) {
		poweralg.is_cable_in = 1;
		poweralg.is_china_ac_in = 1;
		poweralg.is_super_ac = 0;
		poweralg.charging_source = cable_type;
		chg_en_time_sec = super_chg_on_time_sec = delta_time_sec = 0;
		//ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &poweralg.charging_source);
		if (g_di_ptr) {
			alarm_try_to_cancel(&g_di_ptr->alarm);
			ds2746_program_alarm(g_di_ptr, 0);
		}
		else {
			printk(DRIVER_ZONE "charger in but no di ptr.\n");
		}
	} else if (cable_type == CONNECT_TYPE_9V_AC) {
		poweralg.is_cable_in = 1;
		poweralg.is_china_ac_in = 1;
		poweralg.is_super_ac = 1;
		poweralg.charging_source = cable_type;
		chg_en_time_sec = super_chg_on_time_sec = delta_time_sec = 0;
		//ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &poweralg.charging_source);
		if (g_di_ptr) {
			alarm_try_to_cancel(&g_di_ptr->alarm);
			ds2746_program_alarm(g_di_ptr, 0);
		}
		else {
			printk(DRIVER_ZONE "charger in but no di ptr.\n");
		}
	} else if (cable_type == 0xff) {
		if (param)
			config.full_level = *(INT32 *)param;
		printk(DRIVER_ZONE "Set the full level to %d\n", config.full_level);
		return NOTIFY_OK;
	} else if (cable_type == 0x10) {
		poweralg.protect_flags.is_fake_room_temp = TRUE;
		printk(DRIVER_ZONE "enable fake temp mode\n");
		return NOTIFY_OK;
	}

	return NOTIFY_OK;
}

static struct notifier_block cable_status_handler =
{
  .notifier_call = cable_status_handler_func,
};

void ds2746_charger_control(int type)
{
	int charge_type = type;
	/* printk(DRIVER_ZONE "%s(%d)\n",__func__, type); */

	switch (charge_type){
		case DISABLE:
			/* CHARGER_EN is active low.  Set to 1 to disable. */
			ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &charge_type);
			break;
		case ENABLE_SLOW_CHG:
			ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &charge_type);
			break;
		case ENABLE_FAST_CHG:
			ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &charge_type);
			break;
		case ENABLE_SUPER_CHG:
			ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &charge_type);
			break;
		case TOGGLE_CHARGER:
			ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &charge_type);
			break;
		case ENABLE_MIN_TAPER:
			ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &charge_type);
			break;
		case DISABLE_MIN_TAPER:
			ds2746_blocking_notify(DS2746_CHARGING_CONTROL, &charge_type);
			break;
	}
}


static void ds2746_battery_work(struct work_struct *work)
{
	struct ds2746_device_info *di = container_of(work,
				struct ds2746_device_info, monitor_work);
	unsigned long flags;

	do_power_alg(0);
	get_state_check_interval_min_sec();
	di->last_poll = alarm_get_elapsed_realtime();

	/* prevent suspend before starting the alarm */
	local_irq_save(flags);

	wake_unlock(&di->work_wake_lock);
	if (poweralg.battery.is_power_on_reset)
		ds2746_program_alarm(di, PREDIC_POLL);
	else
		ds2746_program_alarm(di, FAST_POLL);

	local_irq_restore(flags);
}

static void ds2746_battery_alarm(struct alarm *alarm)
{
	struct ds2746_device_info *di = container_of(alarm, struct ds2746_device_info, alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
}

static int ds2746_battery_probe(struct platform_device *pdev)
{
	int rc;
	struct ds2746_device_info *di;
	ds2746_platform_data *pdata = pdev->dev.platform_data; // MATT: wait to remove

	poweralg.pdata = pdev->dev.platform_data;
	poweralg.battery.thermal_id = pdata->func_get_thermal_id();
	/* if func_get_battery_id is Null or
		it returns negative value => we need id detection */
	if ((pdata->func_get_battery_id != NULL) && (0 < pdata->func_get_battery_id())) {
		poweralg.battery.id_index = pdata->func_get_battery_id();
		if (NULL != poweralg.pdata->batt_param->fl_25) {
			poweralg.battery.charge_full_design_mAh =
				poweralg.pdata->batt_param->fl_25[poweralg.battery.id_index];
		} else
			poweralg.battery.charge_full_design_mAh = DS2746_FULL_CAPACITY_DEFAULT;
		poweralg.battery.charge_full_real_mAh = poweralg.battery.charge_full_design_mAh;
		is_need_battery_id_detection = FALSE;
	}
	else {
		poweralg.battery.id_index = BATTERY_ID_UNKNOWN;
		is_need_battery_id_detection = TRUE;
	}

	power_alg_preinit();
	power_alg_init(&debug_config);
	// must set func hook after power_alg_init().
	poweralg.protect_flags.func_update_charging_protect_flag = pdata->func_update_charging_protect_flag;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di){
		rc = -ENOMEM;
		goto fail_register;
	}

	di->update_time = jiffies;
	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;

	INIT_WORK(&di->monitor_work, ds2746_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));

	/* init to something sane */
	di->last_poll = alarm_get_elapsed_realtime();

	if (!di->monitor_wqueue){
		rc = -ESRCH;
		goto fail_workqueue;
	}
	wake_lock_init(&di->work_wake_lock, WAKE_LOCK_SUSPEND, "ds2746-battery");
	alarm_init(&di->alarm,
		ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
		ds2746_battery_alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);

	g_di_ptr = di; /* save di to global */
	return 0;

	fail_workqueue : fail_register : kfree(di);
	return rc;
}


static int ds2746_battery_remove(struct platform_device *pdev)
{
	struct ds2746_device_info *di = platform_get_drvdata(pdev);

	cancel_work_sync(&di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);

	return 0;
}

/* FIXME: power down DQ master when not in use. */
static int ds2746_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ds2746_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;
	/* If we are on battery, reduce our update rate until
	 * we next resume.*/
	if (poweralg.charging_source == CONNECT_TYPE_NONE) {
		local_irq_save(flags);
		ds2746_program_alarm(di, SLOW_POLL);
		di->slow_poll = 1;
		local_irq_restore(flags);
	}
	/*gpio_direction_output(87, 0);*/
	return 0;
}
static void ds2746_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ds2746_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;

	/* We might be on a slow sample cycle.  If we're
	 * resuming we should resample the battery state
	 * if it's been over a minute since we last did
	 * so, and move back to sampling every minute until
	 * we suspend again.*/
	/*gpio_direction_output(87, 1);*/
	ndelay(100 * 1000);
	if (di->slow_poll){
		local_irq_save(flags);
		ds2746_program_alarm(di, FAST_POLL);
		di->slow_poll = 0;
		local_irq_restore(flags);
	}
}

static struct dev_pm_ops ds2746_pm_ops = {
       .prepare = ds2746_suspend,
       .complete  = ds2746_resume,
};

MODULE_ALIAS("platform:ds2746-battery");
static struct platform_driver ds2746_battery_driver =
{
	.driver = {
	.name = "ds2746-battery",
	.pm = &ds2746_pm_ops,
	},
	.probe = ds2746_battery_probe,
	.remove = ds2746_battery_remove,
};

static int __init ds2746_battery_init(void)
{
	int ret;

	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	register_notifier_cable_status(&cable_status_handler);

	ret = ds2746_i2c_init();
	if (ret < 0){
		return ret;
	}

	/*mutex_init(&htc_batt_info.lock);*/
	return platform_driver_register(&ds2746_battery_driver);
}

static void __exit ds2746_battery_exit(void)
{
	ds2746_i2c_exit();
	platform_driver_unregister(&ds2746_battery_driver);
}

module_init(ds2746_battery_init);
module_exit(ds2746_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy.YS Wang  <Andy.ys_wang@htc.com>");
MODULE_DESCRIPTION("ds2746 battery driver");

