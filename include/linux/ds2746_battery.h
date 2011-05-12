/*
 * Copyright (C) 2007 HTC Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _DS2746_BATTERY_H_
#define _DS2746_BATTERY_H_
#include <linux/notifier.h>
#include <mach/htc_battery.h>
#include <linux/wrapper_types.h>
#include <linux/ds2746_param.h>


enum ds2746_notify_evt_t {
	DS2746_CHARGING_CONTROL = 0,
	DS2746_LEVEL_UPDATE,
};

/* battery charging state*/

enum {
	CHARGE_STATE_UNKNOWN,   		/* before anything is ready, we are in this state. shall default low current charge and show charging LED*/
	CHARGE_STATE_PREDICTION,		/* in normal case, we need to enter prediction for 10 seconds for 1st KADC*/
	CHARGE_STATE_DISCHARGE, 		/* cable out state*/
	CHARGE_STATE_CHARGING,  		/* charging state*/
	CHARGE_STATE_PENDING,   		/* charging state but no good*/
	CHARGE_STATE_FULL_WAIT_STABLE,  /* charging state but going full*/
	CHARGE_STATE_FULL_CHARGING, 	/* charging full, keep charging*/
	CHARGE_STATE_FULL_PENDING,  	/* charging full, stop charging*/
	CHARGE_STATE_FULL_RECHARGING,  	/* charging full, recharging*/
};

enum {
	THERMAL_300_100_4360,
	THERMAL_300_47_3440,
	THERMAL_1000_100_4360,
};

/* MATT TODO: remove after integration */
enum BATTERY_ID_ENUM {
	BATTERY_ID_UNKNOWN = 0,
	BATTERY_ID_SAMSUNG_1230MAH,
	BATTERY_ID_LG_3260MAH,
	BATTERY_ID_ATL_4000MAH,
	BATTERY_ID_NUM, /* include unknown battery*/
};

/* power algorithm data structure and config data structure*/

typedef struct _ds2746_platform_data ds2746_platform_data;

struct poweralg_type
{
	int charge_state;
	int capacity_01p;
	int last_capacity_01p;
	int fst_discharge_capacity_01p;
	int fst_discharge_acr_mAh;
	int charging_source;
	int charging_enable;
	BOOL is_need_calibrate_at_49p;
	BOOL is_need_calibrate_at_14p;
	BOOL is_charge_over_load;
	struct battery_type battery;
	struct protect_flags_type protect_flags;
	BOOL is_china_ac_in;
	BOOL is_super_ac;
	BOOL is_cable_in;
	BOOL is_voltage_stable;
	BOOL is_software_charger_timeout;
	BOOL is_superchg_software_charger_timeout;
	UINT32 state_start_time_ms;
	UINT32 last_charger_enable_toggled_time_ms;
	BOOL is_need_toggle_charger;
	ds2746_platform_data* pdata;
};

struct poweralg_config_type
{
	INT32 full_charging_mv;
	INT32 full_charging_ma;
	INT32 full_pending_ma;			/* 0 to disable*/
	INT32 full_charging_timeout_sec;	 /* 0 to disable*/
	INT32 voltage_recharge_mv;  		 /* 0 to disable*/
	INT32 capacity_recharge_p;  		 /* 0 to disable*/
	INT32 voltage_exit_full_mv; 		 /* 0 to disable*/
	INT32 min_taper_current_mv;		 /* 0 to disable*/
	INT32 min_taper_current_ma; 		 /* 0 to disable*/
	INT32 wait_votlage_statble_sec;
	INT32 predict_timeout_sec;
	INT32 polling_time_in_charging_sec;
	INT32 polling_time_in_discharging_sec;

	BOOL enable_full_calibration;
	BOOL enable_weight_percentage;
	INT32 software_charger_timeout_sec;  /* 0 to disable*/ /* for china AC */
	INT32 superchg_software_charger_timeout_sec;  /* 0 to disable*/ /* for superchg */
	INT32 charger_hw_safety_timer_watchdog_sec;  /* 0 to disable*/

	BOOL debug_disable_shutdown;
	BOOL debug_fake_room_temp;
	BOOL debug_disable_hw_timer;
	BOOL debug_always_predict;
	INT32 full_level;                  /* 0 to disable*/
};

struct battery_parameter {
	UINT32* fl_25;
	UINT32** pd_m_coef_tbl_boot;	/* selected by temp_index */
	UINT32** pd_m_coef_tbl;		/* selected by temp_index */
	UINT32** pd_m_resl_tbl_boot;	/* selected by temp_index */
	UINT32** pd_m_resl_tbl;		/* selected by temp_index */
	UINT32* capacity_deduction_tbl_01p;	/* selected by temp_index */
	UINT32* pd_t_coef;
	INT32* padc;	/* less than 0: disable */
	INT32* pw;	/* less than 0: disable */
	UINT32* id_tbl;
	INT32* temp_index_tbl;	/* NULL: disable */
	UINT32** m_param_tbl;
	int m_param_tbl_size;
};

struct _ds2746_platform_data {
	struct battery_parameter* batt_param;
	int (*func_get_thermal_id)(void);
	int (*func_get_battery_id)(void);
	void (*func_poweralg_config_init)(struct poweralg_config_type*);
	int (*func_update_charging_protect_flag)(int, int, int, BOOL*, BOOL*);
	int r2_kohm;
	//UINT32* id_tbl;
};


#define DS2746_FULL_CAPACITY_DEFAULT	(999)

/* battery behavior constant*/

#define BATTERY_PERCENTAGE_UNKNOWN  0xFF
#define BATTERY_LOW_PERCENTAGE	    10  	/* in 1%*/
#define BATTERY_CRITICAL_PERCENTAGE 5   	/* in 1%*/
#define BATTERY_EMPTY_PERCENTAGE    0   	/* in 1%*/

/* battery algorithm public functions*/

int get_state_check_interval_min_sec( void);
BOOL do_power_alg( BOOL is_event_triggered);
void power_alg_init( struct poweralg_config_type *debug_config);
void power_alg_preinit( void);
int ds2746_blocking_notify( unsigned long val, void *v);
void ds2746_charger_control( int type);
int ds2746_i2c_write_u8( u8 value, u8 reg);
int ds2746_i2c_read_u8( u8* value, u8 reg);
int ds2746_battery_id_adc_2_ohm(int id_adc, int r2_kohm);
void calibrate_id_ohm( struct battery_type *battery);
/* external function implemented by upper layer*/

/*extern void powerlog_to_file(struct poweralg_type* poweralg);*/
/*extern void update_os_batt_status(struct poweralg_type* poweralg);*/

#ifdef CONFIG_BATTERY_DS2746
extern int ds2746_register_notifier( struct notifier_block *nb);
extern int ds2746_unregister_notifier( struct notifier_block *nb);
extern int ds2746_get_battery_info( struct battery_info_reply *batt_info);
extern ssize_t htc_battery_show_attr( struct device_attribute *attr, char *buf);
#else
static int ds2746_register_notifier( struct notifier_block *nb) {

	return 0;
}
static int ds2746_unregister_notifier( struct notifier_block *nb) {

	return 0;
}
static int ds2746_get_battery_info( struct battery_info_reply *batt_info) {

	batt_info->level = 10;
	return 0;
}
extern ssize_t htc_battery_show_attr( struct device_attribute *attr, char *buf) {

	return 0;
}

#endif

#endif
