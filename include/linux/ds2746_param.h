#ifndef __BATT_PARAM_H__
#define __BATT_PARAM_H__

/* battery status and charging information*/

struct battery_type{

		BOOL is_power_on_reset;

		INT32 voltage_mV;
		INT32 current_mA;
		INT32 discharge_mA;
		INT32 charge_counter_mAh;
		INT32 temp_01c;
		INT32 last_temp_01c;
		INT32 id_ohm;
		INT32 vref_mv;

		INT32 voltage_adc;
		INT32 current_adc;
		INT32 discharge_adc;
		INT32 charge_counter_adc;
		INT32 temp_adc;
		INT32 last_temp_adc;
		INT32 id_adc;
		INT32 vref_adc;

		INT32 id_index;
		INT32 charge_full_design_mAh;
		INT32 charge_full_real_mAh;

		INT32 temp_index;
		INT32 temp_check_index;

		INT32 KADC_01p;
		INT32 RARC_01p;
		INT32 pd_m;

		INT32 software_charge_counter_mAms;
		INT32 thermal_id;
};

struct protect_flags_type{

		BOOL is_charging_enable_available;
		BOOL is_charging_high_current_avaialble;
		BOOL is_charging_indicator_available;
		BOOL is_battery_dead;
#if 0
		BOOL is_battery_overtemp;
#endif
		BOOL is_fake_room_temp;
		int (*func_update_charging_protect_flag)(int, int, int, BOOL*, BOOL*);
};

/* ds2746 register definition*/

#define DS2746_STATUS_PORF  (1 << 6)	/* write to 0 as power-up sequence ready*/
#define DS2746_STATUS_SMOD  (1 << 5)	/* write to 0 to disable DS2746 sleep mode*/
#define DS2746_STATUS_NBEN  (1 << 4)	/* write to 0 to disable blanking of negative currents*/
#define DS2746_STATUS_AIN0  (1 << 0)
#define DS2746_STATUS_AIN1  (1 << 1)

/* function prototypes*/

void battery_capacity_update(struct battery_type *battery, int capacity_01p);
BOOL battery_param_update(struct battery_type *battery, struct protect_flags_type *flags);
DWORD BAHW_MyGetMSecs(void);
void battery_param_init(struct battery_type *battery);

#endif /* __BATT_PARAM_H__*/
