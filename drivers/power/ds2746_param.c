/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Copyright (c) 2010 High Tech Computer Corporation

Module Name:

		ds2784_param.c

Abstract:

		This module implements the battery formula based on power spec, including below concepts:
		1. adc converter
		2. voltage mapping to capacity
		3. over temperature algorithm
		4. id range algorithm
		5. ACR maintainance

		Add from TPE PMA:
		1. temperature index
		2. pd_m_coef_boot
		3. preserved_capacity_by_temp
		Remove from TAO PMA:
		1. pd_temp

		To adapt different PMA/projects, we need to modify below tables:
		1. ID_RANGE: which battery is used in the project?
		2. FL_25: the full capacity in temp 25C.
		3. pd_m_bias_mA: the discharge current threshold to calculating pd_m
		4. M_PARAMTER_TABLE: the voltage-capacity mapping table
		5. TEMP_RANGE: how many temp condition we need to consider
		6. PD_M_COEF_TABLE(BOOT)/PD_M_RESL_TABLE(BOOT): voltage compensation based on current
		7. PD_T_COEF: voltage compensation based on temp
		8. CAPACITY_DEDUCTION_01p: the capacity deduction due to low temperature

Original Auther:

		Andy.ys Wang June-01-2010

---------------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/ds2746_battery.h>
#include <linux/ds2746_param.h>
#include <linux/ds2746_param_config.h>
#include <linux/wrapper_types.h>
#include <linux/time.h>
#include <asm/setup.h>

/*========================================================================================

build flags

========================================================================================*/

#define HTC_ENABLE_POWER_DEBUG  		0
#define HTC_ENABLE_DUMMY_BATTERY		0

/*========================================================================================

battery common parameter defines (independent on battery id yet...)

========================================================================================*/

#define BATTERY_VOLTAGE_MIN 2000
#define BATTERY_VOLTAGE_MAX 20000

/*========================================================================================

battery parameter helper functions

========================================================================================*/

static INT32 get_id_index(struct battery_type *battery)
{
	int i;

	for (i = 0; i < BATTERY_ID_NUM - 1; i++) {
		/* minus 1, unknown battery is not in ID_RANGE
		[min, max)*/
		UINT32 resister_min = ID_RANGE[i*2];
		UINT32 resister_max = ID_RANGE[i*2 + 1];

		if (resister_min <= battery->id_ohm && resister_max > battery->id_ohm) {
			return i + 1;
		}
	}

	return BATTERY_ID_UNKNOWN;
}

static INT32 get_temp_index(struct battery_type *battery)
{
	int i;

	for (i = 0; i < 255; i++) {
		/* the table size shall not be greater then 255, to ensure no infinite looping
		[min, max)*/
		INT32 temp = TEMP_RANGE[i];
		if (battery->temp_01c >= temp)
			return i;
	}

	printk(DRIVER_ZONE " invalid batt_temp (%d) or temp range mapping.\n", battery->temp_01c);
	return -1;
}

/*========================================================================================

temperature formula definitions

========================================================================================*/

static INT32 get_temp_01c(struct battery_type *battery)
{
	int current_index = battery->temp_check_index;
	int search_direction = 0;

	if (battery->last_temp_adc > battery->temp_adc) {
		search_direction = -1;
	} else {
		search_direction = 1;
	}

	while (current_index >= 0 && current_index < TEMP_NUM-1) {

		UINT32 temp_min = TEMP_MAP[current_index];
		UINT32 temp_max = TEMP_MAP[current_index + 1];

		if (temp_max > battery->temp_adc && temp_min <= battery->temp_adc) {
			battery->temp_check_index = current_index;
			battery->last_temp_adc = battery->temp_adc;
			return (TEMP_MAX-current_index)*10;
		}
		current_index += search_direction;
	}

	return (TEMP_MIN-1)*10;
}

/*========================================================================================

over temperature protection

========================================================================================*/

static BOOL is_over_temp(struct battery_type *battery)
{
	/* stop charging*/
	if (battery->temp_01c < over_low_temp_lock_01c || battery->temp_01c >= over_high_temp_lock_01c) {
		return TRUE;
	}

	return FALSE;
}

static BOOL is_not_over_temp(struct battery_type *battery)
{
	/* start charging*/
	if (battery->temp_01c >= over_low_temp_release_01c &&
		battery->temp_01c < over_high_temp_release_01c) {
		return TRUE;
	}

	return FALSE;
}

static void __protect_flags_update(struct battery_type *battery,
	struct protect_flags_type *flags)
{
	/* Flags:
	is_charging_enable_available		- Over temperature, need to stop charging
	is_charging_high_current_avaialble	- Temperature is too high so that we have to slow charge*/

	if (is_over_temp(battery)) {
		/* Ex: T<0 or T>45 */
		flags->is_charging_enable_available = FALSE;
		flags->is_charging_high_current_avaialble = FALSE;
#if 0
		flags->is_battery_overtemp = TRUE;
#endif
	} else if (is_not_over_temp(battery)) {
		/* Ex: T<42 or T>3*/
		flags->is_charging_enable_available = TRUE;
		flags->is_charging_high_current_avaialble = TRUE;
#if 0
		flags->is_battery_overtemp = FALSE;
#endif
	}

	/* Flags:
	is_battery_dead			- If battery is dead, show special indicator for it*/
	if (battery->voltage_mV < BATTERY_DEAD_VOLTAGE_LEVEL) {
		flags->is_battery_dead = TRUE;
	}
	else if (battery->voltage_mV > BATTERY_DEAD_VOLTAGE_RELEASE) {
		flags->is_battery_dead = FALSE;
	}
}

/*========================================================================================

Voltage-Percentage mapping

========================================================================================*/

/*------------------------------------------------------------------------
 Example:
	p0 = (4200, 10000); 	4.2V for 100%
	p1 = (3900, 8000);  	3.9V for 80%
	p2 = (3700, 2000);  	3.7V for 20%
	p3 = (3300, 0); 		3.3V for 0%

	if V = 4000, (3900<4000<4200)
	P = (4000-3900) * (10000-8000)/(4200-3900) + 8000 = 8666*/

#define NUM_SAMPLED_POINTS_MAX 12

struct sampled_point_type {

	DWORD voltage;
	DWORD capacity;
};

struct voltage_curve_translator {

	DWORD voltage_min;
	DWORD voltage_max;
	DWORD capacity_min;
	DWORD capacity_max;
	int sampled_point_count;
	struct sampled_point_type sampled_points[NUM_SAMPLED_POINTS_MAX];
};

static void voltage_curve_translator_init(struct voltage_curve_translator *t)
{
	memset(t, 0, sizeof(*t));
}

static void voltage_curve_translator_add(struct voltage_curve_translator *t, DWORD voltage, DWORD capacity)
{
	struct sampled_point_type *pt;

	if (t->sampled_point_count >= NUM_SAMPLED_POINTS_MAX) {
		return;
	}

	t->sampled_points[t->sampled_point_count].voltage = voltage;
	t->sampled_points[t->sampled_point_count].capacity = capacity;
	pt = &t->sampled_points[t->sampled_point_count];

	t->sampled_point_count++;

	if (pt->voltage > t->voltage_max)
		t->voltage_max = pt->voltage;
	if (pt->voltage < t->voltage_min)
		t->voltage_min = pt->voltage;
	if (pt->capacity > t->capacity_max)
		t->capacity_max = pt->capacity;
	if (pt->capacity < t->capacity_min)
		t->capacity_min = pt->capacity;

#if HTC_ENABLE_POWER_DEBUG
	printk(DRIVER_ZONE " kadc t: capacity=%d voltage=%d\n", capacity, voltage);
#endif /* HTC_ENABLE_POWER_DEBUG*/
}

static INT32 voltage_curve_translator_get(struct voltage_curve_translator *t, DWORD voltage)
{
	struct sampled_point_type *p0, *p1;
	INT32 capacity;
	int i;

	if (voltage > t->voltage_max)
		voltage = t->voltage_max;
	if (voltage < t->voltage_min)
		voltage = t->voltage_min;

	p0 = &t->sampled_points[0];
	p1 = p0 + 1;
	for (i = 0; i < t->sampled_point_count - 1 && voltage < p1->voltage; i++) {
		p0++;
		p1++;
	}

	/* DIV ZERO check*/
	if (p0->voltage - p1->voltage == 0) {
		return 0;
	}

	/* INT32 overflow check: mv(4200) * capacity(10000), shall be no problem at all...*/
	capacity = (voltage - p1->voltage) * (p0->capacity - p1->capacity) / (p0->voltage - p1->voltage) + p1->capacity;
	if (capacity > t->capacity_max) {
		capacity = t->capacity_max;
	}
	if (capacity < t->capacity_min) {
		capacity = t->capacity_min;
	}
	return capacity;
}

/*========================================================================================

KADC mapping functions

========================================================================================*/

static struct voltage_curve_translator __get_kadc_t;

static INT32 get_kadc_001p(struct battery_type *battery)
{
	INT32 pd_m = 0;
	INT32 pd_temp = 0;

	INT32 temp_01c = battery->temp_01c;
	INT32 current_mA = battery->current_mA;

	UINT32 *m_paramtable;

	INT32 pd_m_coef;
	INT32 pd_m_resl;

	INT32 capacity_deduction_01p = CAPACITY_DEDUCTION_01p[battery->temp_index];
	INT32 capacity_predict_001p;
	/* 1. INT32 overflow check: assert abs(iChgCurrent_ma) <= 3000, iBattTemp_01c>-250, pd_t_coef <= 1000
		when calculating pd_temp: 0x7FFFFFFF / (500 * 3000 * 1000) =:= 1.4*/

	if (battery->current_mA > 3000)
		current_mA = 3000;
	else if (battery->current_mA < -3000)
		current_mA = -3000;
	if (battery->temp_01c <= -250)
		temp_01c = -250;

	/* 2. calculate pd_m and pd_temp*/

	if (battery->is_power_on_reset) {
		pd_m_coef = PD_M_COEF_TABLE_BOOT[battery->temp_index][battery->id_index];
		pd_m_resl = PD_M_RESL_TABLE_BOOT[battery->temp_index][battery->id_index];
	}
	else{
		pd_m_coef = PD_M_COEF_TABLE[battery->temp_index][battery->id_index];
		pd_m_resl = PD_M_RESL_TABLE[battery->temp_index][battery->id_index];
	}

	if (battery->current_mA < -pd_m_bias_mA) {
		/* ex: -150mA < -130mA*/
		pd_m = (abs(battery->current_mA) - pd_m_bias_mA) * pd_m_coef /pd_m_resl;
	}

	if (battery->temp_01c < 250) {
		pd_temp = ((250 - battery->temp_01c) * (abs(battery->current_mA) * PD_T_COEF[battery->id_index])) / (10 * 10000);
	}
	battery->pd_m = pd_m;

	/* 3. calculate KADC using M_PARAMTER_TABLE*/

	m_paramtable = M_PARAMTER_TABLE[battery->id_index];
	if (m_paramtable) {
		int i = 0; /* assume that m_paramtable has at least 2 items...the last capacity item must be 0 to end the loop...*/

		voltage_curve_translator_init(&__get_kadc_t);

		while (1) {
			INT32 capacity = m_paramtable[i];
			INT32 voltage = m_paramtable[i + 1];
			if (capacity == 10000) {
				/* full capacity, no need to fix voltage level*/
				voltage_curve_translator_add(&__get_kadc_t, voltage, capacity);
			}
			else {
				voltage_curve_translator_add(&__get_kadc_t, voltage - pd_temp, capacity);
			}

			if (capacity == 0)
				break;

			i += 2;
		}

#if HTC_ENABLE_POWER_DEBUG
		printk(DRIVER_ZONE " pd_m=%d, pd_temp=%d\n", pd_m, pd_temp);
#endif /* HTC_ENABLE_POWER_DEBUG*/

		capacity_predict_001p = voltage_curve_translator_get(&__get_kadc_t, battery->voltage_mV + pd_m);
	}
	else{
		capacity_predict_001p = (battery->voltage_mV - 3400) * 10000 / (4200 - 3400);
	}

	return (capacity_predict_001p - capacity_deduction_01p * 10) * 10000 / (10000 - capacity_deduction_01p * 10);
}

/*========================================================================================

coulomb counter+curve tracer

========================================================================================*/

static INT32 get_software_acr_revise(struct battery_type *battery, UINT32 ms)
{
	INT32 kadc_01p = battery->KADC_01p;
	INT32 ccbi_01p = battery->RARC_01p;
	INT32 delta_01p = kadc_01p - ccbi_01p;

	DWORD C = 5;						/* KADC = 15%~100%*/
	if (kadc_01p <= 150) {
		C = 5;
	}   	/* KADC = 0%~15%*/

	if (delta_01p < 0) {
		/* if KADC is less than RARC, p shall be lower*/
		return -(INT32) (C * ms * delta_01p * delta_01p) / 1000;
	}
	else{
		/* if KADC is larger than RARC, p shall be higher*/
		return (INT32) (C * ms * delta_01p * delta_01p) / 1000;
	}
}

/*========================================================================================

ds2746 gauge ic functions, to access ds2746 registers and convert ADC to battery param

========================================================================================*/

static void __ds2746_clear_porf(void)
{
	UINT8 reg_data;
	if (!ds2746_i2c_read_u8(&reg_data, 0x01)) {
		printk(DRIVER_ZONE " clear porf error in read.\n");
		return;
	}

	if (!ds2746_i2c_write_u8((reg_data & (~DS2746_STATUS_PORF)), 0x01)) {
		printk(DRIVER_ZONE " clear porf error in write.\n");
		return;
	}
}

static void __ds2746_acr_update(struct battery_type *battery, int capacity_01p)
{
	printk(DRIVER_ZONE " acr update: P=%d, C=%d.\n",
		capacity_01p,
		battery->charge_counter_adc);

	ds2746_i2c_write_u8((battery->charge_counter_adc & 0xFF00) >> 8, 0x10);
	ds2746_i2c_write_u8((battery->charge_counter_adc & 0x00FF), 0x11);

	if (battery->is_power_on_reset) {
		__ds2746_clear_porf();
	}
}

static void __ds2746_init_config(struct battery_type *battery)
{
	UINT8 reg_data;

	if (!ds2746_i2c_read_u8(&reg_data, 0x01)) {
		printk(DRIVER_ZONE " init config error in read.\n");
		return;
	}

	/* Erase SMOD and NBEN value in DS2746 status/config register*/
	reg_data &= ~(DS2746_STATUS_SMOD | DS2746_STATUS_NBEN);
	if (!ds2746_i2c_write_u8(reg_data, 0x01)) {
		printk(DRIVER_ZONE " init config error in write.\n");
		return;
	}
}

static BOOL __ds2746_get_reg_data(UINT8 *reg)
{
	memset(reg, 0, 12);

	if (!ds2746_i2c_read_u8(&reg[0], 0x01))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[2], 0x08))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[3], 0x09))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[4], 0x0a))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[5], 0x0b))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[6], 0x0c))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[7], 0x0d))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[8], 0x0e))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[9], 0x0f))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[10], 0x10))
		return FALSE;
	if (!ds2746_i2c_read_u8(&reg[11], 0x11))
		return FALSE;

	return TRUE;
}

static BOOL __ds2746_battery_adc_udpate(struct battery_type *battery)
{
	UINT8 reg[12];

	if (!__ds2746_get_reg_data((UINT8 *) &reg)) {
		printk(DRIVER_ZONE " get ds2746 data failed...\n");
		return FALSE;
	}

	printk(DRIVER_ZONE " [x0]%x [x8]%x %x %x %x %x %x %x %x %x %x\n",
		reg[0],
		reg[2],
		reg[3],
		reg[4],
		reg[5],
		reg[6],
		reg[7],
		reg[8],
		reg[9],
		reg[10],
		reg[11]);

	if (!(reg[0] & DS2746_STATUS_AIN0) || !(reg[0] & DS2746_STATUS_AIN1)) {
		printk(DRIVER_ZONE " AIN not ready...\n");
		return FALSE;
	}

	if (reg[0] & DS2746_STATUS_PORF) {
		battery->is_power_on_reset = TRUE;
	}
	else{
		battery->is_power_on_reset = FALSE;
	}

	/* adc register value*/
	battery->voltage_adc = MAKEWORD(reg[7], reg[6]) >> 4;
	battery->current_adc = MAKEWORD(reg[9], reg[8]);
	if (battery->current_adc & 0x8000) {
		battery->current_adc = -(0x10000 - battery->current_adc);
	}
	battery->current_adc /= 4;
	battery->charge_counter_adc = MAKEWORD(reg[11], reg[10]);
	if (battery->charge_counter_adc & 0x8000) {
		battery->charge_counter_adc = -(0x10000 - battery->charge_counter_adc);
	}
	battery->id_adc = MAKEWORD(reg[5], reg[4]) >> 4;
	battery->temp_adc = MAKEWORD(reg[3], reg[2]) >> 4;
	if (support_ds2746_gauge_ic) {
		/* we preserve 500mAh for capacity lower than 0%, however the 500mAh is still drained out...we need to do predict for correct ACR*/
		if ((battery->charge_counter_adc & 0xFFFF) >= 0xF000){
			printk(DRIVER_ZONE " ACR out of range (x%x)...\n",
				battery->charge_counter_adc);
			battery->is_power_on_reset = TRUE;
		}
	}

	return TRUE;
}

/*========================================================================================

softwar acr functions, to accumulate ACR by software and revise by battery parameter

========================================================================================*/

static void __software_charge_counter_update(struct battery_type *battery, UINT32 ms)
{
	/* if the charge counter is maintained by sw, batt_alg shall use this routine to update charge counter and related parameters*/
	INT32 capacity_deduction_01p = CAPACITY_DEDUCTION_01p[battery->temp_index];
	/* AEL(mAh):	A low temp unusable battery capacity, calculated in runtime*/
	INT32 ael_mAh = capacity_deduction_01p *battery->charge_full_real_mAh /	1000;
#if HTC_ENABLE_POWER_DEBUG
	printk(DRIVER_ZONE "chgctr update: I=%d ms=%d.\n", battery->current_mA, ms);
#endif /* HTC_ENABLE_POWER_DEBUG*/

	/* ACRt(mAh):   The total capacity battery owns, stored in battery->charge_counter_mAh*/
	battery->software_charge_counter_mAms += (INT32) (battery->current_mA * ms);
	battery->charge_counter_mAh += (battery->software_charge_counter_mAms /	3600000);
	battery->software_charge_counter_mAms -= (battery->software_charge_counter_mAms / 3600000) * 3600000;

	/* CCBI(0.1%):  A software RARC*/
	battery->RARC_01p = (battery->charge_counter_mAh - ael_mAh) * 1000 / (battery->charge_full_real_mAh - ael_mAh);
	/* store back the battery->charge_counter_mAh to battery->charge_counter_adc*/
	battery->charge_counter_adc = (battery->charge_counter_mAh + charge_counter_zero_base_mAh) * acr_adc_to_mv_coef / acr_adc_to_mv_resl;
}

static void __software_charge_counter_revise(struct battery_type *battery, UINT32 ms)
{
	if (battery->current_mA < 0) {
#if HTC_ENABLE_POWER_DEBUG
		printk(DRIVER_ZONE "chgctr revise: delta=%d.\n", get_software_acr_revise(battery, ms));
#endif /* HTC_ENABLE_POWER_DEBUG*/

		/* revise software charge counter by coulomb counter+curve tracer*/
		battery->software_charge_counter_mAms += get_software_acr_revise(battery, ms);
		battery->charge_counter_mAh += (battery->software_charge_counter_mAms /	3600000);
		battery->software_charge_counter_mAms -= (battery->software_charge_counter_mAms / 3600000) * 3600000;
		/* store back the battery->charge_counter_mAh to battery->charge_counter_adc*/
		battery->charge_counter_adc = (battery->charge_counter_mAh + charge_counter_zero_base_mAh) * acr_adc_to_mv_coef / acr_adc_to_mv_resl;
	}
}

static void __software_acr_update(struct battery_type *battery)
{
	static BOOL s_bFirstEntry = TRUE;
	static DWORD last_time_ms;
	DWORD now_time_ms = BAHW_MyGetMSecs();

	if (s_bFirstEntry) {
		s_bFirstEntry = FALSE;
		last_time_ms = now_time_ms;
	}

#if HTC_ENABLE_POWER_DEBUG
	printk(DRIVER_ZONE "+acr update: adc=%d C=%d mams=%d.\n",
		battery->charge_counter_adc,
		battery->charge_counter_mAh,
		battery->software_charge_counter_mAms);
#endif /* HTC_ENABLE_POWER_DEBUG*/

	__software_charge_counter_update(battery, now_time_ms - last_time_ms);
	__software_charge_counter_revise(battery, now_time_ms - last_time_ms);

#if HTC_ENABLE_POWER_DEBUG
	printk(DRIVER_ZONE "-acr update: adc=%d C=%d mams=%d.\n",
		battery->charge_counter_adc,
		battery->charge_counter_mAh,
		battery->software_charge_counter_mAms);
#endif /* HTC_ENABLE_POWER_DEBUG*/

	last_time_ms = now_time_ms;
}

/*========================================================================================

battery param update, the coef are referenced from power spec

========================================================================================*/

static BOOL __battery_param_udpate(struct battery_type *battery)
{
	static int batt_id_stable_counter = 0;
	INT32 batt_id_index;
	INT32 temp_01c;

	if (support_ds2746_gauge_ic) {
		/* adc register value are read from __ds2746_battery_adc_udpate()*/
		if (!__ds2746_battery_adc_udpate(battery))
			return FALSE;
	}
	else{
		/* adc register value are read from BAHW_get_batt_info_all()
		if ( !BAHW_get_batt_info_all(battery) ) return FALSE;*/
	}

	/*real physical value*/
	battery->voltage_mV = (battery->voltage_adc * voltage_adc_to_mv_coef / voltage_adc_to_mv_resl);
	battery->current_mA = (battery->current_adc * current_adc_to_mv_coef / current_adc_to_mv_resl);
	battery->discharge_mA = (battery->discharge_adc * discharge_adc_to_mv_coef / discharge_adc_to_mv_resl);
	battery->charge_counter_mAh = (battery->charge_counter_adc * acr_adc_to_mv_coef / acr_adc_to_mv_resl) -	charge_counter_zero_base_mAh;
	battery->current_mA = battery->current_mA - battery->discharge_mA;
	/* prevent from adc out of range*/
	if (battery->id_adc >= id_adc_resl) {
		battery->id_adc = id_adc_resl - 1;
	}
	if (battery->id_adc <= 0) {
		battery->id_adc = 1;
	}
	if (battery->temp_adc >= temp_adc_resl) {
		battery->temp_adc = temp_adc_resl - 1;
	}
	if (battery->temp_adc <= 0) {
		battery->temp_adc = 1;
	}

	/* battery ID shall be ready first for temp/kadc calculation*/
	//   if ( id_conversion ) battery->id_ohm = ((float)id_R_kohm / ((float)id_adc_resl/battery->id_adc - 1)) * 1000;     // kohm -> ohm
	//   else   			  battery->id_ohm = battery->id_adc;
	battery->id_ohm = battery->id_adc;
	calibrate_id_ohm(battery);

	batt_id_index = get_id_index(battery);

	if (is_allow_batt_id_change) {
		/*! TODO: batt_id changes immediately; may need to modify in future*/
		if (batt_id_stable_counter >= 3 && batt_id_index != battery->id_index){
			/* if batt_id is stable but is different from previous one*/
			batt_id_stable_counter = 0; /* reset stable counter and set batt_id to new one*/
		}
	}

	if (batt_id_stable_counter < 3) {
		if (batt_id_stable_counter == 0) {
			/* first time to get the batt id*/
			battery->id_index = batt_id_index;
			battery->charge_full_design_mAh = FL_25[battery->id_index];
			battery->charge_full_real_mAh = battery->charge_full_design_mAh;
			batt_id_stable_counter = 1;
		}
		else{
			/* 2nd and further time to get the batt id*/
			if (batt_id_index == battery->id_index)
				batt_id_stable_counter++;
			else
				batt_id_stable_counter = 0;
		}
	}

	/* calculate temperature*/
	//    battery->temp_01c 			  = get_temp_c((float)temp_R_kohm / ((float)temp_adc_resl/battery->temp_adc - 1))*10;
	temp_01c = get_temp_01c(battery);
	if (temp_01c >= TEMP_MIN*10)
		battery->temp_01c = temp_01c;
	else
		printk(DRIVER_ZONE " get temp_01c(%d) failed...\n", temp_01c);
	battery->temp_index = get_temp_index(battery);

	/* calculate KADC and RARC*/
	battery->KADC_01p = CEILING(get_kadc_001p(battery), 10);
	battery->RARC_01p = CEILING(10000 * battery->charge_counter_mAh / battery->charge_full_real_mAh, 10);
	if (!support_ds2746_gauge_ic) {
		__software_acr_update(battery);
	}

	if (battery->voltage_mV <BATTERY_VOLTAGE_MIN ||
		battery->voltage_mV> BATTERY_VOLTAGE_MAX) {
		printk(DRIVER_ZONE " invalid V(%d).\n", battery->voltage_mV);
		return FALSE;
	}

	/*! star_lee 20100426 - minimum RARC is 0%*/
	if (battery->RARC_01p <= 0) {
		battery->RARC_01p = 0;
	}

	printk(DRIVER_ZONE " V=%d(%x) I=%d(%x) C=%d.%d/%d(%x) id=%d(%x) T=%d(%x) KADC=%d\n",
		battery->voltage_mV,
		battery->voltage_adc,
		battery->current_mA,
		battery->current_adc,
		battery->charge_counter_mAh,
		battery->software_charge_counter_mAms,
		battery->charge_full_real_mAh,
		battery->charge_counter_adc,
		battery->id_index,
		battery->id_adc,
		battery->temp_01c,
		battery->temp_adc,
		battery->KADC_01p);

	return TRUE;
}

/*========================================================================================

time functions

========================================================================================*/

DWORD BAHW_MyGetMSecs(void)
{
	struct timespec now;
	getnstimeofday(&now);
	/*struct timespec t;
	t.tv_sec = t.tv_nsec = 0;
	clock_gettime(CLOCK_MONOTONIC, &t);*/
	return now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

/*========================================================================================

battery param public function

========================================================================================*/

void battery_capacity_update(struct battery_type *battery, int capacity_01p)
{

	/* ACR 500~500+FULL mapping to capacity 0~FULL*/
	battery->charge_counter_mAh = capacity_01p * battery->charge_full_real_mAh / 1000;
	battery->charge_counter_adc = (battery->charge_counter_mAh + charge_counter_zero_base_mAh) * acr_adc_to_mv_resl / acr_adc_to_mv_coef;
	battery->RARC_01p = capacity_01p;
	if (support_ds2746_gauge_ic) {
		__ds2746_acr_update(battery, capacity_01p);
	}

	printk(DRIVER_ZONE "new RARC=%d C=%dmAh adc=%d.\n",
		battery->RARC_01p,
		battery->charge_counter_mAh,
		battery->charge_counter_adc);
	battery->is_power_on_reset = FALSE;
}

BOOL battery_param_update(struct battery_type *battery,	struct protect_flags_type *flags)
{
	if (!__battery_param_udpate(battery)) {
		return FALSE;
	}

	if (flags->is_fake_room_temp) {
		battery->temp_01c = 250;
		printk(DRIVER_ZONE "fake temp=%d(%x)\n",
		battery->temp_01c,
		battery->temp_adc);
	}
	__protect_flags_update(battery, flags);

#if ! HTC_ENABLE_DUMMY_BATTERY
	if (battery->id_index == BATTERY_ID_UNKNOWN) {
		flags->is_charging_enable_available = FALSE;
	}
#else /* HTC_ENABLE_DUMMY_BATTERY*/
	/* do not disable charging for debug stage*/
	flags->is_charging_enable_available = TRUE;
#endif /* HTC_ENABLE_DUMMY_BATTERY*/

	return TRUE;
}

void battery_param_init(struct battery_type *battery)
{
	/* set battery id to unknown to get battery id and related characters*/
	battery->id_index = BATTERY_ID_UNKNOWN;

	/* default to 25C unless we can get valid battery temp from adc*/
	battery->temp_01c = 250;
	battery->last_temp_01c = battery->temp_01c;
	battery->temp_check_index = 0;
	battery->last_temp_adc = 0;

	battery->voltage_mV = 3800;

	/* this is used when accumulate current by software; initial it as 0mAs*/
	battery->software_charge_counter_mAms = 0;

	/* set POR at first by software; gauge ic will has correct value*/
	battery->is_power_on_reset = TRUE;

	if (support_ds2746_gauge_ic) {
		__ds2746_init_config(battery);
	}

	if (battery->thermal_id == THERMAL_1000) {
		TEMP_MAP = TEMP_MAP_1000K;
		printk(DRIVER_ZONE "Use 1000 Kohm thermal resistance");
	} else {
		printk(DRIVER_ZONE "Use default(300 Kohm) thermal resistance");
	}

	/*printk(DRIVER_ZONE "battery param inited with board name <%s>\n", HTC_BATT_BOARD_NAME);*/
}

