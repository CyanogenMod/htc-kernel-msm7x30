/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PM8XXX_CHARGER_H
#define __PM8XXX_CHARGER_H

#include <linux/errno.h>

#define PM8921_CHARGER_DEV_NAME	"pm8921-charger"

struct pm8xxx_charger_core_data {
	unsigned int	vbat_channel;
	unsigned int	batt_temp_channel;
	unsigned int	batt_id_channel;
};

enum pm8921_chg_cold_thr {
	PM_SMBC_BATT_TEMP_COLD_THR__LOW,
	PM_SMBC_BATT_TEMP_COLD_THR__HIGH
};

enum pm8921_chg_hot_thr	{
	PM_SMBC_BATT_TEMP_HOT_THR__LOW,
	PM_SMBC_BATT_TEMP_HOT_THR__HIGH
};

/**
 * struct pm8921_charger_platform_data -
 * @safety_time:	max charging time in minutes
 * @ttrkl_time:		max trckl charging time in minutes
 * @update_time:	how often the userland be updated of the charging (msec)
 * @max_voltage:	the max voltage (mV) the battery should be charged up to
 * @min_voltage:	the voltage (mV) where charging method switches from
 *			trickle to fast. This is also the minimum voltage the
 *			system operates at
 * @resume_voltage_delta:	the (mV) drop to wait for before resume charging
 *				after the battery has been fully charged
 * @term_current:	the charger current (mA) at which EOC happens
 * @cool_temp:		the temperature (degC) at which the battery is
 *			considered cool charging current and voltage is reduced
 * @warm_temp:		the temperature (degC) at which the battery is
 *			considered warm charging current and voltage is reduced
 * @temp_check_period:	The polling interval in seconds to check battery
 *			temeperature if it has gone to cool or warm temperature
 *			area
 * @max_bat_chg_current:	Max charge current of the battery in mA
 *				Usually 70% of full charge capacity
 * @cool_bat_chg_current:	chg current (mA) when the battery is cool
 * @warm_bat_chg_current:	chg current (mA)  when the battery is warm
 * @cool_bat_voltage:		chg voltage (mV) when the battery is cool
 * @warm_bat_voltage:		chg voltage (mV) when the battery is warm
 * @get_batt_capacity_percent:
 *			a board specific function to return battery
 *			capacity. If null - a default one will be used
 * @trkl_voltage:	the trkl voltage in (mV) below which hw controlled
 *			 trkl charging happens with linear charger
 * @weak_voltage:	the weak voltage (mV) below which hw controlled
 *			trkl charging happens with switching mode charger
 * @trkl_current:	the trkl current in (mA) to use for trkl charging phase
 * @weak_current:	the weak current in (mA) to use for weak charging phase
 * @vin_min:		the input voltage regulation point (mV) - if the
 *			voltage falls below this, the charger reduces charge
 *			current or stop charging temporarily
 * @thermal_mitigation: the array of charge currents to use as temperature
 *			increases
 * @thermal_levels:	the number of thermal mitigation levels supported
 * @cold_thr:		if high battery will be cold when VBAT_THERM goes above
 *			80% of VREF_THERM (typically 1.8volts), if low the
 *			battery will be considered cold if VBAT_THERM goes above
 *			70% of VREF_THERM. Hardware defaults to low.
 * @hot_thr:		if high the battery will be considered hot when the
 *			VBAT_THERM goes below 35% of VREF_THERM, if low the
 *			battery will be considered hot when VBAT_THERM goes
 *			below 25% of VREF_THERM. Hardware defaults to low.
 */
struct pm8921_charger_platform_data {
	struct pm8xxx_charger_core_data	charger_cdata;
	unsigned int			safety_time;
	unsigned int			ttrkl_time;
	unsigned int			update_time;
	unsigned int			max_voltage;
	unsigned int			min_voltage;
	unsigned int			resume_voltage_delta;
	unsigned int			term_current;
	unsigned int			cool_temp;
	unsigned int			warm_temp;
	unsigned int			temp_check_period;
	unsigned int			max_bat_chg_current;
	unsigned int			cool_bat_chg_current;
	unsigned int			warm_bat_chg_current;
	unsigned int			cool_bat_voltage;
	unsigned int			warm_bat_voltage;
	unsigned int			(*get_batt_capacity_percent) (void);
	int64_t				batt_id_min;
	int64_t				batt_id_max;
	int				trkl_voltage;
	int				weak_voltage;
	int				trkl_current;
	int				weak_current;
	int				vin_min;
	int				*thermal_mitigation;
	int				thermal_levels;
	enum pm8921_chg_cold_thr	cold_thr;
	enum pm8921_chg_hot_thr		hot_thr;
};

enum pm8921_charger_source {
	PM8921_CHG_SRC_NONE,
	PM8921_CHG_SRC_USB,
	PM8921_CHG_SRC_DC,
};

/**
 * struct ext_chg_pm8921 -
 * @name:		name of the external charger
 * @ctx:		client context.
 * @start_charging:	callback to start charging. Can be called from an
 *			interrupt context
 * @stop_charging:	callback to stop charging. Can be called from an
 *			interrupt context
 * @is_trickle:		callback to check if trickle charging.
 *			Can be called from an interrupt context
 *
 */
struct ext_chg_pm8921 {
	const char	*name;
	void		*ctx;
	int		(*start_charging) (void *ctx);
	int		(*stop_charging) (void *ctx);
	bool		(*is_trickle) (void *ctx);
};

#ifdef CONFIG_HTC_BATT_8960
/* struct to hook in htc_battery_cell.chg_param
 * some charger parameters is depends on battery. */
struct pm8921_charger_batt_param {
	int max_voltage;
	int resume_voltage;
};
#endif /* CONFIG_HTC_BATT_8960 */

#if defined(CONFIG_PM8921_CHARGER) || defined(CONFIG_PM8921_CHARGER_MODULE)
void pm8921_charger_vbus_draw(unsigned int mA);
int pm8921_charger_register_vbus_sn(void (*callback)(int));
void pm8921_charger_unregister_vbus_sn(void (*callback)(int));
/**
 * pm8921_charger_enable -
 *
 * @enable: 1 means enable charging, 0 means disable
 *
 * Enable/Disable battery charging current, the device will still draw current
 * from the charging source
 */
int pm8921_charger_enable(bool enable);

/**
 * pm8921_is_usb_chg_plugged_in - is usb plugged in
 *
 * if usb is under voltage or over voltage this will return false
 */
int pm8921_is_usb_chg_plugged_in(void);

/**
 * pm8921_is_dc_chg_plugged_in - is dc plugged in
 *
 * if dc is under voltage or over voltage this will return false
 */
int pm8921_is_dc_chg_plugged_in(void);

/**
 * pm8921_is_battery_present -
 *
 * returns if the pmic sees the battery present
 */
int pm8921_is_battery_present(void);

/**
 * pm8921_set_max_battery_charge_current - set max battery chg current
 *
 * @ma: max charge current in milliAmperes
 */
int pm8921_set_max_battery_charge_current(int ma);

/**
 * pm8921_disable_source_current - disable drawing current from source
 * @disable: true to disable current drawing from source false otherwise
 *
 * This function will stop all charging activities and disable any current
 * drawn from the charger. The battery provides the system current.
 */
int pm8921_disable_source_current(bool disable);

/**
 * pm8921_regulate_input_voltage -
 * @voltage: voltage in millivolts to regulate
 *		allowable values are from 4300mV to 6500mV
 */
int pm8921_regulate_input_voltage(int voltage);
/**
 * pm8921_is_battery_charging -
 * @source: when the battery is charging the source is updated to reflect which
 *		charger, usb or dc, is charging the battery.
 *
 * RETURNS: bool, whether the battery is being charged or not
 */
bool pm8921_is_battery_charging(int *source);

/**
 * pm8921_batt_temperature - get battery temp in degC
 *
 */
int pm8921_batt_temperature(void);
/**
 * register_external_dc_charger -
 * @ext:	The structure representing an external charger
 *
 * RETURNS:	Negative error code is there was a problem. Zero for sucess
 *
 * The charger callbacks might be called even before this function
 * completes. The external charger driver should be ready to handle
 * it.
 */
int register_external_dc_charger(struct ext_chg_pm8921 *ext);

/**
 * unregister_external_dc_charger -
 * @ext:	The structure representing an external charger
 *
 * The charger callbacks might be called even before this function
 * completes. The external charger driver should be ready to handle
 * it.
 */
void unregister_external_dc_charger(struct ext_chg_pm8921 *ext);

#ifdef CONFIG_HTC_BATT_8960
/********************************************/
/* htc_gauge/htc_charger abstract interface */
/********************************************/
/**
 * pm8921_get_batt_voltage - get battery voltage in mV
 *
 */
int pm8921_get_batt_voltage(int *result);

/**
 * pm8921_get_batt_temperature - get battery temperature in C
 *
 */
int pm8921_get_batt_temperature(int *result);

/**
 * pm8921_get_batt_id - get battery id in ?
 *
 */
int pm8921_get_batt_id(int *result);

/**
 * pm8921_is_batt_temperature_fault
 *
 */
int pm8921_is_batt_temperature_fault(int *result);

/**
 * pm8921_get_charging_source
 *
 */
int pm8921_get_charging_source(int *result);

/**
 * pm8921_get_charging_enabled
 *
 */
int pm8921_get_charging_enabled(int *result);
/**
 * pm8921_dump_all
 *
 */
int pm8921_dump_all(void);
#endif /* CONFIG_HTC_BATT_8960 */
#else
static inline void pm8921_charger_vbus_draw(unsigned int mA)
{
}
static inline int pm8921_charger_register_vbus_sn(void (*callback)(int))
{
	return -ENXIO;
}
static inline void pm8921_charger_unregister_vbus_sn(void (*callback)(int))
{
}
static inline int pm8921_charger_enable(bool enable)
{
	return -ENXIO;
}
static inline int pm8921_is_usb_chg_plugged_in(void)
{
	return -ENXIO;
}
static inline int pm8921_is_dc_chg_plugged_in(void)
{
	return -ENXIO;
}
static inline int pm8921_is_battery_present(void)
{
	return -ENXIO;
}
static inline int pm8921_set_max_battery_charge_current(int ma)
{
	return -ENXIO;
}
static inline int pm8921_disable_source_current(bool disable)
{
	return -ENXIO;
}
static inline int pm8921_regulate_input_voltage(int voltage)
{
	return -ENXIO;
}
static inline bool pm8921_is_battery_charging(int *source)
{
	*source = PM8921_CHG_SRC_NONE;
	return 0;
}
static inline int pm8921_batt_temperature(void)
{
	return -ENXIO;
}
static inline int register_external_dc_charger(struct ext_chg_pm8921 *ext)
{
	pr_err("%s.not implemented.\n", __func__);
	return -ENODEV;
}
static inline void unregister_external_dc_charger(struct ext_chg_pm8921 *ext)
{
	pr_err("%s.not implemented.\n", __func__);
}
#ifdef CONFIG_HTC_BATT_8960
/* htc_gauge/charger interface */
static inline int pm8921_get_batt_voltage(int *result)
{
	return -ENXIO;
}
static inline int pm8921_get_batt_temperature(int *result)
{
	return -ENXIO;
}
static inline int pm8921_get_batt_id(int *result)
{
	return -ENXIO;
}
static inline int pm8921_is_batt_temperature_fault(int *result)
{
	return -ENXIO;
}
static inline int pm8921_get_charging_source(int *result)
{
	return -ENXIO;
}
static inline int pm8921_get_charging_enabled(int *result)
{
	return -ENXIO;
}
#endif /* CONFIG_HTC_BATT_8960 */
#endif

#endif
