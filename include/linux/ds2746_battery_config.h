/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Copyright (c) 2010 High Tech Computer Corporation

Module Name:

	batt_alg_config.c
Abstract:

	This module tells batt_alg.c module how to config power alg.

Original Auther:

	Star Lee (star_lee) Apr-12-2010

---------------------------------------------------------------------------------*/

#define HTC_BATT_BOARD_NAME "ACE"

static void poweralg_config_init(struct poweralg_config_type *config)
{
	config->full_charging_mv = 4110;
	config->full_charging_ma = 50;
	config->full_pending_ma = 0;		/* disabled*/
	config->full_charging_timeout_sec = 60 * 60;
	config->voltage_recharge_mv = 4150;
	config->capacity_recharge_p = 0;		/* disabled*/
	config->voltage_exit_full_mv = 4100;
	config->wait_votlage_statble_sec = 1 * 60;
	config->predict_timeout_sec = 10;
	config->polling_time_in_charging_sec = 30;
	config->polling_time_in_discharging_sec = 30;

	config->enable_full_calibration = TRUE;
	config->enable_weight_percentage = TRUE;
	config->software_charger_timeout_sec = 0;   	 /* disabled*/

	config->debug_disable_shutdown = FALSE;
	config->debug_fake_room_temp = FALSE;
	config->debug_disable_hw_timer = FALSE;
	config->debug_always_predict = FALSE;
	config->full_level = 0;
}

