#ifndef _MSM_PANEL_H_
#define _MSM_PANEL_H_

enum {
	GATE_ON_DCR = 1 << 0,
	CABC_STATE_DCR,
};

struct panel_platform_data {
	struct resource *fb_res;
	int (*power)(int on);
	int (*gpio_switch)(int on);
};
struct msm_mddi_client_data;
/* For those MDDI clients with variadic length of parameter, such as Samsung
 * S6D series controllers.
 */
struct lcm_va_cmd {
	uint32_t reg, delay, size;
	uint8_t *value;
};
#define CMD_VECT(r, d, ...) {r, d, sizeof((uint8_t[]){__VA_ARGS__}), \
	(uint8_t []){__VA_ARGS__} }

	struct cabc_platform_data {
		int (*change_cabcmode)(struct msm_mddi_client_data *client_data,
				int mode, u8 dimming);
	};

struct cabc_config {
	int panel;
	int shrink;
	uint8_t *pwm_data;
	int min_level;
	int default_br;
	struct msm_mddi_client_data *client;
	int (*bl_handle)(struct platform_device *, int);
	int (*shrink_br)(int brightness);
	int (*change_cabcmode)(struct msm_mddi_client_data *client_data,
			int mode, u8 dimming);
};

struct panel_dcr_info {
	int (*lut_table)(void);
	void (*dcr_video_mode)(bool on);
	void (*dcr_power)(bool on);
	void (*bkl_smooth)(bool on);
	bool (*get_bkl_smooth_status)(void);
	atomic_t video_mode;
	unsigned long auto_bkl_stat;
};

extern unsigned long auto_bkl_status;
#endif
