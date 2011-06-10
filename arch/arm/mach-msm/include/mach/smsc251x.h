#ifndef _LINUX_SMSC251X_H
#define _LINUX_SMSC251X_H

#define SMSC251X_NAME "smsc_251x"

#define MANUFACUTRE_STRING_LEN 6
#define PRODUCT_STRING_LEN 16
#define SERIAL_STRING_LEN 18

/*register*/
#define SMSC_VENDOR_ID								0x0
#define SMSC_PRODUCT_ID							0x2
#define SMSC_DEVICE_ID								0x4
#define SMSC_CONFIG_DATA_1						0x6
#define SMSC_CONFIG_DATA_2						0x7
#define SMSC_CONFIG_DATA_3						0x8
#define SMSC_PORT_DISABLE_SELF					0xA
#define SMSC_PORT_DISABLE_BUS						0xB
#define SMSC_MAX_POWER_SELF						0xC
#define SMSC_MAX_POWER_BUS						0xD
#define SMSC_MANUFACTURER_STRING_LEN			0x13
#define SMSC_PRODUCT_STRING_LEN					0x14
#define SMSC_SERIAL_STRING_LEN					0x15
#define SMSC_MANUFACTURER_STRING					0x16
#define SMSC_PRODUCT_STRING						0x54
#define SMSC_SERIAL_STRING							0x92
#define SMSC_STATUS_CMD							0xFF

struct smsc251x_platform_data {
	void (*usb_hub_gpio_config)(bool);
	int (*register_switch_func)(int (*callback)(uint8_t enable));
};
int smsc251x_mdm_port_sw(uint8_t enable);
void smsc251x_set_diag_boot_flag(int enable);
int i2c_smsc251x_read(uint8_t address, uint8_t *data, uint8_t length);
int i2c_smsc251x_write(uint8_t address, uint8_t *data, uint8_t length);
#endif


