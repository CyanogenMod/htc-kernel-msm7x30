/* drivers/input/touchscreen/elan_ktf2k.c - ELAN KTF2000 touchscreen driver
 *
 * Copyright (C) 2010 HTC Corporation.
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

/* #define DEBUG */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/elan_ktf2k.h>
#include <linux/device.h>
#include <linux/jiffies.h>

#define ELAN_I2C_RETRY_TIMES	10

#define ELAN_TS_FUZZ 		0
#define ELAN_TS_FLAT 		0
#define IDX_PACKET_SIZE		21

#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT			0x52
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54

#define HELLO_PKT			0x55
#define RPT_PKT				0x5D
#define NOISE_PKT			0x40

#define REPEAT_PKT			0xA6
#define RESET_PKT			0x77
#define CALIB_PKT			0xA8
#define CALIB_DONE			0x66

#define IDX_NUM				0x01
#define IDX_FINGER			0x02
#define IDX_WIDTH			0x12

#define TEST_MODE_DV		0x01
#define TEST_MODE_OFFSET	0x02
#define TEST_MODE_ADC		0x03
#define TEST_MODE_DV_Y		0x04
#define TEST_MODE_OFFSET_Y	0x05
#define TEST_MODE_ADC_Y		0x06
#define TEST_MODE_CLOSE		0x00
#define TEST_MODE_OPEN		0x01
#define TEST_MODE_SIZE		41
#define TEST_MODE_SIZE_Y	25

struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	int (*power)(int on);
	int (*reset)(void);
	struct early_suspend early_suspend;
	int intr_gpio;
	uint16_t fw_ver;
	uint8_t first_pressed;
	uint8_t finger_pressed;
	uint16_t last_finger_data[10][2];
	uint8_t debug_log_level;
	uint8_t packet_reg_addr;
	uint8_t diag_command;
	uint8_t diag_mode;
};

struct test_mode_cmd_open {
	uint8_t cmd1[6];
	uint8_t cmd2[6];
	uint8_t cmd3[6];
	uint8_t cmd4[4];
	uint8_t cmd5[11];
};

struct test_mode_cmd_open_v2 {
	uint8_t cmd1[4];
	uint8_t cmd2[11];
};

struct test_mode_cmd_close {
	uint8_t cmd1[4];
	uint8_t cmd2[4];
	uint8_t cmd3[6];
	uint8_t cmd4[6];
	uint8_t cmd5[6];
};

struct test_mode_cmd_close_v2 {
	uint8_t cmd1[4];
	uint8_t cmd2[4];
};

static struct elan_ktf2k_ts_data *private_ts;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h);
static void elan_ktf2k_ts_late_resume(struct early_suspend *h);
#endif

static int i2c_elan_ktf2k_read(struct i2c_client *client,
	uint8_t *buf, size_t len);
static int i2c_elan_ktf2k_write(struct i2c_client *client,
	uint8_t *buf, size_t len);

static int elan_ktf2k_ts_poll(struct i2c_client *client);
static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
	uint8_t *buf, size_t len);
static int elan_ktf2k_ts_setup(struct i2c_client *client);
/* static int elan_ktf2k_ts_repeat(struct i2c_client *client); */

static ssize_t elan_ktf2k_gpio_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->intr_gpio);
	printk(KERN_DEBUG "GPIO_TP_INT_N=%d\n", ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);

static ssize_t elan_ktf2k_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF2K", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf2k_vendor_show, NULL);

static ssize_t elan_ktf2k_packet_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int rc = 0;
	ssize_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t data[4] = {0};

	dev_dbg(&ts->client->dev, "%s: enter\n", __func__);
	cmd[1] = ts->packet_reg_addr;
	disable_irq(ts->client->irq);
	rc = elan_ktf2k_ts_get_data(ts->client, cmd, data, sizeof(data));
	enable_irq(ts->client->irq);
	if (rc < 0)
		return ret;

	ret += sprintf(buf, "addr: 0x%2.2X, data1: 0x%2.2X, data2: 0x%X\n",
		data[1], data[2], data[3] >> 4);
	return ret;
}

static ssize_t elan_ktf2k_packet_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct elan_ktf2k_ts_data *ts = private_ts;
	uint8_t cmd[] = {CMD_W_PKT, 0x00, 0x00, 0x01};
	uint8_t data1, data2;
	uint8_t loop_i;
	char buf_tmp[3] = {0};

	/* format r:xx:yyz or w:xx:yyz */
	dev_dbg(&ts->client->dev, "%s: enter\n", __func__);
	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' &&
		(buf[4] == ':' || buf[4] == '\n')) {
		memcpy(buf_tmp, buf + 2, 2);
		buf_tmp[2] = '\0';
		ts->packet_reg_addr = simple_strtol(buf_tmp, NULL, 16);
		if (!ts->packet_reg_addr) {
			if (buf[0] == 'w')
				printk(KERN_WARNING "%s: string to number fail\n", __func__);
			return count;
		}
		if (buf[0] == 'w' && buf[4] == ':' && buf[8] == '\n') {
			memcpy(buf_tmp, buf + 5, 2);
			buf_tmp[2] = '\0';
			data1 = simple_strtol(buf_tmp, NULL, 16);
			memcpy(buf_tmp, buf + 7, 1);
			buf_tmp[1] = '\0';
			data2 = simple_strtol(buf_tmp, NULL, 16);
			printk(KERN_DEBUG
				"write addr: 0x%2.2X, data1: 0x%2.2X data2: 0x%X\n",
				ts->packet_reg_addr, data1, data2);

			cmd[1] = ts->packet_reg_addr;
			cmd[2] = data1;
			cmd[3] = (data2 << 4) | cmd[3];

			if (ts->debug_log_level & 0x1) {
				printk("send ");
				for (loop_i = 0; loop_i < sizeof(cmd); loop_i++)
					printk("0x%2.2X ", cmd[loop_i]);
				printk("\n");
			}

			if (i2c_elan_ktf2k_write(ts->client, cmd, sizeof(cmd)) < 0)
				return count;
		}
	}

    return count;
}

static DEVICE_ATTR(packet, (S_IWUSR|S_IRUGO),
	elan_ktf2k_packet_show,
	elan_ktf2k_packet_store);

static ssize_t elan_ktf2k_debug_level_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	size_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	ret += sprintf(buf, "%d\n", ts->debug_log_level);
	return ret;
}

static ssize_t elan_ktf2k_debug_level_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct elan_ktf2k_ts_data *ts = private_ts;

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts->debug_log_level = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO),
	elan_ktf2k_debug_level_show, elan_ktf2k_debug_level_store);

static ssize_t elan_ktf2k_reset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct elan_ktf2k_ts_data *ts = private_ts;

	disable_irq(ts->client->irq);
	ts->reset();
	elan_ktf2k_ts_setup(ts->client);
	enable_irq(ts->client->irq);

	return count;
}

static DEVICE_ATTR(reset, S_IWUSR, NULL, elan_ktf2k_reset_store);

static int elan_ktf2k_diag_open(struct i2c_client *client, uint8_t diag_command)
{
	struct test_mode_cmd_open cmd[] = {
		{
			.cmd1 = {CMD_W_PKT, 0x9F, 0x01, 0x00, 0x00, 0x01},
			.cmd2 = {CMD_W_PKT, 0x9E, 0x06, 0x00, 0x00, 0x01},
			.cmd3 = {CMD_W_PKT, 0x9F, 0x00, 0x00, 0x00, 0x01},
			.cmd4 = {HELLO_PKT, HELLO_PKT, HELLO_PKT, HELLO_PKT},
			.cmd5 = {0x59, 0xC4, 0x0B, 0x29, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00},
		},
		{
			.cmd1 = {CMD_W_PKT, 0x9F, 0x01, 0x00, 0x00, 0x01},
			.cmd2 = {CMD_W_PKT, 0x9E, 0x00, 0x00, 0x00, 0x01},
			.cmd3 = {CMD_W_PKT, 0x9F, 0x00, 0x00, 0x00, 0x01},
			.cmd4 = {HELLO_PKT, HELLO_PKT, HELLO_PKT, HELLO_PKT},
			.cmd5 = {0x59, 0xC0, 0x0B, 0x29, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00},
		},
		{
			.cmd1 = {CMD_W_PKT, 0x9F, 0x01, 0x00, 0x00, 0x01},
			.cmd2 = {CMD_W_PKT, 0x9E, 0x00, 0x00, 0x00, 0x01},
			.cmd3 = {CMD_W_PKT, 0x9F, 0x00, 0x00, 0x00, 0x01},
			.cmd4 = {HELLO_PKT, HELLO_PKT, HELLO_PKT, HELLO_PKT},
			.cmd5 = {0x59, 0xC0, 0x13, 0x19, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00},
		},
	};
	uint8_t i;

	if (diag_command == TEST_MODE_OFFSET)
		i = 1;
	else if (diag_command == TEST_MODE_OFFSET_Y)
		i = 2;
	else
		i = diag_command - 1;

	if (i2c_elan_ktf2k_write(client, cmd[i].cmd1, sizeof(cmd[i].cmd1)) < 0)
		return -1;
	if (i2c_elan_ktf2k_write(client, cmd[i].cmd2, sizeof(cmd[i].cmd2)) < 0)
		return -1;
	if (i2c_elan_ktf2k_write(client, cmd[i].cmd3, sizeof(cmd[i].cmd3)) < 0)
		return -1;
	msleep(500);
	if (i2c_elan_ktf2k_write(client, cmd[i].cmd4, sizeof(cmd[i].cmd4)) < 0)
		return -1;
	if (i2c_elan_ktf2k_write(client, cmd[i].cmd5, sizeof(cmd[i].cmd5)) < 0)
		return -1;

	return 0;
}

static int elan_ktf2k_diag_open_v2(struct i2c_client *client, uint8_t diag_command)
{
	struct test_mode_cmd_open_v2 cmd[] = {
		{
			.cmd1 = {HELLO_PKT, HELLO_PKT, HELLO_PKT, HELLO_PKT},
			.cmd2 = {0x59, 0xC4, 0x0B, 0x29, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00},
		},
		{
			.cmd1 = {HELLO_PKT, HELLO_PKT, HELLO_PKT, HELLO_PKT},
			.cmd2 = {0x59, 0xC3, 0x0B, 0x29, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00},
		},
		{
			.cmd1 = {HELLO_PKT, HELLO_PKT, HELLO_PKT, HELLO_PKT},
			.cmd2 = {0x59, 0xC4, 0x13, 0x19, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00},
		},
		{
			.cmd1 = {HELLO_PKT, HELLO_PKT, HELLO_PKT, HELLO_PKT},
			.cmd2 = {0x59, 0xC3, 0x13, 0x19, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00},
		},
	};
	uint8_t i;

	if (diag_command == TEST_MODE_DV)
		i = 0;
	else if (diag_command == TEST_MODE_ADC)
		i = 1;
	else if (diag_command == TEST_MODE_DV_Y)
		i = 2;
	else if (diag_command == TEST_MODE_ADC_Y)
		i = 3;

	if (i2c_elan_ktf2k_write(client, cmd[i].cmd1, sizeof(cmd[i].cmd1)) < 0)
		return -1;
	if (i2c_elan_ktf2k_write(client, cmd[i].cmd2, sizeof(cmd[i].cmd2)) < 0)
		return -1;

	return 0;
}

static int elan_ktf2k_diag_close(struct i2c_client *client)
{
	struct test_mode_cmd_close cmd = {
		.cmd1 = {0x9F, 0x00, 0x00, 0x01},
		.cmd2 = {0xA5, 0xA5, 0xA5, 0xA5},
		.cmd3 = {CMD_W_PKT, 0x9F, 0x01, 0x00, 0x00, 0x01},
		.cmd4 = {CMD_W_PKT, 0x9E, 0x06, 0x00, 0x00, 0x01},
		.cmd5 = {CMD_W_PKT, 0x9F, 0x00, 0x00, 0x00, 0x01},
	};

	if (i2c_elan_ktf2k_write(client, cmd.cmd1, sizeof(cmd.cmd1)) < 0)
		return -1;
	if (i2c_elan_ktf2k_write(client, cmd.cmd2, sizeof(cmd.cmd2)) < 0)
		return -1;
	if (i2c_elan_ktf2k_write(client, cmd.cmd3, sizeof(cmd.cmd3)) < 0)
		return -1;
	if (i2c_elan_ktf2k_write(client, cmd.cmd4, sizeof(cmd.cmd4)) < 0)
		return -1;
	if (i2c_elan_ktf2k_write(client, cmd.cmd5, sizeof(cmd.cmd5)) < 0)
		return -1;

	return 0;
}

static int elan_ktf2k_diag_close_v2(struct i2c_client *client)
{
	struct test_mode_cmd_close_v2 cmd = {
		.cmd1 = {0x9F, 0x00, 0x00, 0x01},
		.cmd2 = {0xA5, 0xA5, 0xA5, 0xA5},
	};

	if (i2c_elan_ktf2k_write(client, cmd.cmd1, sizeof(cmd.cmd1)) < 0)
		return -1;
	if (i2c_elan_ktf2k_write(client, cmd.cmd2, sizeof(cmd.cmd2)) < 0)
		return -1;

	return 0;
}

static ssize_t elan_ktf2k_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc = 0;
	size_t count = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;
	uint8_t data[TEST_MODE_SIZE] = {0};
	uint8_t dataY[TEST_MODE_SIZE_Y] = {0};
	uint16_t matrix[19][11];
	uint8_t loop_i, loop_j;
	int x, y, outer, inner;

	if (ts->diag_command != TEST_MODE_DV &&
		ts->diag_command != TEST_MODE_OFFSET &&
		ts->diag_command != TEST_MODE_ADC &&
		ts->diag_command != TEST_MODE_DV_Y &&
		ts->diag_command != TEST_MODE_OFFSET_Y &&
		ts->diag_command != TEST_MODE_ADC_Y)
		return count;

	if (ts->diag_mode != TEST_MODE_OPEN)
		return count;

	x = 11;
	y = 19;
	count += sprintf(buf, "Channel: %d * %d\n", x, y);

	if (ts->diag_command == TEST_MODE_DV_Y ||
		ts->diag_command == TEST_MODE_OFFSET_Y ||
		ts->diag_command == TEST_MODE_ADC_Y) {
		outer = y;
		inner = x;
	} else {
		outer = x;
		inner = y;
	}

	rc = elan_ktf2k_ts_poll(ts->client);
	if (rc < 0)
		goto out;

	for (loop_i = 0; loop_i < outer; loop_i++) {
		if (ts->diag_command == TEST_MODE_DV_Y ||
			ts->diag_command == TEST_MODE_OFFSET_Y ||
			ts->diag_command == TEST_MODE_ADC_Y) {
			if (i2c_elan_ktf2k_read(ts->client, dataY, sizeof(dataY)) < 0)
				goto out;
		} else {
			if (i2c_elan_ktf2k_read(ts->client, data, sizeof(data)) < 0)
				goto out;
		}

		if (ts->diag_command == TEST_MODE_DV_Y ||
			ts->diag_command == TEST_MODE_OFFSET_Y ||
			ts->diag_command == TEST_MODE_ADC_Y) {
			if (dataY[0] != 0x93) {
				printk(KERN_WARNING "%s: not a test data packet: %X\n",
					__func__, dataY[0]);
				goto out;
			}
		} else {
			if (data[0] != 0x93) {
				printk(KERN_WARNING "%s: not a test data packet: %X\n",
					__func__, data[0]);
				goto out;
			}
		}

		if (ts->diag_command == TEST_MODE_DV_Y ||
			ts->diag_command == TEST_MODE_OFFSET_Y ||
			ts->diag_command == TEST_MODE_ADC_Y) {
			for (loop_j = 0; loop_j < inner * 2; loop_j += 2)
				matrix[loop_i][loop_j>>1] = (dataY[loop_j+3] << 8) | dataY[loop_j+4];
		} else {
			for (loop_j = 0; loop_j < inner * 2; loop_j += 2)
				matrix[loop_j>>1][loop_i] = (data[loop_j+3] << 8) | data[loop_j+4];
		}
	}

	for (loop_i = 0; loop_i < y; loop_i++) {
		for (loop_j = 0; loop_j < x; loop_j++)
			count += sprintf(buf + count, "%5d", matrix[loop_i][loop_j]);
		count += sprintf(buf + count, "\n");
	}

out:
	return count;
}

static void elan_ktf2k_diag_print(void)
{
	char buf[256];
	int rc = 0;
	size_t count = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;
	uint8_t data[TEST_MODE_SIZE] = {0};
	uint8_t dataY[TEST_MODE_SIZE_Y] = {0};
	uint16_t matrix[19][11];
	uint8_t loop_i, loop_j;
	int x, y, outer, inner;

	if (ts->diag_command != TEST_MODE_DV &&
		ts->diag_command != TEST_MODE_OFFSET &&
		ts->diag_command != TEST_MODE_ADC &&
		ts->diag_command != TEST_MODE_DV_Y &&
		ts->diag_command != TEST_MODE_OFFSET_Y &&
		ts->diag_command != TEST_MODE_ADC_Y)
		return;

	x = 11;
	y = 19;
	printk(KERN_INFO "Channel: %d * %d\n", x, y);

	if (ts->diag_command == TEST_MODE_DV_Y ||
		ts->diag_command == TEST_MODE_OFFSET_Y ||
		ts->diag_command == TEST_MODE_ADC_Y) {
		outer = y;
		inner = x;
	} else {
		outer = x;
		inner = y;
	}

	rc = elan_ktf2k_ts_poll(ts->client);
	if (rc < 0)
		goto out;

	for (loop_i = 0; loop_i < outer; loop_i++) {
		if (ts->diag_command == TEST_MODE_DV_Y ||
			ts->diag_command == TEST_MODE_OFFSET_Y ||
			ts->diag_command == TEST_MODE_ADC_Y) {
			if (i2c_elan_ktf2k_read(ts->client, dataY, sizeof(dataY)) < 0)
				goto out;
		} else {
			if (i2c_elan_ktf2k_read(ts->client, data, sizeof(data)) < 0)
				goto out;
		}

		if (ts->diag_command == TEST_MODE_DV_Y ||
			ts->diag_command == TEST_MODE_OFFSET_Y ||
			ts->diag_command == TEST_MODE_ADC_Y) {
			if (dataY[0] != 0x93) {
				printk(KERN_WARNING "%s: not a test data packet: %X\n",
					__func__, dataY[0]);
				goto out;
			}
		} else {
			if (data[0] != 0x93) {
				printk(KERN_WARNING "%s: not a test data packet: %X\n",
					__func__, data[0]);
				goto out;
			}
		}

		if (ts->diag_command == TEST_MODE_DV_Y ||
			ts->diag_command == TEST_MODE_OFFSET_Y ||
			ts->diag_command == TEST_MODE_ADC_Y) {
			for (loop_j = 0; loop_j < inner * 2; loop_j += 2)
				matrix[loop_i][loop_j>>1] = (dataY[loop_j+3] << 8) | dataY[loop_j+4];
		} else {
			for (loop_j = 0; loop_j < inner * 2; loop_j += 2)
				matrix[loop_j>>1][loop_i] = (data[loop_j+3] << 8) | data[loop_j+4];
		}
	}

	for (loop_i = 0; loop_i < y; loop_i++) {
		count = 0;
		for (loop_j = 0; loop_j < x; loop_j++)
			count += sprintf(buf + count, "%5d", matrix[loop_i][loop_j]);
		printk(KERN_INFO "%s\n", buf);
	}

out:
	return;
}

static ssize_t elan_ktf2k_diag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	if (buf[0] == '1')
		ts->diag_command = TEST_MODE_DV;
	else if (buf[0] == '2')
		ts->diag_command = TEST_MODE_OFFSET;
	else if (buf[0] == '3')
		ts->diag_command = TEST_MODE_ADC;
	else if (buf[0] == '4')
		ts->diag_command = TEST_MODE_DV_Y;
	else if (buf[0] == '5')
		ts->diag_command = TEST_MODE_OFFSET_Y;
	else if (buf[0] == '6')
		ts->diag_command = TEST_MODE_ADC_Y;
	else
		goto out;

	if (ts->diag_mode == TEST_MODE_OPEN) {
		if (ts->fw_ver < 0x0032 || ts->diag_command == TEST_MODE_OFFSET ||
			ts->diag_command == TEST_MODE_OFFSET_Y)
			rc = elan_ktf2k_diag_close(ts->client);
		else
			rc = elan_ktf2k_diag_close_v2(ts->client);
		if (rc < 0)
			goto out;
	}

	if (buf[1] == 'S' || buf[1] == 'Y') {
		if (buf[1] == 'S')
			disable_irq(ts->client->irq);
		if (ts->fw_ver < 0x0032 || ts->diag_command == TEST_MODE_OFFSET ||
			ts->diag_command == TEST_MODE_OFFSET_Y)
			rc = elan_ktf2k_diag_open(ts->client, ts->diag_command);
		else
			rc = elan_ktf2k_diag_open_v2(ts->client, ts->diag_command);
		if (rc < 0)
			goto out;
		ts->diag_mode = TEST_MODE_OPEN;
	} else if (buf[1] == 'E' || buf[1] == 'N') {
		ts->diag_mode = TEST_MODE_CLOSE;
		if (buf[1] == 'E')
			enable_irq(ts->client->irq);
	}

out:
	return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO),
	elan_ktf2k_diag_show, elan_ktf2k_diag_store);

static struct kobject *android_touch_kobj;

static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "TOUCH_ERR: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		printk(KERN_ERR "TOUCH_ERR: sysfs_create_file gpio failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "TOUCH_ERR: sysfs_create_file vendor failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_packet.attr);
	if (ret) {
		printk(KERN_ERR "TOUCH_ERR: sysfs_create_file packet failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		printk(KERN_ERR "TOUCH_ERR: sysfs_create_file debug_level failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr);
	if (ret) {
		printk(KERN_ERR "TOUCH_ERR: sysfs_create_file reset failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) {
		printk(KERN_ERR "TOUCH_ERR: sysfs_create_file diag failed\n");
		return ret;
	}

	return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_packet.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}

static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 5;

	while ((status = gpio_get_value(ts->intr_gpio)) && retry) {
		dev_dbg(&client->dev, "%s: INT status = %d\n", __func__, status);
		msleep(10);
		retry--;
	}

	dev_dbg(&client->dev, "%s: poll INT status %s\n",
			__func__, status ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf2k_ts_poll(client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t len)
{
	int rc = 0;
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	uint8_t loop_i;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if (ts->debug_log_level & 0x1) {
		printk("send ");
		for (loop_i = 0; loop_i < 4; loop_i++)
			printk("0x%2.2X ", cmd[loop_i]);
		printk("\n");
	}

	rc = i2c_elan_ktf2k_write(client, cmd, 4);
	if (rc < 0)
		return rc;

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		printk(KERN_ERR "TOUCH_ERR: %s: timeout failed!\n", __func__);
		return rc;
	}

	rc = i2c_elan_ktf2k_read(client, buf, len);
	if (rc < 0)
		return rc;

	if (ts->debug_log_level & 0x1) {
		printk("recv ");
		for (loop_i = 0; loop_i < len; loop_i++)
			printk("0x%2.2X ", buf[loop_i]);
		printk("\n");
	}

	if (buf[0] != CMD_S_PKT) {
		printk(KERN_ERR "TOUCH_ERR: %s: not a response packet: %X\n",
			__func__, buf[0]);
		return -EINVAL;
	}

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc = 0;
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	uint8_t data[4] = {0};
	uint8_t loop_i;

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0)
		return -EINVAL;

	rc = i2c_master_recv(client, data, sizeof(data));
	if (rc != sizeof(data)) {
		printk(KERN_INFO "%s: get hello failed! (%d)\n", __func__, rc);
		return rc;
	} else {
		if (ts->debug_log_level & 0x1) {
			printk("recv ");
			for (loop_i = 0; loop_i < sizeof(data); loop_i++)
				printk("0x%2.2X ", data[loop_i]);
			printk("\n");
		}
		for (loop_i = 0; loop_i < sizeof(data); loop_i++)
			if (data[loop_i] != HELLO_PKT)
				return -EINVAL;
	}
	printk(KERN_INFO "Touch: hello\n");

	return 0;
}

static int elan_ktf2k_ts_get_firmware_version(struct i2c_client *client)
{
	int rc = 0;
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	uint8_t major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t data[4] = {0};

	rc = elan_ktf2k_ts_get_data(client, cmd, data, sizeof(data));
	if (rc < 0)
		return rc;

	major = ((data[1] & 0x0f) << 4) | ((data[2] & 0xf0) >> 4);
	minor = ((data[2] & 0x0f) << 4) | ((data[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	printk(KERN_INFO "Touch: firmware version: 0x%4.4x\n", ts->fw_ver);

	return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
	return __hello_packet_handler(client);
}

static int elan_ktf2k_ts_set_packet_state(struct i2c_client *client, int state)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;
	uint8_t cmd[] = {CMD_W_PKT, 0x8E, 0x00, 0x01};
	uint8_t loop_i;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	cmd[2] = state;

	if (ts->debug_log_level & 0x1) {
		printk("send ");
		for (loop_i = 0; loop_i < sizeof(cmd); loop_i++)
			printk("0x%2.2X ", cmd[loop_i]);
		printk("\n");
	}

	rc = i2c_elan_ktf2k_write(client, cmd, sizeof(cmd));
	if (rc < 0)
		return rc;

	return 0;
}

static int elan_ktf2k_ts_get_packet_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x8E, 0x00, 0x01};
	uint8_t data[4] = {0};
	uint8_t state;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	rc = elan_ktf2k_ts_get_data(client, cmd, data, sizeof(data));
	if (rc < 0)
		return rc;

	if (data[1] != 0x8E) {
		printk(KERN_ERR "TOUCH_ERR: not a packet state packet: %X\n", data[1]);
		return -EINVAL;
	}

	state = data[2];
	dev_dbg(&client->dev, "packet state = %s\n",
		state == 1 ?  "Lock" : "Unlock");

	return state;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};
	uint8_t loop_i;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	cmd[1] |= (state << 3);

	if (ts->debug_log_level & 0x1) {
		printk("send ");
		for (loop_i = 0; loop_i < sizeof(cmd); loop_i++)
			printk("0x%2.2X ", cmd[loop_i]);
		printk("\n");
	}

	rc = i2c_elan_ktf2k_write(client, cmd, sizeof(cmd));
	if (rc < 0)
		return rc;

	return 0;
}

static int elan_ktf2k_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t data[4] = {0};
	uint8_t power_state;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	rc = elan_ktf2k_ts_get_data(client, cmd, data, sizeof(data));
	if (rc < 0)
		return rc;

	power_state = data[1];
	if ((power_state >> 4) != 0x5) {
		printk(KERN_ERR "TOUCH_ERR: not a power state packet: %X\n", data[1]);
		return -EINVAL;
	}

	power_state = (power_state & PWR_STATE_MASK) >> 3;
	dev_dbg(&client->dev, "power state = %s\n",
		power_state == PWR_STATE_DEEP_SLEEP ?  "Deep Sleep" : "Normal/Idle");

	return power_state;
}

static int elan_ktf2k_ts_calibration(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;
	uint8_t cmd[4] = {0};
	uint8_t loop_i;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	memset(cmd, CALIB_PKT, sizeof(cmd));

	if (ts->debug_log_level & 0x1) {
		printk("send ");
		for (loop_i = 0; loop_i < sizeof(cmd); loop_i++)
			printk("0x%2.2X ", cmd[loop_i]);
		printk("\n");
	}

	rc = i2c_elan_ktf2k_write(client, cmd, sizeof(cmd));
	if (rc < 0)
		return rc;

	return 0;
}

static int elan_ktf2k_ts_get_finger_state(struct i2c_client *client)
{
    int rc = 0;
    uint8_t cmd[] = {CMD_R_PKT, 0x51, 0x00, 0x01};
    uint8_t data[4] = {0};
	uint8_t finger_state;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

    rc = elan_ktf2k_ts_get_data(client, cmd, data, sizeof(data));
    if (rc < 0)
		return rc;

	if (data[1] != 0x51) {
		printk(KERN_ERR "TOUCH_ERR: not a finger state packet: %X\n", data[1]);
		return -EINVAL;
	}

    finger_state = data[2];
    printk(KERN_INFO "Touch: finger state %d\n", finger_state);

    return finger_state;
}

#if 0
static int elan_ktf2k_ts_repeat(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[4] = {0};
	uint8_t loop_i;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	memset(cmd, REPEAT_PKT, sizeof(cmd));

	if (ts->debug_log_level & 0x1) {
		printk("send ");
		for (loop_i = 0; loop_i < sizeof(cmd); loop_i++)
			printk("0x%2.2X ", cmd[loop_i]);
		printk("\n");
	}

	rc = i2c_elan_ktf2k_write(client, cmd, sizeof(cmd));
	if (rc < 0)
		return rc;

	return 0;
}

static int elan_ktf2k_ts_soft_reset(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[4] = {0};
	uint8_t loop_i;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	memset(cmd, RESET_PKT, sizeof(cmd));

	if (ts->debug_log_level & 0x1) {
		printk("send ");
		for (loop_i = 0; loop_i < sizeof(cmd); loop_i++)
			printk("0x%2.2X ", cmd[loop_i]);
		printk("\n");
	}

	rc = i2c_elan_ktf2k_write(client, cmd, sizeof(cmd));
	if (rc < 0)
		return -EINVAL;

	return 0;
}
#endif

static int i2c_elan_ktf2k_read(struct i2c_client *client,
	uint8_t *buf, size_t len)
{
	int retry;

	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, len);
	for (retry = 0; retry < ELAN_I2C_RETRY_TIMES; retry++) {
		if (i2c_master_recv(client, buf, len) == len)
			break;
		mdelay(3);
	}

	if (retry == ELAN_I2C_RETRY_TIMES) {
		printk(KERN_ERR "TOUCH_ERR: i2c_master_recv retry over %d\n",
			ELAN_I2C_RETRY_TIMES);
		return -EIO;
	}

#if 0
	if (rc != len) {
		dev_err(&client->dev,
			"%s: i2c_master_recv error?!\n", __func__);
		/* software reset */
		elan_ktf2k_ts_soft_reset(client);

		/* re-initial */
		rc = elan_ktf2k_ts_setup(client);
		if (gpio_get_value(ts->intr_gpio) == 0)
			queue_work(ts->elan_wq, &ts->work);
		return -EINVAL;
	}
#endif

	return 0;
}

static int i2c_elan_ktf2k_write(struct i2c_client *client,
	uint8_t *buf, size_t len)
{
	int retry;

	if (buf == NULL)
		return -EINVAL;

	for (retry = 0; retry < ELAN_I2C_RETRY_TIMES; retry++) {
		if (i2c_master_send(client, buf, len) == len)
			break;
		mdelay(3);
	}

	if (retry == ELAN_I2C_RETRY_TIMES) {
		printk(KERN_ERR "TOUCH_ERR: i2c_master_send retry over %d\n",
			ELAN_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	static unsigned report_time;
	unsigned report_time2;
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y, z, w;
	uint8_t i, idx;
	uint8_t finger_count, finger_pressed;
	uint8_t finger_press_changed, finger_release_changed;

	finger_count = buf[IDX_NUM] & 0x7;
	finger_pressed = buf[IDX_NUM] >> 3;
	if (!ts->first_pressed && finger_pressed != ts->finger_pressed) {
		finger_press_changed = finger_pressed ^ ts->finger_pressed;
		finger_release_changed = finger_press_changed & (~finger_pressed);
		finger_press_changed &= finger_pressed;
		ts->finger_pressed = finger_pressed;
	}

	if (finger_count == 0) {
		if (!ts->first_pressed) {
			for (i = 0; i < 5; i++)
				if ((finger_release_changed >> i) & 0x1)
					printk(KERN_INFO "E%d@%d, %d\n", i + 1,
						ts->last_finger_data[i][0],
						ts->last_finger_data[i][1]);
			ts->first_pressed = 1;
		}
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
		input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
#else
		input_report_abs(idev, ABS_MT_AMPLITUDE, 0);
		input_report_abs(idev, ABS_MT_POSITION, BIT(31));
#endif
		if (ts->debug_log_level & 0x2)
			printk(KERN_INFO "Finger leave\n");
	} else {
		uint8_t reported = 0;

		for (i = 0, idx = IDX_FINGER; i < 5; i++, idx += 3) {
			if (!(((finger_pressed | finger_release_changed) >> i) & 0x1))
				continue;
			if (!ts->first_pressed &&
				((finger_release_changed >> i) & 0x01)) {
				printk(KERN_INFO "E%d@%d, %d\n", i + 1,
					ts->last_finger_data[i][0], ts->last_finger_data[i][1]);
				continue;
			}
			elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
			if (!ts->first_pressed &&
				((finger_press_changed >> i) & 0x01))
				printk(KERN_INFO "S%d@%d, %d\n", i + 1, x, y);
			if (!ts->first_pressed) {
				ts->last_finger_data[i][0] = x;
				ts->last_finger_data[i][1] = y;
			}
			if (ts->fw_ver < 0x0026)
				z = w = 2;
			else {
				if (i % 2)
					z = w = buf[IDX_WIDTH + (i >> 1)] & 0xf;
				else
					z = w = (buf[IDX_WIDTH + (i >> 1)] >> 4) & 0xf;
			}
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, w);
			input_report_abs(idev, ABS_MT_POSITION_X, x);
			input_report_abs(idev, ABS_MT_POSITION_Y, y);
			input_mt_sync(idev);
#else
			input_report_abs(idev, ABS_MT_AMPLITUDE, z << 16 | w);
			input_report_abs(idev, ABS_MT_POSITION,
				(reported == finger_count - 1 ? BIT(31) : 0) | x << 16 | y);
#endif
			if (ts->debug_log_level & 0x2)
				printk(KERN_INFO "Finger %d=> X:%d, Y:%d w:%d z:%d F:%d\n",
					i + 1, x, y, w, z, finger_count);
			reported++;
		}
	}
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_sync(ts->input_dev);
#endif

	if (ts->debug_log_level & 0x4) {
		report_time2 = jiffies;
		printk(KERN_INFO "%s: report time = %d\n", __func__,
			jiffies_to_msecs(report_time2 - report_time));
		report_time = report_time2;
	}

	return;
}

static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	int rc = 0;
	struct elan_ktf2k_ts_data *ts =
		container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t data[IDX_PACKET_SIZE] = {0};
	uint8_t msg_byte_num = IDX_PACKET_SIZE;
	uint8_t loop_i;

	/* this means that we have already serviced it */
	if (gpio_get_value(ts->intr_gpio)) {
		enable_irq(ts->client->irq);
		return;
	}

	if (ts->diag_mode == TEST_MODE_OPEN) {
		elan_ktf2k_diag_print();
		enable_irq(ts->client->irq);
		return;
	}

	rc = i2c_elan_ktf2k_read(ts->client, data, sizeof(data));

	switch (data[0]) {
	case RPT_PKT:
		elan_ktf2k_ts_report_data(ts->client, data);
		break;
	case NOISE_PKT:
		if (data[1] == 0x41)
			printk(KERN_INFO "Touch: env noisy\n");
		else
			printk(KERN_INFO "Touch: env normal\n");
		msg_byte_num = 2;
		break;
	case HELLO_PKT:
		printk(KERN_INFO "Touch: hello\n");
		msg_byte_num = 4;
		break;
	case CALIB_DONE:
		printk(KERN_INFO "Touch: calibration done\n");
		msg_byte_num = 4;
		break;
	default:
		printk(KERN_ERR "TOUCH_ERR: unknown packet type: %X\n", data[0]);
		break;
	}

	if (ts->debug_log_level & 0x1) {
		for (loop_i = 0; loop_i < msg_byte_num; loop_i++)
			printk("0x%2.2X ", data[loop_i]);
		printk("\n");
	}

	enable_irq(ts->client->irq);

	return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf2k_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;

	disable_irq_nosync(client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,
			IRQF_TRIGGER_LOW, client->name, ts);
	if (err)
		dev_err(&client->dev, "TOUCH_ERR: request_irq %d failed\n",
			client->irq);

	return err;
}

static int elan_ktf2k_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	struct elan_ktf2k_i2c_platform_data *pdata;
	struct elan_ktf2k_ts_data *ts;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "TOUCH_ERR: need I2C_FUNC_I2C\n");
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "TOUCH_ERR: allocate elan_ktf2k_ts_data failed\n");
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		printk(KERN_ERR "TOUCH_ERR: create workqueue failed\n");
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (likely(pdata != NULL)) {
		ts->power = pdata->power;
		ts->reset = pdata->reset;
		ts->intr_gpio = pdata->intr_gpio;
	}

	if (ts->power)
		ts->power(1);

	err = elan_ktf2k_ts_setup(client);
	if (err < 0) {
		printk(KERN_INFO "No Elan chip inside\n");
		err = -ENODEV;
		goto err_detect_failed;
	}

	elan_ktf2k_ts_get_firmware_version(client);

/*
	if (pdata) {
		while (pdata->version > ts->fw_ver) {
			printk(KERN_INFO "%s: old tp detected, "
					"panel version = 0x%X\n",
					__func__, ts->fw_ver);
			pdata++;
		}
	}
*/

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "TOUCH_ERR: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "elan-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		pdata->abs_x_min,  pdata->abs_x_max,
		ELAN_TS_FUZZ, ELAN_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		pdata->abs_y_min,  pdata->abs_y_max,
		ELAN_TS_FUZZ, ELAN_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
		0, 15,
		ELAN_TS_FUZZ, ELAN_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
		0, 20,
		ELAN_TS_FUZZ, ELAN_TS_FLAT);
#ifndef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE,
		0, ((15 << 16) | 20), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,
		0, (BIT(31) | (pdata->abs_x_max << 16) | pdata->abs_y_max), 0, 0);
#endif

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"TOUCH_ERR: unable to register %s input device\n",
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	elan_ktf2k_ts_register_interrupt(ts->client);

	/* checking the interrupt to avoid missing any interrupt */
	if (gpio_get_value(ts->intr_gpio) == 0) {
		printk(KERN_INFO "TOUCH: handle missed interrupt\n");
		elan_ktf2k_ts_irq_handler(client->irq, ts);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = elan_ktf2k_ts_early_suspend;
	ts->early_suspend.resume = elan_ktf2k_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	private_ts = ts;

	elan_ktf2k_touch_sysfs_init();

	dev_info(&client->dev, "Start touchscreen %s in interrupt mode\n",
		ts->input_dev->name);

	return 0;

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);

err_create_wq_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf2k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc = 0;
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	printk(KERN_INFO "%s: enter\n", __func__);

	disable_irq(client->irq);

	rc = cancel_work_sync(&ts->work);
	if (rc)
		enable_irq(client->irq);

	ts->first_pressed = 0;
	ts->finger_pressed = 0;

	if (elan_ktf2k_ts_set_packet_state(client, 1) < 0)
		printk(KERN_ERR "TOUCH_ERR: send lock normal packet failed\n");

	if (elan_ktf2k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP) < 0)
		printk(KERN_ERR "TOUCH_ERR: send deep sleep failed\n");

	return 0;
}

static int elan_ktf2k_ts_resume(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	uint8_t touch = 0;

	printk(KERN_INFO "%s: enter\n", __func__);

	if (elan_ktf2k_ts_set_power_state(client, PWR_STATE_NORMAL) < 0)
		printk(KERN_ERR "TOUCH_ERR: send wakeup failed\n");

	msleep(50);

	if (elan_ktf2k_ts_get_power_state(client) != PWR_STATE_NORMAL)
		printk(KERN_ERR "TOUCH_ERR: wakeup tp failed!\n");

#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(ts->input_dev);
#else
	input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
	input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif

	touch = elan_ktf2k_ts_get_finger_state(client);
	if (touch < 0)
		printk(KERN_ERR "TOUCH_ERR: get finger state failed!\n");

	if (elan_ktf2k_ts_set_packet_state(client, 0) < 0)
		printk(KERN_ERR "TOUCH_ERR: send unlock normal packet failed\n");
	if (elan_ktf2k_ts_get_packet_state(client) != 0)
		printk(KERN_ERR "TOUCH_ERR: unlock normal packet failed!\n");

	if (touch == 0)
		elan_ktf2k_ts_calibration(client);

	enable_irq(client->irq);

/*
	if (gpio_get_value(ts->intr_gpio) == 0)
	{
		printk(KERN_INFO "%s: handle missed interrupt\n", __func__);
		elan_ktf2k_ts_irq_handler(client->irq, ts);
	}
*/

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts;
	ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);
	elan_ktf2k_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void elan_ktf2k_ts_late_resume(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts;
	ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);
	elan_ktf2k_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id elan_ktf2k_ts_id[] = {
	{ ELAN_KTF2K_NAME, 0 },
	{ }
};

static struct i2c_driver ektf2k_ts_driver = {
	.probe		= elan_ktf2k_ts_probe,
	.remove		= elan_ktf2k_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= elan_ktf2k_ts_suspend,
	.resume		= elan_ktf2k_ts_resume,
#endif
	.id_table	= elan_ktf2k_ts_id,
	.driver		= {
		.name = ELAN_KTF2K_NAME,
	},
};

static int __devinit elan_ktf2k_ts_init(void)
{
	printk(KERN_INFO "%s\n", __func__);
	return i2c_add_driver(&ektf2k_ts_driver);
}

static void __exit elan_ktf2k_ts_exit(void)
{
	i2c_del_driver(&ektf2k_ts_driver);
	return;
}

module_init(elan_ktf2k_ts_init);
module_exit(elan_ktf2k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2000 Touchscreen Driver");
MODULE_LICENSE("GPL");
