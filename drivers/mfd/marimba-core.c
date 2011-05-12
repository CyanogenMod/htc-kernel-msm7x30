/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Qualcomm Marimba Core Driver
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <linux/i2c.h>
#include <linux/mfd/marimba.h>

#define MARIMBA_MODE				0x00

static int marimba_shadow[MARIMBA_NUM_CHILD][0xff];

static bool inuse;

struct marimba marimba_modules[MARIMBA_NUM_CHILD + 1];

#define MARIMBA_VERSION_REG		0x11

#ifdef CONFIG_I2C_SSBI
#define NUM_ADD	MARIMBA_NUM_CHILD
#else
#define NUM_ADD	(MARIMBA_NUM_CHILD - 1)
#endif

/**
 * marimba_ssbi_write - Writes a n bit TSADC register in Marimba
 * @param mariba: marimba structure pointer passed by client
 * @param reg: register address
 * @param value: buffer to be written
 * @param len: num of bytes
 * @returns result of the operation.
 */
int marimba_ssbi_write(struct marimba *marimba, u16 reg , u8 *value, int len)
{
	struct i2c_msg *msg;
	int ret;

	marimba = &marimba_modules[marimba->mod_id];

	mutex_lock(&marimba->xfer_lock);

	msg = &marimba->xfer_msg[0];
	msg->addr = reg;
	msg->flags = 0x0;
	msg->buf = value;
	msg->len = len;

	ret = i2c_transfer(marimba->client->adapter, marimba->xfer_msg, 1);

	mutex_unlock(&marimba->xfer_lock);

	return ret;
}
EXPORT_SYMBOL(marimba_ssbi_write);

/**
 * marimba_ssbi_read - Reads a n bit TSADC register in Marimba
 * @param marimba: marimba structure pointer passed by client
 * @param reg: register address
 * @param value: ssbi read of the register to be stored
 * @param len: num of bytes
 *
 * @returns result of the operation.
*/
int marimba_ssbi_read(struct marimba *marimba, u16 reg, u8 *value, int len)
{
	struct i2c_msg *msg;
	int ret;

	marimba = &marimba_modules[marimba->mod_id];

	mutex_lock(&marimba->xfer_lock);

	msg = &marimba->xfer_msg[0];
	msg->addr = reg;
	msg->flags = I2C_M_RD;
	msg->buf = value;
	msg->len = len;

	ret = i2c_transfer(marimba->client->adapter, marimba->xfer_msg, 1);

	mutex_unlock(&marimba->xfer_lock);

	return ret;
}
EXPORT_SYMBOL(marimba_ssbi_read);

/**
 * marimba_write_bit_mask - Sets n bit register using bit mask
 * @param mariba: marimba structure pointer passed by client
 * @param reg: register address
 * @param value: buffer to be written to the registers
 * @param num_bytes: n bytes to write
 * @param mask: bit mask corresponding to the registers
 *
 * @returns result of the operation.
 */
int marimba_write_bit_mask(struct marimba *marimba, u8 reg, u8 *value,
						unsigned num_bytes, u8 mask)
{
	int ret, i;
	struct i2c_msg *msg;
	u8 data[num_bytes + 1];
	u8 mask_value[num_bytes];

	marimba = &marimba_modules[marimba->mod_id];

	mutex_lock(&marimba->xfer_lock);

	for (i = 0; i < num_bytes; i++)
		mask_value[i] = (marimba_shadow[marimba->mod_id][reg + i]
					& ~mask) | (value[i] & mask);

	msg = &marimba->xfer_msg[0];
	msg->addr = marimba->client->addr;
	msg->flags = 0;
	msg->len = num_bytes + 1;
	msg->buf = data;
	data[0] = reg;
	memcpy(data+1, mask_value, num_bytes);

	ret = i2c_transfer(marimba->client->adapter, marimba->xfer_msg, 1);

	/* Try again if the write fails */
	if (ret != 1)
		ret = i2c_transfer(marimba->client->adapter,
						marimba->xfer_msg, 1);

	if (ret == 1) {
		for (i = 0; i < num_bytes; i++)
			marimba_shadow[marimba->mod_id][reg + i]
							= mask_value[i];
	}

	mutex_unlock(&marimba->xfer_lock);

	return ret;
}
EXPORT_SYMBOL(marimba_write_bit_mask);

/**
 * marimba_write - Sets n bit register in Marimba
 * @param marimba: marimba structure pointer passed by client
 * @param reg: register address
 * @param value: buffer values to be written
 * @param num_bytes: n bytes to write
 *
 * @returns result of the operation.
 */
int marimba_write(struct marimba *marimba, u8 reg, u8 *value,
							unsigned num_bytes)
{
	return marimba_write_bit_mask(marimba, reg, value, num_bytes, 0xff);
}
EXPORT_SYMBOL(marimba_write);

/**
 * marimba_read_bit_mask - Reads a n bit register based on bit mask
 * @param marimba: marimba structure pointer passed by client
 * @param reg: register address
 * @param value: i2c read of the register to be stored
 * @param num_bytes: n bytes to be read.
 * @param mask: bit mask concerning its register
 *
 * @returns result of the operation.
*/
int marimba_read_bit_mask(struct marimba *marimba, u8 reg, u8 *value,
						unsigned num_bytes, u8 mask)
{
	int ret, i;

	struct i2c_msg *msg;

	marimba = &marimba_modules[marimba->mod_id];

	mutex_lock(&marimba->xfer_lock);

	msg = &marimba->xfer_msg[0];
	msg->addr = marimba->client->addr;
	msg->len = 1;
	msg->flags = 0;
	msg->buf = &reg;

	msg = &marimba->xfer_msg[1];
	msg->addr = marimba->client->addr;
	msg->len = num_bytes;
	msg->flags = I2C_M_RD;
	msg->buf = value;

	ret = i2c_transfer(marimba->client->adapter, marimba->xfer_msg, 2);

	/* Try again if read fails first time */
	if (ret != 2)
		ret = i2c_transfer(marimba->client->adapter,
						marimba->xfer_msg, 2);

	if (ret == 2) {
		for (i = 0; i < num_bytes; i++) {
			marimba_shadow[marimba->mod_id][reg + i] = value[i];
			value[i] &= mask;
		}
	}

	mutex_unlock(&marimba->xfer_lock);

	return ret;
}
EXPORT_SYMBOL(marimba_read_bit_mask);

/**
 * marimba_read - Reads n bit registers in Marimba
 * @param marimba: marimba structure pointer passed by client
 * @param reg: register address
 * @param value: i2c read of the register to be stored
 * @param num_bytes: n bytes to read.
 * @param mask: bit mask concerning its register
 *
 * @returns result of the operation.
*/
int marimba_read(struct marimba *marimba, u8 reg, u8 *value, unsigned num_bytes)
{
	return marimba_read_bit_mask(marimba, reg, value, num_bytes, 0xff);
}
EXPORT_SYMBOL(marimba_read);

int timpani_read(struct marimba *marimba, u8 reg, u8 *value, unsigned num_bytes)
{
	return marimba_read_bit_mask(marimba, reg, value, num_bytes, 0xff);
}
EXPORT_SYMBOL(timpani_read);

int timpani_write(struct marimba *marimba, u8 reg,
					u8 *value, unsigned num_bytes)
{
	return marimba_write_bit_mask(marimba, reg, value, num_bytes, 0xff);
}
EXPORT_SYMBOL(timpani_write);

static int cur_codec_type = -1;

int adie_get_detected_codec_type(void)
{
	return cur_codec_type;
}
EXPORT_SYMBOL(adie_get_detected_codec_type);

static struct device *
add_numbered_child(unsigned chip, const char *name, int num,
					void *pdata, unsigned pdata_len)
{
	struct platform_device *pdev;
	struct marimba  *marimba = &marimba_modules[chip];
	int status = 0;

	pdev = platform_device_alloc(name, num);
	if (!pdev) {
		status = -ENOMEM;
		return ERR_PTR(status);
	}

	pdev->dev.parent = &marimba->client->dev;

	marimba->mod_id = chip;

	platform_set_drvdata(pdev, marimba);

	if (pdata) {
		status = platform_device_add_data(pdev, pdata, pdata_len);
		if (status < 0)
			goto err;
	}

	status = platform_device_add(pdev);
	if (status < 0)
		goto err;

err:
	if (status < 0) {
		platform_set_drvdata(pdev, NULL);
		platform_device_put(pdev);
		dev_err(&marimba->client->dev, "can't add %s dev \n", name);
		return ERR_PTR(status);
	}
	return &pdev->dev;
}

static inline struct device *add_child(unsigned chip, const char *name,
						void *pdata, unsigned pdata_len)
{
	return add_numbered_child(chip, name, -1, pdata, pdata_len);
}

static int marimba_add_child(struct marimba_platform_data *pdata,
					u8 driver_data)
{
	struct device	*child;

	/* Add BT,FM and TS for Marimba only */
	if (driver_data == MARIMBA_ID) {
		child = add_child(MARIMBA_SLAVE_ID_FM, "marimba_fm",
					  pdata->fm, sizeof(*pdata->fm));
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	/* Add Codec for Marimba and Timpani */
	if (driver_data == MARIMBA_ID) {
		child = add_child(MARIMBA_SLAVE_ID_CDC, "marimba_codec",
					  pdata->codec, sizeof(*pdata->codec));
		if (IS_ERR(child))
			return PTR_ERR(child);
	} else if (driver_data == TIMPANI_ID) {
		child = add_child(MARIMBA_SLAVE_ID_CDC, "timpani_codec",
					  pdata->codec, sizeof(*pdata->codec));
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

#ifdef CONFIG_I2C_SSBI
	if (pdata->tsadc != NULL) {
		child = add_child(MARIMBA_ID_TSADC, "marimba_tsadc",
				pdata->tsadc, sizeof(*pdata->tsadc));
		if (IS_ERR(child))
			return PTR_ERR(child);
	}
#endif

	return 0;
}

static int get_codec_type(void)
{

	struct marimba *marimba = &marimba_modules[MARIMBA_SLAVE_ID_MARIMBA];
	u8 rd_val;
	int ret;

	marimba->mod_id = MARIMBA_SLAVE_ID_MARIMBA;
	/* Enable the Mode for Marimba/Timpani */
	ret = marimba_read(marimba, MARIMBA_VERSION_REG, &rd_val, 1);
	pr_info("%s, marimba_read = %d\n", __func__, ret);

	if (ret >= 0) {
		if (rd_val & 0x20) {
			pr_info("%s, codec is TIMPANI\n", __func__);
			return TIMPANI_ID;
		} else {
			ret = marimba_read(marimba, 0x02, &rd_val, 1);
			if (rd_val == 0x77) {
				pr_info("%s, codec is TIMPANI\n", __func__);
				return MARIMBA_ID;
			}
		}
	}

	return -ENODEV;
}

static void marimba_init_reg(struct i2c_client *client, u8 driver_data)
{
	struct marimba_platform_data *pdata = client->dev.platform_data;
	struct marimba *marimba = &marimba_modules[MARIMBA_SLAVE_ID_MARIMBA];
	int i, rc;
	u8 buf[1], reg_version;
	static struct vreg *vreg_s2;

	buf[0] = 0x10;

	marimba->mod_id = MARIMBA_SLAVE_ID_MARIMBA;
	/* Enable the Mode for Marimba/Timpani */
	marimba_write(marimba, MARIMBA_MODE, buf, 1);

	for (i = 1; i < MARIMBA_NUM_CHILD; i++)
		marimba_write(marimba, i , &pdata->slave_id[i], 1);

	marimba_read(marimba, MARIMBA_VERSION_REG, &reg_version, 1);

	vreg_s2 = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_s2)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)/n",
			__func__, PTR_ERR(vreg_s2));
		return;
	}

	if (reg_version >= 1) {
		rc = vreg_disable(vreg_s2);
		if (rc) {
			printk(KERN_ERR "%s: return val: %d \n",
				__func__, rc);
		}
	}
}

static int marimba_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct marimba_platform_data *pdata = client->dev.platform_data;
	struct i2c_adapter *ssbi_adap;
	struct marimba *marimba;
	int i, status;
	pr_info("%s\n", __func__);

	if (!pdata) {
		dev_dbg(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		dev_dbg(&client->dev, "can't talk I2C?\n");
		return -EIO;
	}

	if (inuse) {
		dev_dbg(&client->dev, "driver already in use\n");
		return -EBUSY;
	}

	/* First, identify the codec type */
	if (pdata->marimba_setup != NULL)
		pdata->marimba_setup();

	marimba = &marimba_modules[0];
			marimba->client = client;
	mutex_init(&marimba->xfer_lock);

	pr_info("%s, get_codec_type()= %d, driver_data = %s, %d\n", __func__, get_codec_type(), id->name, (int)id->driver_data);

	if (get_codec_type() != (int)id->driver_data) {
		if (pdata->marimba_shutdown != NULL)
			pdata->marimba_shutdown();
		status = -ENODEV;
		mutex_destroy(&marimba->xfer_lock);
		goto fail;
	} else {
		cur_codec_type = (int)id->driver_data;
		dev_dbg(&client->dev, "Device %d available\n",
			(int)id->driver_data);
	}

	for (i = 1; i <= NUM_ADD; i++) {

		/* Skip adding BT/FM for Timpani */
		if (i == 1 && id->driver_data == TIMPANI_ID)
			i++;
		marimba = &marimba_modules[i];
		if (i != MARIMBA_ID_TSADC)
			marimba->client = i2c_new_dummy(client->adapter,
						pdata->slave_id[i]);
		else {
			ssbi_adap = i2c_get_adapter(MARIMBA_SSBI_ADAP);
			marimba->client = i2c_new_dummy(ssbi_adap,
						pdata->slave_id[i]);
		}
		if (!marimba->client) {
			dev_err(&marimba->client->dev,
				"can't attach client %d\n", i);
			status = -ENOMEM;
			goto fail;
		}
		strlcpy(marimba->client->name, id->name,
			sizeof(marimba->client->name));

		mutex_init(&marimba->xfer_lock);
	}

	inuse = true;

	marimba_init_reg(client, id->driver_data);

	status = marimba_add_child(pdata, id->driver_data);

	return 0;

fail:
	return status;
}

static int __devexit marimba_remove(struct i2c_client *client)
{
	int i;
	struct marimba_platform_data *pdata;

	pdata = client->dev.platform_data;
	for (i = 0; i <= MARIMBA_NUM_CHILD; i++) {
		struct marimba *marimba = &marimba_modules[i];

		if (marimba->client && marimba->client != client)
			i2c_unregister_device(marimba->client);

		marimba_modules[i].client = NULL;
	}

	if (pdata->marimba_shutdown != NULL)
		pdata->marimba_shutdown();

	return 0;
}

static struct i2c_device_id marimba_id_table[] = {
	{"marimba", MARIMBA_ID},
	{"timpani", TIMPANI_ID},
	{}
};
MODULE_DEVICE_TABLE(i2c, marimba_id_table);

static struct i2c_driver marimba_driver = {
		.driver			= {
			.owner		=	THIS_MODULE,
			.name		= 	"marimba-core",
		},
		.id_table		=	marimba_id_table,
		.probe			=	marimba_probe,
		.remove			=	__devexit_p(marimba_remove),
};

static int __init marimba_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&marimba_driver);
}
module_init(marimba_init);

static void __exit marimba_exit(void)
{
	i2c_del_driver(&marimba_driver);
}
module_exit(marimba_exit);

MODULE_DESCRIPTION("Marimba Top level Driver");
MODULE_ALIAS("platform:marimba-core");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
