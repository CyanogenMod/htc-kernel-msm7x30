/*
 * ALSA SoC TWL6040 codec driver
 *
 * Author:	 Misael Lopez Cruz <x0052729@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "twl6040.h"

#define TWL6040_RATES		SNDRV_PCM_RATE_8000_96000
#define TWL6040_FORMATS	(SNDRV_PCM_FMTBIT_S32_LE)

#define TWL6040_OUTHS_0dB 0x00
#define TWL6040_OUTHS_M30dB 0x0F
#define TWL6040_OUTHF_0dB 0x03
#define TWL6040_OUTHF_M52dB 0x1D

#define TWL6040_RAMP_NONE	0
#define TWL6040_RAMP_UP		1
#define TWL6040_RAMP_DOWN	2

#define TWL6040_HSL_VOL_MASK	0x0F
#define TWL6040_HSL_VOL_SHIFT	0
#define TWL6040_HSR_VOL_MASK	0xF0
#define TWL6040_HSR_VOL_SHIFT	4
#define TWL6040_HF_VOL_MASK	0x1F
#define TWL6040_HF_VOL_SHIFT	0

struct twl6040_output {
	u16 active;
	u16 left_vol;
	u16 right_vol;
	u16 left_step;
	u16 right_step;
	unsigned int step_delay;
	u16 ramp;
	u16 mute;
	struct completion ramp_done;
};

struct twl6040_jack_data {
	struct snd_soc_jack *jack;
	int report;
};

/* codec private data */
struct twl6040_data {
	int audpwron;
	int naudint;
	int codec_powered;
	int pll;
	int non_lp;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	struct completion ready;
	struct twl6040_jack_data hs_jack;
	struct snd_soc_codec *codec;
	struct workqueue_struct *workqueue;
	struct delayed_work delayed_work;
	struct mutex mutex;
	struct twl6040_output headset;
	struct twl6040_output handsfree;
	struct workqueue_struct *hf_workqueue;
	struct workqueue_struct *hs_workqueue;
	struct delayed_work hs_delayed_work;
	struct delayed_work hf_delayed_work;
};

/*
 * twl6040 register cache & default register settings
 */
static const u8 twl6040_reg[TWL6040_CACHEREGNUM] = {
	0x00, /* not used		0x00	*/
	0x4B, /* TWL6040_ASICID (ro)	0x01	*/
	0x00, /* TWL6040_ASICREV (ro)	0x02	*/
	0x00, /* TWL6040_INTID		0x03	*/
	0x00, /* TWL6040_INTMR		0x04	*/
	0x00, /* TWL6040_NCPCTRL	0x05	*/
	0x00, /* TWL6040_LDOCTL		0x06	*/
	0x60, /* TWL6040_HPPLLCTL	0x07	*/
	0x00, /* TWL6040_LPPLLCTL	0x08	*/
	0x4A, /* TWL6040_LPPLLDIV	0x09	*/
	0x00, /* TWL6040_AMICBCTL	0x0A	*/
	0x00, /* TWL6040_DMICBCTL	0x0B	*/
	0x18, /* TWL6040_MICLCTL	0x0C	- No input selected on Left Mic */
	0x18, /* TWL6040_MICRCTL	0x0D	- No input selected on Right Mic */
	0x00, /* TWL6040_MICGAIN	0x0E	*/
	0x1B, /* TWL6040_LINEGAIN	0x0F	*/
	0x00, /* TWL6040_HSLCTL		0x10	*/
	0x00, /* TWL6040_HSRCTL		0x11	*/
	0x00, /* TWL6040_HSGAIN		0x12	*/
	0x00, /* TWL6040_EARCTL		0x13	*/
	0x00, /* TWL6040_HFLCTL		0x14	*/
	0x00, /* TWL6040_HFLGAIN	0x15	*/
	0x00, /* TWL6040_HFRCTL		0x16	*/
	0x00, /* TWL6040_HFRGAIN	0x17	*/
	0x00, /* TWL6040_VIBCTLL	0x18	*/
	0x00, /* TWL6040_VIBDATL	0x19	*/
	0x00, /* TWL6040_VIBCTLR	0x1A	*/
	0x00, /* TWL6040_VIBDATR	0x1B	*/
	0x00, /* TWL6040_HKCTL1		0x1C	*/
	0x00, /* TWL6040_HKCTL2		0x1D	*/
	0x00, /* TWL6040_GPOCTL		0x1E	*/
	0x00, /* TWL6040_ALB		0x1F	*/
	0x00, /* TWL6040_DLB		0x20	*/
	0x00, /* not used		0x21	*/
	0x00, /* not used		0x22	*/
	0x00, /* not used		0x23	*/
	0x00, /* not used		0x24	*/
	0x00, /* not used		0x25	*/
	0x00, /* not used		0x26	*/
	0x00, /* not used		0x27	*/
	0x00, /* TWL6040_TRIM1		0x28	*/
	0x00, /* TWL6040_TRIM2		0x29	*/
	0x00, /* TWL6040_TRIM3		0x2A	*/
	0x00, /* TWL6040_HSOTRIM	0x2B	*/
	0x00, /* TWL6040_HFOTRIM	0x2C	*/
	0x09, /* TWL6040_ACCCTL		0x2D	*/
	0x00, /* TWL6040_STATUS (ro)	0x2E	*/
};

/*
 * twl6040 vio/gnd registers:
 * registers under vio/gnd supply can be accessed
 * before the power-up sequence, after NRESPWRON goes high
 */
static const int twl6040_vio_reg[TWL6040_VIOREGNUM] = {
	TWL6040_REG_ASICID,
	TWL6040_REG_ASICREV,
	TWL6040_REG_INTID,
	TWL6040_REG_INTMR,
	TWL6040_REG_NCPCTL,
	TWL6040_REG_LDOCTL,
	TWL6040_REG_AMICBCTL,
	TWL6040_REG_DMICBCTL,
	TWL6040_REG_HKCTL1,
	TWL6040_REG_HKCTL2,
	TWL6040_REG_GPOCTL,
	TWL6040_REG_TRIM1,
	TWL6040_REG_TRIM2,
	TWL6040_REG_TRIM3,
	TWL6040_REG_HSOTRIM,
	TWL6040_REG_HFOTRIM,
	TWL6040_REG_ACCCTL,
	TWL6040_REG_STATUS,
};

/*
 * twl6040 vdd/vss registers:
 * registers under vdd/vss supplies can only be accessed
 * after the power-up sequence
 */
static const int twl6040_vdd_reg[TWL6040_VDDREGNUM] = {
	TWL6040_REG_HPPLLCTL,
	TWL6040_REG_LPPLLCTL,
	TWL6040_REG_LPPLLDIV,
	TWL6040_REG_MICLCTL,
	TWL6040_REG_MICRCTL,
	TWL6040_REG_MICGAIN,
	TWL6040_REG_LINEGAIN,
	TWL6040_REG_HSLCTL,
	TWL6040_REG_HSRCTL,
	TWL6040_REG_HSGAIN,
	TWL6040_REG_EARCTL,
	TWL6040_REG_HFLCTL,
	TWL6040_REG_HFLGAIN,
	TWL6040_REG_HFRCTL,
	TWL6040_REG_HFRGAIN,
	TWL6040_REG_VIBCTLL,
	TWL6040_REG_VIBDATL,
	TWL6040_REG_VIBCTLR,
	TWL6040_REG_VIBDATR,
	TWL6040_REG_ALB,
	TWL6040_REG_DLB,
};

/*
 * read twl6040 register cache
 */
static inline unsigned int twl6040_read_reg_cache(struct snd_soc_codec *codec,
						unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL6040_CACHEREGNUM)
		return -EIO;

	return cache[reg];
}

/*
 * write twl6040 register cache
 */
static inline void twl6040_write_reg_cache(struct snd_soc_codec *codec,
						u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL6040_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * read from twl6040 hardware register
 */
static int twl6040_read_reg_volatile(struct snd_soc_codec *codec,
			unsigned int reg)
{
	u8 value;

	if (reg >= TWL6040_CACHEREGNUM)
		return -EIO;

	twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &value, reg);
	twl6040_write_reg_cache(codec, reg, value);

	return value;
}

/*
 * write to the twl6040 register space
 */
static int twl6040_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int value)
{
	if (reg >= TWL6040_CACHEREGNUM)
		return -EIO;

	twl6040_write_reg_cache(codec, reg, value);
	return twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, value, reg);
}

static void twl6040_init_vio_regs(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	int reg, i;

	/* allow registers to be accessed by i2c */
	twl6040_write(codec, TWL6040_REG_ACCCTL, cache[TWL6040_REG_ACCCTL]);

	for (i = 0; i < TWL6040_VIOREGNUM; i++) {
		reg = twl6040_vio_reg[i];
		/* skip read-only registers (ASICID, ASICREV, STATUS) */
		switch (reg) {
		case TWL6040_REG_ASICID:
		case TWL6040_REG_ASICREV:
		case TWL6040_REG_STATUS:
			continue;
		default:
			break;
		}
		twl6040_write(codec, reg, cache[reg]);
	}
}

static void twl6040_init_vdd_regs(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	int reg, i;

	for (i = 0; i < TWL6040_VDDREGNUM; i++) {
		reg = twl6040_vdd_reg[i];
		twl6040_write(codec, reg, cache[reg]);
	}
}

/*
 * Ramp HS PGA volume to minimise pops at stream startup and shutdown.
 */
static inline int twl6040_hs_ramp_step(struct snd_soc_codec *codec,
			unsigned int left_step, unsigned int right_step)
{

	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_output *headset = &priv->headset;
	int left_complete = 0, right_complete = 0;
	u8 reg, val;

	/* left channel */
	left_step = (left_step > 0xF) ? 0xF : left_step;
	reg = twl6040_read_reg_cache(codec, TWL6040_REG_HSGAIN);
	val = (~reg & TWL6040_HSL_VOL_MASK);

	if (headset->ramp == TWL6040_RAMP_UP) {
		/* ramp step up */
		if (val < headset->left_vol) {
			val += left_step;
			reg &= ~TWL6040_HSL_VOL_MASK;
			twl6040_write(codec, TWL6040_REG_HSGAIN,
					(reg | (~val & TWL6040_HSL_VOL_MASK)));
		} else {
			left_complete = 1;
		}
	} else if (headset->ramp == TWL6040_RAMP_DOWN) {
		/* ramp step down */
		if (val > 0x0) {
			val -= left_step;
			reg &= ~TWL6040_HSL_VOL_MASK;
			twl6040_write(codec, TWL6040_REG_HSGAIN, reg |
						(~val & TWL6040_HSL_VOL_MASK));
		} else {
			left_complete = 1;
		}
	}

	/* right channel */
	right_step = (right_step > 0xF) ? 0xF : right_step;
	reg = twl6040_read_reg_cache(codec, TWL6040_REG_HSGAIN);
	val = (~reg & TWL6040_HSR_VOL_MASK) >> TWL6040_HSR_VOL_SHIFT;

	if (headset->ramp == TWL6040_RAMP_UP) {
		/* ramp step up */
		if (val < headset->right_vol) {
			val += right_step;
			reg &= ~TWL6040_HSR_VOL_MASK;
			twl6040_write(codec, TWL6040_REG_HSGAIN,
				(reg | (~val << TWL6040_HSR_VOL_SHIFT)));
		} else {
			right_complete = 1;
		}
	} else if (headset->ramp == TWL6040_RAMP_DOWN) {
		/* ramp step down */
		if (val > 0x0) {
			val -= right_step;
			reg &= ~TWL6040_HSR_VOL_MASK;
			twl6040_write(codec, TWL6040_REG_HSGAIN,
					 reg | (~val << TWL6040_HSR_VOL_SHIFT));
		} else {
			right_complete = 1;
		}
	}

	return left_complete & right_complete;
}

/*
 * Ramp HF PGA volume to minimise pops at stream startup and shutdown.
 */
static inline int twl6040_hf_ramp_step(struct snd_soc_codec *codec,
			unsigned int left_step, unsigned int right_step)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_output *handsfree = &priv->handsfree;
	int left_complete = 0, right_complete = 0;
	u16 reg, val;

	/* left channel */
	left_step = (left_step > 0x1D) ? 0x1D : left_step;
	reg = twl6040_read_reg_cache(codec, TWL6040_REG_HFLGAIN);
	reg = 0x1D - reg;
	val = (reg & TWL6040_HF_VOL_MASK);
	if (handsfree->ramp == TWL6040_RAMP_UP) {
		/* ramp step up */
		if (val < handsfree->left_vol) {
			val += left_step;
			reg &= ~TWL6040_HF_VOL_MASK;
			twl6040_write(codec, TWL6040_REG_HFLGAIN,
						reg | (0x1D - val));
		} else {
			left_complete = 1;
		}
	} else if (handsfree->ramp == TWL6040_RAMP_DOWN) {
		/* ramp step down */
		if (val > 0) {
			val -= left_step;
			reg &= ~TWL6040_HF_VOL_MASK;
			twl6040_write(codec, TWL6040_REG_HFLGAIN,
						reg | (0x1D - val));
		} else {
			left_complete = 1;
		}
	}

	/* right channel */
	right_step = (right_step > 0x1D) ? 0x1D : right_step;
	reg = twl6040_read_reg_cache(codec, TWL6040_REG_HFRGAIN);
	reg = 0x1D - reg;
	val = (reg & TWL6040_HF_VOL_MASK);
	if (handsfree->ramp == TWL6040_RAMP_UP) {
		/* ramp step up */
		if (val < handsfree->right_vol) {
			val += right_step;
			reg &= ~TWL6040_HF_VOL_MASK;
			twl6040_write(codec, TWL6040_REG_HFRGAIN,
						reg | (0x1D - val));
		} else {
			right_complete = 1;
		}
	} else if (handsfree->ramp == TWL6040_RAMP_DOWN) {
		/* ramp step down */
		if (val > 0) {
			val -= right_step;
			reg &= ~TWL6040_HF_VOL_MASK;
			twl6040_write(codec, TWL6040_REG_HFRGAIN,
						reg | (0x1D - val));
		}
	}

	return left_complete & right_complete;
}

/*
 * This work ramps both output PGAs at stream start/stop time to
 * minimise pop associated with DAPM power switching.
 */
static void twl6040_pga_hs_work(struct work_struct *work)
{
	struct twl6040_data *priv =
		container_of(work, struct twl6040_data, hs_delayed_work.work);
	struct snd_soc_codec *codec = priv->codec;
	struct twl6040_output *headset = &priv->headset;
	unsigned int delay = headset->step_delay;
	int i, headset_complete;

	/* do we need to ramp at all ? */
	if (headset->ramp == TWL6040_RAMP_NONE)
		return;

	/* HS PGA volumes have 4 bits of resolution to ramp */
	for (i = 0; i <= 16; i++) {
		headset_complete = 1;
		if (headset->ramp != TWL6040_RAMP_NONE)
			headset_complete = twl6040_hs_ramp_step(codec,
							headset->left_step,
							headset->right_step);

		/* ramp finished ? */
		if (headset_complete)
			break;

		/*
		 * TODO: tune: delay is longer over 0dB
		 * as increases are larger.
		 */
		if (i >= 8)
			schedule_timeout_interruptible(msecs_to_jiffies(delay +
							(delay >> 1)));
		else
			schedule_timeout_interruptible(msecs_to_jiffies(delay));
	}

	if (headset->ramp == TWL6040_RAMP_DOWN) {
		headset->active = 0;
		complete(&headset->ramp_done);
	} else {
		headset->active = 1;
	}
	headset->ramp = TWL6040_RAMP_NONE;
}

static void twl6040_pga_hf_work(struct work_struct *work)
{
	struct twl6040_data *priv =
		container_of(work, struct twl6040_data, hf_delayed_work.work);
	struct snd_soc_codec *codec = priv->codec;
	struct twl6040_output *handsfree = &priv->handsfree;
	unsigned int delay = handsfree->step_delay;
	int i, handsfree_complete;

	/* do we need to ramp at all ? */
	if (handsfree->ramp == TWL6040_RAMP_NONE)
		return;

	/* HF PGA volumes have 5 bits of resolution to ramp */
	for (i = 0; i <= 32; i++) {
		handsfree_complete = 1;
		if (handsfree->ramp != TWL6040_RAMP_NONE)
			handsfree_complete = twl6040_hf_ramp_step(codec,
							handsfree->left_step,
							handsfree->right_step);

		/* ramp finished ? */
		if (handsfree_complete)
			break;

		/*
		 * TODO: tune: delay is longer over 0dB
		 * as increases are larger.
		 */
		if (i >= 16)
			schedule_timeout_interruptible(msecs_to_jiffies(delay +
						       (delay >> 1)));
		else
			schedule_timeout_interruptible(msecs_to_jiffies(delay));
	}


	if (handsfree->ramp == TWL6040_RAMP_DOWN) {
		handsfree->active = 0;
		complete(&handsfree->ramp_done);
	} else
		handsfree->active = 1;
	handsfree->ramp = TWL6040_RAMP_NONE;
}

static int pga_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_output *out;
	struct delayed_work *work;
	struct workqueue_struct *queue;

	switch (w->shift) {
	case 2:
	case 3:
		out = &priv->headset;
		work = &priv->hs_delayed_work;
		queue = priv->hs_workqueue;
		out->step_delay = 5;	/* 5 ms between volume ramp steps */
		break;
	case 4:
		out = &priv->handsfree;
		work = &priv->hf_delayed_work;
		queue = priv->hf_workqueue;
		out->step_delay = 5;	/* 5 ms between volume ramp steps */
		if (SND_SOC_DAPM_EVENT_ON(event))
			priv->non_lp++;
		else
			priv->non_lp--;
		break;
	default:
		return -1;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (out->active)
			break;

		/* don't use volume ramp for power-up */
		out->left_step = out->left_vol;
		out->right_step = out->right_vol;

		if (!delayed_work_pending(work)) {
			out->ramp = TWL6040_RAMP_UP;
			queue_delayed_work(queue, work,
					msecs_to_jiffies(1));
		}
		break;

	case SND_SOC_DAPM_PRE_PMD:
		if (!out->active)
			break;

		if (!delayed_work_pending(work)) {
			/* use volume ramp for power-down */
			out->left_step = 1;
			out->right_step = 1;
			out->ramp = TWL6040_RAMP_DOWN;
			INIT_COMPLETION(out->ramp_done);

			queue_delayed_work(queue, work,
					msecs_to_jiffies(1));

			wait_for_completion_timeout(&out->ramp_done,
					msecs_to_jiffies(2000));
		}
		break;
	}

	return 0;
}

/* twl6040 codec manual power-up sequence */
static void twl6040_power_up(struct snd_soc_codec *codec)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;

	ncpctl = twl6040_read_reg_cache(codec, TWL6040_REG_NCPCTL);
	ldoctl = twl6040_read_reg_cache(codec, TWL6040_REG_LDOCTL);
	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);
	accctl = twl6040_read_reg_cache(codec, TWL6040_REG_ACCCTL);

	/* enable reference system */
	ldoctl |= TWL6040_REFENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	msleep(10);
	/* enable internal oscillator */
	ldoctl |= TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(10);
	/* enable high-side ldo */
	ldoctl |= TWL6040_HSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* enable negative charge pump */
	ncpctl |= TWL6040_NCPENA | TWL6040_NCPOPEN;
	twl6040_write(codec, TWL6040_REG_NCPCTL, ncpctl);
	udelay(488);
	/* enable low-side ldo */
	ldoctl |= TWL6040_LSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* enable low-power pll */
	lppllctl |= TWL6040_LPLLENA;
	twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
	/* reset state machine */
	accctl |= TWL6040_RESETSPLIT;
	twl6040_write(codec, TWL6040_REG_ACCCTL, accctl);
	mdelay(5);
	accctl &= ~TWL6040_RESETSPLIT;
	twl6040_write(codec, TWL6040_REG_ACCCTL, accctl);
	/* disable internal oscillator */
	ldoctl &= ~TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
}

/* twl6040 codec manual power-down sequence */
static void twl6040_power_down(struct snd_soc_codec *codec)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;

	ncpctl = twl6040_read_reg_cache(codec, TWL6040_REG_NCPCTL);
	ldoctl = twl6040_read_reg_cache(codec, TWL6040_REG_LDOCTL);
	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);
	accctl = twl6040_read_reg_cache(codec, TWL6040_REG_ACCCTL);

	/* enable internal oscillator */
	ldoctl |= TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(10);
	/* disable low-power pll */
	lppllctl &= ~TWL6040_LPLLENA;
	twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
	/* disable low-side ldo */
	ldoctl &= ~TWL6040_LSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* disable negative charge pump */
	ncpctl &= ~(TWL6040_NCPENA | TWL6040_NCPOPEN);
	twl6040_write(codec, TWL6040_REG_NCPCTL, ncpctl);
	udelay(488);
	/* disable high-side ldo */
	ldoctl &= ~TWL6040_HSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* disable internal oscillator */
	ldoctl &= ~TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	/* disable reference system */
	ldoctl &= ~TWL6040_REFENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	msleep(10);
}

/* set headset dac and driver power mode */
static int headset_power_mode(struct snd_soc_codec *codec, int high_perf)
{
	int hslctl, hsrctl;
	int mask = TWL6040_HSDRVMODEL | TWL6040_HSDACMODEL;

	hslctl = twl6040_read_reg_cache(codec, TWL6040_REG_HSLCTL);
	hsrctl = twl6040_read_reg_cache(codec, TWL6040_REG_HSRCTL);

	if (high_perf) {
		hslctl &= ~mask;
		hsrctl &= ~mask;
	} else {
		hslctl |= mask;
		hsrctl |= mask;
	}

	twl6040_write(codec, TWL6040_REG_HSLCTL, hslctl);
	twl6040_write(codec, TWL6040_REG_HSRCTL, hsrctl);

	return 0;
}

static int twl6040_hs_dac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	msleep(1);
	return 0;
}

static int twl6040_power_mode_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	if (SND_SOC_DAPM_EVENT_ON(event))
		priv->non_lp++;
	else
		priv->non_lp--;

	msleep(1);

	return 0;
}

static void twl6040_hs_jack_report(struct snd_soc_codec *codec,
				   struct snd_soc_jack *jack, int report)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	int status;

	mutex_lock(&priv->mutex);

	/* Sync status */
	status = twl6040_read_reg_volatile(codec, TWL6040_REG_STATUS);
	if (status & TWL6040_PLUGCOMP)
		snd_soc_jack_report(jack, report, report);
	else
		snd_soc_jack_report(jack, 0, report);

	mutex_unlock(&priv->mutex);
}

void twl6040_hs_jack_detect(struct snd_soc_codec *codec,
				struct snd_soc_jack *jack, int report)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_jack_data *hs_jack = &priv->hs_jack;

	hs_jack->jack = jack;
	hs_jack->report = report;

	twl6040_hs_jack_report(codec, hs_jack->jack, hs_jack->report);
}
EXPORT_SYMBOL_GPL(twl6040_hs_jack_detect);

static void twl6040_accessory_work(struct work_struct *work)
{
	struct twl6040_data *priv = container_of(work,
					struct twl6040_data, delayed_work.work);
	struct snd_soc_codec *codec = priv->codec;
	struct twl6040_jack_data *hs_jack = &priv->hs_jack;

	twl6040_hs_jack_report(codec, hs_jack->jack, hs_jack->report);
}

/* audio interrupt handler */
static irqreturn_t twl6040_naudint_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	u8 intid;

	twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &intid, TWL6040_REG_INTID);

	if (intid & TWL6040_THINT)
		dev_alert(codec->dev, "die temp over-limit detection\n");

	if ((intid & TWL6040_PLUGINT) || (intid & TWL6040_UNPLUGINT))
		queue_delayed_work(priv->workqueue, &priv->delayed_work,
							msecs_to_jiffies(200));

	if (intid & TWL6040_HOOKINT)
		dev_info(codec->dev, "hook detection\n");

	if (intid & TWL6040_HFINT)
		dev_alert(codec->dev, "hf drivers over current detection\n");

	if (intid & TWL6040_VIBINT)
		dev_alert(codec->dev, "vib drivers over current detection\n");

	if (intid & TWL6040_READYINT)
		complete(&priv->ready);

	return IRQ_HANDLED;
}

static int twl6040_put_volsw(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct twl6040_data *twl6040_priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_output *out = NULL;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int ret;
	unsigned int reg = mc->reg;

	/* For HS and HF we shadow the values and only actually write
	 * them out when active in order to ensure the amplifier comes on
	 * as quietly as possible. */
	switch (reg) {
	case TWL6040_REG_HSGAIN:
		out = &twl6040_priv->headset;
		break;
	default:
		break;
	}

	if (out) {
		out->left_vol = ucontrol->value.integer.value[0];
		out->right_vol = ucontrol->value.integer.value[1];
		if (!out->active)
			return 1;
	}

	ret = snd_soc_put_volsw(kcontrol, ucontrol);
	if (ret < 0)
		return ret;

	return 1;
}

static int twl6040_get_volsw(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct twl6040_data *twl6040_priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_output *out = &twl6040_priv->headset;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;

	switch (reg) {
	case TWL6040_REG_HSGAIN:
		out = &twl6040_priv->headset;
		ucontrol->value.integer.value[0] = out->left_vol;
		ucontrol->value.integer.value[1] = out->right_vol;
		return 0;

	default:
		break;
	}

	return snd_soc_get_volsw(kcontrol, ucontrol);
}

static int twl6040_put_volsw_2r_vu(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct twl6040_data *twl6040_priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_output *out = NULL;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int ret;
	unsigned int reg = mc->reg;

	/* For HS and HF we shadow the values and only actually write
	 * them out when active in order to ensure the amplifier comes on
	 * as quietly as possible. */
	switch (reg) {
	case TWL6040_REG_HFLGAIN:
	case TWL6040_REG_HFRGAIN:
		out = &twl6040_priv->handsfree;
		break;
	default:
		break;
	}

	if (out) {
		out->left_vol = ucontrol->value.integer.value[0];
		out->right_vol = ucontrol->value.integer.value[1];
		if (!out->active)
			return 1;
	}

	ret = snd_soc_put_volsw_2r(kcontrol, ucontrol);
	if (ret < 0)
		return ret;

	return 1;
}

static int twl6040_get_volsw_2r(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct twl6040_data *twl6040_priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_output *out = &twl6040_priv->handsfree;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;

	/* If these are cached registers use the cache */
	switch (reg) {
	case TWL6040_REG_HFLGAIN:
	case TWL6040_REG_HFRGAIN:
		out = &twl6040_priv->handsfree;
		ucontrol->value.integer.value[0] = out->left_vol;
		ucontrol->value.integer.value[1] = out->right_vol;
		return 0;

	default:
		break;
	}

	return snd_soc_get_volsw_2r(kcontrol, ucontrol);
}

/* double control with volume update */
#define SOC_TWL6040_DOUBLE_TLV(xname, xreg, shift_left, shift_right, xmax,\
							xinvert, tlv_array)\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = twl6040_get_volsw, \
	.put = twl6040_put_volsw, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .shift = shift_left, .rshift = shift_right,\
		 .max = xmax, .platform_max = xmax, .invert = xinvert} }

/* double control with volume update */
#define SOC_TWL6040_DOUBLE_R_TLV(xname, reg_left, reg_right, xshift, xmax,\
				xinvert, tlv_array)\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		SNDRV_CTL_ELEM_ACCESS_READWRITE | \
		SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_2r, \
	.get = twl6040_get_volsw_2r, .put = twl6040_put_volsw_2r_vu, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = xshift, \
		 .rshift = xshift, .max = xmax, .invert = xinvert}, }

/*
 * MICATT volume control:
 * from -6 to 0 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_preamp_tlv, -600, 600, 0);

/*
 * MICGAIN volume control:
 * from -6 to 30 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_amp_tlv, -600, 600, 0);

/*
 * AFMGAIN volume control:
 * from -18 to 24 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(afm_amp_tlv, -1800, 600, 0);

/*
 * HSGAIN volume control:
 * from -30 to 0 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(hs_tlv, -3000, 200, 0);

/*
 * HFGAIN volume control:
 * from -52 to 6 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(hf_tlv, -5200, 200, 0);

/*
 * EPGAIN volume control:
 * from -24 to 6 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(ep_tlv, -2400, 200, 0);

/* Left analog microphone selection */
static const char *twl6040_amicl_texts[] =
	{"Headset Mic", "Main Mic", "Aux/FM Left", "Off"};

/* Right analog microphone selection */
static const char *twl6040_amicr_texts[] =
	{"Headset Mic", "Sub Mic", "Aux/FM Right", "Off"};

static const struct soc_enum twl6040_enum[] = {
	SOC_ENUM_SINGLE(TWL6040_REG_MICLCTL, 3, 4, twl6040_amicl_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_MICRCTL, 3, 4, twl6040_amicr_texts),
};

static const char *twl6040_hs_texts[] = {
	"Off", "HS DAC", "Line-In amp"
};

static const struct soc_enum twl6040_hs_enum[] = {
	SOC_ENUM_SINGLE(TWL6040_REG_HSLCTL, 5, ARRAY_SIZE(twl6040_hs_texts),
			twl6040_hs_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_HSRCTL, 5, ARRAY_SIZE(twl6040_hs_texts),
			twl6040_hs_texts),
};

static const char *twl6040_hf_texts[] = {
	"Off", "HF DAC", "Line-In amp"
};

static const struct soc_enum twl6040_hf_enum[] = {
	SOC_ENUM_SINGLE(TWL6040_REG_HFLCTL, 2, ARRAY_SIZE(twl6040_hf_texts),
			twl6040_hf_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_HFRCTL, 2, ARRAY_SIZE(twl6040_hf_texts),
			twl6040_hf_texts),
};

static const struct snd_kcontrol_new amicl_control =
	SOC_DAPM_ENUM("Route", twl6040_enum[0]);

static const struct snd_kcontrol_new amicr_control =
	SOC_DAPM_ENUM("Route", twl6040_enum[1]);

/* Headset DAC playback switches */
static const struct snd_kcontrol_new hsl_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_hs_enum[0]);

static const struct snd_kcontrol_new hsr_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_hs_enum[1]);

/* Handsfree DAC playback switches */
static const struct snd_kcontrol_new hfl_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_hf_enum[0]);

static const struct snd_kcontrol_new hfr_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_hf_enum[1]);

static const struct snd_kcontrol_new ep_driver_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6040_REG_EARCTL, 0, 1, 0);

static const struct snd_kcontrol_new twl6040_snd_controls[] = {
	/* Capture gains */
	SOC_DOUBLE_TLV("Capture Preamplifier Volume",
		TWL6040_REG_MICGAIN, 6, 7, 1, 1, mic_preamp_tlv),
	SOC_DOUBLE_TLV("Capture Volume",
		TWL6040_REG_MICGAIN, 0, 3, 4, 0, mic_amp_tlv),

	/* AFM gains */
	SOC_DOUBLE_TLV("Aux FM Volume",
		TWL6040_REG_LINEGAIN, 0, 3, 7, 0, afm_amp_tlv),

	/* Playback gains */
	SOC_TWL6040_DOUBLE_TLV("Headset Playback Volume",
		TWL6040_REG_HSGAIN, 0, 4, 0xF, 1, hs_tlv),
	SOC_TWL6040_DOUBLE_R_TLV("Handsfree Playback Volume",
		TWL6040_REG_HFLGAIN, TWL6040_REG_HFRGAIN, 0, 0x1D, 1, hf_tlv),
	SOC_SINGLE_TLV("Earphone Playback Volume",
		TWL6040_REG_EARCTL, 1, 0xF, 1, ep_tlv),
};

static const struct snd_soc_dapm_widget twl6040_dapm_widgets[] = {
	/* Inputs */
	SND_SOC_DAPM_INPUT("MAINMIC"),
	SND_SOC_DAPM_INPUT("HSMIC"),
	SND_SOC_DAPM_INPUT("SUBMIC"),
	SND_SOC_DAPM_INPUT("AFML"),
	SND_SOC_DAPM_INPUT("AFMR"),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("HSOL"),
	SND_SOC_DAPM_OUTPUT("HSOR"),
	SND_SOC_DAPM_OUTPUT("HFL"),
	SND_SOC_DAPM_OUTPUT("HFR"),
	SND_SOC_DAPM_OUTPUT("EP"),

	/* Analog input muxes for the capture amplifiers */
	SND_SOC_DAPM_MUX("Analog Left Capture Route",
			SND_SOC_NOPM, 0, 0, &amicl_control),
	SND_SOC_DAPM_MUX("Analog Right Capture Route",
			SND_SOC_NOPM, 0, 0, &amicr_control),

	/* Analog capture PGAs */
	SND_SOC_DAPM_PGA("MicAmpL",
			TWL6040_REG_MICLCTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MicAmpR",
			TWL6040_REG_MICRCTL, 0, 0, NULL, 0),

	/* Auxiliary FM PGAs */
	SND_SOC_DAPM_PGA("AFMAmpL",
			TWL6040_REG_MICLCTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AFMAmpR",
			TWL6040_REG_MICRCTL, 1, 0, NULL, 0),

	/* ADCs */
	SND_SOC_DAPM_ADC("ADC Left", "Left Front Capture",
			TWL6040_REG_MICLCTL, 2, 0),
	SND_SOC_DAPM_ADC("ADC Right", "Right Front Capture",
			TWL6040_REG_MICRCTL, 2, 0),

	/* Microphone bias */
	SND_SOC_DAPM_MICBIAS("Headset Mic Bias",
			TWL6040_REG_AMICBCTL, 0, 0),
	SND_SOC_DAPM_MICBIAS("Main Mic Bias",
			TWL6040_REG_AMICBCTL, 4, 0),
	SND_SOC_DAPM_MICBIAS("Digital Mic1 Bias",
			TWL6040_REG_DMICBCTL, 0, 0),
	SND_SOC_DAPM_MICBIAS("Digital Mic2 Bias",
			TWL6040_REG_DMICBCTL, 4, 0),

	/* DACs */
	SND_SOC_DAPM_DAC_E("HSDAC Left", "Headset Playback",
			TWL6040_REG_HSLCTL, 0, 0,
			twl6040_hs_dac_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HSDAC Right", "Headset Playback",
			TWL6040_REG_HSRCTL, 0, 0,
			twl6040_hs_dac_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HFDAC Left", "Handsfree Playback",
			TWL6040_REG_HFLCTL, 0, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HFDAC Right", "Handsfree Playback",
			TWL6040_REG_HFRCTL, 0, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("HF Left Playback",
			SND_SOC_NOPM, 0, 0, &hfl_mux_controls),
	SND_SOC_DAPM_MUX("HF Right Playback",
			SND_SOC_NOPM, 0, 0, &hfr_mux_controls),
	/* Analog playback Muxes */
	SND_SOC_DAPM_MUX("HS Left Playback",
			SND_SOC_NOPM, 0, 0, &hsl_mux_controls),
	SND_SOC_DAPM_MUX("HS Right Playback",
			SND_SOC_NOPM, 0, 0, &hsr_mux_controls),

	/* Analog playback drivers */
	SND_SOC_DAPM_OUT_DRV_E("Handsfree Left Driver",
			TWL6040_REG_HFLCTL, 4, 0, NULL, 0,
			pga_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("Handsfree Right Driver",
			TWL6040_REG_HFRCTL, 4, 0, NULL, 0,
			pga_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("Headset Left Driver",
			TWL6040_REG_HSLCTL, 2, 0, NULL, 0,
			pga_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("Headset Right Driver",
			TWL6040_REG_HSRCTL, 2, 0, NULL, 0,
			pga_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SWITCH_E("Earphone Driver",
			SND_SOC_NOPM, 0, 0, &ep_driver_switch_controls,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	/* Analog playback PGAs */
	SND_SOC_DAPM_PGA("HFDAC Left PGA",
			TWL6040_REG_HFLCTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HFDAC Right PGA",
			TWL6040_REG_HFRCTL, 1, 0, NULL, 0),

};

static const struct snd_soc_dapm_route intercon[] = {
	/* Capture path */
	{"Analog Left Capture Route", "Headset Mic", "HSMIC"},
	{"Analog Left Capture Route", "Main Mic", "MAINMIC"},
	{"Analog Left Capture Route", "Aux/FM Left", "AFML"},

	{"Analog Right Capture Route", "Headset Mic", "HSMIC"},
	{"Analog Right Capture Route", "Sub Mic", "SUBMIC"},
	{"Analog Right Capture Route", "Aux/FM Right", "AFMR"},

	{"MicAmpL", NULL, "Analog Left Capture Route"},
	{"MicAmpR", NULL, "Analog Right Capture Route"},

	{"ADC Left", NULL, "MicAmpL"},
	{"ADC Right", NULL, "MicAmpR"},

	/* AFM path */
	{"AFMAmpL", "NULL", "AFML"},
	{"AFMAmpR", "NULL", "AFMR"},

	{"HS Left Playback", "HS DAC", "HSDAC Left"},
	{"HS Left Playback", "Line-In amp", "AFMAmpL"},

	{"HS Right Playback", "HS DAC", "HSDAC Right"},
	{"HS Right Playback", "Line-In amp", "AFMAmpR"},

	{"Headset Left Driver", "NULL", "HS Left Playback"},
	{"Headset Right Driver", "NULL", "HS Right Playback"},

	{"HSOL", NULL, "Headset Left Driver"},
	{"HSOR", NULL, "Headset Right Driver"},

	/* Earphone playback path */
	{"Earphone Driver", "Switch", "HSDAC Left"},
	{"EP", NULL, "Earphone Driver"},

	{"HF Left Playback", "HF DAC", "HFDAC Left"},
	{"HF Left Playback", "Line-In amp", "AFMAmpL"},

	{"HF Right Playback", "HF DAC", "HFDAC Right"},
	{"HF Right Playback", "Line-In amp", "AFMAmpR"},

	{"HFDAC Left PGA", NULL, "HF Left Playback"},
	{"HFDAC Right PGA", NULL, "HF Right Playback"},

	{"Handsfree Left Driver", "Switch", "HFDAC Left PGA"},
	{"Handsfree Right Driver", "Switch", "HFDAC Right PGA"},

	{"HFL", NULL, "Handsfree Left Driver"},
	{"HFR", NULL, "Handsfree Right Driver"},
};

static int twl6040_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_new_controls(dapm, twl6040_dapm_widgets,
				 ARRAY_SIZE(twl6040_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, intercon, ARRAY_SIZE(intercon));
	snd_soc_dapm_new_widgets(dapm);

	return 0;
}

static int twl6040_power_up_completion(struct snd_soc_codec *codec,
					int naudint)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	int time_left;
	u8 intid;

	time_left = wait_for_completion_timeout(&priv->ready,
				msecs_to_jiffies(144));

	if (!time_left) {
		twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &intid,
							TWL6040_REG_INTID);
		if (!(intid & TWL6040_READYINT)) {
			dev_err(codec->dev, "timeout waiting for READYINT\n");
			return -ETIMEDOUT;
		}
	}

	priv->codec_powered = 1;

	return 0;
}

static int twl6040_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	int audpwron = priv->audpwron;
	int naudint = priv->naudint;
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (priv->codec_powered)
			break;

		if (gpio_is_valid(audpwron)) {
			/* use AUDPWRON line */
			gpio_set_value(audpwron, 1);

			/* wait for power-up completion */
			ret = twl6040_power_up_completion(codec, naudint);
			if (ret)
				return ret;

			/* sync registers updated during power-up sequence */
			twl6040_read_reg_volatile(codec, TWL6040_REG_NCPCTL);
			twl6040_read_reg_volatile(codec, TWL6040_REG_LDOCTL);
			twl6040_read_reg_volatile(codec, TWL6040_REG_LPPLLCTL);
		} else {
			/* use manual power-up sequence */
			twl6040_power_up(codec);
			priv->codec_powered = 1;
		}

		/* initialize vdd/vss registers with reg_cache */
		twl6040_init_vdd_regs(codec);

		/* Set external boost GPO */
		twl6040_write(codec, TWL6040_REG_GPOCTL, 0x02);

		/* Set initial minimal gain values */
		twl6040_write(codec, TWL6040_REG_HSGAIN, 0xFF);
		twl6040_write(codec, TWL6040_REG_EARCTL, 0x1E);
		twl6040_write(codec, TWL6040_REG_HFLGAIN, 0x1D);
		twl6040_write(codec, TWL6040_REG_HFRGAIN, 0x1D);
		break;
	case SND_SOC_BIAS_OFF:
		if (!priv->codec_powered)
			break;

		if (gpio_is_valid(audpwron)) {
			/* use AUDPWRON line */
			gpio_set_value(audpwron, 0);

			/* power-down sequence latency */
			udelay(500);

			/* sync registers updated during power-down sequence */
			twl6040_read_reg_volatile(codec, TWL6040_REG_NCPCTL);
			twl6040_read_reg_volatile(codec, TWL6040_REG_LDOCTL);
			twl6040_write_reg_cache(codec, TWL6040_REG_LPPLLCTL,
						0x00);
		} else {
			/* use manual power-down sequence */
			twl6040_power_down(codec);
		}

		priv->codec_powered = 0;
		break;
	}

	codec->dapm.bias_level = level;

	return 0;
}

/* set of rates for each pll: low-power and high-performance */

static unsigned int lp_rates[] = {
	88200,
	96000,
};

static struct snd_pcm_hw_constraint_list lp_constraints = {
	.count	= ARRAY_SIZE(lp_rates),
	.list	= lp_rates,
};

static unsigned int hp_rates[] = {
	96000,
};

static struct snd_pcm_hw_constraint_list hp_constraints = {
	.count	= ARRAY_SIZE(hp_rates),
	.list	= hp_rates,
};

static int twl6040_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				priv->sysclk_constraints);

	return 0;
}

static int twl6040_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	u8 lppllctl;
	int rate;

	/* nothing to do for high-perf pll, it supports only 48 kHz */
	if (priv->pll == TWL6040_HPPLL_ID)
		return 0;

	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);

	rate = params_rate(params);
	switch (rate) {
	case 11250:
	case 22500:
	case 44100:
	case 88200:
		lppllctl |= TWL6040_LPLLFIN;
		priv->sysclk = 17640000;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 96000:
		lppllctl &= ~TWL6040_LPLLFIN;
		priv->sysclk = 19200000;
		break;
	default:
		dev_err(codec->dev, "unsupported rate %d\n", rate);
		return -EINVAL;
	}

	twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);

	return 0;
}

static int twl6040_prepare(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	if (!priv->sysclk) {
		dev_err(codec->dev,
			"no mclk configured, call set_sysclk() on init\n");
		return -EINVAL;
	}

	/*
	 * capture is not supported at 17.64 MHz,
	 * it's reserved for headset low-power playback scenario
	 */
	if ((priv->sysclk == 17640000) &&
			substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dev_err(codec->dev,
			"capture mode is not supported at %dHz\n",
			priv->sysclk);
		return -EINVAL;
	}

	if ((priv->sysclk == 17640000) && priv->non_lp) {
			dev_err(codec->dev,
				"some enabled paths aren't supported at %dHz\n",
				priv->sysclk);
			return -EPERM;
	}
	return 0;
}

static int twl6040_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	u8 hppllctl, lppllctl;

	hppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_HPPLLCTL);
	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);

	switch (clk_id) {
	case TWL6040_SYSCLK_SEL_LPPLL:
		switch (freq) {
		case 32768:
			/* headset dac and driver must be in low-power mode */
			headset_power_mode(codec, 0);

			/* clk32k input requires low-power pll */
			lppllctl |= TWL6040_LPLLENA;
			twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
			mdelay(5);
			lppllctl &= ~TWL6040_HPLLSEL;
			twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
			hppllctl &= ~TWL6040_HPLLENA;
			twl6040_write(codec, TWL6040_REG_HPPLLCTL, hppllctl);
			break;
		default:
			dev_err(codec->dev, "unknown mclk freq %d\n", freq);
			return -EINVAL;
		}

		/* lppll divider */
		switch (priv->sysclk) {
		case 17640000:
			lppllctl |= TWL6040_LPLLFIN;
			break;
		case 19200000:
			lppllctl &= ~TWL6040_LPLLFIN;
			break;
		default:
			/* sysclk not yet configured */
			lppllctl &= ~TWL6040_LPLLFIN;
			priv->sysclk = 19200000;
			break;
		}

		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);

		priv->pll = TWL6040_LPPLL_ID;
		priv->sysclk_constraints = &lp_constraints;
		break;
	case TWL6040_SYSCLK_SEL_HPPLL:
		hppllctl &= ~TWL6040_MCLK_MSK;

		switch (freq) {
		case 12000000:
			/* mclk input, pll enabled */
			hppllctl |= TWL6040_MCLK_12000KHZ |
				    TWL6040_HPLLSQRBP |
				    TWL6040_HPLLENA;
			break;
		case 19200000:
			/* mclk input, pll disabled */
			hppllctl |= TWL6040_MCLK_19200KHZ |
				    TWL6040_HPLLSQRENA |
				    TWL6040_HPLLBP;
			break;
		case 26000000:
			/* mclk input, pll enabled */
			hppllctl |= TWL6040_MCLK_26000KHZ |
				    TWL6040_HPLLSQRBP |
				    TWL6040_HPLLENA;
			break;
		case 38400000:
			/* clk slicer, pll disabled */
			hppllctl |= TWL6040_MCLK_38400KHZ |
				    TWL6040_HPLLSQRENA |
				    TWL6040_HPLLBP;
			break;
		default:
			dev_err(codec->dev, "unknown mclk freq %d\n", freq);
			return -EINVAL;
		}

		/* headset dac and driver must be in high-performance mode */
		headset_power_mode(codec, 1);

		twl6040_write(codec, TWL6040_REG_HPPLLCTL, hppllctl);
		udelay(500);
		lppllctl |= TWL6040_HPLLSEL;
		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
		lppllctl &= ~TWL6040_LPLLENA;
		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);

		/* high-performance pll can provide only 19.2 MHz */
		priv->pll = TWL6040_HPPLL_ID;
		priv->sysclk = 19200000;
		priv->sysclk_constraints = &hp_constraints;
		break;
	default:
		dev_err(codec->dev, "unknown clk_id %d\n", clk_id);
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops twl6040_dai_ops = {
	.startup	= twl6040_startup,
	.hw_params	= twl6040_hw_params,
	.prepare	= twl6040_prepare,
	.set_sysclk	= twl6040_set_dai_sysclk,
};

static struct snd_soc_dai_driver twl6040_dai = {
	.name = "twl6040-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 4,
		.rates = TWL6040_RATES,
		.formats = TWL6040_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = TWL6040_RATES,
		.formats = TWL6040_FORMATS,
	},
	.ops = &twl6040_dai_ops,
};

#ifdef CONFIG_PM
static int twl6040_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	twl6040_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int twl6040_resume(struct snd_soc_codec *codec)
{
	twl6040_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	twl6040_set_bias_level(codec, codec->dapm.suspend_bias_level);

	return 0;
}
#else
#define twl6040_suspend NULL
#define twl6040_resume NULL
#endif

static int twl6040_probe(struct snd_soc_codec *codec)
{
	struct twl4030_codec_data *twl_codec = codec->dev->platform_data;
	struct twl6040_data *priv;
	int audpwron, naudint;
	int ret = 0;
	u8 icrev, intmr = TWL6040_ALLINT_MSK;

	priv = kzalloc(sizeof(struct twl6040_data), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	snd_soc_codec_set_drvdata(codec, priv);

	priv->codec = codec;

	twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &icrev, TWL6040_REG_ASICREV);

	if (twl_codec && (icrev > 0))
		audpwron = twl_codec->audpwron_gpio;
	else
		audpwron = -EINVAL;

	if (twl_codec)
		naudint = twl_codec->naudint_irq;
	else
		naudint = 0;

	priv->audpwron = audpwron;
	priv->naudint = naudint;
	priv->workqueue = create_singlethread_workqueue("twl6040-codec");

	if (!priv->workqueue) {
		ret = -ENOMEM;
		goto work_err;
	}

	INIT_DELAYED_WORK(&priv->delayed_work, twl6040_accessory_work);

	mutex_init(&priv->mutex);

	init_completion(&priv->ready);
	init_completion(&priv->headset.ramp_done);
	init_completion(&priv->handsfree.ramp_done);

	if (gpio_is_valid(audpwron)) {
		ret = gpio_request(audpwron, "audpwron");
		if (ret)
			goto gpio1_err;

		ret = gpio_direction_output(audpwron, 0);
		if (ret)
			goto gpio2_err;

		priv->codec_powered = 0;

		/* enable only codec ready interrupt */
		intmr &= ~(TWL6040_READYMSK | TWL6040_PLUGMSK);

		/* reset interrupt status to allow correct power up sequence */
		twl6040_read_reg_volatile(codec, TWL6040_REG_INTID);
	}
	twl6040_write(codec, TWL6040_REG_INTMR, intmr);

	if (naudint) {
		/* audio interrupt */
		ret = request_threaded_irq(naudint, NULL,
				twl6040_naudint_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"twl6040_codec", codec);
		if (ret)
			goto gpio2_err;
	}

	/* init vio registers */
	twl6040_init_vio_regs(codec);

	priv->hf_workqueue = create_singlethread_workqueue("twl6040-hf");
	if (priv->hf_workqueue == NULL) {
		ret = -ENOMEM;
		goto irq_err;
	}
	priv->hs_workqueue = create_singlethread_workqueue("twl6040-hs");
	if (priv->hs_workqueue == NULL) {
		ret = -ENOMEM;
		goto wq_err;
	}

	INIT_DELAYED_WORK(&priv->hs_delayed_work, twl6040_pga_hs_work);
	INIT_DELAYED_WORK(&priv->hf_delayed_work, twl6040_pga_hf_work);

	/* power on device */
	ret = twl6040_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (ret)
		goto bias_err;

	snd_soc_add_controls(codec, twl6040_snd_controls,
				ARRAY_SIZE(twl6040_snd_controls));
	twl6040_add_widgets(codec);

	return 0;

bias_err:
	destroy_workqueue(priv->hs_workqueue);
wq_err:
	destroy_workqueue(priv->hf_workqueue);
irq_err:
	if (naudint)
		free_irq(naudint, codec);
gpio2_err:
	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);
gpio1_err:
	destroy_workqueue(priv->workqueue);
work_err:
	kfree(priv);
	return ret;
}

static int twl6040_remove(struct snd_soc_codec *codec)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	int audpwron = priv->audpwron;
	int naudint = priv->naudint;

	twl6040_set_bias_level(codec, SND_SOC_BIAS_OFF);

	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);

	if (naudint)
		free_irq(naudint, codec);

	destroy_workqueue(priv->workqueue);
	destroy_workqueue(priv->hf_workqueue);
	destroy_workqueue(priv->hs_workqueue);
	kfree(priv);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_twl6040 = {
	.probe = twl6040_probe,
	.remove = twl6040_remove,
	.suspend = twl6040_suspend,
	.resume = twl6040_resume,
	.read = twl6040_read_reg_cache,
	.write = twl6040_write,
	.set_bias_level = twl6040_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(twl6040_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = twl6040_reg,
};

static int __devinit twl6040_codec_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_twl6040, &twl6040_dai, 1);
}

static int __devexit twl6040_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver twl6040_codec_driver = {
	.driver = {
		.name = "twl6040-codec",
		.owner = THIS_MODULE,
	},
	.probe = twl6040_codec_probe,
	.remove = __devexit_p(twl6040_codec_remove),
};

static int __init twl6040_codec_init(void)
{
	return platform_driver_register(&twl6040_codec_driver);
}
module_init(twl6040_codec_init);

static void __exit twl6040_codec_exit(void)
{
	platform_driver_unregister(&twl6040_codec_driver);
}
module_exit(twl6040_codec_exit);

MODULE_DESCRIPTION("ASoC TWL6040 codec driver");
MODULE_AUTHOR("Misael Lopez Cruz");
MODULE_LICENSE("GPL");
