/*
 * HD audio interface patch for Conexant HDA audio codec
 *
 * Copyright (c) 2006 Pototskiy Akex <alex.pototskiy@gmail.com>
 * 		      Takashi Iwai <tiwai@suse.de>
 * 		      Tobin Davis  <tdavis@dsl-only.net>
 *
 *  This driver is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This driver is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <sound/core.h>
#include <sound/jack.h>

#include "hda_codec.h"
#include "hda_local.h"

#define CXT_PIN_DIR_IN              0x00
#define CXT_PIN_DIR_OUT             0x01
#define CXT_PIN_DIR_INOUT           0x02
#define CXT_PIN_DIR_IN_NOMICBIAS    0x03
#define CXT_PIN_DIR_INOUT_NOMICBIAS 0x04

#define CONEXANT_HP_EVENT	0x37
#define CONEXANT_MIC_EVENT	0x38

/* Conexant 5051 specific */

#define CXT5051_SPDIF_OUT	0x1C
#define CXT5051_PORTB_EVENT	0x38
#define CXT5051_PORTC_EVENT	0x39


struct conexant_jack {

	hda_nid_t nid;
	int type;
	struct snd_jack *jack;

};

struct conexant_spec {

	struct snd_kcontrol_new *mixers[5];
	int num_mixers;
	hda_nid_t vmaster_nid;

	const struct hda_verb *init_verbs[5];	/* initialization verbs
						 * don't forget NULL
						 * termination!
						 */
	unsigned int num_init_verbs;

	/* playback */
	struct hda_multi_out multiout;	/* playback set-up
					 * max_channels, dacs must be set
					 * dig_out_nid and hp_nid are optional
					 */
	unsigned int cur_eapd;
	unsigned int hp_present;
	unsigned int no_auto_mic;
	unsigned int need_dac_fix;

	/* capture */
	unsigned int num_adc_nids;
	hda_nid_t *adc_nids;
	hda_nid_t dig_in_nid;		/* digital-in NID; optional */

	unsigned int cur_adc_idx;
	hda_nid_t cur_adc;
	unsigned int cur_adc_stream_tag;
	unsigned int cur_adc_format;

	/* capture source */
	const struct hda_input_mux *input_mux;
	hda_nid_t *capsrc_nids;
	unsigned int cur_mux[3];

	/* channel model */
	const struct hda_channel_mode *channel_mode;
	int num_channel_mode;

	/* PCM information */
	struct hda_pcm pcm_rec[2];	/* used in build_pcms() */

	unsigned int spdif_route;

	/* jack detection */
	struct snd_array jacks;

	/* dynamic controls, init_verbs and input_mux */
	struct auto_pin_cfg autocfg;
	struct hda_input_mux private_imux;
	hda_nid_t private_dac_nids[AUTO_CFG_MAX_OUTS];

	unsigned int dell_automute;
	unsigned int port_d_mode;
	unsigned char ext_mic_bias;
};

static int conexant_playback_pcm_open(struct hda_pcm_stream *hinfo,
				      struct hda_codec *codec,
				      struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	return snd_hda_multi_out_analog_open(codec, &spec->multiout, substream,
					     hinfo);
}

static int conexant_playback_pcm_prepare(struct hda_pcm_stream *hinfo,
					 struct hda_codec *codec,
					 unsigned int stream_tag,
					 unsigned int format,
					 struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	return snd_hda_multi_out_analog_prepare(codec, &spec->multiout,
						stream_tag,
						format, substream);
}

static int conexant_playback_pcm_cleanup(struct hda_pcm_stream *hinfo,
					 struct hda_codec *codec,
					 struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	return snd_hda_multi_out_analog_cleanup(codec, &spec->multiout);
}

/*
 * Digital out
 */
static int conexant_dig_playback_pcm_open(struct hda_pcm_stream *hinfo,
					  struct hda_codec *codec,
					  struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	return snd_hda_multi_out_dig_open(codec, &spec->multiout);
}

static int conexant_dig_playback_pcm_close(struct hda_pcm_stream *hinfo,
					 struct hda_codec *codec,
					 struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	return snd_hda_multi_out_dig_close(codec, &spec->multiout);
}

static int conexant_dig_playback_pcm_prepare(struct hda_pcm_stream *hinfo,
					 struct hda_codec *codec,
					 unsigned int stream_tag,
					 unsigned int format,
					 struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	return snd_hda_multi_out_dig_prepare(codec, &spec->multiout,
					     stream_tag,
					     format, substream);
}

/*
 * Analog capture
 */
static int conexant_capture_pcm_prepare(struct hda_pcm_stream *hinfo,
				      struct hda_codec *codec,
				      unsigned int stream_tag,
				      unsigned int format,
				      struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	snd_hda_codec_setup_stream(codec, spec->adc_nids[substream->number],
				   stream_tag, 0, format);
	return 0;
}

static int conexant_capture_pcm_cleanup(struct hda_pcm_stream *hinfo,
				      struct hda_codec *codec,
				      struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	snd_hda_codec_cleanup_stream(codec, spec->adc_nids[substream->number]);
	return 0;
}



static struct hda_pcm_stream conexant_pcm_analog_playback = {
	.substreams = 1,
	.channels_min = 2,
	.channels_max = 2,
	.nid = 0, /* fill later */
	.ops = {
		.open = conexant_playback_pcm_open,
		.prepare = conexant_playback_pcm_prepare,
		.cleanup = conexant_playback_pcm_cleanup
	},
};

static struct hda_pcm_stream conexant_pcm_analog_capture = {
	.substreams = 1,
	.channels_min = 2,
	.channels_max = 2,
	.nid = 0, /* fill later */
	.ops = {
		.prepare = conexant_capture_pcm_prepare,
		.cleanup = conexant_capture_pcm_cleanup
	},
};


static struct hda_pcm_stream conexant_pcm_digital_playback = {
	.substreams = 1,
	.channels_min = 2,
	.channels_max = 2,
	.nid = 0, /* fill later */
	.ops = {
		.open = conexant_dig_playback_pcm_open,
		.close = conexant_dig_playback_pcm_close,
		.prepare = conexant_dig_playback_pcm_prepare
	},
};

static struct hda_pcm_stream conexant_pcm_digital_capture = {
	.substreams = 1,
	.channels_min = 2,
	.channels_max = 2,
	/* NID is set in alc_build_pcms */
};

static int cx5051_capture_pcm_prepare(struct hda_pcm_stream *hinfo,
				      struct hda_codec *codec,
				      unsigned int stream_tag,
				      unsigned int format,
				      struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	spec->cur_adc = spec->adc_nids[spec->cur_adc_idx];
	spec->cur_adc_stream_tag = stream_tag;
	spec->cur_adc_format = format;
	snd_hda_codec_setup_stream(codec, spec->cur_adc, stream_tag, 0, format);
	return 0;
}

static int cx5051_capture_pcm_cleanup(struct hda_pcm_stream *hinfo,
				      struct hda_codec *codec,
				      struct snd_pcm_substream *substream)
{
	struct conexant_spec *spec = codec->spec;
	snd_hda_codec_cleanup_stream(codec, spec->cur_adc);
	spec->cur_adc = 0;
	return 0;
}

static struct hda_pcm_stream cx5051_pcm_analog_capture = {
	.substreams = 1,
	.channels_min = 2,
	.channels_max = 2,
	.nid = 0, /* fill later */
	.ops = {
		.prepare = cx5051_capture_pcm_prepare,
		.cleanup = cx5051_capture_pcm_cleanup
	},
};

static int conexant_build_pcms(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	struct hda_pcm *info = spec->pcm_rec;

	codec->num_pcms = 1;
	codec->pcm_info = info;

	info->name = "CONEXANT Analog";
	info->stream[SNDRV_PCM_STREAM_PLAYBACK] = conexant_pcm_analog_playback;
	info->stream[SNDRV_PCM_STREAM_PLAYBACK].channels_max =
		spec->multiout.max_channels;
	info->stream[SNDRV_PCM_STREAM_PLAYBACK].nid =
		spec->multiout.dac_nids[0];
	if (codec->vendor_id == 0x14f15051)
		info->stream[SNDRV_PCM_STREAM_CAPTURE] =
			cx5051_pcm_analog_capture;
	else
		info->stream[SNDRV_PCM_STREAM_CAPTURE] =
			conexant_pcm_analog_capture;
	info->stream[SNDRV_PCM_STREAM_CAPTURE].substreams = spec->num_adc_nids;
	info->stream[SNDRV_PCM_STREAM_CAPTURE].nid = spec->adc_nids[0];

	if (spec->multiout.dig_out_nid) {
		info++;
		codec->num_pcms++;
		info->name = "Conexant Digital";
		info->pcm_type = HDA_PCM_TYPE_SPDIF;
		info->stream[SNDRV_PCM_STREAM_PLAYBACK] =
			conexant_pcm_digital_playback;
		info->stream[SNDRV_PCM_STREAM_PLAYBACK].nid =
			spec->multiout.dig_out_nid;
		if (spec->dig_in_nid) {
			info->stream[SNDRV_PCM_STREAM_CAPTURE] =
				conexant_pcm_digital_capture;
			info->stream[SNDRV_PCM_STREAM_CAPTURE].nid =
				spec->dig_in_nid;
		}
	}

	return 0;
}

static int conexant_mux_enum_info(struct snd_kcontrol *kcontrol,
	       			  struct snd_ctl_elem_info *uinfo)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;

	return snd_hda_input_mux_info(spec->input_mux, uinfo);
}

static int conexant_mux_enum_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;
	unsigned int adc_idx = snd_ctl_get_ioffidx(kcontrol, &ucontrol->id);

	ucontrol->value.enumerated.item[0] = spec->cur_mux[adc_idx];
	return 0;
}

static int conexant_mux_enum_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;
	unsigned int adc_idx = snd_ctl_get_ioffidx(kcontrol, &ucontrol->id);

	return snd_hda_input_mux_put(codec, spec->input_mux, ucontrol,
				     spec->capsrc_nids[adc_idx],
				     &spec->cur_mux[adc_idx]);
}

#ifdef CONFIG_SND_HDA_INPUT_JACK
static void conexant_free_jack_priv(struct snd_jack *jack)
{
	struct conexant_jack *jacks = jack->private_data;
	jacks->nid = 0;
	jacks->jack = NULL;
}

static int conexant_add_jack(struct hda_codec *codec,
		hda_nid_t nid, int type)
{
	struct conexant_spec *spec;
	struct conexant_jack *jack;
	const char *name;
	int i, err;

	spec = codec->spec;
	snd_array_init(&spec->jacks, sizeof(*jack), 32);

	jack = spec->jacks.list;
	for (i = 0; i < spec->jacks.used; i++, jack++)
		if (jack->nid == nid)
			return 0 ; /* already present */

	jack = snd_array_new(&spec->jacks);
	name = (type == SND_JACK_HEADPHONE) ? "Headphone" : "Mic" ;

	if (!jack)
		return -ENOMEM;

	jack->nid = nid;
	jack->type = type;

	err = snd_jack_new(codec->bus->card, name, type, &jack->jack);
	if (err < 0)
		return err;
	jack->jack->private_data = jack;
	jack->jack->private_free = conexant_free_jack_priv;
	return 0;
}

static void conexant_report_jack(struct hda_codec *codec, hda_nid_t nid)
{
	struct conexant_spec *spec = codec->spec;
	struct conexant_jack *jacks = spec->jacks.list;

	if (jacks) {
		int i;
		for (i = 0; i < spec->jacks.used; i++) {
			if (jacks->nid == nid) {
				unsigned int present;
				present = snd_hda_codec_read(codec, nid, 0,
						AC_VERB_GET_PIN_SENSE, 0) &
					AC_PINSENSE_PRESENCE;

				present = (present) ? jacks->type : 0 ;

				snd_jack_report(jacks->jack,
						present);
			}
			jacks++;
		}
	}
}

static int conexant_init_jacks(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	int i;

	for (i = 0; i < spec->num_init_verbs; i++) {
		const struct hda_verb *hv;

		hv = spec->init_verbs[i];
		while (hv->nid) {
			int err = 0;
			switch (hv->param ^ AC_USRSP_EN) {
			case CONEXANT_HP_EVENT:
				err = conexant_add_jack(codec, hv->nid,
						SND_JACK_HEADPHONE);
				conexant_report_jack(codec, hv->nid);
				break;
			case CXT5051_PORTC_EVENT:
			case CONEXANT_MIC_EVENT:
				err = conexant_add_jack(codec, hv->nid,
						SND_JACK_MICROPHONE);
				conexant_report_jack(codec, hv->nid);
				break;
			}
			if (err < 0)
				return err;
			++hv;
		}
	}
	return 0;

}
#else
static inline void conexant_report_jack(struct hda_codec *codec, hda_nid_t nid)
{
}

static inline int conexant_init_jacks(struct hda_codec *codec)
{
	return 0;
}
#endif

static int conexant_init(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	int i;

	for (i = 0; i < spec->num_init_verbs; i++)
		snd_hda_sequence_write(codec, spec->init_verbs[i]);
	return 0;
}

static void conexant_free(struct hda_codec *codec)
{
#ifdef CONFIG_SND_HDA_INPUT_JACK
	struct conexant_spec *spec = codec->spec;
	if (spec->jacks.list) {
		struct conexant_jack *jacks = spec->jacks.list;
		int i;
		for (i = 0; i < spec->jacks.used; i++, jacks++) {
			if (jacks->jack)
				snd_device_free(codec->bus->card, jacks->jack);
		}
		snd_array_free(&spec->jacks);
	}
#endif
	kfree(codec->spec);
}

static struct snd_kcontrol_new cxt_capture_mixers[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Capture Source",
		.info = conexant_mux_enum_info,
		.get = conexant_mux_enum_get,
		.put = conexant_mux_enum_put
	},
	{}
};

static const char *slave_vols[] = {
	"Headphone Playback Volume",
	"Speaker Playback Volume",
	NULL
};

static const char *slave_sws[] = {
	"Headphone Playback Switch",
	"Speaker Playback Switch",
	NULL
};

static int conexant_build_controls(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	unsigned int i;
	int err;

	for (i = 0; i < spec->num_mixers; i++) {
		err = snd_hda_add_new_ctls(codec, spec->mixers[i]);
		if (err < 0)
			return err;
	}
	if (spec->multiout.dig_out_nid) {
		err = snd_hda_create_spdif_out_ctls(codec,
						    spec->multiout.dig_out_nid);
		if (err < 0)
			return err;
		err = snd_hda_create_spdif_share_sw(codec,
						    &spec->multiout);
		if (err < 0)
			return err;
		spec->multiout.share_spdif = 1;
	} 
	if (spec->dig_in_nid) {
		err = snd_hda_create_spdif_in_ctls(codec,spec->dig_in_nid);
		if (err < 0)
			return err;
	}

	/* if we have no master control, let's create it */
	if (spec->vmaster_nid &&
	    !snd_hda_find_mixer_ctl(codec, "Master Playback Volume")) {
		unsigned int vmaster_tlv[4];
		snd_hda_set_vmaster_tlv(codec, spec->vmaster_nid,
					HDA_OUTPUT, vmaster_tlv);
		err = snd_hda_add_vmaster(codec, "Master Playback Volume",
					  vmaster_tlv, slave_vols);
		if (err < 0)
			return err;
	}
	if (spec->vmaster_nid &&
	    !snd_hda_find_mixer_ctl(codec, "Master Playback Switch")) {
		err = snd_hda_add_vmaster(codec, "Master Playback Switch",
					  NULL, slave_sws);
		if (err < 0)
			return err;
	}

	if (spec->input_mux) {
		err = snd_hda_add_new_ctls(codec, cxt_capture_mixers);
		if (err < 0)
			return err;
	}

	return 0;
}

static struct hda_codec_ops conexant_patch_ops = {
	.build_controls = conexant_build_controls,
	.build_pcms = conexant_build_pcms,
	.init = conexant_init,
	.free = conexant_free,
};

/*
 * EAPD control
 * the private value = nid | (invert << 8)
 */

#define cxt_eapd_info		snd_ctl_boolean_mono_info

static int cxt_eapd_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;
	int invert = (kcontrol->private_value >> 8) & 1;
	if (invert)
		ucontrol->value.integer.value[0] = !spec->cur_eapd;
	else
		ucontrol->value.integer.value[0] = spec->cur_eapd;
	return 0;

}

static int cxt_eapd_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;
	int invert = (kcontrol->private_value >> 8) & 1;
	hda_nid_t nid = kcontrol->private_value & 0xff;
	unsigned int eapd;

	eapd = !!ucontrol->value.integer.value[0];
	if (invert)
		eapd = !eapd;
	if (eapd == spec->cur_eapd)
		return 0;
	
	spec->cur_eapd = eapd;
	snd_hda_codec_write_cache(codec, nid,
				  0, AC_VERB_SET_EAPD_BTLENABLE,
				  eapd ? 0x02 : 0x00);
	return 1;
}

/* controls for test mode */
#ifdef CONFIG_SND_DEBUG

#define CXT_EAPD_SWITCH(xname, nid, mask) \
	{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = 0,  \
	  .info = cxt_eapd_info, \
	  .get = cxt_eapd_get, \
	  .put = cxt_eapd_put, \
	  .private_value = nid | (mask<<16) }



static int conexant_ch_mode_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;
	return snd_hda_ch_mode_info(codec, uinfo, spec->channel_mode,
				    spec->num_channel_mode);
}

static int conexant_ch_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;
	return snd_hda_ch_mode_get(codec, ucontrol, spec->channel_mode,
				   spec->num_channel_mode,
				   spec->multiout.max_channels);
}

static int conexant_ch_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;
	int err = snd_hda_ch_mode_put(codec, ucontrol, spec->channel_mode,
				      spec->num_channel_mode,
				      &spec->multiout.max_channels);
	if (err >= 0 && spec->need_dac_fix)
		spec->multiout.num_dacs = spec->multiout.max_channels / 2;
	return err;
}

#define CXT_PIN_MODE(xname, nid, dir) \
	{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = 0,  \
	  .info = conexant_ch_mode_info, \
	  .get = conexant_ch_mode_get, \
	  .put = conexant_ch_mode_put, \
	  .private_value = nid | (dir<<16) }

#endif /* CONFIG_SND_DEBUG */

/* Conexant 5045 specific */

static hda_nid_t cxt5045_dac_nids[1] = { 0x19 };
static hda_nid_t cxt5045_adc_nids[1] = { 0x1a };
static hda_nid_t cxt5045_capsrc_nids[1] = { 0x1a };
#define CXT5045_SPDIF_OUT	0x18

static struct hda_channel_mode cxt5045_modes[1] = {
	{ 2, NULL },
};

static struct hda_input_mux cxt5045_capture_source = {
	.num_items = 2,
	.items = {
		{ "IntMic", 0x1 },
		{ "ExtMic", 0x2 },
	}
};

static struct hda_input_mux cxt5045_capture_source_benq = {
	.num_items = 5,
	.items = {
		{ "IntMic", 0x1 },
		{ "ExtMic", 0x2 },
		{ "LineIn", 0x3 },
		{ "CD",     0x4 },
		{ "Mixer",  0x0 },
	}
};

static struct hda_input_mux cxt5045_capture_source_hp530 = {
	.num_items = 2,
	.items = {
		{ "ExtMic", 0x1 },
		{ "IntMic", 0x2 },
	}
};

/* turn on/off EAPD (+ mute HP) as a master switch */
static int cxt5045_hp_master_sw_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;
	unsigned int bits;

	if (!cxt_eapd_put(kcontrol, ucontrol))
		return 0;

	/* toggle internal speakers mute depending of presence of
	 * the headphone jack
	 */
	bits = (!spec->hp_present && spec->cur_eapd) ? 0 : HDA_AMP_MUTE;
	snd_hda_codec_amp_stereo(codec, 0x10, HDA_OUTPUT, 0,
				 HDA_AMP_MUTE, bits);

	bits = spec->cur_eapd ? 0 : HDA_AMP_MUTE;
	snd_hda_codec_amp_stereo(codec, 0x11, HDA_OUTPUT, 0,
				 HDA_AMP_MUTE, bits);
	return 1;
}

/* bind volumes of both NID 0x10 and 0x11 */
static struct hda_bind_ctls cxt5045_hp_bind_master_vol = {
	.ops = &snd_hda_bind_vol,
	.values = {
		HDA_COMPOSE_AMP_VAL(0x10, 3, 0, HDA_OUTPUT),
		HDA_COMPOSE_AMP_VAL(0x11, 3, 0, HDA_OUTPUT),
		0
	},
};

/* toggle input of built-in and mic jack appropriately */
static void cxt5045_hp_automic(struct hda_codec *codec)
{
	static struct hda_verb mic_jack_on[] = {
		{0x14, AC_VERB_SET_AMP_GAIN_MUTE, 0xb080},
		{0x12, AC_VERB_SET_AMP_GAIN_MUTE, 0xb000},
		{}
	};
	static struct hda_verb mic_jack_off[] = {
		{0x12, AC_VERB_SET_AMP_GAIN_MUTE, 0xb080},
		{0x14, AC_VERB_SET_AMP_GAIN_MUTE, 0xb000},
		{}
	};
	unsigned int present;

	present = snd_hda_codec_read(codec, 0x12, 0,
				     AC_VERB_GET_PIN_SENSE, 0) & 0x80000000;
	if (present)
		snd_hda_sequence_write(codec, mic_jack_on);
	else
		snd_hda_sequence_write(codec, mic_jack_off);
}


/* mute internal speaker if HP is plugged */
static void cxt5045_hp_automute(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	unsigned int bits;

	spec->hp_present = snd_hda_codec_read(codec, 0x11, 0,
				     AC_VERB_GET_PIN_SENSE, 0) & 0x80000000;

	bits = (spec->hp_present || !spec->cur_eapd) ? HDA_AMP_MUTE : 0; 
	snd_hda_codec_amp_stereo(codec, 0x10, HDA_OUTPUT, 0,
				 HDA_AMP_MUTE, bits);
}

/* unsolicited event for HP jack sensing */
static void cxt5045_hp_unsol_event(struct hda_codec *codec,
				   unsigned int res)
{
	res >>= 26;
	switch (res) {
	case CONEXANT_HP_EVENT:
		cxt5045_hp_automute(codec);
		break;
	case CONEXANT_MIC_EVENT:
		cxt5045_hp_automic(codec);
		break;

	}
}

static struct snd_kcontrol_new cxt5045_mixers[] = {
	HDA_CODEC_VOLUME("Int Mic Capture Volume", 0x1a, 0x01, HDA_INPUT),
	HDA_CODEC_MUTE("Int Mic Capture Switch", 0x1a, 0x01, HDA_INPUT),
	HDA_CODEC_VOLUME("Ext Mic Capture Volume", 0x1a, 0x02, HDA_INPUT),
	HDA_CODEC_MUTE("Ext Mic Capture Switch", 0x1a, 0x02, HDA_INPUT),
	HDA_CODEC_VOLUME("PCM Playback Volume", 0x17, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("PCM Playback Switch", 0x17, 0x0, HDA_INPUT),
	HDA_CODEC_VOLUME("Int Mic Playback Volume", 0x17, 0x1, HDA_INPUT),
	HDA_CODEC_MUTE("Int Mic Playback Switch", 0x17, 0x1, HDA_INPUT),
	HDA_CODEC_VOLUME("Ext Mic Playback Volume", 0x17, 0x2, HDA_INPUT),
	HDA_CODEC_MUTE("Ext Mic Playback Switch", 0x17, 0x2, HDA_INPUT),
	HDA_BIND_VOL("Master Playback Volume", &cxt5045_hp_bind_master_vol),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Switch",
		.info = cxt_eapd_info,
		.get = cxt_eapd_get,
		.put = cxt5045_hp_master_sw_put,
		.private_value = 0x10,
	},

	{}
};

static struct snd_kcontrol_new cxt5045_benq_mixers[] = {
	HDA_CODEC_VOLUME("CD Capture Volume", 0x1a, 0x04, HDA_INPUT),
	HDA_CODEC_MUTE("CD Capture Switch", 0x1a, 0x04, HDA_INPUT),
	HDA_CODEC_VOLUME("CD Playback Volume", 0x17, 0x4, HDA_INPUT),
	HDA_CODEC_MUTE("CD Playback Switch", 0x17, 0x4, HDA_INPUT),

	HDA_CODEC_VOLUME("Line In Capture Volume", 0x1a, 0x03, HDA_INPUT),
	HDA_CODEC_MUTE("Line In Capture Switch", 0x1a, 0x03, HDA_INPUT),
	HDA_CODEC_VOLUME("Line In Playback Volume", 0x17, 0x3, HDA_INPUT),
	HDA_CODEC_MUTE("Line In Playback Switch", 0x17, 0x3, HDA_INPUT),

	HDA_CODEC_VOLUME("Mixer Capture Volume", 0x1a, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("Mixer Capture Switch", 0x1a, 0x0, HDA_INPUT),

	{}
};

static struct snd_kcontrol_new cxt5045_mixers_hp530[] = {
	HDA_CODEC_VOLUME("Int Mic Capture Volume", 0x1a, 0x02, HDA_INPUT),
	HDA_CODEC_MUTE("Int Mic Capture Switch", 0x1a, 0x02, HDA_INPUT),
	HDA_CODEC_VOLUME("Ext Mic Capture Volume", 0x1a, 0x01, HDA_INPUT),
	HDA_CODEC_MUTE("Ext Mic Capture Switch", 0x1a, 0x01, HDA_INPUT),
	HDA_CODEC_VOLUME("PCM Playback Volume", 0x17, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("PCM Playback Switch", 0x17, 0x0, HDA_INPUT),
	HDA_CODEC_VOLUME("Int Mic Playback Volume", 0x17, 0x2, HDA_INPUT),
	HDA_CODEC_MUTE("Int Mic Playback Switch", 0x17, 0x2, HDA_INPUT),
	HDA_CODEC_VOLUME("Ext Mic Playback Volume", 0x17, 0x1, HDA_INPUT),
	HDA_CODEC_MUTE("Ext Mic Playback Switch", 0x17, 0x1, HDA_INPUT),
	HDA_BIND_VOL("Master Playback Volume", &cxt5045_hp_bind_master_vol),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Switch",
		.info = cxt_eapd_info,
		.get = cxt_eapd_get,
		.put = cxt5045_hp_master_sw_put,
		.private_value = 0x10,
	},

	{}
};

static struct hda_verb cxt5045_init_verbs[] = {
	/* Line in, Mic */
	{0x12, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN|AC_PINCTL_VREF_80 },
	{0x14, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN|AC_PINCTL_VREF_80 },
	/* HP, Amp  */
	{0x10, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x10, AC_VERB_SET_CONNECT_SEL, 0x1},
	{0x11, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP},
	{0x11, AC_VERB_SET_CONNECT_SEL, 0x1},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(2)},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(4)},
	/* Record selector: Int mic */
	{0x1a, AC_VERB_SET_CONNECT_SEL,0x1},
	{0x1a, AC_VERB_SET_AMP_GAIN_MUTE,
	 AC_AMP_SET_INPUT|AC_AMP_SET_RIGHT|AC_AMP_SET_LEFT|0x17},
	/* SPDIF route: PCM */
	{0x13, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{ 0x13, AC_VERB_SET_CONNECT_SEL, 0x0 },
	/* EAPD */
	{0x10, AC_VERB_SET_EAPD_BTLENABLE, 0x2 }, /* default on */ 
	{ } /* end */
};

static struct hda_verb cxt5045_benq_init_verbs[] = {
	/* Int Mic, Mic */
	{0x12, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN|AC_PINCTL_VREF_80 },
	{0x14, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN|AC_PINCTL_VREF_80 },
	/* Line In,HP, Amp  */
	{0x10, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x10, AC_VERB_SET_CONNECT_SEL, 0x1},
	{0x11, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN},
	{0x11, AC_VERB_SET_CONNECT_SEL, 0x1},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(2)},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(4)},
	/* Record selector: Int mic */
	{0x1a, AC_VERB_SET_CONNECT_SEL, 0x1},
	{0x1a, AC_VERB_SET_AMP_GAIN_MUTE,
	 AC_AMP_SET_INPUT|AC_AMP_SET_RIGHT|AC_AMP_SET_LEFT|0x17},
	/* SPDIF route: PCM */
	{0x13, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x13, AC_VERB_SET_CONNECT_SEL, 0x0},
	/* EAPD */
	{0x10, AC_VERB_SET_EAPD_BTLENABLE, 0x2}, /* default on */
	{ } /* end */
};

static struct hda_verb cxt5045_hp_sense_init_verbs[] = {
	/* pin sensing on HP jack */
	{0x11, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN | CONEXANT_HP_EVENT},
	{ } /* end */
};

static struct hda_verb cxt5045_mic_sense_init_verbs[] = {
	/* pin sensing on HP jack */
	{0x12, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN | CONEXANT_MIC_EVENT},
	{ } /* end */
};

#ifdef CONFIG_SND_DEBUG
/* Test configuration for debugging, modelled after the ALC260 test
 * configuration.
 */
static struct hda_input_mux cxt5045_test_capture_source = {
	.num_items = 5,
	.items = {
		{ "MIXER", 0x0 },
		{ "MIC1 pin", 0x1 },
		{ "LINE1 pin", 0x2 },
		{ "HP-OUT pin", 0x3 },
		{ "CD pin", 0x4 },
        },
};

static struct snd_kcontrol_new cxt5045_test_mixer[] = {

	/* Output controls */
	HDA_CODEC_VOLUME("Speaker Playback Volume", 0x10, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("Speaker Playback Switch", 0x10, 0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME("Node 11 Playback Volume", 0x11, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("Node 11 Playback Switch", 0x11, 0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME("Node 12 Playback Volume", 0x12, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("Node 12 Playback Switch", 0x12, 0x0, HDA_OUTPUT),
	
	/* Modes for retasking pin widgets */
	CXT_PIN_MODE("HP-OUT pin mode", 0x11, CXT_PIN_DIR_INOUT),
	CXT_PIN_MODE("LINE1 pin mode", 0x12, CXT_PIN_DIR_INOUT),

	/* EAPD Switch Control */
	CXT_EAPD_SWITCH("External Amplifier", 0x10, 0x0),

	/* Loopback mixer controls */

	HDA_CODEC_VOLUME("Mixer-1 Volume", 0x17, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("Mixer-1 Switch", 0x17, 0x0, HDA_INPUT),
	HDA_CODEC_VOLUME("Mixer-2 Volume", 0x17, 0x1, HDA_INPUT),
	HDA_CODEC_MUTE("Mixer-2 Switch", 0x17, 0x1, HDA_INPUT),
	HDA_CODEC_VOLUME("Mixer-3 Volume", 0x17, 0x2, HDA_INPUT),
	HDA_CODEC_MUTE("Mixer-3 Switch", 0x17, 0x2, HDA_INPUT),
	HDA_CODEC_VOLUME("Mixer-4 Volume", 0x17, 0x3, HDA_INPUT),
	HDA_CODEC_MUTE("Mixer-4 Switch", 0x17, 0x3, HDA_INPUT),
	HDA_CODEC_VOLUME("Mixer-5 Volume", 0x17, 0x4, HDA_INPUT),
	HDA_CODEC_MUTE("Mixer-5 Switch", 0x17, 0x4, HDA_INPUT),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Input Source",
		.info = conexant_mux_enum_info,
		.get = conexant_mux_enum_get,
		.put = conexant_mux_enum_put,
	},
	/* Audio input controls */
	HDA_CODEC_VOLUME("Input-1 Volume", 0x1a, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("Input-1 Switch", 0x1a, 0x0, HDA_INPUT),
	HDA_CODEC_VOLUME("Input-2 Volume", 0x1a, 0x1, HDA_INPUT),
	HDA_CODEC_MUTE("Input-2 Switch", 0x1a, 0x1, HDA_INPUT),
	HDA_CODEC_VOLUME("Input-3 Volume", 0x1a, 0x2, HDA_INPUT),
	HDA_CODEC_MUTE("Input-3 Switch", 0x1a, 0x2, HDA_INPUT),
	HDA_CODEC_VOLUME("Input-4 Volume", 0x1a, 0x3, HDA_INPUT),
	HDA_CODEC_MUTE("Input-4 Switch", 0x1a, 0x3, HDA_INPUT),
	HDA_CODEC_VOLUME("Input-5 Volume", 0x1a, 0x4, HDA_INPUT),
	HDA_CODEC_MUTE("Input-5 Switch", 0x1a, 0x4, HDA_INPUT),
	{ } /* end */
};

static struct hda_verb cxt5045_test_init_verbs[] = {
	/* Set connections */
	{ 0x10, AC_VERB_SET_CONNECT_SEL, 0x0 },
	{ 0x11, AC_VERB_SET_CONNECT_SEL, 0x0 },
	{ 0x12, AC_VERB_SET_CONNECT_SEL, 0x0 },
	/* Enable retasking pins as output, initially without power amp */
	{0x12, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x11, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},

	/* Disable digital (SPDIF) pins initially, but users can enable
	 * them via a mixer switch.  In the case of SPDIF-out, this initverb
	 * payload also sets the generation to 0, output to be in "consumer"
	 * PCM format, copyright asserted, no pre-emphasis and no validity
	 * control.
	 */
	{0x13, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x18, AC_VERB_SET_DIGI_CONVERT_1, 0},

	/* Start with output sum widgets muted and their output gains at min */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},

	/* Unmute retasking pin widget output buffers since the default
	 * state appears to be output.  As the pin mode is changed by the
	 * user the pin mode control will take care of enabling the pin's
	 * input/output buffers as needed.
	 */
	{0x12, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},
	{0x11, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},

	/* Mute capture amp left and right */
	{0x1a, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},

	/* Set ADC connection select to match default mixer setting (mic1
	 * pin)
	 */
	{0x1a, AC_VERB_SET_CONNECT_SEL, 0x00},
	{0x17, AC_VERB_SET_CONNECT_SEL, 0x00},

	/* Mute all inputs to mixer widget (even unconnected ones) */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)}, /* Mixer pin */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)}, /* Mic1 pin */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(2)}, /* Line pin */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)}, /* HP pin */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(4)}, /* CD pin */

	{ }
};
#endif


/* initialize jack-sensing, too */
static int cxt5045_init(struct hda_codec *codec)
{
	conexant_init(codec);
	cxt5045_hp_automute(codec);
	return 0;
}


enum {
	CXT5045_LAPTOP_HPSENSE,
	CXT5045_LAPTOP_MICSENSE,
	CXT5045_LAPTOP_HPMICSENSE,
	CXT5045_BENQ,
	CXT5045_LAPTOP_HP530,
#ifdef CONFIG_SND_DEBUG
	CXT5045_TEST,
#endif
	CXT5045_MODELS
};

static const char *cxt5045_models[CXT5045_MODELS] = {
	[CXT5045_LAPTOP_HPSENSE]	= "laptop-hpsense",
	[CXT5045_LAPTOP_MICSENSE]	= "laptop-micsense",
	[CXT5045_LAPTOP_HPMICSENSE]	= "laptop-hpmicsense",
	[CXT5045_BENQ]			= "benq",
	[CXT5045_LAPTOP_HP530]		= "laptop-hp530",
#ifdef CONFIG_SND_DEBUG
	[CXT5045_TEST]		= "test",
#endif
};

static struct snd_pci_quirk cxt5045_cfg_tbl[] = {
	SND_PCI_QUIRK(0x103c, 0x30d5, "HP 530", CXT5045_LAPTOP_HP530),
	SND_PCI_QUIRK_MASK(0x103c, 0xff00, 0x3000, "HP DV Series",
			   CXT5045_LAPTOP_HPSENSE),
	SND_PCI_QUIRK(0x1179, 0xff31, "Toshiba P105", CXT5045_LAPTOP_MICSENSE),
	SND_PCI_QUIRK(0x152d, 0x0753, "Benq R55E", CXT5045_BENQ),
	SND_PCI_QUIRK(0x1734, 0x10ad, "Fujitsu Si1520", CXT5045_LAPTOP_MICSENSE),
	SND_PCI_QUIRK(0x1734, 0x10cb, "Fujitsu Si3515", CXT5045_LAPTOP_HPMICSENSE),
	SND_PCI_QUIRK(0x1734, 0x110e, "Fujitsu V5505",
		      CXT5045_LAPTOP_HPMICSENSE),
	SND_PCI_QUIRK(0x1509, 0x1e40, "FIC", CXT5045_LAPTOP_HPMICSENSE),
	SND_PCI_QUIRK(0x1509, 0x2f05, "FIC", CXT5045_LAPTOP_HPMICSENSE),
	SND_PCI_QUIRK(0x1509, 0x2f06, "FIC", CXT5045_LAPTOP_HPMICSENSE),
	SND_PCI_QUIRK_MASK(0x1631, 0xff00, 0xc100, "Packard Bell",
			   CXT5045_LAPTOP_HPMICSENSE),
	SND_PCI_QUIRK(0x8086, 0x2111, "Conexant Reference board", CXT5045_LAPTOP_HPSENSE),
	{}
};

static int patch_cxt5045(struct hda_codec *codec)
{
	struct conexant_spec *spec;
	int board_config;

	spec = kzalloc(sizeof(*spec), GFP_KERNEL);
	if (!spec)
		return -ENOMEM;
	codec->spec = spec;
	codec->pin_amp_workaround = 1;

	spec->multiout.max_channels = 2;
	spec->multiout.num_dacs = ARRAY_SIZE(cxt5045_dac_nids);
	spec->multiout.dac_nids = cxt5045_dac_nids;
	spec->multiout.dig_out_nid = CXT5045_SPDIF_OUT;
	spec->num_adc_nids = 1;
	spec->adc_nids = cxt5045_adc_nids;
	spec->capsrc_nids = cxt5045_capsrc_nids;
	spec->input_mux = &cxt5045_capture_source;
	spec->num_mixers = 1;
	spec->mixers[0] = cxt5045_mixers;
	spec->num_init_verbs = 1;
	spec->init_verbs[0] = cxt5045_init_verbs;
	spec->spdif_route = 0;
	spec->num_channel_mode = ARRAY_SIZE(cxt5045_modes),
	spec->channel_mode = cxt5045_modes,


	codec->patch_ops = conexant_patch_ops;

	board_config = snd_hda_check_board_config(codec, CXT5045_MODELS,
						  cxt5045_models,
						  cxt5045_cfg_tbl);
	switch (board_config) {
	case CXT5045_LAPTOP_HPSENSE:
		codec->patch_ops.unsol_event = cxt5045_hp_unsol_event;
		spec->input_mux = &cxt5045_capture_source;
		spec->num_init_verbs = 2;
		spec->init_verbs[1] = cxt5045_hp_sense_init_verbs;
		spec->mixers[0] = cxt5045_mixers;
		codec->patch_ops.init = cxt5045_init;
		break;
	case CXT5045_LAPTOP_MICSENSE:
		codec->patch_ops.unsol_event = cxt5045_hp_unsol_event;
		spec->input_mux = &cxt5045_capture_source;
		spec->num_init_verbs = 2;
		spec->init_verbs[1] = cxt5045_mic_sense_init_verbs;
		spec->mixers[0] = cxt5045_mixers;
		codec->patch_ops.init = cxt5045_init;
		break;
	default:
	case CXT5045_LAPTOP_HPMICSENSE:
		codec->patch_ops.unsol_event = cxt5045_hp_unsol_event;
		spec->input_mux = &cxt5045_capture_source;
		spec->num_init_verbs = 3;
		spec->init_verbs[1] = cxt5045_hp_sense_init_verbs;
		spec->init_verbs[2] = cxt5045_mic_sense_init_verbs;
		spec->mixers[0] = cxt5045_mixers;
		codec->patch_ops.init = cxt5045_init;
		break;
	case CXT5045_BENQ:
		codec->patch_ops.unsol_event = cxt5045_hp_unsol_event;
		spec->input_mux = &cxt5045_capture_source_benq;
		spec->num_init_verbs = 1;
		spec->init_verbs[0] = cxt5045_benq_init_verbs;
		spec->mixers[0] = cxt5045_mixers;
		spec->mixers[1] = cxt5045_benq_mixers;
		spec->num_mixers = 2;
		codec->patch_ops.init = cxt5045_init;
		break;
	case CXT5045_LAPTOP_HP530:
		codec->patch_ops.unsol_event = cxt5045_hp_unsol_event;
		spec->input_mux = &cxt5045_capture_source_hp530;
		spec->num_init_verbs = 2;
		spec->init_verbs[1] = cxt5045_hp_sense_init_verbs;
		spec->mixers[0] = cxt5045_mixers_hp530;
		codec->patch_ops.init = cxt5045_init;
		break;
#ifdef CONFIG_SND_DEBUG
	case CXT5045_TEST:
		spec->input_mux = &cxt5045_test_capture_source;
		spec->mixers[0] = cxt5045_test_mixer;
		spec->init_verbs[0] = cxt5045_test_init_verbs;
		break;
		
#endif	
	}

	switch (codec->subsystem_id >> 16) {
	case 0x103c:
		/* HP laptop has a really bad sound over 0dB on NID 0x17.
		 * Fix max PCM level to 0 dB
		 * (originall it has 0x2b steps with 0dB offset 0x14)
		 */
		snd_hda_override_amp_caps(codec, 0x17, HDA_INPUT,
					  (0x14 << AC_AMPCAP_OFFSET_SHIFT) |
					  (0x14 << AC_AMPCAP_NUM_STEPS_SHIFT) |
					  (0x05 << AC_AMPCAP_STEP_SIZE_SHIFT) |
					  (1 << AC_AMPCAP_MUTE_SHIFT));
		break;
	}

	return 0;
}


/* Conexant 5047 specific */
#define CXT5047_SPDIF_OUT	0x11

static hda_nid_t cxt5047_dac_nids[1] = { 0x10 }; /* 0x1c */
static hda_nid_t cxt5047_adc_nids[1] = { 0x12 };
static hda_nid_t cxt5047_capsrc_nids[1] = { 0x1a };

static struct hda_channel_mode cxt5047_modes[1] = {
	{ 2, NULL },
};

static struct hda_input_mux cxt5047_toshiba_capture_source = {
	.num_items = 2,
	.items = {
		{ "ExtMic", 0x2 },
		{ "Line-In", 0x1 },
	}
};

/* turn on/off EAPD (+ mute HP) as a master switch */
static int cxt5047_hp_master_sw_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct conexant_spec *spec = codec->spec;
	unsigned int bits;

	if (!cxt_eapd_put(kcontrol, ucontrol))
		return 0;

	/* toggle internal speakers mute depending of presence of
	 * the headphone jack
	 */
	bits = (!spec->hp_present && spec->cur_eapd) ? 0 : HDA_AMP_MUTE;
	/* NOTE: Conexat codec needs the index for *OUTPUT* amp of
	 * pin widgets unlike other codecs.  In this case, we need to
	 * set index 0x01 for the volume from the mixer amp 0x19.
	 */
	snd_hda_codec_amp_stereo(codec, 0x1d, HDA_OUTPUT, 0x01,
				 HDA_AMP_MUTE, bits);
	bits = spec->cur_eapd ? 0 : HDA_AMP_MUTE;
	snd_hda_codec_amp_stereo(codec, 0x13, HDA_OUTPUT, 0,
				 HDA_AMP_MUTE, bits);
	return 1;
}

/* mute internal speaker if HP is plugged */
static void cxt5047_hp_automute(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	unsigned int bits;

	spec->hp_present = snd_hda_codec_read(codec, 0x13, 0,
				     AC_VERB_GET_PIN_SENSE, 0) & 0x80000000;

	bits = (spec->hp_present || !spec->cur_eapd) ? HDA_AMP_MUTE : 0;
	/* See the note in cxt5047_hp_master_sw_put */
	snd_hda_codec_amp_stereo(codec, 0x1d, HDA_OUTPUT, 0x01,
				 HDA_AMP_MUTE, bits);
}

/* toggle input of built-in and mic jack appropriately */
static void cxt5047_hp_automic(struct hda_codec *codec)
{
	static struct hda_verb mic_jack_on[] = {
		{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_MUTE},
		{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},
		{}
	};
	static struct hda_verb mic_jack_off[] = {
		{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_MUTE},
		{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},
		{}
	};
	unsigned int present;

	present = snd_hda_codec_read(codec, 0x15, 0,
				     AC_VERB_GET_PIN_SENSE, 0) & 0x80000000;
	if (present)
		snd_hda_sequence_write(codec, mic_jack_on);
	else
		snd_hda_sequence_write(codec, mic_jack_off);
}

/* unsolicited event for HP jack sensing */
static void cxt5047_hp_unsol_event(struct hda_codec *codec,
				  unsigned int res)
{
	switch (res >> 26) {
	case CONEXANT_HP_EVENT:
		cxt5047_hp_automute(codec);
		break;
	case CONEXANT_MIC_EVENT:
		cxt5047_hp_automic(codec);
		break;
	}
}

static struct snd_kcontrol_new cxt5047_base_mixers[] = {
	HDA_CODEC_VOLUME("Mic Playback Volume", 0x19, 0x02, HDA_INPUT),
	HDA_CODEC_MUTE("Mic Playback Switch", 0x19, 0x02, HDA_INPUT),
	HDA_CODEC_VOLUME("Mic Boost", 0x1a, 0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME("Capture Volume", 0x12, 0x03, HDA_INPUT),
	HDA_CODEC_MUTE("Capture Switch", 0x12, 0x03, HDA_INPUT),
	HDA_CODEC_VOLUME("PCM Volume", 0x10, 0x00, HDA_OUTPUT),
	HDA_CODEC_MUTE("PCM Switch", 0x10, 0x00, HDA_OUTPUT),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Switch",
		.info = cxt_eapd_info,
		.get = cxt_eapd_get,
		.put = cxt5047_hp_master_sw_put,
		.private_value = 0x13,
	},

	{}
};

static struct snd_kcontrol_new cxt5047_hp_spk_mixers[] = {
	/* See the note in cxt5047_hp_master_sw_put */
	HDA_CODEC_VOLUME("Speaker Playback Volume", 0x1d, 0x01, HDA_OUTPUT),
	HDA_CODEC_VOLUME("Headphone Playback Volume", 0x13, 0x00, HDA_OUTPUT),
	{}
};

static struct snd_kcontrol_new cxt5047_hp_only_mixers[] = {
	HDA_CODEC_VOLUME("Master Playback Volume", 0x13, 0x00, HDA_OUTPUT),
	{ } /* end */
};

static struct hda_verb cxt5047_init_verbs[] = {
	/* Line in, Mic, Built-in Mic */
	{0x14, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN },
	{0x15, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN|AC_PINCTL_VREF_50 },
	{0x17, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN|AC_PINCTL_VREF_50 },
	/* HP, Speaker  */
	{0x13, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP },
	{0x13, AC_VERB_SET_CONNECT_SEL, 0x0}, /* mixer(0x19) */
	{0x1d, AC_VERB_SET_CONNECT_SEL, 0x1}, /* mixer(0x19) */
	/* Record selector: Mic */
	{0x12, AC_VERB_SET_CONNECT_SEL,0x03},
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE,
	 AC_AMP_SET_INPUT|AC_AMP_SET_RIGHT|AC_AMP_SET_LEFT|0x17},
	{0x1A, AC_VERB_SET_CONNECT_SEL,0x02},
	{0x1A, AC_VERB_SET_AMP_GAIN_MUTE,
	 AC_AMP_SET_OUTPUT|AC_AMP_SET_RIGHT|AC_AMP_SET_LEFT|0x00},
	{0x1A, AC_VERB_SET_AMP_GAIN_MUTE,
	 AC_AMP_SET_OUTPUT|AC_AMP_SET_RIGHT|AC_AMP_SET_LEFT|0x03},
	/* SPDIF route: PCM */
	{ 0x18, AC_VERB_SET_CONNECT_SEL, 0x0 },
	/* Enable unsolicited events */
	{0x13, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN | CONEXANT_HP_EVENT},
	{0x15, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN | CONEXANT_MIC_EVENT},
	{ } /* end */
};

/* configuration for Toshiba Laptops */
static struct hda_verb cxt5047_toshiba_init_verbs[] = {
	{0x13, AC_VERB_SET_EAPD_BTLENABLE, 0x0}, /* default off */
	{}
};

/* Test configuration for debugging, modelled after the ALC260 test
 * configuration.
 */
#ifdef CONFIG_SND_DEBUG
static struct hda_input_mux cxt5047_test_capture_source = {
	.num_items = 4,
	.items = {
		{ "LINE1 pin", 0x0 },
		{ "MIC1 pin", 0x1 },
		{ "MIC2 pin", 0x2 },
		{ "CD pin", 0x3 },
        },
};

static struct snd_kcontrol_new cxt5047_test_mixer[] = {

	/* Output only controls */
	HDA_CODEC_VOLUME("OutAmp-1 Volume", 0x10, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("OutAmp-1 Switch", 0x10,0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME("OutAmp-2 Volume", 0x1c, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("OutAmp-2 Switch", 0x1c, 0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME("Speaker Playback Volume", 0x1d, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("Speaker Playback Switch", 0x1d, 0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME("HeadPhone Playback Volume", 0x13, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("HeadPhone Playback Switch", 0x13, 0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME("Line1-Out Playback Volume", 0x14, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("Line1-Out Playback Switch", 0x14, 0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME("Line2-Out Playback Volume", 0x15, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("Line2-Out Playback Switch", 0x15, 0x0, HDA_OUTPUT),

	/* Modes for retasking pin widgets */
	CXT_PIN_MODE("LINE1 pin mode", 0x14, CXT_PIN_DIR_INOUT),
	CXT_PIN_MODE("MIC1 pin mode", 0x15, CXT_PIN_DIR_INOUT),

	/* EAPD Switch Control */
	CXT_EAPD_SWITCH("External Amplifier", 0x13, 0x0),

	/* Loopback mixer controls */
	HDA_CODEC_VOLUME("MIC1 Playback Volume", 0x12, 0x01, HDA_INPUT),
	HDA_CODEC_MUTE("MIC1 Playback Switch", 0x12, 0x01, HDA_INPUT),
	HDA_CODEC_VOLUME("MIC2 Playback Volume", 0x12, 0x02, HDA_INPUT),
	HDA_CODEC_MUTE("MIC2 Playback Switch", 0x12, 0x02, HDA_INPUT),
	HDA_CODEC_VOLUME("LINE Playback Volume", 0x12, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("LINE Playback Switch", 0x12, 0x0, HDA_INPUT),
	HDA_CODEC_VOLUME("CD Playback Volume", 0x12, 0x04, HDA_INPUT),
	HDA_CODEC_MUTE("CD Playback Switch", 0x12, 0x04, HDA_INPUT),

	HDA_CODEC_VOLUME("Capture-1 Volume", 0x19, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("Capture-1 Switch", 0x19, 0x0, HDA_INPUT),
	HDA_CODEC_VOLUME("Capture-2 Volume", 0x19, 0x1, HDA_INPUT),
	HDA_CODEC_MUTE("Capture-2 Switch", 0x19, 0x1, HDA_INPUT),
	HDA_CODEC_VOLUME("Capture-3 Volume", 0x19, 0x2, HDA_INPUT),
	HDA_CODEC_MUTE("Capture-3 Switch", 0x19, 0x2, HDA_INPUT),
	HDA_CODEC_VOLUME("Capture-4 Volume", 0x19, 0x3, HDA_INPUT),
	HDA_CODEC_MUTE("Capture-4 Switch", 0x19, 0x3, HDA_INPUT),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Input Source",
		.info = conexant_mux_enum_info,
		.get = conexant_mux_enum_get,
		.put = conexant_mux_enum_put,
	},
	HDA_CODEC_VOLUME("Input-1 Volume", 0x1a, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("Input-1 Switch", 0x1a, 0x0, HDA_INPUT),
	HDA_CODEC_VOLUME("Input-2 Volume", 0x1a, 0x1, HDA_INPUT),
	HDA_CODEC_MUTE("Input-2 Switch", 0x1a, 0x1, HDA_INPUT),
	HDA_CODEC_VOLUME("Input-3 Volume", 0x1a, 0x2, HDA_INPUT),
	HDA_CODEC_MUTE("Input-3 Switch", 0x1a, 0x2, HDA_INPUT),
	HDA_CODEC_VOLUME("Input-4 Volume", 0x1a, 0x3, HDA_INPUT),
	HDA_CODEC_MUTE("Input-4 Switch", 0x1a, 0x3, HDA_INPUT),
	HDA_CODEC_VOLUME("Input-5 Volume", 0x1a, 0x4, HDA_INPUT),
	HDA_CODEC_MUTE("Input-5 Switch", 0x1a, 0x4, HDA_INPUT),

	{ } /* end */
};

static struct hda_verb cxt5047_test_init_verbs[] = {
	/* Enable retasking pins as output, initially without power amp */
	{0x15, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x14, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x13, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},

	/* Disable digital (SPDIF) pins initially, but users can enable
	 * them via a mixer switch.  In the case of SPDIF-out, this initverb
	 * payload also sets the generation to 0, output to be in "consumer"
	 * PCM format, copyright asserted, no pre-emphasis and no validity
	 * control.
	 */
	{0x18, AC_VERB_SET_DIGI_CONVERT_1, 0},

	/* Ensure mic1, mic2, line1 pin widgets take input from the 
	 * OUT1 sum bus when acting as an output.
	 */
	{0x1a, AC_VERB_SET_CONNECT_SEL, 0},
	{0x1b, AC_VERB_SET_CONNECT_SEL, 0},

	/* Start with output sum widgets muted and their output gains at min */
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},

	/* Unmute retasking pin widget output buffers since the default
	 * state appears to be output.  As the pin mode is changed by the
	 * user the pin mode control will take care of enabling the pin's
	 * input/output buffers as needed.
	 */
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},
	{0x13, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},

	/* Mute capture amp left and right */
	{0x12, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},

	/* Set ADC connection select to match default mixer setting (mic1
	 * pin)
	 */
	{0x12, AC_VERB_SET_CONNECT_SEL, 0x00},

	/* Mute all inputs to mixer widget (even unconnected ones) */
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)}, /* mic1 pin */
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)}, /* mic2 pin */
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(2)}, /* line1 pin */
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)}, /* line2 pin */
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(4)}, /* CD pin */
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(5)}, /* Beep-gen pin */
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(6)}, /* Line-out pin */
	{0x19, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(7)}, /* HP-pin pin */

	{ }
};
#endif


/* initialize jack-sensing, too */
static int cxt5047_hp_init(struct hda_codec *codec)
{
	conexant_init(codec);
	cxt5047_hp_automute(codec);
	return 0;
}


enum {
	CXT5047_LAPTOP,		/* Laptops w/o EAPD support */
	CXT5047_LAPTOP_HP,	/* Some HP laptops */
	CXT5047_LAPTOP_EAPD,	/* Laptops with EAPD support */
#ifdef CONFIG_SND_DEBUG
	CXT5047_TEST,
#endif
	CXT5047_MODELS
};

static const char *cxt5047_models[CXT5047_MODELS] = {
	[CXT5047_LAPTOP]	= "laptop",
	[CXT5047_LAPTOP_HP]	= "laptop-hp",
	[CXT5047_LAPTOP_EAPD]	= "laptop-eapd",
#ifdef CONFIG_SND_DEBUG
	[CXT5047_TEST]		= "test",
#endif
};

static struct snd_pci_quirk cxt5047_cfg_tbl[] = {
	SND_PCI_QUIRK(0x103c, 0x30a5, "HP DV5200T/DV8000T", CXT5047_LAPTOP_HP),
	SND_PCI_QUIRK_MASK(0x103c, 0xff00, 0x3000, "HP DV Series",
			   CXT5047_LAPTOP),
	SND_PCI_QUIRK(0x1179, 0xff31, "Toshiba P100", CXT5047_LAPTOP_EAPD),
	{}
};

static int patch_cxt5047(struct hda_codec *codec)
{
	struct conexant_spec *spec;
	int board_config;

	spec = kzalloc(sizeof(*spec), GFP_KERNEL);
	if (!spec)
		return -ENOMEM;
	codec->spec = spec;
	codec->pin_amp_workaround = 1;

	spec->multiout.max_channels = 2;
	spec->multiout.num_dacs = ARRAY_SIZE(cxt5047_dac_nids);
	spec->multiout.dac_nids = cxt5047_dac_nids;
	spec->multiout.dig_out_nid = CXT5047_SPDIF_OUT;
	spec->num_adc_nids = 1;
	spec->adc_nids = cxt5047_adc_nids;
	spec->capsrc_nids = cxt5047_capsrc_nids;
	spec->num_mixers = 1;
	spec->mixers[0] = cxt5047_base_mixers;
	spec->num_init_verbs = 1;
	spec->init_verbs[0] = cxt5047_init_verbs;
	spec->spdif_route = 0;
	spec->num_channel_mode = ARRAY_SIZE(cxt5047_modes),
	spec->channel_mode = cxt5047_modes,

	codec->patch_ops = conexant_patch_ops;

	board_config = snd_hda_check_board_config(codec, CXT5047_MODELS,
						  cxt5047_models,
						  cxt5047_cfg_tbl);
	switch (board_config) {
	case CXT5047_LAPTOP:
		spec->num_mixers = 2;
		spec->mixers[1] = cxt5047_hp_spk_mixers;
		codec->patch_ops.unsol_event = cxt5047_hp_unsol_event;
		break;
	case CXT5047_LAPTOP_HP:
		spec->num_mixers = 2;
		spec->mixers[1] = cxt5047_hp_only_mixers;
		codec->patch_ops.unsol_event = cxt5047_hp_unsol_event;
		codec->patch_ops.init = cxt5047_hp_init;
		break;
	case CXT5047_LAPTOP_EAPD:
		spec->input_mux = &cxt5047_toshiba_capture_source;
		spec->num_mixers = 2;
		spec->mixers[1] = cxt5047_hp_spk_mixers;
		spec->num_init_verbs = 2;
		spec->init_verbs[1] = cxt5047_toshiba_init_verbs;
		codec->patch_ops.unsol_event = cxt5047_hp_unsol_event;
		break;
#ifdef CONFIG_SND_DEBUG
	case CXT5047_TEST:
		spec->input_mux = &cxt5047_test_capture_source;
		spec->mixers[0] = cxt5047_test_mixer;
		spec->init_verbs[0] = cxt5047_test_init_verbs;
		codec->patch_ops.unsol_event = cxt5047_hp_unsol_event;
#endif	
	}
	spec->vmaster_nid = 0x13;
	return 0;
}

/* Conexant 5051 specific */
static hda_nid_t cxt5051_dac_nids[1] = { 0x10 };
static hda_nid_t cxt5051_adc_nids[2] = { 0x14, 0x15 };

static struct hda_channel_mode cxt5051_modes[1] = {
	{ 2, NULL },
};

static void cxt5051_update_speaker(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	unsigned int pinctl;
	pinctl = (!spec->hp_present && spec->cur_eapd) ? PIN_OUT : 0;
	snd_hda_codec_write(codec, 0x1a, 0, AC_VERB_SET_PIN_WIDGET_CONTROL,
			    pinctl);
}

/* turn on/off EAPD (+ mute HP) as a master switch */
static int cxt5051_hp_master_sw_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);

	if (!cxt_eapd_put(kcontrol, ucontrol))
		return 0;
	cxt5051_update_speaker(codec);
	return 1;
}

/* toggle input of built-in and mic jack appropriately */
static void cxt5051_portb_automic(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	unsigned int present;

	if (spec->no_auto_mic)
		return;
	present = snd_hda_codec_read(codec, 0x17, 0,
				     AC_VERB_GET_PIN_SENSE, 0) &
		AC_PINSENSE_PRESENCE;
	snd_hda_codec_write(codec, 0x14, 0,
			    AC_VERB_SET_CONNECT_SEL,
			    present ? 0x01 : 0x00);
}

/* switch the current ADC according to the jack state */
static void cxt5051_portc_automic(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	unsigned int present;
	hda_nid_t new_adc;

	if (spec->no_auto_mic)
		return;
	present = snd_hda_codec_read(codec, 0x18, 0,
				     AC_VERB_GET_PIN_SENSE, 0) &
		AC_PINSENSE_PRESENCE;
	if (present)
		spec->cur_adc_idx = 1;
	else
		spec->cur_adc_idx = 0;
	new_adc = spec->adc_nids[spec->cur_adc_idx];
	if (spec->cur_adc && spec->cur_adc != new_adc) {
		/* stream is running, let's swap the current ADC */
		snd_hda_codec_cleanup_stream(codec, spec->cur_adc);
		spec->cur_adc = new_adc;
		snd_hda_codec_setup_stream(codec, new_adc,
					   spec->cur_adc_stream_tag, 0,
					   spec->cur_adc_format);
	}
}

/* mute internal speaker if HP is plugged */
static void cxt5051_hp_automute(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;

	spec->hp_present = snd_hda_codec_read(codec, 0x16, 0,
				     AC_VERB_GET_PIN_SENSE, 0) &
		AC_PINSENSE_PRESENCE;
	cxt5051_update_speaker(codec);
}

/* unsolicited event for HP jack sensing */
static void cxt5051_hp_unsol_event(struct hda_codec *codec,
				   unsigned int res)
{
	int nid = (res & AC_UNSOL_RES_SUBTAG) >> 20;
	switch (res >> 26) {
	case CONEXANT_HP_EVENT:
		cxt5051_hp_automute(codec);
		break;
	case CXT5051_PORTB_EVENT:
		cxt5051_portb_automic(codec);
		break;
	case CXT5051_PORTC_EVENT:
		cxt5051_portc_automic(codec);
		break;
	}
	conexant_report_jack(codec, nid);
}

static struct snd_kcontrol_new cxt5051_mixers[] = {
	HDA_CODEC_VOLUME("Internal Mic Volume", 0x14, 0x00, HDA_INPUT),
	HDA_CODEC_MUTE("Internal Mic Switch", 0x14, 0x00, HDA_INPUT),
	HDA_CODEC_VOLUME("External Mic Volume", 0x14, 0x01, HDA_INPUT),
	HDA_CODEC_MUTE("External Mic Switch", 0x14, 0x01, HDA_INPUT),
	HDA_CODEC_VOLUME("Docking Mic Volume", 0x15, 0x00, HDA_INPUT),
	HDA_CODEC_MUTE("Docking Mic Switch", 0x15, 0x00, HDA_INPUT),
	HDA_CODEC_VOLUME("Master Playback Volume", 0x10, 0x00, HDA_OUTPUT),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Switch",
		.info = cxt_eapd_info,
		.get = cxt_eapd_get,
		.put = cxt5051_hp_master_sw_put,
		.private_value = 0x1a,
	},

	{}
};

static struct snd_kcontrol_new cxt5051_hp_mixers[] = {
	HDA_CODEC_VOLUME("Internal Mic Volume", 0x14, 0x00, HDA_INPUT),
	HDA_CODEC_MUTE("Internal Mic Switch", 0x14, 0x00, HDA_INPUT),
	HDA_CODEC_VOLUME("External Mic Volume", 0x15, 0x00, HDA_INPUT),
	HDA_CODEC_MUTE("External Mic Switch", 0x15, 0x00, HDA_INPUT),
	HDA_CODEC_VOLUME("Master Playback Volume", 0x10, 0x00, HDA_OUTPUT),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Switch",
		.info = cxt_eapd_info,
		.get = cxt_eapd_get,
		.put = cxt5051_hp_master_sw_put,
		.private_value = 0x1a,
	},

	{}
};

static struct snd_kcontrol_new cxt5051_hp_dv6736_mixers[] = {
	HDA_CODEC_VOLUME("Mic Volume", 0x14, 0x00, HDA_INPUT),
	HDA_CODEC_MUTE("Mic Switch", 0x14, 0x00, HDA_INPUT),
	HDA_CODEC_VOLUME("Master Playback Volume", 0x10, 0x00, HDA_OUTPUT),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Switch",
		.info = cxt_eapd_info,
		.get = cxt_eapd_get,
		.put = cxt5051_hp_master_sw_put,
		.private_value = 0x1a,
	},

	{}
};

static struct hda_verb cxt5051_init_verbs[] = {
	/* Line in, Mic */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x03},
	{0x17, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80},
	{0x18, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x03},
	{0x18, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80},
	{0x1d, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN},
	{0x1d, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x03},
	/* SPK  */
	{0x1a, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x1a, AC_VERB_SET_CONNECT_SEL, 0x00},
	/* HP, Amp  */
	{0x16, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP},
	{0x16, AC_VERB_SET_CONNECT_SEL, 0x00},
	/* DAC1 */	
	{0x10, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},
	/* Record selector: Int mic */
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x44},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(1) | 0x44},
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x44},
	/* SPDIF route: PCM */
	{0x1c, AC_VERB_SET_CONNECT_SEL, 0x0},
	/* EAPD */
	{0x1a, AC_VERB_SET_EAPD_BTLENABLE, 0x2}, /* default on */ 
	{0x16, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN|CONEXANT_HP_EVENT},
	{0x17, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN|CXT5051_PORTB_EVENT},
	{0x18, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN|CXT5051_PORTC_EVENT},
	{ } /* end */
};

static struct hda_verb cxt5051_hp_dv6736_init_verbs[] = {
	/* Line in, Mic */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x03},
	{0x17, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80},
	{0x18, AC_VERB_SET_PIN_WIDGET_CONTROL, 0x0},
	{0x1d, AC_VERB_SET_PIN_WIDGET_CONTROL, 0x0},
	/* SPK  */
	{0x1a, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x1a, AC_VERB_SET_CONNECT_SEL, 0x00},
	/* HP, Amp  */
	{0x16, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP},
	{0x16, AC_VERB_SET_CONNECT_SEL, 0x00},
	/* DAC1 */
	{0x10, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},
	/* Record selector: Int mic */
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(1) | 0x44},
	{0x14, AC_VERB_SET_CONNECT_SEL, 0x1},
	/* SPDIF route: PCM */
	{0x1c, AC_VERB_SET_CONNECT_SEL, 0x0},
	/* EAPD */
	{0x1a, AC_VERB_SET_EAPD_BTLENABLE, 0x2}, /* default on */
	{0x16, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN|CONEXANT_HP_EVENT},
	{0x17, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN|CXT5051_PORTB_EVENT},
	{ } /* end */
};

static struct hda_verb cxt5051_lenovo_x200_init_verbs[] = {
	/* Line in, Mic */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x03},
	{0x17, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80},
	{0x18, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x03},
	{0x18, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80},
	{0x1d, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN},
	{0x1d, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x03},
	/* SPK  */
	{0x1a, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x1a, AC_VERB_SET_CONNECT_SEL, 0x00},
	/* HP, Amp  */
	{0x16, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP},
	{0x16, AC_VERB_SET_CONNECT_SEL, 0x00},
	/* Docking HP */
	{0x19, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP},
	{0x19, AC_VERB_SET_CONNECT_SEL, 0x00},
	/* DAC1 */
	{0x10, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},
	/* Record selector: Int mic */
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x44},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(1) | 0x44},
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x44},
	/* SPDIF route: PCM */
	{0x1c, AC_VERB_SET_CONNECT_SEL, 0x0},
	/* EAPD */
	{0x1a, AC_VERB_SET_EAPD_BTLENABLE, 0x2}, /* default on */
	{0x16, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN|CONEXANT_HP_EVENT},
	{0x17, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN|CXT5051_PORTB_EVENT},
	{0x18, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN|CXT5051_PORTC_EVENT},
	{0x19, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN|CONEXANT_HP_EVENT},
	{ } /* end */
};

/* initialize jack-sensing, too */
static int cxt5051_init(struct hda_codec *codec)
{
	conexant_init(codec);
	conexant_init_jacks(codec);
	if (codec->patch_ops.unsol_event) {
		cxt5051_hp_automute(codec);
		cxt5051_portb_automic(codec);
		cxt5051_portc_automic(codec);
	}
	return 0;
}


enum {
	CXT5051_LAPTOP,	 /* Laptops w/ EAPD support */
	CXT5051_HP,	/* no docking */
	CXT5051_HP_DV6736,	/* HP without mic switch */
	CXT5051_LENOVO_X200,	/* Lenovo X200 laptop */
	CXT5051_MODELS
};

static const char *cxt5051_models[CXT5051_MODELS] = {
	[CXT5051_LAPTOP]	= "laptop",
	[CXT5051_HP]		= "hp",
	[CXT5051_HP_DV6736]	= "hp-dv6736",
	[CXT5051_LENOVO_X200]	= "lenovo-x200",
};

static struct snd_pci_quirk cxt5051_cfg_tbl[] = {
	SND_PCI_QUIRK(0x103c, 0x30cf, "HP DV6736", CXT5051_HP_DV6736),
	SND_PCI_QUIRK(0x103c, 0x360b, "Compaq Presario CQ60", CXT5051_HP),
	SND_PCI_QUIRK(0x14f1, 0x0101, "Conexant Reference board",
		      CXT5051_LAPTOP),
	SND_PCI_QUIRK(0x14f1, 0x5051, "HP Spartan 1.1", CXT5051_HP),
	SND_PCI_QUIRK(0x17aa, 0x20f2, "Lenovo X200", CXT5051_LENOVO_X200),
	{}
};

static int patch_cxt5051(struct hda_codec *codec)
{
	struct conexant_spec *spec;
	int board_config;

	spec = kzalloc(sizeof(*spec), GFP_KERNEL);
	if (!spec)
		return -ENOMEM;
	codec->spec = spec;
	codec->pin_amp_workaround = 1;

	codec->patch_ops = conexant_patch_ops;
	codec->patch_ops.init = cxt5051_init;

	spec->multiout.max_channels = 2;
	spec->multiout.num_dacs = ARRAY_SIZE(cxt5051_dac_nids);
	spec->multiout.dac_nids = cxt5051_dac_nids;
	spec->multiout.dig_out_nid = CXT5051_SPDIF_OUT;
	spec->num_adc_nids = 1; /* not 2; via auto-mic switch */
	spec->adc_nids = cxt5051_adc_nids;
	spec->num_mixers = 1;
	spec->mixers[0] = cxt5051_mixers;
	spec->num_init_verbs = 1;
	spec->init_verbs[0] = cxt5051_init_verbs;
	spec->spdif_route = 0;
	spec->num_channel_mode = ARRAY_SIZE(cxt5051_modes);
	spec->channel_mode = cxt5051_modes;
	spec->cur_adc = 0;
	spec->cur_adc_idx = 0;

	codec->patch_ops.unsol_event = cxt5051_hp_unsol_event;

	board_config = snd_hda_check_board_config(codec, CXT5051_MODELS,
						  cxt5051_models,
						  cxt5051_cfg_tbl);
	switch (board_config) {
	case CXT5051_HP:
		spec->mixers[0] = cxt5051_hp_mixers;
		break;
	case CXT5051_HP_DV6736:
		spec->init_verbs[0] = cxt5051_hp_dv6736_init_verbs;
		spec->mixers[0] = cxt5051_hp_dv6736_mixers;
		spec->no_auto_mic = 1;
		break;
	case CXT5051_LENOVO_X200:
		spec->init_verbs[0] = cxt5051_lenovo_x200_init_verbs;
		break;
	}

	return 0;
}

/* Conexant 5066 specific */

static hda_nid_t cxt5066_dac_nids[1] = { 0x10 };
static hda_nid_t cxt5066_adc_nids[3] = { 0x14, 0x15, 0x16 };
static hda_nid_t cxt5066_capsrc_nids[1] = { 0x17 };
#define CXT5066_SPDIF_OUT	0x21

/* OLPC's microphone port is DC coupled for use with external sensors,
 * therefore we use a 50% mic bias in order to center the input signal with
 * the DC input range of the codec. */
#define CXT5066_OLPC_EXT_MIC_BIAS PIN_VREF50

static struct hda_channel_mode cxt5066_modes[1] = {
	{ 2, NULL },
};

static void cxt5066_update_speaker(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	unsigned int pinctl;

	snd_printdd("CXT5066: update speaker, hp_present=%d\n",
		spec->hp_present);

	/* Port A (HP) */
	pinctl = ((spec->hp_present & 1) && spec->cur_eapd) ? PIN_HP : 0;
	snd_hda_codec_write(codec, 0x19, 0, AC_VERB_SET_PIN_WIDGET_CONTROL,
			pinctl);

	/* Port D (HP/LO) */
	pinctl = ((spec->hp_present & 2) && spec->cur_eapd)
		? spec->port_d_mode : 0;
	snd_hda_codec_write(codec, 0x1c, 0, AC_VERB_SET_PIN_WIDGET_CONTROL,
			pinctl);

	/* CLASS_D AMP */
	pinctl = (!spec->hp_present && spec->cur_eapd) ? PIN_OUT : 0;
	snd_hda_codec_write(codec, 0x1f, 0, AC_VERB_SET_PIN_WIDGET_CONTROL,
			pinctl);

	if (spec->dell_automute) {
		/* DELL AIO Port Rule: PortA > PortD > IntSpk */
		pinctl = (!(spec->hp_present & 1) && spec->cur_eapd)
			? PIN_OUT : 0;
		snd_hda_codec_write(codec, 0x1c, 0,
			AC_VERB_SET_PIN_WIDGET_CONTROL, pinctl);
	}
}

/* turn on/off EAPD (+ mute HP) as a master switch */
static int cxt5066_hp_master_sw_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);

	if (!cxt_eapd_put(kcontrol, ucontrol))
		return 0;

	cxt5066_update_speaker(codec);
	return 1;
}

/* toggle input of built-in and mic jack appropriately */
static void cxt5066_automic(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	struct hda_verb ext_mic_present[] = {
		/* enable external mic, port B */
		{0x1a, AC_VERB_SET_PIN_WIDGET_CONTROL, spec->ext_mic_bias},

		/* switch to external mic input */
		{0x17, AC_VERB_SET_CONNECT_SEL, 0},

		/* disable internal mic, port C */
		{0x1b, AC_VERB_SET_PIN_WIDGET_CONTROL, 0},
		{}
	};
	static struct hda_verb ext_mic_absent[] = {
		/* enable internal mic, port C */
		{0x1b, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80},

		/* switch to internal mic input */
		{0x17, AC_VERB_SET_CONNECT_SEL, 1},

		/* disable external mic, port B */
		{0x1a, AC_VERB_SET_PIN_WIDGET_CONTROL, 0},
		{}
	};
	unsigned int present;

	present = snd_hda_codec_read(codec, 0x1a, 0,
				     AC_VERB_GET_PIN_SENSE, 0) & 0x80000000;
	if (present) {
		snd_printdd("CXT5066: external microphone detected\n");
		snd_hda_sequence_write(codec, ext_mic_present);
	} else {
		snd_printdd("CXT5066: external microphone absent\n");
		snd_hda_sequence_write(codec, ext_mic_absent);
	}
}

/* mute internal speaker if HP is plugged */
static void cxt5066_hp_automute(struct hda_codec *codec)
{
	struct conexant_spec *spec = codec->spec;
	unsigned int portA, portD;

	/* Port A */
	portA = snd_hda_codec_read(codec, 0x19, 0, AC_VERB_GET_PIN_SENSE, 0)
		& AC_PINSENSE_PRESENCE;

	/* Port D */
	portD = (snd_hda_codec_read(codec, 0x1c, 0, AC_VERB_GET_PIN_SENSE, 0)
		& AC_PINSENSE_PRESENCE) << 1;

	spec->hp_present = !!(portA | portD);
	snd_printdd("CXT5066: hp automute portA=%x portD=%x present=%d\n",
		portA, portD, spec->hp_present);
	cxt5066_update_speaker(codec);
}

/* unsolicited event for jack sensing */
static void cxt5066_unsol_event(struct hda_codec *codec, unsigned int res)
{
	snd_printdd("CXT5066: unsol event %x (%x)\n", res, res >> 26);
	switch (res >> 26) {
	case CONEXANT_HP_EVENT:
		cxt5066_hp_automute(codec);
		break;
	case CONEXANT_MIC_EVENT:
		cxt5066_automic(codec);
		break;
	}
}

static const struct hda_input_mux cxt5066_analog_mic_boost = {
	.num_items = 5,
	.items = {
		{ "0dB",  0 },
		{ "10dB", 1 },
		{ "20dB", 2 },
		{ "30dB", 3 },
		{ "40dB", 4 },
	},
};

static int cxt5066_mic_boost_mux_enum_info(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_info *uinfo)
{
	return snd_hda_input_mux_info(&cxt5066_analog_mic_boost, uinfo);
}

static int cxt5066_mic_boost_mux_enum_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	int val;

	val = snd_hda_codec_read(codec, 0x17, 0,
		AC_VERB_GET_AMP_GAIN_MUTE, AC_AMP_GET_OUTPUT);

	ucontrol->value.enumerated.item[0] = val & AC_AMP_GAIN;
	return 0;
}

static int cxt5066_mic_boost_mux_enum_put(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	const struct hda_input_mux *imux = &cxt5066_analog_mic_boost;
	unsigned int idx;

	if (!imux->num_items)
		return 0;
	idx = ucontrol->value.enumerated.item[0];
	if (idx >= imux->num_items)
		idx = imux->num_items - 1;

	snd_hda_codec_write_cache(codec, 0x17, 0,
		AC_VERB_SET_AMP_GAIN_MUTE,
		AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT |
			imux->items[idx].index);

	return 1;
}

static struct hda_input_mux cxt5066_capture_source = {
	.num_items = 4,
	.items = {
		{ "Mic B", 0 },
		{ "Mic C", 1 },
		{ "Mic E", 2 },
		{ "Mic F", 3 },
	},
};

static struct hda_bind_ctls cxt5066_bind_capture_vol_others = {
	.ops = &snd_hda_bind_vol,
	.values = {
		HDA_COMPOSE_AMP_VAL(0x14, 3, 0, HDA_INPUT),
		HDA_COMPOSE_AMP_VAL(0x14, 3, 2, HDA_INPUT),
		0
	},
};

static struct hda_bind_ctls cxt5066_bind_capture_sw_others = {
	.ops = &snd_hda_bind_sw,
	.values = {
		HDA_COMPOSE_AMP_VAL(0x14, 3, 0, HDA_INPUT),
		HDA_COMPOSE_AMP_VAL(0x14, 3, 2, HDA_INPUT),
		0
	},
};

static struct snd_kcontrol_new cxt5066_mixer_master[] = {
	HDA_CODEC_VOLUME("Master Playback Volume", 0x10, 0x00, HDA_OUTPUT),
	{}
};

static struct snd_kcontrol_new cxt5066_mixer_master_olpc[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
				  SNDRV_CTL_ELEM_ACCESS_TLV_READ |
				  SNDRV_CTL_ELEM_ACCESS_TLV_CALLBACK,
		.info = snd_hda_mixer_amp_volume_info,
		.get = snd_hda_mixer_amp_volume_get,
		.put = snd_hda_mixer_amp_volume_put,
		.tlv = { .c = snd_hda_mixer_amp_tlv },
		/* offset by 28 volume steps to limit minimum gain to -46dB */
		.private_value =
			HDA_COMPOSE_AMP_VAL_OFS(0x10, 3, 0, HDA_OUTPUT, 28),
	},
	{}
};

static struct snd_kcontrol_new cxt5066_mixers[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Switch",
		.info = cxt_eapd_info,
		.get = cxt_eapd_get,
		.put = cxt5066_hp_master_sw_put,
		.private_value = 0x1d,
	},

	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Analog Mic Boost Capture Enum",
		.info = cxt5066_mic_boost_mux_enum_info,
		.get = cxt5066_mic_boost_mux_enum_get,
		.put = cxt5066_mic_boost_mux_enum_put,
	},

	HDA_BIND_VOL("Capture Volume", &cxt5066_bind_capture_vol_others),
	HDA_BIND_SW("Capture Switch", &cxt5066_bind_capture_sw_others),
	{}
};

static struct hda_verb cxt5066_init_verbs[] = {
	{0x1a, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80}, /* Port B */
	{0x1b, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80}, /* Port C */
	{0x1e, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN}, /* Port F */
	{0x1d, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_IN}, /* Port E */

	/* Speakers  */
	{0x1f, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x1f, AC_VERB_SET_CONNECT_SEL, 0x00}, /* DAC1 */

	/* HP, Amp  */
	{0x19, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP},
	{0x19, AC_VERB_SET_CONNECT_SEL, 0x00}, /* DAC1 */

	{0x1c, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP},
	{0x1c, AC_VERB_SET_CONNECT_SEL, 0x00}, /* DAC1 */

	/* DAC1 */
	{0x10, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},

	/* Node 14 connections: 0x17 0x18 0x23 0x24 0x27 */
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x50},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(2) | 0x50},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(4)},

	/* no digital microphone support yet */
	{0x23, AC_VERB_SET_PIN_WIDGET_CONTROL, 0},

	/* Audio input selector */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE | 0x3},

	/* SPDIF route: PCM */
	{0x20, AC_VERB_SET_CONNECT_SEL, 0x0},
	{0x22, AC_VERB_SET_CONNECT_SEL, 0x0},

	{0x20, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x22, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},

	/* EAPD */
	{0x1d, AC_VERB_SET_EAPD_BTLENABLE, 0x2}, /* default on */

	/* not handling these yet */
	{0x19, AC_VERB_SET_UNSOLICITED_ENABLE, 0},
	{0x1a, AC_VERB_SET_UNSOLICITED_ENABLE, 0},
	{0x1b, AC_VERB_SET_UNSOLICITED_ENABLE, 0},
	{0x1c, AC_VERB_SET_UNSOLICITED_ENABLE, 0},
	{0x1d, AC_VERB_SET_UNSOLICITED_ENABLE, 0},
	{0x1e, AC_VERB_SET_UNSOLICITED_ENABLE, 0},
	{0x20, AC_VERB_SET_UNSOLICITED_ENABLE, 0},
	{0x22, AC_VERB_SET_UNSOLICITED_ENABLE, 0},
	{ } /* end */
};

static struct hda_verb cxt5066_init_verbs_olpc[] = {
	/* Port A: headphones */
	{0x19, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP},
	{0x19, AC_VERB_SET_CONNECT_SEL, 0x00}, /* DAC1 */

	/* Port B: external microphone */
	{0x1a, AC_VERB_SET_PIN_WIDGET_CONTROL, CXT5066_OLPC_EXT_MIC_BIAS},

	/* Port C: internal microphone */
	{0x1b, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80},

	/* Port D: unused */
	{0x1c, AC_VERB_SET_PIN_WIDGET_CONTROL, 0},

	/* Port E: unused, but has primary EAPD */
	{0x1d, AC_VERB_SET_PIN_WIDGET_CONTROL, 0},
	{0x1d, AC_VERB_SET_EAPD_BTLENABLE, 0x2}, /* default on */

	/* Port F: unused */
	{0x1e, AC_VERB_SET_PIN_WIDGET_CONTROL, 0},

	/* Port G: internal speakers */
	{0x1f, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x1f, AC_VERB_SET_CONNECT_SEL, 0x00}, /* DAC1 */

	/* DAC1 */
	{0x10, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE},

	/* DAC2: unused */
	{0x11, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_MUTE},

	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0) | 0x50},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(2)},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)},
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(2)},
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)},
	{0x16, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x16, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},
	{0x16, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(2)},
	{0x16, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)},

	/* Disable digital microphone port */
	{0x23, AC_VERB_SET_PIN_WIDGET_CONTROL, 0},

	/* Audio input selectors */
	{0x17, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE | 0x3},
	{0x18, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_MUTE },

	/* Disable SPDIF */
	{0x20, AC_VERB_SET_PIN_WIDGET_CONTROL, 0},
	{0x22, AC_VERB_SET_PIN_WIDGET_CONTROL, 0},

	/* enable unsolicited events for Port A and B */
	{0x19, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN | CONEXANT_HP_EVENT},
	{0x1a, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN | CONEXANT_MIC_EVENT},
	{ } /* end */
};

static struct hda_verb cxt5066_init_verbs_portd_lo[] = {
	{0x1c, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{ } /* end */
};

/* initialize jack-sensing, too */
static int cxt5066_init(struct hda_codec *codec)
{
	snd_printdd("CXT5066: init\n");
	conexant_init(codec);
	if (codec->patch_ops.unsol_event) {
		cxt5066_hp_automute(codec);
		cxt5066_automic(codec);
	}
	return 0;
}

enum {
	CXT5066_LAPTOP,			/* Laptops w/ EAPD support */
	CXT5066_DELL_LAPTOP,	/* Dell Laptop */
	CXT5066_OLPC_XO_1_5,	/* OLPC XO 1.5 */
	CXT5066_MODELS
};

static const char *cxt5066_models[CXT5066_MODELS] = {
	[CXT5066_LAPTOP]		= "laptop",
	[CXT5066_DELL_LAPTOP]	= "dell-laptop",
	[CXT5066_OLPC_XO_1_5]	= "olpc-xo-1_5",
};

static struct snd_pci_quirk cxt5066_cfg_tbl[] = {
	SND_PCI_QUIRK(0x14f1, 0x0101, "Conexant Reference board",
		      CXT5066_LAPTOP),
	SND_PCI_QUIRK(0x1028, 0x02f5, "Dell",
		      CXT5066_DELL_LAPTOP),
	SND_PCI_QUIRK(0x152d, 0x0833, "OLPC XO-1.5", CXT5066_OLPC_XO_1_5),
	{}
};

static int patch_cxt5066(struct hda_codec *codec)
{
	struct conexant_spec *spec;
	int board_config;

	spec = kzalloc(sizeof(*spec), GFP_KERNEL);
	if (!spec)
		return -ENOMEM;
	codec->spec = spec;

	codec->patch_ops = conexant_patch_ops;
	codec->patch_ops.init = cxt5066_init;

	spec->dell_automute = 0;
	spec->multiout.max_channels = 2;
	spec->multiout.num_dacs = ARRAY_SIZE(cxt5066_dac_nids);
	spec->multiout.dac_nids = cxt5066_dac_nids;
	spec->multiout.dig_out_nid = CXT5066_SPDIF_OUT;
	spec->num_adc_nids = 1;
	spec->adc_nids = cxt5066_adc_nids;
	spec->capsrc_nids = cxt5066_capsrc_nids;
	spec->input_mux = &cxt5066_capture_source;

	spec->port_d_mode = PIN_HP;
	spec->ext_mic_bias = PIN_VREF80;

	spec->num_init_verbs = 1;
	spec->init_verbs[0] = cxt5066_init_verbs;
	spec->num_channel_mode = ARRAY_SIZE(cxt5066_modes);
	spec->channel_mode = cxt5066_modes;
	spec->cur_adc = 0;
	spec->cur_adc_idx = 0;

	board_config = snd_hda_check_board_config(codec, CXT5066_MODELS,
						  cxt5066_models, cxt5066_cfg_tbl);
	switch (board_config) {
	default:
	case CXT5066_LAPTOP:
		spec->mixers[spec->num_mixers++] = cxt5066_mixer_master;
		spec->mixers[spec->num_mixers++] = cxt5066_mixers;
		break;
	case CXT5066_DELL_LAPTOP:
		spec->mixers[spec->num_mixers++] = cxt5066_mixer_master;
		spec->mixers[spec->num_mixers++] = cxt5066_mixers;

		spec->port_d_mode = PIN_OUT;
		spec->init_verbs[spec->num_init_verbs] = cxt5066_init_verbs_portd_lo;
		spec->num_init_verbs++;
		spec->dell_automute = 1;
		break;
	case CXT5066_OLPC_XO_1_5:
		codec->patch_ops.unsol_event = cxt5066_unsol_event;
		spec->init_verbs[0] = cxt5066_init_verbs_olpc;
		spec->mixers[spec->num_mixers++] = cxt5066_mixer_master_olpc;
		spec->mixers[spec->num_mixers++] = cxt5066_mixers;
		spec->port_d_mode = 0;
		spec->ext_mic_bias = CXT5066_OLPC_EXT_MIC_BIAS;

		/* no S/PDIF out */
		spec->multiout.dig_out_nid = 0;

		/* input source automatically selected */
		spec->input_mux = NULL;
		break;
	}

	return 0;
}

/*
 */

static struct hda_codec_preset snd_hda_preset_conexant[] = {
	{ .id = 0x14f15045, .name = "CX20549 (Venice)",
	  .patch = patch_cxt5045 },
	{ .id = 0x14f15047, .name = "CX20551 (Waikiki)",
	  .patch = patch_cxt5047 },
	{ .id = 0x14f15051, .name = "CX20561 (Hermosa)",
	  .patch = patch_cxt5051 },
	{ .id = 0x14f15066, .name = "CX20582 (Pebble)",
	  .patch = patch_cxt5066 },
	{} /* terminator */
};

MODULE_ALIAS("snd-hda-codec-id:14f15045");
MODULE_ALIAS("snd-hda-codec-id:14f15047");
MODULE_ALIAS("snd-hda-codec-id:14f15051");
MODULE_ALIAS("snd-hda-codec-id:14f15066");

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Conexant HD-audio codec");

static struct hda_codec_preset_list conexant_list = {
	.preset = snd_hda_preset_conexant,
	.owner = THIS_MODULE,
};

static int __init patch_conexant_init(void)
{
	return snd_hda_add_codec_preset(&conexant_list);
}

static void __exit patch_conexant_exit(void)
{
	snd_hda_delete_codec_preset(&conexant_list);
}

module_init(patch_conexant_init)
module_exit(patch_conexant_exit)
