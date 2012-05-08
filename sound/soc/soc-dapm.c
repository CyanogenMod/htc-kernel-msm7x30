/*
 * soc-dapm.c  --  ALSA SoC Dynamic Audio Power Management
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood <lrg@slimlogic.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  only version 2 of the  License.
 *
 *  Features:
 *    o Changes power status of internal codec blocks depending on the
 *      dynamic configuration of codec internal audio paths and active
 *      DACs/ADCs.
 *    o Platform power domain - can support external components i.e. amps and
 *      mic/meadphone insertion events.
 *    o Automatic Mic Bias support
 *    o Jack insertion power event initiation - e.g. hp insertion will enable
 *      sinks, dacs, etc
 *    o Delayed powerdown of audio susbsystem to reduce pops between a quick
 *      device reopen.
 *
 *  Todo:
 *    o DAPM power change sequencing - allow for configurable per
 *      codec sequences.
 *    o Support for analogue bias optimisation.
 *    o Support for reduced codec oversampling rates.
 *    o Support for reduced codec bias currents.
 */

#undef DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>

#include <trace/events/asoc.h>

#define PATH_MAX_HOPS 16

int soc_dsp_runtime_update(struct snd_soc_dapm_widget *);

/* dapm power sequences - make this per codec in the future */
static int dapm_up_seq[] = {
	[snd_soc_dapm_pre] = 0,
	[snd_soc_dapm_supply] = 1,
	[snd_soc_dapm_micbias] = 2,
	[snd_soc_dapm_aif_in] = 3,
	[snd_soc_dapm_aif_out] = 3,
	[snd_soc_dapm_mic] = 4,
	[snd_soc_dapm_mux] = 5,
	[snd_soc_dapm_virt_mux] = 5,
	[snd_soc_dapm_value_mux] = 5,
	[snd_soc_dapm_dac] = 6,
	[snd_soc_dapm_mixer] = 7,
	[snd_soc_dapm_mixer_named_ctl] = 7,
	[snd_soc_dapm_pga] = 8,
	[snd_soc_dapm_adc] = 9,
	[snd_soc_dapm_out_drv] = 10,
	[snd_soc_dapm_hp] = 10,
	[snd_soc_dapm_spk] = 10,
	[snd_soc_dapm_post] = 11,
};

static int dapm_down_seq[] = {
	[snd_soc_dapm_pre] = 0,
	[snd_soc_dapm_adc] = 1,
	[snd_soc_dapm_hp] = 2,
	[snd_soc_dapm_spk] = 2,
	[snd_soc_dapm_out_drv] = 2,
	[snd_soc_dapm_pga] = 4,
	[snd_soc_dapm_mixer_named_ctl] = 5,
	[snd_soc_dapm_mixer] = 5,
	[snd_soc_dapm_dac] = 6,
	[snd_soc_dapm_mic] = 7,
	[snd_soc_dapm_micbias] = 8,
	[snd_soc_dapm_mux] = 9,
	[snd_soc_dapm_virt_mux] = 9,
	[snd_soc_dapm_value_mux] = 9,
	[snd_soc_dapm_aif_in] = 10,
	[snd_soc_dapm_aif_out] = 10,
	[snd_soc_dapm_supply] = 11,
	[snd_soc_dapm_post] = 12,
};

static void pop_wait(u32 pop_time)
{
	if (pop_time)
		schedule_timeout_uninterruptible(msecs_to_jiffies(pop_time));
}

static void pop_dbg(struct device *dev, u32 pop_time, const char *fmt, ...)
{
	va_list args;
	char *buf;

	if (!pop_time)
		return;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL)
		return;

	va_start(args, fmt);
	vsnprintf(buf, PAGE_SIZE, fmt, args);
	dev_info(dev, "%s", buf);
	va_end(args);

	kfree(buf);
}

/* create a new dapm widget */
static inline struct snd_soc_dapm_widget *dapm_cnew_widget(
	const struct snd_soc_dapm_widget *_widget)
{
	return kmemdup(_widget, sizeof(*_widget), GFP_KERNEL);
}

static inline struct snd_card *dapm_get_card(struct snd_soc_dapm_context *dapm)
{
	if (dapm->codec)
		return dapm->codec->card->snd_card;
	else if (dapm->platform)
		return dapm->platform->card->snd_card;
	else
		BUG();
}

static inline struct snd_soc_card *dapm_get_soc_card(
		struct snd_soc_dapm_context *dapm)
{
	if (dapm->codec)
		return dapm->codec->card;
	else if (dapm->platform)
		return dapm->platform->card;
	else
		BUG();
}

static int soc_widget_read(struct snd_soc_dapm_widget *w, int reg)
{
	if (w->codec)
		return snd_soc_read(w->codec, reg);
	else if  (w->platform)
		return snd_soc_platform_read(w->platform, reg);
	return 0;
}

static int soc_widget_write(struct snd_soc_dapm_widget *w,int reg, int val)
{
	if (w->codec)
		return snd_soc_write(w->codec, reg, val);
	else if  (w->platform)
		return snd_soc_platform_write(w->platform, reg, val);
	return 0;
}

int soc_widget_update_bits(struct snd_soc_dapm_widget *w, unsigned short reg,
				unsigned int mask, unsigned int value)
{
	int change;
	unsigned int old, new;

	old = soc_widget_read(w, reg);
	new = (old & ~mask) | value;
	change = old != new;
	if (change)
		soc_widget_write(w, reg, new);

	return change;
}

int soc_widget_test_bits(struct snd_soc_dapm_widget *w, unsigned short reg,
				unsigned int mask, unsigned int value)
{
	int change;
	unsigned int old, new;

	old = soc_widget_read(w, reg);
	new = (old & ~mask) | value;
	change = old != new;

	return change;
}

/* reset 'walked' bit for each dapm path */
static inline void dapm_clear_walk(struct snd_soc_dapm_context *dapm)
{
	struct snd_soc_dapm_path *p;

	list_for_each_entry(p, &dapm->card->paths, list)
		p->walked = 0;
}

static void dapm_clear_paths(struct snd_soc_dapm_context *dapm)
{
	struct snd_soc_dapm_path *p;
	struct snd_soc_dapm_widget *w;
	struct list_head *l;

	list_for_each(l, &dapm->card->paths) {
		p = list_entry(l, struct snd_soc_dapm_path, list);
		p->length = 0;
	}
	list_for_each(l, &dapm->card->widgets) {
		w = list_entry(l, struct snd_soc_dapm_widget, list);
		w->hops = 0;
	}
	dapm_clear_walk(dapm);
}

static int dapm_add_unique_widget(struct snd_soc_dapm_context *dapm,
	struct snd_soc_dapm_widget_list **list, struct snd_soc_dapm_widget *w)
{
	struct snd_soc_dapm_widget_list *wlist;
	int wlistsize, wlistentries, i;

	/* is the list empty ? */
	if (*list == NULL) {

		wlistsize = sizeof(struct snd_soc_dapm_widget_list) +
				sizeof(struct snd_soc_dapm_widget *);
		*list = kzalloc(wlistsize, GFP_KERNEL);
		if (*list == NULL) {
			dev_err(dapm->dev, "can't allocate widget list for %s\n", w->name);
			return -ENOMEM;
		}
	} else {

		wlist = *list;
		/* is this widget already in the list */
		for (i = 0; i < wlist->num_widgets; i++) {
			if (wlist->widgets[i] == w)
				return 0;
		}

		wlistentries = wlist->num_widgets + 1;
		wlistsize = sizeof(struct snd_soc_dapm_widget_list) +
				wlistentries * sizeof(struct snd_soc_dapm_widget *);
		*list = krealloc(wlist, wlistsize, GFP_KERNEL);
		if (*list == NULL) {
			dev_err(dapm->dev, "can't allocate widget list for %s\n", w->name);
			return -ENOMEM;
		}
	}
	wlist = *list;

	/* insert the widget */
	dev_dbg(dapm->dev, "added %s in widget list pos %d\n",
			w->name, wlist->num_widgets);
	wlist->widgets[wlist->num_widgets] = w;
	wlist->num_widgets++;
	return 1;
}

static int is_output_widget_ep(struct snd_soc_dapm_widget *widget)
{
	switch (widget->id) {
	case snd_soc_dapm_adc:
	case snd_soc_dapm_aif_out:
		return 1;
	case snd_soc_dapm_output:
		if (widget->connected && !widget->ext)
			return 1;
		else
			return 0;
	case snd_soc_dapm_hp:
	case snd_soc_dapm_spk:
	case snd_soc_dapm_line:
		return !list_empty(&widget->sources);
	default:
		return 0;
	}
}

static int is_input_widget_ep(struct snd_soc_dapm_widget *widget)
{
	switch (widget->id) {
	case snd_soc_dapm_dac:
	case snd_soc_dapm_aif_in:
		return 1;
	case snd_soc_dapm_input:
		if (widget->connected && !widget->ext)
			return 1;
		else
			return 0;
	case snd_soc_dapm_mic:
		return !list_empty(&widget->sources);
	default:
		return 0;
	}
}

/*
 * find all the paths between source and sink
 */
static int dapm_find_playback_paths(struct snd_soc_dapm_context *dapm,
		struct snd_soc_dapm_widget *root,
		struct snd_soc_dapm_widget_list **list, int hops)
{
	struct list_head *lp;
	struct snd_soc_dapm_path *path;
	int dist = 0;

	if (hops > PATH_MAX_HOPS)
		return 0;

	if (is_output_widget_ep(root) && hops != 1) {
		dev_dbg(dapm->dev," ! %d: valid playback route found\n", hops);
		dapm->num_valid_paths++;
		return 1;
	}

	if (root->hops && root->hops <= hops)
		return 0;
	root->hops = hops;

	/* check all the output paths on this source widget by walking
	 * from source to sink */
	list_for_each(lp, &root->sinks) {
		path = list_entry(lp, struct snd_soc_dapm_path, list_source);

		dev_dbg(dapm->dev," %c %d: %s -> %s -> %s\n",
				path->connect ? '*' : ' ', hops,
				root->name, path->name, path->sink->name);

		/* been here before ? */
		if (path->length && path->length <= hops)
			continue;

		/* check down the next path if connected */
		if (path->sink && path->connect &&
				dapm_find_playback_paths(dapm, path->sink, list, hops + 1)) {
			path->length = hops;

			/* add widget to list */
			dapm_add_unique_widget(dapm, list, path->sink);

			if (!dist || dist > path->length)
				dist = path->length;
		}
	}

	return dist;
}

static int dapm_find_capture_paths(struct snd_soc_dapm_context *dapm,
		struct snd_soc_dapm_widget *root,
		struct snd_soc_dapm_widget_list **list, int hops)
{
	struct list_head *lp;
	struct snd_soc_dapm_path *path;
	int dist = 0;

	if (hops > PATH_MAX_HOPS)
		return 0;

	if (is_input_widget_ep(root) && hops != 1) {
		dev_dbg(dapm->dev," ! %d: valid capture route found\n", hops);
		dapm->num_valid_paths++;
		return 1;
	}

	if (root->hops && root->hops <= hops)
		return 0;
	root->hops = hops;

	/* check all the output paths on this source widget by walking from
	 * sink to source */
	list_for_each(lp, &root->sources) {
		path = list_entry(lp, struct snd_soc_dapm_path, list_sink);

		dev_dbg(dapm->dev," %c %d: %s <- %s <- %s\n",
				path->connect ? '*' : ' ', hops,
				root->name, path->name, path->source->name);

		/* been here before ? */
		if (path->length && path->length <= hops)
			continue;

		/* check down the next path if connected */
		if (path->source && path->connect &&
				dapm_find_capture_paths(dapm, path->source, list, hops + 1)) {
			path->length = hops;

			/* add widget to list */
			dapm_add_unique_widget(dapm, list, path->source);

			if (!dist || dist > path->length)
				dist = path->length;
		}
	}

	return dist;
}

/*
 * traverse the tree from sink to source via the shortest path
 */
static int dapm_get_playback_paths(struct snd_soc_dapm_context *dapm,
		struct snd_soc_dapm_widget *root,
		struct snd_soc_dapm_widget_list **list)
{
	dev_dbg(dapm->dev, "Playback: checking paths from %s\n",root->name);
	dapm_find_playback_paths(dapm, root, list, 1);
	return dapm->num_valid_paths;
}

static int dapm_get_capture_paths(struct snd_soc_dapm_context *dapm,
		struct snd_soc_dapm_widget *root,
		struct snd_soc_dapm_widget_list **list)
{
	dev_dbg(dapm->dev, "Capture: checking paths to %s\n", root->name);
	dapm_find_capture_paths(dapm, root, list, 1);
	return dapm->num_valid_paths;
}

/**
 * snd_soc_dapm_get_connected_widgets_type - query audio path and it's widgets.
 * @dapm: the dapm context.
 * @stream_name: stream name.
 * @list: list of active widgets for this stream.
 * @stream: stream direction.
 * @type: Initial widget type.
 *
 * Queries DAPM graph as to whether an valid audio stream path exists for
 * the DAPM stream and initial widget type specified. This takes into account
 * current mixer and mux kcontrol settings. Creates list of valid widgets.
 *
 * Returns the number of valid paths or negative error.
 */
int snd_soc_dapm_get_connected_widgets_type(struct snd_soc_dapm_context *dapm,
		const char *stream_name, struct snd_soc_dapm_widget_list **list,
		int stream, enum snd_soc_dapm_type type)
{
	struct snd_soc_dapm_widget *w;
	int paths;

	/* get stream root widget AIF, DAC or ADC from stream string and direction */
	list_for_each_entry(w, &dapm->card->widgets, list) {

		if (!w->sname)
			continue;

		if (w->id != type)
			continue;

		if (strstr(w->sname, stream_name))
			goto found;
	}
	dev_err(dapm->dev, "root widget not found\n");
	return 0;

found:
	dapm->num_valid_paths = 0;
	*list = NULL;
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		paths = dapm_get_playback_paths(dapm, w, list);
	else
		paths = dapm_get_capture_paths(dapm, w, list);

	dapm_clear_paths(dapm);
	return paths;
}
/**
 * snd_soc_dapm_get_connected_widgets_name - query audio path and it's widgets.
 * @dapm: the dapm context.
 * @name: initial widget name.
 * @list: list of active widgets for this stream.
 * @stream: stream direction.
 *
 * Queries DAPM graph as to whether an valid audio stream path exists for
 * the initial widget specified by name. This takes into account
 * current mixer and mux kcontrol settings. Creates list of valid widgets.
 *
 * Returns the number of valid paths or negative error.
 */
int snd_soc_dapm_get_connected_widgets_name(struct snd_soc_dapm_context *dapm,
		const char *name, struct snd_soc_dapm_widget_list **list, int stream)
{
	struct snd_soc_dapm_widget *w;
	int paths;

	/* get stream root widget AIF, DAC or ADC from stream string and direction */
	list_for_each_entry(w, &dapm->card->widgets, list) {

		if (strstr(w->name, name))
			goto found;
	}
	dev_err(dapm->dev, "root widget %s not found\n", name);
	return 0;

found:
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		paths = dapm_get_playback_paths(dapm, w, list);
	else
		paths = dapm_get_capture_paths(dapm, w, list);

	dapm_clear_paths(dapm);
	return paths;
}

/**
 * snd_soc_dapm_set_bias_level - set the bias level for the system
 * @dapm: DAPM context
 * @level: level to configure
 *
 * Configure the bias (power) levels for the SoC audio device.
 *
 * Returns 0 for success else error.
 */
static int snd_soc_dapm_set_bias_level(struct snd_soc_dapm_context *dapm,
				       enum snd_soc_bias_level level)
{
	struct snd_soc_card *card = dapm->card;
	int ret = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
		dev_dbg(dapm->dev, "Setting full bias\n");
		break;
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(dapm->dev, "Setting bias prepare\n");
		break;
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(dapm->dev, "Setting standby bias\n");
		break;
	case SND_SOC_BIAS_OFF:
		dev_dbg(dapm->dev, "Setting bias off\n");
		break;
	default:
		dev_err(dapm->dev, "Setting invalid bias %d\n", level);
		return -EINVAL;
	}

	trace_snd_soc_bias_level_start(card, level);

	if (card && card->set_bias_level)
		ret = card->set_bias_level(card, level);
	if (ret == 0) {
		if (dapm->codec && dapm->codec->driver->set_bias_level)
			ret = dapm->codec->driver->set_bias_level(dapm->codec, level);
		else
			dapm->bias_level = level;
	}
	if (ret == 0) {
		if (card && card->set_bias_level_post)
			ret = card->set_bias_level_post(card, level);
	}

	trace_snd_soc_bias_level_done(card, level);

	return ret;
}

/* set up initial codec paths */
static void dapm_set_path_status(struct snd_soc_dapm_widget *w,
	struct snd_soc_dapm_path *p, int i)
{
	switch (w->id) {
	case snd_soc_dapm_switch:
	case snd_soc_dapm_mixer:
	case snd_soc_dapm_mixer_named_ctl: {
		int val;
		struct soc_mixer_control *mc = (struct soc_mixer_control *)
			w->kcontrol_news[i].private_value;
		unsigned int reg = mc->reg;
		unsigned int shift = mc->shift;
		int max = mc->max;
		unsigned int mask = (1 << fls(max)) - 1;
		unsigned int invert = mc->invert;

		val = soc_widget_read(w, reg);
		val = (val >> shift) & mask;

		if ((invert && !val) || (!invert && val))
			p->connect = 1;
		else
			p->connect = 0;
	}
	break;
	case snd_soc_dapm_mux: {
		struct soc_enum *e = (struct soc_enum *)
			w->kcontrol_news[i].private_value;
		int val, item, bitmask;

		for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
		val = soc_widget_read(w, e->reg);
		item = (val >> e->shift_l) & (bitmask - 1);

		p->connect = 0;
		for (i = 0; i < e->max; i++) {
			if (!(strcmp(p->name, snd_soc_get_enum_text(e, i))) && item == i)
				p->connect = 1;
		}
	}
	break;
	case snd_soc_dapm_virt_mux: {
		struct soc_enum *e = (struct soc_enum *)
			w->kcontrol_news[i].private_value;

		p->connect = 0;
		/* since a virtual mux has no backing registers to
		 * decide which path to connect, it will try to match
		 * with the first enumeration.  This is to ensure
		 * that the default mux choice (the first) will be
		 * correctly powered up during initialization.
		 */
		if (!strcmp(p->name, snd_soc_get_enum_text(e, 0)))
			p->connect = 1;
	}
	break;
	case snd_soc_dapm_value_mux: {
		struct soc_enum *e = (struct soc_enum *)
			w->kcontrol_news[i].private_value;
		int val, item;

		val = soc_widget_read(w, e->reg);
		val = (val >> e->shift_l) & e->mask;
		for (item = 0; item < e->max; item++) {
			if (val == e->values[item])
				break;
		}

		p->connect = 0;
		for (i = 0; i < e->max; i++) {
			if (!(strcmp(p->name, snd_soc_get_enum_text(e, i))) && item == i)
				p->connect = 1;
		}
	}
	break;
	/* does not effect routing - always connected */
	case snd_soc_dapm_pga:
	case snd_soc_dapm_out_drv:
	case snd_soc_dapm_output:
	case snd_soc_dapm_adc:
	case snd_soc_dapm_input:
	case snd_soc_dapm_dac:
	case snd_soc_dapm_micbias:
	case snd_soc_dapm_vmid:
	case snd_soc_dapm_supply:
	case snd_soc_dapm_aif_in:
	case snd_soc_dapm_aif_out:
		p->connect = 1;
	break;
	/* does effect routing - dynamically connected */
	case snd_soc_dapm_hp:
	case snd_soc_dapm_mic:
	case snd_soc_dapm_spk:
	case snd_soc_dapm_line:
	case snd_soc_dapm_pre:
	case snd_soc_dapm_post:
		p->connect = 0;
	break;
	}
}

/* connect mux widget to its interconnecting audio paths */
static int dapm_connect_mux(struct snd_soc_dapm_context *dapm,
	struct snd_soc_dapm_widget *src, struct snd_soc_dapm_widget *dest,
	struct snd_soc_dapm_path *path, const char *control_name,
	const struct snd_kcontrol_new *kcontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	int i;

	for (i = 0; i < e->max; i++) {
		if (!(strcmp(control_name, snd_soc_get_enum_text(e, i)))) {
			list_add(&path->list, &dapm->card->paths);
			list_add(&path->list_sink, &dest->sources);
			list_add(&path->list_source, &src->sinks);
			path->name = (char*)snd_soc_get_enum_text(e, i);
			dapm_set_path_status(dest, path, 0);
			return 0;
		}
	}

	return -ENODEV;
}

/* connect mixer widget to its interconnecting audio paths */
static int dapm_connect_mixer(struct snd_soc_dapm_context *dapm,
	struct snd_soc_dapm_widget *src, struct snd_soc_dapm_widget *dest,
	struct snd_soc_dapm_path *path, const char *control_name)
{
	int i;

	/* search for mixer kcontrol */
	for (i = 0; i < dest->num_kcontrols; i++) {
		if (!strcmp(control_name, dest->kcontrol_news[i].name)) {
			list_add(&path->list, &dapm->card->paths);
			list_add(&path->list_sink, &dest->sources);
			list_add(&path->list_source, &src->sinks);
			path->name = dest->kcontrol_news[i].name;
			dapm_set_path_status(dest, path, i);
			return 0;
		}
	}
	return -ENODEV;
}

static int dapm_is_shared_kcontrol(struct snd_soc_dapm_context *dapm,
	struct snd_soc_dapm_widget *kcontrolw,
	const struct snd_kcontrol_new *kcontrol_new,
	struct snd_kcontrol **kcontrol)
{
	struct snd_soc_dapm_widget *w;
	int i;

	*kcontrol = NULL;

	list_for_each_entry(w, &dapm->card->widgets, list) {
		if (w == kcontrolw || w->dapm != kcontrolw->dapm)
			continue;
		for (i = 0; i < w->num_kcontrols; i++) {
			if (&w->kcontrol_news[i] == kcontrol_new) {
				if (w->kcontrols)
					*kcontrol = w->kcontrols[i];
				return 1;
			}
		}
	}

	return 0;
}

/* create new dapm mixer control */
static int dapm_new_mixer(struct snd_soc_dapm_widget *w)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	int i, ret = 0;
	size_t name_len, prefix_len;
	struct snd_soc_dapm_path *path;
	struct snd_card *card = dapm->card->snd_card;
	const char *prefix;
	struct snd_soc_dapm_widget_list *wlist;
	size_t wlistsize;

	if (dapm->codec)
		prefix = dapm->codec->name_prefix;
	else
		prefix = NULL;

	if (prefix)
		prefix_len = strlen(prefix) + 1;
	else
		prefix_len = 0;

	/* add kcontrol */
	for (i = 0; i < w->num_kcontrols; i++) {

		/* match name */
		list_for_each_entry(path, &w->sources, list_sink) {

			/* mixer/mux paths name must match control name */
			if (path->name != (char *)w->kcontrol_news[i].name)
				continue;

			wlistsize = sizeof(struct snd_soc_dapm_widget_list) +
				    sizeof(struct snd_soc_dapm_widget *),
			wlist = kzalloc(wlistsize, GFP_KERNEL);
			if (wlist == NULL) {
				dev_err(dapm->dev,
					"asoc: can't allocate widget list for %s\n",
					w->name);
				return -ENOMEM;
			}
			wlist->num_widgets = 1;
			wlist->widgets[0] = w;

			/* add dapm control with long name.
			 * for dapm_mixer this is the concatenation of the
			 * mixer and kcontrol name.
			 * for dapm_mixer_named_ctl this is simply the
			 * kcontrol name.
			 */
			name_len = strlen(w->kcontrol_news[i].name) + 1;
			if (w->id != snd_soc_dapm_mixer_named_ctl)
				name_len += 1 + strlen(w->name);

			path->long_name = kmalloc(name_len, GFP_KERNEL);

			if (path->long_name == NULL) {
				kfree(wlist);
				return -ENOMEM;
			}

			switch (w->id) {
			default:
				/* The control will get a prefix from
				 * the control creation process but
				 * we're also using the same prefix
				 * for widgets so cut the prefix off
				 * the front of the widget name.
				 */
				snprintf(path->long_name, name_len, "%s %s",
					 w->name + prefix_len,
					 w->kcontrol_news[i].name);
				break;
			case snd_soc_dapm_mixer_named_ctl:
				snprintf(path->long_name, name_len, "%s",
					 w->kcontrol_news[i].name);
				break;
			}

			path->long_name[name_len - 1] = '\0';

			path->kcontrol = snd_soc_cnew(&w->kcontrol_news[i],
						      wlist, path->long_name,
						      prefix);
			ret = snd_ctl_add(card, path->kcontrol);
			if (ret < 0) {
				dev_err(dapm->dev,
					"asoc: failed to add dapm kcontrol %s: %d\n",
					path->long_name, ret);
				kfree(wlist);
				kfree(path->long_name);
				path->long_name = NULL;
				return ret;
			}
			w->kcontrols[i] = path->kcontrol;
		}
	}
	return ret;
}

/* create new dapm mux control */
static int dapm_new_mux(struct snd_soc_dapm_widget *w)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_dapm_path *path = NULL;
	struct snd_kcontrol *kcontrol;
	struct snd_card *card = dapm->card->snd_card;
	const char *prefix;
	size_t prefix_len;
	int ret;
	struct snd_soc_dapm_widget_list *wlist;
	int shared, wlistentries;
	size_t wlistsize;
	char *name;

	if (w->num_kcontrols != 1) {
		dev_err(dapm->dev,
			"asoc: mux %s has incorrect number of controls\n",
			w->name);
		return -EINVAL;
	}

	shared = dapm_is_shared_kcontrol(dapm, w, &w->kcontrol_news[0],
					 &kcontrol);
	if (kcontrol) {
		wlist = kcontrol->private_data;
		wlistentries = wlist->num_widgets + 1;
	} else {
		wlist = NULL;
		wlistentries = 1;
	}
	wlistsize = sizeof(struct snd_soc_dapm_widget_list) +
		wlistentries * sizeof(struct snd_soc_dapm_widget *),
	wlist = krealloc(wlist, wlistsize, GFP_KERNEL);
	if (wlist == NULL) {
		dev_err(dapm->dev,
			"asoc: can't allocate widget list for %s\n", w->name);
		return -ENOMEM;
	}
	wlist->num_widgets = wlistentries;
	wlist->widgets[wlistentries - 1] = w;

	if (!kcontrol) {
		if (dapm->codec && dapm->codec->name_prefix)
			prefix = dapm->codec->name_prefix;
		else
			prefix = NULL;

		if (shared) {
			name = w->kcontrol_news[0].name;
			prefix_len = 0;
		} else {
			name = w->name;
			if (prefix)
				prefix_len = strlen(prefix) + 1;
			else
				prefix_len = 0;
		}

		/*
		 * The control will get a prefix from the control creation
		 * process but we're also using the same prefix for widgets so
		 * cut the prefix off the front of the widget name.
		 */
		kcontrol = snd_soc_cnew(&w->kcontrol_news[0], wlist,
					name, prefix);
		ret = snd_ctl_add(card, kcontrol);
		if (ret < 0) {
			dev_err(dapm->dev,
				"asoc: failed to add kcontrol %s\n", w->name);
			kfree(wlist);
			return ret;
		}
	}

	kcontrol->private_data = wlist;

	w->kcontrols[0] = kcontrol;

	list_for_each_entry(path, &w->sources, list_sink)
		path->kcontrol = kcontrol;

	return 0;
}

/* create new dapm volume control */
static int dapm_new_pga(struct snd_soc_dapm_widget *w)
{
	if (w->num_kcontrols)
		dev_err(w->dapm->dev,
			"asoc: PGA controls not supported: '%s'\n", w->name);

	return 0;
}

/* We implement power down on suspend by checking the power state of
 * the ALSA card - when we are suspending the ALSA state for the card
 * is set to D3.
 */
static int snd_soc_dapm_suspend_check(struct snd_soc_dapm_widget *widget)
{
	int level = snd_power_get_state(widget->dapm->card->snd_card);

	switch (level) {
	case SNDRV_CTL_POWER_D3hot:
	case SNDRV_CTL_POWER_D3cold:
		if (widget->ignore_suspend)
			dev_dbg(widget->dapm->dev, "%s ignoring suspend\n",
				widget->name);
		return widget->ignore_suspend;
	default:
		return 1;
	}
}

/*
 * Recursively check for a completed path to an active or physically connected
 * output widget. Returns number of complete paths.
 */
static int is_connected_output_ep(struct snd_soc_dapm_widget *widget)
{
	struct snd_soc_dapm_path *path;
	int con = 0;

	if (widget->id == snd_soc_dapm_supply)
		return 0;

	switch (widget->id) {
	case snd_soc_dapm_adc:
	case snd_soc_dapm_aif_out:
		if (widget->active)
			return snd_soc_dapm_suspend_check(widget);
	default:
		break;
	}

	if (widget->connected) {
		/* connected pin ? */
		if (widget->id == snd_soc_dapm_output && !widget->ext)
			return snd_soc_dapm_suspend_check(widget);

		/* connected jack or spk ? */
		if (widget->id == snd_soc_dapm_hp || widget->id == snd_soc_dapm_spk ||
		    (widget->id == snd_soc_dapm_line && !list_empty(&widget->sources)))
			return snd_soc_dapm_suspend_check(widget);
	}

	list_for_each_entry(path, &widget->sinks, list_source) {
		if (path->walked)
			continue;

		if (path->sink && path->connect) {
			path->walked = 1;
			con += is_connected_output_ep(path->sink);
		}
	}

	return con;
}

/*
 * Recursively check for a completed path to an active or physically connected
 * input widget. Returns number of complete paths.
 */
static int is_connected_input_ep(struct snd_soc_dapm_widget *widget)
{
	struct snd_soc_dapm_path *path;
	int con = 0;

	if (widget->id == snd_soc_dapm_supply)
		return 0;

	/* active stream ? */
	switch (widget->id) {
	case snd_soc_dapm_dac:
	case snd_soc_dapm_aif_in:
		if (widget->active)
			return snd_soc_dapm_suspend_check(widget);
	default:
		break;
	}

	if (widget->connected) {
		/* connected pin ? */
		if (widget->id == snd_soc_dapm_input && !widget->ext)
			return snd_soc_dapm_suspend_check(widget);

		/* connected VMID/Bias for lower pops */
		if (widget->id == snd_soc_dapm_vmid)
			return snd_soc_dapm_suspend_check(widget);

		/* connected jack ? */
		if (widget->id == snd_soc_dapm_mic ||
		    (widget->id == snd_soc_dapm_line && !list_empty(&widget->sinks)))
			return snd_soc_dapm_suspend_check(widget);
	}

	list_for_each_entry(path, &widget->sources, list_sink) {
		if (path->walked)
			continue;

		if (path->source && path->connect) {
			path->walked = 1;
			con += is_connected_input_ep(path->source);
		}
	}

	return con;
}

/*
 * Handler for generic register modifier widget.
 */
int dapm_reg_event(struct snd_soc_dapm_widget *w,
		   struct snd_kcontrol *kcontrol, int event)
{
	unsigned int val;

	if (SND_SOC_DAPM_EVENT_ON(event))
		val = w->on_val;
	else
		val = w->off_val;

	soc_widget_update_bits(w, -(w->reg + 1),
			    w->mask << w->shift, val << w->shift);

	return 0;
}
EXPORT_SYMBOL_GPL(dapm_reg_event);

/* Generic check to see if a widget should be powered.
 */
static int dapm_generic_check_power(struct snd_soc_dapm_widget *w)
{
	int in, out;

	in = is_connected_input_ep(w);
	dapm_clear_walk(w->dapm);
	out = is_connected_output_ep(w);
	dapm_clear_walk(w->dapm);
	return out != 0 && in != 0;
}

/* Check to see if an ADC has power */
static int dapm_adc_check_power(struct snd_soc_dapm_widget *w)
{
	int in;

	if (w->active) {
		in = is_connected_input_ep(w);
		dapm_clear_walk(w->dapm);
		return in != 0;
	} else {
		return dapm_generic_check_power(w);
	}
}

/* Check to see if a DAC has power */
static int dapm_dac_check_power(struct snd_soc_dapm_widget *w)
{
	int out;

	if (w->active) {
		out = is_connected_output_ep(w);
		dapm_clear_walk(w->dapm);
		return out != 0;
	} else {
		return dapm_generic_check_power(w);
	}
}

/* Check to see if a power supply is needed */
static int dapm_supply_check_power(struct snd_soc_dapm_widget *w)
{
	struct snd_soc_dapm_path *path;
	int power = 0;

	/* Check if one of our outputs is connected */
	list_for_each_entry(path, &w->sinks, list_source) {
		if (path->connected &&
		    !path->connected(path->source, path->sink))
			continue;

		if (!path->sink)
			continue;

		if (path->sink->force) {
			power = 1;
			break;
		}

		if (path->sink->power_check &&
		    path->sink->power_check(path->sink)) {
			power = 1;
			break;
		}
	}

	dapm_clear_walk(w->dapm);

	return power;
}

static int dapm_seq_compare(struct snd_soc_dapm_widget *a,
			    struct snd_soc_dapm_widget *b,
			    bool power_up)
{
	int *sort;

	if (power_up)
		sort = dapm_up_seq;
	else
		sort = dapm_down_seq;

	if (sort[a->id] != sort[b->id])
		return sort[a->id] - sort[b->id];
	if (a->subseq != b->subseq) {
		if (power_up)
			return a->subseq - b->subseq;
		else
			return b->subseq - a->subseq;
	}
	if (a->reg != b->reg)
		return a->reg - b->reg;
	if (a->dapm != b->dapm)
		return (unsigned long)a->dapm - (unsigned long)b->dapm;

	return 0;
}

/* Insert a widget in order into a DAPM power sequence. */
static void dapm_seq_insert(struct snd_soc_dapm_widget *new_widget,
			    struct list_head *list,
			    bool power_up)
{
	struct snd_soc_dapm_widget *w;

	list_for_each_entry(w, list, power_list)
		if (dapm_seq_compare(new_widget, w, power_up) < 0) {
			list_add_tail(&new_widget->power_list, &w->power_list);
			return;
		}

	list_add_tail(&new_widget->power_list, list);
}

static void dapm_seq_check_event(struct snd_soc_dapm_context *dapm,
				 struct snd_soc_dapm_widget *w, int event)
{
	struct snd_soc_card *card = dapm->card;
	const char *ev_name;
	int power, ret;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ev_name = "PRE_PMU";
		power = 1;
		break;
	case SND_SOC_DAPM_POST_PMU:
		ev_name = "POST_PMU";
		power = 1;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ev_name = "PRE_PMD";
		power = 0;
		break;
	case SND_SOC_DAPM_POST_PMD:
		ev_name = "POST_PMD";
		power = 0;
		break;
	default:
		BUG();
		return;
	}

	if (w->power != power)
		return;

	if (w->event && (w->event_flags & event)) {
		pop_dbg(dapm->dev, card->pop_time, "pop test : %s %s\n",
			w->name, ev_name);
		trace_snd_soc_dapm_widget_event_start(w, event);
		ret = w->event(w, NULL, event);
		trace_snd_soc_dapm_widget_event_done(w, event);
		if (ret < 0)
			pr_err("%s: %s event failed: %d\n",
			       ev_name, w->name, ret);
	}
}

/* Apply the coalesced changes from a DAPM sequence */
static void dapm_seq_run_coalesced(struct snd_soc_dapm_context *dapm,
				   struct list_head *pending)
{
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_dapm_widget *w, *_w;
	int reg, power;
	unsigned int value = 0;
	unsigned int mask = 0;
	unsigned int cur_mask;

	_w = list_first_entry(pending, struct snd_soc_dapm_widget,
				power_list);
	reg = _w->reg;

	list_for_each_entry(w, pending, power_list) {
		cur_mask = 1 << w->shift;
		BUG_ON(reg != w->reg);

		if (w->invert)
			power = !w->power;
		else
			power = w->power;

		mask |= cur_mask;
		if (power)
			value |= cur_mask;

		pop_dbg(dapm->dev, card->pop_time,
			"pop test : Queue %s: reg=0x%x, 0x%x/0x%x\n",
			w->name, reg, value, mask);

		/* Check for events */
		dapm_seq_check_event(dapm, w, SND_SOC_DAPM_PRE_PMU);
		dapm_seq_check_event(dapm, w, SND_SOC_DAPM_PRE_PMD);
	}

	if (reg >= 0) {
		/* Any widget will do, they should all be updating the
		 * same register.
		 */
		w = list_first_entry(pending, struct snd_soc_dapm_widget,
				     power_list);

		pop_dbg(dapm->dev, card->pop_time,
			"pop test : Applying 0x%x/0x%x to %x in %dms\n",
			value, mask, reg, card->pop_time);
		pop_wait(card->pop_time);
		soc_widget_update_bits(_w, reg, mask, value);
	}

	list_for_each_entry(w, pending, power_list) {
		dapm_seq_check_event(dapm, w, SND_SOC_DAPM_POST_PMU);
		dapm_seq_check_event(dapm, w, SND_SOC_DAPM_POST_PMD);
	}
}

/* Apply a DAPM power sequence.
 *
 * We walk over a pre-sorted list of widgets to apply power to.  In
 * order to minimise the number of writes to the device required
 * multiple widgets will be updated in a single write where possible.
 * Currently anything that requires more than a single write is not
 * handled.
 */
static void dapm_seq_run(struct snd_soc_dapm_context *dapm,
			 struct list_head *list, int event, bool power_up)
{
	struct snd_soc_dapm_widget *w, *n;
	LIST_HEAD(pending);
	int cur_sort = -1;
	int cur_subseq = -1;
	int cur_reg = SND_SOC_NOPM;
	struct snd_soc_dapm_context *cur_dapm = NULL;
	int ret, i;
	int *sort;

	if (power_up)
		sort = dapm_up_seq;
	else
		sort = dapm_down_seq;

	list_for_each_entry_safe(w, n, list, power_list) {
		ret = 0;

		/* Do we need to apply any queued changes? */
		if (sort[w->id] != cur_sort || w->reg != cur_reg ||
		    w->dapm != cur_dapm || w->subseq != cur_subseq) {
			if (!list_empty(&pending))
				dapm_seq_run_coalesced(cur_dapm, &pending);

			if (cur_dapm && cur_dapm->seq_notifier) {
				for (i = 0; i < ARRAY_SIZE(dapm_up_seq); i++)
					if (sort[i] == cur_sort)
						cur_dapm->seq_notifier(cur_dapm,
								       i,
								       cur_subseq);
			}

			INIT_LIST_HEAD(&pending);
			cur_sort = -1;
			cur_subseq = -1;
			cur_reg = SND_SOC_NOPM;
			cur_dapm = NULL;
		}

		switch (w->id) {
		case snd_soc_dapm_pre:
			if (!w->event)
				list_for_each_entry_safe_continue(w, n, list,
								  power_list);

			if (event == SND_SOC_DAPM_STREAM_START)
				ret = w->event(w,
					       NULL, SND_SOC_DAPM_PRE_PMU);
			else if (event == SND_SOC_DAPM_STREAM_STOP)
				ret = w->event(w,
					       NULL, SND_SOC_DAPM_PRE_PMD);
			break;

		case snd_soc_dapm_post:
			if (!w->event)
				list_for_each_entry_safe_continue(w, n, list,
								  power_list);

			if (event == SND_SOC_DAPM_STREAM_START)
				ret = w->event(w,
					       NULL, SND_SOC_DAPM_POST_PMU);
			else if (event == SND_SOC_DAPM_STREAM_STOP)
				ret = w->event(w,
					       NULL, SND_SOC_DAPM_POST_PMD);
			break;

		default:
			/* Queue it up for application */
			cur_sort = sort[w->id];
			cur_subseq = w->subseq;
			cur_reg = w->reg;
			cur_dapm = w->dapm;
			list_move(&w->power_list, &pending);
			break;
		}

		if (ret < 0)
			dev_err(w->dapm->dev,
				"Failed to apply widget power: %d\n", ret);
	}

	if (!list_empty(&pending))
		dapm_seq_run_coalesced(cur_dapm, &pending);

	if (cur_dapm && cur_dapm->seq_notifier) {
		for (i = 0; i < ARRAY_SIZE(dapm_up_seq); i++)
			if (sort[i] == cur_sort)
				cur_dapm->seq_notifier(cur_dapm,
						       i, cur_subseq);
	}
}

static void dapm_widget_update(struct snd_soc_dapm_context *dapm)
{
	struct snd_soc_dapm_update *update = dapm->update;
	struct snd_soc_dapm_widget *w;
	int ret;

	if (!update)
		return;

	w = update->widget;

	if (w->event &&
	    (w->event_flags & SND_SOC_DAPM_PRE_REG)) {
		ret = w->event(w, update->kcontrol, SND_SOC_DAPM_PRE_REG);
		if (ret != 0)
			pr_err("%s DAPM pre-event failed: %d\n",
			       w->name, ret);
	}

	ret = snd_soc_update_bits(w->codec, update->reg, update->mask,
				  update->val);
	if (ret < 0)
		pr_err("%s DAPM update failed: %d\n", w->name, ret);

	if (w->event &&
	    (w->event_flags & SND_SOC_DAPM_POST_REG)) {
		ret = w->event(w, update->kcontrol, SND_SOC_DAPM_POST_REG);
		if (ret != 0)
			pr_err("%s DAPM post-event failed: %d\n",
			       w->name, ret);
	}
}

/* Async callback run prior to DAPM sequences - brings to _PREPARE if
 * they're changing state.
 */
static void dapm_pre_sequence_async(void *data, async_cookie_t cookie)
{
	struct snd_soc_dapm_context *d = data;
	int ret;

	if (d->dev_power && d->bias_level == SND_SOC_BIAS_OFF) {
		ret = snd_soc_dapm_set_bias_level(d, SND_SOC_BIAS_STANDBY);
		if (ret != 0)
			dev_err(d->dev,
				"Failed to turn on bias: %d\n", ret);
	}

	/* If we're changing to all on or all off then prepare */
	if ((d->dev_power && d->bias_level == SND_SOC_BIAS_STANDBY) ||
	    (!d->dev_power && d->bias_level == SND_SOC_BIAS_ON)) {
		ret = snd_soc_dapm_set_bias_level(d, SND_SOC_BIAS_PREPARE);
		if (ret != 0)
			dev_err(d->dev,
				"Failed to prepare bias: %d\n", ret);
	}
}

/* Async callback run prior to DAPM sequences - brings to their final
 * state.
 */
static void dapm_post_sequence_async(void *data, async_cookie_t cookie)
{
	struct snd_soc_dapm_context *d = data;
	int ret;

	/* If we just powered the last thing off drop to standby bias */
	if (d->bias_level == SND_SOC_BIAS_PREPARE && !d->dev_power) {
		ret = snd_soc_dapm_set_bias_level(d, SND_SOC_BIAS_STANDBY);
		if (ret != 0)
			dev_err(d->dev, "Failed to apply standby bias: %d\n",
				ret);
	}

	/* If we're in standby and can support bias off then do that */
	if (d->bias_level == SND_SOC_BIAS_STANDBY && d->idle_bias_off) {
		ret = snd_soc_dapm_set_bias_level(d, SND_SOC_BIAS_OFF);
		if (ret != 0)
			dev_err(d->dev, "Failed to turn off bias: %d\n", ret);
	}

	/* If we just powered up then move to active bias */
	if (d->bias_level == SND_SOC_BIAS_PREPARE && d->dev_power) {
		ret = snd_soc_dapm_set_bias_level(d, SND_SOC_BIAS_ON);
		if (ret != 0)
			dev_err(d->dev, "Failed to apply active bias: %d\n",
				ret);
	}
}

/*
 * Scan each dapm widget for complete audio path.
 * A complete path is a route that has valid endpoints i.e.:-
 *
 *  o DAC to output pin.
 *  o Input Pin to ADC.
 *  o Input pin to Output pin (bypass, sidetone)
 *  o DAC to ADC (loopback).
 */
static int dapm_power_widgets(struct snd_soc_dapm_context *dapm, int event)
{
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_dapm_widget *w;
	struct snd_soc_dapm_context *d;
	LIST_HEAD(up_list);
	LIST_HEAD(down_list);
	LIST_HEAD(async_domain);
	int power;

	trace_snd_soc_dapm_start(card);

	list_for_each_entry(d, &card->dapm_list, list)
		if (d->n_widgets || d->codec == NULL)
			d->dev_power = 0;

	/* Check which widgets we need to power and store them in
	 * lists indicating if they should be powered up or down.
	 */
	list_for_each_entry(w, &card->widgets, list) {
		switch (w->id) {
		case snd_soc_dapm_pre:
			dapm_seq_insert(w, &down_list, false);
			break;
		case snd_soc_dapm_post:
			dapm_seq_insert(w, &up_list, true);
			break;

		default:
			if (!w->power_check)
				continue;

			if (!w->force)
				power = w->power_check(w);
			else
				power = 1;
			if (power)
				w->dapm->dev_power = 1;

			if (w->power == power)
				continue;

			trace_snd_soc_dapm_widget_power(w, power);

			if (power)
				dapm_seq_insert(w, &up_list, true);
			else
				dapm_seq_insert(w, &down_list, false);

			w->power = power;
			break;
		}
	}

	/* If there are no DAPM widgets then try to figure out power from the
	 * event type.
	 */
	if (!dapm->n_widgets) {
		switch (event) {
		case SND_SOC_DAPM_STREAM_START:
		case SND_SOC_DAPM_STREAM_RESUME:
			dapm->dev_power = 1;
			break;
		case SND_SOC_DAPM_STREAM_STOP:
			if (dapm->codec)
				dapm->dev_power = !!dapm->codec->active;
			else
				dapm->dev_power = 0;
			break;
		case SND_SOC_DAPM_STREAM_SUSPEND:
			dapm->dev_power = 0;
			break;
		case SND_SOC_DAPM_STREAM_NOP:
			switch (dapm->bias_level) {
				case SND_SOC_BIAS_STANDBY:
				case SND_SOC_BIAS_OFF:
					dapm->dev_power = 0;
					break;
				default:
					dapm->dev_power = 1;
					break;
			}
			break;
		default:
			break;
		}
	}

	/* Force all contexts in the card to the same bias state */
	power = 0;
	list_for_each_entry(d, &card->dapm_list, list)
		if (d->dev_power)
			power = 1;
	list_for_each_entry(d, &card->dapm_list, list)
		d->dev_power = power;


	/* Run all the bias changes in parallel */
	list_for_each_entry(d, &dapm->card->dapm_list, list)
		async_schedule_domain(dapm_pre_sequence_async, d,
					&async_domain);
	async_synchronize_full_domain(&async_domain);

	/* Power down widgets first; try to avoid amplifying pops. */
	dapm_seq_run(dapm, &down_list, event, false);

	dapm_widget_update(dapm);

	/* Now power up. */
	dapm_seq_run(dapm, &up_list, event, true);

	/* Run all the bias changes in parallel */
	list_for_each_entry(d, &dapm->card->dapm_list, list)
		async_schedule_domain(dapm_post_sequence_async, d,
					&async_domain);
	async_synchronize_full_domain(&async_domain);

	pop_dbg(dapm->dev, card->pop_time,
		"DAPM sequencing finished, waiting %dms\n", card->pop_time);
	pop_wait(card->pop_time);

	trace_snd_soc_dapm_done(card);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int dapm_widget_power_open_file(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dapm_widget_power_read_file(struct file *file,
					   char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct snd_soc_dapm_widget *w = file->private_data;
	char *buf;
	int in, out;
	ssize_t ret;
	struct snd_soc_dapm_path *p = NULL;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	in = is_connected_input_ep(w);
	dapm_clear_walk(w->dapm);
	out = is_connected_output_ep(w);
	dapm_clear_walk(w->dapm);

	ret = snprintf(buf, PAGE_SIZE, "%s: %s  in %d out %d",
		       w->name, w->power ? "On" : "Off", in, out);

	if (w->reg >= 0)
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				" - R%d(0x%x) bit %d",
				w->reg, w->reg, w->shift);

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");

	if (w->sname)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, " stream %s %s\n",
				w->sname,
				w->active ? "active" : "inactive");

	list_for_each_entry(p, &w->sources, list_sink) {
		if (p->connected && !p->connected(w, p->sink))
			continue;

		if (p->connect)
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					" in  \"%s\" \"%s\"\n",
					p->name ? p->name : "static",
					p->source->name);
	}
	list_for_each_entry(p, &w->sinks, list_source) {
		if (p->connected && !p->connected(w, p->sink))
			continue;

		if (p->connect)
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					" out \"%s\" \"%s\"\n",
					p->name ? p->name : "static",
					p->sink->name);
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);

	kfree(buf);
	return ret;
}

static const struct file_operations dapm_widget_power_fops = {
	.open = dapm_widget_power_open_file,
	.read = dapm_widget_power_read_file,
	.llseek = default_llseek,
};

static int dapm_bias_open_file(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dapm_bias_read_file(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct snd_soc_dapm_context *dapm = file->private_data;
	char *level;

	switch (dapm->bias_level) {
	case SND_SOC_BIAS_ON:
		level = "On\n";
		break;
	case SND_SOC_BIAS_PREPARE:
		level = "Prepare\n";
		break;
	case SND_SOC_BIAS_STANDBY:
		level = "Standby\n";
		break;
	case SND_SOC_BIAS_OFF:
		level = "Off\n";
		break;
	default:
		BUG();
		level = "Unknown\n";
		break;
	}

	return simple_read_from_buffer(user_buf, count, ppos, level,
				       strlen(level));
}

static const struct file_operations dapm_bias_fops = {
	.open = dapm_bias_open_file,
	.read = dapm_bias_read_file,
	.llseek = default_llseek,
};

void snd_soc_dapm_debugfs_init(struct snd_soc_dapm_context *dapm,
	struct dentry *parent)
{
	struct dentry *d;

	dapm->debugfs_dapm = debugfs_create_dir("dapm", parent);

	if (!dapm->debugfs_dapm) {
		printk(KERN_WARNING
		       "Failed to create DAPM debugfs directory\n");
		return;
	}

	d = debugfs_create_file("bias_level", 0444,
				dapm->debugfs_dapm, dapm,
				&dapm_bias_fops);
	if (!d)
		dev_warn(dapm->dev,
			 "ASoC: Failed to create bias level debugfs file\n");
}

static void dapm_debugfs_add_widget(struct snd_soc_dapm_widget *w)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct dentry *d;

	if (!dapm->debugfs_dapm || !w->name)
		return;

	d = debugfs_create_file(w->name, 0444,
				dapm->debugfs_dapm, w,
				&dapm_widget_power_fops);
	if (!d)
		dev_warn(w->dapm->dev,
			"ASoC: Failed to create %s debugfs file\n",
			w->name);
}

static void dapm_debugfs_cleanup(struct snd_soc_dapm_context *dapm)
{
	debugfs_remove_recursive(dapm->debugfs_dapm);
}

#else
void snd_soc_dapm_debugfs_init(struct snd_soc_dapm_context *dapm,
	struct dentry *parent)
{
}

static inline void dapm_debugfs_add_widget(struct snd_soc_dapm_widget *w)
{
}

static inline void dapm_debugfs_cleanup(struct snd_soc_dapm_context *dapm)
{
}

#endif

/* test and update the power status of a mux widget */
int snd_soc_dapm_mux_update_power(struct snd_soc_dapm_widget *widget,
				 struct snd_kcontrol *kcontrol, int change,
				 int mux, struct soc_enum *e)
{
	struct snd_soc_dapm_path *path;
	int found = 0;

	if (widget->id != snd_soc_dapm_mux &&
	    widget->id != snd_soc_dapm_virt_mux &&
	    widget->id != snd_soc_dapm_value_mux)
		return -ENODEV;

	if (!change)
		return 0;

	/* find dapm widget path assoc with kcontrol */
	list_for_each_entry(path, &widget->dapm->card->paths, list) {

		if (path->kcontrol != kcontrol)
			continue;

		if (!path->name || !snd_soc_get_enum_text(e, mux))
			continue;

		found = 1;
		/* we now need to match the string in the enum to the path */
		if (!(strcmp(path->name, snd_soc_get_enum_text(e, mux))))
			path->connect = 1; /* new connection */
		else
			path->connect = 0; /* old connection must be powered down */
	}

	if (found) {
		if (widget->platform)
			soc_dsp_runtime_update(widget);
		else
			dapm_power_widgets(widget->dapm,
				SND_SOC_DAPM_STREAM_NOP);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_mux_update_power);

/* test and update the power status of a mixer or switch widget */
int snd_soc_dapm_mixer_update_power(struct snd_soc_dapm_widget *widget,
		struct snd_kcontrol *kcontrol, int connect)
{
	struct snd_soc_dapm_path *path;
	int found = 0;

	if (widget->id != snd_soc_dapm_mixer &&
	    widget->id != snd_soc_dapm_mixer_named_ctl &&
	    widget->id != snd_soc_dapm_switch)
		return -ENODEV;

	/* find dapm widget path assoc with kcontrol */
	list_for_each_entry(path, &widget->dapm->card->paths, list) {
		if (path->kcontrol != kcontrol)
			continue;

		/* found, now check type */
		found = 1;
		path->connect = connect;
		break;
	}

	if (found) {
		if (widget->platform)
			soc_dsp_runtime_update(widget);
		else
			dapm_power_widgets(widget->dapm,
				SND_SOC_DAPM_STREAM_NOP);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_mixer_update_power);

/* show dapm widget status in sys fs */
static ssize_t widget_show(struct snd_soc_dapm_context *dapm,
	const char *name, char *buf, ssize_t count)
{
	struct snd_soc_dapm_widget *w;
	char *state = "not set";

	count += sprintf(buf + count, "\n%s\n", name);

	list_for_each_entry(w, &dapm->card->widgets, list) {

		/* only display widgets that burnm power */
		switch (w->id) {
		case snd_soc_dapm_hp:
		case snd_soc_dapm_mic:
		case snd_soc_dapm_spk:
		case snd_soc_dapm_line:
		case snd_soc_dapm_micbias:
		case snd_soc_dapm_dac:
		case snd_soc_dapm_adc:
		case snd_soc_dapm_pga:
		case snd_soc_dapm_out_drv:
		case snd_soc_dapm_mixer:
		case snd_soc_dapm_mixer_named_ctl:
		case snd_soc_dapm_supply:
			if (w->name)
				count += sprintf(buf + count, "%s: %s\n",
					w->name, w->power ? "On":"Off");
		break;
		default:
		break;
		}
	}

	switch (dapm->bias_level) {
	case SND_SOC_BIAS_ON:
		state = "On";
		break;
	case SND_SOC_BIAS_PREPARE:
		state = "Prepare";
		break;
	case SND_SOC_BIAS_STANDBY:
		state = "Standby";
		break;
	case SND_SOC_BIAS_OFF:
		state = "Off";
		break;
	}
	count += sprintf(buf + count, "PM State: %s\n", state);

	return count;
}

/* show dapm widget status in sys fs */
static ssize_t dapm_widget_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct snd_soc_pcm_runtime *rtd =
			container_of(dev, struct snd_soc_pcm_runtime, dev);
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_platform *platform = rtd->platform;
	ssize_t count = 0;

	count += widget_show(&codec->dapm, codec->name, buf, count);
	count += widget_show(&platform->dapm, platform->name, buf, count);
	return count;
}

static DEVICE_ATTR(dapm_widget, 0444, dapm_widget_show, NULL);

int snd_soc_dapm_sys_add(struct device *dev)
{
	return device_create_file(dev, &dev_attr_dapm_widget);
}

static void snd_soc_dapm_sys_remove(struct device *dev)
{
	device_remove_file(dev, &dev_attr_dapm_widget);
}

/* free all dapm widgets and resources */
static void dapm_free_widgets(struct snd_soc_dapm_context *dapm)
{
	struct snd_soc_dapm_widget *w, *next_w;
	struct snd_soc_dapm_path *p, *next_p;

	list_for_each_entry_safe(w, next_w, &dapm->card->widgets, list) {
		if (w->dapm != dapm)
			continue;
		list_del(&w->list);
		/*
		 * remove source and sink paths associated to this widget.
		 * While removing the path, remove reference to it from both
		 * source and sink widgets so that path is removed only once.
		 */
		list_for_each_entry_safe(p, next_p, &w->sources, list_sink) {
			list_del(&p->list_sink);
			list_del(&p->list_source);
			list_del(&p->list);
			kfree(p->long_name);
			kfree(p);
		}
		list_for_each_entry_safe(p, next_p, &w->sinks, list_source) {
			list_del(&p->list_sink);
			list_del(&p->list_source);
			list_del(&p->list);
			kfree(p->long_name);
			kfree(p);
		}
		kfree(w->kcontrols);
		kfree(w->name);
		kfree(w);
	}
}

static struct snd_soc_dapm_widget *dapm_find_widget(
			struct snd_soc_dapm_context *dapm, const char *pin,
			bool search_other_contexts)
{
	struct snd_soc_dapm_widget *w;
	struct snd_soc_dapm_widget *fallback = NULL;

	list_for_each_entry(w, &dapm->card->widgets, list) {
		if (!strcmp(w->name, pin)) {
			if (w->dapm == dapm)
				return w;
			else
				fallback = w;
		}
	}

	if (search_other_contexts)
		return fallback;

	return NULL;
}

static int snd_soc_dapm_set_pin(struct snd_soc_dapm_context *dapm,
				const char *pin, int status)
{
	struct snd_soc_dapm_widget *w = dapm_find_widget(dapm, pin, true);

	if (!w) {
		dev_err(dapm->dev, "dapm: unknown pin %s\n", pin);
		return -EINVAL;
	}

	w->connected = status;
	if (status == 0)
		w->force = 0;

	return 0;
}

/**
 * snd_soc_dapm_sync - scan and power dapm paths
 * @dapm: DAPM context
 *
 * Walks all dapm audio paths and powers widgets according to their
 * stream or path usage.
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_sync(struct snd_soc_dapm_context *dapm)
{
	return dapm_power_widgets(dapm, SND_SOC_DAPM_STREAM_NOP);
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_sync);

static int snd_soc_dapm_add_route(struct snd_soc_dapm_context *dapm,
				  const struct snd_soc_dapm_route *route)
{
	struct snd_soc_dapm_path *path;
	struct snd_soc_dapm_widget *wsource = NULL, *wsink = NULL, *w;
	struct snd_soc_dapm_widget *wtsource = NULL, *wtsink = NULL;
	const char *sink;
	const char *control = route->control;
	const char *source;
	char prefixed_sink[80];
	char prefixed_source[80];
	int ret = 0;

	if (dapm->codec && dapm->codec->name_prefix) {
		snprintf(prefixed_sink, sizeof(prefixed_sink), "%s %s",
			 dapm->codec->name_prefix, route->sink);
		sink = prefixed_sink;
		snprintf(prefixed_source, sizeof(prefixed_source), "%s %s",
			 dapm->codec->name_prefix, route->source);
		source = prefixed_source;
	} else {
		sink = route->sink;
		source = route->source;
	}

	/*
	 * find src and dest widgets over all widgets but favor a widget from
	 * current DAPM context
	 */
	list_for_each_entry(w, &dapm->card->widgets, list) {
		if (!wsink && !(strcmp(w->name, sink))) {
			wtsink = w;
			if (w->dapm == dapm)
				wsink = w;
			continue;
		}
		if (!wsource && !(strcmp(w->name, source))) {
			wtsource = w;
			if (w->dapm == dapm)
				wsource = w;
		}
	}
	/* use widget from another DAPM context if not found from this */
	if (!wsink)
		wsink = wtsink;
	if (!wsource)
		wsource = wtsource;

	if (wsource == NULL || wsink == NULL)
		return -ENODEV;

	path = kzalloc(sizeof(struct snd_soc_dapm_path), GFP_KERNEL);
	if (!path)
		return -ENOMEM;

	path->source = wsource;
	path->sink = wsink;
	path->connected = route->connected;
	INIT_LIST_HEAD(&path->list);
	INIT_LIST_HEAD(&path->list_source);
	INIT_LIST_HEAD(&path->list_sink);

	/* check for external widgets */
	if (wsink->id == snd_soc_dapm_input) {
		if (wsource->id == snd_soc_dapm_micbias ||
			wsource->id == snd_soc_dapm_mic ||
			wsource->id == snd_soc_dapm_line ||
			wsource->id == snd_soc_dapm_output)
			wsink->ext = 1;
	}
	if (wsource->id == snd_soc_dapm_output) {
		if (wsink->id == snd_soc_dapm_spk ||
			wsink->id == snd_soc_dapm_hp ||
			wsink->id == snd_soc_dapm_line ||
			wsink->id == snd_soc_dapm_input)
			wsource->ext = 1;
	}

	/* connect static paths */
	if (control == NULL) {
		list_add(&path->list, &dapm->card->paths);
		list_add(&path->list_sink, &wsink->sources);
		list_add(&path->list_source, &wsource->sinks);
		path->connect = 1;
		return 0;
	}

	/* connect dynamic paths */
	switch (wsink->id) {
	case snd_soc_dapm_adc:
	case snd_soc_dapm_dac:
	case snd_soc_dapm_pga:
	case snd_soc_dapm_out_drv:
	case snd_soc_dapm_input:
	case snd_soc_dapm_output:
	case snd_soc_dapm_micbias:
	case snd_soc_dapm_vmid:
	case snd_soc_dapm_pre:
	case snd_soc_dapm_post:
	case snd_soc_dapm_supply:
	case snd_soc_dapm_aif_in:
	case snd_soc_dapm_aif_out:
		list_add(&path->list, &dapm->card->paths);
		list_add(&path->list_sink, &wsink->sources);
		list_add(&path->list_source, &wsource->sinks);
		path->connect = 1;
		return 0;
	case snd_soc_dapm_mux:
	case snd_soc_dapm_virt_mux:
	case snd_soc_dapm_value_mux:
		ret = dapm_connect_mux(dapm, wsource, wsink, path, control,
			&wsink->kcontrol_news[0]);
		if (ret != 0)
			goto err;
		break;
	case snd_soc_dapm_switch:
	case snd_soc_dapm_mixer:
	case snd_soc_dapm_mixer_named_ctl:
		ret = dapm_connect_mixer(dapm, wsource, wsink, path, control);
		if (ret != 0)
			goto err;
		break;
	case snd_soc_dapm_hp:
	case snd_soc_dapm_mic:
	case snd_soc_dapm_line:
	case snd_soc_dapm_spk:
		list_add(&path->list, &dapm->card->paths);
		list_add(&path->list_sink, &wsink->sources);
		list_add(&path->list_source, &wsource->sinks);
		path->connect = 0;
		return 0;
	}
	return 0;

err:
	dev_warn(dapm->dev, "asoc: no dapm match for %s --> %s --> %s\n",
		 source, control, sink);
	kfree(path);
	return ret;
}

/**
 * snd_soc_dapm_add_routes - Add routes between DAPM widgets
 * @dapm: DAPM context
 * @route: audio routes
 * @num: number of routes
 *
 * Connects 2 dapm widgets together via a named audio path. The sink is
 * the widget receiving the audio signal, whilst the source is the sender
 * of the audio signal.
 *
 * Returns 0 for success else error. On error all resources can be freed
 * with a call to snd_soc_card_free().
 */
int snd_soc_dapm_add_routes(struct snd_soc_dapm_context *dapm,
			    const struct snd_soc_dapm_route *route, int num)
{
	int i, ret;

	for (i = 0; i < num; i++) {
		ret = snd_soc_dapm_add_route(dapm, route);
		if (ret < 0) {
			dev_err(dapm->dev, "Failed to add route %s->%s\n",
				route->source, route->sink);
			return ret;
		}
		route++;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_add_routes);

/**
 * snd_soc_dapm_new_widgets - add new dapm widgets
 * @dapm: DAPM context
 *
 * Checks the codec for any new dapm widgets and creates them if found.
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_new_widgets(struct snd_soc_dapm_context *dapm)
{
	struct snd_soc_dapm_widget *w;
	unsigned int val;

	list_for_each_entry(w, &dapm->card->widgets, list)
	{
		if (w->new)
			continue;

		if (w->num_kcontrols) {
			w->kcontrols = kzalloc(w->num_kcontrols *
						sizeof(struct snd_kcontrol *),
						GFP_KERNEL);
			if (!w->kcontrols)
				return -ENOMEM;
		}

		switch(w->id) {
		case snd_soc_dapm_switch:
		case snd_soc_dapm_mixer:
		case snd_soc_dapm_mixer_named_ctl:
			w->power_check = dapm_generic_check_power;
			dapm_new_mixer(w);
			break;
		case snd_soc_dapm_mux:
		case snd_soc_dapm_virt_mux:
		case snd_soc_dapm_value_mux:
			w->power_check = dapm_generic_check_power;
			dapm_new_mux(w);
			break;
		case snd_soc_dapm_adc:
		case snd_soc_dapm_aif_out:
			w->power_check = dapm_adc_check_power;
			break;
		case snd_soc_dapm_dac:
		case snd_soc_dapm_aif_in:
			w->power_check = dapm_dac_check_power;
			break;
		case snd_soc_dapm_pga:
		case snd_soc_dapm_out_drv:
			w->power_check = dapm_generic_check_power;
			dapm_new_pga(w);
			break;
		case snd_soc_dapm_input:
		case snd_soc_dapm_output:
		case snd_soc_dapm_micbias:
		case snd_soc_dapm_spk:
		case snd_soc_dapm_hp:
		case snd_soc_dapm_mic:
		case snd_soc_dapm_line:
			w->power_check = dapm_generic_check_power;
			break;
		case snd_soc_dapm_supply:
			w->power_check = dapm_supply_check_power;
		case snd_soc_dapm_vmid:
		case snd_soc_dapm_pre:
		case snd_soc_dapm_post:
			break;
		}

		/* Read the initial power state from the device */
		if (w->reg >= 0) {
			val = soc_widget_read(w, w->reg);
			val &= 1 << w->shift;
			if (w->invert)
				val = !val;

			if (val)
				w->power = 1;
		}

		w->new = 1;

		dapm_debugfs_add_widget(w);
	}

	dapm_power_widgets(dapm, SND_SOC_DAPM_STREAM_NOP);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_new_widgets);

const char *snd_soc_dapm_get_aif(struct snd_soc_dapm_context *dapm,
		const char *stream_name, enum snd_soc_dapm_type type)
{
	struct snd_soc_dapm_widget *w;

	list_for_each_entry(w, &dapm->card->widgets, list) {

		if (!w->sname)
			continue;

		if (w->id == type && strstr(w->sname, stream_name))
			return w->name;

	}
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_get_aif);

/**
 * snd_soc_dapm_get_volsw - dapm mixer get callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to get the value of a dapm mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int invert = mc->invert;
	unsigned int mask = (1 << fls(max)) - 1;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(widget->codec, reg) >> shift) & mask;
	if (shift != rshift)
		ucontrol->value.integer.value[1] =
			(snd_soc_read(widget->codec, reg) >> rshift) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		if (shift != rshift)
			ucontrol->value.integer.value[1] =
				max - ucontrol->value.integer.value[1];
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_get_volsw);

/**
 * snd_soc_dapm_put_volsw - dapm mixer set callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to set the value of a dapm mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val;
	int connect, change;
	struct snd_soc_dapm_update update;
	int wi;

	val = (ucontrol->value.integer.value[0] & mask);

	if (invert)
		val = max - val;
	mask = mask << shift;
	val = val << shift;

	if (val)
		/* new connection */
		connect = invert ? 0 : 1;
	else
		/* old connection must be powered down */
		connect = invert ? 1 : 0;

	mutex_lock(&codec->mutex);

	change = snd_soc_test_bits(widget->codec, reg, mask, val);
	if (change) {
		for (wi = 0; wi < wlist->num_widgets; wi++) {
			widget = wlist->widgets[wi];

			widget->value = val;

			update.kcontrol = kcontrol;
			update.widget = widget;
			update.reg = reg;
			update.mask = mask;
			update.val = val;
			widget->dapm->update = &update;

			snd_soc_dapm_mixer_update_power(widget, kcontrol, connect);

			widget->dapm->update = NULL;
		}
	}

	mutex_unlock(&codec->mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_put_volsw);

/**
 * snd_soc_dapm_get_enum_double - dapm enumerated double mixer get callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to get the value of a dapm enumerated double mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_get_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val, bitmask;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	val = snd_soc_read(widget->codec, e->reg);
	ucontrol->value.enumerated.item[0] = (val >> e->shift_l) & (bitmask - 1);
	if (e->shift_l != e->shift_r)
		ucontrol->value.enumerated.item[1] =
			(val >> e->shift_r) & (bitmask - 1);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_get_enum_double);

/**
 * snd_soc_dapm_put_enum_double - dapm enumerated double mixer set callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to set the value of a dapm enumerated double mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_put_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val, mux, change;
	unsigned int mask, bitmask;
	struct snd_soc_dapm_update update;
	int wi;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;
	mux = ucontrol->value.enumerated.item[0];
	val = mux << e->shift_l;
	mask = (bitmask - 1) << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->max - 1)
			return -EINVAL;
		val |= ucontrol->value.enumerated.item[1] << e->shift_r;
		mask |= (bitmask - 1) << e->shift_r;
	}

	mutex_lock(&codec->mutex);

	change = snd_soc_test_bits(widget->codec, e->reg, mask, val);
	if (change) {
		for (wi = 0; wi < wlist->num_widgets; wi++) {
			widget = wlist->widgets[wi];

			widget->value = val;

			update.kcontrol = kcontrol;
			update.widget = widget;
			update.reg = e->reg;
			update.mask = mask;
			update.val = val;
			widget->dapm->update = &update;

			snd_soc_dapm_mux_update_power(widget, kcontrol, change, mux, e);

			widget->dapm->update = NULL;
		}
	}

	mutex_unlock(&codec->mutex);
	return change;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_put_enum_double);

/**
 * snd_soc_dapm_get_enum_virt - Get virtual DAPM mux
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_get_enum_virt(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];

	ucontrol->value.enumerated.item[0] = widget->value;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_get_enum_virt);

/**
 * snd_soc_dapm_put_enum_virt - Set virtual DAPM mux
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_put_enum_virt(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	int change;
	int ret = 0;
	int wi;

	if (ucontrol->value.enumerated.item[0] >= e->max)
		return -EINVAL;

	mutex_lock(&codec->mutex);

	change = widget->value != ucontrol->value.enumerated.item[0];
	if (change) {
		for (wi = 0; wi < wlist->num_widgets; wi++) {
			widget = wlist->widgets[wi];

			widget->value = ucontrol->value.enumerated.item[0];

			snd_soc_dapm_mux_update_power(widget, kcontrol, change,
					widget->value, e);
		}
	}

	mutex_unlock(&codec->mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_put_enum_virt);

/**
 * snd_soc_dapm_get_value_enum_double - dapm semi enumerated double mixer get
 *					callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to get the value of a dapm semi enumerated double mixer control.
 *
 * Semi enumerated mixer: the enumerated items are referred as values. Can be
 * used for handling bitfield coded enumeration for example.
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_get_value_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg_val, val, mux;

	reg_val = snd_soc_read(widget->codec, e->reg);
	val = (reg_val >> e->shift_l) & e->mask;
	for (mux = 0; mux < e->max; mux++) {
		if (val == e->values[mux])
			break;
	}
	ucontrol->value.enumerated.item[0] = mux;
	if (e->shift_l != e->shift_r) {
		val = (reg_val >> e->shift_r) & e->mask;
		for (mux = 0; mux < e->max; mux++) {
			if (val == e->values[mux])
				break;
		}
		ucontrol->value.enumerated.item[1] = mux;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_get_value_enum_double);

/**
 * snd_soc_dapm_put_value_enum_double - dapm semi enumerated double mixer set
 *					callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to set the value of a dapm semi enumerated double mixer control.
 *
 * Semi enumerated mixer: the enumerated items are referred as values. Can be
 * used for handling bitfield coded enumeration for example.
 *
 * Returns 0 for success.
 */
int snd_soc_dapm_put_value_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val, mux, change;
	unsigned int mask;
	struct snd_soc_dapm_update update;
	int wi;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;
	mux = ucontrol->value.enumerated.item[0];
	val = e->values[ucontrol->value.enumerated.item[0]] << e->shift_l;
	mask = e->mask << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->max - 1)
			return -EINVAL;
		val |= e->values[ucontrol->value.enumerated.item[1]] << e->shift_r;
		mask |= e->mask << e->shift_r;
	}

	mutex_lock(&codec->mutex);

	change = snd_soc_test_bits(widget->codec, e->reg, mask, val);
	if (change) {
		for (wi = 0; wi < wlist->num_widgets; wi++) {
			widget = wlist->widgets[wi];

			widget->value = val;

			update.kcontrol = kcontrol;
			update.widget = widget;
			update.reg = e->reg;
			update.mask = mask;
			update.val = val;
			widget->dapm->update = &update;

			snd_soc_dapm_mux_update_power(widget, kcontrol, change, mux, e);

			widget->dapm->update = NULL;
		}
	}

	mutex_unlock(&codec->mutex);
	return change;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_put_value_enum_double);

/**
 * snd_soc_dapm_info_pin_switch - Info for a pin switch
 *
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a pin switch control.
 */
int snd_soc_dapm_info_pin_switch(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_info_pin_switch);

/**
 * snd_soc_dapm_get_pin_switch - Get information for a pin switch
 *
 * @kcontrol: mixer control
 * @ucontrol: Value
 */
int snd_soc_dapm_get_pin_switch(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	const char *pin = (const char *)kcontrol->private_value;

	mutex_lock(&codec->mutex);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_pin_status(&codec->dapm, pin);

	mutex_unlock(&codec->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_get_pin_switch);

/**
 * snd_soc_dapm_put_pin_switch - Set information for a pin switch
 *
 * @kcontrol: mixer control
 * @ucontrol: Value
 */
int snd_soc_dapm_put_pin_switch(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	const char *pin = (const char *)kcontrol->private_value;

	mutex_lock(&codec->mutex);

	if (ucontrol->value.integer.value[0])
		snd_soc_dapm_enable_pin(&codec->dapm, pin);
	else
		snd_soc_dapm_disable_pin(&codec->dapm, pin);

	snd_soc_dapm_sync(&codec->dapm);

	mutex_unlock(&codec->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_put_pin_switch);

/**
 * snd_soc_dapm_new_control - create new dapm control
 * @dapm: DAPM context
 * @widget: widget template
 *
 * Creates a new dapm control based upon the template.
 *
 * Returns 0 for success else error.
 */
int snd_soc_dapm_new_control(struct snd_soc_dapm_context *dapm,
	const struct snd_soc_dapm_widget *widget)
{
	struct snd_soc_dapm_widget *w;
	size_t name_len;

	if ((w = dapm_cnew_widget(widget)) == NULL)
		return -ENOMEM;

	name_len = strlen(widget->name) + 1;
	if (dapm->codec && dapm->codec->name_prefix)
		name_len += 1 + strlen(dapm->codec->name_prefix);
	w->name = kmalloc(name_len, GFP_KERNEL);
	if (w->name == NULL) {
		kfree(w);
		return -ENOMEM;
	}
	if (dapm->codec && dapm->codec->name_prefix)
		snprintf(w->name, name_len, "%s %s",
			dapm->codec->name_prefix, widget->name);
	else
		snprintf(w->name, name_len, "%s", widget->name);

	dapm->n_widgets++;
	w->dapm = dapm;
	w->codec = dapm->codec;
	w->platform = dapm->platform;
	INIT_LIST_HEAD(&w->sources);
	INIT_LIST_HEAD(&w->sinks);
	INIT_LIST_HEAD(&w->list);
	list_add(&w->list, &dapm->card->widgets);

	/* machine layer set ups unconnected pins and insertions */
	w->connected = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_new_control);

/**
 * snd_soc_dapm_new_controls - create new dapm controls
 * @dapm: DAPM context
 * @widget: widget array
 * @num: number of widgets
 *
 * Creates new DAPM controls based upon the templates.
 *
 * Returns 0 for success else error.
 */
int snd_soc_dapm_new_controls(struct snd_soc_dapm_context *dapm,
	const struct snd_soc_dapm_widget *widget,
	int num)
{
	int i, ret;

	for (i = 0; i < num; i++) {
		ret = snd_soc_dapm_new_control(dapm, widget);
		if (ret < 0) {
			dev_err(dapm->dev,
				"ASoC: Failed to create DAPM control %s: %d\n",
				widget->name, ret);
			return ret;
		}
		widget++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_new_controls);

static void soc_dapm_stream_event(struct snd_soc_dapm_context *dapm,
	const char *stream, int event)
{
	struct snd_soc_dapm_widget *w;

	if (!dapm)
		return;

	list_for_each_entry(w, &dapm->card->widgets, list)
	{
		if (!w->sname || w->dapm != dapm)
			continue;
		dev_dbg(w->dapm->dev, "widget %s\n %s stream %s event %d\n",
			w->name, w->sname, stream, event);
		if (strstr(w->sname, stream)) {
			switch(event) {
			case SND_SOC_DAPM_STREAM_START:
				w->active = 1;
				break;
			case SND_SOC_DAPM_STREAM_STOP:
				w->active = 0;
				break;
			case SND_SOC_DAPM_STREAM_SUSPEND:
			case SND_SOC_DAPM_STREAM_RESUME:
			case SND_SOC_DAPM_STREAM_PAUSE_PUSH:
			case SND_SOC_DAPM_STREAM_PAUSE_RELEASE:
				break;
			}
		}
	}

	dapm_power_widgets(dapm, event);
	/* do we need to notify any clients that DAPM stream is complete */
	if (dapm->stream_event)
		dapm->stream_event(dapm);
}

/**
 * snd_soc_dapm_stream_event - send a stream event to the dapm core
 * @rtd: PCM runtime data
 * @stream: stream name
 * @event: stream event
 *
 * Sends a stream event to the dapm core. The core then makes any
 * necessary widget power changes.
 *
 * Returns 0 for success else error.
 */
int snd_soc_dapm_stream_event(struct snd_soc_pcm_runtime *rtd,
	const char *stream, int event)
{
	if (stream == NULL)
		return 0;

	mutex_lock(&rtd->card->dapm_mutex);
	mutex_lock(&rtd->codec->mutex);
	soc_dapm_stream_event(&rtd->platform->dapm, stream, event);
	soc_dapm_stream_event(&rtd->codec->dapm, stream, event);
	mutex_unlock(&rtd->codec->mutex);
	mutex_unlock(&rtd->card->dapm_mutex);
	return 0;
}

/**
 * snd_soc_dapm_enable_pin - enable pin.
 * @dapm: DAPM context
 * @pin: pin name
 *
 * Enables input/output pin and its parents or children widgets iff there is
 * a valid audio route and active audio stream.
 * NOTE: snd_soc_dapm_sync() needs to be called after this for DAPM to
 * do any widget power switching.
 */
int snd_soc_dapm_enable_pin(struct snd_soc_dapm_context *dapm, const char *pin)
{
	return snd_soc_dapm_set_pin(dapm, pin, 1);
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_enable_pin);

/**
 * snd_soc_dapm_force_enable_pin - force a pin to be enabled
 * @dapm: DAPM context
 * @pin: pin name
 *
 * Enables input/output pin regardless of any other state.  This is
 * intended for use with microphone bias supplies used in microphone
 * jack detection.
 *
 * NOTE: snd_soc_dapm_sync() needs to be called after this for DAPM to
 * do any widget power switching.
 */
int snd_soc_dapm_force_enable_pin(struct snd_soc_dapm_context *dapm,
				  const char *pin)
{
	struct snd_soc_dapm_widget *w = dapm_find_widget(dapm, pin, true);

	if (!w) {
		dev_err(dapm->dev, "dapm: unknown pin %s\n", pin);
		return -EINVAL;
	}

	dev_dbg(w->dapm->dev, "dapm: force enable pin %s\n", pin);
	w->connected = 1;
	w->force = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_force_enable_pin);

/**
 * snd_soc_dapm_disable_pin - disable pin.
 * @dapm: DAPM context
 * @pin: pin name
 *
 * Disables input/output pin and its parents or children widgets.
 * NOTE: snd_soc_dapm_sync() needs to be called after this for DAPM to
 * do any widget power switching.
 */
int snd_soc_dapm_disable_pin(struct snd_soc_dapm_context *dapm,
			     const char *pin)
{
	return snd_soc_dapm_set_pin(dapm, pin, 0);
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_disable_pin);

/**
 * snd_soc_dapm_nc_pin - permanently disable pin.
 * @dapm: DAPM context
 * @pin: pin name
 *
 * Marks the specified pin as being not connected, disabling it along
 * any parent or child widgets.  At present this is identical to
 * snd_soc_dapm_disable_pin() but in future it will be extended to do
 * additional things such as disabling controls which only affect
 * paths through the pin.
 *
 * NOTE: snd_soc_dapm_sync() needs to be called after this for DAPM to
 * do any widget power switching.
 */
int snd_soc_dapm_nc_pin(struct snd_soc_dapm_context *dapm, const char *pin)
{
	return snd_soc_dapm_set_pin(dapm, pin, 0);
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_nc_pin);

/**
 * snd_soc_dapm_get_pin_status - get audio pin status
 * @dapm: DAPM context
 * @pin: audio signal pin endpoint (or start point)
 *
 * Get audio pin status - connected or disconnected.
 *
 * Returns 1 for connected otherwise 0.
 */
int snd_soc_dapm_get_pin_status(struct snd_soc_dapm_context *dapm,
				const char *pin)
{
	struct snd_soc_dapm_widget *w = dapm_find_widget(dapm, pin, true);

	if (w)
		return w->connected;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_get_pin_status);

/**
 * snd_soc_dapm_ignore_suspend - ignore suspend status for DAPM endpoint
 * @dapm: DAPM context
 * @pin: audio signal pin endpoint (or start point)
 *
 * Mark the given endpoint or pin as ignoring suspend.  When the
 * system is disabled a path between two endpoints flagged as ignoring
 * suspend will not be disabled.  The path must already be enabled via
 * normal means at suspend time, it will not be turned on if it was not
 * already enabled.
 */
int snd_soc_dapm_ignore_suspend(struct snd_soc_dapm_context *dapm,
				const char *pin)
{
	struct snd_soc_dapm_widget *w = dapm_find_widget(dapm, pin, false);

	if (!w) {
		dev_err(dapm->dev, "dapm: unknown pin %s\n", pin);
		return -EINVAL;
	}

	w->ignore_suspend = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_ignore_suspend);

/**
 * snd_soc_dapm_free - free dapm resources
 * @card: SoC device
 *
 * Free all dapm widgets and resources.
 */
void snd_soc_dapm_free(struct snd_soc_dapm_context *dapm)
{
	snd_soc_dapm_sys_remove(dapm->dev);
	dapm_debugfs_cleanup(dapm);
	dapm_free_widgets(dapm);
	list_del(&dapm->list);
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_free);

static void soc_dapm_shutdown_codec(struct snd_soc_dapm_context *dapm)
{
	struct snd_soc_dapm_widget *w;
	LIST_HEAD(down_list);
	int powerdown = 0;

	list_for_each_entry(w, &dapm->card->widgets, list) {
		if (w->dapm != dapm)
			continue;
		if (w->power) {
			dapm_seq_insert(w, &down_list, false);
			w->power = 0;
			powerdown = 1;
		}
	}

	/* If there were no widgets to power down we're already in
	 * standby.
	 */
	if (powerdown) {
		snd_soc_dapm_set_bias_level(dapm, SND_SOC_BIAS_PREPARE);
		dapm_seq_run(dapm, &down_list, 0, false);
		snd_soc_dapm_set_bias_level(dapm, SND_SOC_BIAS_STANDBY);
	}
}

/*
 * snd_soc_dapm_shutdown - callback for system shutdown
 */
void snd_soc_dapm_shutdown(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec;
	struct snd_soc_platform *platform;

	list_for_each_entry(codec, &card->codec_dev_list, list) {
		soc_dapm_shutdown_codec(&codec->dapm);
		snd_soc_dapm_set_bias_level(&codec->dapm, SND_SOC_BIAS_OFF);
	}

	list_for_each_entry(platform, &card->platform_dev_list, list) {
		soc_dapm_shutdown_codec(&platform->dapm);
		snd_soc_dapm_set_bias_level(&platform->dapm, SND_SOC_BIAS_OFF);
	}
}

/* Module information */
MODULE_AUTHOR("Liam Girdwood, lrg@slimlogic.co.uk");
MODULE_DESCRIPTION("Dynamic Audio Power Management core for ALSA SoC");
MODULE_LICENSE("GPL v2");
