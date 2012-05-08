/******************************************************************************
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2008 - 2011 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <net/mac80211.h>

#include "iwl-eeprom.h"
#include "iwl-dev.h" /* FIXME: remove */
#include "iwl-debug.h"
#include "iwl-core.h"
#include "iwl-io.h"
#include "iwl-power.h"
#include "iwl-sta.h"
#include "iwl-helpers.h"
#include "iwl-agn.h"


/*
 * set bt_coex_active to true, uCode will do kill/defer
 * every time the priority line is asserted (BT is sending signals on the
 * priority line in the PCIx).
 * set bt_coex_active to false, uCode will ignore the BT activity and
 * perform the normal operation
 *
 * User might experience transmit issue on some platform due to WiFi/BT
 * co-exist problem. The possible behaviors are:
 *   Able to scan and finding all the available AP
 *   Not able to associate with any AP
 * On those platforms, WiFi communication can be restored by set
 * "bt_coex_active" module parameter to "false"
 *
 * default: bt_coex_active = true (BT_COEX_ENABLE)
 */
bool bt_coex_active = true;
module_param(bt_coex_active, bool, S_IRUGO);
MODULE_PARM_DESC(bt_coex_active, "enable wifi/bluetooth co-exist");

u32 iwl_debug_level;

const u8 iwl_bcast_addr[ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

#define MAX_BIT_RATE_40_MHZ 150 /* Mbps */
#define MAX_BIT_RATE_20_MHZ 72 /* Mbps */
static void iwlcore_init_ht_hw_capab(const struct iwl_priv *priv,
			      struct ieee80211_sta_ht_cap *ht_info,
			      enum ieee80211_band band)
{
	u16 max_bit_rate = 0;
	u8 rx_chains_num = priv->hw_params.rx_chains_num;
	u8 tx_chains_num = priv->hw_params.tx_chains_num;

	ht_info->cap = 0;
	memset(&ht_info->mcs, 0, sizeof(ht_info->mcs));

	ht_info->ht_supported = true;

	if (priv->cfg->ht_params &&
	    priv->cfg->ht_params->ht_greenfield_support)
		ht_info->cap |= IEEE80211_HT_CAP_GRN_FLD;
	ht_info->cap |= IEEE80211_HT_CAP_SGI_20;
	max_bit_rate = MAX_BIT_RATE_20_MHZ;
	if (priv->hw_params.ht40_channel & BIT(band)) {
		ht_info->cap |= IEEE80211_HT_CAP_SUP_WIDTH_20_40;
		ht_info->cap |= IEEE80211_HT_CAP_SGI_40;
		ht_info->mcs.rx_mask[4] = 0x01;
		max_bit_rate = MAX_BIT_RATE_40_MHZ;
	}

	if (iwlagn_mod_params.amsdu_size_8K)
		ht_info->cap |= IEEE80211_HT_CAP_MAX_AMSDU;

	ht_info->ampdu_factor = CFG_HT_RX_AMPDU_FACTOR_DEF;
	if (priv->cfg->bt_params && priv->cfg->bt_params->ampdu_factor)
		ht_info->ampdu_factor = priv->cfg->bt_params->ampdu_factor;
	ht_info->ampdu_density = CFG_HT_MPDU_DENSITY_DEF;
	if (priv->cfg->bt_params && priv->cfg->bt_params->ampdu_density)
		ht_info->ampdu_density = priv->cfg->bt_params->ampdu_density;

	ht_info->mcs.rx_mask[0] = 0xFF;
	if (rx_chains_num >= 2)
		ht_info->mcs.rx_mask[1] = 0xFF;
	if (rx_chains_num >= 3)
		ht_info->mcs.rx_mask[2] = 0xFF;

	/* Highest supported Rx data rate */
	max_bit_rate *= rx_chains_num;
	WARN_ON(max_bit_rate & ~IEEE80211_HT_MCS_RX_HIGHEST_MASK);
	ht_info->mcs.rx_highest = cpu_to_le16(max_bit_rate);

	/* Tx MCS capabilities */
	ht_info->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
	if (tx_chains_num != rx_chains_num) {
		ht_info->mcs.tx_params |= IEEE80211_HT_MCS_TX_RX_DIFF;
		ht_info->mcs.tx_params |= ((tx_chains_num - 1) <<
				IEEE80211_HT_MCS_TX_MAX_STREAMS_SHIFT);
	}
}

/**
 * iwlcore_init_geos - Initialize mac80211's geo/channel info based from eeprom
 */
int iwlcore_init_geos(struct iwl_priv *priv)
{
	struct iwl_channel_info *ch;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *channels;
	struct ieee80211_channel *geo_ch;
	struct ieee80211_rate *rates;
	int i = 0;
	s8 max_tx_power = IWLAGN_TX_POWER_TARGET_POWER_MIN;

	if (priv->bands[IEEE80211_BAND_2GHZ].n_bitrates ||
	    priv->bands[IEEE80211_BAND_5GHZ].n_bitrates) {
		IWL_DEBUG_INFO(priv, "Geography modes already initialized.\n");
		set_bit(STATUS_GEO_CONFIGURED, &priv->status);
		return 0;
	}

	channels = kzalloc(sizeof(struct ieee80211_channel) *
			   priv->channel_count, GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	rates = kzalloc((sizeof(struct ieee80211_rate) * IWL_RATE_COUNT_LEGACY),
			GFP_KERNEL);
	if (!rates) {
		kfree(channels);
		return -ENOMEM;
	}

	/* 5.2GHz channels start after the 2.4GHz channels */
	sband = &priv->bands[IEEE80211_BAND_5GHZ];
	sband->channels = &channels[ARRAY_SIZE(iwl_eeprom_band_1)];
	/* just OFDM */
	sband->bitrates = &rates[IWL_FIRST_OFDM_RATE];
	sband->n_bitrates = IWL_RATE_COUNT_LEGACY - IWL_FIRST_OFDM_RATE;

	if (priv->cfg->sku & IWL_SKU_N)
		iwlcore_init_ht_hw_capab(priv, &sband->ht_cap,
					 IEEE80211_BAND_5GHZ);

	sband = &priv->bands[IEEE80211_BAND_2GHZ];
	sband->channels = channels;
	/* OFDM & CCK */
	sband->bitrates = rates;
	sband->n_bitrates = IWL_RATE_COUNT_LEGACY;

	if (priv->cfg->sku & IWL_SKU_N)
		iwlcore_init_ht_hw_capab(priv, &sband->ht_cap,
					 IEEE80211_BAND_2GHZ);

	priv->ieee_channels = channels;
	priv->ieee_rates = rates;

	for (i = 0;  i < priv->channel_count; i++) {
		ch = &priv->channel_info[i];

		/* FIXME: might be removed if scan is OK */
		if (!is_channel_valid(ch))
			continue;

		sband =  &priv->bands[ch->band];

		geo_ch = &sband->channels[sband->n_channels++];

		geo_ch->center_freq =
			ieee80211_channel_to_frequency(ch->channel, ch->band);
		geo_ch->max_power = ch->max_power_avg;
		geo_ch->max_antenna_gain = 0xff;
		geo_ch->hw_value = ch->channel;

		if (is_channel_valid(ch)) {
			if (!(ch->flags & EEPROM_CHANNEL_IBSS))
				geo_ch->flags |= IEEE80211_CHAN_NO_IBSS;

			if (!(ch->flags & EEPROM_CHANNEL_ACTIVE))
				geo_ch->flags |= IEEE80211_CHAN_PASSIVE_SCAN;

			if (ch->flags & EEPROM_CHANNEL_RADAR)
				geo_ch->flags |= IEEE80211_CHAN_RADAR;

			geo_ch->flags |= ch->ht40_extension_channel;

			if (ch->max_power_avg > max_tx_power)
				max_tx_power = ch->max_power_avg;
		} else {
			geo_ch->flags |= IEEE80211_CHAN_DISABLED;
		}

		IWL_DEBUG_INFO(priv, "Channel %d Freq=%d[%sGHz] %s flag=0x%X\n",
				ch->channel, geo_ch->center_freq,
				is_channel_a_band(ch) ?  "5.2" : "2.4",
				geo_ch->flags & IEEE80211_CHAN_DISABLED ?
				"restricted" : "valid",
				 geo_ch->flags);
	}

	priv->tx_power_device_lmt = max_tx_power;
	priv->tx_power_user_lmt = max_tx_power;
	priv->tx_power_next = max_tx_power;

	if ((priv->bands[IEEE80211_BAND_5GHZ].n_channels == 0) &&
	     priv->cfg->sku & IWL_SKU_A) {
		IWL_INFO(priv, "Incorrectly detected BG card as ABG. "
			"Please send your PCI ID 0x%04X:0x%04X to maintainer.\n",
			   priv->pci_dev->device,
			   priv->pci_dev->subsystem_device);
		priv->cfg->sku &= ~IWL_SKU_A;
	}

	IWL_INFO(priv, "Tunable channels: %d 802.11bg, %d 802.11a channels\n",
		   priv->bands[IEEE80211_BAND_2GHZ].n_channels,
		   priv->bands[IEEE80211_BAND_5GHZ].n_channels);

	set_bit(STATUS_GEO_CONFIGURED, &priv->status);

	return 0;
}

/*
 * iwlcore_free_geos - undo allocations in iwlcore_init_geos
 */
void iwlcore_free_geos(struct iwl_priv *priv)
{
	kfree(priv->ieee_channels);
	kfree(priv->ieee_rates);
	clear_bit(STATUS_GEO_CONFIGURED, &priv->status);
}

static bool iwl_is_channel_extension(struct iwl_priv *priv,
				     enum ieee80211_band band,
				     u16 channel, u8 extension_chan_offset)
{
	const struct iwl_channel_info *ch_info;

	ch_info = iwl_get_channel_info(priv, band, channel);
	if (!is_channel_valid(ch_info))
		return false;

	if (extension_chan_offset == IEEE80211_HT_PARAM_CHA_SEC_ABOVE)
		return !(ch_info->ht40_extension_channel &
					IEEE80211_CHAN_NO_HT40PLUS);
	else if (extension_chan_offset == IEEE80211_HT_PARAM_CHA_SEC_BELOW)
		return !(ch_info->ht40_extension_channel &
					IEEE80211_CHAN_NO_HT40MINUS);

	return false;
}

bool iwl_is_ht40_tx_allowed(struct iwl_priv *priv,
			    struct iwl_rxon_context *ctx,
			    struct ieee80211_sta_ht_cap *ht_cap)
{
	if (!ctx->ht.enabled || !ctx->ht.is_40mhz)
		return false;

	/*
	 * We do not check for IEEE80211_HT_CAP_SUP_WIDTH_20_40
	 * the bit will not set if it is pure 40MHz case
	 */
	if (ht_cap && !ht_cap->ht_supported)
		return false;

#ifdef CONFIG_IWLWIFI_DEBUGFS
	if (priv->disable_ht40)
		return false;
#endif

	return iwl_is_channel_extension(priv, priv->band,
			le16_to_cpu(ctx->staging.channel),
			ctx->ht.extension_chan_offset);
}

static u16 iwl_adjust_beacon_interval(u16 beacon_val, u16 max_beacon_val)
{
	u16 new_val;
	u16 beacon_factor;

	/*
	 * If mac80211 hasn't given us a beacon interval, program
	 * the default into the device (not checking this here
	 * would cause the adjustment below to return the maximum
	 * value, which may break PAN.)
	 */
	if (!beacon_val)
		return DEFAULT_BEACON_INTERVAL;

	/*
	 * If the beacon interval we obtained from the peer
	 * is too large, we'll have to wake up more often
	 * (and in IBSS case, we'll beacon too much)
	 *
	 * For example, if max_beacon_val is 4096, and the
	 * requested beacon interval is 7000, we'll have to
	 * use 3500 to be able to wake up on the beacons.
	 *
	 * This could badly influence beacon detection stats.
	 */

	beacon_factor = (beacon_val + max_beacon_val) / max_beacon_val;
	new_val = beacon_val / beacon_factor;

	if (!new_val)
		new_val = max_beacon_val;

	return new_val;
}

int iwl_send_rxon_timing(struct iwl_priv *priv, struct iwl_rxon_context *ctx)
{
	u64 tsf;
	s32 interval_tm, rem;
	struct ieee80211_conf *conf = NULL;
	u16 beacon_int;
	struct ieee80211_vif *vif = ctx->vif;

	conf = ieee80211_get_hw_conf(priv->hw);

	lockdep_assert_held(&priv->mutex);

	memset(&ctx->timing, 0, sizeof(struct iwl_rxon_time_cmd));

	ctx->timing.timestamp = cpu_to_le64(priv->timestamp);
	ctx->timing.listen_interval = cpu_to_le16(conf->listen_interval);

	beacon_int = vif ? vif->bss_conf.beacon_int : 0;

	/*
	 * TODO: For IBSS we need to get atim_window from mac80211,
	 *	 for now just always use 0
	 */
	ctx->timing.atim_window = 0;

	if (ctx->ctxid == IWL_RXON_CTX_PAN &&
	    (!ctx->vif || ctx->vif->type != NL80211_IFTYPE_STATION) &&
	    iwl_is_associated(priv, IWL_RXON_CTX_BSS) &&
	    priv->contexts[IWL_RXON_CTX_BSS].vif &&
	    priv->contexts[IWL_RXON_CTX_BSS].vif->bss_conf.beacon_int) {
		ctx->timing.beacon_interval =
			priv->contexts[IWL_RXON_CTX_BSS].timing.beacon_interval;
		beacon_int = le16_to_cpu(ctx->timing.beacon_interval);
	} else if (ctx->ctxid == IWL_RXON_CTX_BSS &&
		   iwl_is_associated(priv, IWL_RXON_CTX_PAN) &&
		   priv->contexts[IWL_RXON_CTX_PAN].vif &&
		   priv->contexts[IWL_RXON_CTX_PAN].vif->bss_conf.beacon_int &&
		   (!iwl_is_associated_ctx(ctx) || !ctx->vif ||
		    !ctx->vif->bss_conf.beacon_int)) {
		ctx->timing.beacon_interval =
			priv->contexts[IWL_RXON_CTX_PAN].timing.beacon_interval;
		beacon_int = le16_to_cpu(ctx->timing.beacon_interval);
	} else {
		beacon_int = iwl_adjust_beacon_interval(beacon_int,
				priv->hw_params.max_beacon_itrvl * TIME_UNIT);
		ctx->timing.beacon_interval = cpu_to_le16(beacon_int);
	}

	tsf = priv->timestamp; /* tsf is modifed by do_div: copy it */
	interval_tm = beacon_int * TIME_UNIT;
	rem = do_div(tsf, interval_tm);
	ctx->timing.beacon_init_val = cpu_to_le32(interval_tm - rem);

	ctx->timing.dtim_period = vif ? (vif->bss_conf.dtim_period ?: 1) : 1;

	IWL_DEBUG_ASSOC(priv,
			"beacon interval %d beacon timer %d beacon tim %d\n",
			le16_to_cpu(ctx->timing.beacon_interval),
			le32_to_cpu(ctx->timing.beacon_init_val),
			le16_to_cpu(ctx->timing.atim_window));

	return iwl_send_cmd_pdu(priv, ctx->rxon_timing_cmd,
				sizeof(ctx->timing), &ctx->timing);
}

void iwl_set_rxon_hwcrypto(struct iwl_priv *priv, struct iwl_rxon_context *ctx,
			   int hw_decrypt)
{
	struct iwl_rxon_cmd *rxon = &ctx->staging;

	if (hw_decrypt)
		rxon->filter_flags &= ~RXON_FILTER_DIS_DECRYPT_MSK;
	else
		rxon->filter_flags |= RXON_FILTER_DIS_DECRYPT_MSK;

}

/* validate RXON structure is valid */
int iwl_check_rxon_cmd(struct iwl_priv *priv, struct iwl_rxon_context *ctx)
{
	struct iwl_rxon_cmd *rxon = &ctx->staging;
	u32 errors = 0;

	if (rxon->flags & RXON_FLG_BAND_24G_MSK) {
		if (rxon->flags & RXON_FLG_TGJ_NARROW_BAND_MSK) {
			IWL_WARN(priv, "check 2.4G: wrong narrow\n");
			errors |= BIT(0);
		}
		if (rxon->flags & RXON_FLG_RADAR_DETECT_MSK) {
			IWL_WARN(priv, "check 2.4G: wrong radar\n");
			errors |= BIT(1);
		}
	} else {
		if (!(rxon->flags & RXON_FLG_SHORT_SLOT_MSK)) {
			IWL_WARN(priv, "check 5.2G: not short slot!\n");
			errors |= BIT(2);
		}
		if (rxon->flags & RXON_FLG_CCK_MSK) {
			IWL_WARN(priv, "check 5.2G: CCK!\n");
			errors |= BIT(3);
		}
	}
	if ((rxon->node_addr[0] | rxon->bssid_addr[0]) & 0x1) {
		IWL_WARN(priv, "mac/bssid mcast!\n");
		errors |= BIT(4);
	}

	/* make sure basic rates 6Mbps and 1Mbps are supported */
	if ((rxon->ofdm_basic_rates & IWL_RATE_6M_MASK) == 0 &&
	    (rxon->cck_basic_rates & IWL_RATE_1M_MASK) == 0) {
		IWL_WARN(priv, "neither 1 nor 6 are basic\n");
		errors |= BIT(5);
	}

	if (le16_to_cpu(rxon->assoc_id) > 2007) {
		IWL_WARN(priv, "aid > 2007\n");
		errors |= BIT(6);
	}

	if ((rxon->flags & (RXON_FLG_CCK_MSK | RXON_FLG_SHORT_SLOT_MSK))
			== (RXON_FLG_CCK_MSK | RXON_FLG_SHORT_SLOT_MSK)) {
		IWL_WARN(priv, "CCK and short slot\n");
		errors |= BIT(7);
	}

	if ((rxon->flags & (RXON_FLG_CCK_MSK | RXON_FLG_AUTO_DETECT_MSK))
			== (RXON_FLG_CCK_MSK | RXON_FLG_AUTO_DETECT_MSK)) {
		IWL_WARN(priv, "CCK and auto detect");
		errors |= BIT(8);
	}

	if ((rxon->flags & (RXON_FLG_AUTO_DETECT_MSK |
			    RXON_FLG_TGG_PROTECT_MSK)) ==
			    RXON_FLG_TGG_PROTECT_MSK) {
		IWL_WARN(priv, "TGg but no auto-detect\n");
		errors |= BIT(9);
	}

	if (rxon->channel == 0) {
		IWL_WARN(priv, "zero channel is invalid\n");
		errors |= BIT(10);
	}

	WARN(errors, "Invalid RXON (%#x), channel %d",
	     errors, le16_to_cpu(rxon->channel));

	return errors ? -EINVAL : 0;
}

/**
 * iwl_full_rxon_required - check if full RXON (vs RXON_ASSOC) cmd is needed
 * @priv: staging_rxon is compared to active_rxon
 *
 * If the RXON structure is changing enough to require a new tune,
 * or is clearing the RXON_FILTER_ASSOC_MSK, then return 1 to indicate that
 * a new tune (full RXON command, rather than RXON_ASSOC cmd) is required.
 */
int iwl_full_rxon_required(struct iwl_priv *priv,
			   struct iwl_rxon_context *ctx)
{
	const struct iwl_rxon_cmd *staging = &ctx->staging;
	const struct iwl_rxon_cmd *active = &ctx->active;

#define CHK(cond)							\
	if ((cond)) {							\
		IWL_DEBUG_INFO(priv, "need full RXON - " #cond "\n");	\
		return 1;						\
	}

#define CHK_NEQ(c1, c2)						\
	if ((c1) != (c2)) {					\
		IWL_DEBUG_INFO(priv, "need full RXON - "	\
			       #c1 " != " #c2 " - %d != %d\n",	\
			       (c1), (c2));			\
		return 1;					\
	}

	/* These items are only settable from the full RXON command */
	CHK(!iwl_is_associated_ctx(ctx));
	CHK(compare_ether_addr(staging->bssid_addr, active->bssid_addr));
	CHK(compare_ether_addr(staging->node_addr, active->node_addr));
	CHK(compare_ether_addr(staging->wlap_bssid_addr,
				active->wlap_bssid_addr));
	CHK_NEQ(staging->dev_type, active->dev_type);
	CHK_NEQ(staging->channel, active->channel);
	CHK_NEQ(staging->air_propagation, active->air_propagation);
	CHK_NEQ(staging->ofdm_ht_single_stream_basic_rates,
		active->ofdm_ht_single_stream_basic_rates);
	CHK_NEQ(staging->ofdm_ht_dual_stream_basic_rates,
		active->ofdm_ht_dual_stream_basic_rates);
	CHK_NEQ(staging->ofdm_ht_triple_stream_basic_rates,
		active->ofdm_ht_triple_stream_basic_rates);
	CHK_NEQ(staging->assoc_id, active->assoc_id);

	/* flags, filter_flags, ofdm_basic_rates, and cck_basic_rates can
	 * be updated with the RXON_ASSOC command -- however only some
	 * flag transitions are allowed using RXON_ASSOC */

	/* Check if we are not switching bands */
	CHK_NEQ(staging->flags & RXON_FLG_BAND_24G_MSK,
		active->flags & RXON_FLG_BAND_24G_MSK);

	/* Check if we are switching association toggle */
	CHK_NEQ(staging->filter_flags & RXON_FILTER_ASSOC_MSK,
		active->filter_flags & RXON_FILTER_ASSOC_MSK);

#undef CHK
#undef CHK_NEQ

	return 0;
}

u8 iwl_rate_get_lowest_plcp(struct iwl_priv *priv,
			    struct iwl_rxon_context *ctx)
{
	/*
	 * Assign the lowest rate -- should really get this from
	 * the beacon skb from mac80211.
	 */
	if (ctx->staging.flags & RXON_FLG_BAND_24G_MSK)
		return IWL_RATE_1M_PLCP;
	else
		return IWL_RATE_6M_PLCP;
}

static void _iwl_set_rxon_ht(struct iwl_priv *priv,
			     struct iwl_ht_config *ht_conf,
			     struct iwl_rxon_context *ctx)
{
	struct iwl_rxon_cmd *rxon = &ctx->staging;

	if (!ctx->ht.enabled) {
		rxon->flags &= ~(RXON_FLG_CHANNEL_MODE_MSK |
			RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK |
			RXON_FLG_HT40_PROT_MSK |
			RXON_FLG_HT_PROT_MSK);
		return;
	}

	/* FIXME: if the definition of ht.protection changed, the "translation"
	 * will be needed for rxon->flags
	 */
	rxon->flags |= cpu_to_le32(ctx->ht.protection << RXON_FLG_HT_OPERATING_MODE_POS);

	/* Set up channel bandwidth:
	 * 20 MHz only, 20/40 mixed or pure 40 if ht40 ok */
	/* clear the HT channel mode before set the mode */
	rxon->flags &= ~(RXON_FLG_CHANNEL_MODE_MSK |
			 RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK);
	if (iwl_is_ht40_tx_allowed(priv, ctx, NULL)) {
		/* pure ht40 */
		if (ctx->ht.protection == IEEE80211_HT_OP_MODE_PROTECTION_20MHZ) {
			rxon->flags |= RXON_FLG_CHANNEL_MODE_PURE_40;
			/* Note: control channel is opposite of extension channel */
			switch (ctx->ht.extension_chan_offset) {
			case IEEE80211_HT_PARAM_CHA_SEC_ABOVE:
				rxon->flags &= ~RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK;
				break;
			case IEEE80211_HT_PARAM_CHA_SEC_BELOW:
				rxon->flags |= RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK;
				break;
			}
		} else {
			/* Note: control channel is opposite of extension channel */
			switch (ctx->ht.extension_chan_offset) {
			case IEEE80211_HT_PARAM_CHA_SEC_ABOVE:
				rxon->flags &= ~(RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK);
				rxon->flags |= RXON_FLG_CHANNEL_MODE_MIXED;
				break;
			case IEEE80211_HT_PARAM_CHA_SEC_BELOW:
				rxon->flags |= RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK;
				rxon->flags |= RXON_FLG_CHANNEL_MODE_MIXED;
				break;
			case IEEE80211_HT_PARAM_CHA_SEC_NONE:
			default:
				/* channel location only valid if in Mixed mode */
				IWL_ERR(priv, "invalid extension channel offset\n");
				break;
			}
		}
	} else {
		rxon->flags |= RXON_FLG_CHANNEL_MODE_LEGACY;
	}

	if (priv->cfg->ops->hcmd->set_rxon_chain)
		priv->cfg->ops->hcmd->set_rxon_chain(priv, ctx);

	IWL_DEBUG_ASSOC(priv, "rxon flags 0x%X operation mode :0x%X "
			"extension channel offset 0x%x\n",
			le32_to_cpu(rxon->flags), ctx->ht.protection,
			ctx->ht.extension_chan_offset);
}

void iwl_set_rxon_ht(struct iwl_priv *priv, struct iwl_ht_config *ht_conf)
{
	struct iwl_rxon_context *ctx;

	for_each_context(priv, ctx)
		_iwl_set_rxon_ht(priv, ht_conf, ctx);
}

/* Return valid, unused, channel for a passive scan to reset the RF */
u8 iwl_get_single_channel_number(struct iwl_priv *priv,
				 enum ieee80211_band band)
{
	const struct iwl_channel_info *ch_info;
	int i;
	u8 channel = 0;
	u8 min, max;
	struct iwl_rxon_context *ctx;

	if (band == IEEE80211_BAND_5GHZ) {
		min = 14;
		max = priv->channel_count;
	} else {
		min = 0;
		max = 14;
	}

	for (i = min; i < max; i++) {
		bool busy = false;

		for_each_context(priv, ctx) {
			busy = priv->channel_info[i].channel ==
				le16_to_cpu(ctx->staging.channel);
			if (busy)
				break;
		}

		if (busy)
			continue;

		channel = priv->channel_info[i].channel;
		ch_info = iwl_get_channel_info(priv, band, channel);
		if (is_channel_valid(ch_info))
			break;
	}

	return channel;
}

/**
 * iwl_set_rxon_channel - Set the band and channel values in staging RXON
 * @ch: requested channel as a pointer to struct ieee80211_channel

 * NOTE:  Does not commit to the hardware; it sets appropriate bit fields
 * in the staging RXON flag structure based on the ch->band
 */
int iwl_set_rxon_channel(struct iwl_priv *priv, struct ieee80211_channel *ch,
			 struct iwl_rxon_context *ctx)
{
	enum ieee80211_band band = ch->band;
	u16 channel = ch->hw_value;

	if ((le16_to_cpu(ctx->staging.channel) == channel) &&
	    (priv->band == band))
		return 0;

	ctx->staging.channel = cpu_to_le16(channel);
	if (band == IEEE80211_BAND_5GHZ)
		ctx->staging.flags &= ~RXON_FLG_BAND_24G_MSK;
	else
		ctx->staging.flags |= RXON_FLG_BAND_24G_MSK;

	priv->band = band;

	IWL_DEBUG_INFO(priv, "Staging channel set to %d [%d]\n", channel, band);

	return 0;
}

void iwl_set_flags_for_band(struct iwl_priv *priv,
			    struct iwl_rxon_context *ctx,
			    enum ieee80211_band band,
			    struct ieee80211_vif *vif)
{
	if (band == IEEE80211_BAND_5GHZ) {
		ctx->staging.flags &=
		    ~(RXON_FLG_BAND_24G_MSK | RXON_FLG_AUTO_DETECT_MSK
		      | RXON_FLG_CCK_MSK);
		ctx->staging.flags |= RXON_FLG_SHORT_SLOT_MSK;
	} else {
		/* Copied from iwl_post_associate() */
		if (vif && vif->bss_conf.use_short_slot)
			ctx->staging.flags |= RXON_FLG_SHORT_SLOT_MSK;
		else
			ctx->staging.flags &= ~RXON_FLG_SHORT_SLOT_MSK;

		ctx->staging.flags |= RXON_FLG_BAND_24G_MSK;
		ctx->staging.flags |= RXON_FLG_AUTO_DETECT_MSK;
		ctx->staging.flags &= ~RXON_FLG_CCK_MSK;
	}
}

/*
 * initialize rxon structure with default values from eeprom
 */
void iwl_connection_init_rx_config(struct iwl_priv *priv,
				   struct iwl_rxon_context *ctx)
{
	const struct iwl_channel_info *ch_info;

	memset(&ctx->staging, 0, sizeof(ctx->staging));

	if (!ctx->vif) {
		ctx->staging.dev_type = ctx->unused_devtype;
	} else switch (ctx->vif->type) {
	case NL80211_IFTYPE_AP:
		ctx->staging.dev_type = ctx->ap_devtype;
		break;

	case NL80211_IFTYPE_STATION:
		ctx->staging.dev_type = ctx->station_devtype;
		ctx->staging.filter_flags = RXON_FILTER_ACCEPT_GRP_MSK;
		break;

	case NL80211_IFTYPE_ADHOC:
		ctx->staging.dev_type = ctx->ibss_devtype;
		ctx->staging.flags = RXON_FLG_SHORT_PREAMBLE_MSK;
		ctx->staging.filter_flags = RXON_FILTER_BCON_AWARE_MSK |
						  RXON_FILTER_ACCEPT_GRP_MSK;
		break;

	default:
		IWL_ERR(priv, "Unsupported interface type %d\n",
			ctx->vif->type);
		break;
	}

#if 0
	/* TODO:  Figure out when short_preamble would be set and cache from
	 * that */
	if (!hw_to_local(priv->hw)->short_preamble)
		ctx->staging.flags &= ~RXON_FLG_SHORT_PREAMBLE_MSK;
	else
		ctx->staging.flags |= RXON_FLG_SHORT_PREAMBLE_MSK;
#endif

	ch_info = iwl_get_channel_info(priv, priv->band,
				       le16_to_cpu(ctx->active.channel));

	if (!ch_info)
		ch_info = &priv->channel_info[0];

	ctx->staging.channel = cpu_to_le16(ch_info->channel);
	priv->band = ch_info->band;

	iwl_set_flags_for_band(priv, ctx, priv->band, ctx->vif);

	ctx->staging.ofdm_basic_rates =
	    (IWL_OFDM_RATES_MASK >> IWL_FIRST_OFDM_RATE) & 0xFF;
	ctx->staging.cck_basic_rates =
	    (IWL_CCK_RATES_MASK >> IWL_FIRST_CCK_RATE) & 0xF;

	/* clear both MIX and PURE40 mode flag */
	ctx->staging.flags &= ~(RXON_FLG_CHANNEL_MODE_MIXED |
					RXON_FLG_CHANNEL_MODE_PURE_40);
	if (ctx->vif)
		memcpy(ctx->staging.node_addr, ctx->vif->addr, ETH_ALEN);

	ctx->staging.ofdm_ht_single_stream_basic_rates = 0xff;
	ctx->staging.ofdm_ht_dual_stream_basic_rates = 0xff;
	ctx->staging.ofdm_ht_triple_stream_basic_rates = 0xff;
}

void iwl_set_rate(struct iwl_priv *priv)
{
	const struct ieee80211_supported_band *hw = NULL;
	struct ieee80211_rate *rate;
	struct iwl_rxon_context *ctx;
	int i;

	hw = iwl_get_hw_mode(priv, priv->band);
	if (!hw) {
		IWL_ERR(priv, "Failed to set rate: unable to get hw mode\n");
		return;
	}

	priv->active_rate = 0;

	for (i = 0; i < hw->n_bitrates; i++) {
		rate = &(hw->bitrates[i]);
		if (rate->hw_value < IWL_RATE_COUNT_LEGACY)
			priv->active_rate |= (1 << rate->hw_value);
	}

	IWL_DEBUG_RATE(priv, "Set active_rate = %0x\n", priv->active_rate);

	for_each_context(priv, ctx) {
		ctx->staging.cck_basic_rates =
		    (IWL_CCK_BASIC_RATES_MASK >> IWL_FIRST_CCK_RATE) & 0xF;

		ctx->staging.ofdm_basic_rates =
		   (IWL_OFDM_BASIC_RATES_MASK >> IWL_FIRST_OFDM_RATE) & 0xFF;
	}
}

void iwl_chswitch_done(struct iwl_priv *priv, bool is_success)
{
	/*
	 * MULTI-FIXME
	 * See iwl_mac_channel_switch.
	 */
	struct iwl_rxon_context *ctx = &priv->contexts[IWL_RXON_CTX_BSS];

	if (test_bit(STATUS_EXIT_PENDING, &priv->status))
		return;

	if (test_and_clear_bit(STATUS_CHANNEL_SWITCH_PENDING, &priv->status))
		ieee80211_chswitch_done(ctx->vif, is_success);
}

#ifdef CONFIG_IWLWIFI_DEBUG
void iwl_print_rx_config_cmd(struct iwl_priv *priv,
			     struct iwl_rxon_context *ctx)
{
	struct iwl_rxon_cmd *rxon = &ctx->staging;

	IWL_DEBUG_RADIO(priv, "RX CONFIG:\n");
	iwl_print_hex_dump(priv, IWL_DL_RADIO, (u8 *) rxon, sizeof(*rxon));
	IWL_DEBUG_RADIO(priv, "u16 channel: 0x%x\n", le16_to_cpu(rxon->channel));
	IWL_DEBUG_RADIO(priv, "u32 flags: 0x%08X\n", le32_to_cpu(rxon->flags));
	IWL_DEBUG_RADIO(priv, "u32 filter_flags: 0x%08x\n",
			le32_to_cpu(rxon->filter_flags));
	IWL_DEBUG_RADIO(priv, "u8 dev_type: 0x%x\n", rxon->dev_type);
	IWL_DEBUG_RADIO(priv, "u8 ofdm_basic_rates: 0x%02x\n",
			rxon->ofdm_basic_rates);
	IWL_DEBUG_RADIO(priv, "u8 cck_basic_rates: 0x%02x\n", rxon->cck_basic_rates);
	IWL_DEBUG_RADIO(priv, "u8[6] node_addr: %pM\n", rxon->node_addr);
	IWL_DEBUG_RADIO(priv, "u8[6] bssid_addr: %pM\n", rxon->bssid_addr);
	IWL_DEBUG_RADIO(priv, "u16 assoc_id: 0x%x\n", le16_to_cpu(rxon->assoc_id));
}
#endif

static void iwlagn_abort_notification_waits(struct iwl_priv *priv)
{
	unsigned long flags;
	struct iwl_notification_wait *wait_entry;

	spin_lock_irqsave(&priv->_agn.notif_wait_lock, flags);
	list_for_each_entry(wait_entry, &priv->_agn.notif_waits, list)
		wait_entry->aborted = true;
	spin_unlock_irqrestore(&priv->_agn.notif_wait_lock, flags);

	wake_up_all(&priv->_agn.notif_waitq);
}

void iwlagn_fw_error(struct iwl_priv *priv, bool ondemand)
{
	unsigned int reload_msec;
	unsigned long reload_jiffies;

	/* Set the FW error flag -- cleared on iwl_down */
	set_bit(STATUS_FW_ERROR, &priv->status);

	/* Cancel currently queued command. */
	clear_bit(STATUS_HCMD_ACTIVE, &priv->status);

	iwlagn_abort_notification_waits(priv);

	/* Keep the restart process from trying to send host
	 * commands by clearing the ready bit */
	clear_bit(STATUS_READY, &priv->status);

	wake_up(&priv->wait_command_queue);

	if (!ondemand) {
		/*
		 * If firmware keep reloading, then it indicate something
		 * serious wrong and firmware having problem to recover
		 * from it. Instead of keep trying which will fill the syslog
		 * and hang the system, let's just stop it
		 */
		reload_jiffies = jiffies;
		reload_msec = jiffies_to_msecs((long) reload_jiffies -
					(long) priv->reload_jiffies);
		priv->reload_jiffies = reload_jiffies;
		if (reload_msec <= IWL_MIN_RELOAD_DURATION) {
			priv->reload_count++;
			if (priv->reload_count >= IWL_MAX_CONTINUE_RELOAD_CNT) {
				IWL_ERR(priv, "BUG_ON, Stop restarting\n");
				return;
			}
		} else
			priv->reload_count = 0;
	}

	if (!test_bit(STATUS_EXIT_PENDING, &priv->status)) {
		if (iwlagn_mod_params.restart_fw) {
			IWL_DEBUG(priv, IWL_DL_FW_ERRORS,
				  "Restarting adapter due to uCode error.\n");
			queue_work(priv->workqueue, &priv->restart);
		} else
			IWL_DEBUG(priv, IWL_DL_FW_ERRORS,
				  "Detected FW error, but not restarting\n");
	}
}

/**
 * iwl_irq_handle_error - called for HW or SW error interrupt from card
 */
void iwl_irq_handle_error(struct iwl_priv *priv)
{
	/* W/A for WiFi/WiMAX coex and WiMAX own the RF */
	if (priv->cfg->internal_wimax_coex &&
	    (!(iwl_read_prph(priv, APMG_CLK_CTRL_REG) &
			APMS_CLK_VAL_MRB_FUNC_MODE) ||
	     (iwl_read_prph(priv, APMG_PS_CTRL_REG) &
			APMG_PS_CTRL_VAL_RESET_REQ))) {
		/*
		 * Keep the restart process from trying to send host
		 * commands by clearing the ready bit.
		 */
		clear_bit(STATUS_READY, &priv->status);
		clear_bit(STATUS_HCMD_ACTIVE, &priv->status);
		wake_up(&priv->wait_command_queue);
		IWL_ERR(priv, "RF is used by WiMAX\n");
		return;
	}

	IWL_ERR(priv, "Loaded firmware version: %s\n",
		priv->hw->wiphy->fw_version);

	iwl_dump_nic_error_log(priv);
	iwl_dump_csr(priv);
	iwl_dump_fh(priv, NULL, false);
	iwl_dump_nic_event_log(priv, false, NULL, false);
#ifdef CONFIG_IWLWIFI_DEBUG
	if (iwl_get_debug_level(priv) & IWL_DL_FW_ERRORS)
		iwl_print_rx_config_cmd(priv,
					&priv->contexts[IWL_RXON_CTX_BSS]);
#endif

	iwlagn_fw_error(priv, false);
}

static int iwl_apm_stop_master(struct iwl_priv *priv)
{
	int ret = 0;

	/* stop device's busmaster DMA activity */
	iwl_set_bit(priv, CSR_RESET, CSR_RESET_REG_FLAG_STOP_MASTER);

	ret = iwl_poll_bit(priv, CSR_RESET, CSR_RESET_REG_FLAG_MASTER_DISABLED,
			CSR_RESET_REG_FLAG_MASTER_DISABLED, 100);
	if (ret)
		IWL_WARN(priv, "Master Disable Timed Out, 100 usec\n");

	IWL_DEBUG_INFO(priv, "stop master\n");

	return ret;
}

void iwl_apm_stop(struct iwl_priv *priv)
{
	IWL_DEBUG_INFO(priv, "Stop card, put in low power state\n");

	clear_bit(STATUS_DEVICE_ENABLED, &priv->status);

	/* Stop device's DMA activity */
	iwl_apm_stop_master(priv);

	/* Reset the entire device */
	iwl_set_bit(priv, CSR_RESET, CSR_RESET_REG_FLAG_SW_RESET);

	udelay(10);

	/*
	 * Clear "initialization complete" bit to move adapter from
	 * D0A* (powered-up Active) --> D0U* (Uninitialized) state.
	 */
	iwl_clear_bit(priv, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);
}


/*
 * Start up NIC's basic functionality after it has been reset
 * (e.g. after platform boot, or shutdown via iwl_apm_stop())
 * NOTE:  This does not load uCode nor start the embedded processor
 */
int iwl_apm_init(struct iwl_priv *priv)
{
	int ret = 0;
	u16 lctl;

	IWL_DEBUG_INFO(priv, "Init card's basic functions\n");

	/*
	 * Use "set_bit" below rather than "write", to preserve any hardware
	 * bits already set by default after reset.
	 */

	/* Disable L0S exit timer (platform NMI Work/Around) */
	iwl_set_bit(priv, CSR_GIO_CHICKEN_BITS,
			  CSR_GIO_CHICKEN_BITS_REG_BIT_DIS_L0S_EXIT_TIMER);

	/*
	 * Disable L0s without affecting L1;
	 *  don't wait for ICH L0s (ICH bug W/A)
	 */
	iwl_set_bit(priv, CSR_GIO_CHICKEN_BITS,
			  CSR_GIO_CHICKEN_BITS_REG_BIT_L1A_NO_L0S_RX);

	/* Set FH wait threshold to maximum (HW error during stress W/A) */
	iwl_set_bit(priv, CSR_DBG_HPET_MEM_REG, CSR_DBG_HPET_MEM_REG_VAL);

	/*
	 * Enable HAP INTA (interrupt from management bus) to
	 * wake device's PCI Express link L1a -> L0s
	 */
	iwl_set_bit(priv, CSR_HW_IF_CONFIG_REG,
				    CSR_HW_IF_CONFIG_REG_BIT_HAP_WAKE_L1A);

	/*
	 * HW bug W/A for instability in PCIe bus L0->L0S->L1 transition.
	 * Check if BIOS (or OS) enabled L1-ASPM on this device.
	 * If so (likely), disable L0S, so device moves directly L0->L1;
	 *    costs negligible amount of power savings.
	 * If not (unlikely), enable L0S, so there is at least some
	 *    power savings, even without L1.
	 */
	lctl = iwl_pcie_link_ctl(priv);
	if ((lctl & PCI_CFG_LINK_CTRL_VAL_L1_EN) ==
				PCI_CFG_LINK_CTRL_VAL_L1_EN) {
		/* L1-ASPM enabled; disable(!) L0S  */
		iwl_set_bit(priv, CSR_GIO_REG,
				CSR_GIO_REG_VAL_L0S_ENABLED);
		IWL_DEBUG_POWER(priv, "L1 Enabled; Disabling L0S\n");
	} else {
		/* L1-ASPM disabled; enable(!) L0S */
		iwl_clear_bit(priv, CSR_GIO_REG,
				CSR_GIO_REG_VAL_L0S_ENABLED);
		IWL_DEBUG_POWER(priv, "L1 Disabled; Enabling L0S\n");
	}

	/* Configure analog phase-lock-loop before activating to D0A */
	if (priv->cfg->base_params->pll_cfg_val)
		iwl_set_bit(priv, CSR_ANA_PLL_CFG,
			    priv->cfg->base_params->pll_cfg_val);

	/*
	 * Set "initialization complete" bit to move adapter from
	 * D0U* --> D0A* (powered-up active) state.
	 */
	iwl_set_bit(priv, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	/*
	 * Wait for clock stabilization; once stabilized, access to
	 * device-internal resources is supported, e.g. iwl_write_prph()
	 * and accesses to uCode SRAM.
	 */
	ret = iwl_poll_bit(priv, CSR_GP_CNTRL,
			CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
			CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY, 25000);
	if (ret < 0) {
		IWL_DEBUG_INFO(priv, "Failed to init the card\n");
		goto out;
	}

	/*
	 * Enable DMA clock and wait for it to stabilize.
	 *
	 * Write to "CLK_EN_REG"; "1" bits enable clocks, while "0" bits
	 * do not disable clocks.  This preserves any hardware bits already
	 * set by default in "CLK_CTRL_REG" after reset.
	 */
	iwl_write_prph(priv, APMG_CLK_EN_REG, APMG_CLK_VAL_DMA_CLK_RQT);
	udelay(20);

	/* Disable L1-Active */
	iwl_set_bits_prph(priv, APMG_PCIDEV_STT_REG,
			  APMG_PCIDEV_STT_VAL_L1_ACT_DIS);

	set_bit(STATUS_DEVICE_ENABLED, &priv->status);

out:
	return ret;
}


int iwl_set_tx_power(struct iwl_priv *priv, s8 tx_power, bool force)
{
	int ret;
	s8 prev_tx_power;
	bool defer;
	struct iwl_rxon_context *ctx = &priv->contexts[IWL_RXON_CTX_BSS];

	lockdep_assert_held(&priv->mutex);

	if (priv->tx_power_user_lmt == tx_power && !force)
		return 0;

	if (!priv->cfg->ops->lib->send_tx_power)
		return -EOPNOTSUPP;

	if (tx_power < IWLAGN_TX_POWER_TARGET_POWER_MIN) {
		IWL_WARN(priv,
			 "Requested user TXPOWER %d below lower limit %d.\n",
			 tx_power,
			 IWLAGN_TX_POWER_TARGET_POWER_MIN);
		return -EINVAL;
	}

	if (tx_power > priv->tx_power_device_lmt) {
		IWL_WARN(priv,
			"Requested user TXPOWER %d above upper limit %d.\n",
			 tx_power, priv->tx_power_device_lmt);
		return -EINVAL;
	}

	if (!iwl_is_ready_rf(priv))
		return -EIO;

	/* scan complete and commit_rxon use tx_power_next value,
	 * it always need to be updated for newest request */
	priv->tx_power_next = tx_power;

	/* do not set tx power when scanning or channel changing */
	defer = test_bit(STATUS_SCANNING, &priv->status) ||
		memcmp(&ctx->active, &ctx->staging, sizeof(ctx->staging));
	if (defer && !force) {
		IWL_DEBUG_INFO(priv, "Deferring tx power set\n");
		return 0;
	}

	prev_tx_power = priv->tx_power_user_lmt;
	priv->tx_power_user_lmt = tx_power;

	ret = priv->cfg->ops->lib->send_tx_power(priv);

	/* if fail to set tx_power, restore the orig. tx power */
	if (ret) {
		priv->tx_power_user_lmt = prev_tx_power;
		priv->tx_power_next = prev_tx_power;
	}
	return ret;
}

void iwl_send_bt_config(struct iwl_priv *priv)
{
	struct iwl_bt_cmd bt_cmd = {
		.lead_time = BT_LEAD_TIME_DEF,
		.max_kill = BT_MAX_KILL_DEF,
		.kill_ack_mask = 0,
		.kill_cts_mask = 0,
	};

	if (!bt_coex_active)
		bt_cmd.flags = BT_COEX_DISABLE;
	else
		bt_cmd.flags = BT_COEX_ENABLE;

	priv->bt_enable_flag = bt_cmd.flags;
	IWL_DEBUG_INFO(priv, "BT coex %s\n",
		(bt_cmd.flags == BT_COEX_DISABLE) ? "disable" : "active");

	if (iwl_send_cmd_pdu(priv, REPLY_BT_CONFIG,
			     sizeof(struct iwl_bt_cmd), &bt_cmd))
		IWL_ERR(priv, "failed to send BT Coex Config\n");
}

int iwl_send_statistics_request(struct iwl_priv *priv, u8 flags, bool clear)
{
	struct iwl_statistics_cmd statistics_cmd = {
		.configuration_flags =
			clear ? IWL_STATS_CONF_CLEAR_STATS : 0,
	};

	if (flags & CMD_ASYNC)
		return iwl_send_cmd_pdu_async(priv, REPLY_STATISTICS_CMD,
					       sizeof(struct iwl_statistics_cmd),
					       &statistics_cmd, NULL);
	else
		return iwl_send_cmd_pdu(priv, REPLY_STATISTICS_CMD,
					sizeof(struct iwl_statistics_cmd),
					&statistics_cmd);
}

void iwl_clear_isr_stats(struct iwl_priv *priv)
{
	memset(&priv->isr_stats, 0, sizeof(priv->isr_stats));
}

int iwl_mac_conf_tx(struct ieee80211_hw *hw, u16 queue,
			   const struct ieee80211_tx_queue_params *params)
{
	struct iwl_priv *priv = hw->priv;
	struct iwl_rxon_context *ctx;
	unsigned long flags;
	int q;

	IWL_DEBUG_MAC80211(priv, "enter\n");

	if (!iwl_is_ready_rf(priv)) {
		IWL_DEBUG_MAC80211(priv, "leave - RF not ready\n");
		return -EIO;
	}

	if (queue >= AC_NUM) {
		IWL_DEBUG_MAC80211(priv, "leave - queue >= AC_NUM %d\n", queue);
		return 0;
	}

	q = AC_NUM - 1 - queue;

	spin_lock_irqsave(&priv->lock, flags);

	/*
	 * MULTI-FIXME
	 * This may need to be done per interface in nl80211/cfg80211/mac80211.
	 */
	for_each_context(priv, ctx) {
		ctx->qos_data.def_qos_parm.ac[q].cw_min =
			cpu_to_le16(params->cw_min);
		ctx->qos_data.def_qos_parm.ac[q].cw_max =
			cpu_to_le16(params->cw_max);
		ctx->qos_data.def_qos_parm.ac[q].aifsn = params->aifs;
		ctx->qos_data.def_qos_parm.ac[q].edca_txop =
				cpu_to_le16((params->txop * 32));

		ctx->qos_data.def_qos_parm.ac[q].reserved1 = 0;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	IWL_DEBUG_MAC80211(priv, "leave\n");
	return 0;
}

int iwl_mac_tx_last_beacon(struct ieee80211_hw *hw)
{
	struct iwl_priv *priv = hw->priv;

	return priv->ibss_manager == IWL_IBSS_MANAGER;
}

static int iwl_set_mode(struct iwl_priv *priv, struct iwl_rxon_context *ctx)
{
	iwl_connection_init_rx_config(priv, ctx);

	if (priv->cfg->ops->hcmd->set_rxon_chain)
		priv->cfg->ops->hcmd->set_rxon_chain(priv, ctx);

	return iwlcore_commit_rxon(priv, ctx);
}

static int iwl_setup_interface(struct iwl_priv *priv,
			       struct iwl_rxon_context *ctx)
{
	struct ieee80211_vif *vif = ctx->vif;
	int err;

	lockdep_assert_held(&priv->mutex);

	/*
	 * This variable will be correct only when there's just
	 * a single context, but all code using it is for hardware
	 * that supports only one context.
	 */
	priv->iw_mode = vif->type;

	ctx->is_active = true;

	err = iwl_set_mode(priv, ctx);
	if (err) {
		if (!ctx->always_active)
			ctx->is_active = false;
		return err;
	}

	if (priv->cfg->bt_params && priv->cfg->bt_params->advanced_bt_coexist &&
	    vif->type == NL80211_IFTYPE_ADHOC) {
		/*
		 * pretend to have high BT traffic as long as we
		 * are operating in IBSS mode, as this will cause
		 * the rate scaling etc. to behave as intended.
		 */
		priv->bt_traffic_load = IWL_BT_COEX_TRAFFIC_LOAD_HIGH;
	}

	return 0;
}

int iwl_mac_add_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct iwl_priv *priv = hw->priv;
	struct iwl_vif_priv *vif_priv = (void *)vif->drv_priv;
	struct iwl_rxon_context *tmp, *ctx = NULL;
	int err;
	enum nl80211_iftype viftype = ieee80211_vif_type_p2p(vif);

	IWL_DEBUG_MAC80211(priv, "enter: type %d, addr %pM\n",
			   viftype, vif->addr);

	mutex_lock(&priv->mutex);

	if (!iwl_is_ready_rf(priv)) {
		IWL_WARN(priv, "Try to add interface when device not ready\n");
		err = -EINVAL;
		goto out;
	}

	for_each_context(priv, tmp) {
		u32 possible_modes =
			tmp->interface_modes | tmp->exclusive_interface_modes;

		if (tmp->vif) {
			/* check if this busy context is exclusive */
			if (tmp->exclusive_interface_modes &
						BIT(tmp->vif->type)) {
				err = -EINVAL;
				goto out;
			}
			continue;
		}

		if (!(possible_modes & BIT(viftype)))
			continue;

		/* have maybe usable context w/o interface */
		ctx = tmp;
		break;
	}

	if (!ctx) {
		err = -EOPNOTSUPP;
		goto out;
	}

	vif_priv->ctx = ctx;
	ctx->vif = vif;

	err = iwl_setup_interface(priv, ctx);
	if (!err)
		goto out;

	ctx->vif = NULL;
	priv->iw_mode = NL80211_IFTYPE_STATION;
 out:
	mutex_unlock(&priv->mutex);

	IWL_DEBUG_MAC80211(priv, "leave\n");
	return err;
}

static void iwl_teardown_interface(struct iwl_priv *priv,
				   struct ieee80211_vif *vif,
				   bool mode_change)
{
	struct iwl_rxon_context *ctx = iwl_rxon_ctx_from_vif(vif);

	lockdep_assert_held(&priv->mutex);

	if (priv->scan_vif == vif) {
		iwl_scan_cancel_timeout(priv, 200);
		iwl_force_scan_end(priv);
	}

	if (!mode_change) {
		iwl_set_mode(priv, ctx);
		if (!ctx->always_active)
			ctx->is_active = false;
	}

	/*
	 * When removing the IBSS interface, overwrite the
	 * BT traffic load with the stored one from the last
	 * notification, if any. If this is a device that
	 * doesn't implement this, this has no effect since
	 * both values are the same and zero.
	 */
	if (vif->type == NL80211_IFTYPE_ADHOC)
		priv->bt_traffic_load = priv->last_bt_traffic_load;
}

void iwl_mac_remove_interface(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif)
{
	struct iwl_priv *priv = hw->priv;
	struct iwl_rxon_context *ctx = iwl_rxon_ctx_from_vif(vif);

	IWL_DEBUG_MAC80211(priv, "enter\n");

	mutex_lock(&priv->mutex);

	WARN_ON(ctx->vif != vif);
	ctx->vif = NULL;

	iwl_teardown_interface(priv, vif, false);

	mutex_unlock(&priv->mutex);

	IWL_DEBUG_MAC80211(priv, "leave\n");

}

int iwl_alloc_txq_mem(struct iwl_priv *priv)
{
	if (!priv->txq)
		priv->txq = kzalloc(
			sizeof(struct iwl_tx_queue) *
				priv->cfg->base_params->num_of_queues,
			GFP_KERNEL);
	if (!priv->txq) {
		IWL_ERR(priv, "Not enough memory for txq\n");
		return -ENOMEM;
	}
	return 0;
}

void iwl_free_txq_mem(struct iwl_priv *priv)
{
	kfree(priv->txq);
	priv->txq = NULL;
}

#ifdef CONFIG_IWLWIFI_DEBUGFS

#define IWL_TRAFFIC_DUMP_SIZE	(IWL_TRAFFIC_ENTRY_SIZE * IWL_TRAFFIC_ENTRIES)

void iwl_reset_traffic_log(struct iwl_priv *priv)
{
	priv->tx_traffic_idx = 0;
	priv->rx_traffic_idx = 0;
	if (priv->tx_traffic)
		memset(priv->tx_traffic, 0, IWL_TRAFFIC_DUMP_SIZE);
	if (priv->rx_traffic)
		memset(priv->rx_traffic, 0, IWL_TRAFFIC_DUMP_SIZE);
}

int iwl_alloc_traffic_mem(struct iwl_priv *priv)
{
	u32 traffic_size = IWL_TRAFFIC_DUMP_SIZE;

	if (iwl_debug_level & IWL_DL_TX) {
		if (!priv->tx_traffic) {
			priv->tx_traffic =
				kzalloc(traffic_size, GFP_KERNEL);
			if (!priv->tx_traffic)
				return -ENOMEM;
		}
	}
	if (iwl_debug_level & IWL_DL_RX) {
		if (!priv->rx_traffic) {
			priv->rx_traffic =
				kzalloc(traffic_size, GFP_KERNEL);
			if (!priv->rx_traffic)
				return -ENOMEM;
		}
	}
	iwl_reset_traffic_log(priv);
	return 0;
}

void iwl_free_traffic_mem(struct iwl_priv *priv)
{
	kfree(priv->tx_traffic);
	priv->tx_traffic = NULL;

	kfree(priv->rx_traffic);
	priv->rx_traffic = NULL;
}

void iwl_dbg_log_tx_data_frame(struct iwl_priv *priv,
		      u16 length, struct ieee80211_hdr *header)
{
	__le16 fc;
	u16 len;

	if (likely(!(iwl_debug_level & IWL_DL_TX)))
		return;

	if (!priv->tx_traffic)
		return;

	fc = header->frame_control;
	if (ieee80211_is_data(fc)) {
		len = (length > IWL_TRAFFIC_ENTRY_SIZE)
		       ? IWL_TRAFFIC_ENTRY_SIZE : length;
		memcpy((priv->tx_traffic +
		       (priv->tx_traffic_idx * IWL_TRAFFIC_ENTRY_SIZE)),
		       header, len);
		priv->tx_traffic_idx =
			(priv->tx_traffic_idx + 1) % IWL_TRAFFIC_ENTRIES;
	}
}

void iwl_dbg_log_rx_data_frame(struct iwl_priv *priv,
		      u16 length, struct ieee80211_hdr *header)
{
	__le16 fc;
	u16 len;

	if (likely(!(iwl_debug_level & IWL_DL_RX)))
		return;

	if (!priv->rx_traffic)
		return;

	fc = header->frame_control;
	if (ieee80211_is_data(fc)) {
		len = (length > IWL_TRAFFIC_ENTRY_SIZE)
		       ? IWL_TRAFFIC_ENTRY_SIZE : length;
		memcpy((priv->rx_traffic +
		       (priv->rx_traffic_idx * IWL_TRAFFIC_ENTRY_SIZE)),
		       header, len);
		priv->rx_traffic_idx =
			(priv->rx_traffic_idx + 1) % IWL_TRAFFIC_ENTRIES;
	}
}

const char *get_mgmt_string(int cmd)
{
	switch (cmd) {
		IWL_CMD(MANAGEMENT_ASSOC_REQ);
		IWL_CMD(MANAGEMENT_ASSOC_RESP);
		IWL_CMD(MANAGEMENT_REASSOC_REQ);
		IWL_CMD(MANAGEMENT_REASSOC_RESP);
		IWL_CMD(MANAGEMENT_PROBE_REQ);
		IWL_CMD(MANAGEMENT_PROBE_RESP);
		IWL_CMD(MANAGEMENT_BEACON);
		IWL_CMD(MANAGEMENT_ATIM);
		IWL_CMD(MANAGEMENT_DISASSOC);
		IWL_CMD(MANAGEMENT_AUTH);
		IWL_CMD(MANAGEMENT_DEAUTH);
		IWL_CMD(MANAGEMENT_ACTION);
	default:
		return "UNKNOWN";

	}
}

const char *get_ctrl_string(int cmd)
{
	switch (cmd) {
		IWL_CMD(CONTROL_BACK_REQ);
		IWL_CMD(CONTROL_BACK);
		IWL_CMD(CONTROL_PSPOLL);
		IWL_CMD(CONTROL_RTS);
		IWL_CMD(CONTROL_CTS);
		IWL_CMD(CONTROL_ACK);
		IWL_CMD(CONTROL_CFEND);
		IWL_CMD(CONTROL_CFENDACK);
	default:
		return "UNKNOWN";

	}
}

void iwl_clear_traffic_stats(struct iwl_priv *priv)
{
	memset(&priv->tx_stats, 0, sizeof(struct traffic_stats));
	memset(&priv->rx_stats, 0, sizeof(struct traffic_stats));
}

/*
 * if CONFIG_IWLWIFI_DEBUGFS defined, iwl_update_stats function will
 * record all the MGMT, CTRL and DATA pkt for both TX and Rx pass.
 * Use debugFs to display the rx/rx_statistics
 * if CONFIG_IWLWIFI_DEBUGFS not being defined, then no MGMT and CTRL
 * information will be recorded, but DATA pkt still will be recorded
 * for the reason of iwl_led.c need to control the led blinking based on
 * number of tx and rx data.
 *
 */
void iwl_update_stats(struct iwl_priv *priv, bool is_tx, __le16 fc, u16 len)
{
	struct traffic_stats	*stats;

	if (is_tx)
		stats = &priv->tx_stats;
	else
		stats = &priv->rx_stats;

	if (ieee80211_is_mgmt(fc)) {
		switch (fc & cpu_to_le16(IEEE80211_FCTL_STYPE)) {
		case cpu_to_le16(IEEE80211_STYPE_ASSOC_REQ):
			stats->mgmt[MANAGEMENT_ASSOC_REQ]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_ASSOC_RESP):
			stats->mgmt[MANAGEMENT_ASSOC_RESP]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_REASSOC_REQ):
			stats->mgmt[MANAGEMENT_REASSOC_REQ]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_REASSOC_RESP):
			stats->mgmt[MANAGEMENT_REASSOC_RESP]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_PROBE_REQ):
			stats->mgmt[MANAGEMENT_PROBE_REQ]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_PROBE_RESP):
			stats->mgmt[MANAGEMENT_PROBE_RESP]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_BEACON):
			stats->mgmt[MANAGEMENT_BEACON]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_ATIM):
			stats->mgmt[MANAGEMENT_ATIM]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_DISASSOC):
			stats->mgmt[MANAGEMENT_DISASSOC]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_AUTH):
			stats->mgmt[MANAGEMENT_AUTH]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_DEAUTH):
			stats->mgmt[MANAGEMENT_DEAUTH]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_ACTION):
			stats->mgmt[MANAGEMENT_ACTION]++;
			break;
		}
	} else if (ieee80211_is_ctl(fc)) {
		switch (fc & cpu_to_le16(IEEE80211_FCTL_STYPE)) {
		case cpu_to_le16(IEEE80211_STYPE_BACK_REQ):
			stats->ctrl[CONTROL_BACK_REQ]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_BACK):
			stats->ctrl[CONTROL_BACK]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_PSPOLL):
			stats->ctrl[CONTROL_PSPOLL]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_RTS):
			stats->ctrl[CONTROL_RTS]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_CTS):
			stats->ctrl[CONTROL_CTS]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_ACK):
			stats->ctrl[CONTROL_ACK]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_CFEND):
			stats->ctrl[CONTROL_CFEND]++;
			break;
		case cpu_to_le16(IEEE80211_STYPE_CFENDACK):
			stats->ctrl[CONTROL_CFENDACK]++;
			break;
		}
	} else {
		/* data */
		stats->data_cnt++;
		stats->data_bytes += len;
	}
}
#endif

static void iwl_force_rf_reset(struct iwl_priv *priv)
{
	if (test_bit(STATUS_EXIT_PENDING, &priv->status))
		return;

	if (!iwl_is_any_associated(priv)) {
		IWL_DEBUG_SCAN(priv, "force reset rejected: not associated\n");
		return;
	}
	/*
	 * There is no easy and better way to force reset the radio,
	 * the only known method is switching channel which will force to
	 * reset and tune the radio.
	 * Use internal short scan (single channel) operation to should
	 * achieve this objective.
	 * Driver should reset the radio when number of consecutive missed
	 * beacon, or any other uCode error condition detected.
	 */
	IWL_DEBUG_INFO(priv, "perform radio reset.\n");
	iwl_internal_short_hw_scan(priv);
}


int iwl_force_reset(struct iwl_priv *priv, int mode, bool external)
{
	struct iwl_force_reset *force_reset;

	if (test_bit(STATUS_EXIT_PENDING, &priv->status))
		return -EINVAL;

	if (mode >= IWL_MAX_FORCE_RESET) {
		IWL_DEBUG_INFO(priv, "invalid reset request.\n");
		return -EINVAL;
	}
	force_reset = &priv->force_reset[mode];
	force_reset->reset_request_count++;
	if (!external) {
		if (force_reset->last_force_reset_jiffies &&
		    time_after(force_reset->last_force_reset_jiffies +
		    force_reset->reset_duration, jiffies)) {
			IWL_DEBUG_INFO(priv, "force reset rejected\n");
			force_reset->reset_reject_count++;
			return -EAGAIN;
		}
	}
	force_reset->reset_success_count++;
	force_reset->last_force_reset_jiffies = jiffies;
	IWL_DEBUG_INFO(priv, "perform force reset (%d)\n", mode);
	switch (mode) {
	case IWL_RF_RESET:
		iwl_force_rf_reset(priv);
		break;
	case IWL_FW_RESET:
		/*
		 * if the request is from external(ex: debugfs),
		 * then always perform the request in regardless the module
		 * parameter setting
		 * if the request is from internal (uCode error or driver
		 * detect failure), then fw_restart module parameter
		 * need to be check before performing firmware reload
		 */
		if (!external && !iwlagn_mod_params.restart_fw) {
			IWL_DEBUG_INFO(priv, "Cancel firmware reload based on "
				       "module parameter setting\n");
			break;
		}
		IWL_ERR(priv, "On demand firmware reload\n");
		iwlagn_fw_error(priv, true);
		break;
	}
	return 0;
}

int iwl_mac_change_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			     enum nl80211_iftype newtype, bool newp2p)
{
	struct iwl_priv *priv = hw->priv;
	struct iwl_rxon_context *ctx = iwl_rxon_ctx_from_vif(vif);
	struct iwl_rxon_context *bss_ctx = &priv->contexts[IWL_RXON_CTX_BSS];
	struct iwl_rxon_context *tmp;
	enum nl80211_iftype newviftype = newtype;
	u32 interface_modes;
	int err;

	newtype = ieee80211_iftype_p2p(newtype, newp2p);

	mutex_lock(&priv->mutex);

	if (!ctx->vif || !iwl_is_ready_rf(priv)) {
		/*
		 * Huh? But wait ... this can maybe happen when
		 * we're in the middle of a firmware restart!
		 */
		err = -EBUSY;
		goto out;
	}

	interface_modes = ctx->interface_modes | ctx->exclusive_interface_modes;

	if (!(interface_modes & BIT(newtype))) {
		err = -EBUSY;
		goto out;
	}

	/*
	 * Refuse a change that should be done by moving from the PAN
	 * context to the BSS context instead, if the BSS context is
	 * available and can support the new interface type.
	 */
	if (ctx->ctxid == IWL_RXON_CTX_PAN && !bss_ctx->vif &&
	    (bss_ctx->interface_modes & BIT(newtype) ||
	     bss_ctx->exclusive_interface_modes & BIT(newtype))) {
		BUILD_BUG_ON(NUM_IWL_RXON_CTX != 2);
		err = -EBUSY;
		goto out;
	}

	if (ctx->exclusive_interface_modes & BIT(newtype)) {
		for_each_context(priv, tmp) {
			if (ctx == tmp)
				continue;

			if (!tmp->vif)
				continue;

			/*
			 * The current mode switch would be exclusive, but
			 * another context is active ... refuse the switch.
			 */
			err = -EBUSY;
			goto out;
		}
	}

	/* success */
	iwl_teardown_interface(priv, vif, true);
	vif->type = newviftype;
	vif->p2p = newp2p;
	err = iwl_setup_interface(priv, ctx);
	WARN_ON(err);
	/*
	 * We've switched internally, but submitting to the
	 * device may have failed for some reason. Mask this
	 * error, because otherwise mac80211 will not switch
	 * (and set the interface type back) and we'll be
	 * out of sync with it.
	 */
	err = 0;

 out:
	mutex_unlock(&priv->mutex);
	return err;
}

/*
 * On every watchdog tick we check (latest) time stamp. If it does not
 * change during timeout period and queue is not empty we reset firmware.
 */
static int iwl_check_stuck_queue(struct iwl_priv *priv, int cnt)
{
	struct iwl_tx_queue *txq = &priv->txq[cnt];
	struct iwl_queue *q = &txq->q;
	unsigned long timeout;
	int ret;

	if (q->read_ptr == q->write_ptr) {
		txq->time_stamp = jiffies;
		return 0;
	}

	timeout = txq->time_stamp +
		  msecs_to_jiffies(priv->cfg->base_params->wd_timeout);

	if (time_after(jiffies, timeout)) {
		IWL_ERR(priv, "Queue %d stuck for %u ms.\n",
				q->id, priv->cfg->base_params->wd_timeout);
		ret = iwl_force_reset(priv, IWL_FW_RESET, false);
		return (ret == -EAGAIN) ? 0 : 1;
	}

	return 0;
}

/*
 * Making watchdog tick be a quarter of timeout assure we will
 * discover the queue hung between timeout and 1.25*timeout
 */
#define IWL_WD_TICK(timeout) ((timeout) / 4)

/*
 * Watchdog timer callback, we check each tx queue for stuck, if if hung
 * we reset the firmware. If everything is fine just rearm the timer.
 */
void iwl_bg_watchdog(unsigned long data)
{
	struct iwl_priv *priv = (struct iwl_priv *)data;
	int cnt;
	unsigned long timeout;

	if (test_bit(STATUS_EXIT_PENDING, &priv->status))
		return;

	timeout = priv->cfg->base_params->wd_timeout;
	if (timeout == 0)
		return;

	/* monitor and check for stuck cmd queue */
	if (iwl_check_stuck_queue(priv, priv->cmd_queue))
		return;

	/* monitor and check for other stuck queues */
	if (iwl_is_any_associated(priv)) {
		for (cnt = 0; cnt < priv->hw_params.max_txq_num; cnt++) {
			/* skip as we already checked the command queue */
			if (cnt == priv->cmd_queue)
				continue;
			if (iwl_check_stuck_queue(priv, cnt))
				return;
		}
	}

	mod_timer(&priv->watchdog, jiffies +
		  msecs_to_jiffies(IWL_WD_TICK(timeout)));
}

void iwl_setup_watchdog(struct iwl_priv *priv)
{
	unsigned int timeout = priv->cfg->base_params->wd_timeout;

	if (timeout)
		mod_timer(&priv->watchdog,
			  jiffies + msecs_to_jiffies(IWL_WD_TICK(timeout)));
	else
		del_timer(&priv->watchdog);
}

/*
 * extended beacon time format
 * time in usec will be changed into a 32-bit value in extended:internal format
 * the extended part is the beacon counts
 * the internal part is the time in usec within one beacon interval
 */
u32 iwl_usecs_to_beacons(struct iwl_priv *priv, u32 usec, u32 beacon_interval)
{
	u32 quot;
	u32 rem;
	u32 interval = beacon_interval * TIME_UNIT;

	if (!interval || !usec)
		return 0;

	quot = (usec / interval) &
		(iwl_beacon_time_mask_high(priv,
		priv->hw_params.beacon_time_tsf_bits) >>
		priv->hw_params.beacon_time_tsf_bits);
	rem = (usec % interval) & iwl_beacon_time_mask_low(priv,
				   priv->hw_params.beacon_time_tsf_bits);

	return (quot << priv->hw_params.beacon_time_tsf_bits) + rem;
}

/* base is usually what we get from ucode with each received frame,
 * the same as HW timer counter counting down
 */
__le32 iwl_add_beacon_time(struct iwl_priv *priv, u32 base,
			   u32 addon, u32 beacon_interval)
{
	u32 base_low = base & iwl_beacon_time_mask_low(priv,
					priv->hw_params.beacon_time_tsf_bits);
	u32 addon_low = addon & iwl_beacon_time_mask_low(priv,
					priv->hw_params.beacon_time_tsf_bits);
	u32 interval = beacon_interval * TIME_UNIT;
	u32 res = (base & iwl_beacon_time_mask_high(priv,
				priv->hw_params.beacon_time_tsf_bits)) +
				(addon & iwl_beacon_time_mask_high(priv,
				priv->hw_params.beacon_time_tsf_bits));

	if (base_low > addon_low)
		res += base_low - addon_low;
	else if (base_low < addon_low) {
		res += interval + base_low - addon_low;
		res += (1 << priv->hw_params.beacon_time_tsf_bits);
	} else
		res += (1 << priv->hw_params.beacon_time_tsf_bits);

	return cpu_to_le32(res);
}

#ifdef CONFIG_PM

int iwl_pci_suspend(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct iwl_priv *priv = pci_get_drvdata(pdev);

	/*
	 * This function is called when system goes into suspend state
	 * mac80211 will call iwl_mac_stop() from the mac80211 suspend function
	 * first but since iwl_mac_stop() has no knowledge of who the caller is,
	 * it will not call apm_ops.stop() to stop the DMA operation.
	 * Calling apm_ops.stop here to make sure we stop the DMA.
	 */
	iwl_apm_stop(priv);

	return 0;
}

int iwl_pci_resume(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct iwl_priv *priv = pci_get_drvdata(pdev);
	bool hw_rfkill = false;

	/*
	 * We disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state.
	 */
	pci_write_config_byte(pdev, PCI_CFG_RETRY_TIMEOUT, 0x00);

	iwl_enable_interrupts(priv);

	if (!(iwl_read32(priv, CSR_GP_CNTRL) &
				CSR_GP_CNTRL_REG_FLAG_HW_RF_KILL_SW))
		hw_rfkill = true;

	if (hw_rfkill)
		set_bit(STATUS_RF_KILL_HW, &priv->status);
	else
		clear_bit(STATUS_RF_KILL_HW, &priv->status);

	wiphy_rfkill_set_hw_state(priv->hw->wiphy, hw_rfkill);

	return 0;
}

const struct dev_pm_ops iwl_pm_ops = {
	.suspend = iwl_pci_suspend,
	.resume = iwl_pci_resume,
	.freeze = iwl_pci_suspend,
	.thaw = iwl_pci_resume,
	.poweroff = iwl_pci_suspend,
	.restore = iwl_pci_resume,
};

#endif /* CONFIG_PM */
