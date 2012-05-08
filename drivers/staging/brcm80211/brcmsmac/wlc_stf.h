/*
 * Copyright (c) 2010 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _wlc_stf_h_
#define _wlc_stf_h_

extern int wlc_stf_attach(struct wlc_info *wlc);
extern void wlc_stf_detach(struct wlc_info *wlc);

extern void wlc_tempsense_upd(struct wlc_info *wlc);
extern void wlc_stf_ss_algo_channel_get(struct wlc_info *wlc,
					u16 *ss_algo_channel,
					chanspec_t chanspec);
extern int wlc_stf_ss_update(struct wlc_info *wlc, struct wlcband *band);
extern void wlc_stf_phy_txant_upd(struct wlc_info *wlc);
extern int wlc_stf_txchain_set(struct wlc_info *wlc, s32 int_val, bool force);
extern bool wlc_stf_stbc_rx_set(struct wlc_info *wlc, s32 int_val);

extern int wlc_stf_ant_txant_validate(struct wlc_info *wlc, s8 val);
extern void wlc_stf_phy_txant_upd(struct wlc_info *wlc);
extern void wlc_stf_phy_chain_calc(struct wlc_info *wlc);
extern u16 wlc_stf_phytxchain_sel(struct wlc_info *wlc, ratespec_t rspec);
extern u16 wlc_stf_d11hdrs_phyctl_txant(struct wlc_info *wlc, ratespec_t rspec);

#endif				/* _wlc_stf_h_ */
