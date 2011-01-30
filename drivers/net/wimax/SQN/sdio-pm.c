 /*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Dmitriy Chumak <chumakd@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

/*
 * This file includes code that is responsible for
 * power management and LSP notifications handling.
 */


#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/string.h>
#include <linux/byteorder/generic.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>

#include <linux/earlysuspend.h>

#include "sdio-netdev.h"
#include "sdio.h"
#include "sdio-pm.h"
#include "msg.h"
#include "thp.h"
#include "sdio-sqn.h"

#define SDIO_CLAIM_HOST_DEBUG 0

#if SDIO_CLAIM_HOST_DEBUG
#define sqn_sdio_claim_host(func)			\
({							\
	struct mmc_host *h = (func)->card->host;		\
	sqn_pr_info("%s: claim_host+\n", __func__);	\
	sqn_pr_info("%s: BEFORE claim: claimed %d, claim_cnt %d, claimer 0x%p\n" \
	, __func__, h->claimed, h->claim_cnt, h->claimer);	\
	sdio_claim_host((func));			\
	sqn_pr_info("%s: AFTER claim: claimed %d, claim_cnt %d, claimer 0x%p\n" \
	, __func__, h->claimed, h->claim_cnt, h->claimer);	\
	sqn_pr_info("%s: claim_host-\n", __func__);	\
})

#define sqn_sdio_release_host(func)			\
({							\
	struct mmc_host *h = (func)->card->host;		\
	sqn_pr_info("%s: release_host+\n", __func__);	\
	sqn_pr_info("%s: BEFORE release: claimed %d, claim_cnt %d, claimer 0x%p\n" \
	, __func__, h->claimed, h->claim_cnt, h->claimer);	\
	sdio_release_host((func));			\
	sqn_pr_info("%s: AFTER release: claimed %d, claim_cnt %d, claimer 0x%p\n" \
	, __func__, h->claimed, h->claim_cnt, h->claimer);	\
	sqn_pr_info("%s: release_host-\n", __func__);	\
})
#else
#define sqn_sdio_claim_host(func)			\
({							\
	sdio_claim_host((func));			\
})

#define sqn_sdio_release_host(func)			\
({							\
	sdio_release_host((func));			\
})
#endif

#define IGNORE_CARRIER_STATE 1
extern int mmc_wimax_get_hostwakeup_gpio(void);
extern void mmc_wimax_enable_host_wakeup(int on);

enum sqn_thsp_service {
#define	THSP_LSP_SERVICE_BASE		0x10010000
	THSP_GET_MEDIA_CONNECTION_STATE = THSP_LSP_SERVICE_BASE
	, THSP_MEDIA_CONNECTION_STATE_CHANGE
	, THSP_SET_POWER_MODE		/* deprecated */

	, THSP_SET_HOST_POWER_MODE
	, THSP_SET_HOST_POWER_MODE_ACK

	, THSP_SET_FW_POWER_MODE
	, THSP_SET_FW_POWER_MODE_ACK

	, THSP_SQN_STATE_CHANGE
	, THSP_SQN_STATE_CHANGE_REPLY

	, THSP_THP_AVAILABLE
	, THSP_THP_AVAILABLE_REPLY
};


/* Deprecated */
/* enum sqn_power_mode { */
	/* SQN_PM_OPERATIONAL */
	/* , SQN_PM_SHUTDOWN */
	/* , SQN_PM_STANDBY */
/* }; */


enum sqn_host_power_mode {
    LSP_HPM_OPERATIONAL
    , LSP_HPM_ASLEEP
};


enum sqn_fw_power_mode {
    LSP_FPM_OPERATIONAL
    , LSP_FPM_SHUTDOWN
    , LSP_FPM_STANDBY
};


enum sqn_pm_status {
	SQN_PM_STATUS_SUCCES
	, SQN_PM_STATUS_CHANGE_IN_PROGRESS
	, SQN_PM_STATUS_UNKNOWN
};


enum sqn_thsp_media_connection_state {
	THSP_MEDIA_CONNECTION_DISCONNECTED
	, THSP_MEDIA_CONNECTION_CONNECTING
	, THSP_MEDIA_CONNECTION_CONNECTED
	, THSP_MEDIA_CONNECTION_ATTACHED
};


enum sqn_fw_state {
    LSP_SQN_ACTIVE
    , LSP_SQN_IDLE
    , LSP_SQN_DROPPED
    , LSP_SQN_REENTRY
};


enum sqn_thp_available_reply {
    LSP_THPA_ACK
    , LSP_THPA_FINISHED
    , LSP_THPA_EXIT
};


struct sqn_eth_header {
	u8	dst_addr[ETH_ALEN];
	u8	src_addr[ETH_ALEN];
	u16	len;
};


struct sqn_lsp_header {
	u32	id;
	union {
		u32	tid;
		enum sqn_thsp_media_connection_state	media_con_state;

		struct {
			u32				tid;
			enum sqn_host_power_mode	mode;
		} host_power;

		struct {
			u32				tid;
			enum sqn_fw_power_mode		mode;
		} fw_power;

		struct {
			u32				tid;
			enum sqn_fw_state		state;
		} fw_state;

		struct {
			u32				tid;
			enum sqn_thp_available_reply	reply;
		} thp_avl;
	} u;
};


struct sqn_lsp_packet {
    struct sqn_thp_header	thp_header;
    struct sqn_lsp_header	lsp_header;
};


static u8 g_lsp_host_mac[] = {0x00, 0x16, 0x08, 0xff, 0x00, 0x06};
static u8 g_lsp_device_mac[] = {0x00, 0x16, 0x08, 0xff, 0x00, 0x05};


/* TODO: add all global variables to private per-card structure */
static struct sk_buff	*g_last_request_skb = 0;
static spinlock_t	g_last_request_lock = SPIN_LOCK_UNLOCKED;
static u32		g_last_request_pm = 0;


/* TODO: move this to per-card private structure */
DECLARE_WAIT_QUEUE_HEAD(g_card_sleep_waitq);

/* Transaction ID for lsp requests */
static u32		g_tid = 0;
static spinlock_t	g_tid_lock = SPIN_LOCK_UNLOCKED;


static u32 get_current_tid(void)
{
	u32 tid = 0;

	spin_lock(&g_tid_lock);
	tid = g_tid;
	spin_unlock(&g_tid_lock);

	return tid;
}


static u32 get_next_tid(void)
{
	u32 tid = 0;

	spin_lock(&g_tid_lock);
	g_tid += 1;
	tid = g_tid;
	spin_unlock(&g_tid_lock);

	return tid;
}


static void free_last_request(void)
{
	sqn_pr_enter();

	spin_lock(&g_last_request_lock);
	if (0 != g_last_request_skb) {
		dev_kfree_skb_any(g_last_request_skb);
		g_last_request_skb = 0;
	}
	spin_unlock(&g_last_request_lock);

	sqn_pr_leave();
}


static struct sk_buff* lsp_to_skb(struct sqn_lsp_packet *lsp_packet)
{
	struct sqn_eth_header eth_header = {
		.len = htons(sizeof(struct sqn_lsp_packet))
	};

	struct sk_buff *skb =
		__dev_alloc_skb(sizeof(eth_header) + sizeof(struct sqn_lsp_packet)
				, GFP_ATOMIC | GFP_DMA);

	sqn_pr_enter();

	if (0 == skb)
		goto out;

	skb_reserve(skb, 2);

	memcpy(eth_header.dst_addr, g_lsp_device_mac, sizeof(g_lsp_device_mac));
	memcpy(eth_header.src_addr, g_lsp_host_mac, sizeof(g_lsp_host_mac));

	memcpy(skb->data, &eth_header, sizeof(eth_header));
	skb_put(skb, sizeof(eth_header));

	memcpy(skb->data + skb->len, lsp_packet, sizeof(struct sqn_lsp_packet));
	skb_put(skb, sizeof(struct sqn_lsp_packet));

	sqn_pr_leave();

out:
	return skb;

}


static struct sk_buff* construct_lsp_packet(u32 id, u32 param1, u32 param2)
{
	struct sqn_lsp_packet lsp_packet = {
		.thp_header = {
			.transport_version = 1
			, .flags = 1
			, .seq_number = 0
			, .ack_number = 0
		}
		, .lsp_header = {
			.id = htonl(id)
		}
	};

	struct sk_buff *skb = 0;

	sqn_pr_enter();

	switch (id) {
	case THSP_GET_MEDIA_CONNECTION_STATE:
		/* no parameters are needed */
		sqn_pr_dbg("id: THSP_GET_MEDIA_CONNECTION_STATE\n");
		lsp_packet.thp_header.length =
			htons(sizeof(struct sqn_lsp_header) - 4);
		lsp_packet.thp_header.total_length =
			htonl(sizeof(struct sqn_lsp_header) - 4);
		break;
	case THSP_SET_POWER_MODE:
		/* deprecated */
		sqn_pr_dbg("id: THSP_SET_POWER_MODE (deprecated)\n");
		break;
	case THSP_SET_HOST_POWER_MODE:
		lsp_packet.thp_header.length =
			htons(sizeof(struct sqn_lsp_header));
		lsp_packet.thp_header.total_length =
			htonl(sizeof(struct sqn_lsp_header));
		lsp_packet.lsp_header.u.host_power.tid = htonl(get_next_tid());
		lsp_packet.lsp_header.u.host_power.mode = htonl(param1);
		sqn_pr_dbg("id: THSP_SET_HOST_POWER_MODE, tid: 0x%x, mode: %d\n"
			, ntohl(lsp_packet.lsp_header.u.host_power.tid)
			, param1);
		break;
	case THSP_SET_FW_POWER_MODE:
		lsp_packet.thp_header.length =
			htons(sizeof(struct sqn_lsp_header));
		lsp_packet.thp_header.total_length =
			htonl(sizeof(struct sqn_lsp_header));
		lsp_packet.lsp_header.u.fw_power.tid = htonl(get_next_tid());
		lsp_packet.lsp_header.u.fw_power.mode = htonl(param1);
		sqn_pr_dbg("id: THSP_SET_FW_POWER_MODE, tid: 0x%x, mode: %d\n"
			, htonl(lsp_packet.lsp_header.u.fw_power.tid)
			, param1);
		break;
	case THSP_SQN_STATE_CHANGE_REPLY:
		lsp_packet.thp_header.length =
			htons(sizeof(struct sqn_lsp_header) - 4);
		lsp_packet.thp_header.total_length =
			htonl(sizeof(struct sqn_lsp_header) - 4);
		lsp_packet.lsp_header.u.fw_state.tid = htonl(param1);
		sqn_pr_dbg("id: THSP_SQN_STATE_CHANGE_REPLY, tid: %xh\n"
			, param1);
		break;
	case THSP_THP_AVAILABLE_REPLY:
		lsp_packet.thp_header.length =
			htons(sizeof(struct sqn_lsp_header));
		lsp_packet.thp_header.total_length =
			htonl(sizeof(struct sqn_lsp_header));
		lsp_packet.lsp_header.u.thp_avl.tid = htonl(param1);
		lsp_packet.lsp_header.u.thp_avl.reply = htonl(param2);
		sqn_pr_dbg("id: THSP_THP_AVAILABLE_REPLY, tid: 0x%x, reply: %d\n"
			, param1, param2);
		break;
	default:
		sqn_pr_dbg("id: UNKNOWN\n");
	}

	skb = lsp_to_skb(&lsp_packet);

	sqn_pr_leave();

	return skb;
}


int is_lsp_packet(const struct sk_buff *skb)
{
	struct sqn_eth_header *eth_hdr = (struct sqn_eth_header*)skb->data;

	/* sqn_pr_dbg_dump("skb________", skb->data, skb->len); */
	/* sqn_pr_dbg_dump("lsp_dev_mac", g_lsp_device_mac, sizeof(g_lsp_device_mac)); */
	/* sqn_pr_dbg_dump("skb_addr___", eth_hdr->src_addr, sizeof(g_lsp_device_mac)); */

	return !memcmp(eth_hdr->dst_addr, g_lsp_host_mac
		, sizeof(g_lsp_host_mac));
}


static int sqn_set_power_mode_helper(struct sdio_func *func
	, enum sqn_thsp_service command_id, u32 pm)
{
	unsigned long irq_flags = 0;
	struct sqn_sdio_card *card = sdio_get_drvdata(func);

	sqn_pr_enter();

	free_last_request();

	spin_lock(&g_last_request_lock);

	g_last_request_pm = pm;
	g_last_request_skb = construct_lsp_packet(command_id, pm, 0);

	if (0 == g_last_request_skb)
		return 1;

	netif_stop_queue(card->priv->dev);

	/*
	 * We can't call sqn_sdio_tx_skb() from here, because we are not in
	 * process context
	 */
	skb_queue_tail(&card->tx_queue, g_last_request_skb);
	g_last_request_skb = 0;

	spin_unlock(&g_last_request_lock);

	spin_lock_irqsave(&card->priv->drv_lock, irq_flags);
	card->pm_complete = 0;
	spin_unlock_irqrestore(&card->priv->drv_lock, irq_flags);

	wake_up_interruptible(&card->priv->tx_waitq);

	sqn_pr_leave();

	return 0;
}


static int sqn_set_host_power_mode(struct sdio_func *func, enum sqn_host_power_mode pm)
{
	int rv = 0;

	sqn_pr_enter();

	rv = sqn_set_power_mode_helper(func, THSP_SET_HOST_POWER_MODE, pm);

	sqn_pr_leave();

	return rv;
}


static int sqn_set_fw_power_mode(struct sdio_func *func, enum sqn_fw_power_mode pm)
{
	int rv = 0;

	sqn_pr_enter();

	rv = sqn_set_power_mode_helper(func, THSP_SET_FW_POWER_MODE, pm);

	sqn_pr_leave();

	return rv;
}


static void signal_pm_request_completion(struct sqn_private *priv)
{
	struct sqn_sdio_card *card = priv->card;
	unsigned long irq_flags = 0;

	sqn_pr_enter();

	spin_lock_irqsave(&priv->drv_lock, irq_flags);
	card->pm_complete = 1;
	spin_unlock_irqrestore(&priv->drv_lock, irq_flags);

	wake_up_interruptible(&card->pm_waitq);

	sqn_pr_leave();
}


void signal_card_sleep_completion(struct sqn_private *priv)
{
	struct sqn_sdio_card *card = priv->card;
	unsigned long irq_flags = 0;

	sqn_pr_enter();

	spin_lock_irqsave(&priv->drv_lock, irq_flags);
	card->is_card_sleeps = 0;
	spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
	wake_up_interruptible(&g_card_sleep_waitq);
	sqn_pr_dbg("card sleep completion is signaled\n");

	sqn_pr_leave();
}


int sqn_notify_host_sleep(struct sdio_func *func)
{
	int rv = 0;
	unsigned long irq_flags = 0;
	u32 timeout = 0;
	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);

	sqn_pr_enter();

	sqn_card->waiting_pm_notification = 1;

	sqn_pr_info("notify card about host goes to sleep...\n");
	sqn_set_host_power_mode(func, LSP_HPM_ASLEEP);

	timeout = 50;
	sqn_pr_info("wait for completion (timeout %u msec)...\n", timeout);
	rv = wait_event_interruptible_timeout(sqn_card->pm_waitq
		, sqn_card->pm_complete, msecs_to_jiffies(timeout));
	if (-ERESTARTSYS == rv) {
		sqn_pr_warn("got a signal from kernel %d\n", rv);
	} else if (0 == rv) {
		/* a timeout elapsed */
		sqn_pr_warn("timeout elapsed - still no ack from card"
			", assume that card in sleep mode now\n");
		sqn_card->is_card_sleeps = 1;
	} else {
		/* we got an ack from card */
		sqn_pr_info("card in sleep mode now\n");
		sqn_card->is_card_sleeps = 1;
		rv = 0;
	}

	sqn_card->pm_complete = 0;
	sqn_card->waiting_pm_notification = 0;

	sqn_pr_leave();

	return rv;
}


int sqn_notify_host_wakeup(struct sdio_func *func)
{
	int rv = 0;

	sqn_pr_enter();

	rv = sqn_wakeup_fw(func);

	sqn_pr_leave();

	return rv;
};


static void handle_sqn_state_change_msg(struct sqn_private *priv
	, struct sqn_lsp_packet *lsp)
{
	struct sqn_sdio_card *card = priv->card;
	struct sk_buff *skb_reply = 0;
	unsigned long irq_flags = 0;
	const int card_state = ntohl(lsp->lsp_header.u.fw_state.state);

	sqn_pr_enter();

	switch (card_state) {
	case LSP_SQN_ACTIVE:
		sqn_pr_info("card switched to ACTIVE state\n");
		spin_lock_irqsave(&priv->drv_lock, irq_flags);
		card->is_card_sleeps = 0;
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
		break;
	case LSP_SQN_IDLE:
		sqn_pr_info("card switched to IDLE state\n");
		spin_lock_irqsave(&priv->drv_lock, irq_flags);
		card->is_card_sleeps = 1;
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
		break;
	case LSP_SQN_DROPPED:
		sqn_pr_info("card switched to DROPPED state\n");
		spin_lock_irqsave(&priv->drv_lock, irq_flags);
		card->is_card_sleeps = 1;
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
		break;
	case LSP_SQN_REENTRY:
		sqn_pr_info("card switched to REENTRY state\n");
		spin_lock_irqsave(&priv->drv_lock, irq_flags);
		card->is_card_sleeps = 1;
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
		break;
	default:
		sqn_pr_info("card switched to UNSUPPORTED mode %d/0x%x\n"
			, card_state, card_state);
		spin_lock_irqsave(&priv->drv_lock, irq_flags);
		card->is_card_sleeps = 0;
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
		break;
	}
	skb_reply = construct_lsp_packet(THSP_SQN_STATE_CHANGE_REPLY
			, ntohl(lsp->lsp_header.u.thp_avl.tid), 0);
	if (0 != (skb_reply = sqn_sdio_prepare_skb_for_tx(skb_reply)))
		sqn_sdio_tx_skb(card, skb_reply, 0);
	wake_up_interruptible(&g_card_sleep_waitq);

	sqn_pr_leave();
}


static void handle_thp_avl_msg(struct sqn_private *priv
	, struct sqn_lsp_packet *lsp)
{
	struct sqn_sdio_card *card = priv->card;
	struct sk_buff *skb_reply = 0;
	enum sqn_thp_available_reply thp_rpl; 
	unsigned long irq_flags = 0;

	sqn_pr_enter();

	spin_lock_irqsave(&priv->drv_lock, irq_flags);
	/* if (card->is_card_sleeps) { */
	if (priv->is_tx_queue_empty(priv)) {
		sqn_pr_dbg("TX queue empty, thp_rpl=FINISH\n");
		/* sqn_pr_dbg("card was asleep, thp_rpl=FINISH\n"); */
		thp_rpl = LSP_THPA_FINISHED;
		card->is_card_sleeps = 1;
	/* } else if (priv->is_tx_queue_empty(priv)) { */
		/* sqn_pr_dbg("card was not asleep and tx_queue is empty, thp_rpl=FINISHED\n"); */
		/* thp_rpl = LSP_THPA_FINISHED; */
		/* card->is_card_sleeps = 1; */
	} else {
		/* sqn_pr_info("card was not asleep but tx_queue is no empty, thp_rpl=EXIT\n"); */
		sqn_pr_dbg("TX queue not empty, thp_rpl=ACK\n");
		/* sqn_pr_dbg("card was not asleep, thp_rpl=ACK\n"); */
		thp_rpl = LSP_THPA_ACK;
		card->is_card_sleeps = 0;
	}
	spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
	skb_reply = construct_lsp_packet(THSP_THP_AVAILABLE_REPLY
			, ntohl(lsp->lsp_header.u.thp_avl.tid)
			, thp_rpl);
	if (0 != (skb_reply = sqn_sdio_prepare_skb_for_tx(skb_reply)))
		sqn_sdio_tx_skb(card, skb_reply, 0);
	wake_up_interruptible(&g_card_sleep_waitq);
	if (netif_queue_stopped(priv->dev))
		netif_wake_queue(priv->dev);

	sqn_pr_leave();
}


int sqn_handle_lsp_packet(struct sqn_private *priv
	, struct sk_buff *skb)
{
	struct sqn_sdio_card *card = priv->card;
	unsigned long irq_flags = 0;
	struct sqn_lsp_packet *lsp_response = (struct sqn_lsp_packet*)
		((u8*)skb->data + sizeof(struct sqn_eth_header));

	sqn_pr_enter();

	if (!is_lsp_packet(skb)) {
		sqn_pr_dbg("not LSP packet\n");
		sqn_pr_leave();
		return 0;
	}

	sqn_pr_dbg("LSP packet\n");

	switch (ntohl(lsp_response->lsp_header.id)) {
	case THSP_GET_MEDIA_CONNECTION_STATE:
		sqn_pr_dbg("id: THSP_GET_MEDIA_CONNECTION_STATE state=%xh\n"
			, ntohl(lsp_response->lsp_header.u.media_con_state));
		sqn_pr_warn("THSP_GET_MEDIA_CONNECTION_STATE not implemented\n");
		break;
	case THSP_MEDIA_CONNECTION_STATE_CHANGE:
		sqn_pr_dbg("id: THSP_MEDIA_CONNECTION_STATE_CHANGE state=%xh\n"
			, ntohl(lsp_response->lsp_header.u.media_con_state));
		if (THSP_MEDIA_CONNECTION_ATTACHED
			== ntohl(lsp_response->lsp_header.u.media_con_state))
		{
			
#if IGNORE_CARRIER_STATE
            /* netif_carrier_on(priv->dev); */
            sqn_pr_info("WiMAX carrier PRESENT [ignored]\n");
#else
            netif_carrier_on(priv->dev);
			sqn_pr_info("WiMAX carrier PRESENT\n");
#endif            
		} else {
#if IGNORE_CARRIER_STATE
			/* netif_carrier_off(priv->dev); */
			sqn_pr_info("WiMAX carrier LOST [ignored]\n");
#else
			netif_carrier_off(priv->dev);
			sqn_pr_info("WiMAX carrier LOST\n");
#endif
		}
		break;
	case THSP_SET_HOST_POWER_MODE_ACK:
		sqn_pr_dbg("id: THSP_SET_HOST_POWER_MODE_ACK tid=0x%x\n"
			, ntohl(lsp_response->lsp_header.u.host_power.tid));
		free_last_request();
		spin_lock_irqsave(&priv->drv_lock, irq_flags);
		card->is_card_sleeps = 1;
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
		signal_pm_request_completion(priv);
		break;
	case THSP_SET_FW_POWER_MODE_ACK:
		sqn_pr_dbg("id: THSP_SET_FW_POWER_MODE_ACK tid=0x%x\n"
			, ntohl(lsp_response->lsp_header.u.fw_power.tid));
		sqn_pr_dbg("THSP_SET_FW_POWER_MODE_ACK not implemented\n");
		break;
	case THSP_SQN_STATE_CHANGE:
		sqn_pr_dbg("id: THSP_SQN_STATE_CHANGE tid=0x%x, state=%xh\n"
			, ntohl(lsp_response->lsp_header.u.fw_state.tid)
			, ntohl(lsp_response->lsp_header.u.fw_state.state));
		handle_sqn_state_change_msg(priv, lsp_response);
		break;
	case THSP_THP_AVAILABLE:
		sqn_pr_dbg("id: THSP_THP_AVAILABLE tid=0x%x, reply=%xh\n"
			, ntohl(lsp_response->lsp_header.u.thp_avl.tid)
			, ntohl(lsp_response->lsp_header.u.thp_avl.reply));
		handle_thp_avl_msg(priv, lsp_response);
		break;
	default:
		sqn_pr_dbg("lsp_id: UNKNOWN=0x%x\n"
			, ntohl(lsp_response->lsp_header.id));
	}

	dev_kfree_skb_any(skb);
	sqn_pr_leave();

	return 1;
}


int sqn_wakeup_fw(struct sdio_func *func)
{
	int rv = 0;
	int ver = 0;
	int counter = 0;
  
	int retry_cnt = 3;
	u32 wakeup_delay = 0;
	unsigned long timeout = msecs_to_jiffies(800);

	unsigned long irq_flags = 0;
	struct sqn_private *priv = ((struct sqn_sdio_card *)sdio_get_drvdata(func))->priv;
	struct sqn_sdio_card *card = priv->card;
	u8 need_to_unlock_wakelock = 0;

	sqn_pr_enter();
	sqn_pr_info("waking up the card...\n");
	
	if (!wake_lock_active(&card->wakelock)) {
		sqn_pr_dbg("lock wake_lock\n");
		wake_lock(&card->wakelock);
		need_to_unlock_wakelock = 1;
	}

retry:
	if (priv->removed)
		goto out;

	sqn_sdio_claim_host(func);

#define  SDIO_CCCR_CCCR_SDIO_VERSION_VALUE	0x11

    wakeup_delay = 2;
	counter = 5;
	do {
		ver = sdio_readb(func, SDIO_CCCR_CCCR_SDIO_VERSION, &rv);
		// To avoid FW sutck in PLLOFF, SDIO isn't able to wake up it.
		mdelay(wakeup_delay);
		--counter;
	} while((rv || ver != SDIO_CCCR_CCCR_SDIO_VERSION_VALUE) && counter > 0);

	if (rv) {
		sqn_pr_err("error when reading SDIO_VERSION\n");
		sqn_sdio_release_host(func);
		goto out;
	} else {
		sqn_pr_dbg("SDIO_VERSION has been read successfully\n");
	}

	sqn_pr_dbg("send wake-up signal to card\n");
	sdio_writeb(func, 1, SQN_SOC_SIGS_LSBS, &rv);
	if (rv)
		sqn_pr_err("error when writing to SQN_SOC_SIGS_LSBS: %d\n", rv);

	sqn_sdio_release_host(func);

	sqn_pr_info("wait for completion (timeout %d msec)...\n"
		, jiffies_to_msecs(timeout));

	rv = wait_event_interruptible_timeout(g_card_sleep_waitq
		, 0 == card->is_card_sleeps || priv->removed, timeout);

	if (priv->removed)
		goto out;

	if (-ERESTARTSYS == rv) {
		sqn_pr_warn("got a signal from kernel %d\n", rv);
	} else if (0 == rv) {
		rv = -1;
		sqn_pr_err("can't wake up the card - timeout elapsed\n");

		if (retry_cnt-- > 0 && card->is_card_sleeps) {
			sqn_pr_info("retry wake up\n");
			goto retry;
    	}
		sqn_pr_info("giving up to wake up the card\n");

		spin_lock_irqsave(&priv->drv_lock, irq_flags);
		card->is_card_sleeps = 0;
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
	} else {
		rv = 0;
		sqn_pr_info("card is waked up successfully\n");
	}

out:
	if (need_to_unlock_wakelock && wake_lock_active(&card->wakelock)) {
		sqn_pr_dbg("wake_lock is active, release it\n");
		wake_unlock(&card->wakelock);
	}

	sqn_pr_leave();
	return rv;
}

extern void mmc_wimax_enable_host_wakeup(int on);

static void sqn_handle_android_early_suspend(struct early_suspend *h)
{
	sqn_pr_enter();
	sqn_pr_info("%s: enter\n", __func__);

	mmc_wimax_enable_host_wakeup(1);

	sqn_pr_info("%s: leave\n", __func__);
	sqn_pr_leave();
}


static void sqn_handle_android_late_resume(struct early_suspend *h)
{
	sqn_pr_enter();
	sqn_pr_info("%s: enter\n", __func__);

	mmc_wimax_enable_host_wakeup(0);

	sqn_pr_info("%s: leave\n", __func__);
	sqn_pr_leave();
}


static struct early_suspend sqn_early_suspend_desc = {
        .level = EARLY_SUSPEND_LEVEL_DISABLE_FB
        , .suspend = sqn_handle_android_early_suspend
        , .resume = sqn_handle_android_late_resume
};


void register_android_earlysuspend(void)
{
	sqn_pr_enter();

	register_early_suspend(&sqn_early_suspend_desc);

	sqn_pr_leave();
}


void unregister_android_earlysuspend(void)
{
	sqn_pr_enter();

	unregister_early_suspend(&sqn_early_suspend_desc);

	sqn_pr_leave();
}
