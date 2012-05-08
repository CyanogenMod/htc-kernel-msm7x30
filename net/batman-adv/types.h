/*
 * Copyright (C) 2007-2011 B.A.T.M.A.N. contributors:
 *
 * Marek Lindner, Simon Wunderlich
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 *
 */



#ifndef _NET_BATMAN_ADV_TYPES_H_
#define _NET_BATMAN_ADV_TYPES_H_

#include "packet.h"
#include "bitarray.h"

#define BAT_HEADER_LEN (sizeof(struct ethhdr) + \
	((sizeof(struct unicast_packet) > sizeof(struct bcast_packet) ? \
	 sizeof(struct unicast_packet) : \
	 sizeof(struct bcast_packet))))


struct hard_iface {
	struct list_head list;
	int16_t if_num;
	char if_status;
	struct net_device *net_dev;
	atomic_t seqno;
	atomic_t frag_seqno;
	unsigned char *packet_buff;
	int packet_len;
	struct kobject *hardif_obj;
	atomic_t refcount;
	struct packet_type batman_adv_ptype;
	struct net_device *soft_iface;
	struct rcu_head rcu;
};

/**
 *	orig_node - structure for orig_list maintaining nodes of mesh
 *	@primary_addr: hosts primary interface address
 *	@last_valid: when last packet from this node was received
 *	@bcast_seqno_reset: time when the broadcast seqno window was reset
 *	@batman_seqno_reset: time when the batman seqno window was reset
 *	@gw_flags: flags related to gateway class
 *	@flags: for now only VIS_SERVER flag
 *	@last_real_seqno: last and best known squence number
 *	@last_ttl: ttl of last received packet
 *	@last_bcast_seqno: last broadcast sequence number received by this host
 *
 *	@candidates: how many candidates are available
 *	@selected: next bonding candidate
 */
struct orig_node {
	uint8_t orig[ETH_ALEN];
	uint8_t primary_addr[ETH_ALEN];
	struct neigh_node __rcu *router; /* rcu protected pointer */
	unsigned long *bcast_own;
	uint8_t *bcast_own_sum;
	unsigned long last_valid;
	unsigned long bcast_seqno_reset;
	unsigned long batman_seqno_reset;
	uint8_t gw_flags;
	uint8_t flags;
	unsigned char *tt_buff;
	int16_t tt_buff_len;
	uint32_t last_real_seqno;
	uint8_t last_ttl;
	unsigned long bcast_bits[NUM_WORDS];
	uint32_t last_bcast_seqno;
	struct hlist_head neigh_list;
	struct list_head frag_list;
	spinlock_t neigh_list_lock; /* protects neigh_list and router */
	atomic_t refcount;
	struct rcu_head rcu;
	struct hlist_node hash_entry;
	struct bat_priv *bat_priv;
	unsigned long last_frag_packet;
	/* ogm_cnt_lock protects: bcast_own, bcast_own_sum,
	 * neigh_node->real_bits, neigh_node->real_packet_count */
	spinlock_t ogm_cnt_lock;
	/* bcast_seqno_lock protects bcast_bits, last_bcast_seqno */
	spinlock_t bcast_seqno_lock;
	atomic_t bond_candidates;
	struct list_head bond_list;
};

struct gw_node {
	struct hlist_node list;
	struct orig_node *orig_node;
	unsigned long deleted;
	atomic_t refcount;
	struct rcu_head rcu;
};

/**
 *	neigh_node
 *	@last_valid: when last packet via this neighbor was received
 */
struct neigh_node {
	struct hlist_node list;
	uint8_t addr[ETH_ALEN];
	uint8_t real_packet_count;
	uint8_t tq_recv[TQ_GLOBAL_WINDOW_SIZE];
	uint8_t tq_index;
	uint8_t tq_avg;
	uint8_t last_ttl;
	struct list_head bonding_list;
	unsigned long last_valid;
	unsigned long real_bits[NUM_WORDS];
	atomic_t refcount;
	struct rcu_head rcu;
	struct orig_node *orig_node;
	struct hard_iface *if_incoming;
	spinlock_t tq_lock;	/* protects: tq_recv, tq_index */
};


struct bat_priv {
	atomic_t mesh_state;
	struct net_device_stats stats;
	atomic_t aggregated_ogms;	/* boolean */
	atomic_t bonding;		/* boolean */
	atomic_t fragmentation;		/* boolean */
	atomic_t vis_mode;		/* VIS_TYPE_* */
	atomic_t gw_mode;		/* GW_MODE_* */
	atomic_t gw_sel_class;		/* uint */
	atomic_t gw_bandwidth;		/* gw bandwidth */
	atomic_t orig_interval;		/* uint */
	atomic_t hop_penalty;		/* uint */
	atomic_t log_level;		/* uint */
	atomic_t bcast_seqno;
	atomic_t bcast_queue_left;
	atomic_t batman_queue_left;
	char num_ifaces;
	struct debug_log *debug_log;
	struct kobject *mesh_obj;
	struct dentry *debug_dir;
	struct hlist_head forw_bat_list;
	struct hlist_head forw_bcast_list;
	struct hlist_head gw_list;
	struct hlist_head softif_neigh_vids;
	struct list_head vis_send_list;
	struct hashtable_t *orig_hash;
	struct hashtable_t *tt_local_hash;
	struct hashtable_t *tt_global_hash;
	struct hashtable_t *vis_hash;
	spinlock_t forw_bat_list_lock; /* protects forw_bat_list */
	spinlock_t forw_bcast_list_lock; /* protects  */
	spinlock_t tt_lhash_lock; /* protects tt_local_hash */
	spinlock_t tt_ghash_lock; /* protects tt_global_hash */
	spinlock_t gw_list_lock; /* protects gw_list and curr_gw */
	spinlock_t vis_hash_lock; /* protects vis_hash */
	spinlock_t vis_list_lock; /* protects vis_info::recv_list */
	spinlock_t softif_neigh_lock; /* protects soft-interface neigh list */
	spinlock_t softif_neigh_vid_lock; /* protects soft-interface vid list */
	int16_t num_local_tt;
	atomic_t tt_local_changed;
	struct delayed_work tt_work;
	struct delayed_work orig_work;
	struct delayed_work vis_work;
	struct gw_node __rcu *curr_gw;  /* rcu protected pointer */
	struct hard_iface __rcu *primary_if;  /* rcu protected pointer */
	struct vis_info *my_vis_info;
};

struct socket_client {
	struct list_head queue_list;
	unsigned int queue_len;
	unsigned char index;
	spinlock_t lock; /* protects queue_list, queue_len, index */
	wait_queue_head_t queue_wait;
	struct bat_priv *bat_priv;
};

struct socket_packet {
	struct list_head list;
	size_t icmp_len;
	struct icmp_packet_rr icmp_packet;
};

struct tt_local_entry {
	uint8_t addr[ETH_ALEN];
	unsigned long last_seen;
	char never_purge;
	struct hlist_node hash_entry;
};

struct tt_global_entry {
	uint8_t addr[ETH_ALEN];
	struct orig_node *orig_node;
	struct hlist_node hash_entry;
};

/**
 *	forw_packet - structure for forw_list maintaining packets to be
 *	              send/forwarded
 */
struct forw_packet {
	struct hlist_node list;
	unsigned long send_time;
	uint8_t own;
	struct sk_buff *skb;
	uint16_t packet_len;
	uint32_t direct_link_flags;
	uint8_t num_packets;
	struct delayed_work delayed_work;
	struct hard_iface *if_incoming;
};

/* While scanning for vis-entries of a particular vis-originator
 * this list collects its interfaces to create a subgraph/cluster
 * out of them later
 */
struct if_list_entry {
	uint8_t addr[ETH_ALEN];
	bool primary;
	struct hlist_node list;
};

struct debug_log {
	char log_buff[LOG_BUF_LEN];
	unsigned long log_start;
	unsigned long log_end;
	spinlock_t lock; /* protects log_buff, log_start and log_end */
	wait_queue_head_t queue_wait;
};

struct frag_packet_list_entry {
	struct list_head list;
	uint16_t seqno;
	struct sk_buff *skb;
};

struct vis_info {
	unsigned long       first_seen;
	struct list_head    recv_list;
			    /* list of server-neighbors we received a vis-packet
			     * from.  we should not reply to them. */
	struct list_head send_list;
	struct kref refcount;
	struct hlist_node hash_entry;
	struct bat_priv *bat_priv;
	/* this packet might be part of the vis send queue. */
	struct sk_buff *skb_packet;
	/* vis_info may follow here*/
} __packed;

struct vis_info_entry {
	uint8_t  src[ETH_ALEN];
	uint8_t  dest[ETH_ALEN];
	uint8_t  quality;	/* quality = 0 client */
} __packed;

struct recvlist_node {
	struct list_head list;
	uint8_t mac[ETH_ALEN];
};

struct softif_neigh_vid {
	struct hlist_node list;
	struct bat_priv *bat_priv;
	short vid;
	atomic_t refcount;
	struct softif_neigh __rcu *softif_neigh;
	struct rcu_head rcu;
	struct hlist_head softif_neigh_list;
};

struct softif_neigh {
	struct hlist_node list;
	uint8_t addr[ETH_ALEN];
	unsigned long last_seen;
	atomic_t refcount;
	struct rcu_head rcu;
};

#endif /* _NET_BATMAN_ADV_TYPES_H_ */
