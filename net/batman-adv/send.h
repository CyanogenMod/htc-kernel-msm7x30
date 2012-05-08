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

#ifndef _NET_BATMAN_ADV_SEND_H_
#define _NET_BATMAN_ADV_SEND_H_

int send_skb_packet(struct sk_buff *skb,
				struct hard_iface *hard_iface,
				uint8_t *dst_addr);
void schedule_own_packet(struct hard_iface *hard_iface);
void schedule_forward_packet(struct orig_node *orig_node,
			     struct ethhdr *ethhdr,
			     struct batman_packet *batman_packet,
			     uint8_t directlink, int tt_buff_len,
			     struct hard_iface *if_outgoing);
int add_bcast_packet_to_list(struct bat_priv *bat_priv, struct sk_buff *skb);
void send_outstanding_bat_packet(struct work_struct *work);
void purge_outstanding_packets(struct bat_priv *bat_priv,
			       struct hard_iface *hard_iface);

#endif /* _NET_BATMAN_ADV_SEND_H_ */
