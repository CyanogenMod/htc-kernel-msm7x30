/*
 * Copyright (c) 2009, Microsoft Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 * Authors:
 *   Haiyang Zhang <haiyangz@microsoft.com>
 *   Hank Janssen  <hjanssen@microsoft.com>
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>

#include "hyperv.h"
#include "hyperv_net.h"


enum rndis_device_state {
	RNDIS_DEV_UNINITIALIZED = 0,
	RNDIS_DEV_INITIALIZING,
	RNDIS_DEV_INITIALIZED,
	RNDIS_DEV_DATAINITIALIZED,
};

struct rndis_device {
	struct netvsc_device *net_dev;

	enum rndis_device_state state;
	u32 link_stat;
	atomic_t new_req_id;

	spinlock_t request_lock;
	struct list_head req_list;

	unsigned char hw_mac_adr[ETH_ALEN];
};

struct rndis_request {
	struct list_head list_ent;
	struct completion  wait_event;

	/*
	 * FIXME: We assumed a fixed size response here. If we do ever need to
	 * handle a bigger response, we can either define a max response
	 * message or add a response buffer variable above this field
	 */
	struct rndis_message response_msg;

	/* Simplify allocation by having a netvsc packet inline */
	struct hv_netvsc_packet	pkt;
	struct hv_page_buffer buf;
	/* FIXME: We assumed a fixed size request here. */
	struct rndis_message request_msg;
};

static void rndis_filter_send_completion(void *ctx);

static void rndis_filter_send_request_completion(void *ctx);



static struct rndis_device *get_rndis_device(void)
{
	struct rndis_device *device;

	device = kzalloc(sizeof(struct rndis_device), GFP_KERNEL);
	if (!device)
		return NULL;

	spin_lock_init(&device->request_lock);

	INIT_LIST_HEAD(&device->req_list);

	device->state = RNDIS_DEV_UNINITIALIZED;

	return device;
}

static struct rndis_request *get_rndis_request(struct rndis_device *dev,
					     u32 msg_type,
					     u32 msg_len)
{
	struct rndis_request *request;
	struct rndis_message *rndis_msg;
	struct rndis_set_request *set;
	unsigned long flags;

	request = kzalloc(sizeof(struct rndis_request), GFP_KERNEL);
	if (!request)
		return NULL;

	init_completion(&request->wait_event);

	rndis_msg = &request->request_msg;
	rndis_msg->ndis_msg_type = msg_type;
	rndis_msg->msg_len = msg_len;

	/*
	 * Set the request id. This field is always after the rndis header for
	 * request/response packet types so we just used the SetRequest as a
	 * template
	 */
	set = &rndis_msg->msg.set_req;
	set->req_id = atomic_inc_return(&dev->new_req_id);

	/* Add to the request list */
	spin_lock_irqsave(&dev->request_lock, flags);
	list_add_tail(&request->list_ent, &dev->req_list);
	spin_unlock_irqrestore(&dev->request_lock, flags);

	return request;
}

static void put_rndis_request(struct rndis_device *dev,
			    struct rndis_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->request_lock, flags);
	list_del(&req->list_ent);
	spin_unlock_irqrestore(&dev->request_lock, flags);

	kfree(req);
}

static void dump_rndis_message(struct rndis_message *rndis_msg)
{
	switch (rndis_msg->ndis_msg_type) {
	case REMOTE_NDIS_PACKET_MSG:
		DPRINT_DBG(NETVSC, "REMOTE_NDIS_PACKET_MSG (len %u, "
			   "data offset %u data len %u, # oob %u, "
			   "oob offset %u, oob len %u, pkt offset %u, "
			   "pkt len %u",
			   rndis_msg->msg_len,
			   rndis_msg->msg.pkt.data_offset,
			   rndis_msg->msg.pkt.data_len,
			   rndis_msg->msg.pkt.num_oob_data_elements,
			   rndis_msg->msg.pkt.oob_data_offset,
			   rndis_msg->msg.pkt.oob_data_len,
			   rndis_msg->msg.pkt.per_pkt_info_offset,
			   rndis_msg->msg.pkt.per_pkt_info_len);
		break;

	case REMOTE_NDIS_INITIALIZE_CMPLT:
		DPRINT_DBG(NETVSC, "REMOTE_NDIS_INITIALIZE_CMPLT "
			"(len %u, id 0x%x, status 0x%x, major %d, minor %d, "
			"device flags %d, max xfer size 0x%x, max pkts %u, "
			"pkt aligned %u)",
			rndis_msg->msg_len,
			rndis_msg->msg.init_complete.req_id,
			rndis_msg->msg.init_complete.status,
			rndis_msg->msg.init_complete.major_ver,
			rndis_msg->msg.init_complete.minor_ver,
			rndis_msg->msg.init_complete.dev_flags,
			rndis_msg->msg.init_complete.max_xfer_size,
			rndis_msg->msg.init_complete.
			   max_pkt_per_msg,
			rndis_msg->msg.init_complete.
			   pkt_alignment_factor);
		break;

	case REMOTE_NDIS_QUERY_CMPLT:
		DPRINT_DBG(NETVSC, "REMOTE_NDIS_QUERY_CMPLT "
			"(len %u, id 0x%x, status 0x%x, buf len %u, "
			"buf offset %u)",
			rndis_msg->msg_len,
			rndis_msg->msg.query_complete.req_id,
			rndis_msg->msg.query_complete.status,
			rndis_msg->msg.query_complete.
			   info_buflen,
			rndis_msg->msg.query_complete.
			   info_buf_offset);
		break;

	case REMOTE_NDIS_SET_CMPLT:
		DPRINT_DBG(NETVSC,
			"REMOTE_NDIS_SET_CMPLT (len %u, id 0x%x, status 0x%x)",
			rndis_msg->msg_len,
			rndis_msg->msg.set_complete.req_id,
			rndis_msg->msg.set_complete.status);
		break;

	case REMOTE_NDIS_INDICATE_STATUS_MSG:
		DPRINT_DBG(NETVSC, "REMOTE_NDIS_INDICATE_STATUS_MSG "
			"(len %u, status 0x%x, buf len %u, buf offset %u)",
			rndis_msg->msg_len,
			rndis_msg->msg.indicate_status.status,
			rndis_msg->msg.indicate_status.status_buflen,
			rndis_msg->msg.indicate_status.status_buf_offset);
		break;

	default:
		DPRINT_DBG(NETVSC, "0x%x (len %u)",
			rndis_msg->ndis_msg_type,
			rndis_msg->msg_len);
		break;
	}
}

static int rndis_filter_send_request(struct rndis_device *dev,
				  struct rndis_request *req)
{
	int ret;
	struct hv_netvsc_packet *packet;

	/* Setup the packet to send it */
	packet = &req->pkt;

	packet->is_data_pkt = false;
	packet->total_data_buflen = req->request_msg.msg_len;
	packet->page_buf_cnt = 1;

	packet->page_buf[0].pfn = virt_to_phys(&req->request_msg) >>
					PAGE_SHIFT;
	packet->page_buf[0].len = req->request_msg.msg_len;
	packet->page_buf[0].offset =
		(unsigned long)&req->request_msg & (PAGE_SIZE - 1);

	packet->completion.send.send_completion_ctx = req;/* packet; */
	packet->completion.send.send_completion =
		rndis_filter_send_request_completion;
	packet->completion.send.send_completion_tid = (unsigned long)dev;

	ret = netvsc_send(dev->net_dev->dev, packet);
	return ret;
}

static void rndis_filter_receive_response(struct rndis_device *dev,
				       struct rndis_message *resp)
{
	struct rndis_request *request = NULL;
	bool found = false;
	unsigned long flags;

	spin_lock_irqsave(&dev->request_lock, flags);
	list_for_each_entry(request, &dev->req_list, list_ent) {
		/*
		 * All request/response message contains RequestId as the 1st
		 * field
		 */
		if (request->request_msg.msg.init_req.req_id
		    == resp->msg.init_complete.req_id) {
			found = true;
			break;
		}
	}
	spin_unlock_irqrestore(&dev->request_lock, flags);

	if (found) {
		if (resp->msg_len <= sizeof(struct rndis_message)) {
			memcpy(&request->response_msg, resp,
			       resp->msg_len);
		} else {
			dev_err(&dev->net_dev->dev->device,
				"rndis response buffer overflow "
				"detected (size %u max %zu)\n",
				resp->msg_len,
				sizeof(struct rndis_filter_packet));

			if (resp->ndis_msg_type ==
			    REMOTE_NDIS_RESET_CMPLT) {
				/* does not have a request id field */
				request->response_msg.msg.reset_complete.
					status = STATUS_BUFFER_OVERFLOW;
			} else {
				request->response_msg.msg.
				init_complete.status =
					STATUS_BUFFER_OVERFLOW;
			}
		}

		complete(&request->wait_event);
	} else {
		dev_err(&dev->net_dev->dev->device,
			"no rndis request found for this response "
			"(id 0x%x res type 0x%x)\n",
			resp->msg.init_complete.req_id,
			resp->ndis_msg_type);
	}
}

static void rndis_filter_receive_indicate_status(struct rndis_device *dev,
					     struct rndis_message *resp)
{
	struct rndis_indicate_status *indicate =
			&resp->msg.indicate_status;

	if (indicate->status == RNDIS_STATUS_MEDIA_CONNECT) {
		netvsc_linkstatus_callback(
			dev->net_dev->dev, 1);
	} else if (indicate->status == RNDIS_STATUS_MEDIA_DISCONNECT) {
		netvsc_linkstatus_callback(
			dev->net_dev->dev, 0);
	} else {
		/*
		 * TODO:
		 */
	}
}

static void rndis_filter_receive_data(struct rndis_device *dev,
				   struct rndis_message *msg,
				   struct hv_netvsc_packet *pkt)
{
	struct rndis_packet *rndis_pkt;
	u32 data_offset;

	rndis_pkt = &msg->msg.pkt;

	/*
	 * FIXME: Handle multiple rndis pkt msgs that maybe enclosed in this
	 * netvsc packet (ie TotalDataBufferLength != MessageLength)
	 */

	/* Remove the rndis header and pass it back up the stack */
	data_offset = RNDIS_HEADER_SIZE + rndis_pkt->data_offset;

	pkt->total_data_buflen -= data_offset;
	pkt->page_buf[0].offset += data_offset;
	pkt->page_buf[0].len -= data_offset;

	pkt->is_data_pkt = true;

	netvsc_recv_callback(dev->net_dev->dev, pkt);
}

int rndis_filter_receive(struct hv_device *dev,
				struct hv_netvsc_packet	*pkt)
{
	struct netvsc_device *net_dev = dev->ext;
	struct rndis_device *rndis_dev;
	struct rndis_message rndis_msg;
	struct rndis_message *rndis_hdr;

	if (!net_dev)
		return -EINVAL;

	/* Make sure the rndis device state is initialized */
	if (!net_dev->extension) {
		dev_err(&dev->device, "got rndis message but no rndis device - "
			  "dropping this message!\n");
		return -1;
	}

	rndis_dev = (struct rndis_device *)net_dev->extension;
	if (rndis_dev->state == RNDIS_DEV_UNINITIALIZED) {
		dev_err(&dev->device, "got rndis message but rndis device "
			   "uninitialized...dropping this message!\n");
		return -1;
	}

	rndis_hdr = (struct rndis_message *)kmap_atomic(
			pfn_to_page(pkt->page_buf[0].pfn), KM_IRQ0);

	rndis_hdr = (void *)((unsigned long)rndis_hdr +
			pkt->page_buf[0].offset);

	/* Make sure we got a valid rndis message */
	/*
	 * FIXME: There seems to be a bug in set completion msg where its
	 * MessageLength is 16 bytes but the ByteCount field in the xfer page
	 * range shows 52 bytes
	 * */
#if 0
	if (pkt->total_data_buflen != rndis_hdr->msg_len) {
		kunmap_atomic(rndis_hdr - pkt->page_buf[0].offset,
			      KM_IRQ0);

		dev_err(&dev->device, "invalid rndis message? (expected %u "
			   "bytes got %u)...dropping this message!\n",
			   rndis_hdr->msg_len,
			   pkt->total_data_buflen);
		return -1;
	}
#endif

	if ((rndis_hdr->ndis_msg_type != REMOTE_NDIS_PACKET_MSG) &&
	    (rndis_hdr->msg_len > sizeof(struct rndis_message))) {
		dev_err(&dev->device, "incoming rndis message buffer overflow "
			   "detected (got %u, max %zu)..marking it an error!\n",
			   rndis_hdr->msg_len,
			   sizeof(struct rndis_message));
	}

	memcpy(&rndis_msg, rndis_hdr,
		(rndis_hdr->msg_len > sizeof(struct rndis_message)) ?
			sizeof(struct rndis_message) :
			rndis_hdr->msg_len);

	kunmap_atomic(rndis_hdr - pkt->page_buf[0].offset, KM_IRQ0);

	dump_rndis_message(&rndis_msg);

	switch (rndis_msg.ndis_msg_type) {
	case REMOTE_NDIS_PACKET_MSG:
		/* data msg */
		rndis_filter_receive_data(rndis_dev, &rndis_msg, pkt);
		break;

	case REMOTE_NDIS_INITIALIZE_CMPLT:
	case REMOTE_NDIS_QUERY_CMPLT:
	case REMOTE_NDIS_SET_CMPLT:
		/* completion msgs */
		rndis_filter_receive_response(rndis_dev, &rndis_msg);
		break;

	case REMOTE_NDIS_INDICATE_STATUS_MSG:
		/* notification msgs */
		rndis_filter_receive_indicate_status(rndis_dev, &rndis_msg);
		break;
	default:
		dev_err(&dev->device,
			"unhandled rndis message (type %u len %u)\n",
			   rndis_msg.ndis_msg_type,
			   rndis_msg.msg_len);
		break;
	}

	return 0;
}

static int rndis_filter_query_device(struct rndis_device *dev, u32 oid,
				  void *result, u32 *result_size)
{
	struct rndis_request *request;
	u32 inresult_size = *result_size;
	struct rndis_query_request *query;
	struct rndis_query_complete *query_complete;
	int ret = 0;
	int t;

	if (!result)
		return -EINVAL;

	*result_size = 0;
	request = get_rndis_request(dev, REMOTE_NDIS_QUERY_MSG,
			RNDIS_MESSAGE_SIZE(struct rndis_query_request));
	if (!request) {
		ret = -1;
		goto Cleanup;
	}

	/* Setup the rndis query */
	query = &request->request_msg.msg.query_req;
	query->oid = oid;
	query->info_buf_offset = sizeof(struct rndis_query_request);
	query->info_buflen = 0;
	query->dev_vc_handle = 0;

	ret = rndis_filter_send_request(dev, request);
	if (ret != 0)
		goto Cleanup;

	t = wait_for_completion_timeout(&request->wait_event, 5*HZ);
	if (t == 0) {
		ret = -ETIMEDOUT;
		goto Cleanup;
	}

	/* Copy the response back */
	query_complete = &request->response_msg.msg.query_complete;

	if (query_complete->info_buflen > inresult_size) {
		ret = -1;
		goto Cleanup;
	}

	memcpy(result,
	       (void *)((unsigned long)query_complete +
			 query_complete->info_buf_offset),
	       query_complete->info_buflen);

	*result_size = query_complete->info_buflen;

Cleanup:
	if (request)
		put_rndis_request(dev, request);

	return ret;
}

static int rndis_filter_query_device_mac(struct rndis_device *dev)
{
	u32 size = ETH_ALEN;

	return rndis_filter_query_device(dev,
				      RNDIS_OID_802_3_PERMANENT_ADDRESS,
				      dev->hw_mac_adr, &size);
}

static int rndis_filter_query_device_link_status(struct rndis_device *dev)
{
	u32 size = sizeof(u32);

	return rndis_filter_query_device(dev,
				      RNDIS_OID_GEN_MEDIA_CONNECT_STATUS,
				      &dev->link_stat, &size);
}

static int rndis_filter_set_packet_filter(struct rndis_device *dev,
				      u32 new_filter)
{
	struct rndis_request *request;
	struct rndis_set_request *set;
	struct rndis_set_complete *set_complete;
	u32 status;
	int ret, t;

	request = get_rndis_request(dev, REMOTE_NDIS_SET_MSG,
			RNDIS_MESSAGE_SIZE(struct rndis_set_request) +
			sizeof(u32));
	if (!request) {
		ret = -1;
		goto Cleanup;
	}

	/* Setup the rndis set */
	set = &request->request_msg.msg.set_req;
	set->oid = RNDIS_OID_GEN_CURRENT_PACKET_FILTER;
	set->info_buflen = sizeof(u32);
	set->info_buf_offset = sizeof(struct rndis_set_request);

	memcpy((void *)(unsigned long)set + sizeof(struct rndis_set_request),
	       &new_filter, sizeof(u32));

	ret = rndis_filter_send_request(dev, request);
	if (ret != 0)
		goto Cleanup;

	t = wait_for_completion_timeout(&request->wait_event, 5*HZ);

	if (t == 0) {
		ret = -1;
		dev_err(&dev->net_dev->dev->device,
			"timeout before we got a set response...\n");
		/*
		 * We can't deallocate the request since we may still receive a
		 * send completion for it.
		 */
		goto Exit;
	} else {
		if (ret > 0)
			ret = 0;
		set_complete = &request->response_msg.msg.set_complete;
		status = set_complete->status;
	}

Cleanup:
	if (request)
		put_rndis_request(dev, request);
Exit:
	return ret;
}


static int rndis_filter_init_device(struct rndis_device *dev)
{
	struct rndis_request *request;
	struct rndis_initialize_request *init;
	struct rndis_initialize_complete *init_complete;
	u32 status;
	int ret, t;

	request = get_rndis_request(dev, REMOTE_NDIS_INITIALIZE_MSG,
			RNDIS_MESSAGE_SIZE(struct rndis_initialize_request));
	if (!request) {
		ret = -1;
		goto Cleanup;
	}

	/* Setup the rndis set */
	init = &request->request_msg.msg.init_req;
	init->major_ver = RNDIS_MAJOR_VERSION;
	init->minor_ver = RNDIS_MINOR_VERSION;
	/* FIXME: Use 1536 - rounded ethernet frame size */
	init->max_xfer_size = 2048;

	dev->state = RNDIS_DEV_INITIALIZING;

	ret = rndis_filter_send_request(dev, request);
	if (ret != 0) {
		dev->state = RNDIS_DEV_UNINITIALIZED;
		goto Cleanup;
	}


	t = wait_for_completion_timeout(&request->wait_event, 5*HZ);

	if (t == 0) {
		ret = -ETIMEDOUT;
		goto Cleanup;
	}

	init_complete = &request->response_msg.msg.init_complete;
	status = init_complete->status;
	if (status == RNDIS_STATUS_SUCCESS) {
		dev->state = RNDIS_DEV_INITIALIZED;
		ret = 0;
	} else {
		dev->state = RNDIS_DEV_UNINITIALIZED;
		ret = -1;
	}

Cleanup:
	if (request)
		put_rndis_request(dev, request);

	return ret;
}

static void rndis_filter_halt_device(struct rndis_device *dev)
{
	struct rndis_request *request;
	struct rndis_halt_request *halt;

	/* Attempt to do a rndis device halt */
	request = get_rndis_request(dev, REMOTE_NDIS_HALT_MSG,
				RNDIS_MESSAGE_SIZE(struct rndis_halt_request));
	if (!request)
		goto Cleanup;

	/* Setup the rndis set */
	halt = &request->request_msg.msg.halt_req;
	halt->req_id = atomic_inc_return(&dev->new_req_id);

	/* Ignore return since this msg is optional. */
	rndis_filter_send_request(dev, request);

	dev->state = RNDIS_DEV_UNINITIALIZED;

Cleanup:
	if (request)
		put_rndis_request(dev, request);
	return;
}

static int rndis_filter_open_device(struct rndis_device *dev)
{
	int ret;

	if (dev->state != RNDIS_DEV_INITIALIZED)
		return 0;

	ret = rndis_filter_set_packet_filter(dev,
					 NDIS_PACKET_TYPE_BROADCAST |
					 NDIS_PACKET_TYPE_ALL_MULTICAST |
					 NDIS_PACKET_TYPE_DIRECTED);
	if (ret == 0)
		dev->state = RNDIS_DEV_DATAINITIALIZED;

	return ret;
}

static int rndis_filter_close_device(struct rndis_device *dev)
{
	int ret;

	if (dev->state != RNDIS_DEV_DATAINITIALIZED)
		return 0;

	ret = rndis_filter_set_packet_filter(dev, 0);
	if (ret == 0)
		dev->state = RNDIS_DEV_INITIALIZED;

	return ret;
}

int rndis_filte_device_add(struct hv_device *dev,
				  void *additional_info)
{
	int ret;
	struct netvsc_device *netDevice;
	struct rndis_device *rndisDevice;
	struct netvsc_device_info *deviceInfo = additional_info;

	rndisDevice = get_rndis_device();
	if (!rndisDevice)
		return -1;

	/*
	 * Let the inner driver handle this first to create the netvsc channel
	 * NOTE! Once the channel is created, we may get a receive callback
	 * (RndisFilterOnReceive()) before this call is completed
	 */
	ret = netvsc_device_add(dev, additional_info);
	if (ret != 0) {
		kfree(rndisDevice);
		return ret;
	}


	/* Initialize the rndis device */
	netDevice = dev->ext;

	netDevice->extension = rndisDevice;
	rndisDevice->net_dev = netDevice;

	/* Send the rndis initialization message */
	ret = rndis_filter_init_device(rndisDevice);
	if (ret != 0) {
		/*
		 * TODO: If rndis init failed, we will need to shut down the
		 * channel
		 */
	}

	/* Get the mac address */
	ret = rndis_filter_query_device_mac(rndisDevice);
	if (ret != 0) {
		/*
		 * TODO: shutdown rndis device and the channel
		 */
	}

	memcpy(deviceInfo->mac_adr, rndisDevice->hw_mac_adr, ETH_ALEN);

	rndis_filter_query_device_link_status(rndisDevice);

	deviceInfo->link_state = rndisDevice->link_stat;

	dev_info(&dev->device, "Device MAC %pM link state %s",
		 rndisDevice->hw_mac_adr,
		 ((deviceInfo->link_state) ? ("down\n") : ("up\n")));

	return ret;
}

int rndis_filter_device_remove(struct hv_device *dev)
{
	struct netvsc_device *net_dev = dev->ext;
	struct rndis_device *rndis_dev = net_dev->extension;

	/* Halt and release the rndis device */
	rndis_filter_halt_device(rndis_dev);

	kfree(rndis_dev);
	net_dev->extension = NULL;

	netvsc_device_remove(dev);

	return 0;
}


int rndis_filter_open(struct hv_device *dev)
{
	struct netvsc_device *netDevice = dev->ext;

	if (!netDevice)
		return -EINVAL;

	return rndis_filter_open_device(netDevice->extension);
}

int rndis_filter_close(struct hv_device *dev)
{
	struct netvsc_device *netDevice = dev->ext;

	if (!netDevice)
		return -EINVAL;

	return rndis_filter_close_device(netDevice->extension);
}

int rndis_filter_send(struct hv_device *dev,
			     struct hv_netvsc_packet *pkt)
{
	int ret;
	struct rndis_filter_packet *filterPacket;
	struct rndis_message *rndisMessage;
	struct rndis_packet *rndisPacket;
	u32 rndisMessageSize;

	/* Add the rndis header */
	filterPacket = (struct rndis_filter_packet *)pkt->extension;

	memset(filterPacket, 0, sizeof(struct rndis_filter_packet));

	rndisMessage = &filterPacket->msg;
	rndisMessageSize = RNDIS_MESSAGE_SIZE(struct rndis_packet);

	rndisMessage->ndis_msg_type = REMOTE_NDIS_PACKET_MSG;
	rndisMessage->msg_len = pkt->total_data_buflen +
				      rndisMessageSize;

	rndisPacket = &rndisMessage->msg.pkt;
	rndisPacket->data_offset = sizeof(struct rndis_packet);
	rndisPacket->data_len = pkt->total_data_buflen;

	pkt->is_data_pkt = true;
	pkt->page_buf[0].pfn = virt_to_phys(rndisMessage) >> PAGE_SHIFT;
	pkt->page_buf[0].offset =
			(unsigned long)rndisMessage & (PAGE_SIZE-1);
	pkt->page_buf[0].len = rndisMessageSize;

	/* Save the packet send completion and context */
	filterPacket->completion = pkt->completion.send.send_completion;
	filterPacket->completion_ctx =
				pkt->completion.send.send_completion_ctx;

	/* Use ours */
	pkt->completion.send.send_completion = rndis_filter_send_completion;
	pkt->completion.send.send_completion_ctx = filterPacket;

	ret = netvsc_send(dev, pkt);
	if (ret != 0) {
		/*
		 * Reset the completion to originals to allow retries from
		 * above
		 */
		pkt->completion.send.send_completion =
				filterPacket->completion;
		pkt->completion.send.send_completion_ctx =
				filterPacket->completion_ctx;
	}

	return ret;
}

static void rndis_filter_send_completion(void *ctx)
{
	struct rndis_filter_packet *filterPacket = ctx;

	/* Pass it back to the original handler */
	filterPacket->completion(filterPacket->completion_ctx);
}


static void rndis_filter_send_request_completion(void *ctx)
{
	/* Noop */
}
