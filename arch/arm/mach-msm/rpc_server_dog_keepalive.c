/* arch/arm/mach-msm/rpc_server_dog_keepalive.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Iliyan Malchev <ibm@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_rpc_version.h>

/* dog_keepalive server definitions */

#define DOG_KEEPALIVE_PROG 0x30000015
#define RPC_DOG_KEEPALIVE_NULL 0


/* TODO: Remove server registration with _VERS when modem is upated with _COMP*/

static int handle_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	switch (req->procedure) {
	case RPC_DOG_KEEPALIVE_NULL:
		return 0;
	case RPC_DOG_KEEPALIVE_BEACON:
		//printk(KERN_INFO "DOG KEEPALIVE PING\n");
		return 0;
	default:
		return -ENODEV;
	}
}

static struct msm_rpc_server rpc_server = {
	.prog = DOG_KEEPALIVE_PROG,
	.vers = DOG_KEEPALIVE_VERS,
	.rpc_call = handle_rpc_call,
};

static int __init rpc_server_init(void)
{
	/* Dual server registration to support backwards compatibility vers */
	return msm_rpc_create_server(&rpc_server);
}


module_init(rpc_server_init);
