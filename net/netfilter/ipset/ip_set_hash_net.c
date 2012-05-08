/* Copyright (C) 2003-2011 Jozsef Kadlecsik <kadlec@blackhole.kfki.hu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Kernel module implementing an IP set type: the hash:net type */

#include <linux/jhash.h>
#include <linux/module.h>
#include <linux/ip.h>
#include <linux/skbuff.h>
#include <linux/errno.h>
#include <linux/random.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/netlink.h>

#include <linux/netfilter.h>
#include <linux/netfilter/ipset/pfxlen.h>
#include <linux/netfilter/ipset/ip_set.h>
#include <linux/netfilter/ipset/ip_set_timeout.h>
#include <linux/netfilter/ipset/ip_set_hash.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jozsef Kadlecsik <kadlec@blackhole.kfki.hu>");
MODULE_DESCRIPTION("hash:net type of IP sets");
MODULE_ALIAS("ip_set_hash:net");

/* Type specific function prefix */
#define TYPE		hash_net

static bool
hash_net_same_set(const struct ip_set *a, const struct ip_set *b);

#define hash_net4_same_set	hash_net_same_set
#define hash_net6_same_set	hash_net_same_set

/* The type variant functions: IPv4 */

/* Member elements without timeout */
struct hash_net4_elem {
	__be32 ip;
	u16 padding0;
	u8 padding1;
	u8 cidr;
};

/* Member elements with timeout support */
struct hash_net4_telem {
	__be32 ip;
	u16 padding0;
	u8 padding1;
	u8 cidr;
	unsigned long timeout;
};

static inline bool
hash_net4_data_equal(const struct hash_net4_elem *ip1,
		    const struct hash_net4_elem *ip2)
{
	return ip1->ip == ip2->ip && ip1->cidr == ip2->cidr;
}

static inline bool
hash_net4_data_isnull(const struct hash_net4_elem *elem)
{
	return elem->cidr == 0;
}

static inline void
hash_net4_data_copy(struct hash_net4_elem *dst,
		    const struct hash_net4_elem *src)
{
	dst->ip = src->ip;
	dst->cidr = src->cidr;
}

static inline void
hash_net4_data_netmask(struct hash_net4_elem *elem, u8 cidr)
{
	elem->ip &= ip_set_netmask(cidr);
	elem->cidr = cidr;
}

/* Zero CIDR values cannot be stored */
static inline void
hash_net4_data_zero_out(struct hash_net4_elem *elem)
{
	elem->cidr = 0;
}

static bool
hash_net4_data_list(struct sk_buff *skb, const struct hash_net4_elem *data)
{
	NLA_PUT_IPADDR4(skb, IPSET_ATTR_IP, data->ip);
	NLA_PUT_U8(skb, IPSET_ATTR_CIDR, data->cidr);
	return 0;

nla_put_failure:
	return 1;
}

static bool
hash_net4_data_tlist(struct sk_buff *skb, const struct hash_net4_elem *data)
{
	const struct hash_net4_telem *tdata =
		(const struct hash_net4_telem *)data;

	NLA_PUT_IPADDR4(skb, IPSET_ATTR_IP, tdata->ip);
	NLA_PUT_U8(skb, IPSET_ATTR_CIDR, tdata->cidr);
	NLA_PUT_NET32(skb, IPSET_ATTR_TIMEOUT,
		      htonl(ip_set_timeout_get(tdata->timeout)));

	return 0;

nla_put_failure:
	return 1;
}

#define IP_SET_HASH_WITH_NETS

#define PF		4
#define HOST_MASK	32
#include <linux/netfilter/ipset/ip_set_ahash.h>

static int
hash_net4_kadt(struct ip_set *set, const struct sk_buff *skb,
	       enum ipset_adt adt, u8 pf, u8 dim, u8 flags)
{
	const struct ip_set_hash *h = set->data;
	ipset_adtfn adtfn = set->variant->adt[adt];
	struct hash_net4_elem data = {
		.cidr = h->nets[0].cidr ? h->nets[0].cidr : HOST_MASK
	};

	if (data.cidr == 0)
		return -EINVAL;
	if (adt == IPSET_TEST)
		data.cidr = HOST_MASK;

	ip4addrptr(skb, flags & IPSET_DIM_ONE_SRC, &data.ip);
	data.ip &= ip_set_netmask(data.cidr);

	return adtfn(set, &data, h->timeout);
}

static int
hash_net4_uadt(struct ip_set *set, struct nlattr *tb[],
	       enum ipset_adt adt, u32 *lineno, u32 flags)
{
	const struct ip_set_hash *h = set->data;
	ipset_adtfn adtfn = set->variant->adt[adt];
	struct hash_net4_elem data = { .cidr = HOST_MASK };
	u32 timeout = h->timeout;
	int ret;

	if (unlikely(!tb[IPSET_ATTR_IP] ||
		     !ip_set_optattr_netorder(tb, IPSET_ATTR_TIMEOUT)))
		return -IPSET_ERR_PROTOCOL;

	if (tb[IPSET_ATTR_LINENO])
		*lineno = nla_get_u32(tb[IPSET_ATTR_LINENO]);

	ret = ip_set_get_ipaddr4(tb[IPSET_ATTR_IP], &data.ip);
	if (ret)
		return ret;

	if (tb[IPSET_ATTR_CIDR])
		data.cidr = nla_get_u8(tb[IPSET_ATTR_CIDR]);

	if (!data.cidr)
		return -IPSET_ERR_INVALID_CIDR;

	data.ip &= ip_set_netmask(data.cidr);

	if (tb[IPSET_ATTR_TIMEOUT]) {
		if (!with_timeout(h->timeout))
			return -IPSET_ERR_TIMEOUT;
		timeout = ip_set_timeout_uget(tb[IPSET_ATTR_TIMEOUT]);
	}

	ret = adtfn(set, &data, timeout);

	return ip_set_eexist(ret, flags) ? 0 : ret;
}

static bool
hash_net_same_set(const struct ip_set *a, const struct ip_set *b)
{
	const struct ip_set_hash *x = a->data;
	const struct ip_set_hash *y = b->data;

	/* Resizing changes htable_bits, so we ignore it */
	return x->maxelem == y->maxelem &&
	       x->timeout == y->timeout;
}

/* The type variant functions: IPv6 */

struct hash_net6_elem {
	union nf_inet_addr ip;
	u16 padding0;
	u8 padding1;
	u8 cidr;
};

struct hash_net6_telem {
	union nf_inet_addr ip;
	u16 padding0;
	u8 padding1;
	u8 cidr;
	unsigned long timeout;
};

static inline bool
hash_net6_data_equal(const struct hash_net6_elem *ip1,
		     const struct hash_net6_elem *ip2)
{
	return ipv6_addr_cmp(&ip1->ip.in6, &ip2->ip.in6) == 0 &&
	       ip1->cidr == ip2->cidr;
}

static inline bool
hash_net6_data_isnull(const struct hash_net6_elem *elem)
{
	return elem->cidr == 0;
}

static inline void
hash_net6_data_copy(struct hash_net6_elem *dst,
		    const struct hash_net6_elem *src)
{
	ipv6_addr_copy(&dst->ip.in6, &src->ip.in6);
	dst->cidr = src->cidr;
}

static inline void
hash_net6_data_zero_out(struct hash_net6_elem *elem)
{
	elem->cidr = 0;
}

static inline void
ip6_netmask(union nf_inet_addr *ip, u8 prefix)
{
	ip->ip6[0] &= ip_set_netmask6(prefix)[0];
	ip->ip6[1] &= ip_set_netmask6(prefix)[1];
	ip->ip6[2] &= ip_set_netmask6(prefix)[2];
	ip->ip6[3] &= ip_set_netmask6(prefix)[3];
}

static inline void
hash_net6_data_netmask(struct hash_net6_elem *elem, u8 cidr)
{
	ip6_netmask(&elem->ip, cidr);
	elem->cidr = cidr;
}

static bool
hash_net6_data_list(struct sk_buff *skb, const struct hash_net6_elem *data)
{
	NLA_PUT_IPADDR6(skb, IPSET_ATTR_IP, &data->ip);
	NLA_PUT_U8(skb, IPSET_ATTR_CIDR, data->cidr);
	return 0;

nla_put_failure:
	return 1;
}

static bool
hash_net6_data_tlist(struct sk_buff *skb, const struct hash_net6_elem *data)
{
	const struct hash_net6_telem *e =
		(const struct hash_net6_telem *)data;

	NLA_PUT_IPADDR6(skb, IPSET_ATTR_IP, &e->ip);
	NLA_PUT_U8(skb, IPSET_ATTR_CIDR, e->cidr);
	NLA_PUT_NET32(skb, IPSET_ATTR_TIMEOUT,
		      htonl(ip_set_timeout_get(e->timeout)));
	return 0;

nla_put_failure:
	return 1;
}

#undef PF
#undef HOST_MASK

#define PF		6
#define HOST_MASK	128
#include <linux/netfilter/ipset/ip_set_ahash.h>

static int
hash_net6_kadt(struct ip_set *set, const struct sk_buff *skb,
	       enum ipset_adt adt, u8 pf, u8 dim, u8 flags)
{
	const struct ip_set_hash *h = set->data;
	ipset_adtfn adtfn = set->variant->adt[adt];
	struct hash_net6_elem data = {
		.cidr = h->nets[0].cidr ? h->nets[0].cidr : HOST_MASK
	};

	if (data.cidr == 0)
		return -EINVAL;
	if (adt == IPSET_TEST)
		data.cidr = HOST_MASK;

	ip6addrptr(skb, flags & IPSET_DIM_ONE_SRC, &data.ip.in6);
	ip6_netmask(&data.ip, data.cidr);

	return adtfn(set, &data, h->timeout);
}

static int
hash_net6_uadt(struct ip_set *set, struct nlattr *tb[],
	       enum ipset_adt adt, u32 *lineno, u32 flags)
{
	const struct ip_set_hash *h = set->data;
	ipset_adtfn adtfn = set->variant->adt[adt];
	struct hash_net6_elem data = { .cidr = HOST_MASK };
	u32 timeout = h->timeout;
	int ret;

	if (unlikely(!tb[IPSET_ATTR_IP] ||
		     !ip_set_optattr_netorder(tb, IPSET_ATTR_TIMEOUT)))
		return -IPSET_ERR_PROTOCOL;

	if (tb[IPSET_ATTR_LINENO])
		*lineno = nla_get_u32(tb[IPSET_ATTR_LINENO]);

	ret = ip_set_get_ipaddr6(tb[IPSET_ATTR_IP], &data.ip);
	if (ret)
		return ret;

	if (tb[IPSET_ATTR_CIDR])
		data.cidr = nla_get_u8(tb[IPSET_ATTR_CIDR]);

	if (!data.cidr)
		return -IPSET_ERR_INVALID_CIDR;

	ip6_netmask(&data.ip, data.cidr);

	if (tb[IPSET_ATTR_TIMEOUT]) {
		if (!with_timeout(h->timeout))
			return -IPSET_ERR_TIMEOUT;
		timeout = ip_set_timeout_uget(tb[IPSET_ATTR_TIMEOUT]);
	}

	ret = adtfn(set, &data, timeout);

	return ip_set_eexist(ret, flags) ? 0 : ret;
}

/* Create hash:ip type of sets */

static int
hash_net_create(struct ip_set *set, struct nlattr *tb[], u32 flags)
{
	u32 hashsize = IPSET_DEFAULT_HASHSIZE, maxelem = IPSET_DEFAULT_MAXELEM;
	struct ip_set_hash *h;
	u8 hbits;

	if (!(set->family == AF_INET || set->family == AF_INET6))
		return -IPSET_ERR_INVALID_FAMILY;

	if (unlikely(!ip_set_optattr_netorder(tb, IPSET_ATTR_HASHSIZE) ||
		     !ip_set_optattr_netorder(tb, IPSET_ATTR_MAXELEM) ||
		     !ip_set_optattr_netorder(tb, IPSET_ATTR_TIMEOUT)))
		return -IPSET_ERR_PROTOCOL;

	if (tb[IPSET_ATTR_HASHSIZE]) {
		hashsize = ip_set_get_h32(tb[IPSET_ATTR_HASHSIZE]);
		if (hashsize < IPSET_MIMINAL_HASHSIZE)
			hashsize = IPSET_MIMINAL_HASHSIZE;
	}

	if (tb[IPSET_ATTR_MAXELEM])
		maxelem = ip_set_get_h32(tb[IPSET_ATTR_MAXELEM]);

	h = kzalloc(sizeof(*h)
		    + sizeof(struct ip_set_hash_nets)
		      * (set->family == AF_INET ? 32 : 128), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

	h->maxelem = maxelem;
	get_random_bytes(&h->initval, sizeof(h->initval));
	h->timeout = IPSET_NO_TIMEOUT;

	hbits = htable_bits(hashsize);
	h->table = ip_set_alloc(
			sizeof(struct htable)
			+ jhash_size(hbits) * sizeof(struct hbucket));
	if (!h->table) {
		kfree(h);
		return -ENOMEM;
	}
	h->table->htable_bits = hbits;

	set->data = h;

	if (tb[IPSET_ATTR_TIMEOUT]) {
		h->timeout = ip_set_timeout_uget(tb[IPSET_ATTR_TIMEOUT]);

		set->variant = set->family == AF_INET
			? &hash_net4_tvariant : &hash_net6_tvariant;

		if (set->family == AF_INET)
			hash_net4_gc_init(set);
		else
			hash_net6_gc_init(set);
	} else {
		set->variant = set->family == AF_INET
			? &hash_net4_variant : &hash_net6_variant;
	}

	pr_debug("create %s hashsize %u (%u) maxelem %u: %p(%p)\n",
		 set->name, jhash_size(h->table->htable_bits),
		 h->table->htable_bits, h->maxelem, set->data, h->table);

	return 0;
}

static struct ip_set_type hash_net_type __read_mostly = {
	.name		= "hash:net",
	.protocol	= IPSET_PROTOCOL,
	.features	= IPSET_TYPE_IP,
	.dimension	= IPSET_DIM_ONE,
	.family		= AF_UNSPEC,
	.revision	= 0,
	.create		= hash_net_create,
	.create_policy	= {
		[IPSET_ATTR_HASHSIZE]	= { .type = NLA_U32 },
		[IPSET_ATTR_MAXELEM]	= { .type = NLA_U32 },
		[IPSET_ATTR_PROBES]	= { .type = NLA_U8 },
		[IPSET_ATTR_RESIZE]	= { .type = NLA_U8  },
		[IPSET_ATTR_TIMEOUT]	= { .type = NLA_U32 },
	},
	.adt_policy	= {
		[IPSET_ATTR_IP]		= { .type = NLA_NESTED },
		[IPSET_ATTR_CIDR]	= { .type = NLA_U8 },
		[IPSET_ATTR_TIMEOUT]	= { .type = NLA_U32 },
	},
	.me		= THIS_MODULE,
};

static int __init
hash_net_init(void)
{
	return ip_set_type_register(&hash_net_type);
}

static void __exit
hash_net_fini(void)
{
	ip_set_type_unregister(&hash_net_type);
}

module_init(hash_net_init);
module_exit(hash_net_fini);
