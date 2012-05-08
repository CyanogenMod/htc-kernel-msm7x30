/*
 * soc-cache.c  --  ASoC register cache helpers
 *
 * Copyright 2009 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <sound/soc.h>
#include <linux/lzo.h>
#include <linux/bitmap.h>
#include <linux/rbtree.h>

#include <trace/events/asoc.h>

#ifdef CONFIG_SPI_MASTER
static int do_spi_write(void *control, const char *data, int len)
{
	struct spi_device *spi = control;
	int ret;

	ret = spi_write(spi, data, len);
	if (ret < 0)
		return ret;

	return len;
}
#endif

static int do_hw_write(struct snd_soc_codec *codec, unsigned int reg,
		       unsigned int value, const void *data, int len)
{
	int ret;

	if (!snd_soc_codec_volatile_register(codec, reg) &&
	    reg < codec->driver->reg_cache_size &&
	    !codec->cache_bypass) {
		ret = snd_soc_cache_write(codec, reg, value);
		if (ret < 0)
			return -1;
	}

	if (codec->cache_only) {
		codec->cache_sync = 1;
		return 0;
	}

	ret = codec->hw_write(codec->control_data, data, len);
	if (ret == len)
		return 0;
	if (ret < 0)
		return ret;
	else
		return -EIO;
}

static unsigned int do_hw_read(struct snd_soc_codec *codec, unsigned int reg)
{
	int ret;
	unsigned int val;

	if (reg >= codec->driver->reg_cache_size ||
	    snd_soc_codec_volatile_register(codec, reg) ||
	    codec->cache_bypass) {
		if (codec->cache_only)
			return -1;

		BUG_ON(!codec->hw_read);
		return codec->hw_read(codec, reg);
	}

	ret = snd_soc_cache_read(codec, reg, &val);
	if (ret < 0)
		return -1;
	return val;
}

static unsigned int snd_soc_4_12_read(struct snd_soc_codec *codec,
				      unsigned int reg)
{
	return do_hw_read(codec, reg);
}

static int snd_soc_4_12_write(struct snd_soc_codec *codec, unsigned int reg,
			      unsigned int value)
{
	u16 data;

	data = cpu_to_be16((reg << 12) | (value & 0xffffff));

	return do_hw_write(codec, reg, value, &data, 2);
}

static unsigned int snd_soc_7_9_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	return do_hw_read(codec, reg);
}

static int snd_soc_7_9_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{
	u8 data[2];

	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	return do_hw_write(codec, reg, value, data, 2);
}

static int snd_soc_8_8_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{
	u8 data[2];

	reg &= 0xff;
	data[0] = reg;
	data[1] = value & 0xff;

	return do_hw_write(codec, reg, value, data, 2);
}

static unsigned int snd_soc_8_8_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	return do_hw_read(codec, reg);
}

static int snd_soc_8_16_write(struct snd_soc_codec *codec, unsigned int reg,
			      unsigned int value)
{
	u8 data[3];

	data[0] = reg;
	data[1] = (value >> 8) & 0xff;
	data[2] = value & 0xff;

	return do_hw_write(codec, reg, value, data, 3);
}

static unsigned int snd_soc_8_16_read(struct snd_soc_codec *codec,
				      unsigned int reg)
{
	return do_hw_read(codec, reg);
}

#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
static unsigned int do_i2c_read(struct snd_soc_codec *codec,
				void *reg, int reglen,
				void *data, int datalen)
{
	struct i2c_msg xfer[2];
	int ret;
	struct i2c_client *client = codec->control_data;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = reglen;
	xfer[0].buf = reg;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = datalen;
	xfer[1].buf = data;

	ret = i2c_transfer(client->adapter, xfer, 2);
	if (ret == 2)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}
#endif

#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
static unsigned int snd_soc_8_8_read_i2c(struct snd_soc_codec *codec,
					 unsigned int r)
{
	u8 reg = r;
	u8 data = 0;
	int ret = 0;

	ret = do_i2c_read(codec, &reg, 1, &data, 1);
	if (ret < 0)
		return 0;
	return data;
}
#else
#define snd_soc_8_8_read_i2c NULL
#endif

#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
static unsigned int snd_soc_8_16_read_i2c(struct snd_soc_codec *codec,
					  unsigned int r)
{
	u8 reg = r;
	u16 data = 0;
	int ret = 0;

	ret = do_i2c_read(codec, &reg, 1, &data, 2);
	if (ret < 0)
		return 0;
	return (data >> 8) | ((data & 0xff) << 8);
}
#else
#define snd_soc_8_16_read_i2c NULL
#endif

#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
static unsigned int snd_soc_16_8_read_i2c(struct snd_soc_codec *codec,
					  unsigned int r)
{
	u16 reg = r;
	u8 data = 0;
	int ret = 0;

	ret = do_i2c_read(codec, &reg, 2, &data, 1);
	if (ret < 0)
		return 0;
	return data;
}
#else
#define snd_soc_16_8_read_i2c NULL
#endif

static unsigned int snd_soc_16_8_read(struct snd_soc_codec *codec,
				      unsigned int reg)
{
	return do_hw_read(codec, reg);
}

static int snd_soc_16_8_write(struct snd_soc_codec *codec, unsigned int reg,
			      unsigned int value)
{
	u8 data[3];

	data[0] = (reg >> 8) & 0xff;
	data[1] = reg & 0xff;
	data[2] = value;

	return do_hw_write(codec, reg, value, data, 3);
}

#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
static unsigned int snd_soc_16_16_read_i2c(struct snd_soc_codec *codec,
					   unsigned int r)
{
	u16 reg = cpu_to_be16(r);
	u16 data = 0;
	int ret = 0;

	ret = do_i2c_read(codec, &reg, 2, &data, 2);
	if (ret < 0)
		return 0;
	return be16_to_cpu(data);
}
#else
#define snd_soc_16_16_read_i2c NULL
#endif

static unsigned int snd_soc_16_16_read(struct snd_soc_codec *codec,
				       unsigned int reg)
{
	return do_hw_read(codec, reg);
}

static int snd_soc_16_16_write(struct snd_soc_codec *codec, unsigned int reg,
			       unsigned int value)
{
	u8 data[4];

	data[0] = (reg >> 8) & 0xff;
	data[1] = reg & 0xff;
	data[2] = (value >> 8) & 0xff;
	data[3] = value & 0xff;

	return do_hw_write(codec, reg, value, data, 4);
}

/* Primitive bulk write support for soc-cache.  The data pointed to by
 * `data' needs to already be in the form the hardware expects
 * including any leading register specific data.  Any data written
 * through this function will not go through the cache as it only
 * handles writing to volatile or out of bounds registers.
 */
static int snd_soc_hw_bulk_write_raw(struct snd_soc_codec *codec, unsigned int reg,
				     const void *data, size_t len)
{
	int ret;

	/* To ensure that we don't get out of sync with the cache, check
	 * whether the base register is volatile or if we've directly asked
	 * to bypass the cache.  Out of bounds registers are considered
	 * volatile.
	 */
	if (!codec->cache_bypass
	    && !snd_soc_codec_volatile_register(codec, reg)
	    && reg < codec->driver->reg_cache_size)
		return -EINVAL;

	switch (codec->control_type) {
#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
	case SND_SOC_I2C:
		ret = i2c_master_send(codec->control_data, data, len);
		break;
#endif
#if defined(CONFIG_SPI_MASTER)
	case SND_SOC_SPI:
		ret = spi_write(codec->control_data, data, len);
		break;
#endif
	default:
		BUG();
	}

	if (ret == len)
		return 0;
	if (ret < 0)
		return ret;
	else
		return -EIO;
}

static struct {
	int addr_bits;
	int data_bits;
	int (*write)(struct snd_soc_codec *codec, unsigned int, unsigned int);
	unsigned int (*read)(struct snd_soc_codec *, unsigned int);
	unsigned int (*i2c_read)(struct snd_soc_codec *, unsigned int);
} io_types[] = {
	{
		.addr_bits = 4, .data_bits = 12,
		.write = snd_soc_4_12_write, .read = snd_soc_4_12_read,
	},
	{
		.addr_bits = 7, .data_bits = 9,
		.write = snd_soc_7_9_write, .read = snd_soc_7_9_read,
	},
	{
		.addr_bits = 8, .data_bits = 8,
		.write = snd_soc_8_8_write, .read = snd_soc_8_8_read,
		.i2c_read = snd_soc_8_8_read_i2c,
	},
	{
		.addr_bits = 8, .data_bits = 16,
		.write = snd_soc_8_16_write, .read = snd_soc_8_16_read,
		.i2c_read = snd_soc_8_16_read_i2c,
	},
	{
		.addr_bits = 16, .data_bits = 8,
		.write = snd_soc_16_8_write, .read = snd_soc_16_8_read,
		.i2c_read = snd_soc_16_8_read_i2c,
	},
	{
		.addr_bits = 16, .data_bits = 16,
		.write = snd_soc_16_16_write, .read = snd_soc_16_16_read,
		.i2c_read = snd_soc_16_16_read_i2c,
	},
};

/**
 * snd_soc_codec_set_cache_io: Set up standard I/O functions.
 *
 * @codec: CODEC to configure.
 * @addr_bits: Number of bits of register address data.
 * @data_bits: Number of bits of data per register.
 * @control: Control bus used.
 *
 * Register formats are frequently shared between many I2C and SPI
 * devices.  In order to promote code reuse the ASoC core provides
 * some standard implementations of CODEC read and write operations
 * which can be set up using this function.
 *
 * The caller is responsible for allocating and initialising the
 * actual cache.
 *
 * Note that at present this code cannot be used by CODECs with
 * volatile registers.
 */
int snd_soc_codec_set_cache_io(struct snd_soc_codec *codec,
			       int addr_bits, int data_bits,
			       enum snd_soc_control_type control)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(io_types); i++)
		if (io_types[i].addr_bits == addr_bits &&
		    io_types[i].data_bits == data_bits)
			break;
	if (i == ARRAY_SIZE(io_types)) {
		printk(KERN_ERR
		       "No I/O functions for %d bit address %d bit data\n",
		       addr_bits, data_bits);
		return -EINVAL;
	}

	codec->write = io_types[i].write;
	codec->read = io_types[i].read;
	codec->bulk_write_raw = snd_soc_hw_bulk_write_raw;

	switch (control) {
	case SND_SOC_I2C:
#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
		codec->hw_write = (hw_write_t)i2c_master_send;
#endif
		if (io_types[i].i2c_read)
			codec->hw_read = io_types[i].i2c_read;

		codec->control_data = container_of(codec->dev,
						   struct i2c_client,
						   dev);
		break;

	case SND_SOC_SPI:
#ifdef CONFIG_SPI_MASTER
		codec->hw_write = do_spi_write;
#endif

		codec->control_data = container_of(codec->dev,
						   struct spi_device,
						   dev);
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_codec_set_cache_io);

static bool snd_soc_set_cache_val(void *base, unsigned int idx,
				  unsigned int val, unsigned int word_size)
{
	switch (word_size) {
	case 1: {
		u8 *cache = base;
		if (cache[idx] == val)
			return true;
		cache[idx] = val;
		break;
	}
	case 2: {
		u16 *cache = base;
		if (cache[idx] == val)
			return true;
		cache[idx] = val;
		break;
	}
	default:
		BUG();
	}
	return false;
}

static unsigned int snd_soc_get_cache_val(const void *base, unsigned int idx,
		unsigned int word_size)
{
	if (!base)
		return -1;

	switch (word_size) {
	case 1: {
		const u8 *cache = base;
		return cache[idx];
	}
	case 2: {
		const u16 *cache = base;
		return cache[idx];
	}
	default:
		BUG();
	}
	/* unreachable */
	return -1;
}

struct snd_soc_rbtree_node {
	struct rb_node node;
	unsigned int reg;
	unsigned int value;
	unsigned int defval;
} __attribute__ ((packed));

struct snd_soc_rbtree_ctx {
	struct rb_root root;
};

static struct snd_soc_rbtree_node *snd_soc_rbtree_lookup(
	struct rb_root *root, unsigned int reg)
{
	struct rb_node *node;
	struct snd_soc_rbtree_node *rbnode;

	node = root->rb_node;
	while (node) {
		rbnode = container_of(node, struct snd_soc_rbtree_node, node);
		if (rbnode->reg < reg)
			node = node->rb_left;
		else if (rbnode->reg > reg)
			node = node->rb_right;
		else
			return rbnode;
	}

	return NULL;
}

static int snd_soc_rbtree_insert(struct rb_root *root,
				 struct snd_soc_rbtree_node *rbnode)
{
	struct rb_node **new, *parent;
	struct snd_soc_rbtree_node *rbnode_tmp;

	parent = NULL;
	new = &root->rb_node;
	while (*new) {
		rbnode_tmp = container_of(*new, struct snd_soc_rbtree_node,
					  node);
		parent = *new;
		if (rbnode_tmp->reg < rbnode->reg)
			new = &((*new)->rb_left);
		else if (rbnode_tmp->reg > rbnode->reg)
			new = &((*new)->rb_right);
		else
			return 0;
	}

	/* insert the node into the rbtree */
	rb_link_node(&rbnode->node, parent, new);
	rb_insert_color(&rbnode->node, root);

	return 1;
}

static int snd_soc_rbtree_cache_sync(struct snd_soc_codec *codec)
{
	struct snd_soc_rbtree_ctx *rbtree_ctx;
	struct rb_node *node;
	struct snd_soc_rbtree_node *rbnode;
	unsigned int val;
	int ret;

	rbtree_ctx = codec->reg_cache;
	for (node = rb_first(&rbtree_ctx->root); node; node = rb_next(node)) {
		rbnode = rb_entry(node, struct snd_soc_rbtree_node, node);
		if (rbnode->value == rbnode->defval)
			continue;
		WARN_ON(codec->writable_register &&
			codec->writable_register(codec, rbnode->reg));
		ret = snd_soc_cache_read(codec, rbnode->reg, &val);
		if (ret)
			return ret;
		codec->cache_bypass = 1;
		ret = snd_soc_write(codec, rbnode->reg, val);
		codec->cache_bypass = 0;
		if (ret)
			return ret;
		dev_dbg(codec->dev, "Synced register %#x, value = %#x\n",
			rbnode->reg, val);
	}

	return 0;
}

static int snd_soc_rbtree_cache_write(struct snd_soc_codec *codec,
				      unsigned int reg, unsigned int value)
{
	struct snd_soc_rbtree_ctx *rbtree_ctx;
	struct snd_soc_rbtree_node *rbnode;

	rbtree_ctx = codec->reg_cache;
	rbnode = snd_soc_rbtree_lookup(&rbtree_ctx->root, reg);
	if (rbnode) {
		if (rbnode->value == value)
			return 0;
		rbnode->value = value;
	} else {
		/* bail out early, no need to create the rbnode yet */
		if (!value)
			return 0;
		/*
		 * for uninitialized registers whose value is changed
		 * from the default zero, create an rbnode and insert
		 * it into the tree.
		 */
		rbnode = kzalloc(sizeof *rbnode, GFP_KERNEL);
		if (!rbnode)
			return -ENOMEM;
		rbnode->reg = reg;
		rbnode->value = value;
		snd_soc_rbtree_insert(&rbtree_ctx->root, rbnode);
	}

	return 0;
}

static int snd_soc_rbtree_cache_read(struct snd_soc_codec *codec,
				     unsigned int reg, unsigned int *value)
{
	struct snd_soc_rbtree_ctx *rbtree_ctx;
	struct snd_soc_rbtree_node *rbnode;

	rbtree_ctx = codec->reg_cache;
	rbnode = snd_soc_rbtree_lookup(&rbtree_ctx->root, reg);
	if (rbnode) {
		*value = rbnode->value;
	} else {
		/* uninitialized registers default to 0 */
		*value = 0;
	}

	return 0;
}

static int snd_soc_rbtree_cache_exit(struct snd_soc_codec *codec)
{
	struct rb_node *next;
	struct snd_soc_rbtree_ctx *rbtree_ctx;
	struct snd_soc_rbtree_node *rbtree_node;

	/* if we've already been called then just return */
	rbtree_ctx = codec->reg_cache;
	if (!rbtree_ctx)
		return 0;

	/* free up the rbtree */
	next = rb_first(&rbtree_ctx->root);
	while (next) {
		rbtree_node = rb_entry(next, struct snd_soc_rbtree_node, node);
		next = rb_next(&rbtree_node->node);
		rb_erase(&rbtree_node->node, &rbtree_ctx->root);
		kfree(rbtree_node);
	}

	/* release the resources */
	kfree(codec->reg_cache);
	codec->reg_cache = NULL;

	return 0;
}

static int snd_soc_rbtree_cache_init(struct snd_soc_codec *codec)
{
	struct snd_soc_rbtree_node *rbtree_node;
	struct snd_soc_rbtree_ctx *rbtree_ctx;
	unsigned int val;
	unsigned int word_size;
	int i;
	int ret;

	codec->reg_cache = kmalloc(sizeof *rbtree_ctx, GFP_KERNEL);
	if (!codec->reg_cache)
		return -ENOMEM;

	rbtree_ctx = codec->reg_cache;
	rbtree_ctx->root = RB_ROOT;

	if (!codec->reg_def_copy)
		return 0;

	/*
	 * populate the rbtree with the initialized registers.  All other
	 * registers will be inserted when they are first modified.
	 */
	word_size = codec->driver->reg_word_size;
	for (i = 0; i < codec->driver->reg_cache_size; ++i) {
		val = snd_soc_get_cache_val(codec->reg_def_copy, i, word_size);
		if (!val)
			continue;
		rbtree_node = kzalloc(sizeof *rbtree_node, GFP_KERNEL);
		if (!rbtree_node) {
			ret = -ENOMEM;
			snd_soc_cache_exit(codec);
			break;
		}
		rbtree_node->reg = i;
		rbtree_node->value = val;
		rbtree_node->defval = val;
		snd_soc_rbtree_insert(&rbtree_ctx->root, rbtree_node);
	}

	return 0;
}

#ifdef CONFIG_SND_SOC_CACHE_LZO
struct snd_soc_lzo_ctx {
	void *wmem;
	void *dst;
	const void *src;
	size_t src_len;
	size_t dst_len;
	size_t decompressed_size;
	unsigned long *sync_bmp;
	int sync_bmp_nbits;
};

#define LZO_BLOCK_NUM 8
static int snd_soc_lzo_block_count(void)
{
	return LZO_BLOCK_NUM;
}

static int snd_soc_lzo_prepare(struct snd_soc_lzo_ctx *lzo_ctx)
{
	lzo_ctx->wmem = kmalloc(LZO1X_MEM_COMPRESS, GFP_KERNEL);
	if (!lzo_ctx->wmem)
		return -ENOMEM;
	return 0;
}

static int snd_soc_lzo_compress(struct snd_soc_lzo_ctx *lzo_ctx)
{
	size_t compress_size;
	int ret;

	ret = lzo1x_1_compress(lzo_ctx->src, lzo_ctx->src_len,
			       lzo_ctx->dst, &compress_size, lzo_ctx->wmem);
	if (ret != LZO_E_OK || compress_size > lzo_ctx->dst_len)
		return -EINVAL;
	lzo_ctx->dst_len = compress_size;
	return 0;
}

static int snd_soc_lzo_decompress(struct snd_soc_lzo_ctx *lzo_ctx)
{
	size_t dst_len;
	int ret;

	dst_len = lzo_ctx->dst_len;
	ret = lzo1x_decompress_safe(lzo_ctx->src, lzo_ctx->src_len,
				    lzo_ctx->dst, &dst_len);
	if (ret != LZO_E_OK || dst_len != lzo_ctx->dst_len)
		return -EINVAL;
	return 0;
}

static int snd_soc_lzo_compress_cache_block(struct snd_soc_codec *codec,
		struct snd_soc_lzo_ctx *lzo_ctx)
{
	int ret;

	lzo_ctx->dst_len = lzo1x_worst_compress(PAGE_SIZE);
	lzo_ctx->dst = kmalloc(lzo_ctx->dst_len, GFP_KERNEL);
	if (!lzo_ctx->dst) {
		lzo_ctx->dst_len = 0;
		return -ENOMEM;
	}

	ret = snd_soc_lzo_compress(lzo_ctx);
	if (ret < 0)
		return ret;
	return 0;
}

static int snd_soc_lzo_decompress_cache_block(struct snd_soc_codec *codec,
		struct snd_soc_lzo_ctx *lzo_ctx)
{
	int ret;

	lzo_ctx->dst_len = lzo_ctx->decompressed_size;
	lzo_ctx->dst = kmalloc(lzo_ctx->dst_len, GFP_KERNEL);
	if (!lzo_ctx->dst) {
		lzo_ctx->dst_len = 0;
		return -ENOMEM;
	}

	ret = snd_soc_lzo_decompress(lzo_ctx);
	if (ret < 0)
		return ret;
	return 0;
}

static inline int snd_soc_lzo_get_blkindex(struct snd_soc_codec *codec,
		unsigned int reg)
{
	const struct snd_soc_codec_driver *codec_drv;

	codec_drv = codec->driver;
	return (reg * codec_drv->reg_word_size) /
	       DIV_ROUND_UP(codec->reg_size, snd_soc_lzo_block_count());
}

static inline int snd_soc_lzo_get_blkpos(struct snd_soc_codec *codec,
		unsigned int reg)
{
	const struct snd_soc_codec_driver *codec_drv;

	codec_drv = codec->driver;
	return reg % (DIV_ROUND_UP(codec->reg_size, snd_soc_lzo_block_count()) /
		      codec_drv->reg_word_size);
}

static inline int snd_soc_lzo_get_blksize(struct snd_soc_codec *codec)
{
	const struct snd_soc_codec_driver *codec_drv;

	codec_drv = codec->driver;
	return DIV_ROUND_UP(codec->reg_size, snd_soc_lzo_block_count());
}

static int snd_soc_lzo_cache_sync(struct snd_soc_codec *codec)
{
	struct snd_soc_lzo_ctx **lzo_blocks;
	unsigned int val;
	int i;
	int ret;

	lzo_blocks = codec->reg_cache;
	for_each_set_bit(i, lzo_blocks[0]->sync_bmp, lzo_blocks[0]->sync_bmp_nbits) {
		WARN_ON(codec->writable_register &&
			codec->writable_register(codec, i));
		ret = snd_soc_cache_read(codec, i, &val);
		if (ret)
			return ret;
		codec->cache_bypass = 1;
		ret = snd_soc_write(codec, i, val);
		codec->cache_bypass = 0;
		if (ret)
			return ret;
		dev_dbg(codec->dev, "Synced register %#x, value = %#x\n",
			i, val);
	}

	return 0;
}

static int snd_soc_lzo_cache_write(struct snd_soc_codec *codec,
				   unsigned int reg, unsigned int value)
{
	struct snd_soc_lzo_ctx *lzo_block, **lzo_blocks;
	int ret, blkindex, blkpos;
	size_t blksize, tmp_dst_len;
	void *tmp_dst;

	/* index of the compressed lzo block */
	blkindex = snd_soc_lzo_get_blkindex(codec, reg);
	/* register index within the decompressed block */
	blkpos = snd_soc_lzo_get_blkpos(codec, reg);
	/* size of the compressed block */
	blksize = snd_soc_lzo_get_blksize(codec);
	lzo_blocks = codec->reg_cache;
	lzo_block = lzo_blocks[blkindex];

	/* save the pointer and length of the compressed block */
	tmp_dst = lzo_block->dst;
	tmp_dst_len = lzo_block->dst_len;

	/* prepare the source to be the compressed block */
	lzo_block->src = lzo_block->dst;
	lzo_block->src_len = lzo_block->dst_len;

	/* decompress the block */
	ret = snd_soc_lzo_decompress_cache_block(codec, lzo_block);
	if (ret < 0) {
		kfree(lzo_block->dst);
		goto out;
	}

	/* write the new value to the cache */
	if (snd_soc_set_cache_val(lzo_block->dst, blkpos, value,
				  codec->driver->reg_word_size)) {
		kfree(lzo_block->dst);
		goto out;
	}

	/* prepare the source to be the decompressed block */
	lzo_block->src = lzo_block->dst;
	lzo_block->src_len = lzo_block->dst_len;

	/* compress the block */
	ret = snd_soc_lzo_compress_cache_block(codec, lzo_block);
	if (ret < 0) {
		kfree(lzo_block->dst);
		kfree(lzo_block->src);
		goto out;
	}

	/* set the bit so we know we have to sync this register */
	set_bit(reg, lzo_block->sync_bmp);
	kfree(tmp_dst);
	kfree(lzo_block->src);
	return 0;
out:
	lzo_block->dst = tmp_dst;
	lzo_block->dst_len = tmp_dst_len;
	return ret;
}

static int snd_soc_lzo_cache_read(struct snd_soc_codec *codec,
				  unsigned int reg, unsigned int *value)
{
	struct snd_soc_lzo_ctx *lzo_block, **lzo_blocks;
	int ret, blkindex, blkpos;
	size_t blksize, tmp_dst_len;
	void *tmp_dst;

	*value = 0;
	/* index of the compressed lzo block */
	blkindex = snd_soc_lzo_get_blkindex(codec, reg);
	/* register index within the decompressed block */
	blkpos = snd_soc_lzo_get_blkpos(codec, reg);
	/* size of the compressed block */
	blksize = snd_soc_lzo_get_blksize(codec);
	lzo_blocks = codec->reg_cache;
	lzo_block = lzo_blocks[blkindex];

	/* save the pointer and length of the compressed block */
	tmp_dst = lzo_block->dst;
	tmp_dst_len = lzo_block->dst_len;

	/* prepare the source to be the compressed block */
	lzo_block->src = lzo_block->dst;
	lzo_block->src_len = lzo_block->dst_len;

	/* decompress the block */
	ret = snd_soc_lzo_decompress_cache_block(codec, lzo_block);
	if (ret >= 0)
		/* fetch the value from the cache */
		*value = snd_soc_get_cache_val(lzo_block->dst, blkpos,
					       codec->driver->reg_word_size);

	kfree(lzo_block->dst);
	/* restore the pointer and length of the compressed block */
	lzo_block->dst = tmp_dst;
	lzo_block->dst_len = tmp_dst_len;
	return 0;
}

static int snd_soc_lzo_cache_exit(struct snd_soc_codec *codec)
{
	struct snd_soc_lzo_ctx **lzo_blocks;
	int i, blkcount;

	lzo_blocks = codec->reg_cache;
	if (!lzo_blocks)
		return 0;

	blkcount = snd_soc_lzo_block_count();
	/*
	 * the pointer to the bitmap used for syncing the cache
	 * is shared amongst all lzo_blocks.  Ensure it is freed
	 * only once.
	 */
	if (lzo_blocks[0])
		kfree(lzo_blocks[0]->sync_bmp);
	for (i = 0; i < blkcount; ++i) {
		if (lzo_blocks[i]) {
			kfree(lzo_blocks[i]->wmem);
			kfree(lzo_blocks[i]->dst);
		}
		/* each lzo_block is a pointer returned by kmalloc or NULL */
		kfree(lzo_blocks[i]);
	}
	kfree(lzo_blocks);
	codec->reg_cache = NULL;
	return 0;
}

static int snd_soc_lzo_cache_init(struct snd_soc_codec *codec)
{
	struct snd_soc_lzo_ctx **lzo_blocks;
	size_t bmp_size;
	const struct snd_soc_codec_driver *codec_drv;
	int ret, tofree, i, blksize, blkcount;
	const char *p, *end;
	unsigned long *sync_bmp;

	ret = 0;
	codec_drv = codec->driver;

	/*
	 * If we have not been given a default register cache
	 * then allocate a dummy zero-ed out region, compress it
	 * and remember to free it afterwards.
	 */
	tofree = 0;
	if (!codec->reg_def_copy)
		tofree = 1;

	if (!codec->reg_def_copy) {
		codec->reg_def_copy = kzalloc(codec->reg_size, GFP_KERNEL);
		if (!codec->reg_def_copy)
			return -ENOMEM;
	}

	blkcount = snd_soc_lzo_block_count();
	codec->reg_cache = kzalloc(blkcount * sizeof *lzo_blocks,
				   GFP_KERNEL);
	if (!codec->reg_cache) {
		ret = -ENOMEM;
		goto err_tofree;
	}
	lzo_blocks = codec->reg_cache;

	/*
	 * allocate a bitmap to be used when syncing the cache with
	 * the hardware.  Each time a register is modified, the corresponding
	 * bit is set in the bitmap, so we know that we have to sync
	 * that register.
	 */
	bmp_size = codec_drv->reg_cache_size;
	sync_bmp = kmalloc(BITS_TO_LONGS(bmp_size) * sizeof(long),
			   GFP_KERNEL);
	if (!sync_bmp) {
		ret = -ENOMEM;
		goto err;
	}
	bitmap_zero(sync_bmp, bmp_size);

	/* allocate the lzo blocks and initialize them */
	for (i = 0; i < blkcount; ++i) {
		lzo_blocks[i] = kzalloc(sizeof **lzo_blocks,
					GFP_KERNEL);
		if (!lzo_blocks[i]) {
			kfree(sync_bmp);
			ret = -ENOMEM;
			goto err;
		}
		lzo_blocks[i]->sync_bmp = sync_bmp;
		lzo_blocks[i]->sync_bmp_nbits = bmp_size;
		/* alloc the working space for the compressed block */
		ret = snd_soc_lzo_prepare(lzo_blocks[i]);
		if (ret < 0)
			goto err;
	}

	blksize = snd_soc_lzo_get_blksize(codec);
	p = codec->reg_def_copy;
	end = codec->reg_def_copy + codec->reg_size;
	/* compress the register map and fill the lzo blocks */
	for (i = 0; i < blkcount; ++i, p += blksize) {
		lzo_blocks[i]->src = p;
		if (p + blksize > end)
			lzo_blocks[i]->src_len = end - p;
		else
			lzo_blocks[i]->src_len = blksize;
		ret = snd_soc_lzo_compress_cache_block(codec,
						       lzo_blocks[i]);
		if (ret < 0)
			goto err;
		lzo_blocks[i]->decompressed_size =
			lzo_blocks[i]->src_len;
	}

	if (tofree) {
		kfree(codec->reg_def_copy);
		codec->reg_def_copy = NULL;
	}
	return 0;
err:
	snd_soc_cache_exit(codec);
err_tofree:
	if (tofree) {
		kfree(codec->reg_def_copy);
		codec->reg_def_copy = NULL;
	}
	return ret;
}
#endif

static int snd_soc_flat_cache_sync(struct snd_soc_codec *codec)
{
	int i;
	int ret;
	const struct snd_soc_codec_driver *codec_drv;
	unsigned int val;

	codec_drv = codec->driver;
	for (i = 0; i < codec_drv->reg_cache_size; ++i) {
		WARN_ON(codec->writable_register &&
			codec->writable_register(codec, i));
		ret = snd_soc_cache_read(codec, i, &val);
		if (ret)
			return ret;
		if (codec->reg_def_copy)
			if (snd_soc_get_cache_val(codec->reg_def_copy,
						  i, codec_drv->reg_word_size) == val)
				continue;
		ret = snd_soc_write(codec, i, val);
		if (ret)
			return ret;
		dev_dbg(codec->dev, "Synced register %#x, value = %#x\n",
			i, val);
	}
	return 0;
}

static int snd_soc_flat_cache_write(struct snd_soc_codec *codec,
				    unsigned int reg, unsigned int value)
{
	snd_soc_set_cache_val(codec->reg_cache, reg, value,
			      codec->driver->reg_word_size);
	return 0;
}

static int snd_soc_flat_cache_read(struct snd_soc_codec *codec,
				   unsigned int reg, unsigned int *value)
{
	*value = snd_soc_get_cache_val(codec->reg_cache, reg,
				       codec->driver->reg_word_size);
	return 0;
}

static int snd_soc_flat_cache_exit(struct snd_soc_codec *codec)
{
	if (!codec->reg_cache)
		return 0;
	kfree(codec->reg_cache);
	codec->reg_cache = NULL;
	return 0;
}

static int snd_soc_flat_cache_init(struct snd_soc_codec *codec)
{
	const struct snd_soc_codec_driver *codec_drv;

	codec_drv = codec->driver;

	if (codec->reg_def_copy)
		codec->reg_cache = kmemdup(codec->reg_def_copy,
					   codec->reg_size, GFP_KERNEL);
	else
		codec->reg_cache = kzalloc(codec->reg_size, GFP_KERNEL);
	if (!codec->reg_cache)
		return -ENOMEM;

	return 0;
}

/* an array of all supported compression types */
static const struct snd_soc_cache_ops cache_types[] = {
	/* Flat *must* be the first entry for fallback */
	{
		.id = SND_SOC_FLAT_COMPRESSION,
		.name = "flat",
		.init = snd_soc_flat_cache_init,
		.exit = snd_soc_flat_cache_exit,
		.read = snd_soc_flat_cache_read,
		.write = snd_soc_flat_cache_write,
		.sync = snd_soc_flat_cache_sync
	},
#ifdef CONFIG_SND_SOC_CACHE_LZO
	{
		.id = SND_SOC_LZO_COMPRESSION,
		.name = "LZO",
		.init = snd_soc_lzo_cache_init,
		.exit = snd_soc_lzo_cache_exit,
		.read = snd_soc_lzo_cache_read,
		.write = snd_soc_lzo_cache_write,
		.sync = snd_soc_lzo_cache_sync
	},
#endif
	{
		.id = SND_SOC_RBTREE_COMPRESSION,
		.name = "rbtree",
		.init = snd_soc_rbtree_cache_init,
		.exit = snd_soc_rbtree_cache_exit,
		.read = snd_soc_rbtree_cache_read,
		.write = snd_soc_rbtree_cache_write,
		.sync = snd_soc_rbtree_cache_sync
	}
};

int snd_soc_cache_init(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cache_types); ++i)
		if (cache_types[i].id == codec->compress_type)
			break;

	/* Fall back to flat compression */
	if (i == ARRAY_SIZE(cache_types)) {
		dev_warn(codec->dev, "Could not match compress type: %d\n",
			 codec->compress_type);
		i = 0;
	}

	mutex_init(&codec->cache_rw_mutex);
	codec->cache_ops = &cache_types[i];

	if (codec->cache_ops->init) {
		if (codec->cache_ops->name)
			dev_dbg(codec->dev, "Initializing %s cache for %s codec\n",
				codec->cache_ops->name, codec->name);
		return codec->cache_ops->init(codec);
	}
	return -ENOSYS;
}

/*
 * NOTE: keep in mind that this function might be called
 * multiple times.
 */
int snd_soc_cache_exit(struct snd_soc_codec *codec)
{
	if (codec->cache_ops && codec->cache_ops->exit) {
		if (codec->cache_ops->name)
			dev_dbg(codec->dev, "Destroying %s cache for %s codec\n",
				codec->cache_ops->name, codec->name);
		return codec->cache_ops->exit(codec);
	}
	return -ENOSYS;
}

/**
 * snd_soc_cache_read: Fetch the value of a given register from the cache.
 *
 * @codec: CODEC to configure.
 * @reg: The register index.
 * @value: The value to be returned.
 */
int snd_soc_cache_read(struct snd_soc_codec *codec,
		       unsigned int reg, unsigned int *value)
{
	int ret;

	mutex_lock(&codec->cache_rw_mutex);

	if (value && codec->cache_ops && codec->cache_ops->read) {
		ret = codec->cache_ops->read(codec, reg, value);
		mutex_unlock(&codec->cache_rw_mutex);
		return ret;
	}

	mutex_unlock(&codec->cache_rw_mutex);
	return -ENOSYS;
}
EXPORT_SYMBOL_GPL(snd_soc_cache_read);

/**
 * snd_soc_cache_write: Set the value of a given register in the cache.
 *
 * @codec: CODEC to configure.
 * @reg: The register index.
 * @value: The new register value.
 */
int snd_soc_cache_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int value)
{
	int ret;

	mutex_lock(&codec->cache_rw_mutex);

	if (codec->cache_ops && codec->cache_ops->write) {
		ret = codec->cache_ops->write(codec, reg, value);
		mutex_unlock(&codec->cache_rw_mutex);
		return ret;
	}

	mutex_unlock(&codec->cache_rw_mutex);
	return -ENOSYS;
}
EXPORT_SYMBOL_GPL(snd_soc_cache_write);

/**
 * snd_soc_cache_sync: Sync the register cache with the hardware.
 *
 * @codec: CODEC to configure.
 *
 * Any registers that should not be synced should be marked as
 * volatile.  In general drivers can choose not to use the provided
 * syncing functionality if they so require.
 */
int snd_soc_cache_sync(struct snd_soc_codec *codec)
{
	int ret;
	const char *name;

	if (!codec->cache_sync) {
		return 0;
	}

	if (!codec->cache_ops || !codec->cache_ops->sync)
		return -ENOSYS;

	if (codec->cache_ops->name)
		name = codec->cache_ops->name;
	else
		name = "unknown";

	if (codec->cache_ops->name)
		dev_dbg(codec->dev, "Syncing %s cache for %s codec\n",
			codec->cache_ops->name, codec->name);
	trace_snd_soc_cache_sync(codec, name, "start");
	ret = codec->cache_ops->sync(codec);
	if (!ret)
		codec->cache_sync = 0;
	trace_snd_soc_cache_sync(codec, name, "end");
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_cache_sync);

static int snd_soc_get_reg_access_index(struct snd_soc_codec *codec,
					unsigned int reg)
{
	const struct snd_soc_codec_driver *codec_drv;
	unsigned int min, max, index;

	codec_drv = codec->driver;
	min = 0;
	max = codec_drv->reg_access_size - 1;
	do {
		index = (min + max) / 2;
		if (codec_drv->reg_access_default[index].reg == reg)
			return index;
		if (codec_drv->reg_access_default[index].reg < reg)
			min = index + 1;
		else
			max = index;
	} while (min <= max);
	return -1;
}

int snd_soc_default_volatile_register(struct snd_soc_codec *codec,
				      unsigned int reg)
{
	int index;

	if (reg >= codec->driver->reg_cache_size)
		return 1;
	index = snd_soc_get_reg_access_index(codec, reg);
	if (index < 0)
		return 0;
	return codec->driver->reg_access_default[index].vol;
}
EXPORT_SYMBOL_GPL(snd_soc_default_volatile_register);

int snd_soc_default_readable_register(struct snd_soc_codec *codec,
				      unsigned int reg)
{
	int index;

	if (reg >= codec->driver->reg_cache_size)
		return 1;
	index = snd_soc_get_reg_access_index(codec, reg);
	if (index < 0)
		return 0;
	return codec->driver->reg_access_default[index].read;
}
EXPORT_SYMBOL_GPL(snd_soc_default_readable_register);

int snd_soc_default_writable_register(struct snd_soc_codec *codec,
				      unsigned int reg)
{
	int index;

	if (reg >= codec->driver->reg_cache_size)
		return 1;
	index = snd_soc_get_reg_access_index(codec, reg);
	if (index < 0)
		return 0;
	return codec->driver->reg_access_default[index].write;
}
EXPORT_SYMBOL_GPL(snd_soc_default_writable_register);
