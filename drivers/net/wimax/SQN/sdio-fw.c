/* * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Dmitriy Chumak <chumakd@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */


#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/firmware.h>
#include <linux/byteorder/generic.h>

#include "sdio.h"
#include "msg.h"
#include "sdio-sqn.h"
#include "sdio-netdev.h"
#include "version.h"
#include "sdio-fw.h"

char	*fw1130_name = "sqn1130.bin";
char	*fw1210_name = "sqn1210.bin";


/* Tag, Lenght, Value struct */
struct sqn_tlv {
	u32	tag;
#define SWM_INFO_TAG_SQN_ROOT		0x80000000	/* SEQUANS root tag */
#define SWM_INFO_TAG_SQN_MEMCPY		0x80000005	/* SEQUANS memcpy tag */
#define SWM_INFO_TAG_SQN_MEMSET		0x80000006	/* SEQUANS memset tag */
#define SWM_INFO_TAG_SQN_BOOTROM_GROUP	0x80040000
#define SWM_INFO_TAG_SQN_ID_GROUP	0x80010000	/* SEQUANS identification group tag */
#define SWM_INFO_TAG_SQN_MAC_ADDRESS	0x80010010	/* SEQUANS mac address tag */
	u32	length;
	u8	value[0];
};


/* body of SWM_INFO_TAG_SQN_MEMCPY tag */
struct sqn_tag_memcpy {
	u32	address;
	u32	access_size;
	u32	data_size;
	u8	data[0];
};


/* body of SWM_INFO_TAG_SQN_MEMSET tag */
struct sqn_tag_memset {
	u32	address;
	u32	access_size;
	u32	size;
	u8	pattern;
};


#define SQN_1130_SDRAM_BASE      0x00000000
#define SQN_1130_SDRAM_END       0x03FFFFFF
#define SQN_1130_SDRAMCTL_BASE   0x4B400000
#define SQN_1130_SDRAMCTL_END    0x4B4003FF

#define SQN_1210_SDRAM_BASE      0x00000000
#define SQN_1210_SDRAM_END       0x07FFFFFF
#define SQN_1210_SDRAMCTL_BASE   0x20002000
#define SQN_1210_SDRAMCTL_END    0x2000207F

static int is_good_ahb_address(u32 address, enum sqn_card_version card_version)
{
	u32 sdram_base = 0;
	u32 sdram_end = 0;
	u32 sdram_ctl_base = 0;
	u32 sdram_ctl_end = 0;
	int status = 0;

	sqn_pr_enter();

	if (address % 4)
		return 0;

	if (SQN_1130 == card_version) {
		sqn_pr_dbg("using 1130 AHB address boundaries\n");
		sdram_base	= SQN_1130_SDRAM_BASE;
		sdram_end	= SQN_1130_SDRAM_END;
		sdram_ctl_base	= SQN_1130_SDRAMCTL_BASE;
		sdram_ctl_end	= SQN_1130_SDRAMCTL_END;
	} else if (SQN_1210 == card_version) {
		sqn_pr_dbg("using 1210 AHB address boundaries\n");
		sdram_base	= SQN_1210_SDRAM_BASE;
		sdram_end	= SQN_1210_SDRAM_END;
		sdram_ctl_base	= SQN_1210_SDRAMCTL_BASE;
		sdram_ctl_end	= SQN_1210_SDRAMCTL_END;
	} else {
		sqn_pr_warn("Can't check AHB address because of unknown"
			" card version\n");
		status = 0;
		goto out;
	}

	status = ((sdram_base <= address && address < sdram_end)
			|| (sdram_ctl_base <= address && address < sdram_ctl_end));
out:
	sqn_pr_leave();
	return status;
}
 
// Fix big buffer allocation problem during Firmware loading
/**
 *	sqn_alloc_big_buffer - tries to alloc a big buffer with kmalloc
 *	@buf: pointer to buffer
 *	@size: required buffer size
 *	@gfp_flags: GFP_* flags
 *
 *	Tries to allocate a buffer of requested size with kmalloc. If it fails,
 *	then decrease buffer size in two times (adjusting the new size to be a
 *	multiple of 4) and try again. Use 6 retries in case of failures, after
 *	this give up and try to alloc 4KB buffer if requested size bigger than
 *	4KB, otherwise allocate nothing and return 0.
 *
 *  @return a real size of allocated buffer or 0 if allocation failed
 * 
 *   Normal: 3912*4kB 4833*8kB 0*16kB 0*32kB 0*64kB 0*128kB 0*256kB 0*512kB 0*1024kB 0*2048kB 0*4096kB = 54312kB
 */

static size_t sqn_alloc_big_buffer(u8 **buf, size_t size, gfp_t gfp_flags)
{
	size_t	real_size = size;
	// int	retries   = 6;
    // int	retries   = 3;

	sqn_pr_enter();

	/* Try to allocate buffer of requested size, if it failes try to
	 * allocate a twice smaller buffer. Repeat this <retries> number of
	 * times. */
	/*
	do
	{
		*buf = kmalloc(real_size, gfp_flags);
		printk("%s: kmalloc %d in %x trial:%d\n", __func__, real_size, *buf, retries); 

		if (!(*buf)) {
            printk("%s: kmalloc %d failed, trial:%d\n", __func__, real_size, retries); 
			// real_size /= 2;
            real_size /= 4;
			// adjust the size to be a multiple of 4
			real_size += real_size % 4 ? 4 - real_size % 4 : 0;
		}
	} while (retries-- > 0 && !(*buf));
    */

	// If all retries failed, then allocate 4KB buffer
	if (!(*buf)) {
		real_size = 8 * 1024;
		if (size >= real_size) {
			*buf = kmalloc(real_size, gfp_flags);
			// printk("%s: kmalloc %d in %x\n", __func__, real_size, *buf); 

			// If it also failed, then just return 0, indicating
			// that we failed to alloc buffer
			if (!(*buf))
				real_size = 0;
		} else {
			// We should _not_ return buffer bigger than requested
			// real_size = 0;
						
			// printk("%s: We should _not_ return buffer bigger than requested size:%d real_size:%d\n", __func__, size, real_size); 
			*buf = kmalloc(size, gfp_flags);
			real_size = size;			
		}
	} 

	sqn_pr_leave();

	return real_size;
}

#define SQN_SDIO_ADA_ADDR	0x00002060
#define SQN_SDIO_ADA_RDWR	0x00002064


static int write_data(struct sdio_func *func, u32 addr, void *data
	, u32 size, u32 access_size)
{
	int rv = 0, retry = 0;
	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);

	sqn_pr_enter();
	sdio_claim_host(func);

	if (is_good_ahb_address(addr, sqn_card->version)
		&& 0 == (size % 4) && 4 == access_size)
	{
		/* write data using AHB */
		u8 *buf = 0;
		size_t buf_size = 0;
		u32 written_size = 0;

#ifdef DEBUG
		u8 *read_data  = 0;
#endif

		sqn_pr_dbg("write data using AHB\n");
		sdio_writel(func, addr, SQN_SDIO_ADA_ADDR, &rv);
		if (rv) {
			sqn_pr_dbg("can't set SQN_SDIO_ADA_ADDR register\n");
			goto out;
		}
		sqn_pr_dbg("after SQN_SDIO_ADA_ADDR\n");

		written_size = 0;
		buf_size = sqn_alloc_big_buffer(&buf, size, GFP_KERNEL | GFP_DMA);
		if (!buf) {
			sqn_pr_err("failed to allocate buffer of %u bytes\n", size);
			goto out;
		}

		do {
			memcpy(buf, data + written_size, buf_size);
			rv = sdio_writesb(func, SQN_SDIO_ADA_RDWR, buf, buf_size);
			if (rv) {
				sqn_pr_dbg("can't write to SQN_SDIO_ADA_RDWR register\n");
				goto out;
			}
			written_size += buf_size;
			if (written_size + buf_size > size)
				buf_size = size - written_size;
		} while (written_size < size);
		kfree(buf);

		/*
		 * Workaround when sdio_writesb doesn't work because DMA
		 * alignment
		 */
		/*
		int i = 0;
		for (; i < size/4; ++i) {
			sdio_writel(func, *((u32*)data + i), SQN_SDIO_ADA_RDWR, &rv);
			if (rv) {
				sqn_pr_dbg("can't write to SQN_SDIO_ADA_RDWR register\n");
				goto out;
			}
		}
		*/

		sqn_pr_dbg("after SQN_SDIO_ADA_RDWR\n");

		/* ******** only for debugging ******** */
		/* validate written data */
/* #ifdef DEBUG */
#if 0
		sqn_pr_dbg("reading data using AHB\n");
		sdio_writel(func, addr, SQN_SDIO_ADA_ADDR, &rv);
		if (rv) {
			sqn_pr_dbg("can't set SQN_SDIO_ADA_ADDR register\n");
			goto out;
		}
		sqn_pr_dbg("after SQN_SDIO_ADA_ADDR\n");

		read_data = kmalloc(size, GFP_KERNEL);
		rv = sdio_readsb(func, read_data, SQN_SDIO_ADA_RDWR, size);
		if (rv) {
			sqn_pr_dbg("can't read from SQN_SDIO_ADA_RDWR register\n");
			kfree(read_data);
			goto out;
		}

		if (memcmp(data, read_data, size))
			sqn_pr_dbg("WARNING: written data are __not__ equal\n");
		else
			sqn_pr_dbg("OK: written data are equal\n");

		kfree(read_data);
#endif /* DEBUG */
		/* ******** only for debugging ******** */

	} else if (4 == access_size && size >= 4) {
		/* write data using CMD53 */
		sqn_pr_dbg("write data using CMD53\n");
		rv = sdio_memcpy_toio(func, addr, data , size);
	} else {
		/* write data using CMD52 */
		/* not implemented yet, so we use CMD53 */
		/* rv = sdio_memcpy_toio(func, addr, data , size); */
		int i = 0;
		sqn_pr_dbg("write data using CMD52\n");
		for (i = 0; i < size; ++i) {
			sdio_writeb(func, *((u8*)data + i), addr + i, &rv);
			if (rv) {
				sqn_pr_dbg("can't write 1 byte to %xh addr using CMD52\n"
					, addr + i);
				goto out;
			}
		}
	}

out:
	sdio_release_host(func);
	sqn_pr_leave();
	return rv;
}


static int sqn_handle_memcpy_tag(struct sdio_func *func
	, struct sqn_tag_memcpy * mcpy_tag)
{
	int rv = 0;

	sqn_pr_enter();

	/*
	 * Convert values accordingly to platform "endianes"
	 * (big or little endian) because bootstrapper file
	 * data is big endian
	 */
	mcpy_tag->address = be32_to_cpu(mcpy_tag->address);
	mcpy_tag->access_size = be32_to_cpu(mcpy_tag->access_size);
	mcpy_tag->data_size = be32_to_cpu(mcpy_tag->data_size);

	/* sqn_pr_dbg("----------------------------------------\n"); */
	sqn_pr_dbg("address: 0x%02X access_size: %u data_size: %u\n"
			, mcpy_tag->address, mcpy_tag->access_size
			, mcpy_tag->data_size);
	/* sqn_pr_dbg_dump("|", mcpy_tag->data, mcpy_tag->data_size); */

	rv = write_data(func, mcpy_tag->address, mcpy_tag->data
		, mcpy_tag->data_size, mcpy_tag->access_size);

	sqn_pr_leave();
	return rv;
}


static int sqn_handle_memset_tag(struct sdio_func *func
	, struct sqn_tag_memset * mset_tag)
{
	int rv = 0;
	u8 *buf = 0;
	const u32 buf_size = 1024;
	u32 left_bytes = 0;

	sqn_pr_enter();

	/*
	 * Convert values accordingly to platform "endianes"
	 * (big or little endian) because bootstrapper file
	 * data is big endian
	 */
	mset_tag->address = be32_to_cpu(mset_tag->address);
	mset_tag->access_size = be32_to_cpu(mset_tag->access_size);
	mset_tag->size = be32_to_cpu(mset_tag->size);

	/* sqn_pr_dbg("----------------------------------------\n"); */
	sqn_pr_dbg("address: 0x%02X access_size: %u size: %u pattern 0x%02X\n"
			, mset_tag->address, mset_tag->access_size
			, mset_tag->size, mset_tag->pattern);

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (0 == buf)
		return -ENOMEM;

	memset(buf, mset_tag->pattern, buf_size);

	left_bytes = mset_tag->size;

	while (left_bytes) {
		u32 bytes_to_write = min(buf_size, left_bytes);
		rv = write_data(func, mset_tag->address, buf, bytes_to_write,
			mset_tag->access_size);
		if (rv)
			goto out;
		left_bytes -= bytes_to_write;
	}

out:
	kfree(buf);
	sqn_pr_leave();
	return rv;
}


static int char_to_int(u8 c)
{
	int rv = 0;

	if ('0' <= c && c <= '9') {
		rv = c - '0';
	} else if ('a' <= c && c <= 'f') {
		rv = c - 'a' + 0xA;
	} else if ('A' <= c && c <= 'F') {
		rv = c - 'A' + 0xA;
	} else {
		rv = -1;
	}

	return rv;
}


static int get_mac_addr_from_str(u8 *data, u32 length, u8 *result)
{
	int rv = 0;
	int i = 0;

	sqn_pr_enter();

	if (0 == length) {
		rv = -1;
		goto out;
	}

	/*
	 * Check if we have delimiters on appropriate places:
	 *
	 * X X : X X : X X : X X  :  X  X  :  X  X
	 * 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
	 */

	if ( !( ( ':' == data[2] || '-' == data[2])
		&& ( ':' == data[5] || '-' == data[5])
		&& ( ':' == data[8] || '-' == data[8])
		&& ( ':' == data[11] || '-' == data[11])
		&& ( ':' == data[14] || '-' == data[14]) ))
	{
		sqn_pr_err("can't get mac address from firmware"
			" - incorrect mac address\n");
		rv = -1;
		goto out;
	}

	i = 0;
	while (i < length) {
		int high = 0;
		int low = 0;

		if ((high = char_to_int(data[i])) >= 0
			&& (low = char_to_int(data[i + 1])) >= 0)
		{
			result[i/3] = low;
			result[i/3] |= high << 4;
		} else {
			sqn_pr_err("can't get mac address from firmware"
					" - incorrect mac address\n");
			rv = -1;
			goto out;
		}

		i += 3;
	}

out:
	if (length > 0) {
		data[length - 1] = 0;
		sqn_pr_dbg("mac addr string: %s\n", data);
	}
	sqn_pr_leave();
	return rv;
}


static int sqn_handle_mac_addr_tag(struct sdio_func *func, u8 *data, u32 length)
{
	int rv = 0;
	struct sqn_private *priv =
		((struct sqn_sdio_card *)sdio_get_drvdata(func))->priv;

	sqn_pr_enter();

	/*
	 * This tag could contain one or two mac addresses in string
	 * form, delimited by some symbol (space or something else).
	 * Each mac address written as a string has constant length.
	 * Thus we can determine the number of mac addresses by the
	 * length of the tag:
	 *
	 * mac addr length in string form: XX:XX:XX:XX:XX:XX = 17 bytes
	 * tag length: 17 bytes [ + 1 byte + 17 bytes ]
	 */

#define MAC_ADDR_STRING_LEN	17

	/*
	 * If we have only one mac addr we should increment it by one
	 * and use it.
	 * If we have two mac addresses we should use a second one.
	 */

	if (MAC_ADDR_STRING_LEN <= length
		&& length < 2 * MAC_ADDR_STRING_LEN + 1)
	{
		sqn_pr_dbg("single mac address\n");
		/* we have only one mac addr */
		get_mac_addr_from_str(data, length, priv->mac_addr);

		// Andrew 0720
		// ++(priv->mac_addr[ETH_ALEN - 1])
		// real MAC: 38:E6:D8:86:00:00 
		// hboot will store: 38:E6:D8:85:FF:FF (minus 1)
		// sdio need to recovery it by plusing 1: 38:E6:D8:86:00:00 (plus 1)

		if ((++(priv->mac_addr[ETH_ALEN - 1])) == 0x00)
			if ((++(priv->mac_addr[ETH_ALEN - 2])) == 0x00)
				if ((++(priv->mac_addr[ETH_ALEN - 3])) == 0x00)
					if ((++(priv->mac_addr[ETH_ALEN - 4])) == 0x00)
						if ((++(priv->mac_addr[ETH_ALEN - 5])) == 0x00)
							++(priv->mac_addr[ETH_ALEN - 6]);

	}
	else if (2 * MAC_ADDR_STRING_LEN + 1 == length) { /* we have two macs */
		sqn_pr_dbg("two mac addresses, using second\n");
		get_mac_addr_from_str(data + MAC_ADDR_STRING_LEN + 1
			, length - (MAC_ADDR_STRING_LEN + 1), priv->mac_addr);
	}
	else { /* incorrect data length */
		sqn_pr_err("can't get mac address from bootloader"
			" - incorrect mac address length\n");
		rv = -1;
		goto out;
	}

	sqn_pr_info("setting MAC address from bootloader: "
		"%02x:%02x:%02x:%02x:%02x:%02x\n", priv->mac_addr[0]
		, priv->mac_addr[1], priv->mac_addr[2], priv->mac_addr[3]
		, priv->mac_addr[4], priv->mac_addr[5]);

out:
	sqn_pr_leave();
	return rv;
}


/** sqn_load_bootstrapper - reads a binary boostrapper file, analize it
 *  and loads data to the card.
 *
 *  Bootstrapper is consists of Tag, Length, Value (TLV) sections.
 *  Each section starts with 4 bytes tag. Then goes length of data (4 bytes)
 *  and then the data itself.
 *
 *  All fields of bootstrapper file is in BIG ENDIAN format.
 */
static int sqn_load_bootstrapper(struct sdio_func *func, u8 *data, int size)
{
	struct sqn_tlv *tlv = (struct sqn_tlv*) data;
	int rv = 0;

	sqn_pr_enter();

	while (size > 0) {
		/*
		 * Convert values accordingly to platform "endianes"
		 * (big or little endian) because bootstrapper file
		 * data is big endian
		 */
		tlv->tag = be32_to_cpu(tlv->tag);
		tlv->length = be32_to_cpu(tlv->length);

		switch (tlv->tag) {
		case SWM_INFO_TAG_SQN_ROOT:
		case SWM_INFO_TAG_SQN_BOOTROM_GROUP:
		case SWM_INFO_TAG_SQN_ID_GROUP:
			/*
			 * This tag is a "container" tag - it's value field
			 * contains other tags
			 */

			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: CONTAINER %x length: %u\n", tlv->tag
				, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */

			/*
			 * If this is a buggy tag, adjust length to
			 * the rest of data
			 */
			if (0 == tlv->length)
				tlv->length = size - sizeof(*tlv);

			rv = sqn_load_bootstrapper(func, (u8*) tlv->value
				, tlv->length);
			if (rv)
				goto out;
			break;

		case SWM_INFO_TAG_SQN_MEMCPY:
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: SWM_INFO_TAG_SQN_MEMCPY length: %u\n"
					, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */
			rv = sqn_handle_memcpy_tag(func
				, (struct sqn_tag_memcpy*) tlv->value);
			if (rv)
				goto out;
			break;

		case SWM_INFO_TAG_SQN_MEMSET:
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: SWM_INFO_TAG_SQN_MEMSET length: %u\n"
					, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */
			rv = sqn_handle_memset_tag(func
				, (struct sqn_tag_memset*) tlv->value);
			if (rv)
				goto out;
			break;

		case SWM_INFO_TAG_SQN_MAC_ADDRESS:
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: SWM_INFO_TAG_SQN_MAC_ADDRESS length: %u\n"
					, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */

			rv = sqn_handle_mac_addr_tag(func, tlv->value
				, tlv->length);
			if (rv)
				goto out;
			break;

		default:
			/* skip all other tags */
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: UNKNOWN %x length: %u\n"
					, tlv->tag, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */
			break;
		}

		/* increment tlv to point it to the beginning of the next
		 * sqn_tlv struct and decrement size accordingly
		 */
		size = (int)(size - (sizeof(*tlv) + tlv->length));
		tlv = (struct sqn_tlv*) ((u8*)tlv + sizeof(*tlv) + tlv->length);
	}

	if (0 != size) {
		/* something wrong with parsing of tlv values */
		rv = -1;
		goto out;
	}

out:
	sqn_pr_leave();
	return rv;
}


extern char *firmware_name;

/** sqn_load_firmware - loads firmware to card
 *  @func: SDIO function, used to transfer data via SDIO interface,
 *         also used to obtain pointer to device structure.
 *
 *  But now the only work it does - is loading of bootstrapper to card,
 *  because firmware is supposed to be loaded by a userspace program.
 */
int sqn_load_firmware(struct sdio_func *func)
{
	int rv = 0;
	const struct firmware *fw = 0;
//Create a local firmware_name with path to replace original global firmware_name -- Tony Wu.
	const char *firmware_name = "../../../data/wimax/Boot.bin";

	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);

	sqn_pr_enter();

	sqn_pr_info("trying to find bootloader image: \"%s\"\n", firmware_name);
	if ((rv = request_firmware(&fw, firmware_name, &func->dev)))
		goto out;

	if (SQN_1130 == sqn_card->version) {
		sdio_claim_host(func);

		/* properly setup registers for firmware loading */
		sqn_pr_dbg("setting up SQN_H_SDRAM_NO_EMR register\n");
		sdio_writeb(func, 0, SQN_H_SDRAM_NO_EMR, &rv);
		if (rv) {
			sdio_release_host(func);
			goto out;
		}

		sqn_pr_dbg("setting up SQN_H_SDRAMCTL_RSTN register\n");
		sdio_writeb(func, 1, SQN_H_SDRAMCTL_RSTN, &rv);
		sdio_release_host(func);
		if (rv)
			goto out;
	}

	sqn_pr_info("loading bootloader to the card...\n");
	if ((rv = sqn_load_bootstrapper(func, (u8*) fw->data, fw->size)))
		goto out;

	/* boot the card */
	sqn_pr_info("bootting the card...\n");
	sdio_claim_host(func); // by daniel
	sdio_writeb(func, 1, SQN_H_CRSTN, &rv);
	sdio_release_host(func); // by daniel
	if (rv)
		goto out;
	sqn_pr_info("  done\n");

out:
	// To avoid kzalloc leakage in /drivers/base/firmware_class.c	
	if (fw) {
		release_firmware(fw);
		fw = NULL;
	}

	sqn_pr_leave();
	return rv;
}
