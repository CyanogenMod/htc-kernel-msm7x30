/*
 * This file is part of UBIFS.
 *
 * Copyright (C) 2006-2008 Nokia Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Authors: Artem Bityutskiy (Битюцкий Артём)
 *          Adrian Hunter
 */

#ifndef __UBIFS_DEBUG_H__
#define __UBIFS_DEBUG_H__

/* Checking helper functions */
typedef int (*dbg_leaf_callback)(struct ubifs_info *c,
				 struct ubifs_zbranch *zbr, void *priv);
typedef int (*dbg_znode_callback)(struct ubifs_info *c,
				  struct ubifs_znode *znode, void *priv);

#ifdef CONFIG_UBIFS_FS_DEBUG

#include <linux/random.h>

/**
 * ubifs_debug_info - per-FS debugging information.
 * @old_zroot: old index root - used by 'dbg_check_old_index()'
 * @old_zroot_level: old index root level - used by 'dbg_check_old_index()'
 * @old_zroot_sqnum: old index root sqnum - used by 'dbg_check_old_index()'
 * @failure_mode: failure mode for recovery testing
 * @fail_delay: 0=>don't delay, 1=>delay a time, 2=>delay a number of calls
 * @fail_timeout: time in jiffies when delay of failure mode expires
 * @fail_cnt: current number of calls to failure mode I/O functions
 * @fail_cnt_max: number of calls by which to delay failure mode
 * @chk_lpt_sz: used by LPT tree size checker
 * @chk_lpt_sz2: used by LPT tree size checker
 * @chk_lpt_wastage: used by LPT tree size checker
 * @chk_lpt_lebs: used by LPT tree size checker
 * @new_nhead_offs: used by LPT tree size checker
 * @new_ihead_lnum: used by debugging to check @c->ihead_lnum
 * @new_ihead_offs: used by debugging to check @c->ihead_offs
 *
 * @saved_lst: saved lprops statistics (used by 'dbg_save_space_info()')
 * @saved_bi: saved budgeting information
 * @saved_free: saved amount of free space
 * @saved_idx_gc_cnt: saved value of @c->idx_gc_cnt
 *
 * @dfs_dir_name: name of debugfs directory containing this file-system's files
 * @dfs_dir: direntry object of the file-system debugfs directory
 * @dfs_dump_lprops: "dump lprops" debugfs knob
 * @dfs_dump_budg: "dump budgeting information" debugfs knob
 * @dfs_dump_tnc: "dump TNC" debugfs knob
 */
struct ubifs_debug_info {
	struct ubifs_zbranch old_zroot;
	int old_zroot_level;
	unsigned long long old_zroot_sqnum;
	int failure_mode;
	int fail_delay;
	unsigned long fail_timeout;
	unsigned int fail_cnt;
	unsigned int fail_cnt_max;
	long long chk_lpt_sz;
	long long chk_lpt_sz2;
	long long chk_lpt_wastage;
	int chk_lpt_lebs;
	int new_nhead_offs;
	int new_ihead_lnum;
	int new_ihead_offs;

	struct ubifs_lp_stats saved_lst;
	struct ubifs_budg_info saved_bi;
	long long saved_free;
	int saved_idx_gc_cnt;

	char dfs_dir_name[100];
	struct dentry *dfs_dir;
	struct dentry *dfs_dump_lprops;
	struct dentry *dfs_dump_budg;
	struct dentry *dfs_dump_tnc;
};

#define ubifs_assert(expr) do {                                                \
	if (unlikely(!(expr))) {                                               \
		printk(KERN_CRIT "UBIFS assert failed in %s at %u (pid %d)\n", \
		       __func__, __LINE__, current->pid);                      \
		dbg_dump_stack();                                              \
	}                                                                      \
} while (0)

#define ubifs_assert_cmt_locked(c) do {                                        \
	if (unlikely(down_write_trylock(&(c)->commit_sem))) {                  \
		up_write(&(c)->commit_sem);                                    \
		printk(KERN_CRIT "commit lock is not locked!\n");              \
		ubifs_assert(0);                                               \
	}                                                                      \
} while (0)

#define dbg_dump_stack() dump_stack()

#define dbg_err(fmt, ...) do {                                                 \
	spin_lock(&dbg_lock);                                                  \
	ubifs_err(fmt, ##__VA_ARGS__);                                         \
	spin_unlock(&dbg_lock);                                                \
} while (0)

const char *dbg_key_str0(const struct ubifs_info *c,
			 const union ubifs_key *key);
const char *dbg_key_str1(const struct ubifs_info *c,
			 const union ubifs_key *key);

/*
 * DBGKEY macros require @dbg_lock to be held, which it is in the dbg message
 * macros.
 */
#define DBGKEY(key) dbg_key_str0(c, (key))
#define DBGKEY1(key) dbg_key_str1(c, (key))

#define ubifs_dbg_msg(type, fmt, ...) do {                        \
	spin_lock(&dbg_lock);                                     \
	pr_debug("UBIFS DBG " type ": " fmt "\n", ##__VA_ARGS__); \
	spin_unlock(&dbg_lock);                                   \
} while (0)

/* Just a debugging messages not related to any specific UBIFS subsystem */
#define dbg_msg(fmt, ...)   ubifs_dbg_msg("msg", fmt, ##__VA_ARGS__)
/* General messages */
#define dbg_gen(fmt, ...)   ubifs_dbg_msg("gen", fmt, ##__VA_ARGS__)
/* Additional journal messages */
#define dbg_jnl(fmt, ...)   ubifs_dbg_msg("jnl", fmt, ##__VA_ARGS__)
/* Additional TNC messages */
#define dbg_tnc(fmt, ...)   ubifs_dbg_msg("tnc", fmt, ##__VA_ARGS__)
/* Additional lprops messages */
#define dbg_lp(fmt, ...)    ubifs_dbg_msg("lp", fmt, ##__VA_ARGS__)
/* Additional LEB find messages */
#define dbg_find(fmt, ...)  ubifs_dbg_msg("find", fmt, ##__VA_ARGS__)
/* Additional mount messages */
#define dbg_mnt(fmt, ...)   ubifs_dbg_msg("mnt", fmt, ##__VA_ARGS__)
/* Additional I/O messages */
#define dbg_io(fmt, ...)    ubifs_dbg_msg("io", fmt, ##__VA_ARGS__)
/* Additional commit messages */
#define dbg_cmt(fmt, ...)   ubifs_dbg_msg("cmt", fmt, ##__VA_ARGS__)
/* Additional budgeting messages */
#define dbg_budg(fmt, ...)  ubifs_dbg_msg("budg", fmt, ##__VA_ARGS__)
/* Additional log messages */
#define dbg_log(fmt, ...)   ubifs_dbg_msg("log", fmt, ##__VA_ARGS__)
/* Additional gc messages */
#define dbg_gc(fmt, ...)    ubifs_dbg_msg("gc", fmt, ##__VA_ARGS__)
/* Additional scan messages */
#define dbg_scan(fmt, ...)  ubifs_dbg_msg("scan", fmt, ##__VA_ARGS__)
/* Additional recovery messages */
#define dbg_rcvry(fmt, ...) ubifs_dbg_msg("rcvry", fmt, ##__VA_ARGS__)

/*
 * Debugging check flags.
 *
 * UBIFS_CHK_GEN: general checks
 * UBIFS_CHK_TNC: check TNC
 * UBIFS_CHK_IDX_SZ: check index size
 * UBIFS_CHK_ORPH: check orphans
 * UBIFS_CHK_OLD_IDX: check the old index
 * UBIFS_CHK_LPROPS: check lprops
 * UBIFS_CHK_FS: check the file-system
 */
enum {
	UBIFS_CHK_GEN     = 0x1,
	UBIFS_CHK_TNC     = 0x2,
	UBIFS_CHK_IDX_SZ  = 0x4,
	UBIFS_CHK_ORPH    = 0x8,
	UBIFS_CHK_OLD_IDX = 0x10,
	UBIFS_CHK_LPROPS  = 0x20,
	UBIFS_CHK_FS      = 0x40,
};

/*
 * Special testing flags.
 *
 * UBIFS_TST_RCVRY: failure mode for recovery testing
 */
enum {
	UBIFS_TST_RCVRY             = 0x4,
};

extern spinlock_t dbg_lock;

extern unsigned int ubifs_msg_flags;
extern unsigned int ubifs_chk_flags;
extern unsigned int ubifs_tst_flags;

int ubifs_debugging_init(struct ubifs_info *c);
void ubifs_debugging_exit(struct ubifs_info *c);

/* Dump functions */
const char *dbg_ntype(int type);
const char *dbg_cstate(int cmt_state);
const char *dbg_jhead(int jhead);
const char *dbg_get_key_dump(const struct ubifs_info *c,
			     const union ubifs_key *key);
void dbg_dump_inode(const struct ubifs_info *c, const struct inode *inode);
void dbg_dump_node(const struct ubifs_info *c, const void *node);
void dbg_dump_lpt_node(const struct ubifs_info *c, void *node, int lnum,
		       int offs);
void dbg_dump_budget_req(const struct ubifs_budget_req *req);
void dbg_dump_lstats(const struct ubifs_lp_stats *lst);
void dbg_dump_budg(struct ubifs_info *c, const struct ubifs_budg_info *bi);
void dbg_dump_lprop(const struct ubifs_info *c, const struct ubifs_lprops *lp);
void dbg_dump_lprops(struct ubifs_info *c);
void dbg_dump_lpt_info(struct ubifs_info *c);
void dbg_dump_leb(const struct ubifs_info *c, int lnum);
void dbg_dump_znode(const struct ubifs_info *c,
		    const struct ubifs_znode *znode);
void dbg_dump_heap(struct ubifs_info *c, struct ubifs_lpt_heap *heap, int cat);
void dbg_dump_pnode(struct ubifs_info *c, struct ubifs_pnode *pnode,
		    struct ubifs_nnode *parent, int iip);
void dbg_dump_tnc(struct ubifs_info *c);
void dbg_dump_index(struct ubifs_info *c);
void dbg_dump_lpt_lebs(const struct ubifs_info *c);

int dbg_walk_index(struct ubifs_info *c, dbg_leaf_callback leaf_cb,
		   dbg_znode_callback znode_cb, void *priv);

/* Checking functions */
void dbg_save_space_info(struct ubifs_info *c);
int dbg_check_space_info(struct ubifs_info *c);
int dbg_check_lprops(struct ubifs_info *c);
int dbg_old_index_check_init(struct ubifs_info *c, struct ubifs_zbranch *zroot);
int dbg_check_old_index(struct ubifs_info *c, struct ubifs_zbranch *zroot);
int dbg_check_cats(struct ubifs_info *c);
int dbg_check_ltab(struct ubifs_info *c);
int dbg_chk_lpt_free_spc(struct ubifs_info *c);
int dbg_chk_lpt_sz(struct ubifs_info *c, int action, int len);
int dbg_check_synced_i_size(struct inode *inode);
int dbg_check_dir_size(struct ubifs_info *c, const struct inode *dir);
int dbg_check_tnc(struct ubifs_info *c, int extra);
int dbg_check_idx_size(struct ubifs_info *c, long long idx_size);
int dbg_check_filesystem(struct ubifs_info *c);
void dbg_check_heap(struct ubifs_info *c, struct ubifs_lpt_heap *heap, int cat,
		    int add_pos);
int dbg_check_lpt_nodes(struct ubifs_info *c, struct ubifs_cnode *cnode,
			int row, int col);
int dbg_check_inode_size(struct ubifs_info *c, const struct inode *inode,
			 loff_t size);
int dbg_check_data_nodes_order(struct ubifs_info *c, struct list_head *head);
int dbg_check_nondata_nodes_order(struct ubifs_info *c, struct list_head *head);

/* Force the use of in-the-gaps method for testing */
static inline int dbg_force_in_the_gaps_enabled(void)
{
	return ubifs_chk_flags & UBIFS_CHK_GEN;
}
int dbg_force_in_the_gaps(void);

/* Failure mode for recovery testing */
#define dbg_failure_mode (ubifs_tst_flags & UBIFS_TST_RCVRY)

#ifndef UBIFS_DBG_PRESERVE_UBI
#define ubi_leb_read   dbg_leb_read
#define ubi_leb_write  dbg_leb_write
#define ubi_leb_change dbg_leb_change
#define ubi_leb_erase  dbg_leb_erase
#define ubi_leb_unmap  dbg_leb_unmap
#define ubi_is_mapped  dbg_is_mapped
#define ubi_leb_map    dbg_leb_map
#endif

int dbg_leb_read(struct ubi_volume_desc *desc, int lnum, char *buf, int offset,
		 int len, int check);
int dbg_leb_write(struct ubi_volume_desc *desc, int lnum, const void *buf,
		  int offset, int len, int dtype);
int dbg_leb_change(struct ubi_volume_desc *desc, int lnum, const void *buf,
		   int len, int dtype);
int dbg_leb_erase(struct ubi_volume_desc *desc, int lnum);
int dbg_leb_unmap(struct ubi_volume_desc *desc, int lnum);
int dbg_is_mapped(struct ubi_volume_desc *desc, int lnum);
int dbg_leb_map(struct ubi_volume_desc *desc, int lnum, int dtype);

static inline int dbg_read(struct ubi_volume_desc *desc, int lnum, char *buf,
			   int offset, int len)
{
	return dbg_leb_read(desc, lnum, buf, offset, len, 0);
}

static inline int dbg_write(struct ubi_volume_desc *desc, int lnum,
			    const void *buf, int offset, int len)
{
	return dbg_leb_write(desc, lnum, buf, offset, len, UBI_UNKNOWN);
}

static inline int dbg_change(struct ubi_volume_desc *desc, int lnum,
				    const void *buf, int len)
{
	return dbg_leb_change(desc, lnum, buf, len, UBI_UNKNOWN);
}

/* Debugfs-related stuff */
int dbg_debugfs_init(void);
void dbg_debugfs_exit(void);
int dbg_debugfs_init_fs(struct ubifs_info *c);
void dbg_debugfs_exit_fs(struct ubifs_info *c);

#else /* !CONFIG_UBIFS_FS_DEBUG */

/* Use "if (0)" to make compiler check arguments even if debugging is off */
#define ubifs_assert(expr)  do {                                               \
	if (0 && (expr))                                                       \
		printk(KERN_CRIT "UBIFS assert failed in %s at %u (pid %d)\n", \
		       __func__, __LINE__, current->pid);                      \
} while (0)

#define dbg_err(fmt, ...)   do {                   \
	if (0)                                     \
		ubifs_err(fmt, ##__VA_ARGS__);     \
} while (0)

#define ubifs_dbg_msg(fmt, ...) do {               \
	if (0)                                     \
		pr_debug(fmt "\n", ##__VA_ARGS__); \
} while (0)

#define dbg_dump_stack()
#define ubifs_assert_cmt_locked(c)

#define dbg_msg(fmt, ...)   ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_gen(fmt, ...)   ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_jnl(fmt, ...)   ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_tnc(fmt, ...)   ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_lp(fmt, ...)    ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_find(fmt, ...)  ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_mnt(fmt, ...)   ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_io(fmt, ...)    ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_cmt(fmt, ...)   ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_budg(fmt, ...)  ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_log(fmt, ...)   ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_gc(fmt, ...)    ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_scan(fmt, ...)  ubifs_dbg_msg(fmt, ##__VA_ARGS__)
#define dbg_rcvry(fmt, ...) ubifs_dbg_msg(fmt, ##__VA_ARGS__)

#define DBGKEY(key)  ((char *)(key))
#define DBGKEY1(key) ((char *)(key))

static inline int ubifs_debugging_init(struct ubifs_info *c)      { return 0; }
static inline void ubifs_debugging_exit(struct ubifs_info *c)     { return; }
static inline const char *dbg_ntype(int type)                     { return ""; }
static inline const char *dbg_cstate(int cmt_state)               { return ""; }
static inline const char *dbg_jhead(int jhead)                    { return ""; }
static inline const char *
dbg_get_key_dump(const struct ubifs_info *c,
		 const union ubifs_key *key)                      { return ""; }
static inline void dbg_dump_inode(const struct ubifs_info *c,
				  const struct inode *inode)      { return; }
static inline void dbg_dump_node(const struct ubifs_info *c,
				 const void *node)                { return; }
static inline void dbg_dump_lpt_node(const struct ubifs_info *c,
				     void *node, int lnum,
				     int offs)                    { return; }
static inline void
dbg_dump_budget_req(const struct ubifs_budget_req *req)           { return; }
static inline void
dbg_dump_lstats(const struct ubifs_lp_stats *lst)                 { return; }
static inline void
dbg_dump_budg(struct ubifs_info *c,
	      const struct ubifs_budg_info *bi)                   { return; }
static inline void dbg_dump_lprop(const struct ubifs_info *c,
				  const struct ubifs_lprops *lp)  { return; }
static inline void dbg_dump_lprops(struct ubifs_info *c)          { return; }
static inline void dbg_dump_lpt_info(struct ubifs_info *c)        { return; }
static inline void dbg_dump_leb(const struct ubifs_info *c,
				int lnum)                         { return; }
static inline void
dbg_dump_znode(const struct ubifs_info *c,
	       const struct ubifs_znode *znode)                   { return; }
static inline void dbg_dump_heap(struct ubifs_info *c,
				 struct ubifs_lpt_heap *heap,
				 int cat)                         { return; }
static inline void dbg_dump_pnode(struct ubifs_info *c,
				  struct ubifs_pnode *pnode,
				  struct ubifs_nnode *parent,
				  int iip)                        { return; }
static inline void dbg_dump_tnc(struct ubifs_info *c)             { return; }
static inline void dbg_dump_index(struct ubifs_info *c)           { return; }
static inline void dbg_dump_lpt_lebs(const struct ubifs_info *c)  { return; }

static inline int dbg_walk_index(struct ubifs_info *c,
				 dbg_leaf_callback leaf_cb,
				 dbg_znode_callback znode_cb,
				 void *priv)                      { return 0; }
static inline void dbg_save_space_info(struct ubifs_info *c)      { return; }
static inline int dbg_check_space_info(struct ubifs_info *c)      { return 0; }
static inline int dbg_check_lprops(struct ubifs_info *c)          { return 0; }
static inline int
dbg_old_index_check_init(struct ubifs_info *c,
			 struct ubifs_zbranch *zroot)             { return 0; }
static inline int
dbg_check_old_index(struct ubifs_info *c,
		    struct ubifs_zbranch *zroot)                  { return 0; }
static inline int dbg_check_cats(struct ubifs_info *c)            { return 0; }
static inline int dbg_check_ltab(struct ubifs_info *c)            { return 0; }
static inline int dbg_chk_lpt_free_spc(struct ubifs_info *c)      { return 0; }
static inline int dbg_chk_lpt_sz(struct ubifs_info *c,
				 int action, int len)             { return 0; }
static inline int dbg_check_synced_i_size(struct inode *inode)    { return 0; }
static inline int dbg_check_dir_size(struct ubifs_info *c,
				     const struct inode *dir)     { return 0; }
static inline int dbg_check_tnc(struct ubifs_info *c, int extra)  { return 0; }
static inline int dbg_check_idx_size(struct ubifs_info *c,
				     long long idx_size)          { return 0; }
static inline int dbg_check_filesystem(struct ubifs_info *c)      { return 0; }
static inline void dbg_check_heap(struct ubifs_info *c,
				  struct ubifs_lpt_heap *heap,
				  int cat, int add_pos)           { return; }
static inline int dbg_check_lpt_nodes(struct ubifs_info *c,
	struct ubifs_cnode *cnode, int row, int col)              { return 0; }
static inline int dbg_check_inode_size(struct ubifs_info *c,
				       const struct inode *inode,
				       loff_t size)               { return 0; }
static inline int
dbg_check_data_nodes_order(struct ubifs_info *c,
			   struct list_head *head)                { return 0; }
static inline int
dbg_check_nondata_nodes_order(struct ubifs_info *c,
			      struct list_head *head)             { return 0; }

static inline int dbg_force_in_the_gaps(void)                     { return 0; }
#define dbg_force_in_the_gaps_enabled() 0
#define dbg_failure_mode                0

static inline int dbg_debugfs_init(void)                          { return 0; }
static inline void dbg_debugfs_exit(void)                         { return; }
static inline int dbg_debugfs_init_fs(struct ubifs_info *c)       { return 0; }
static inline int dbg_debugfs_exit_fs(struct ubifs_info *c)       { return 0; }

#endif /* !CONFIG_UBIFS_FS_DEBUG */
#endif /* !__UBIFS_DEBUG_H__ */
