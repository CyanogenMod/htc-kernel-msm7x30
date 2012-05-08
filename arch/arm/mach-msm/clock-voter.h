/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_VOTER_H
#define __ARCH_ARM_MACH_MSM_CLOCK_VOTER_H

struct clk_ops;
extern struct clk_ops clk_ops_voter;

struct clk_voter {
	bool enabled;
	unsigned long rate;
	struct clk *parent;
	struct clk c;
};

static inline struct clk_voter *to_clk_voter(struct clk *clk)
{
	return container_of(clk, struct clk_voter, c);
}

#define DEFINE_CLK_VOTER(clk_name, _parent) \
	struct clk_voter clk_name = { \
		.parent = _parent, \
		.c = { \
			.dbg_name = #clk_name, \
			.ops = &clk_ops_voter, \
			.flags = CLKFLAG_SKIP_AUTO_OFF, \
			CLK_INIT(clk_name.c), \
		}, \
	}

#endif
