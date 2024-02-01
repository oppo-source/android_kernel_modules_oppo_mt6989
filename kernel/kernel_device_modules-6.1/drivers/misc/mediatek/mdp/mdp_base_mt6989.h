/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#ifndef __MDP_BASE_H__
#define __MDP_BASE_H__

#define MDP_HW_CHECK

static const phys_addr_t mdp_base[ENGBASE_COUNT] = {
	[ENGBASE_MMSYS_CONFIG] = 0x1f000000,
	[ENGBASE_MMSYS_MUTEX] = 0x1f001000,
	[ENGBASE_MDP_RDMA0] = 0x1f003000,
	[ENGBASE_MDP_RDMA2] = 0x1f011000,
	[ENGBASE_MDP_HDR0] = 0x1f005000,
	[ENGBASE_MDP_AAL0] = 0x1f007000,
	[ENGBASE_MDP_RSZ0] = 0x1f009000,
	[ENGBASE_MDP_BIRSZ0] = 0x1f018000,
	[ENGBASE_MDP_TDSHP0] = 0x1f00b000,
	[ENGBASE_MDP_COLOR0] = 0x1f00d000,
	[ENGBASE_MDP_WROT0] = 0x1f00f000,
};

#endif
