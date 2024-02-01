/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __CONN_MCU_BUS_CR_REGS_H__
#define __CONN_MCU_BUS_CR_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
*
*                     CONN_MCU_BUS_CR CR Definitions
*
*****************************************************************************
*/

#define CONN_MCU_BUS_CR_BASE \
	0x830C0000

#define CONN_MCU_BUS_CR_AP2WF_REMAP_1_ADDR \
	(CONN_MCU_BUS_CR_BASE + 0x0120)
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_ADDR \
	(CONN_MCU_BUS_CR_BASE + 0x1000)
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ADDR_ADDR \
	(CONN_MCU_BUS_CR_BASE + 0x1004)
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ID_ADDR \
	(CONN_MCU_BUS_CR_BASE + 0x1008)
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_LYR_ADDR \
	(CONN_MCU_BUS_CR_BASE + 0x100C)
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_ADDR \
	(CONN_MCU_BUS_CR_BASE + 0x1010)


#define CONN_MCU_BUS_CR_AP2WF_REMAP_1_R_AP2WF_PUBLIC_REMAPPING_0_START_ADDRESS_ADDR \
	CONN_MCU_BUS_CR_AP2WF_REMAP_1_ADDR
#define CONN_MCU_BUS_CR_AP2WF_REMAP_1_R_AP2WF_PUBLIC_REMAPPING_0_START_ADDRESS_MASK \
	0xFFFFFFFF
#define CONN_MCU_BUS_CR_AP2WF_REMAP_1_R_AP2WF_PUBLIC_REMAPPING_0_START_ADDRESS_SHFT \
	0

#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_DEBUG_EN_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_DEBUG_EN_MASK \
	0x80000000
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_DEBUG_EN_SHFT \
	31
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_SLV_HREADY_SEL_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_SLV_HREADY_SEL_MASK \
	0x40000000
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_SLV_HREADY_SEL_SHFT \
	30
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_IRQ_CLEAR_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_IRQ_CLEAR_MASK \
	0x20000000
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_IRQ_CLEAR_SHFT \
	29
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_DEBUG_CG_EN_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_DEBUG_CG_EN_MASK \
	0x10000000
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_DEBUG_CG_EN_SHFT \
	28
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_FORCE_VLP_PREADY_HIGH_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_FORCE_VLP_PREADY_HIGH_MASK \
	0x08000000
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_FORCE_VLP_PREADY_HIGH_SHFT \
	27
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_TIME_LIMIT_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_TIME_LIMIT_MASK \
	0x000000FF
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_CTRL_BUS_HANG_TIME_LIMIT_SHFT \
	0

#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ADDR_AHB_TIMEOUT_HADDR_LATCH_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ADDR_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ADDR_AHB_TIMEOUT_HADDR_LATCH_MASK \
	0xFFFFFFFF
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ADDR_AHB_TIMEOUT_HADDR_LATCH_SHFT \
	0

#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ID_BUS_HANG_ID_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ID_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ID_BUS_HANG_ID_MASK \
	0x00001FFF
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_ID_BUS_HANG_ID_SHFT \
	0

#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_LYR_BUS_HANG_BUS_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_LYR_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_LYR_BUS_HANG_BUS_MASK \
	0xFFFFFFFF
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_LYR_BUS_HANG_BUS_SHFT \
	0

#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_BUS_HANG_HTRANS_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_BUS_HANG_HTRANS_MASK \
	0x00000300
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_BUS_HANG_HTRANS_SHFT \
	8
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_BUS_HANG_WRITE_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_BUS_HANG_WRITE_MASK \
	0x00000080
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_BUS_HANG_WRITE_SHFT \
	7
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_BUS_HANG_HBURST_ADDR \
	CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_ADDR
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_BUS_HANG_HBURST_MASK \
	0x00000070
#define CONN_MCU_BUS_CR_AHB_APB_TIMEOUT_INFO_BUS_HANG_HBURST_SHFT \
	4


#ifdef __cplusplus
}
#endif

#endif
