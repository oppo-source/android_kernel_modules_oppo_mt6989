/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#ifndef __CONN_RGU_ON_REGS_H__
#define __CONN_RGU_ON_REGS_H__


#define CONN_RGU_ON_BASE                                       0x18000000

#define CONN_RGU_ON_BGFYS_ON_TOP_PWR_CTL_1_ADDR                (CONN_RGU_ON_BASE + 0x028)
#define CONN_RGU_ON_GPSSYS_CPU_SW_RST_B_ADDR                   (CONN_RGU_ON_BASE + 0x130)
#define CONN_RGU_ON_BGFSYS_ON_TOP_PWR_ACK_ST_ADDR              (CONN_RGU_ON_BASE + 0x414)
#define CONN_RGU_ON_BGFSYS_OFF_TOP_PWR_ACK_ST_ADDR             (CONN_RGU_ON_BASE + 0x424)

#define CONN_RGU_ON_BGFSYS_ON_TOP_PWR_ACK_ST_AN_BGFSYS_ON_TOP_PWR_ACK_S_ADDR \
	CONN_RGU_ON_BGFSYS_ON_TOP_PWR_ACK_ST_ADDR
#define CONN_RGU_ON_BGFSYS_ON_TOP_PWR_ACK_ST_AN_BGFSYS_ON_TOP_PWR_ACK_S_MASK 0x02000000
#define CONN_RGU_ON_BGFSYS_ON_TOP_PWR_ACK_ST_AN_BGFSYS_ON_TOP_PWR_ACK_S_SHFT 25
#define CONN_RGU_ON_BGFSYS_ON_TOP_PWR_ACK_ST_BGFSYS_ON_TOP_PWR_ACK_ADDR \
	CONN_RGU_ON_BGFSYS_ON_TOP_PWR_ACK_ST_ADDR
#define CONN_RGU_ON_BGFSYS_ON_TOP_PWR_ACK_ST_BGFSYS_ON_TOP_PWR_ACK_MASK 0x01000000
#define CONN_RGU_ON_BGFSYS_ON_TOP_PWR_ACK_ST_BGFSYS_ON_TOP_PWR_ACK_SHFT 24

#define CONN_RGU_ON_GPSSYS_CPU_SW_RST_B_GPSSYS_CPU_SW_RST_B_ADDR \
	CONN_RGU_ON_GPSSYS_CPU_SW_RST_B_ADDR
#define CONN_RGU_ON_GPSSYS_CPU_SW_RST_B_GPSSYS_CPU_SW_RST_B_MASK 0x00000001
#define CONN_RGU_ON_GPSSYS_CPU_SW_RST_B_GPSSYS_CPU_SW_RST_B_SHFT 0

#define CONN_RGU_ON_BGFSYS_OFF_TOP_PWR_ACK_ST_BGFSYS_OFF_TOP_PWR_ACK_ADDR \
	CONN_RGU_ON_BGFSYS_OFF_TOP_PWR_ACK_ST_ADDR
#define CONN_RGU_ON_BGFSYS_OFF_TOP_PWR_ACK_ST_BGFSYS_OFF_TOP_PWR_ACK_MASK 0x01000000
#define CONN_RGU_ON_BGFSYS_OFF_TOP_PWR_ACK_ST_BGFSYS_OFF_TOP_PWR_ACK_SHFT 24
#define CONN_RGU_ON_BGFSYS_OFF_TOP_PWR_ACK_ST_BGFSYS_OFF_TOP_PWR_ACK_S_ADDR \
	CONN_RGU_ON_BGFSYS_OFF_TOP_PWR_ACK_ST_ADDR
#define CONN_RGU_ON_BGFSYS_OFF_TOP_PWR_ACK_ST_BGFSYS_OFF_TOP_PWR_ACK_S_MASK 0x00000003
#define CONN_RGU_ON_BGFSYS_OFF_TOP_PWR_ACK_ST_BGFSYS_OFF_TOP_PWR_ACK_S_SHFT 0

#endif /* __CONN_RGU_ON_REGS_H__*/
