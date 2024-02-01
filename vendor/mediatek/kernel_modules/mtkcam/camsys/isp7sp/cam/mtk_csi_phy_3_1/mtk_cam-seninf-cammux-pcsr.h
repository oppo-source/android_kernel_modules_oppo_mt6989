/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __SENINF_CAM_MUX_PCSR0_H__
#define __SENINF_CAM_MUX_PCSR0_H__

#define SENINF_CAM_MUX_PCSR_CTRL 0x0000
#define RG_SENINF_CAM_MUX_PCSR_SRC_SEL_SHIFT 0
#define RG_SENINF_CAM_MUX_PCSR_SRC_SEL_MASK (0x7f << 0)
#define RG_SENINF_CAM_MUX_PCSR_EN_SHIFT 7
#define RG_SENINF_CAM_MUX_PCSR_EN_MASK (0x1 << 7)
#define RG_SENINF_CAM_MUX_PCSR_CHK_PIX_MODE_SHIFT 8
#define RG_SENINF_CAM_MUX_PCSR_CHK_PIX_MODE_MASK (0x7 << 8)
#define RG_SENINF_CAM_MUX_PCSR_CHK_EN_SHIFT 15
#define RG_SENINF_CAM_MUX_PCSR_CHK_EN_MASK (0x1 << 15)
#define CAM_MUX_PCSR_NEXT_SRC_SEL_SHIFT 16
#define CAM_MUX_PCSR_NEXT_SRC_SEL_MASK (0x7f << 16)
#define RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN0_SHIFT 23
#define RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN0_MASK (0x1 << 23)
#define RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN1_SHIFT 24
#define RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN1_MASK (0x1 << 24)

#define SENINF_CAM_MUX_PCSR_OPT 0x0004
#define RG_SENINF_CAM_MUX_PCSR_VC_SEL_SHIFT 0
#define RG_SENINF_CAM_MUX_PCSR_VC_SEL_MASK (0x1f << 0)
#define RG_SENINF_CAM_MUX_PCSR_VC_SEL_EN_SHIFT 7
#define RG_SENINF_CAM_MUX_PCSR_VC_SEL_EN_MASK (0x1 << 7)
#define RG_SENINF_CAM_MUX_PCSR_DT_SEL_SHIFT 8
#define RG_SENINF_CAM_MUX_PCSR_DT_SEL_MASK (0x3f << 8)
#define RG_SENINF_CAM_MUX_PCSR_DT_SEL_EN_SHIFT 15
#define RG_SENINF_CAM_MUX_PCSR_DT_SEL_EN_MASK (0x1 << 15)
#define RG_SENINF_CAM_MUX_PCSR_VSYNC_BYPASS_SHIFT 16
#define RG_SENINF_CAM_MUX_PCSR_VSYNC_BYPASS_MASK (0x1 << 16)
#define RG_SENINF_CAM_MUX_PCSR_TAG_VC_DT_PAGE_SEL_SHIFT 24
#define RG_SENINF_CAM_MUX_PCSR_TAG_VC_DT_PAGE_SEL_MASK (0x7 << 24)
#define RG_SENINF_CAM_MUX_PCSR_TAG_BC_FILTER_EN_SHIFT 28
#define RG_SENINF_CAM_MUX_PCSR_TAG_BC_FILTER_EN_MASK (0x1 << 28)
#define RG_SENINF_CAM_MUX_PCSR_TAG_MERGE_SEL_EN_SHIFT 29
#define RG_SENINF_CAM_MUX_PCSR_TAG_MERGE_SEL_EN_MASK (0x1 << 29)
#define RG_SENINF_CAM_MUX_PCSR_FORCE_FIFO_FULL_SHIFT 30
#define RG_SENINF_CAM_MUX_PCSR_FORCE_FIFO_FULL_MASK (0x1 << 30)
#define RG_SENINF_CAM_MUX_PCSR_SRC_SEL_SYNC_MODE_SHIFT 31
#define RG_SENINF_CAM_MUX_PCSR_SRC_SEL_SYNC_MODE_MASK (0x1 << 31)

#define SENINF_CAM_MUX_PCSR_IRQ_EN 0x0008
#define RG_SENINF_CAM_MUX_PCSR_HSIZE_ERR_IRQ_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_PCSR_HSIZE_ERR_IRQ_EN_MASK (0x1 << 0)
#define RG_SENINF_CAM_MUX_PCSR_VSIZE_ERR_IRQ_EN_SHIFT 1
#define RG_SENINF_CAM_MUX_PCSR_VSIZE_ERR_IRQ_EN_MASK (0x1 << 1)
#define RG_SENINF_CAM_MUX_PCSR_VSYNC_IRQ_EN_SHIFT 8
#define RG_SENINF_CAM_MUX_PCSR_VSYNC_IRQ_EN_MASK (0x1 << 8)

#define SENINF_CAM_MUX_PCSR_IRQ_STATUS 0x000c
#define RO_SENINF_CAM_MUX_PCSR_HSIZE_ERR_IRQ_SHIFT 0
#define RO_SENINF_CAM_MUX_PCSR_HSIZE_ERR_IRQ_MASK (0x1 << 0)
#define RO_SENINF_CAM_MUX_PCSR_VSIZE_ERR_IRQ_SHIFT 1
#define RO_SENINF_CAM_MUX_PCSR_VSIZE_ERR_IRQ_MASK (0x1 << 1)
#define RO_SENINF_CAM_MUX_PCSR_VSYNC_IRQ_SHIFT 8
#define RO_SENINF_CAM_MUX_PCSR_VSYNC_IRQ_MASK (0x1 << 8)

#define SENINF_CAM_MUX_PCSR_IRQ_TRIG 0x0010
#define RG_SENINF_CAM_MUX_PCSR_IRQ_TRIG_SHIFT 0
#define RG_SENINF_CAM_MUX_PCSR_IRQ_TRIG_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_PCSR_CHK_CTL 0x0014
#define RG_SENINF_CAM_MUX_PCSR_EXP_HSIZE_SHIFT 0
#define RG_SENINF_CAM_MUX_PCSR_EXP_HSIZE_MASK (0xffff << 0)
#define RG_SENINF_CAM_MUX_PCSR_EXP_VSIZE_SHIFT 16
#define RG_SENINF_CAM_MUX_PCSR_EXP_VSIZE_MASK (0xffff << 16)

#define SENINF_CAM_MUX_PCSR_CHK_RES 0x0018
#define RO_SENINF_CAM_MUX_PCSR_RCV_HSIZE_SHIFT 0
#define RO_SENINF_CAM_MUX_PCSR_RCV_HSIZE_MASK (0xffff << 0)
#define RO_SENINF_CAM_MUX_PCSR_RCV_VSIZE_SHIFT 16
#define RO_SENINF_CAM_MUX_PCSR_RCV_VSIZE_MASK (0xffff << 16)

#define SENINF_CAM_MUX_PCSR_CHK_ERR_RES 0x001c
#define RO_SENINF_CAM_MUX_PCSR_RCV_ERR_HSIZE_SHIFT 0
#define RO_SENINF_CAM_MUX_PCSR_RCV_ERR_HSIZE_MASK (0xffff << 0)
#define RO_SENINF_CAM_MUX_PCSR_RCV_ERR_VSIZE_SHIFT 16
#define RO_SENINF_CAM_MUX_PCSR_RCV_ERR_VSIZE_MASK (0xffff << 16)

#define SENINF_CAM_MUX_PCSR_TAG_VC_SEL 0x0020
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_0_SHIFT 0
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_0_MASK (0x1f << 0)
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_0_EN_SHIFT 7
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_0_EN_MASK (0x1 << 7)
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_1_SHIFT 8
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_1_MASK (0x1f << 8)
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_1_EN_SHIFT 15
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_1_EN_MASK (0x1 << 15)
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_2_SHIFT 16
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_2_MASK (0x1f << 16)
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_2_EN_SHIFT 23
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_2_EN_MASK (0x1 << 23)
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_3_SHIFT 24
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_3_MASK (0x1f << 24)
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_3_EN_SHIFT 31
#define RG_SENINF_CAM_MUX_TAG_VC_SEL_4N_3_EN_MASK (0x1 << 31)

#define SENINF_CAM_MUX_PCSR_TAG_DT_SEL 0x0024
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_0_SHIFT 0
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_0_MASK (0x3f << 0)
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_0_EN_SHIFT 7
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_0_EN_MASK (0x1 << 7)
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_1_SHIFT 8
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_1_MASK (0x3f << 8)
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_1_EN_SHIFT 15
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_1_EN_MASK (0x1 << 15)
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_2_SHIFT 16
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_2_MASK (0x3f << 16)
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_2_EN_SHIFT 23
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_2_EN_MASK (0x1 << 23)
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_3_SHIFT 24
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_3_MASK (0x3f << 24)
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_3_EN_SHIFT 31
#define RG_SENINF_CAM_MUX_TAG_DT_SEL_4N_3_EN_MASK (0x1 << 31)

#endif
