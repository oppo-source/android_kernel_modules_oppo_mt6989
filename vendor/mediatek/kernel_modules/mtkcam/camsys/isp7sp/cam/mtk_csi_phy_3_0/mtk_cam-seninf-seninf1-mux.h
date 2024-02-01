/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __SENINF1_MUX_H__
#define __SENINF1_MUX_H__

#define SENINF_MUX_CTRL_0 0x0000
#define SENINF_MUX_EN_SHIFT 0
#define SENINF_MUX_EN_MASK (0x1 << 0)
#define SENINF_MUX_IRQ_SW_RST_SHIFT 1
#define SENINF_MUX_IRQ_SW_RST_MASK (0x1 << 1)
#define SENINF_MUX_SW_RST_SHIFT 2
#define SENINF_MUX_SW_RST_MASK (0x1 << 2)

#define SENINF_MUX_CTRL_1 0x0004
#define RG_SENINF_MUX_SRC_SEL_SHIFT 0
#define RG_SENINF_MUX_SRC_SEL_MASK (0xf << 0)
#define RG_SENINF_MUX_PIX_MODE_SEL_SHIFT 8
#define RG_SENINF_MUX_PIX_MODE_SEL_MASK (0x7 << 8)
#define RG_SENINF_MUX_FIFO_PUSH_EN_SHIFT 16
#define RG_SENINF_MUX_FIFO_PUSH_EN_MASK (0x3f << 16)
#define RG_SENINF_MUX_RDY_FORCE_MODE_EN_SHIFT 24
#define RG_SENINF_MUX_RDY_FORCE_MODE_EN_MASK (0x1 << 24)
#define RG_SENINF_MUX_RDY_FORCE_MODE_VAL_SHIFT 25
#define RG_SENINF_MUX_RDY_FORCE_MODE_VAL_MASK (0x1 << 25)
#define RG_SENINF_MUX_CROP_EN_SHIFT 28
#define RG_SENINF_MUX_CROP_EN_MASK (0x1 << 28)

#define SENINF_MUX_OPT 0x0008
#define RG_SENINF_MUX_CNT_INIT_OPT_SHIFT 0
#define RG_SENINF_MUX_CNT_INIT_OPT_MASK (0x3 << 0)
#define RG_SENINF_MUX_VS_SPLIT_EN_SHIFT 3
#define RG_SENINF_MUX_VS_SPLIT_EN_MASK (0x1 << 3)
#define RG_SENINF_MUX_FIFO_FULL_OUTPUT_OPT_SHIFT 8
#define RG_SENINF_MUX_FIFO_FULL_OUTPUT_OPT_MASK (0x3 << 8)
#define RG_SENINF_MUX_FIFO_FULL_WR_MODE_B1B0_SHIFT 10
#define RG_SENINF_MUX_FIFO_FULL_WR_MODE_B1B0_MASK (0x3 << 10)
#define RG_SENINF_MUX_FIFO_OVERRUN_RST_EN_SHIFT 12
#define RG_SENINF_MUX_FIFO_OVERRUN_RST_EN_MASK (0x1 << 12)
#define RG_SENINF_MUX_FIFO_FULL_OUTPUT_OPT_B2_SHIFT 13
#define RG_SENINF_MUX_FIFO_FULL_OUTPUT_OPT_B2_MASK (0x1 << 13)
#define RG_SENINF_MUX_FIFO_FULL_WR_MODE_B3B2_SHIFT 14
#define RG_SENINF_MUX_FIFO_FULL_WR_MODE_B3B2_MASK (0x3 << 14)
#define RG_SENINF_MUX_HSYNC_POL_SHIFT 16
#define RG_SENINF_MUX_HSYNC_POL_MASK (0x1 << 16)
#define RG_SENINF_MUX_VSYNC_POL_SHIFT 17
#define RG_SENINF_MUX_VSYNC_POL_MASK (0x1 << 17)
#define RG_SENINF_MUX_FULL_CNT_SHIFT 18
#define RG_SENINF_MUX_FULL_CNT_MASK (0x3fff << 18)

#define SENINF_MUX_IRQ_EN 0x0010
#define RG_SENINF_MUX_FIFO_OVERRUN_IRQ_EN_SHIFT 0
#define RG_SENINF_MUX_FIFO_OVERRUN_IRQ_EN_MASK (0x1 << 0)
#define RG_SENINF_MUX_FSM_ERR_IRQ_EN_SHIFT 1
#define RG_SENINF_MUX_FSM_ERR_IRQ_EN_MASK (0x1 << 1)
#define RG_SENINF_MUX_HSIZE_ERR_IRQ_EN_SHIFT 2
#define RG_SENINF_MUX_HSIZE_ERR_IRQ_EN_MASK (0x1 << 2)
#define RG_SENINF_MUX_VSIZE_ERR_IRQ_EN_SHIFT 3
#define RG_SENINF_MUX_VSIZE_ERR_IRQ_EN_MASK (0x1 << 3)
#define RG_SENINF_MUX_IRQ_CLR_MODE_SHIFT 31
#define RG_SENINF_MUX_IRQ_CLR_MODE_MASK (0x1 << 31)

#define SENINF_MUX_IRQ_STATUS 0x0018
#define RO_SENINF_MUX_FIFO_OVERRUN_IRQ_SHIFT 0
#define RO_SENINF_MUX_FIFO_OVERRUN_IRQ_MASK (0x1 << 0)
#define RO_SENINF_MUX_FSM_ERR_IRQ_SHIFT 1
#define RO_SENINF_MUX_FSM_ERR_IRQ_MASK (0x1 << 1)
#define RO_SENINF_MUX_HSIZE_ERR_IRQ_SHIFT 2
#define RO_SENINF_MUX_HSIZE_ERR_IRQ_MASK (0x1 << 2)
#define RO_SENINF_MUX_VSIZE_ERR_IRQ_SHIFT 3
#define RO_SENINF_MUX_VSIZE_ERR_IRQ_MASK (0x1 << 3)

#define SENINF_MUX_IRQ_TRIG 0x001c
#define RG_SENINF_MUX_IRQ_TRIG_SHIFT 0
#define RG_SENINF_MUX_IRQ_TRIG_MASK (0xf << 0)
#define RG_SENINF_MUX_IRQ_VERIF_EN_SHIFT 31
#define RG_SENINF_MUX_IRQ_VERIF_EN_MASK (0x1 << 31)

#define SENINF_MUX_IMG_SIZE 0x0020
#define RG_SENINF_MUX_EXPECT_HSIZE_SHIFT 0
#define RG_SENINF_MUX_EXPECT_HSIZE_MASK (0xffff << 0)
#define RG_SENINF_MUX_EXPECT_VSIZE_SHIFT 16
#define RG_SENINF_MUX_EXPECT_VSIZE_MASK (0xffff << 16)

#define SENINF_MUX_CROP_PIX_CTRL 0x0028
#define RG_SENINF_MUX_CROP_START_NPIX_CNT_SHIFT 0
#define RG_SENINF_MUX_CROP_START_NPIX_CNT_MASK (0xfff << 0)
#define RG_SENINF_MUX_CROP_END_NPIX_CNT_SHIFT 16
#define RG_SENINF_MUX_CROP_END_NPIX_CNT_MASK (0xfff << 16)

#define SENINF_MUX_SIZE 0x0030
#define RO_SENINF_MUX_RCV_HSIZE_SHIFT 0
#define RO_SENINF_MUX_RCV_HSIZE_MASK (0xffff << 0)
#define RO_SENINF_MUX_RCV_VSIZE_SHIFT 16
#define RO_SENINF_MUX_RCV_VSIZE_MASK (0xffff << 16)

#define SENINF_MUX_ERR_SIZE 0x0034
#define RO_SENINF_MUX_RCV_ERR_HSIZE_SHIFT 0
#define RO_SENINF_MUX_RCV_ERR_HSIZE_MASK (0xffff << 0)
#define RO_SENINF_MUX_RCV_ERR_VSIZE_SHIFT 16
#define RO_SENINF_MUX_RCV_ERR_VSIZE_MASK (0xffff << 16)

#define SENINF_MUX_FIFO_STATUS 0x0040
#define RO_SENINF_MUX_FIFO_WA_SHIFT 0
#define RO_SENINF_MUX_FIFO_WA_MASK (0x1ff << 0)
#define RO_SENINF_MUX_FIFO_WCS_SHIFT 12
#define RO_SENINF_MUX_FIFO_WCS_MASK (0x1 << 12)
#define RO_SENINF_MUX_FIFO_RA_SHIFT 16
#define RO_SENINF_MUX_FIFO_RA_MASK (0x1ff << 16)
#define RO_SENINF_MUX_FIFO_RCS_SHIFT 28
#define RO_SENINF_MUX_FIFO_RCS_MASK (0x1 << 28)

#define SENINF_MUX_DBG_CTRL 0x0080
#define RG_SENINF_MUX_DBG_EN_SHIFT 0
#define RG_SENINF_MUX_DBG_EN_MASK (0x1 << 0)
#define RG_SENINF_MUX_DBG_SEL_SHIFT 8
#define RG_SENINF_MUX_DBG_SEL_MASK (0xff << 8)

#define SENINF_MUX_DBG_OUT 0x0088
#define RO_SENINF_MUX_DBG_OUT_SHIFT 0
#define RO_SENINF_MUX_DBG_OUT_MASK (0xffffffff << 0)

#define SENINF_MUX_CAM_MON 0x00a0
#define RO_SENINF_MUX_CAM_MON_0_SHIFT 0
#define RO_SENINF_MUX_CAM_MON_0_MASK (0xffff << 0)
#define RO_SENINF_MUX_CAM_MON_1_SHIFT 16
#define RO_SENINF_MUX_CAM_MON_1_MASK (0xffff << 16)

#define SENINF_MUX_PIX_CNT 0x00a4
#define RO_SENINF_MUX_PIX_CNT_SHIFT 0
#define RO_SENINF_MUX_PIX_CNT_MASK (0xffffffff << 0)

#define SENINF_MUX_FRAME_SIZE_MON_CTRL 0x00a8
#define RG_SENINF_MUX_FRAME_SIZE_MON_EN_SHIFT 0
#define RG_SENINF_MUX_FRAME_SIZE_MON_EN_MASK (0x1 << 0)

#define SENINF_MUX_FRAME_SIZE_MON_H_VALID 0x00b0
#define RO_SENINF_MUX_FRAME_H_VALID_SHIFT 0
#define RO_SENINF_MUX_FRAME_H_VALID_MASK (0xffffffff << 0)

#define SENINF_MUX_FRAME_SIZE_MON_H_BLANK 0x00b4
#define RO_SENINF_MUX_FRAME_H_BLANK_SHIFT 0
#define RO_SENINF_MUX_FRAME_H_BLANK_MASK (0xffffffff << 0)

#define SENINF_MUX_FRAME_SIZE_MON_V_VALID 0x00b8
#define RO_SENINF_MUX_FRAME_V_VALID_SHIFT 0
#define RO_SENINF_MUX_FRAME_V_VALID_MASK (0xffffffff << 0)

#define SENINF_MUX_FRAME_SIZE_MON_V_BLANK 0x00bc
#define RO_SENINF_MUX_FRAME_V_BLANK_SHIFT 0
#define RO_SENINF_MUX_FRAME_V_BLANK_MASK (0xffffffff << 0)

#define SENINF_MUX_SPARE 0x00f0
#define RG_SENINF_MUX_SPARE_0_SHIFT 0
#define RG_SENINF_MUX_SPARE_0_MASK (0xff << 0)
#define RG_SENINF_MUX_SPARE_1_SHIFT 16
#define RG_SENINF_MUX_SPARE_1_MASK (0xff << 16)

#define SENINF_MUX_VC_SEL0 0x0100
#define RG_SENINF_MUX_B0_VC_SEL_SHIFT 0
#define RG_SENINF_MUX_B0_VC_SEL_MASK (0x1f << 0)
#define RG_SENINF_MUX_B0_VC_REF_EN_SHIFT 7
#define RG_SENINF_MUX_B0_VC_REF_EN_MASK (0x1 << 7)
#define RG_SENINF_MUX_B1_VC_SEL_SHIFT 8
#define RG_SENINF_MUX_B1_VC_SEL_MASK (0x1f << 8)
#define RG_SENINF_MUX_B1_VC_REF_EN_SHIFT 15
#define RG_SENINF_MUX_B1_VC_REF_EN_MASK (0x1 << 15)
#define RG_SENINF_MUX_B2_VC_SEL_SHIFT 16
#define RG_SENINF_MUX_B2_VC_SEL_MASK (0x1f << 16)
#define RG_SENINF_MUX_B2_VC_REF_EN_SHIFT 23
#define RG_SENINF_MUX_B2_VC_REF_EN_MASK (0x1 << 23)
#define RG_SENINF_MUX_B3_VC_SEL_SHIFT 24
#define RG_SENINF_MUX_B3_VC_SEL_MASK (0x1f << 24)
#define RG_SENINF_MUX_B3_VC_REF_EN_SHIFT 31
#define RG_SENINF_MUX_B3_VC_REF_EN_MASK (0x1 << 31)

#define SENINF_MUX_VC_SEL1 0x0104
#define RG_SENINF_MUX_B4_VC_SEL_SHIFT 0
#define RG_SENINF_MUX_B4_VC_SEL_MASK (0x1f << 0)
#define RG_SENINF_MUX_B4_VC_REF_EN_SHIFT 7
#define RG_SENINF_MUX_B4_VC_REF_EN_MASK (0x1 << 7)
#define RG_SENINF_MUX_B5_VC_SEL_SHIFT 8
#define RG_SENINF_MUX_B5_VC_SEL_MASK (0x1f << 8)
#define RG_SENINF_MUX_B5_VC_REF_EN_SHIFT 15
#define RG_SENINF_MUX_B5_VC_REF_EN_MASK (0x1 << 15)
#define RG_SENINF_MUX_B6_VC_SEL_SHIFT 16
#define RG_SENINF_MUX_B6_VC_SEL_MASK (0x1f << 16)
#define RG_SENINF_MUX_B6_VC_REF_EN_SHIFT 23
#define RG_SENINF_MUX_B6_VC_REF_EN_MASK (0x1 << 23)
#define RG_SENINF_MUX_B7_VC_SEL_SHIFT 24
#define RG_SENINF_MUX_B7_VC_SEL_MASK (0x1f << 24)
#define RG_SENINF_MUX_B7_VC_REF_EN_SHIFT 31
#define RG_SENINF_MUX_B7_VC_REF_EN_MASK (0x1 << 31)

#define SENINF_MUX_VC_SEL2 0x0108
#define RG_SENINF_MUX_B8_VC_SEL_SHIFT 0
#define RG_SENINF_MUX_B8_VC_SEL_MASK (0x1f << 0)
#define RG_SENINF_MUX_B8_VC_REF_EN_SHIFT 7
#define RG_SENINF_MUX_B8_VC_REF_EN_MASK (0x1 << 7)
#define RG_SENINF_MUX_B9_VC_SEL_SHIFT 8
#define RG_SENINF_MUX_B9_VC_SEL_MASK (0x1f << 8)
#define RG_SENINF_MUX_B9_VC_REF_EN_SHIFT 15
#define RG_SENINF_MUX_B9_VC_REF_EN_MASK (0x1 << 15)
#define RG_SENINF_MUX_B10_VC_SEL_SHIFT 16
#define RG_SENINF_MUX_B10_VC_SEL_MASK (0x1f << 16)
#define RG_SENINF_MUX_B10_VC_REF_EN_SHIFT 23
#define RG_SENINF_MUX_B10_VC_REF_EN_MASK (0x1 << 23)
#define RG_SENINF_MUX_B11_VC_SEL_SHIFT 24
#define RG_SENINF_MUX_B11_VC_SEL_MASK (0x1f << 24)
#define RG_SENINF_MUX_B11_VC_REF_EN_SHIFT 31
#define RG_SENINF_MUX_B11_VC_REF_EN_MASK (0x1 << 31)

#define SENINF_MUX_VC_SEL3 0x010c
#define RG_SENINF_MUX_B12_VC_SEL_SHIFT 0
#define RG_SENINF_MUX_B12_VC_SEL_MASK (0x1f << 0)
#define RG_SENINF_MUX_B8_VC_REF_EN_SHIFT 7
#define RG_SENINF_MUX_B8_VC_REF_EN_MASK (0x1 << 7)
#define RG_SENINF_MUX_B13_VC_SEL_SHIFT 8
#define RG_SENINF_MUX_B13_VC_SEL_MASK (0x1f << 8)
#define RG_SENINF_MUX_B9_VC_REF_EN_SHIFT 15
#define RG_SENINF_MUX_B9_VC_REF_EN_MASK (0x1 << 15)
#define RG_SENINF_MUX_B14_VC_SEL_SHIFT 16
#define RG_SENINF_MUX_B14_VC_SEL_MASK (0x1f << 16)
#define RG_SENINF_MUX_B10_VC_REF_EN_SHIFT 23
#define RG_SENINF_MUX_B10_VC_REF_EN_MASK (0x1 << 23)
#define RG_SENINF_MUX_B15_VC_SEL_SHIFT 24
#define RG_SENINF_MUX_B15_VC_SEL_MASK (0x1f << 24)
#define RG_SENINF_MUX_B11_VC_REF_EN_SHIFT 31
#define RG_SENINF_MUX_B11_VC_REF_EN_MASK (0x1 << 31)

#define SENINF_MUX_VC_SEL4 0x0110
#define RG_SENINF_MUX_B16_VC_SEL_SHIFT 0
#define RG_SENINF_MUX_B16_VC_SEL_MASK (0x1f << 0)
#define RG_SENINF_MUX_B16_VC_REF_EN_SHIFT 7
#define RG_SENINF_MUX_B16_VC_REF_EN_MASK (0x1 << 7)
#define RG_SENINF_MUX_B17_VC_SEL_SHIFT 8
#define RG_SENINF_MUX_B17_VC_SEL_MASK (0x1f << 8)
#define RG_SENINF_MUX_B17_VC_REF_EN_SHIFT 15
#define RG_SENINF_MUX_B17_VC_REF_EN_MASK (0x1 << 15)
#define RG_SENINF_MUX_B18_VC_SEL_SHIFT 16
#define RG_SENINF_MUX_B18_VC_SEL_MASK (0x1f << 16)
#define RG_SENINF_MUX_B18_VC_REF_EN_SHIFT 23
#define RG_SENINF_MUX_B18_VC_REF_EN_MASK (0x1 << 23)
#define RG_SENINF_MUX_B19_VC_SEL_SHIFT 24
#define RG_SENINF_MUX_B19_VC_SEL_MASK (0x1f << 24)
#define RG_SENINF_MUX_B19_VC_REF_EN_SHIFT 31
#define RG_SENINF_MUX_B19_VC_REF_EN_MASK (0x1 << 31)

#define SENINF_MUX_VC_SEL5 0x0114
#define RG_SENINF_MUX_B20_VC_SEL_SHIFT 0
#define RG_SENINF_MUX_B20_VC_SEL_MASK (0x1f << 0)
#define RG_SENINF_MUX_B20_VC_REF_EN_SHIFT 7
#define RG_SENINF_MUX_B20_VC_REF_EN_MASK (0x1 << 7)
#define RG_SENINF_MUX_B21_VC_SEL_SHIFT 8
#define RG_SENINF_MUX_B21_VC_SEL_MASK (0x1f << 8)
#define RG_SENINF_MUX_B21_VC_REF_EN_SHIFT 15
#define RG_SENINF_MUX_B21_VC_REF_EN_MASK (0x1 << 15)
#define RG_SENINF_MUX_B22_VC_SEL_SHIFT 16
#define RG_SENINF_MUX_B22_VC_SEL_MASK (0x1f << 16)
#define RG_SENINF_MUX_B22_VC_REF_EN_SHIFT 23
#define RG_SENINF_MUX_B22_VC_REF_EN_MASK (0x1 << 23)
#define RG_SENINF_MUX_B23_VC_SEL_SHIFT 24
#define RG_SENINF_MUX_B23_VC_SEL_MASK (0x1f << 24)
#define RG_SENINF_MUX_B23_VC_REF_EN_SHIFT 31
#define RG_SENINF_MUX_B23_VC_REF_EN_MASK (0x1 << 31)

#define SENINF_MUX_VC_SEL6 0x0118
#define RG_SENINF_MUX_B24_VC_SEL_SHIFT 0
#define RG_SENINF_MUX_B24_VC_SEL_MASK (0x1f << 0)
#define RG_SENINF_MUX_B24_VC_REF_EN_SHIFT 7
#define RG_SENINF_MUX_B24_VC_REF_EN_MASK (0x1 << 7)
#define RG_SENINF_MUX_B25_VC_SEL_SHIFT 8
#define RG_SENINF_MUX_B25_VC_SEL_MASK (0x1f << 8)
#define RG_SENINF_MUX_B25_VC_REF_EN_SHIFT 15
#define RG_SENINF_MUX_B25_VC_REF_EN_MASK (0x1 << 15)
#define RG_SENINF_MUX_B26_VC_SEL_SHIFT 16
#define RG_SENINF_MUX_B26_VC_SEL_MASK (0x1f << 16)
#define RG_SENINF_MUX_B26_VC_REF_EN_SHIFT 23
#define RG_SENINF_MUX_B26_VC_REF_EN_MASK (0x1 << 23)
#define RG_SENINF_MUX_B27_VC_SEL_SHIFT 24
#define RG_SENINF_MUX_B27_VC_SEL_MASK (0x1f << 24)
#define RG_SENINF_MUX_B27_VC_REF_EN_SHIFT 31
#define RG_SENINF_MUX_B27_VC_REF_EN_MASK (0x1 << 31)

#define SENINF_MUX_VC_SEL7 0x011c
#define RG_SENINF_MUX_B28_VC_SEL_SHIFT 0
#define RG_SENINF_MUX_B28_VC_SEL_MASK (0x1f << 0)
#define RG_SENINF_MUX_B28_VC_REF_EN_SHIFT 7
#define RG_SENINF_MUX_B28_VC_REF_EN_MASK (0x1 << 7)
#define RG_SENINF_MUX_B29_VC_SEL_SHIFT 8
#define RG_SENINF_MUX_B29_VC_SEL_MASK (0x1f << 8)
#define RG_SENINF_MUX_B29_VC_REF_EN_SHIFT 15
#define RG_SENINF_MUX_B29_VC_REF_EN_MASK (0x1 << 15)
#define RG_SENINF_MUX_B30_VC_SEL_SHIFT 16
#define RG_SENINF_MUX_B30_VC_SEL_MASK (0x1f << 16)
#define RG_SENINF_MUX_B30_VC_REF_EN_SHIFT 23
#define RG_SENINF_MUX_B30_VC_REF_EN_MASK (0x1 << 23)
#define RG_SENINF_MUX_B31_VC_SEL_SHIFT 24
#define RG_SENINF_MUX_B31_VC_SEL_MASK (0x1f << 24)
#define RG_SENINF_MUX_B31_VC_REF_EN_SHIFT 31
#define RG_SENINF_MUX_B31_VC_REF_EN_MASK (0x1 << 31)

#endif
