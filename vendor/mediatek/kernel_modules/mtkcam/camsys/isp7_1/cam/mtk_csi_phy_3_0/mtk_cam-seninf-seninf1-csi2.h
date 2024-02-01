/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __SENINF1_CSI2_H__
#define __SENINF1_CSI2_H__

#define SENINF_CSI2_EN 0x0000
#define CSI2_LANE0_EN_SHIFT 0
#define CSI2_LANE0_EN_MASK (0x1 << 0)
#define CSI2_LANE1_EN_SHIFT 1
#define CSI2_LANE1_EN_MASK (0x1 << 1)
#define CSI2_LANE2_EN_SHIFT 2
#define CSI2_LANE2_EN_MASK (0x1 << 2)
#define CSI2_LANE3_EN_SHIFT 3
#define CSI2_LANE3_EN_MASK (0x1 << 3)

#define SENINF_CSI2_OPT 0x0004
#define RG_CSI2_CPHY_SEL_SHIFT 0
#define RG_CSI2_CPHY_SEL_MASK (0x1 << 0)
#define RG_CSI2_ECC_EN_SHIFT 1
#define RG_CSI2_ECC_EN_MASK (0x1 << 1)
#define RG_CSI2_B2P_EN_SHIFT 2
#define RG_CSI2_B2P_EN_MASK (0x1 << 2)
#define RG_CSI2_GENERIC_LONG_PACKET_EN_SHIFT 3
#define RG_CSI2_GENERIC_LONG_PACKET_EN_MASK (0x1 << 3)
#define RG_CSI2_IMG_PACKET_EN_SHIFT 4
#define RG_CSI2_IMG_PACKET_EN_MASK (0x1 << 4)
#define RG_CSI2_SPEC_V2P0_SEL_SHIFT 5
#define RG_CSI2_SPEC_V2P0_SEL_MASK (0x1 << 5)
#define RG_CSI2_DESCRAMBLE_EN_SHIFT 6
#define RG_CSI2_DESCRAMBLE_EN_MASK (0x1 << 6)
#define RG_CSI2_VS_OUTPUT_MODE_SHIFT 8
#define RG_CSI2_VS_OUTPUT_MODE_MASK (0x1 << 8)
#define RG_CSI2_VS_OUTPUT_LEN_SEL_SHIFT 9
#define RG_CSI2_VS_OUTPUT_LEN_SEL_MASK (0x1 << 9)
#define RG_CSI2_HSYNC_POL_SHIFT 12
#define RG_CSI2_HSYNC_POL_MASK (0x1 << 12)
#define RG_CSI2_VSYNC_POL_SHIFT 13
#define RG_CSI2_VSYNC_POL_MASK (0x1 << 13)
#define RG_CSI2_FIFO_PUSH_EN_SHIFT 16
#define RG_CSI2_FIFO_PUSH_EN_MASK (0x3f << 16)

#define SENINF_CSI2_HDR_MODE_0 0x0008
#define RG_CSI2_HEADER_MODE_SHIFT 0
#define RG_CSI2_HEADER_MODE_MASK (0xff << 0)
#define RG_CSI2_HEADER_LEN_SHIFT 8
#define RG_CSI2_HEADER_LEN_MASK (0x7 << 8)

#define SENINF_CSI2_HDR_MODE_1 0x000c
#define RG_CSI2_CPHY_HEADER_DI_POS_SHIFT 0
#define RG_CSI2_CPHY_HEADER_DI_POS_MASK (0xff << 0)
#define RG_CSI2_CPHY_HEADER_WC_POS_SHIFT 8
#define RG_CSI2_CPHY_HEADER_WC_POS_MASK (0xff << 8)
#define RG_CSI2_CPHY_HEADER_VCX_POS_SHIFT 16
#define RG_CSI2_CPHY_HEADER_VCX_POS_MASK (0xff << 16)

#define SENINF_CSI2_RESYNC_MERGE_CTRL 0x0010
#define RG_CSI2_RESYNC_CYCLE_CNT_SHIFT 0
#define RG_CSI2_RESYNC_CYCLE_CNT_MASK (0x1f << 0)
#define RG_CSI2_RESYNC_CYCLE_CNT_OPT_SHIFT 8
#define RG_CSI2_RESYNC_CYCLE_CNT_OPT_MASK (0x1 << 8)
#define RG_CSI2_RESYNC_DATAOUT_OPT_SHIFT 9
#define RG_CSI2_RESYNC_DATAOUT_OPT_MASK (0x1 << 9)
#define RG_CSI2_RESYNC_BYPASS_SHIFT 10
#define RG_CSI2_RESYNC_BYPASS_MASK (0x1 << 10)
#define RG_CSI2_RESYNC_LRTE_EN_SHIFT 11
#define RG_CSI2_RESYNC_LRTE_EN_MASK (0x1 << 11)
#define RG_CSI2_RESYNC_DMY_EN_SHIFT 12
#define RG_CSI2_RESYNC_DMY_EN_MASK (0xf << 12)
#define RG_CSI2_RESYNC_DMY_CYCLE_SHIFT 16
#define RG_CSI2_RESYNC_DMY_CYCLE_MASK (0xfff << 16)
#define RG_CSI2_RESYNC_DMY_CNT_SHIFT 28
#define RG_CSI2_RESYNC_DMY_CNT_MASK (0xf << 28)

#define SENINF_CSI2_DPCM_CTRL 0x0018
#define RG_CSI2_DPCM_MODE_SHIFT 0
#define RG_CSI2_DPCM_MODE_MASK (0xf << 0)
#define RG_CSI2_DT_30_DPCM_EN_SHIFT 8
#define RG_CSI2_DT_30_DPCM_EN_MASK (0x1 << 8)
#define RG_CSI2_DT_31_DPCM_EN_SHIFT 9
#define RG_CSI2_DT_31_DPCM_EN_MASK (0x1 << 9)
#define RG_CSI2_DT_32_DPCM_EN_SHIFT 10
#define RG_CSI2_DT_32_DPCM_EN_MASK (0x1 << 10)
#define RG_CSI2_DT_33_DPCM_EN_SHIFT 11
#define RG_CSI2_DT_33_DPCM_EN_MASK (0x1 << 11)
#define RG_CSI2_DT_34_DPCM_EN_SHIFT 12
#define RG_CSI2_DT_34_DPCM_EN_MASK (0x1 << 12)
#define RG_CSI2_DT_35_DPCM_EN_SHIFT 13
#define RG_CSI2_DT_35_DPCM_EN_MASK (0x1 << 13)
#define RG_CSI2_DT_36_DPCM_EN_SHIFT 14
#define RG_CSI2_DT_36_DPCM_EN_MASK (0x1 << 14)
#define RG_CSI2_DT_37_DPCM_EN_SHIFT 15
#define RG_CSI2_DT_37_DPCM_EN_MASK (0x1 << 15)
#define RG_CSI2_DT_2A_DPCM_EN_SHIFT 16
#define RG_CSI2_DT_2A_DPCM_EN_MASK (0x1 << 16)

#define SENINF_CSI2_S0_DI_CTRL 0x0020
#define RG_CSI2_S0_VC_INTERLEAVE_EN_SHIFT 0
#define RG_CSI2_S0_VC_INTERLEAVE_EN_MASK (0x1 << 0)
#define RG_CSI2_S0_DT_INTERLEAVE_MODE_SHIFT 4
#define RG_CSI2_S0_DT_INTERLEAVE_MODE_MASK (0x3 << 4)
#define RG_CSI2_S0_VC_SEL_SHIFT 8
#define RG_CSI2_S0_VC_SEL_MASK (0x1f << 8)
#define RG_CSI2_S0_DT_SEL_SHIFT 16
#define RG_CSI2_S0_DT_SEL_MASK (0x3f << 16)

#define SENINF_CSI2_S1_DI_CTRL 0x0024
#define RG_CSI2_S1_VC_INTERLEAVE_EN_SHIFT 0
#define RG_CSI2_S1_VC_INTERLEAVE_EN_MASK (0x1 << 0)
#define RG_CSI2_S1_DT_INTERLEAVE_MODE_SHIFT 4
#define RG_CSI2_S1_DT_INTERLEAVE_MODE_MASK (0x3 << 4)
#define RG_CSI2_S1_VC_SEL_SHIFT 8
#define RG_CSI2_S1_VC_SEL_MASK (0x1f << 8)
#define RG_CSI2_S1_DT_SEL_SHIFT 16
#define RG_CSI2_S1_DT_SEL_MASK (0x3f << 16)

#define SENINF_CSI2_S2_DI_CTRL 0x0028
#define RG_CSI2_S2_VC_INTERLEAVE_EN_SHIFT 0
#define RG_CSI2_S2_VC_INTERLEAVE_EN_MASK (0x1 << 0)
#define RG_CSI2_S2_DT_INTERLEAVE_MODE_SHIFT 4
#define RG_CSI2_S2_DT_INTERLEAVE_MODE_MASK (0x3 << 4)
#define RG_CSI2_S2_VC_SEL_SHIFT 8
#define RG_CSI2_S2_VC_SEL_MASK (0x1f << 8)
#define RG_CSI2_S2_DT_SEL_SHIFT 16
#define RG_CSI2_S2_DT_SEL_MASK (0x3f << 16)

#define SENINF_CSI2_S3_DI_CTRL 0x002c
#define RG_CSI2_S3_VC_INTERLEAVE_EN_SHIFT 0
#define RG_CSI2_S3_VC_INTERLEAVE_EN_MASK (0x1 << 0)
#define RG_CSI2_S3_DT_INTERLEAVE_MODE_SHIFT 4
#define RG_CSI2_S3_DT_INTERLEAVE_MODE_MASK (0x3 << 4)
#define RG_CSI2_S3_VC_SEL_SHIFT 8
#define RG_CSI2_S3_VC_SEL_MASK (0x1f << 8)
#define RG_CSI2_S3_DT_SEL_SHIFT 16
#define RG_CSI2_S3_DT_SEL_MASK (0x3f << 16)

#define SENINF_CSI2_S4_DI_CTRL 0x0030
#define RG_CSI2_S4_VC_INTERLEAVE_EN_SHIFT 0
#define RG_CSI2_S4_VC_INTERLEAVE_EN_MASK (0x1 << 0)
#define RG_CSI2_S4_DT_INTERLEAVE_MODE_SHIFT 4
#define RG_CSI2_S4_DT_INTERLEAVE_MODE_MASK (0x3 << 4)
#define RG_CSI2_S4_VC_SEL_SHIFT 8
#define RG_CSI2_S4_VC_SEL_MASK (0x1f << 8)
#define RG_CSI2_S4_DT_SEL_SHIFT 16
#define RG_CSI2_S4_DT_SEL_MASK (0x3f << 16)

#define SENINF_CSI2_S5_DI_CTRL 0x0034
#define RG_CSI2_S5_VC_INTERLEAVE_EN_SHIFT 0
#define RG_CSI2_S5_VC_INTERLEAVE_EN_MASK (0x1 << 0)
#define RG_CSI2_S5_DT_INTERLEAVE_MODE_SHIFT 4
#define RG_CSI2_S5_DT_INTERLEAVE_MODE_MASK (0x3 << 4)
#define RG_CSI2_S5_VC_SEL_SHIFT 8
#define RG_CSI2_S5_VC_SEL_MASK (0x1f << 8)
#define RG_CSI2_S5_DT_SEL_SHIFT 16
#define RG_CSI2_S5_DT_SEL_MASK (0x3f << 16)

#define SENINF_CSI2_S6_DI_CTRL 0x0038
#define RG_CSI2_S6_VC_INTERLEAVE_EN_SHIFT 0
#define RG_CSI2_S6_VC_INTERLEAVE_EN_MASK (0x1 << 0)
#define RG_CSI2_S6_DT_INTERLEAVE_MODE_SHIFT 4
#define RG_CSI2_S6_DT_INTERLEAVE_MODE_MASK (0x3 << 4)
#define RG_CSI2_S6_VC_SEL_SHIFT 8
#define RG_CSI2_S6_VC_SEL_MASK (0x1f << 8)
#define RG_CSI2_S6_DT_SEL_SHIFT 16
#define RG_CSI2_S6_DT_SEL_MASK (0x3f << 16)

#define SENINF_CSI2_S7_DI_CTRL 0x003c
#define RG_CSI2_S7_VC_INTERLEAVE_EN_SHIFT 0
#define RG_CSI2_S7_VC_INTERLEAVE_EN_MASK (0x1 << 0)
#define RG_CSI2_S7_DT_INTERLEAVE_MODE_SHIFT 4
#define RG_CSI2_S7_DT_INTERLEAVE_MODE_MASK (0x3 << 4)
#define RG_CSI2_S7_VC_SEL_SHIFT 8
#define RG_CSI2_S7_VC_SEL_MASK (0x1f << 8)
#define RG_CSI2_S7_DT_SEL_SHIFT 16
#define RG_CSI2_S7_DT_SEL_MASK (0x3f << 16)

#define SENINF_CSI2_CH0_CTRL 0x0060
#define RG_CSI2_CH0_EXCL_EN_SHIFT 0
#define RG_CSI2_CH0_EXCL_EN_MASK (0x1 << 0)
#define RG_CSI2_CH0_GRP_MODE_SHIFT 1
#define RG_CSI2_CH0_GRP_MODE_MASK (0x1 << 1)
#define RG_CSI2_CH0_VSYNC_BYPASS_SHIFT 3
#define RG_CSI2_CH0_VSYNC_BYPASS_MASK (0x1 << 3)
#define RG_CSI2_CH0_S0_GRP_EN_SHIFT 8
#define RG_CSI2_CH0_S0_GRP_EN_MASK (0x1 << 8)
#define RG_CSI2_CH0_S1_GRP_EN_SHIFT 9
#define RG_CSI2_CH0_S1_GRP_EN_MASK (0x1 << 9)
#define RG_CSI2_CH0_S2_GRP_EN_SHIFT 10
#define RG_CSI2_CH0_S2_GRP_EN_MASK (0x1 << 10)
#define RG_CSI2_CH0_S3_GRP_EN_SHIFT 11
#define RG_CSI2_CH0_S3_GRP_EN_MASK (0x1 << 11)
#define RG_CSI2_CH0_S4_GRP_EN_SHIFT 12
#define RG_CSI2_CH0_S4_GRP_EN_MASK (0x1 << 12)
#define RG_CSI2_CH0_S5_GRP_EN_SHIFT 13
#define RG_CSI2_CH0_S5_GRP_EN_MASK (0x1 << 13)
#define RG_CSI2_CH0_S6_GRP_EN_SHIFT 14
#define RG_CSI2_CH0_S6_GRP_EN_MASK (0x1 << 14)
#define RG_CSI2_CH0_S7_GRP_EN_SHIFT 15
#define RG_CSI2_CH0_S7_GRP_EN_MASK (0x1 << 15)

#define SENINF_CSI2_CH1_CTRL 0x0064
#define RG_CSI2_CH1_EXCL_EN_SHIFT 0
#define RG_CSI2_CH1_EXCL_EN_MASK (0x1 << 0)
#define RG_CSI2_CH1_GRP_MODE_SHIFT 1
#define RG_CSI2_CH1_GRP_MODE_MASK (0x1 << 1)
#define RG_CSI2_CH1_VSYNC_BYPASS_SHIFT 3
#define RG_CSI2_CH1_VSYNC_BYPASS_MASK (0x1 << 3)
#define RG_CSI2_CH1_S0_GRP_EN_SHIFT 8
#define RG_CSI2_CH1_S0_GRP_EN_MASK (0x1 << 8)
#define RG_CSI2_CH1_S1_GRP_EN_SHIFT 9
#define RG_CSI2_CH1_S1_GRP_EN_MASK (0x1 << 9)
#define RG_CSI2_CH1_S2_GRP_EN_SHIFT 10
#define RG_CSI2_CH1_S2_GRP_EN_MASK (0x1 << 10)
#define RG_CSI2_CH1_S3_GRP_EN_SHIFT 11
#define RG_CSI2_CH1_S3_GRP_EN_MASK (0x1 << 11)
#define RG_CSI2_CH1_S4_GRP_EN_SHIFT 12
#define RG_CSI2_CH1_S4_GRP_EN_MASK (0x1 << 12)
#define RG_CSI2_CH1_S5_GRP_EN_SHIFT 13
#define RG_CSI2_CH1_S5_GRP_EN_MASK (0x1 << 13)
#define RG_CSI2_CH1_S6_GRP_EN_SHIFT 14
#define RG_CSI2_CH1_S6_GRP_EN_MASK (0x1 << 14)
#define RG_CSI2_CH1_S7_GRP_EN_SHIFT 15
#define RG_CSI2_CH1_S7_GRP_EN_MASK (0x1 << 15)

#define SENINF_CSI2_CH2_CTRL 0x0068
#define RG_CSI2_CH2_EXCL_EN_SHIFT 0
#define RG_CSI2_CH2_EXCL_EN_MASK (0x1 << 0)
#define RG_CSI2_CH2_GRP_MODE_SHIFT 1
#define RG_CSI2_CH2_GRP_MODE_MASK (0x1 << 1)
#define RG_CSI2_CH2_VSYNC_BYPASS_SHIFT 3
#define RG_CSI2_CH2_VSYNC_BYPASS_MASK (0x1 << 3)
#define RG_CSI2_CH2_S0_GRP_EN_SHIFT 8
#define RG_CSI2_CH2_S0_GRP_EN_MASK (0x1 << 8)
#define RG_CSI2_CH2_S1_GRP_EN_SHIFT 9
#define RG_CSI2_CH2_S1_GRP_EN_MASK (0x1 << 9)
#define RG_CSI2_CH2_S2_GRP_EN_SHIFT 10
#define RG_CSI2_CH2_S2_GRP_EN_MASK (0x1 << 10)
#define RG_CSI2_CH2_S3_GRP_EN_SHIFT 11
#define RG_CSI2_CH2_S3_GRP_EN_MASK (0x1 << 11)
#define RG_CSI2_CH2_S4_GRP_EN_SHIFT 12
#define RG_CSI2_CH2_S4_GRP_EN_MASK (0x1 << 12)
#define RG_CSI2_CH2_S5_GRP_EN_SHIFT 13
#define RG_CSI2_CH2_S5_GRP_EN_MASK (0x1 << 13)
#define RG_CSI2_CH2_S6_GRP_EN_SHIFT 14
#define RG_CSI2_CH2_S6_GRP_EN_MASK (0x1 << 14)
#define RG_CSI2_CH2_S7_GRP_EN_SHIFT 15
#define RG_CSI2_CH2_S7_GRP_EN_MASK (0x1 << 15)

#define SENINF_CSI2_CH3_CTRL 0x006c
#define RG_CSI2_CH3_EXCL_EN_SHIFT 0
#define RG_CSI2_CH3_EXCL_EN_MASK (0x1 << 0)
#define RG_CSI2_CH3_GRP_MODE_SHIFT 1
#define RG_CSI2_CH3_GRP_MODE_MASK (0x1 << 1)
#define RG_CSI2_CH3_VSYNC_BYPASS_SHIFT 3
#define RG_CSI2_CH3_VSYNC_BYPASS_MASK (0x1 << 3)
#define RG_CSI2_CH3_S0_GRP_EN_SHIFT 8
#define RG_CSI2_CH3_S0_GRP_EN_MASK (0x1 << 8)
#define RG_CSI2_CH3_S1_GRP_EN_SHIFT 9
#define RG_CSI2_CH3_S1_GRP_EN_MASK (0x1 << 9)
#define RG_CSI2_CH3_S2_GRP_EN_SHIFT 10
#define RG_CSI2_CH3_S2_GRP_EN_MASK (0x1 << 10)
#define RG_CSI2_CH3_S3_GRP_EN_SHIFT 11
#define RG_CSI2_CH3_S3_GRP_EN_MASK (0x1 << 11)
#define RG_CSI2_CH3_S4_GRP_EN_SHIFT 12
#define RG_CSI2_CH3_S4_GRP_EN_MASK (0x1 << 12)
#define RG_CSI2_CH3_S5_GRP_EN_SHIFT 13
#define RG_CSI2_CH3_S5_GRP_EN_MASK (0x1 << 13)
#define RG_CSI2_CH3_S6_GRP_EN_SHIFT 14
#define RG_CSI2_CH3_S6_GRP_EN_MASK (0x1 << 14)
#define RG_CSI2_CH3_S7_GRP_EN_SHIFT 15
#define RG_CSI2_CH3_S7_GRP_EN_MASK (0x1 << 15)

#define SENINF_CSI2_L0_DESCRAMBLE_SEED_0 0x0080
#define RG_CSI2_L0_DESCRAMBLE_TYPE0_SEED_SHIFT 0
#define RG_CSI2_L0_DESCRAMBLE_TYPE0_SEED_MASK (0xffff << 0)
#define RG_CSI2_L0_DESCRAMBLE_TYPE1_SEED_SHIFT 16
#define RG_CSI2_L0_DESCRAMBLE_TYPE1_SEED_MASK (0xffff << 16)

#define SENINF_CSI2_L0_DESCRAMBLE_SEED_1 0x0084
#define RG_CSI2_L0_DESCRAMBLE_TYPE2_SEED_SHIFT 0
#define RG_CSI2_L0_DESCRAMBLE_TYPE2_SEED_MASK (0xffff << 0)
#define RG_CSI2_L0_DESCRAMBLE_TYPE3_SEED_SHIFT 16
#define RG_CSI2_L0_DESCRAMBLE_TYPE3_SEED_MASK (0xffff << 16)

#define SENINF_CSI2_L1_DESCRAMBLE_SEED_0 0x0090
#define RG_CSI2_L1_DESCRAMBLE_TYPE0_SEED_SHIFT 0
#define RG_CSI2_L1_DESCRAMBLE_TYPE0_SEED_MASK (0xffff << 0)
#define RG_CSI2_L1_DESCRAMBLE_TYPE1_SEED_SHIFT 16
#define RG_CSI2_L1_DESCRAMBLE_TYPE1_SEED_MASK (0xffff << 16)

#define SENINF_CSI2_L1_DESCRAMBLE_SEED_1 0x0094
#define RG_CSI2_L1_DESCRAMBLE_TYPE2_SEED_SHIFT 0
#define RG_CSI2_L1_DESCRAMBLE_TYPE2_SEED_MASK (0xffff << 0)
#define RG_CSI2_L1_DESCRAMBLE_TYPE3_SEED_SHIFT 16
#define RG_CSI2_L1_DESCRAMBLE_TYPE3_SEED_MASK (0xffff << 16)

#define SENINF_CSI2_L2_DESCRAMBLE_SEED_0 0x00a0
#define RG_CSI2_L2_DESCRAMBLE_TYPE0_SEED_SHIFT 0
#define RG_CSI2_L2_DESCRAMBLE_TYPE0_SEED_MASK (0xffff << 0)
#define RG_CSI2_L2_DESCRAMBLE_TYPE1_SEED_SHIFT 16
#define RG_CSI2_L2_DESCRAMBLE_TYPE1_SEED_MASK (0xffff << 16)

#define SENINF_CSI2_L2_DESCRAMBLE_SEED_1 0x00a4
#define RG_CSI2_L2_DESCRAMBLE_TYPE2_SEED_SHIFT 0
#define RG_CSI2_L2_DESCRAMBLE_TYPE2_SEED_MASK (0xffff << 0)
#define RG_CSI2_L2_DESCRAMBLE_TYPE3_SEED_SHIFT 16
#define RG_CSI2_L2_DESCRAMBLE_TYPE3_SEED_MASK (0xffff << 16)

#define SENINF_CSI2_L3_DESCRAMBLE_SEED_0 0x00b0
#define RG_CSI2_L3_DESCRAMBLE_TYPE0_SEED_SHIFT 0
#define RG_CSI2_L3_DESCRAMBLE_TYPE0_SEED_MASK (0xffff << 0)
#define RG_CSI2_L3_DESCRAMBLE_TYPE1_SEED_SHIFT 16
#define RG_CSI2_L3_DESCRAMBLE_TYPE1_SEED_MASK (0xffff << 16)

#define SENINF_CSI2_L3_DESCRAMBLE_SEED_1 0x00b4
#define RG_CSI2_L3_DESCRAMBLE_TYPE2_SEED_SHIFT 0
#define RG_CSI2_L3_DESCRAMBLE_TYPE2_SEED_MASK (0xffff << 0)
#define RG_CSI2_L3_DESCRAMBLE_TYPE3_SEED_SHIFT 16
#define RG_CSI2_L3_DESCRAMBLE_TYPE3_SEED_MASK (0xffff << 16)

#define SENINF_CSI2_IRQ_EN 0x00c0
#define RG_CSI2_ERR_FRAME_SYNC_IRQ_EN_SHIFT 0
#define RG_CSI2_ERR_FRAME_SYNC_IRQ_EN_MASK (0x1 << 0)
#define RG_CSI2_ERR_ID_IRQ_EN_SHIFT 1
#define RG_CSI2_ERR_ID_IRQ_EN_MASK (0x1 << 1)
#define RG_CSI2_ECC_NO_ERR_IRQ_EN_SHIFT 2
#define RG_CSI2_ECC_NO_ERR_IRQ_EN_MASK (0x1 << 2)
#define RG_CSI2_ECC_ERR_CORRECTED_IRQ_EN_SHIFT 3
#define RG_CSI2_ECC_ERR_CORRECTED_IRQ_EN_MASK (0x1 << 3)
#define RG_CSI2_ECC_ERR_DOUBLE_IRQ_EN_SHIFT 4
#define RG_CSI2_ECC_ERR_DOUBLE_IRQ_EN_MASK (0x1 << 4)
#define RG_CSI2_CRC_CORRECT_IRQ_EN_SHIFT 5
#define RG_CSI2_CRC_CORRECT_IRQ_EN_MASK (0x1 << 5)
#define RG_CSI2_CRC_ERR_IRQ_EN_SHIFT 6
#define RG_CSI2_CRC_ERR_IRQ_EN_MASK (0x1 << 6)
#define RG_CSI2_ERR_MULTI_LANE_SYNC_IRQ_EN_SHIFT 7
#define RG_CSI2_ERR_MULTI_LANE_SYNC_IRQ_EN_MASK (0x1 << 7)
#define RG_CSI2_FS_RECEIVE_IRQ_EN_SHIFT 8
#define RG_CSI2_FS_RECEIVE_IRQ_EN_MASK (0x1 << 8)
#define RG_CSI2_FE_RECEIVE_IRQ_EN_SHIFT 9
#define RG_CSI2_FE_RECEIVE_IRQ_EN_MASK (0x1 << 9)
#define RG_CSI2_LS_RECEIVE_IRQ_EN_SHIFT 10
#define RG_CSI2_LS_RECEIVE_IRQ_EN_MASK (0x1 << 10)
#define RG_CSI2_LE_RECEIVE_IRQ_EN_SHIFT 11
#define RG_CSI2_LE_RECEIVE_IRQ_EN_MASK (0x1 << 11)
#define RG_CSI2_GS_RECEIVE_IRQ_EN_SHIFT 12
#define RG_CSI2_GS_RECEIVE_IRQ_EN_MASK (0x1 << 12)
#define RG_CSI2_ERR_LANE_RESYNC_IRQ_EN_SHIFT 13
#define RG_CSI2_ERR_LANE_RESYNC_IRQ_EN_MASK (0x1 << 13)
#define RG_CSI2_LANE_MERGE_FIFO_AF_IRQ_EN_SHIFT 14
#define RG_CSI2_LANE_MERGE_FIFO_AF_IRQ_EN_MASK (0x1 << 14)
#define RG_CSI2_ERR_FRAME_SYNC_S0_IRQ_EN_SHIFT 16
#define RG_CSI2_ERR_FRAME_SYNC_S0_IRQ_EN_MASK (0x1 << 16)
#define RG_CSI2_ERR_FRAME_SYNC_S1_IRQ_EN_SHIFT 17
#define RG_CSI2_ERR_FRAME_SYNC_S1_IRQ_EN_MASK (0x1 << 17)
#define RG_CSI2_ERR_FRAME_SYNC_S2_IRQ_EN_SHIFT 18
#define RG_CSI2_ERR_FRAME_SYNC_S2_IRQ_EN_MASK (0x1 << 18)
#define RG_CSI2_ERR_FRAME_SYNC_S3_IRQ_EN_SHIFT 19
#define RG_CSI2_ERR_FRAME_SYNC_S3_IRQ_EN_MASK (0x1 << 19)
#define RG_CSI2_ERR_FRAME_SYNC_S4_IRQ_EN_SHIFT 20
#define RG_CSI2_ERR_FRAME_SYNC_S4_IRQ_EN_MASK (0x1 << 20)
#define RG_CSI2_ERR_FRAME_SYNC_S5_IRQ_EN_SHIFT 21
#define RG_CSI2_ERR_FRAME_SYNC_S5_IRQ_EN_MASK (0x1 << 21)
#define RG_CSI2_ERR_FRAME_SYNC_S6_IRQ_EN_SHIFT 22
#define RG_CSI2_ERR_FRAME_SYNC_S6_IRQ_EN_MASK (0x1 << 22)
#define RG_CSI2_ERR_FRAME_SYNC_S7_IRQ_EN_SHIFT 23
#define RG_CSI2_ERR_FRAME_SYNC_S7_IRQ_EN_MASK (0x1 << 23)
#define RG_CSI2_RESYNC_FIFO_OVERFLOW_L0_IRQ_EN_SHIFT 24
#define RG_CSI2_RESYNC_FIFO_OVERFLOW_L0_IRQ_EN_MASK (0x1 << 24)
#define RG_CSI2_RESYNC_FIFO_OVERFLOW_L1_IRQ_EN_SHIFT 25
#define RG_CSI2_RESYNC_FIFO_OVERFLOW_L1_IRQ_EN_MASK (0x1 << 25)
#define RG_CSI2_RESYNC_FIFO_OVERFLOW_L2_IRQ_EN_SHIFT 26
#define RG_CSI2_RESYNC_FIFO_OVERFLOW_L2_IRQ_EN_MASK (0x1 << 26)
#define RG_CSI2_RESYNC_FIFO_OVERFLOW_L3_IRQ_EN_SHIFT 27
#define RG_CSI2_RESYNC_FIFO_OVERFLOW_L3_IRQ_EN_MASK (0x1 << 27)
#define RG_CSI2_ASYNC_FIFO_OVERRUN_IRQ_EN_SHIFT 28
#define RG_CSI2_ASYNC_FIFO_OVERRUN_IRQ_EN_MASK (0x1 << 28)
#define RG_CSI2_RECEIVE_DATA_NOT_ENOUGH_IRQ_EN_SHIFT 29
#define RG_CSI2_RECEIVE_DATA_NOT_ENOUGH_IRQ_EN_MASK (0x1 << 29)
#define RG_CSI2_IRQ_CLR_MODE_SHIFT 31
#define RG_CSI2_IRQ_CLR_MODE_MASK (0x1 << 31)

#define SENINF_CSI2_IRQ_STATUS 0x00c8
#define RO_CSI2_ERR_FRAME_SYNC_IRQ_SHIFT 0
#define RO_CSI2_ERR_FRAME_SYNC_IRQ_MASK (0x1 << 0)
#define RO_CSI2_ERR_ID_IRQ_SHIFT 1
#define RO_CSI2_ERR_ID_IRQ_MASK (0x1 << 1)
#define RO_CSI2_ECC_NO_ERR_IRQ_SHIFT 2
#define RO_CSI2_ECC_NO_ERR_IRQ_MASK (0x1 << 2)
#define RO_CSI2_ECC_ERR_CORRECTED_IRQ_SHIFT 3
#define RO_CSI2_ECC_ERR_CORRECTED_IRQ_MASK (0x1 << 3)
#define RO_CSI2_ECC_ERR_DOUBLE_IRQ_SHIFT 4
#define RO_CSI2_ECC_ERR_DOUBLE_IRQ_MASK (0x1 << 4)
#define RO_CSI2_CRC_CORRECT_IRQ_SHIFT 5
#define RO_CSI2_CRC_CORRECT_IRQ_MASK (0x1 << 5)
#define RO_CSI2_CRC_ERR_IRQ_SHIFT 6
#define RO_CSI2_CRC_ERR_IRQ_MASK (0x1 << 6)
#define RO_CSI2_ERR_MULTI_LANE_SYNC_IRQ_SHIFT 7
#define RO_CSI2_ERR_MULTI_LANE_SYNC_IRQ_MASK (0x1 << 7)
#define RO_CSI2_FS_RECEIVE_IRQ_SHIFT 8
#define RO_CSI2_FS_RECEIVE_IRQ_MASK (0x1 << 8)
#define RO_CSI2_FE_RECEIVE_IRQ_SHIFT 9
#define RO_CSI2_FE_RECEIVE_IRQ_MASK (0x1 << 9)
#define RO_CSI2_LS_RECEIVE_IRQ_SHIFT 10
#define RO_CSI2_LS_RECEIVE_IRQ_MASK (0x1 << 10)
#define RO_CSI2_LE_RECEIVE_IRQ_SHIFT 11
#define RO_CSI2_LE_RECEIVE_IRQ_MASK (0x1 << 11)
#define RO_CSI2_GS_RECEIVE_IRQ_SHIFT 12
#define RO_CSI2_GS_RECEIVE_IRQ_MASK (0x1 << 12)
#define RO_CSI2_ERR_LANE_RESYNC_IRQ_SHIFT 13
#define RO_CSI2_ERR_LANE_RESYNC_IRQ_MASK (0x1 << 13)
#define RO_CSI2_LANE_MERGE_FIFO_AF_IRQ_SHIFT 14
#define RO_CSI2_LANE_MERGE_FIFO_AF_IRQ_MASK (0x1 << 14)
#define RO_CSI2_ERR_FRAME_SYNC_S0_IRQ_SHIFT 16
#define RO_CSI2_ERR_FRAME_SYNC_S0_IRQ_MASK (0x1 << 16)
#define RO_CSI2_ERR_FRAME_SYNC_S1_IRQ_SHIFT 17
#define RO_CSI2_ERR_FRAME_SYNC_S1_IRQ_MASK (0x1 << 17)
#define RO_CSI2_ERR_FRAME_SYNC_S2_IRQ_SHIFT 18
#define RO_CSI2_ERR_FRAME_SYNC_S2_IRQ_MASK (0x1 << 18)
#define RO_CSI2_ERR_FRAME_SYNC_S3_IRQ_SHIFT 19
#define RO_CSI2_ERR_FRAME_SYNC_S3_IRQ_MASK (0x1 << 19)
#define RO_CSI2_ERR_FRAME_SYNC_S4_IRQ_SHIFT 20
#define RO_CSI2_ERR_FRAME_SYNC_S4_IRQ_MASK (0x1 << 20)
#define RO_CSI2_ERR_FRAME_SYNC_S5_IRQ_SHIFT 21
#define RO_CSI2_ERR_FRAME_SYNC_S5_IRQ_MASK (0x1 << 21)
#define RO_CSI2_ERR_FRAME_SYNC_S6_IRQ_SHIFT 22
#define RO_CSI2_ERR_FRAME_SYNC_S6_IRQ_MASK (0x1 << 22)
#define RO_CSI2_ERR_FRAME_SYNC_S7_IRQ_SHIFT 23
#define RO_CSI2_ERR_FRAME_SYNC_S7_IRQ_MASK (0x1 << 23)
#define RO_CSI2_RESYNC_FIFO_OVERFLOW_L0_IRQ_SHIFT 24
#define RO_CSI2_RESYNC_FIFO_OVERFLOW_L0_IRQ_MASK (0x1 << 24)
#define RO_CSI2_RESYNC_FIFO_OVERFLOW_L1_IRQ_SHIFT 25
#define RO_CSI2_RESYNC_FIFO_OVERFLOW_L1_IRQ_MASK (0x1 << 25)
#define RO_CSI2_RESYNC_FIFO_OVERFLOW_L2_IRQ_SHIFT 26
#define RO_CSI2_RESYNC_FIFO_OVERFLOW_L2_IRQ_MASK (0x1 << 26)
#define RO_CSI2_RESYNC_FIFO_OVERFLOW_L3_IRQ_SHIFT 27
#define RO_CSI2_RESYNC_FIFO_OVERFLOW_L3_IRQ_MASK (0x1 << 27)
#define RO_CSI2_ASYNC_FIFO_OVERRUN_IRQ_SHIFT 28
#define RO_CSI2_ASYNC_FIFO_OVERRUN_IRQ_MASK (0x1 << 28)
#define RO_CSI2_RECEIVE_DATA_NOT_ENOUGH_IRQ_SHIFT 29
#define RO_CSI2_RECEIVE_DATA_NOT_ENOUGH_IRQ_MASK (0x1 << 29)

#define SENINF_CSI2_IRQ_TRIG 0x00cc
#define RG_CSI2_IRQ_TRIG_SHIFT 0
#define RG_CSI2_IRQ_TRIG_MASK (0x1fffffff << 0)
#define RG_CSI2_IRQ_VERIF_EN_SHIFT 31
#define RG_CSI2_IRQ_VERIF_EN_MASK (0x1 << 31)

#define SENINF_CSI2_LINE_FRAME_NUM 0x00d0
#define RO_CSI2_LINE_NUM_SHIFT 0
#define RO_CSI2_LINE_NUM_MASK (0xffff << 0)
#define RO_CSI2_FRAME_NUM_SHIFT 16
#define RO_CSI2_FRAME_NUM_MASK (0xffff << 16)

#define SENINF_CSI2_PACKET_STATUS 0x00d4
#define RO_CSI2_PACKET_DT_SHIFT 0
#define RO_CSI2_PACKET_DT_MASK (0x3f << 0)
#define RO_CSI2_PACKET_VC_SHIFT 8
#define RO_CSI2_PACKET_VC_MASK (0x1f << 8)
#define RO_CSI2_PACKET_WC_SHIFT 16
#define RO_CSI2_PACKET_WC_MASK (0xffff << 16)

#define SENINF_CSI2_GEN_SHORT_PACKET_STATUS 0x00d8
#define RO_CSI2_GENERIC_SHORT_PACKET_DT_SHIFT 0
#define RO_CSI2_GENERIC_SHORT_PACKET_DT_MASK (0x3f << 0)
#define RO_CSI2_GENERIC_SHORT_PACKET_WC_SHIFT 16
#define RO_CSI2_GENERIC_SHORT_PACKET_WC_MASK (0xffff << 16)

#define SENINF_CSI2_PACKET_CNT_STATUS 0x00dc
#define RO_CSI2_PACKET_CNT_SHIFT 0
#define RO_CSI2_PACKET_CNT_MASK (0xffff << 0)
#define RO_CSI2_PACKET_CNT_BUF_SHIFT 16
#define RO_CSI2_PACKET_CNT_BUF_MASK (0xffff << 16)

#define SENINF_CSI2_DBG_CTRL 0x00e0
#define RG_CSI2_DBG_SEL_SHIFT 0
#define RG_CSI2_DBG_SEL_MASK (0xff << 0)
#define RG_CSI2_DBG_EN_SHIFT 16
#define RG_CSI2_DBG_EN_MASK (0x1 << 16)
#define RG_CSI2_DBG_PACKET_CNT_EN_SHIFT 17
#define RG_CSI2_DBG_PACKET_CNT_EN_MASK (0x1 << 17)

#define SENINF_CSI2_DBG_OUT 0x00f4
#define RO_CSI2_DBG_OUT_SHIFT 0
#define RO_CSI2_DBG_OUT_MASK (0xffffffff << 0)

#define SENINF_CSI2_SPARE 0x00f8
#define RG_CSI2_SPARE_0_SHIFT 0
#define RG_CSI2_SPARE_0_MASK (0xff << 0)
#define RG_CSI2_SPARE_1_SHIFT 16
#define RG_CSI2_SPARE_1_MASK (0xff << 16)

#define SENINF_CSI2_SOFEOF_PARAMETER 0x00fc
#define RG_CSI2_SOF_PULSE_WIDTH_SHIFT 0
#define RG_CSI2_SOF_PULSE_WIDTH_MASK (0xffff << 0)
#define RG_CSI2_EOF_PULSE_WIDTH_SHIFT 16
#define RG_CSI2_EOF_PULSE_WIDTH_MASK (0xffff << 16)

#define SENINF_CSI2_R24USERDEF_DT 0x0100
#define RG_CSI2_RAW24LIKE_USERDEF_DT_SHIFT 0
#define RG_CSI2_RAW24LIKE_USERDEF_DT_MASK (0x3f << 0)
#define RG_CSI2_USERDEF_DT_EN_SHIFT 8
#define RG_CSI2_USERDEF_DT_EN_MASK (0x1 << 8)
#define RG_ADAS_CTRL_WORD_EN_SHIFT 16
#define RG_ADAS_CTRL_WORD_EN_MASK (0x1 << 16)
#define RG_USER_DEF_38_2_3F_EN_SHIFT 17
#define RG_USER_DEF_38_2_3F_EN_MASK (0x1 << 17)

#define SENINF_CSI2_DT0 0x0104
#define RG_CSI2_RAW10_DT_SHIFT 0
#define RG_CSI2_RAW10_DT_MASK (0x3f << 0)
#define RG_CSI2_RAW12_DT_SHIFT 8
#define RG_CSI2_RAW12_DT_MASK (0x3f << 8)
#define RG_CSI2_RAW14_DT_SHIFT 16
#define RG_CSI2_RAW14_DT_MASK (0x3f << 16)
#define RG_CSI2_RAW16_DT_SHIFT 24
#define RG_CSI2_RAW16_DT_MASK (0x3f << 24)

#define SENINF_CSI2_DT1 0x0108
#define RG_CSI2_RAW20_DT_SHIFT 0
#define RG_CSI2_RAW20_DT_MASK (0x3f << 0)
#define RG_CSI2_YUV420_10_SHIFT 8
#define RG_CSI2_YUV420_10_MASK (0x3f << 8)
#define RG_CSI2_YUV420CSPS_10_SHIFT 16
#define RG_CSI2_YUV420CSPS_10_MASK (0x3f << 16)
#define RG_CSI2_YUV422_10_SHIFT 24
#define RG_CSI2_YUV422_10_MASK (0x3f << 24)

#define SENINF_CSI2_DT2 0x010c
#define RG_CSI2_RGB565_DT_SHIFT 0
#define RG_CSI2_RGB565_DT_MASK (0x3f << 0)
#define RG_CSI2_RGB888_DT_SHIFT 8
#define RG_CSI2_RGB888_DT_MASK (0x3f << 8)
#define RG_FORCE_LONG_PKT_SHIFT 16
#define RG_FORCE_LONG_PKT_MASK (0x1 << 16)
#define RG_FORCE_RAW8_PKT_SHIFT 17
#define RG_FORCE_RAW8_PKT_MASK (0x1 << 17)

#define SENINF_CSI2_FORCEDT0 0x0110
#define RG_FORCE_DT0_SHIFT 0
#define RG_FORCE_DT0_MASK (0x3f << 0)
#define RG_FORCE_DT0_SEL_SHIFT 8
#define RG_FORCE_DT0_SEL_MASK (0x7 << 8)
#define RG_FORCE_DT0_EN_SHIFT 12
#define RG_FORCE_DT0_EN_MASK (0x1 << 12)
#define RG_FORCE_DT1_SHIFT 16
#define RG_FORCE_DT1_MASK (0x3f << 16)
#define RG_FORCE_DT1_SEL_SHIFT 24
#define RG_FORCE_DT1_SEL_MASK (0x7 << 24)
#define RG_FORCE_DT1_EN_SHIFT 28
#define RG_FORCE_DT1_EN_MASK (0x1 << 28)

#define SENINF_CSI2_FORCEDT1 0x0114
#define RG_FORCE_DT2_SHIFT 0
#define RG_FORCE_DT2_MASK (0x3f << 0)
#define RG_FORCE_DT2_SEL_SHIFT 8
#define RG_FORCE_DT2_SEL_MASK (0x7 << 8)
#define RG_FORCE_DT2_EN_SHIFT 12
#define RG_FORCE_DT2_EN_MASK (0x1 << 12)
#define RG_FORCE_DT3_SHIFT 16
#define RG_FORCE_DT3_MASK (0x3f << 16)
#define RG_FORCE_DT3_SEL_SHIFT 24
#define RG_FORCE_DT3_SEL_MASK (0x7 << 24)
#define RG_FORCE_DT3_EN_SHIFT 28
#define RG_FORCE_DT3_EN_MASK (0x1 << 28)

#define SENINF_CSI2_FORCEDT2 0x0118
#define RG_SEQ_DT0_SHIFT 0
#define RG_SEQ_DT0_MASK (0x3f << 0)
#define RG_SEQ10_DT_EN_SHIFT 7
#define RG_SEQ10_DT_EN_MASK (0x1 << 7)
#define RG_SEQ_DT1_SHIFT 8
#define RG_SEQ_DT1_MASK (0x3f << 8)
#define RG_SEQ12_DT_EN_SHIFT 15
#define RG_SEQ12_DT_EN_MASK (0x1 << 15)
#define RG_SEQ_DT2_SHIFT 16
#define RG_SEQ_DT2_MASK (0x3f << 16)
#define RG_SEQ14_DT_EN_SHIFT 23
#define RG_SEQ14_DT_EN_MASK (0x1 << 23)
#define RG_SEQ_DT3_SHIFT 24
#define RG_SEQ_DT3_MASK (0x3f << 24)
#define RG_SEQ_DT_38_TO_3F_EN_SHIFT 30
#define RG_SEQ_DT_38_TO_3F_EN_MASK (0x1 << 30)
#define RG_SEQ_DT_30_TO_37_EN_SHIFT 31
#define RG_SEQ_DT_30_TO_37_EN_MASK (0x1 << 31)

#define SENINF_CSI2_DTCTRL_WORD00 0x0120
#define RG_CSI2_DTCTRL_WORD00_SHIFT 0
#define RG_CSI2_DTCTRL_WORD00_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD01 0x0124
#define RG_CSI2_DTCTRL_WORD01_SHIFT 0
#define RG_CSI2_DTCTRL_WORD01_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD02 0x0128
#define RG_CSI2_DTCTRL_WORD02_SHIFT 0
#define RG_CSI2_DTCTRL_WORD02_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORDDT0 0x012c
#define RG_CSI2_DTCTRL_WORDDT0_SHIFT 0
#define RG_CSI2_DTCTRL_WORDDT0_MASK (0x3f << 0)

#define SENINF_CSI2_DTCTRL_WORD10 0x0130
#define RG_CSI2_DTCTRL_WORD10_SHIFT 0
#define RG_CSI2_DTCTRL_WORD10_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD11 0x0134
#define RG_CSI2_DTCTRL_WORD11_SHIFT 0
#define RG_CSI2_DTCTRL_WORD11_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD12 0x0138
#define RG_CSI2_DTCTRL_WORD12_SHIFT 0
#define RG_CSI2_DTCTRL_WORD12_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORDDT1 0x013c
#define RG_CSI2_DTCTRL_WORDDT1_SHIFT 0
#define RG_CSI2_DTCTRL_WORDDT1_MASK (0x3f << 0)

#define SENINF_CSI2_DTCTRL_WORD20 0x0140
#define RG_CSI2_DTCTRL_WORD20_SHIFT 0
#define RG_CSI2_DTCTRL_WORD20_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD21 0x0144
#define RG_CSI2_DTCTRL_WORD21_SHIFT 0
#define RG_CSI2_DTCTRL_WORD21_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD22 0x0148
#define RG_CSI2_DTCTRL_WORD22_SHIFT 0
#define RG_CSI2_DTCTRL_WORD22_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORDDT2 0x014c
#define RG_CSI2_DTCTRL_WORDDT2_SHIFT 0
#define RG_CSI2_DTCTRL_WORDDT2_MASK (0x3f << 0)

#define SENINF_CSI2_DTCTRL_WORD30 0x0150
#define RG_CSI2_DTCTRL_WORD30_SHIFT 0
#define RG_CSI2_DTCTRL_WORD30_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD31 0x0154
#define RG_CSI2_DTCTRL_WORD31_SHIFT 0
#define RG_CSI2_DTCTRL_WORD31_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD32 0x0158
#define RG_CSI2_DTCTRL_WORD32_SHIFT 0
#define RG_CSI2_DTCTRL_WORD32_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORDDT3 0x015c
#define RG_CSI2_DTCTRL_WORDDT3_SHIFT 0
#define RG_CSI2_DTCTRL_WORDDT3_MASK (0x3f << 0)

#define SENINF_CSI2_DTCTRL_WORD40 0x0160
#define RG_CSI2_DTCTRL_WORD40_SHIFT 0
#define RG_CSI2_DTCTRL_WORD40_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD41 0x0164
#define RG_CSI2_DTCTRL_WORD41_SHIFT 0
#define RG_CSI2_DTCTRL_WORD41_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD42 0x0168
#define RG_CSI2_DTCTRL_WORD42_SHIFT 0
#define RG_CSI2_DTCTRL_WORD42_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORDDT4 0x016c
#define RG_CSI2_DTCTRL_WORDDT4_SHIFT 0
#define RG_CSI2_DTCTRL_WORDDT4_MASK (0x3f << 0)

#define SENINF_CSI2_DTCTRL_WORD50 0x0170
#define RG_CSI2_DTCTRL_WORD50_SHIFT 0
#define RG_CSI2_DTCTRL_WORD50_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD51 0x0174
#define RG_CSI2_DTCTRL_WORD51_SHIFT 0
#define RG_CSI2_DTCTRL_WORD51_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD52 0x0178
#define RG_CSI2_DTCTRL_WORD52_SHIFT 0
#define RG_CSI2_DTCTRL_WORD52_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORDDT5 0x017c
#define RG_CSI2_DTCTRL_WORDDT5_SHIFT 0
#define RG_CSI2_DTCTRL_WORDDT5_MASK (0x3f << 0)

#define SENINF_CSI2_DTCTRL_WORD60 0x0180
#define RG_CSI2_DTCTRL_WORD60_SHIFT 0
#define RG_CSI2_DTCTRL_WORD60_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD61 0x0184
#define RG_CSI2_DTCTRL_WORD61_SHIFT 0
#define RG_CSI2_DTCTRL_WORD61_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD62 0x0188
#define RG_CSI2_DTCTRL_WORD62_SHIFT 0
#define RG_CSI2_DTCTRL_WORD62_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORDDT6 0x018c
#define RG_CSI2_DTCTRL_WORDDT6_SHIFT 0
#define RG_CSI2_DTCTRL_WORDDT6_MASK (0x3f << 0)

#define SENINF_CSI2_DTCTRL_WORD70 0x0190
#define RG_CSI2_DTCTRL_WORD70_SHIFT 0
#define RG_CSI2_DTCTRL_WORD70_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD71 0x0194
#define RG_CSI2_DTCTRL_WORD71_SHIFT 0
#define RG_CSI2_DTCTRL_WORD71_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD72 0x0198
#define RG_CSI2_DTCTRL_WORD72_SHIFT 0
#define RG_CSI2_DTCTRL_WORD72_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORDDT7 0x019c
#define RG_CSI2_DTCTRL_WORDDT7_SHIFT 0
#define RG_CSI2_DTCTRL_WORDDT7_MASK (0x3f << 0)

#define SENINF_CSI2_DTCTRL_WORD_MASK0 0x01a0
#define RG_CSI2_DTCTRL_WORD_MASK0_SHIFT 0
#define RG_CSI2_DTCTRL_WORD_MASK0_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD_MASK1 0x01a4
#define RG_CSI2_DTCTRL_WORD_MASK1_SHIFT 0
#define RG_CSI2_DTCTRL_WORD_MASK1_MASK (0xffffffff << 0)

#define SENINF_CSI2_DTCTRL_WORD_MASK2 0x01a8
#define RG_CSI2_DTCTRL_WORD_MASK2_SHIFT 0
#define RG_CSI2_DTCTRL_WORD_MASK2_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD00 0x0220
#define RG_CSI2_VCCTRL_WORD00_SHIFT 0
#define RG_CSI2_VCCTRL_WORD00_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD01 0x0224
#define RG_CSI2_VCCTRL_WORD01_SHIFT 0
#define RG_CSI2_VCCTRL_WORD01_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD02 0x0228
#define RG_CSI2_VCCTRL_WORD02_SHIFT 0
#define RG_CSI2_VCCTRL_WORD02_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORDDT0 0x022c
#define RG_CSI2_VCCTRL_WORDVC0_SHIFT 0
#define RG_CSI2_VCCTRL_WORDVC0_MASK (0x1f << 0)

#define SENINF_CSI2_VCCTRL_WORD10 0x0230
#define RG_CSI2_VCCTRL_WORD10_SHIFT 0
#define RG_CSI2_VCCTRL_WORD10_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD11 0x0234
#define RG_CSI2_VCCTRL_WORD11_SHIFT 0
#define RG_CSI2_VCCTRL_WORD11_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD12 0x0238
#define RG_CSI2_VCCTRL_WORD12_SHIFT 0
#define RG_CSI2_VCCTRL_WORD12_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORDDT1 0x023c
#define RG_CSI2_VCCTRL_WORDVC1_SHIFT 0
#define RG_CSI2_VCCTRL_WORDVC1_MASK (0x1f << 0)

#define SENINF_CSI2_VCCTRL_WORD20 0x0240
#define RG_CSI2_VCCTRL_WORD20_SHIFT 0
#define RG_CSI2_VCCTRL_WORD20_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD21 0x0244
#define RG_CSI2_VCCTRL_WORD21_SHIFT 0
#define RG_CSI2_VCCTRL_WORD21_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD22 0x0248
#define RG_CSI2_VCCTRL_WORD22_SHIFT 0
#define RG_CSI2_VCCTRL_WORD22_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORDDT2 0x024c
#define RG_CSI2_VCCTRL_WORDVC2_SHIFT 0
#define RG_CSI2_VCCTRL_WORDVC2_MASK (0x1f << 0)

#define SENINF_CSI2_VCCTRL_WORD30 0x0250
#define RG_CSI2_VCCTRL_WORD30_SHIFT 0
#define RG_CSI2_VCCTRL_WORD30_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD31 0x0254
#define RG_CSI2_VCCTRL_WORD31_SHIFT 0
#define RG_CSI2_VCCTRL_WORD31_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD32 0x0258
#define RG_CSI2_VCCTRL_WORD32_SHIFT 0
#define RG_CSI2_VCCTRL_WORD32_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORDDT3 0x025c
#define RG_CSI2_VCCTRL_WORDVC3_SHIFT 0
#define RG_CSI2_VCCTRL_WORDVC3_MASK (0x1f << 0)

#define SENINF_CSI2_VCCTRL_WORD40 0x0260
#define RG_CSI2_VCCTRL_WORD40_SHIFT 0
#define RG_CSI2_VCCTRL_WORD40_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD41 0x0264
#define RG_CSI2_VCCTRL_WORD41_SHIFT 0
#define RG_CSI2_VCCTRL_WORD41_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD42 0x0268
#define RG_CSI2_VCCTRL_WORD42_SHIFT 0
#define RG_CSI2_VCCTRL_WORD42_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORDDT4 0x026c
#define RG_CSI2_VCCTRL_WORDVC4_SHIFT 0
#define RG_CSI2_VCCTRL_WORDVC4_MASK (0x1f << 0)

#define SENINF_CSI2_VCCTRL_WORD50 0x0270
#define RG_CSI2_VCCTRL_WORD50_SHIFT 0
#define RG_CSI2_VCCTRL_WORD50_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD51 0x0274
#define RG_CSI2_VCCTRL_WORD51_SHIFT 0
#define RG_CSI2_VCCTRL_WORD51_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD52 0x0278
#define RG_CSI2_VCCTRL_WORD52_SHIFT 0
#define RG_CSI2_VCCTRL_WORD52_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORDDT5 0x027c
#define RG_CSI2_VCCTRL_WORDVC5_SHIFT 0
#define RG_CSI2_VCCTRL_WORDVC5_MASK (0x1f << 0)

#define SENINF_CSI2_VCCTRL_WORD60 0x0280
#define RG_CSI2_VCCTRL_WORD60_SHIFT 0
#define RG_CSI2_VCCTRL_WORD60_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD61 0x0284
#define RG_CSI2_VCCTRL_WORD61_SHIFT 0
#define RG_CSI2_VCCTRL_WORD61_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD62 0x0288
#define RG_CSI2_VCCTRL_WORD62_SHIFT 0
#define RG_CSI2_VCCTRL_WORD62_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORDDT6 0x028c
#define RG_CSI2_VCCTRL_WORDVC6_SHIFT 0
#define RG_CSI2_VCCTRL_WORDVC6_MASK (0x1f << 0)

#define SENINF_CSI2_VCCTRL_WORD70 0x0290
#define RG_CSI2_VCCTRL_WORD70_SHIFT 0
#define RG_CSI2_VCCTRL_WORD70_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD71 0x0294
#define RG_CSI2_VCCTRL_WORD71_SHIFT 0
#define RG_CSI2_VCCTRL_WORD71_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD72 0x0298
#define RG_CSI2_VCCTRL_WORD72_SHIFT 0
#define RG_CSI2_VCCTRL_WORD72_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORDDT7 0x029c
#define RG_CSI2_VCCTRL_WORDVC7_SHIFT 0
#define RG_CSI2_VCCTRL_WORDVC7_MASK (0x1f << 0)

#define SENINF_CSI2_VCCTRL_WORD_MASK0 0x02a0
#define RG_CSI2_VCCTRL_WORD_MASK0_SHIFT 0
#define RG_CSI2_VCCTRL_WORD_MASK0_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD_MASK1 0x02a4
#define RG_CSI2_VCCTRL_WORD_MASK1_SHIFT 0
#define RG_CSI2_VCCTRL_WORD_MASK1_MASK (0xffffffff << 0)

#define SENINF_CSI2_VCCTRL_WORD_MASK2 0x02a8
#define RG_CSI2_VCCTRL_WORD_MASK2_SHIFT 0
#define RG_CSI2_VCCTRL_WORD_MASK2_MASK (0xffffffff << 0)

#endif
