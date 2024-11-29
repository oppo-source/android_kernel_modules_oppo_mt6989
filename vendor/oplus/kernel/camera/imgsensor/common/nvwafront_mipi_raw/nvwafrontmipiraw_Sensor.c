// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 nvwafrontmipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include "nvwafrontmipiraw_Sensor.h"

#define NVWAFRONT_EEPROM_READ_ID	0xA8
#define NVWAFRONT_MAX_OFFSET		0x8000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define OTP_SIZE    0x4000
#define LRC_L_REG   0xCE00
#define LRC_R_REG   0xCF00
#define OTP_LRC_VALID_ADDR 0x2024
#define OTP_QSC_VALID_ADDR 0x1F18
#define OTP_QCOM_PDAF_DATA_LENGTH 0x468
#define OTP_QCOM_PDAF_DATA_START_ADDR 0x600

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static kal_uint8 otp_qcom_pdaf_data[OTP_QCOM_PDAF_DATA_LENGTH] = {0};
static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int nvwafront_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwafront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwafront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwafront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwafront_open(struct subdrv_ctx *ctx);
static int nvwafront_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static void set_sensor_lrc(void *arg);
static void get_sensor_cali(void* arg);
static int feedback_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int nvwafront_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, nvwafront_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, nvwafront_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, nvwafront_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, nvwafront_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_AWB_GAIN, feedback_awbgain},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_DATA, nvwafront_get_otp_qcom_pdaf_data},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x0059000B,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA8,

		.qsc_support = TRUE,
		.qsc_size = 1560,
		.addr_qsc = 0x1900,
		.sensor_reg_addr_qsc = 0x1000,

		.lrc_support = TRUE,
		.lrc_size = 260,
		.addr_lrc = 0x1F20,
		.sensor_reg_addr_lrc = 0xC800, // useless
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_pre_cap = {
	.i4OffsetX = 16,
	.i4OffsetY = 16,
	.i4PitchX  = 16,
	.i4PitchY  = 16,
	.i4PairNum  = 16,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosR = {
		{18, 17}, {26, 17}, {16, 19}, {24, 19},
		{20, 21}, {28, 21}, {22, 23}, {30, 23},
		{18, 25}, {26, 25}, {16, 27}, {24, 27},
		{20, 29}, {28, 29}, {22, 31}, {30, 31},
	},
	.i4PosL = {
		{19, 17}, {27, 17}, {17, 19}, {25, 19},
		{21, 21}, {29, 21}, {23, 23}, {31, 23},
		{19, 25}, {27, 25}, {17, 27}, {25, 27},
		{21, 29}, {29, 29}, {23, 31}, {31, 31},
	},
	.i4BlockNumX = 202,
	.i4BlockNumY = 152,
	.iMirrorFlip = 3,
	.i4Crop = {
		{8, 8}, {8, 8}, {8, 304}, {0, 0}, {0, 0},
		{0, 0}, {0, 0}, {8, 8}, {0, 0}, {0, 0},
		{0, 0}, {0, 0},{0, 0}, {0, 0}, {824, 620},
	},
	.i4FullRawW = 3280,
	.i4FullRawH = 2464,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 2,
		.i4PDRepetition = 4,
		.i4PDOrder = { 1 },
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
	.i4OffsetX = 16,
	.i4OffsetY = 16,
	.i4PitchX  = 16,
	.i4PitchY  = 16,
	.i4PairNum  = 16,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosR = {
		{18, 17}, {26, 17}, {16, 19}, {24, 19},
		{20, 21}, {28, 21}, {22, 23}, {30, 23},
		{18, 25}, {26, 25}, {16, 27}, {24, 27},
		{20, 29}, {28, 29}, {22, 31}, {30, 31},
	},
	.i4PosL = {
		{19, 17}, {27, 17}, {17, 19}, {25, 19},
		{21, 21}, {29, 21}, {23, 23}, {31, 23},
		{19, 25}, {27, 25}, {17, 27}, {25, 27},
		{21, 29}, {29, 29}, {23, 31}, {31, 31},
	},
	.i4BlockNumX = 202,
	.i4BlockNumY = 116,
	.iMirrorFlip = 3,
	.i4Crop = {
		{8, 8}, {8, 8}, {8, 304}, {0, 0}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}, {8, 304}, {8, 304},
		{8, 304}
	},
	.i4FullRawW = 3280,
	.i4FullRawH = 2464,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 2,
		.i4PDRepetition = 4,
		.i4PDOrder = { 1 },
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_cus2 = {
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  = 8,
	.i4SubBlkW  = 4,
	.i4SubBlkH  = 2,
	.i4PosR = {
		{8, 9},  {12, 9},  {10, 11}, {14, 11},
		{8, 13}, {12, 13}, {10, 15}, {14, 15},
	},
	.i4PosL = {
		{9, 9},  {13, 9},  {11, 11}, {15, 11},
		{9, 13}, {13, 13}, {11, 15}, {15, 15},
	},
	.i4BlockNumX = 202,
	.i4BlockNumY = 152,
	.iMirrorFlip = 3,
	.i4Crop = {
		{8, 8}, {8, 8}, {8, 304}, {0, 0}, {0, 0},
		{0, 0}, {4, 4}, {0, 0}, {8, 304}, {8, 304},
		{8, 304},
	},
	.i4FullRawW = 1640,
	.i4FullRawH = 1232,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 2,
		.i4PDRepetition = 2,
		.i4PDOrder = { 1 },
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 2448,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 808, // pixel
            .vsize = 1216,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 2448,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 808, // pixel
            .vsize = 1216,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    },
};
//reg C4-3-1
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 808, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 1632,
            .vsize = 1224,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 1632,
            .vsize = 1224,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
#if 1
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 808, // pixel
            .vsize = 608,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
#endif
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 6528,
            .vsize = 4896,
            .user_data_desc = VC_STAGGER_NE,
        },

    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 808, // pixel
            .vsize = 1216,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
#if 1
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 808, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    },
#endif
    {
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_ME,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
#if 1
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 808, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
#if 0
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 808, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
#endif
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = nvwafront_preview_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_preview_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = nvwafront_seamless_custom3,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwafront_seamless_custom3),
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2528,
		.max_framerate = 293,
		.mipi_pixel_rate = 882400000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 16,
			.w0_size = 6560,
			.h0_size = 4896,
			.scale_w = 3280,
			.scale_h = 2448,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_pre_cap,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = nvwafront_capture_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2528,
		.max_framerate = 293,
		.mipi_pixel_rate = 882400000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 16,
			.w0_size = 6560,
			.h0_size = 4896,
			.scale_w = 3280,
			.scale_h = 2448,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_pre_cap,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = nvwafront_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_normal_video_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = nvwafront_seamless_normal_video,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwafront_seamless_normal_video),
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2448,
		.max_framerate = 300,
		.mipi_pixel_rate = 447200000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
	{
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = nvwafront_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 849600000,
		.linelength = 7400,
		.framelength = 1912,
		.max_framerate = 600,
		.mipi_pixel_rate = 416000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 795,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
	{
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = nvwafront_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 849600000,
		.linelength = 7400,
		.framelength = 1912,
		.max_framerate = 600,
		.mipi_pixel_rate = 416000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 795,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = nvwafront_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 240000000,
		.linelength = 5880,
		.framelength = 2720,
		.max_framerate = 150,
		.mipi_pixel_rate = 120000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6560,
			.h0_size = 4928,
			.scale_w = 1640,
			.scale_h = 1232,
			.x1_offset = 4,
			.y1_offset = 4,
			.w1_size = 1632,
			.h1_size = 1224,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1632,
			.h2_tg_size = 1224,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1000,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = nvwafront_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 355200000,
		.linelength = 5880,
		.framelength = 2008,
		.max_framerate = 300,
		.mipi_pixel_rate = 182800000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6560,
			.h0_size = 4928,
			.scale_w = 1640,
			.scale_h = 1232,
			.x1_offset = 4,
			.y1_offset = 4,
			.w1_size = 1632,
			.h1_size = 1224,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1632,
			.h2_tg_size = 1224,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_cus2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1000,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = nvwafront_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_custom3_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = nvwafront_seamless_preview,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwafront_seamless_preview),
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 5056,
		.max_framerate = 243,
		.mipi_pixel_rate = 882400000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 16,
			.w0_size = 6560,
			.h0_size = 4896,
			.scale_w = 6560,
			.scale_h = 4896,
			.x1_offset = 16,
			.y1_offset = 0,
			.w1_size = 6528,
			.h1_size = 4896,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 6528,
			.h2_tg_size = 4896,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_pre_cap,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 826,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x34,
		},
		.ana_gain_max = BASEGAIN * 16,
	},
	{
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = nvwafront_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_custom4_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = nvwafront_seamless_custom4,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwafront_seamless_custom4),
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 7456,
		.framelength = 1928*2,
		.max_framerate = 300,
		.mipi_pixel_rate = 447200000,
		.readout_length = 0*2,
		.read_margin = 10*2,
		.framelength_step = 4*2,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 789,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x82,
		},
	},
	{
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = nvwafront_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_custom5_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2448,
		.max_framerate = 300,
		.mipi_pixel_rate = 447200000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
	{
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = nvwafront_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(nvwafront_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 7456,
		.framelength = 1928 * 2,
		.max_framerate = 300,
		.mipi_pixel_rate = 447200000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 789,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = NVWAFRONT_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {6560, 4928},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 32,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = nvwafront_ana_gain_table,
	.ana_gain_table_size = sizeof(nvwafront_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max =  0xffff - 24,
	.exposure_step = 1,
	.exposure_margin = 48,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 1836500,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,

	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.g_cali = get_sensor_cali,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203}, // Long exposure
			{0x0000, 0x0000}, // Not supported
			{0x0224, 0x0225}, // Short exposure
	},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205}, // Long gain
			{0x0000, 0x0000}, // Not supported
			{0x0216, 0x0217}, // Short gain
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = 0x0138,
	.reg_addr_temp_read = 0x013A,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.reg_addr_fast_mode = 0x3010,

	.init_setting_table = nvwafront_init_setting,
	.init_setting_len = ARRAY_SIZE(nvwafront_init_setting),
	//.aonhemd_setting_table = nvwafront_aonhemd_setting,
	//.aonhemd_setting_len = ARRAY_SIZE(nvwafront_aonhemd_setting),
	//.aonhemd_clear_setting_table = nvwafront_aonhemd_clear_setting,
	//.aonhemd_clear_setting_len   = ARRAY_SIZE(nvwafront_aonhemd_clear_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	//.chk_s_off_before_s_on = 1,
	//.chk_s_off_before_control = 1,
	//.chk_s_off_after_s_off = 0,

	.checksum_value = 0xAF3E324F,
};

static struct subdrv_ops ops = {
	.get_id = nvwafront_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = nvwafront_open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
    {HW_ID_MCLK1, 24, 0},
    {HW_ID_PONV, 0, 0},
    {HW_ID_RST, 0, 1},
    {HW_ID_AVDD, 2908000, 3},
    {HW_ID_AFVDD, 2804000, 3},
    {HW_ID_DVDD, 816000, 4},
    {HW_ID_DOVDD, 1804000, 3},
    {HW_ID_MCLK1_DRIVING_CURRENT, 4, 6},
    {HW_ID_PONV, 1, 1},
    {HW_ID_RST, 1, 4}
};

/*
static struct subdrv_pw_seq_entry aon_pw_seq[] = {
    {HW_ID_MCLK, 24, 0},
    {HW_ID_PONV, 0, 0},
    {HW_ID_RST, 0, 4},
    {HW_ID_AVDD, 2904000, 3},
    {HW_ID_DVDD, 816000, 4},
    {HW_ID_DOVDD, 1804000, 3},
    {HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
    {HW_ID_PONV, 0, 1},
    {HW_ID_RST, 1, 4}
};nvwafront_eeprom_info
*/

const struct subdrv_entry nvwafront_mipi_raw_entry = {
	.name = "nvwafront_mipi_raw",
	.id = NVWAFRONT_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	//.aon_pw_seq = aon_pw_seq,
	//.aon_pw_seq_cnt = ARRAY_SIZE(aon_pw_seq),
	.ops = &ops,
};


 static struct eeprom_addr_table_struct  oplus_eeprom_addr_table =
{
    .i2c_read_id = 0xA9,
	.i2c_write_id = 0xA8,

	.addr_modinfo = 0x0000,
	.addr_lens = 0x0008,
	.addr_vcm = 0x000A,
    .addr_modinfoflag = 0x000F,

	.addr_af = 0x0092,
	.addr_afmacro = 0x0092,
	.addr_afinf = 0x0094,
	.addr_afflag = 0x0098,

	.addr_qrcode = 0x00B0,
	.addr_qrcodeflag = 0x00C1,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	*len = sizeof(oplus_eeprom_info);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, NVWAFRONT_EEPROM_READ_ID >> 1,
		addr, data, size) < 0) {
		return false;
	}
	return true;
}

static int nvwafront_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;

	read_cmos_eeprom_p8(ctx, OTP_QCOM_PDAF_DATA_START_ADDR, otp_qcom_pdaf_data, OTP_QCOM_PDAF_DATA_LENGTH);

	memcpy(feature_return_para_32, (UINT32 *)otp_qcom_pdaf_data, sizeof(otp_qcom_pdaf_data));
	*len = sizeof(otp_qcom_pdaf_data);

	return 0;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "nvwafront read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "nvwafront read_otp_info end\n");
}

static int nvwafront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOGE(ctx, "get otp data");
	if (otp_data_checksum[0] == 0) {
		read_otp_info(ctx);
	} else {
		DRV_LOG(ctx, "otp data has already read read read");
	}
	memcpy(feature_return_para_32, (UINT32 *)otp_data_checksum, sizeof(otp_data_checksum));
	*len = sizeof(otp_data_checksum);
	return 0;
}

static int nvwafront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	nvwafront_get_imgsensor_id(ctx, (u32*)para);
	return 0;
}

static int nvwafront_open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	u32 rmsc_ip_setting_len = ARRAY_SIZE(nvwafront_rmsc_ip_setting);

	/* get sensor id */
	if (nvwafront_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	sensor_init(ctx);

	/*rmsc_ip_setting*/
	DRV_LOG(ctx, "E: rmsc_ip_setting_len size:%u\n", rmsc_ip_setting_len);
	i2c_table_write(ctx, nvwafront_rmsc_ip_setting, rmsc_ip_setting_len);
	DRV_LOG(ctx, "X: rmsc_ip_setting_len size:%u\n", rmsc_ip_setting_len);

	/*QSC setting*/
	if (ctx->s_ctx.s_cali != NULL) {
		ctx->s_ctx.s_cali((void*)ctx);
	} else {
		write_sensor_Cali(ctx);
	}

	/*LRC setting*/
	set_sensor_lrc((void*)ctx);

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->current_fps = 10 * ctx->pclk / ctx->line_length / ctx->frame_length;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;

	return ERROR_NONE;
}

static int nvwafront_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	static bool first_read = KAL_TRUE;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			DRV_LOG(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x0709) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static void set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	pbuf = info[idx].preload_qsc_table;
	size = info[idx].qsc_size;
	addr = info[idx].sensor_reg_addr_qsc;
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			subdrv_i2c_wr_u8(ctx, 0x3207, 0x01); // enable QSC
			DRV_LOG(ctx, "set QSC calibration data done.");
		} else {
			subdrv_i2c_wr_u8(ctx, 0x3207, 0x00); // disable QSC
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature <= 0x60)
		temperature_convert = temperature;
	else if (temperature >= 0x61 && temperature <= 0x7F)
		temperature_convert = 97;
	else if (temperature >= 0x80 && temperature <= 0xE2)
		temperature_convert = -30;
	else
		temperature_convert = (char)temperature | 0xFFFFFF0;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en) {
		set_i2c_buffer(ctx, 0x0104, 0x01);
		if (ctx->s_ctx.reg_addr_fast_mode && ctx->fast_mode_on) {
			ctx->fast_mode_on = FALSE;
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		}
	}
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return (16384 - (16384 * BASEGAIN) / gain);
}

static int nvwafront_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	u32 *ae_ctrl = NULL;
	u32 *feature_data = (u32 *)para;

	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return ERROR_INVALID_SCENARIO_ID;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (u32 *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOGE(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);

	if (ctx->current_scenario_id == scenario_id) {
		DRV_LOGE(ctx, "E: set seamless switch %u %u same,return\n", ctx->current_scenario_id, scenario_id);
		return ERROR_INVALID_SCENARIO_ID;
	}

	if (!ctx->extend_frame_length_en)
		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return ERROR_INVALID_SCENARIO_ID;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return ERROR_INVALID_SCENARIO_ID;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return ERROR_INVALID_SCENARIO_ID;
	}

	ctx->is_seamless = TRUE;
	update_mode_info(ctx, scenario_id);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x3010, 0x02);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		/*
		case HDR_RAW_STAGGER_2EXP:
			set_multi_shutter_frame_length(ctx, (u64 *)ae_ctrl, 2, 0);
			set_multi_gain(ctx, ae_ctrl + 5, 2);
			break;
		case HDR_RAW_STAGGER_3EXP:
			set_multi_shutter_frame_length(ctx, (u64 *)ae_ctrl, 3, 0);
			set_multi_gain(ctx, ae_ctrl + 5, 3);
			break;
		*/
		default:
			set_shutter(ctx, *ae_ctrl);
			set_gain(ctx, *(ae_ctrl + 5));
			break;
		}
	}
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOGE(ctx, "X: set seamless switch done\n");
	return 0;
}

static int nvwafront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	switch (mode) {
	case 5:
		subdrv_i2c_wr_u8(ctx, 0x020E, 0x00); /* dig_gain = 0 */
		break;
	default:
		subdrv_i2c_wr_u8(ctx, 0x0601, mode);
		break;
	}

	if ((ctx->test_pattern) && (mode != ctx->test_pattern)) {
		if (ctx->test_pattern == 5)
			subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);
		else if (mode == 0)
			subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /* No pattern */
	}

	ctx->test_pattern = mode;
	return 0;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, 0x3010, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}

static void set_sensor_lrc(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* LRC data */
	support = info[idx].lrc_support;
	pbuf = info[idx].preload_lrc_table;
	size = info[idx].lrc_size;
	if (support) {
		if (pbuf != NULL && size > 0) {
			subdrv_i2c_wr_seq_p8(ctx, LRC_L_REG, pbuf, size / 2); // L data
			subdrv_i2c_wr_seq_p8(ctx, LRC_R_REG, pbuf + size / 2, size / 2); // R data
			DRV_LOG(ctx, "set LRC calibration data done.");
		} else {
			DRV_LOGE(ctx, "LRC calibration data error");
		}
	}
}

void get_sensor_cali(void* arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *buf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u8 qsc_is_valid = 0, lrc_is_valid = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	/* Probe EEPROM device */
	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	size = info[idx].qsc_size;
	addr = info[idx].addr_qsc;
	buf = info[idx].qsc_table;
	if (support && size > 0) {
		// Check QSC validation
		qsc_is_valid = i2c_read_eeprom(ctx, OTP_QSC_VALID_ADDR);
		if (qsc_is_valid != 0x01) {
			DRV_LOGE(ctx, "QSC data is invalid, flag(%02x)", qsc_is_valid);
		} else if (info[idx].preload_qsc_table == NULL) {
			info[idx].preload_qsc_table = kmalloc(size, GFP_KERNEL);
			if (buf == NULL) {
				if (!read_cmos_eeprom_p8(ctx, addr, info[idx].preload_qsc_table, size)) {
					DRV_LOGE(ctx, "preload QSC data failed");
				}
			} else {
				memcpy(info[idx].preload_qsc_table, buf, size);
			}
			DRV_LOG(ctx, "preload QSC data %u bytes", size);
		} else {
			DRV_LOG(ctx, "QSC data is already preloaded %u bytes", size);
		}
	}

	/* LRC data */
	support = info[idx].lrc_support;
	size = info[idx].lrc_size;
	addr = info[idx].addr_lrc;
	buf = info[idx].lrc_table;
	if (support && size > 0) {
		lrc_is_valid = i2c_read_eeprom(ctx, OTP_LRC_VALID_ADDR);
		if (lrc_is_valid != 0x01) {
			DRV_LOGE(ctx, "LRC data is invalid, flag(%02x)", lrc_is_valid);
		} else if (info[idx].preload_lrc_table == NULL) {
			info[idx].preload_lrc_table = kmalloc(size, GFP_KERNEL);
			if (buf == NULL) {
				if (!read_cmos_eeprom_p8(ctx, addr, info[idx].preload_lrc_table, size)) {
					DRV_LOGE(ctx, "preload LRC data failed");
				}
			} else {
				memcpy(info[idx].preload_lrc_table, buf, size);
			}
			DRV_LOG(ctx, "preload LRC data %u bytes", size);
		} else {
			DRV_LOG(ctx, "LRC data is already preloaded %u bytes", size);
		}
	}

	ctx->is_read_preload_eeprom = 1;
}


/*write AWB gain to sensor*/
static u16 nvwafront_feedback_awbgain[] = {
	0x0b90, 0x00,
	0x0b91, 0x01,
	0x0b92, 0x00,
	0x0b93, 0x01,
	0x0B8E, 0x01,
	0x0B8F, 0x00,
	0x0B94, 0x01,
	0x0B95, 0x00,
};


static int feedback_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	u32 *feature_data_32 = (u32 *) para;
	u32 r_gain = (u32)*(feature_data_32 + 1);
	u32 b_gain = (u32)*(feature_data_32 + 2);
	/* modify to separate 3hdr and remosaic */
	if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3) {
		/*write AWB gain to sensor*/
		u32 r_gain_int = 0;
		u32 b_gain_int = 0;

		DRV_LOG(ctx, "feedback_awbgain r_gain: %d, b_gain: %d\n", r_gain, b_gain);
		r_gain_int = r_gain / 512;
		b_gain_int = b_gain / 512;

		nvwafront_feedback_awbgain[1] = r_gain_int;
		nvwafront_feedback_awbgain[3] = (r_gain - r_gain_int * 512) / 2;
		nvwafront_feedback_awbgain[5] = b_gain_int;
		nvwafront_feedback_awbgain[7] = (b_gain - b_gain_int * 512) / 2;
		subdrv_i2c_wr_regs_u8(ctx, nvwafront_feedback_awbgain,
				sizeof(nvwafront_feedback_awbgain)/sizeof(u16));
	}
	return 0;
}
