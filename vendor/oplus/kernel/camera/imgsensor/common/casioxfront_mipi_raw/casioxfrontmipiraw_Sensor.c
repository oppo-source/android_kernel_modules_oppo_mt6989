// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 casioxfrontmipiraw_Sensor.c
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
#include "casioxfrontmipiraw_Sensor.h"

#define CASIOXFRONT_EEPROM_READ_ID	0xA9
#define CASIOXFRONT_EEPROM_WRITE_ID   0xA8
#define CASIOXFRONT_MAX_OFFSET		0x4000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "casioxfront_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x4000
#define OTP_QCOM_PDAF_DATA_LENGTH 0x468
#define OTP_QCOM_PDAF_DATA_START_ADDR 0x600
#define SXTC_DATA_NUM           784
#define SXTC_DATA_LENS          SXTC_DATA_NUM*sizeof(__u8)
#define PD_XTC_DATA_NUM         588
#define PD_XTC_DATA_LENS        PD_XTC_DATA_NUM*sizeof(__u8)

struct oplus_eeprom_4cell_info_struct
{
	__u8 sxtc_otp_table[SXTC_DATA_NUM];
	__u8 pd_xtc_otp_table[PD_XTC_DATA_NUM];
};

struct eeprom_4cell_addr_table_struct
{
	u16 addr_sxtc;
	u16 addr_pdxtc;
};

static bool bNeedSetNormalMode = FALSE;
static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static kal_uint8 otp_qcom_pdaf_data[OTP_QCOM_PDAF_DATA_LENGTH] = {0};
static int casioxfront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_get_eeprom_4cell_info(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int close(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void set_sensor_cali(void *arg);
static int casioxfront_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casioxfront_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int casioxfront_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);

/* STRUCT */
//自定义
static void gc32e2_set_dummy(struct subdrv_ctx *ctx);
static int gc32e2_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static unsigned int read_casioxfront_eeprom_4cell_info(struct subdrv_ctx *ctx,
	struct oplus_eeprom_4cell_info_struct* oplus_eeprom_4cell_info,
	struct eeprom_4cell_addr_table_struct oplus_eeprom_4cell_addr_table);

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, casioxfront_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, casioxfront_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, casioxfront_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, casioxfront_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, casioxfront_streaming_resume},
	{SENSOR_FEATURE_SET_ESHUTTER, casioxfront_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, casioxfront_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_GAIN, casioxfront_set_gain},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, casioxfront_set_hdr_tri_shutter2},
	{SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, casioxfront_set_hdr_tri_shutter3},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, casioxfront_set_multi_shutter_frame_length_ctrl},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_DATA, casioxfront_get_otp_qcom_pdaf_data},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, gc32e2_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_GET_4CELL_DATA, casioxfront_get_eeprom_4cell_info},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x016E007F,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA8,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_partial_pd_info = {
	.i4OffsetX = 32,
	.i4OffsetY = 14,
	.i4PitchX = 8,
	.i4PitchY = 8,
	.i4PairNum = 2,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{33, 16}, {37, 20}},
	.i4PosR = {{34, 16}, {38, 20}},
	.i4BlockNumX = 400,
	.i4BlockNumY = 302,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 304}, {0, 304}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = 3,
	.i4FullRawW = 3264,
	.i4FullRawH = 2448,
	.i4ModeIndex = 0,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1}, /*R = 1, L = 0*/
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_partial_pd_info_16_9 = {
	.i4OffsetX = 32,
	.i4OffsetY = 14,
	.i4PitchX = 8,
	.i4PitchY = 8,
	.i4PairNum = 2,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{33, 16}, {37, 20}},
	.i4PosR = {{34, 16}, {38, 20}},
	.i4BlockNumX = 400,
	.i4BlockNumY = 228,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 304}, {0, 304}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = 3,
	.i4FullRawW = 3264,
	.i4FullRawH = 2448,
	.i4ModeIndex = 0,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1}, /*R = 1, L = 0*/
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
    // partial pd
	{
	    .bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 400,
			.vsize = 1208,
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
    // partial pd
	{
	    .bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 400,
			.vsize = 1208,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1836,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    // partial pd
	{
	    .bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 400,
			.vsize = 918,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1836,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    // partial pd
	{
	    .bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 400,
			.vsize = 918,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1836,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
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
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1632,
			.vsize = 918,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
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
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = casioxfront_preview_setting,
		.mode_setting_len = ARRAY_SIZE(casioxfront_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 370000000,
		.linelength = 4756,
		.framelength = 2584,
		.max_framerate = 300,
		.mipi_pixel_rate = 331200000, //828*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_partial_pd_info,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 69,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = casioxfront_capture_setting,
		.mode_setting_len = ARRAY_SIZE(casioxfront_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 370000000,
		.linelength = 4756,
		.framelength = 2584,
		.max_framerate = 300,
		.mipi_pixel_rate = 331200000, //828*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_partial_pd_info,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 69,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = casioxfront_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(casioxfront_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 370000000,
		.linelength = 4756,
		.framelength = 2584,
		.max_framerate = 300,
		.mipi_pixel_rate = 331200000, //828*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 612,
			.w0_size = 6528,
			.h0_size = 3672,
			.scale_w = 3264,
			.scale_h = 1836,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_partial_pd_info_16_9,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 71,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		}
	},
	{
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = casioxfront_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(casioxfront_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 370000000,
		.linelength = 2352,
		.framelength = 2616,
		.max_framerate = 600,
		.mipi_pixel_rate = 640000000, //1600*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 612,
			.w0_size = 6528,
			.h0_size = 3672,
			.scale_w = 3264,
			.scale_h = 1836,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_partial_pd_info_16_9,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 66,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 60,
		}
	},
	{
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = casioxfront_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(casioxfront_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 370000000,
		.linelength = 2352,
		.framelength = 2616,
		.max_framerate = 600,
		.mipi_pixel_rate = 640000000, //828*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 612,
			.w0_size = 6528,
			.h0_size = 3672,
			.scale_w = 3264,
			.scale_h = 1836,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 77,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = casioxfront_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(casioxfront_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 370000000,
		.linelength = 2270,
		.framelength = 5396,
		.max_framerate = 300,
		.mipi_pixel_rate = 300800000, //752*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 1632,
			.scale_h = 1224,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1632,
			.h1_size = 1224,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1632,
			.h2_tg_size = 1224,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 77,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = casioxfront_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(casioxfront_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 370000000,
		.linelength = 2270,
		.framelength = 5396,
		.max_framerate = 300,
		.mipi_pixel_rate = 300800000, //752*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 612,
			.w0_size = 6528,
			.h0_size = 3672,
			.scale_w = 1632,
			.scale_h = 918,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1632,
			.h1_size = 918,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1632,
			.h2_tg_size = 918,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 77,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = casioxfront_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(casioxfront_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 740000000,
		.linelength = 9568,
		.framelength = 5104,
		.max_framerate = 150,
		.mipi_pixel_rate = 601600000, //1504*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 6528,
			.scale_h = 4896,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 6528,
			.h1_size = 4896,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 6528,
			.h2_tg_size = 4896,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 65,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = RMSC_MASK,
			.equivalent_fps = 15,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = CASIOXFRONT_SENSOR_ID,
	.reg_addr_sensor_id = {0x03f0, 0x03f1},
	.i2c_addr_table = {0x20, 0xff},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {6528, 4896},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 4, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 1,
	.ana_gain_table = casioxfront_ana_gain_table,
	.ana_gain_table_size = sizeof(casioxfront_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0xFFFF - 64,//0x3FFE - 64
	.exposure_step = 4,
	.exposure_margin = 64,

	.frame_length_max = 0xFFFF,//0x3FFE
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 1000000,

	.pdaf_type = PDAF_SUPPORT_CAMSV, //PDAF_SUPPORT_CAMSV,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = PARAM_UNDEFINED,
	.g_cali = PARAM_UNDEFINED,
	.s_gph = PARAM_UNDEFINED,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,

	.init_setting_table = casioxfront_sensor_init_setting,
	.init_setting_len =  ARRAY_SIZE(casioxfront_sensor_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
	.checksum_value = 0x3a98f032,
};


static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_PONV, 0, 1},
	{HW_ID_RST, 0, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 3},
	{HW_ID_AFVDD, 2825000, 3},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD, 1100000, 1},
	{HW_ID_AVDD, 2825000, 1},
	{HW_ID_PONV, 1, 1},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry casioxfront_mipi_raw_entry = {
	.name = "casioxfront_mipi_raw",
	.id = CASIOXFRONT_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */


static void casioxfront_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	/* return; //for test */
	subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
}				/*      set_dummy  */

static void casioxfront_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
			kal_bool min_framelength_en)
{

	kal_uint32 frame_length = ctx->frame_length;

	DRV_LOG(ctx, "framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;

	if (frame_length >= ctx->min_frame_length)
		ctx->frame_length = frame_length;
	else
		ctx->frame_length = ctx->min_frame_length;

	ctx->dummy_line =
		ctx->frame_length - ctx->min_frame_length;

	if (ctx->frame_length > ctx->max_frame_length) {
		ctx->frame_length = ctx->max_frame_length;

		ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;

	casioxfront_set_dummy(ctx);
}


static int casioxfront_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = *(feature_data + 1);
	u32 fine_integ_line = 0;
	DRV_LOG(ctx, "gc32e2_shutter = 0x%x ,frame_length = 0x%x\n", shutter,frame_length);
	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of framelength */
	ctx->frame_length =	max(shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max(shutter, ctx->s_ctx.exposure_min);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;
	shutter = min(shutter, ctx->s_ctx.exposure_max);
	/* write framelength&shutter */
		if (set_auto_flicker(ctx, 0) || ctx->frame_length) {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],
			(ctx->frame_length >> 8) & 0xFF);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1],
			ctx->frame_length & 0xFF);
			}
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],
		(ctx->exposure[0] >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],
		ctx->exposure[0] & 0xFF);

	DRV_LOG(ctx, "gc32e2 exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);

	return 0;
}

static void casioxfront_write_shutter(struct subdrv_ctx *ctx)
{
	kal_uint16 realtime_fps = 0;
	DRV_LOG(ctx, "===brad shutter:%d\n", ctx->exposure[0]);

	if (ctx->exposure[0] > ctx->min_frame_length - ctx->s_ctx.exposure_margin) {
		ctx->frame_length = ctx->exposure[0] + ctx->s_ctx.exposure_margin;
	} else {
		ctx->frame_length = ctx->min_frame_length;
	}
	if (ctx->frame_length > ctx->max_frame_length) {
		ctx->frame_length = ctx->max_frame_length;
	}

	if (ctx->exposure[0] < ctx->s_ctx.exposure_min) {
		ctx->exposure[0] = ctx->s_ctx.exposure_min;
	}

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps > 592 && realtime_fps <= 607) {
			casioxfront_set_max_framerate(ctx, 592, 0);
		} else if (realtime_fps > 296 && realtime_fps <= 305) {
			casioxfront_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 246 && realtime_fps <= 253) {
			casioxfront_set_max_framerate(ctx, 246, 0);
		} else if (realtime_fps > 236 && realtime_fps <= 243) {
			casioxfront_set_max_framerate(ctx, 236, 0);
		} else if (realtime_fps > 146 && realtime_fps <= 153) {
			casioxfront_set_max_framerate(ctx, 146, 0);
		} else {
			// Extend frame length
			subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
		}
	} else {
		// Extend frame length
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
		subdrv_i2c_wr_u16(ctx, 0x0202, ctx->exposure[0]);
	}
	DRV_LOG(ctx, "shutter =%d, framelength =%d\n", ctx->exposure[0], ctx->frame_length);
}	/*	write_shutter  */

static void casioxfront_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	casioxfront_write_shutter(ctx);
}

static int casioxfront_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	casioxfront_set_shutter_frame_length(ctx, para, len);
	return 0;
}

static void streaming_ctrl(struct subdrv_ctx *ctx, bool enable)
{
	check_current_scenario_id_bound(ctx);
	if (ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode) {
		DRV_LOG(ctx, "AOV mode set stream in SCP side! (sid:%u)\n",
			ctx->current_scenario_id);
		return;
	}

	if (enable) {
		if (ctx->s_ctx.chk_s_off_sta) {
			DRV_LOG(ctx, "check_stream_off before stream on");
			check_stream_off(ctx);
		}
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
	} else {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x80);
		if (ctx->s_ctx.reg_addr_fast_mode && ctx->fast_mode_on) {
			ctx->fast_mode_on = FALSE;
			ctx->ref_sof_cnt = 0;
			DRV_LOG(ctx, "seamless_switch disabled.");
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
			commit_i2c_buffer(ctx);
		}
	}
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static int casioxfront_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			casioxfront_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int casioxfront_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
		return 0;
}

static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
	.i2c_read_id = 0xA9,
	.i2c_write_id = 0xA8,

	.addr_modinfo = 0x0000,
	.addr_sensorid = 0x0006,
	.addr_lens = 0x0008,
	.addr_vcm = 0x000A,
	.addr_modinfoflag = 0x0010,

	.addr_af = 0x0092,
	.addr_afmacro = 0x0092,
	.addr_afinf = 0x0094,
	.addr_afflag = 0x0098,

	.addr_qrcode = 0x00B0,
	.addr_qrcodeflag = 0x00C7,
};

static struct eeprom_4cell_addr_table_struct oplus_eeprom_4cell_addr_table = {
	.addr_sxtc = 0x2300,
	.addr_pdxtc = 0x2612,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};
static struct oplus_eeprom_4cell_info_struct  oplus_eeprom_4cell_info = {0};

static unsigned int read_casioxfront_eeprom_4cell_info(struct subdrv_ctx *ctx,
	struct oplus_eeprom_4cell_info_struct* oplus_eeprom_4cell_info,
	struct eeprom_4cell_addr_table_struct oplus_eeprom_4cell_addr_table)
{
	kal_uint16 addr = oplus_eeprom_4cell_addr_table.addr_sxtc;
	BYTE *data = oplus_eeprom_4cell_info->sxtc_otp_table;
	if (!read_cmos_eeprom_p8(ctx, addr, data, SXTC_DATA_LENS)) {
		DRV_LOGE(ctx, "read SXTC_DATA fail!");
	}

	addr = oplus_eeprom_4cell_addr_table.addr_pdxtc;
	data = oplus_eeprom_4cell_info->pd_xtc_otp_table;
	if (!read_cmos_eeprom_p8(ctx, addr, data, PD_XTC_DATA_LENS)) {
		DRV_LOGE(ctx, "read PD_XTC_DATA fail!");
	}

	return 0;
}

static int casioxfront_get_eeprom_4cell_info(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 * feature_data = (u64 *) para;
	u8 * data = (char *)(uintptr_t)(*(feature_data + 1));
	u16 type = (u16)(*feature_data);
	if (type  == FOUR_CELL_CAL_TYPE_XTALK_CAL){
		*len = sizeof(oplus_eeprom_4cell_info);
		data[0] = *len & 0xFF;
		data[1] = (*len >> 8) & 0xFF;
		memcpy(data + 2, (u8*)(&oplus_eeprom_4cell_info), sizeof(oplus_eeprom_4cell_info));
	}

	return 0;
}

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	*len = sizeof(oplus_eeprom_info);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, CASIOXFRONT_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static int casioxfront_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;

	read_cmos_eeprom_p8(ctx, OTP_QCOM_PDAF_DATA_START_ADDR, otp_qcom_pdaf_data, OTP_QCOM_PDAF_DATA_LENGTH);

	memcpy(feature_return_para_32, (UINT32 *)otp_qcom_pdaf_data, sizeof(otp_qcom_pdaf_data));
	*len = sizeof(otp_qcom_pdaf_data);

	return 0;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "jn1 read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "jn1 read_otp_info end\n");
}

static int casioxfront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOGE(ctx, "get otp data");
	if (otp_data_checksum[0] == 0) {
		read_otp_info(ctx);
	} else {
		DRV_LOG(ctx, "otp data has already read");
	}
	memcpy(feature_return_para_32, (UINT32 *)otp_data_checksum, sizeof(otp_data_checksum));
	*len = sizeof(otp_data_checksum);
	return 0;
}

static int casioxfront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	static bool first_read = TRUE;
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
			DRV_LOG(ctx, "i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x32E2) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_casioxfront_eeprom_4cell_info(ctx, &oplus_eeprom_4cell_info, oplus_eeprom_4cell_addr_table);
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					first_read = FALSE;
				}
				return ERROR_NONE;
			}
			DRV_LOG(ctx, "Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
			DRV_LOG(ctx, "sensor_id = 0x%x, ctx->s_ctx.sensor_id = 0x%x\n",
				*sensor_id, ctx->s_ctx.sensor_id);
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

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	i2c_table_write(ctx, casioxfront_sensor_preinit_setting, ARRAY_SIZE(casioxfront_sensor_preinit_setting));
	mdelay(5);
	sensor_init(ctx);

	/* HW GGC*/
	set_sensor_cali(ctx);

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

static int close(struct subdrv_ctx *ctx)
{
	streaming_ctrl(ctx, false);
	DRV_LOG(ctx, "subdrv closed\n");
	return ERROR_NONE;
}

static int casioxfront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool enable = mode;
	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "gc32e2 test_pattern mode(%u->%u)\n", ctx->test_pattern, mode);
	if (enable) {
			subdrv_i2c_wr_u8(ctx, 0x0089, 0x04);
			subdrv_i2c_wr_u8(ctx, 0x035d, 0x01); /* 100% Color bar */
			subdrv_i2c_wr_u8(ctx, 0x035e, 0x00);
		}
	else {
			subdrv_i2c_wr_u8(ctx, 0x035d, 0x00); /* No pattern */
			subdrv_i2c_wr_u8(ctx, 0x0089, 0x00);
		}
	ctx->test_pattern = enable;
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


static void set_sensor_cali(void *arg)
{
	//struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	return;
}

static int casioxfront_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *)para);
	u32 rg_gain;
	DRV_LOG(ctx, "gc32e2 platform_gain = 0x%x basegain = %d\n", gain, BASEGAIN);
	/* check boundary of gain */
	gain = max(gain, ctx->s_ctx.ana_gain_min);
	gain = min(gain, ctx->s_ctx.ana_gain_max);
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	/* write gain */
	rg_gain = gain * (BASEGAIN / 128);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[0],
		(rg_gain >> 16) & 0x07);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[1],
		(rg_gain >> 8)  & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0206, rg_gain & 0xFF);
	DRV_LOG(ctx, "gc32e2 rg_gain = 0x%x \n", rg_gain);
	return 0;
}

static void casioxfront_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
		u32 *shutters, u16 exp_cnt,	u16 frame_length)
{
	int i = 0;
	u32 fine_integ_line = 0;
	u16 last_exp_cnt = 1;
	u32 calc_fl[3] = {0};
	int readout_diff = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32 rg_shutters[3] = {0};
	u32 cit_step = 0;

	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	if (exp_cnt > ARRAY_SIZE(ctx->exposure)) {
		DRV_LOGE(ctx, "invalid exp_cnt:%u>%lu\n", exp_cnt, ARRAY_SIZE(ctx->exposure));
		exp_cnt = ARRAY_SIZE(ctx->exposure);
	}
	check_current_scenario_id_bound(ctx);

	/* check boundary of shutter */
	for (i = 1; i < ARRAY_SIZE(ctx->exposure); i++)
		last_exp_cnt += ctx->exposure[i] ? 1 : 0;
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	cit_step = ctx->s_ctx.mode[ctx->current_scenario_id].coarse_integ_step;
	for (i = 0; i < exp_cnt; i++) {
		shutters[i] = FINE_INTEG_CONVERT(shutters[i], fine_integ_line);
		shutters[i] = max(shutters[i], ctx->s_ctx.exposure_min);
		shutters[i] = min(shutters[i], ctx->s_ctx.exposure_max);
		if (cit_step)
			shutters[i] = round_up(shutters[i], cit_step);
	}

	/* check boundary of framelength */
	/* - (1) previous se + previous me + current le */
	calc_fl[0] = shutters[0];
	for (i = 1; i < last_exp_cnt; i++)
		calc_fl[0] += ctx->exposure[i];
	calc_fl[0] += ctx->s_ctx.exposure_margin*exp_cnt*exp_cnt;

	/* - (2) current se + current me + current le */
	calc_fl[1] = shutters[0];
	for (i = 1; i < exp_cnt; i++)
		calc_fl[1] += shutters[i];
	calc_fl[1] += ctx->s_ctx.exposure_margin*exp_cnt*exp_cnt;

	/* - (3) readout time cannot be overlapped */
	calc_fl[2] =
		(ctx->s_ctx.mode[ctx->current_scenario_id].readout_length +
		ctx->s_ctx.mode[ctx->current_scenario_id].read_margin);
	if (last_exp_cnt == exp_cnt)
		for (i = 1; i < exp_cnt; i++) {
			readout_diff = ctx->exposure[i] - shutters[i];
			calc_fl[2] += readout_diff > 0 ? readout_diff : 0;
		}
	for (i = 0; i < ARRAY_SIZE(calc_fl); i++)
		ctx->frame_length = max(ctx->frame_length, calc_fl[i]);
	ctx->frame_length =	max(ctx->frame_length, ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	for (i = 0; i < exp_cnt; i++)
		ctx->exposure[i] = shutters[i];
	/* exit long exposure if necessary */
	if ((ctx->exposure[0] < 0xFFF0) && bNeedSetNormalMode) {
		DRV_LOG(ctx, "exit long shutter\n");
		//subdrv_i2c_wr_u16(ctx, 0x0702, 0x0000);
		//subdrv_i2c_wr_u16(ctx, 0x0704, 0x0000);
		bNeedSetNormalMode = FALSE;
	}
	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		write_frame_length(ctx, ctx->frame_length);
	/* write shutter */
	switch (exp_cnt) {
	case 1:
		rg_shutters[0] = shutters[0] / exp_cnt;
		break;
	case 2:
		rg_shutters[0] = shutters[0] / exp_cnt;
		rg_shutters[2] = shutters[1] / exp_cnt;
		break;
	case 3:
		rg_shutters[0] = shutters[0] / exp_cnt;
		rg_shutters[1] = shutters[1] / exp_cnt;
		rg_shutters[2] = shutters[2] / exp_cnt;
		break;
	default:
		break;
	}
	if (ctx->s_ctx.reg_addr_exposure_lshift != PARAM_UNDEFINED)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift, 0);
	for (i = 0; i < 3; i++) {
		if (rg_shutters[i]) {
			if (ctx->s_ctx.reg_addr_exposure[i].addr[2]) {
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 16) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					(rg_shutters[i] >> 8) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[2],
					rg_shutters[i] & 0xFF);
			} else {
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 8) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					rg_shutters[i] & 0xFF);
			}
		}
	}
	DRV_LOG(ctx, "exp[0x%x/0x%x/0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		rg_shutters[0], rg_shutters[1], rg_shutters[2],
		frame_length, ctx->frame_length, ctx->autoflicker_en);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */
}

static int casioxfront_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	casioxfront_set_multi_shutter_frame_length(ctx, (u32 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

static void casioxfront_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
	int i = 0;
	u32 values[3] = {0};

	if (shutters != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u32) *(shutters + i);
	}
	casioxfront_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

static int casioxfront_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	casioxfront_set_hdr_tri_shutter(ctx, feature_data, 2);
	return 0;
}

static int casioxfront_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	casioxfront_set_hdr_tri_shutter(ctx, feature_data, 3);
	return 0;
}

static void gc32e2_set_dummy(struct subdrv_ctx *ctx)
{
	write_frame_length(ctx, ctx->frame_length);
	commit_i2c_buffer(ctx);
}

static int gc32e2_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)*feature_data;
	u32 framerate = *(feature_data + 1);
	u32 frame_length;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	if (framerate == 0) {
		DRV_LOG(ctx, "framerate should not be 0\n");
		return 0;
	}

	if (ctx->s_ctx.mode[scenario_id].linelength == 0) {
		DRV_LOG(ctx, "linelength should not be 0\n");
		return 0;
	}

	if (ctx->line_length == 0) {
		DRV_LOG(ctx, "ctx->line_length should not be 0\n");
		return 0;
	}

	if (ctx->frame_length == 0) {
		DRV_LOG(ctx, "ctx->frame_length should not be 0\n");
		return 0;
	}

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	DRV_LOG(ctx, "gc32e2 max_fps(input/output):%u/%u(sid:%u), min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id);
	if (ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin))
		gc32e2_set_dummy(ctx);

	return 0;
}
