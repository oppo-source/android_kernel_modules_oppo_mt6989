// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 OPLUS. All rights reserved.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 omegas2widemipiraw_Sensor.c
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
#include "omegas2widemipiraw_Sensor.h"

#define OMEGAS2WIDE_EEPROM_READ_ID	0xA3
#define OMEGAS2WIDE_EEPROM_WRITE_ID	0xA2
#define OMEGAS2WIDE_MAX_OFFSET		0x4000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "omegas2wide_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE	0x4000
#define OTP_QSC_VALID_ADDR 0x2A30
#define SENSOR_ID	0x355

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int omegas2wide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas2wide_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas2wide_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas2wide_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas2wide_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas2wide_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size);
/* STRUCT */

static struct eeprom_map_info omegas2wide_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x000f, 0x0010, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x000f, 0x0010, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x000f, 0x0010, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x000f, 0x0010, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x000f, 0x0010, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C1, 0x00C2, 17, true },
	{ EEPROM_META_AF_CODE, 0x0000, 0x0000, 0x0000, 6, false },
	{ EEPROM_META_AF_FLAG, 0x0000, 0x0000, 0x0000, 1, false },
	{ EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x2E00, 0x0000, 0x0000, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0, 0, 0, 0, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA_105CM, 0, 0, 0, 0, false },
	{ EEPROM_META_DISTORTION_DATA, 0, 0, 0, 0, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, omegas2wide_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, omegas2wide_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, omegas2wide_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, omegas2wide_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, omegas2wide_get_otp_checksum_data},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, omegas2wide_get_min_shutter_by_scenario_adapter},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x00650006,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA2,
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
	}
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
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1840,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1408,
			.vsize = 792,
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
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2304,
			.vsize = 1728,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1640,
			.vsize = 1232,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1840,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1640,
			.vsize = 1232,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 3264,
			.vsize = 1840,
			.user_data_desc = VC_RAW_PROCESSED_DATA,
		},
	},
	{/*Virtual Channel for 3a_MetaData*/
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x12,
			.hsize = 1024,
			.vsize = 564,
			.user_data_desc = VC_GENERAL_EMBEDDED,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
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

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = omegas2wide_preview_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_preview_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280800000,
		.linelength = 3672,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 280800000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.ae_binning_ratio = 1,
		.delay_frame = 4,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},

	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = omegas2wide_capture_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_capture_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280800000,
		.linelength = 3672,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 280800000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.ae_binning_ratio = 1,
		.delay_frame = 4,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = omegas2wide_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_normal_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280800000,
		.linelength = 3672,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 280800000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 0,
			.y0_offset = 312,
			.w0_size = 3280,
			.h0_size = 1840,
			.scale_w = 3280,
			.scale_h = 1840,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1840,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1840,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},

	{
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = omegas2wide_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_hs_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 288000000,
		.linelength = 1836,
		.framelength = 870,
		.max_framerate = 1800,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3280,
			.h0_size = 2464,
			.scale_w = 1640,
			.scale_h = 1232,
			.x1_offset = 116,
			.y1_offset = 220,
			.w1_size = 1408,
			.h1_size = 792,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1408,
			.h2_tg_size = 792,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 180,
		},
	},

	{
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = omegas2wide_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_slim_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280800000,
		.linelength = 3672,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 280800000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = omegas2wide_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_custom1_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 273600000,
		.linelength = 3672,
		.framelength = 3104,
		.max_framerate = 240,
		.mipi_pixel_rate = 273600000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 488,
			.y0_offset = 368,
			.w0_size = 2304,
			.h0_size = 1728,
			.scale_w = 2304,
			.scale_h = 1728,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2304,
			.h1_size = 1728,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1728,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},

	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = omegas2wide_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_custom2_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 144000000,
		.linelength = 1836,
		.framelength = 2614,
		.max_framerate = 300,
		.mipi_pixel_rate = 144000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3280,
			.h0_size = 2464,
			.scale_w = 1640,
			.scale_h = 1232,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1640,
			.h1_size = 1232,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1640,
			.h2_tg_size = 1232,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = omegas2wide_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_custom3_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280800000,
		.linelength = 3672,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 280800000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = omegas2wide_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_custom4_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 211200000,
		.linelength = 3672,
		.framelength = 1916,
		.max_framerate = 300,
		.mipi_pixel_rate = 211200000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 312,
			.w0_size = 3264,
			.h0_size = 1840,
			.scale_w = 3264,
			.scale_h = 1840,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1840,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1840,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = omegas2wide_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_custom5_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 144000000,
		.linelength = 1836,
		.framelength = 2614,
		.max_framerate = 300,
		.mipi_pixel_rate = 144000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3280,
			.h0_size = 2464,
			.scale_w = 1640,
			.scale_h = 1232,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1640,
			.h1_size = 1232,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1640,
			.h2_tg_size = 1232,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{/*Reg H-3_QBIN-V2H2 FHD 2048x1152_480FPS */
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = omegas2wide_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_custom6_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 288000000,
		.linelength = 3672,
		.framelength = 2614,
		.max_framerate = 300,
		.mipi_pixel_rate = 322285715,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 0,
			.y0_offset = 312,
			.w0_size = 3280,
			.h0_size = 1840,
			.scale_w = 3280,
			.scale_h = 1840,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1840,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1840,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{ /*reg_K 1632*1224@24*/
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = omegas2wide_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(omegas2wide_custom7_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 144000000,
		.linelength = 1836,
		.framelength = 3266,
		.max_framerate = 240,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OMEGAS2WIDE_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x34, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {3264, 2448},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = omegas2wide_ana_gain_table,
	.ana_gain_table_size = sizeof(omegas2wide_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 1,
	.exposure_max = (0xffff * 128) - 18,
	.exposure_step = 1,
	.exposure_margin = 10,

	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 934000,
	.pdaf_type = PDAF_SUPPORT_NA,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203},
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3060,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = 0x0005,
	.reg_addr_fast_mode = 0x3010,

	.init_setting_table = omegas2wide_init_setting,
	.init_setting_len = ARRAY_SIZE(omegas2wide_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0xD1EFF68B,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
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
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2704000, 0},
	{HW_ID_DVDD, 1184000, 0},
	{HW_ID_DOVDD, 1800000, 3},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry omegas2wide_mipi_raw_entry = {
	.name = "omegas2wide_mipi_raw",
	.id = OMEGAS2WIDE_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static unsigned int read_omegas2wide_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != omegas2wide_eeprom_info[meta_id].meta)
		return -1;

	if (size != omegas2wide_eeprom_info[meta_id].size)
		return -1;

	addr = omegas2wide_eeprom_info[meta_id].start;
	readsize = omegas2wide_eeprom_info[meta_id].size;

	if(!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
	.i2c_read_id = 0xA3,
	.i2c_write_id = 0xA2,

	.addr_modinfo = 0x0000,
	.addr_sensorid = 0x0006,
	.addr_lens = 0x0008,
	.addr_vcm = 0x000A,
	.addr_modinfoflag = 0x000f,

	.addr_af = 0,
	.addr_afmacro = 0,
	.addr_afinf = 0,
	.addr_afflag = 0,

	.addr_qrcode = 0x00B0,
	.addr_qrcodeflag = 0x00C1,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct oplus_eeprom_info_struct* infoPtr;
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	infoPtr = (struct oplus_eeprom_info_struct*)(para);
	*len = sizeof(oplus_eeprom_info);

	return 0;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, OMEGAS2WIDE_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
		kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
	ret = adaptor_i2c_wr_p8(ctx->i2c_client, OMEGAS2WIDE_EEPROM_WRITE_ID >> 1,
			addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
	kal_int32 ret = ERROR_NONE;
	kal_uint16 reg = 0xE000;
	if (enable) {
		adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAS2WIDE_EEPROM_READ_ID >> 1, reg, 0xA3);
	}
	else {
		adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAS2WIDE_EEPROM_READ_ID >> 1, reg, 0xA2);
	}

	return ret;
}

static kal_uint16 get_64align_addr(kal_uint16 data_base) {
	kal_uint16 multiple = 0;
	kal_uint16 surplus = 0;
	kal_uint16 addr_64align = 0;

	multiple = data_base / 64;
	surplus = data_base % 64;
	if(surplus) {
		addr_64align = (multiple + 1) * 64;
	} else {
		addr_64align = multiple * 64;
	}
	return addr_64align;
}

static kal_int32 eeprom_table_write(struct subdrv_ctx *ctx, kal_uint16 data_base, kal_uint8 *pData, kal_uint16 data_length) {
	kal_uint16 idx;
	kal_uint16 idy;
	kal_int32 ret = ERROR_NONE;
	UINT32 i = 0;

	idx = data_length/WRITE_DATA_MAX_LENGTH;
	idy = data_length%WRITE_DATA_MAX_LENGTH;

	LOG_INF("[test] data_base(0x%x) data_length(%d) idx(%d) idy(%d)\n", data_base, data_length, idx, idy);

	for (i = 0; i < idx; i++) {
		ret = table_write_eeprom_30Bytes(ctx, (data_base + WRITE_DATA_MAX_LENGTH * i),
				&pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
		if (ret != ERROR_NONE) {
			LOG_INF("write_eeprom error: i=%d\n", i);
			return -1;
		}
		msleep(6);
	}

	msleep(6);
	if(idy) {
		ret = table_write_eeprom_30Bytes(ctx, (data_base + WRITE_DATA_MAX_LENGTH*idx),
				&pData[WRITE_DATA_MAX_LENGTH*idx], idy);
		if (ret != ERROR_NONE) {
			LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
			return -1;
		}
	}
	return 0;
}

static kal_int32 eeprom_64align_write(struct subdrv_ctx *ctx, kal_uint16 data_base, kal_uint8 *pData, kal_uint16 data_length) {
	kal_uint16 addr_64align = 0;
	kal_uint16 part1_length = 0;
	kal_uint16 part2_length = 0;
	kal_int32 ret = ERROR_NONE;

	addr_64align = get_64align_addr(data_base);

	part1_length = addr_64align - data_base;
	if(part1_length > data_length) {
		part1_length = data_length;
	}
	part2_length = data_length - part1_length;

	write_eeprom_protect(ctx, 0);
	msleep(6);

	if (part1_length) {
		ret = eeprom_table_write(ctx, data_base, pData, part1_length);
		if (ret == -1) {
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			LOG_INF("write_eeprom error part1\n");
			msleep(6);
			return -1;
		}
	}

	msleep(6);
	if (part2_length) {
		ret = eeprom_table_write(ctx, addr_64align, pData + part1_length, part2_length);
		if (ret == -1) {
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			LOG_INF("write_eeprom error part2\n");
			msleep(6);
			return -1;
		}
	}
	msleep(6);
	write_eeprom_protect(ctx, 1);
	msleep(6);

	return 0;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
			ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
	kal_int32  ret = ERROR_NONE;
	kal_uint16 data_base, data_length;
	kal_uint8 *pData;

	if(pStereodata != NULL) {
		LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
					   pStereodata->uSensorId,
					   pStereodata->uDeviceId,
					   pStereodata->baseAddr,
					   pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == OMEGAS2WIDE_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
			&& (data_base == OMEGAS2WIDE_STEREO_START_ADDR)) {
			LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);

			eeprom_64align_write(ctx, data_base, pData, data_length);

			LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
			LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
			LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
			LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
			LOG_INF("write_Module_data Write end\n");

		} else if ((pStereodata->uSensorId == OMEGAS2WIDE_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
			&& (data_base == OMEGAS2WIDE_AESYNC_START_ADDR)) {
			LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
				pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);

			eeprom_64align_write(ctx, data_base, pData, data_length);

			LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
					read_cmos_eeprom_8(ctx, OMEGAS2WIDE_AESYNC_START_ADDR),
					read_cmos_eeprom_8(ctx, OMEGAS2WIDE_AESYNC_START_ADDR+1),
					read_cmos_eeprom_8(ctx, OMEGAS2WIDE_AESYNC_START_ADDR+2),
					read_cmos_eeprom_8(ctx, OMEGAS2WIDE_AESYNC_START_ADDR+3),
					read_cmos_eeprom_8(ctx, OMEGAS2WIDE_AESYNC_START_ADDR+4),
					read_cmos_eeprom_8(ctx, OMEGAS2WIDE_AESYNC_START_ADDR+5),
					read_cmos_eeprom_8(ctx, OMEGAS2WIDE_AESYNC_START_ADDR+6),
					read_cmos_eeprom_8(ctx, OMEGAS2WIDE_AESYNC_START_ADDR+7));
			LOG_INF("AESync write_Module_data Write end\n");
		} else {
			LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
			return -1;
		}
	} else {
		LOG_INF("omegas2 write_Module_data pStereodata is null\n");
		return -1;
	}
	return ret;
}

static int omegas2wide_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	int ret = ERROR_NONE;
	ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
	if (ret != ERROR_NONE) {
		*len = (u32)-1; /*write eeprom failed*/
		LOG_INF("ret=%d\n", ret);
	}
	return 0;
}

static int omegas2wide_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	UINT16 *feature_data_16 = (UINT16 *) para;
	UINT32 *feature_return_para_32 = (UINT32 *) para;
	if(*len > CALI_DATA_SLAVE_LENGTH)
		*len = CALI_DATA_SLAVE_LENGTH;
	LOG_INF("feature_data mode:%d  lens:%d", *feature_data_16, *len);
	read_omegas2wide_eeprom_info(ctx, EEPROM_META_STEREO_MW_MAIN_DATA,
			(BYTE *)feature_return_para_32, *len);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
					BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, OMEGAS2WIDE_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "omegas2wide read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "omegas2wide read_otp_info end\n");
}

static int omegas2wide_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int omegas2wide_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
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
			DRV_LOG(ctx, "i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == SENSOR_ID) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			DRV_LOGE(ctx, "Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
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

	/* initail setting */
	sensor_init(ctx);

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

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return (1024 - (1024 * BASEGAIN) / gain);
}

void omegas2wide_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u set default\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = 0;
	}
	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "sensor_mode_num[%d]", ctx->s_ctx.sensor_mode_num);
	if (scenario_id < ctx->s_ctx.sensor_mode_num) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_NONE:
			if (ctx->s_ctx.mode[scenario_id].coarse_integ_step &&
				ctx->s_ctx.mode[scenario_id].min_exposure_line) {
				*exposure_step = ctx->s_ctx.mode[scenario_id].coarse_integ_step;
				*min_shutter = ctx->s_ctx.mode[scenario_id].min_exposure_line;
			} else {
				*exposure_step = ctx->s_ctx.exposure_step;
				*min_shutter = ctx->s_ctx.exposure_min;
			}
			break;
		default:
			*exposure_step = ctx->s_ctx.exposure_step;
			*min_shutter = ctx->s_ctx.exposure_min;
			break;
		}
	} else {
		DRV_LOG(ctx, "over sensor_mode_num[%d], use default", ctx->s_ctx.sensor_mode_num);
		*exposure_step = ctx->s_ctx.exposure_step;
		*min_shutter = ctx->s_ctx.exposure_min;
	}
	DRV_LOG(ctx, "scenario_id[%d] exposure_step[%llu] min_shutter[%llu]\n", scenario_id, *exposure_step, *min_shutter);
}

int omegas2wide_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	omegas2wide_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return 0;
}

static int omegas2wide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	if (mode) {
		switch(mode) {
		case 5:
			subdrv_i2c_wr_u8(ctx, 0x020E, 0x00);
			subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
			break;
		default:
			subdrv_i2c_wr_u8(ctx, 0x0601, mode);
			break;
		}
	} else if (ctx->test_pattern) {
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/
		subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x0218, 0x01);
	}

	ctx->test_pattern = mode;
	return ERROR_NONE;
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
	ctx->sof_cnt = sof_cnt;
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode_in_lbmf, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}
