// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 alphahfrontmipiraw_Sensor.c
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
#include "alphahfrontmipiraw_Sensor.h"

#define PFX "alphahfront_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define ALPHAHFRONT_EEPROM_READ_ID			0xA9
#define ALPHAHFRONT_EEPROM_WRITE_ID			0xA8
#define ALPHAHFRONT_MAX_OFFSET				0x4000
#define OTP_SIZE							0x2000
#define OTP_QSC_VALID_ADDR    				0x0EC0
#define QSC_IS_VALID_VAL      				0x01
#define OPLUS_CAMERA_COMMON_DATA_LENGTH		40
#define QSC_TYPE               (0x01)
#define ALPHAHFRONT_UNIQUE_SENSOR_ID_ADDR    (0x0A20)
#define ALPHAHFRONT_UNIQUE_SENSOR_ID_LENGTH  (11)

static BYTE alphahfront_unique_id[ALPHAHFRONT_UNIQUE_SENSOR_ID_LENGTH] = { 0 };
static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int alphahfront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void get_sensor_cali(void* arg);
static void set_sensor_cali(void *arg);
static int alphahfront_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int alphahfront_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_set_cali_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahfront_get_cloud_otp_info(struct subdrv_ctx *ctx, u8 *para, u32 *len);
/* STRUCT */

static struct eeprom_map_info alphahfront_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x000D, 0x000E, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x000D, 0x000E, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x000D, 0x000E, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x000D, 0x000E, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000C, 0x000D, 0x000E, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8, 23, true },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, alphahfront_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, alphahfront_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, alphahfront_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, alphahfront_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, alphahfront_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, alphahfront_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, alphahfront_streaming_resume},
	{SENSOR_FEATURE_SET_ESHUTTER, alphahfront_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, alphahfront_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_GAIN, alphahfront_set_gain},
	{SENSOR_FEATURE_GET_UNIQUE_SENSORID, alphahfront_get_unique_sensorid},
	{SENSOR_FEATURE_SET_CALI_DATA, alphahfront_set_cali_data},
	{SENSOR_FEATURE_GET_CLOUD_OTP_INFO, alphahfront_get_cloud_otp_info},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01C40055,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA8,

		.qsc_support = TRUE,
		.qsc_size = 560,
		.addr_qsc = 0x0C90, //QSC_EEPROM_ADDR
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2304,
			.vsize = 1728,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2304,
			.vsize = 1728,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2304,
			.vsize = 1296,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2304,
			.vsize = 1728,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2304,
			.vsize = 1296,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
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
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2304,
			.vsize = 1296,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4608,
			.vsize = 3456,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1632,
			.vsize = 1244,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = alphahfront_preview_setting,
		.mode_setting_len = ARRAY_SIZE(alphahfront_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 576000000,
		.linelength = 2560,
		.framelength = 7500,
		.max_framerate = 300,
		.mipi_pixel_rate = 587200000,
		.readout_length = 0,
		.read_margin = 18,
		.imgsensor_winsize_info = {
			.full_w = 4656,
			.full_h = 3496,
			.x0_offset = 24,
			.y0_offset = 20,
			.w0_size = 4608,
			.h0_size = 3456,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = alphahfront_capture_setting,
		.mode_setting_len = ARRAY_SIZE(alphahfront_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 576000000,
		.linelength = 2560,
		.framelength = 7500,
		.max_framerate = 300,
		.mipi_pixel_rate = 587200000,
		.readout_length = 0,
		.read_margin = 18,
		.imgsensor_winsize_info = {
			.full_w = 4656,
			.full_h = 3496,
			.x0_offset = 24,
			.y0_offset = 20,
			.w0_size = 4608,
			.h0_size = 3456,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = alphahfront_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(alphahfront_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 576000000,
		.linelength = 2560,
		.framelength = 7500,
		.max_framerate = 300,
		.mipi_pixel_rate = 587200000,
		.readout_length = 0,
		.read_margin = 18,
		.imgsensor_winsize_info = {
			.full_w = 4656,
			.full_h = 3496,
			.x0_offset = 24,
			.y0_offset = 444,
			.w0_size = 4608,
			.h0_size = 2608 ,
			.scale_w = 2304,
			.scale_h = 1304,
			.x1_offset = 0,
			.y1_offset = 4,
			.w1_size = 2304,
			.h1_size = 1296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1296,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = alphahfront_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(alphahfront_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 576000000,
		.linelength = 2560,
		.framelength = 3750,
		.max_framerate = 600,
		.mipi_pixel_rate = 587200000,
		.readout_length = 0,
		.read_margin = 18,
		.imgsensor_winsize_info = {
			.full_w = 4656,
			.full_h = 3496,
			.x0_offset = 24,
			.y0_offset = 444,
			.w0_size = 4608,
			.h0_size = 2608 ,
			.scale_w = 2304,
			.scale_h = 1304,
			.x1_offset = 0,
			.y1_offset = 4,
			.w1_size = 2304,
			.h1_size = 1296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1296,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 60,
		},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = alphahfront_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(alphahfront_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 216000000,
		.linelength = 2560,
		.framelength = 1406,
		.max_framerate = 600,
		.mipi_pixel_rate = 259200000,
		.readout_length = 0,
		.read_margin = 18,
		.imgsensor_winsize_info = {
			.full_w = 4656,
			.full_h = 3496,
			.x0_offset = 24,
			.y0_offset = 444,
			.w0_size = 4608,
			.h0_size = 2608,
			.scale_w = 2304,
			.scale_h = 1304,
			.x1_offset = 0,
			.y1_offset = 4,
			.w1_size = 2304,
			.h1_size = 1296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1296,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 120,
		},
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = alphahfront_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(alphahfront_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 576000000,
		.linelength = 2560,
		.framelength = 9374,
		.max_framerate = 240,
		.mipi_pixel_rate = 587200000,
		.readout_length = 0,
		.read_margin = 18,
		.imgsensor_winsize_info = {
			.full_w = 4656,
			.full_h = 3496,
			.x0_offset = 24,
			.y0_offset = 20,
			.w0_size = 4608,
			.h0_size = 3456,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},
	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = alphahfront_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(alphahfront_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 220000000,
		.linelength = 2560,
		.framelength = 1432,
		.max_framerate = 600,
		.mipi_pixel_rate = 218400000,
		.readout_length = 0,
		.read_margin = 18,
		.imgsensor_winsize_info = {
			.full_w = 4656,
			.full_h = 3496,
			.x0_offset = 24,
			.y0_offset = 444,
			.w0_size = 4608,
			.h0_size = 2608,
			.scale_w = 2304,
			.scale_h = 1304,
			.x1_offset = 0,
			.y1_offset = 4,
			.w1_size = 2304,
			.h1_size = 1296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1296,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = alphahfront_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(alphahfront_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 576000000,
		.linelength = 5120,
		.framelength = 3750,
		.max_framerate = 300,
		.mipi_pixel_rate = 551200000,
		.readout_length = 0,
		.read_margin = 18,
		.imgsensor_winsize_info = {
			.full_w = 4656,
			.full_h = 3496,
			.x0_offset = 24,
			.y0_offset = 20,
			.w0_size = 4608,
			.h0_size = 3456,
			.scale_w = 4608,
			.scale_h = 3456,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4608,
			.h1_size = 3456,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4608,
			.h2_tg_size = 3456,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 2,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_setting_info = {
			.sensor_scenario_usage = RMSC_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = alphahfront_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(alphahfront_custom4_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 132000000,
		.linelength = 2560,
		.framelength = 1718,
		.max_framerate = 300,
		.mipi_pixel_rate = 681600000,
		.readout_length = 0,
		.read_margin = 18,
		.imgsensor_winsize_info = {
			.full_w = 4656,
			.full_h = 3496,
			.x0_offset = 696,
			.y0_offset = 504,
			.w0_size = 3264,
			.h0_size = 2488,
			.scale_w = 1632,
			.scale_h = 1244,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1632,
			.h1_size = 1244,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1632,
			.h2_tg_size = 1244,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {0},
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 30,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = ALPHAHFRONT_SENSOR_ID,
	.reg_addr_sensor_id = {0x0A22, 0x0A23},
	.i2c_addr_table = {0x20, 0x34, 0xff},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {4656, 3496},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 0, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 1,
	.ana_gain_table = alphahfront_ana_gain_table,
	.ana_gain_table_size = sizeof(alphahfront_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xffff -18) * 128,
	.exposure_step = 1,
	.exposure_margin = 18,

	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 1935000,

	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.g_cali = get_sensor_cali,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3100,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = 0x0020,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = alphahfront_sensor_init_setting,
	.init_setting_len =  ARRAY_SIZE(alphahfront_sensor_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
	.checksum_value = 0xb1893b4f,
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
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 11},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DVDD, 1056000, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 6, 1},
	{HW_ID_RST, 1, 2},
};

static struct subdrv_pw_seq_entry oplus_pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DVDD, 1056000, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 6, 1},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry alphahfront_mipi_raw_entry = {
	.name = "alphahfront_mipi_raw",
	.id = ALPHAHFRONT_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static void alphahfront_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	subdrv_i2c_wr_u8(ctx, 0x0350, 0x00); /* Disable auto extend */
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);

	subdrv_i2c_wr_u8(ctx, 0x0340, ctx->frame_length >> 8);
	subdrv_i2c_wr_u8(ctx, 0x0341, ctx->frame_length & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0342, ctx->line_length >> 8);
	subdrv_i2c_wr_u8(ctx, 0x0343, ctx->line_length & 0xFF);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
}				/*      set_dummy  */

static void alphahfront_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	alphahfront_set_dummy(ctx);
}

void alphahfront_set_long_exposure(struct subdrv_ctx *ctx)
{
	u32 shutter = ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE];
	u32 l_shutter = 0;
	u16 l_shift = 0;

	if (shutter > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) {
		if (ctx->s_ctx.long_exposure_support == FALSE) {
			DRV_LOGE(ctx, "sensor no support of exposure lshift!\n");
			return;
		}
		if (ctx->s_ctx.reg_addr_exposure_lshift == PARAM_UNDEFINED) {
			DRV_LOGE(ctx, "please implement lshift register address\n");
			return;
		}
		for (l_shift = 1; l_shift < 7; l_shift++) {
			l_shutter = ((shutter - 1) >> l_shift) + 1;
			if (l_shutter
				< (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
				break;
		}
		if (l_shift > 7) {
			DRV_LOGE(ctx, "unable to set exposure:%u, set to max\n", shutter);
			l_shift = 7;
		}
		shutter = ((shutter - 1) >> l_shift) + 1;
		ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
		DRV_LOG(ctx, "long exposure mode: lshift %u times", l_shift);
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure_lshift, l_shift);
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
		ctx->l_shift = l_shift;
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		if (ctx->s_ctx.reg_addr_exposure_lshift != PARAM_UNDEFINED) {
			subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure_lshift, l_shift);
			subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
			ctx->l_shift = l_shift;
		}
		ctx->current_ae_effective_frame = 2;
	}

	ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE] = shutter;
}

static void alphahfront_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 *shutter, u32 frame_length, bool auto_extend_en)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	u8 exposure_margin = ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin ? ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin : ctx->s_ctx.exposure_margin;

	ctx->exposure[0] = *shutter;

	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;

	ctx->frame_length = ctx->frame_length + dummy_line;

	if (ctx->exposure[0] > ctx->frame_length - exposure_margin)
		ctx->frame_length = ctx->exposure[0] + exposure_margin;

	if (ctx->frame_length > ctx->max_frame_length)
		ctx->frame_length = ctx->max_frame_length;

	ctx->exposure[0] = (ctx->exposure[0] < ctx->s_ctx.exposure_min)
			? ctx->s_ctx.exposure_min : ctx->exposure[0];

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			alphahfront_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			alphahfront_set_max_framerate(ctx, 146, 0);
		} else {
			// Extend frame length
			subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x0340, ctx->frame_length >> 8);
			subdrv_i2c_wr_u8(ctx, 0x0341, ctx->frame_length & 0xFF);
			subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
		}
	} else {
		// Extend frame length
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x0340, ctx->frame_length >> 8);
		subdrv_i2c_wr_u8(ctx, 0x0341, ctx->frame_length & 0xFF);
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
	}
	/* Update Shutter */
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	if (auto_extend_en)
		subdrv_i2c_wr_u8(ctx, 0x0350, 0x01); /* Enable auto extend */
	else
		subdrv_i2c_wr_u8(ctx, 0x0350, 0x00); /* Disable auto extend */

	subdrv_i2c_wr_u8(ctx, 0x0202, (ctx->exposure[0] >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0203, ctx->exposure[0] & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
	DRV_LOG(ctx, "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		ctx->exposure[0], ctx->frame_length, frame_length, dummy_line, subdrv_i2c_rd_u16(ctx, 0x0350));
}	/* set_shutter_frame_length */

static int alphahfront_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "shutter:%u, frame_length:%u\n", (u32)(*para), (u32) (*(para + 1)));
	alphahfront_set_shutter_frame_length_convert(ctx, (u32 *)para, (u32) (*(para + 1)), (u16) (*(para + 2)));
	return 0;
}

static void alphahfront_write_shutter(struct subdrv_ctx *ctx)
{
	kal_uint16 realtime_fps = 0;
	u8 exposure_margin = 0;
	DRV_LOG(ctx, "===brad shutter:%d\n", ctx->exposure[0]);
	exposure_margin = ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin ? ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin : ctx->s_ctx.exposure_margin;
  	LOG_INF("exposure_margin:%d\n", exposure_margin);

	if (ctx->exposure[0] > ctx->min_frame_length - exposure_margin) {
		ctx->frame_length = ctx->exposure[0] + exposure_margin;
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
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			alphahfront_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			alphahfront_set_max_framerate(ctx, 146, 0);
		} else {
			// Extend frame length
			subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x0340, ctx->frame_length >> 8);
			subdrv_i2c_wr_u8(ctx, 0x0341, ctx->frame_length & 0xFF);
			subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
		}
	} else {
		// Extend frame length
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x0340, ctx->frame_length >> 8);
		subdrv_i2c_wr_u8(ctx, 0x0341, ctx->frame_length & 0xFF);
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
	}
	/* Long Exposure */
	alphahfront_set_long_exposure(ctx);

	/* Update Shutter */
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0350, 0x01); /* Enable auto extend */
	subdrv_i2c_wr_u8(ctx, 0x0202, (ctx->exposure[0] >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0203, ctx->exposure[0] & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
	DRV_LOG(ctx, "shutter =%d, framelength =%d\n", ctx->exposure[0], ctx->frame_length);
}	/*	write_shutter  */

static void alphahfront_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	alphahfront_write_shutter(ctx);
}

static int alphahfront_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *para);
	alphahfront_set_shutter_convert(ctx, (u32 *)para);
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
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
		if (ctx->s_ctx.reg_addr_fast_mode && ctx->fast_mode_on) {
			ctx->fast_mode_on = FALSE;
			ctx->ref_sof_cnt = 0;
			DRV_LOG(ctx, "seamless_switch disabled.");
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
			commit_i2c_buffer(ctx);
		}
	}
	mdelay(10);
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static int alphahfront_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			alphahfront_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int alphahfront_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
		return 0;
}

static unsigned int read_alphahfront_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != alphahfront_eeprom_info[meta_id].meta)
		return -1;

	if (size != alphahfront_eeprom_info[meta_id].size)
		return -1;

	addr = alphahfront_eeprom_info[meta_id].start;
	readsize = alphahfront_eeprom_info[meta_id].size;

	if (!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
	.i2c_read_id = 0xA9,
	.i2c_write_id = 0xA8,

	.addr_modinfo = 0x0000,
	.addr_sensorid = 0x0006,
	.addr_lens = 0x0008,
	.addr_vcm = 0x000A,
	.addr_modinfoflag = 0x000D,

	.addr_qrcode = 0x00B0,
	.addr_qrcodeflag = 0x00C7,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	*len = sizeof(oplus_eeprom_info);
	return 0;
}

static struct SENSOR_OTP_INFO_STRUCT cloud_otp_info[OPLUS_CAM_CAL_DATA_MAX] = {
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0000, 14}}, /*{addr_modinfo, addr_modinfolen}*/
	}, /*OPLUS_CAM_CAL_DATA_MODULE_VERSION*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0000, 14}}, /*{addr_modinfo, addr_modinfolen}*/
	}, /*OPLUS_CAM_CAL_DATA_PART_NUMBER*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0530, 1868}},
	}, /*OPLUS_CAM_CAL_DATA_SHADING_TABLE--LSC*/
	{
		.OtpInfoLen = 5,
		.OtpInfo = {{0x0010, 16}, {0x0034, 16}, {0x0060, 4}, {0x006c, 4}, {0x0072, 6}},
		.isAFCodeOffset = KAL_FALSE,
	}, /*OPLUS_CAM_CAL_DATA_3A_GAIN-awb5000\awb2850\awb5000Light\awb2850light*/
	{
		.OtpInfoLen = 0,
	}, /*OPLUS_CAM_CAL_DATA_PDAF*/
	{
		.OtpInfoLen = 8,
		.OtpInfo = {{0x0000, 17}, {0x0006, 2}, {0x0008, 2}, {0x000a, 2}, {0x0072, 7}, {0x0072, 2}, {0x0074, 2}, {0x00b0, 24}},
		.isAFCodeOffset = KAL_FALSE,
	}, /*OPLUS_CAM_CAL_DATA_CAMERA_INFO-modid\sensor\lens\vcmid\af\macpos\infpos\qrcode\*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0008, 2}},
	}, /*OPLUS_CAM_CAL_DATA_DUMP*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0008, 2}},
	}, /*OPLUS_CAM_CAL_DATA_LENS_ID*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0C90, 560}},
	}, /*OPLUS_CAM_CAL_DATA_QSC*/
	{
		.OtpInfoLen = 0,
	}, /*OPLUS_CAM_CAL_DATA_LRC*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0000, 8192}},
	}, /*OPLUS_CAM_CAL_DATA_ALL*/
};

static int alphahfront_get_cloud_otp_info(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	struct SENSOR_OTP_INFO_STRUCT *cloudinfo;
	LOG_INF("SENSOR_FEATURE_GET_CLOUD_OTP_INFO otp_type:%d", (UINT32)(*feature_data));
	cloudinfo = (struct SENSOR_OTP_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
	switch (*feature_data) {
	case OPLUS_CAM_CAL_DATA_MODULE_VERSION:
	case OPLUS_CAM_CAL_DATA_PART_NUMBER:
	case OPLUS_CAM_CAL_DATA_SHADING_TABLE:
	case OPLUS_CAM_CAL_DATA_3A_GAIN:
	case OPLUS_CAM_CAL_DATA_PDAF:
	case OPLUS_CAM_CAL_DATA_CAMERA_INFO:
	case OPLUS_CAM_CAL_DATA_DUMP:
	case OPLUS_CAM_CAL_DATA_LENS_ID:
	case OPLUS_CAM_CAL_DATA_QSC:
	case OPLUS_CAM_CAL_DATA_LRC:
	case OPLUS_CAM_CAL_DATA_ALL:
		memcpy((void *)cloudinfo, (void *)&cloud_otp_info[*feature_data], sizeof(struct SENSOR_OTP_INFO_STRUCT));
		break;
	default:
		break;
	}
	return 0;
}

static void read_unique_sensorid(struct subdrv_ctx *ctx)
{
	u8 i = 0;
	LOG_INF("read sensor unique sensorid");
	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		subdrv_i2c_wr_u8(ctx, 0x0A02, 0x1B);
		subdrv_i2c_wr_u8(ctx, 0x0A00, 0x01);
		if (adaptor_i2c_rd_p8(ctx->i2c_client, ctx->i2c_write_id >> 1, ALPHAHFRONT_UNIQUE_SENSOR_ID_ADDR,
			&(alphahfront_unique_id[0]), ALPHAHFRONT_UNIQUE_SENSOR_ID_LENGTH) < 0) {
			LOG_INF("Read sensor unique sensorid fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
		}
		i++;
	}
}

static int alphahfront_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	*len = ALPHAHFRONT_UNIQUE_SENSOR_ID_LENGTH;
	memcpy(feature_return_para_32, alphahfront_unique_id,
		ALPHAHFRONT_UNIQUE_SENSOR_ID_LENGTH);
	LOG_INF("para :%x, get unique sensorid", *para);
	return 0;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
    kal_uint16 get_byte = 0;

    adaptor_i2c_rd_u8(ctx->i2c_client, ALPHAHFRONT_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
    return get_byte;
}

#ifdef WRITE_DATA_MAX_LENGTH
#undef WRITE_DATA_MAX_LENGTH
#endif
#define   WRITE_DATA_MAX_LENGTH     (32)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, ALPHAHFRONT_EEPROM_WRITE_ID >> 1,
            addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xE000;
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, ALPHAHFRONT_EEPROM_WRITE_ID >> 1, reg, 0xA1);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, ALPHAHFRONT_EEPROM_WRITE_ID >> 1, reg, 0xA0);
    }

    return ret;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
    ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
    kal_int32  ret = ERROR_NONE;
    kal_uint16 data_base, data_length;
    kal_uint32 idx, idy;
    kal_uint8 *pData;
    UINT32 i = 0;
    kal_uint16 offset = 0;
    if(pStereodata != NULL) {
        LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
            pStereodata->uSensorId,
            pStereodata->uDeviceId,
            pStereodata->baseAddr,
            pStereodata->dataLength);

        data_base = pStereodata->baseAddr;
        data_length = pStereodata->dataLength;
        pData = pStereodata->uData;
        offset = ALIGN(data_base, WRITE_DATA_MAX_LENGTH) - data_base;
        if (offset > data_length) {
            offset = data_length;
        }
        if ((pStereodata->uSensorId == ALPHAHFRONT_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == ALPHAHFRONT_STEREO_START_ADDR)) {
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);
            /* close write protect */
            write_eeprom_protect(ctx, 0);
            msleep(6);
            if (offset > 0) {
                ret = table_write_eeprom_30Bytes(ctx, data_base, &pData[0], offset);
                if (ret != ERROR_NONE) {
                    LOG_INF("write_eeprom error: offset\n");
                    /* open write protect */
                    write_eeprom_protect(ctx, 1);
                    msleep(6);
                    return -1;
                }
                msleep(6);
                data_base += offset;
                data_length -= offset;
                pData += offset;
            }
            idx = data_length/WRITE_DATA_MAX_LENGTH;
            idy = data_length%WRITE_DATA_MAX_LENGTH;
            for (i = 0; i < idx; i++ ) {
                ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*i),
                    &pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
                if (ret != ERROR_NONE) {
                    LOG_INF("write_eeprom error: i= %d\n", i);
                    /* open write protect */
                    write_eeprom_protect(ctx, 1);
                    msleep(6);
                    return -1;
                }
                msleep(6);
            }
            ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*idx),
                &pData[WRITE_DATA_MAX_LENGTH*idx], idy);
            if (ret != ERROR_NONE) {
                LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
                /* open write protect */
                write_eeprom_protect(ctx, 1);
                msleep(6);
                return -1;
            }
            msleep(6);
            /* open write protect */
            write_eeprom_protect(ctx, 1);
            msleep(6);
            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");
        } else if ((pStereodata->uSensorId == ALPHAHFRONT_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == ALPHAHFRONT_AESYNC_START_ADDR)) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
            /* close write protect */
            write_eeprom_protect(ctx, 0);
            msleep(6);
            if (offset > 0) {
                ret = table_write_eeprom_30Bytes(ctx, data_base, &pData[0], offset);
                if (ret != ERROR_NONE) {
                    LOG_INF("write_eeprom error: offset\n");
                    /* open write protect */
                    write_eeprom_protect(ctx, 1);
                    msleep(6);
                    return -1;
                }
                msleep(6);
                data_base += offset;
                data_length -= offset;
                pData += offset;
            }
            idx = data_length/WRITE_DATA_MAX_LENGTH;
            idy = data_length%WRITE_DATA_MAX_LENGTH;
            for (i = 0; i < idx; i++ ) {
                ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*i),
                    &pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
                if (ret != ERROR_NONE) {
                    LOG_INF("write_eeprom error: i= %d\n", i);
                    /* open write protect */
                    write_eeprom_protect(ctx, 1);
                    msleep(6);
                    return -1;
                }
                msleep(6);
            }
            ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*idx),
                &pData[WRITE_DATA_MAX_LENGTH*idx], idy);
            if (ret != ERROR_NONE) {
                LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
                /* open write protect */
                write_eeprom_protect(ctx, 1);
                msleep(6);
                return -1;
            }
            msleep(6);
            /* open write protect */
            write_eeprom_protect(ctx, 1);
            msleep(6);
            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                read_cmos_eeprom_8(ctx, ALPHAHFRONT_AESYNC_START_ADDR),
                read_cmos_eeprom_8(ctx, ALPHAHFRONT_AESYNC_START_ADDR+1),
                read_cmos_eeprom_8(ctx, ALPHAHFRONT_AESYNC_START_ADDR+2),
                read_cmos_eeprom_8(ctx, ALPHAHFRONT_AESYNC_START_ADDR+3),
                read_cmos_eeprom_8(ctx, ALPHAHFRONT_AESYNC_START_ADDR+4),
                read_cmos_eeprom_8(ctx, ALPHAHFRONT_AESYNC_START_ADDR+5),
                read_cmos_eeprom_8(ctx, ALPHAHFRONT_AESYNC_START_ADDR+6),
                read_cmos_eeprom_8(ctx, ALPHAHFRONT_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("alphahfront write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int alphahfront_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        *len = (u32)-1; /*write eeprom failed*/
        LOG_INF("ret=%d\n", ret);
    }
    return 0;
}

static int alphahfront_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	if(*len > CALI_DATA_SLAVE_LENGTH) {
		*len = CALI_DATA_SLAVE_LENGTH;
	}
	read_alphahfront_eeprom_info(ctx, EEPROM_META_STEREO_DATA,
			(BYTE *)para, *len);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, ALPHAHFRONT_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "alphahfront read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "alphahfront read_otp_info end\n");
}

static int alphahfront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int alphahfront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

	ctx->i2c_write_id = 0x20;
	LOG_INF("reg(0x300a) = 0x%x, reg(0x300b) = 0x%x\n", subdrv_i2c_rd_u8(ctx, 0x300a), subdrv_i2c_rd_u8(ctx, 0x300b));
	LOG_INF("reg(0x0A22) = 0x%x, reg(0x0A23) = 0x%x\n", subdrv_i2c_rd_u8(ctx, 0x0A22), subdrv_i2c_rd_u8(ctx, 0x0A23));
	LOG_INF("reg(0x0136) = 0x%x, reg(0x0137) = 0x%x\n", subdrv_i2c_rd_u8(ctx, 0x0136), subdrv_i2c_rd_u8(ctx, 0x0137));

	subdrv_i2c_wr_u8(ctx, 0x0A02, 0x1B);
	subdrv_i2c_wr_u8(ctx, 0x0A00, 0x01);
	LOG_INF("rst delay = %d, func: %s, line: %d\n", pw_seq[1].delay, __FUNCTION__, __LINE__);

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			*sensor_id = (*sensor_id >> 4) & 0xFFFF;
			LOG_INF("i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x0471) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					read_unique_sensorid(ctx);
					memcpy(&pw_seq, &oplus_pw_seq, sizeof(oplus_pw_seq));
					first_read = FALSE;
				}
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
			LOG_INF("sensor_id = 0x%x, ctx->s_ctx.sensor_id = 0x%x\n",
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

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	else
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	u16 reg_gain = 0x0;
	reg_gain = 1024 - (1024 * BASEGAIN) / gain;
	return reg_gain;
}

static int alphahfront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode) {
		if (mode == 5) {
			subdrv_i2c_wr_u8(ctx, 0x0600, mode >> 4); /*black*/
			subdrv_i2c_wr_u8(ctx, 0x0601, mode); /*black*/
		} else {
			subdrv_i2c_wr_u8(ctx, 0x0600, mode); /*100% Color bar*/
		}
	}
	else if (ctx->test_pattern) {
		subdrv_i2c_wr_u8(ctx, 0x0600, 0x0000); /*No pattern*/
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x0000);
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

void get_sensor_cali(void* arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *buf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u8 qsc_is_valid = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	size = info[idx].qsc_size;
	addr = info[idx].addr_qsc;
	buf = info[idx].qsc_table;
	if (support && size > 0) {
		// Check QSC validation
		qsc_is_valid = i2c_read_eeprom(ctx, OTP_QSC_VALID_ADDR);
		if (qsc_is_valid != QSC_IS_VALID_VAL) {
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

	ctx->is_read_preload_eeprom = 1;
}

static int alphahfront_set_cali_data(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	u16 idx = 0;
	u8 support = FALSE;
	u8 *buf = NULL;
	u32 size = 0;
	u8 qsc_is_valid = 0;
	u8 type = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (ctx->is_read_preload_eeprom >= 2) {
        return 0;
	}
	type = para[(*len) + 1];
	LOG_INF("set_cali_data flow enter");
	if (type == QSC_TYPE) {
		/* QSC data */
		support = info[idx].qsc_support;
		info[idx].qsc_size = *len;
		size = *len;
		buf = para;
		if (support && size > 0 && buf != NULL) {
			/* Check QSC validation */
			qsc_is_valid = buf[*len];
			if (qsc_is_valid == QSC_IS_VALID_VAL) {
				DRV_LOG(ctx, "QSC data is valid, flag(%02x)", qsc_is_valid);
				if (info[idx].preload_qsc_table == NULL) {
					info[idx].preload_qsc_table = kmalloc(size, GFP_KERNEL);
				}
				memcpy(info[idx].preload_qsc_table, buf, size);
				ctx->is_read_preload_eeprom = 2;
				DRV_LOG(ctx, "preload QSC data %u bytes", size);
			} else {
				DRV_LOGE(ctx, "QSC data is invalid, flag(%02x)", qsc_is_valid);
			}
		}
	}
	LOG_INF("set_cali_data flow exit");

	return 0;
}

static void set_sensor_cali(void *arg)
{
	//struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	return;
}

int alphahfront_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;
	u32 gain = *feature_data;
	u16 rg_gain;

	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	/* check boundary of gain */
	gain = max(gain, ctx->s_ctx.ana_gain_min);
	gain = min(gain, ctx->s_ctx.ana_gain_max);
	/* mapping of gain to register value */
	if (ctx->s_ctx.g_gain2reg != NULL)
		rg_gain = ctx->s_ctx.g_gain2reg(gain);
	else
		rg_gain = gain2reg(gain);
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	/* group hold start */
	if (gph && !ctx->ae_ctrl_gph_en)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* write gain */
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[0],
		(rg_gain >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[1],
		rg_gain & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
	DRV_LOG(ctx, "gain[0x%x]\n", rg_gain);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	/* group hold end */
	return 0;
}
