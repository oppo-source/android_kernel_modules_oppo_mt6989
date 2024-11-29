// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 casiofrontmipiraw_Sensor.c
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
#include "casiofrontmipiraw_Sensor.h"

#define CASIOFRONT_EEPROM_READ_ID	0xA9
#define CASIOFRONT_EEPROM_WRITE_ID   0xA8
#define CASIOFRONT_MAX_OFFSET		0x4000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "casiofront_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x4000
#define OTP_QCOM_PDAF_DATA_LENGTH 0x468
#define OTP_QCOM_PDAF_DATA_START_ADDR 0x600
#define FOUR_CELL_SIZE 2048
#define FOUR_CELL_ADDR 0x0E00
#define casiofront_table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u16(__VA_ARGS__)
#define casiofront_burst_write_cmos_sensor(...) subdrv_i2c_wr_p16(__VA_ARGS__)
static bool bNeedSetNormalMode = FALSE;
static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static kal_uint8 otp_qcom_pdaf_data[OTP_QCOM_PDAF_DATA_LENGTH] = {0};
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int casiofront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void casiofront_sensor_init(struct subdrv_ctx *ctx);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void get_sensor_cali(void* arg);
static void set_sensor_cali(void *arg);
static int casiofront_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiofront_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int casiofront_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int casiofront_get_eeprom_4cell_info(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void casiofront_read_eeprom_4cell_info(struct subdrv_ctx *ctx);
/* STRUCT */

static struct eeprom_map_info casiofront_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8, 23, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, false },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, false },
	{ EEPROM_META_STEREO_DATA, 0x51F0, 0x0000, 0x0000, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x2B00, 0x3199, 0x319A, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0x31C0, 0x3859, 0x385A, CALI_DATA_MASTER_LENGTH, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, casiofront_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, casiofront_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, casiofront_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, casiofront_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, casiofront_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, casiofront_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, casiofront_streaming_resume},
	{SENSOR_FEATURE_SET_ESHUTTER, casiofront_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, casiofront_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_GAIN, casiofront_set_gain},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, casiofront_set_hdr_tri_shutter2},
	{SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, casiofront_set_hdr_tri_shutter3},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, casiofront_set_multi_shutter_frame_length_ctrl},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_DATA, casiofront_get_otp_qcom_pdaf_data},
	{SENSOR_FEATURE_GET_4CELL_DATA, casiofront_get_eeprom_4cell_info},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01A8010A,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA8,
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

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
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

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
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
			.hsize = 2320,
			.vsize = 1744,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2320,
			.vsize = 1744,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4640,
			.vsize = 3488,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = casiofront_preview_setting,
		.mode_setting_len = ARRAY_SIZE(casiofront_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 5088,
		.framelength = 3664,
		.max_framerate = 300,
		.mipi_pixel_rate = 480000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4640,
			.full_h = 3488,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4640,
			.h0_size = 3488,
			.scale_w = 2320,
			.scale_h = 1744,
			.x1_offset = 8,
			.y1_offset = 8,
			.w1_size = 2304,
			.h1_size = 1728,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1728,
		},
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = casiofront_capture_setting,
		.mode_setting_len = ARRAY_SIZE(casiofront_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 5088,
		.framelength = 3664,
		.max_framerate = 300,
		.mipi_pixel_rate = 480000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4640,
			.full_h = 3488,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4640,
			.h0_size = 3488,
			.scale_w = 2320,
			.scale_h = 1744,
			.x1_offset = 8,
			.y1_offset = 8,
			.w1_size = 2304,
			.h1_size = 1728,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1728,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = casiofront_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(casiofront_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 12960,
		.framelength = 1440,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4640,
			.full_h = 3488,
			.x0_offset = 8,
			.y0_offset = 448,
			.w0_size = 4624,
			.h0_size = 2592,
			.scale_w = 2312,
			.scale_h = 1296,
			.x1_offset = 4,
			.y1_offset = 0,
			.w1_size = 2304,
			.h1_size = 1296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1296,
		},
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = casiofront_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(casiofront_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 12960,
		.framelength = 1440,
		.max_framerate = 300,
		.mipi_pixel_rate = 220800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4640,
			.full_h = 3488,
			.x0_offset = 8,
			.y0_offset = 448,
			.w0_size = 4624,
			.h0_size = 2592,
			.scale_w = 2312,
			.scale_h = 1296,
			.x1_offset = 4,
			.y1_offset = 0,
			.w1_size = 2304,
			.h1_size = 1296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1296,
		},
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = casiofront_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(casiofront_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 12960,
		.framelength = 1440,
		.max_framerate = 300,
		.mipi_pixel_rate = 220800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4640,
			.full_h = 3488,
			.x0_offset = 8,
			.y0_offset = 448,
			.w0_size = 4624,
			.h0_size = 2592,
			.scale_w = 2312,
			.scale_h = 1296,
			.x1_offset = 4,
			.y1_offset = 0,
			.w1_size = 2304,
			.h1_size = 1296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1296,
		},
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = casiofront_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(casiofront_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 10036,
		.framelength = 2321,
		.max_framerate = 240,
		.mipi_pixel_rate = 292800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4640,
			.full_h = 3488,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4640,
			.h0_size = 3488,
			.scale_w = 2320,
			.scale_h = 1744,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2320,
			.h1_size = 1744,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2320,
			.h2_tg_size = 1744,
		},
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = casiofront_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(casiofront_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 7152,
		.framelength = 2608,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4640,
			.full_h = 3488,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4640,
			.h0_size = 3488,
			.scale_w = 2320,
			.scale_h = 1744,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2320,
			.h1_size = 1744,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2320,
			.h2_tg_size = 1744,
		},
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = casiofront_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(casiofront_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 5088,
		.framelength = 3668,
		.max_framerate = 300,
		.mipi_pixel_rate = 585600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4640,
			.full_h = 3488,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4640,
			.h0_size = 3488,
			.scale_w = 4640,
			.scale_h = 3488,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4640,
			.h1_size = 3488,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4640,
			.h2_tg_size = 3488,
		},
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = CASIOFRONT_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x20, 0x21, 0xFF},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {4640, 3488},
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
	.ana_gain_type = 2, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 32,
	.ana_gain_table = casiofront_ana_gain_table,
	.ana_gain_table_size = sizeof(casiofront_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 3,
	.exposure_max = 0xFFFF - 3,
	.exposure_step = 1,
	.exposure_margin = 3,

	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 500000,

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
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len =  PARAM_UNDEFINED,
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 1,
	.checksum_value = 0x31E3FBE2,
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
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DVDD, 1062000, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry casiofront_mipi_raw_entry = {
	.name = "casiofront_mipi_raw",
	.id = CASIOFRONT_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static void casiofront_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	/* return; //for test */
	subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
	subdrv_i2c_wr_u16(ctx, 0x0342, ctx->line_length);
}				/*      set_dummy  */

static void casiofront_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	casiofront_set_dummy(ctx);
}

static void casiofront_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 *shutter, u32 frame_length, bool auto_extend_en)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	u64 CintR = 0;
	u64 Time_Farme = 0;

	ctx->exposure[0] = *shutter;

	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;

	ctx->frame_length = ctx->frame_length + dummy_line;

	if (ctx->exposure[0] > ctx->frame_length - ctx->s_ctx.exposure_margin)
		ctx->frame_length = ctx->exposure[0] + ctx->s_ctx.exposure_margin;

	if (ctx->frame_length > ctx->max_frame_length)
		ctx->frame_length = ctx->max_frame_length;

	ctx->exposure[0] = (ctx->exposure[0] < ctx->s_ctx.exposure_min)
			? ctx->s_ctx.exposure_min : ctx->exposure[0];

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps > 592 && realtime_fps <= 607) {
			casiofront_set_max_framerate(ctx, 592, 0);
		} else if (realtime_fps > 296 && realtime_fps <= 305) {
			casiofront_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 246 && realtime_fps <= 253) {
			casiofront_set_max_framerate(ctx, 246, 0);
		} else if (realtime_fps > 236 && realtime_fps <= 243) {
			casiofront_set_max_framerate(ctx, 236, 0);
		} else if (realtime_fps > 146 && realtime_fps <= 153) {
			casiofront_set_max_framerate(ctx, 146, 0);
		} else {
			/* Extend frame length */
			subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
		}
	} else {
		/* Extend frame length */
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
	}

	if (ctx->exposure[0] >= 0xFFF0) {
		bNeedSetNormalMode = TRUE;

		if (ctx->exposure[0] >= 1538000) {
			ctx->exposure[0] = 1538000;
		}
		CintR = (5013 * (unsigned long long)(ctx->exposure[0])) / 321536;
		Time_Farme = CintR + 0x0002;
		DRV_LOG(ctx, "CintR =%llu \n", CintR);

		subdrv_i2c_wr_u16(ctx, 0x0340, Time_Farme & 0xFFFF);
		subdrv_i2c_wr_u16(ctx, 0x0202, CintR & 0xFFFF);
		subdrv_i2c_wr_u16(ctx, 0x0702, 0x0600);
		subdrv_i2c_wr_u16(ctx, 0x0704, 0x0600);
	} else {
		if (bNeedSetNormalMode) {
			DRV_LOG(ctx, "exit long shutter\n");
			subdrv_i2c_wr_u16(ctx, 0x0702, 0x0000);
			subdrv_i2c_wr_u16(ctx, 0x0704, 0x0000);
			bNeedSetNormalMode = FALSE;
		}

		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
		subdrv_i2c_wr_u16(ctx, 0x0202, ctx->exposure[0]);
	}

	DRV_LOG(ctx, "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		ctx->exposure[0], ctx->frame_length, frame_length, dummy_line, subdrv_i2c_rd_u16(ctx, 0x0350));
}	/* set_shutter_frame_length */

static int casiofront_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "shutter:%u, frame_length:%u\n", (u32)(*para), (u32) (*(para + 1)));
	casiofront_set_shutter_frame_length_convert(ctx, (u32 *)para, (u32) (*(para + 1)), (u16) (*(para + 2)));
	return 0;
}

static void casiofront_write_shutter(struct subdrv_ctx *ctx)
{
	kal_uint16 realtime_fps = 0;
	u64 CintR = 0;
	u64 Time_Farme = 0;
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
			casiofront_set_max_framerate(ctx, 592, 0);
		} else if (realtime_fps > 296 && realtime_fps <= 305) {
			casiofront_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 246 && realtime_fps <= 253) {
			casiofront_set_max_framerate(ctx, 246, 0);
		} else if (realtime_fps > 236 && realtime_fps <= 243) {
			casiofront_set_max_framerate(ctx, 236, 0);
		} else if (realtime_fps > 146 && realtime_fps <= 153) {
			casiofront_set_max_framerate(ctx, 146, 0);
		} else {
			// Extend frame length
			subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
		}
	} else {
		// Extend frame length
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
	}

	if (ctx->exposure[0] >= 0xFFF0) {  // need to modify line_length & PCLK
		bNeedSetNormalMode = TRUE;

		if (ctx->exposure[0] >= 6153846) {  //>32s
			ctx->exposure[0] = 6153846;
		}

		CintR = ( (unsigned long long)(ctx->exposure[0])) / 128;
		Time_Farme = CintR + 0x0002;  // 1st framelength
		DRV_LOG(ctx, "CintR =%llu \n", CintR);

		subdrv_i2c_wr_u16(ctx, 0x0340, Time_Farme & 0xFFFF);  // Framelength
		subdrv_i2c_wr_u16(ctx, 0x0202, CintR & 0xFFFF);  //shutter
		subdrv_i2c_wr_u16(ctx, 0x0702, 0x0700);
		subdrv_i2c_wr_u16(ctx, 0x0704, 0x0700);
	} else {
		if (bNeedSetNormalMode) {
			DRV_LOG(ctx, "exit long shutter\n");
			subdrv_i2c_wr_u16(ctx, 0x0702, 0x0000);
			subdrv_i2c_wr_u16(ctx, 0x0704, 0x0000);
			bNeedSetNormalMode = FALSE;
		}

		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
		subdrv_i2c_wr_u16(ctx, 0x0202, ctx->exposure[0]);
	}
	DRV_LOG(ctx, "shutter =%d, framelength =%d\n", ctx->exposure[0], ctx->frame_length);
}	/*	write_shutter  */

static void casiofront_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	casiofront_write_shutter(ctx);
}

static int casiofront_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *para);
	casiofront_set_shutter_convert(ctx, (u32 *)para);
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
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static int casiofront_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOGE(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			casiofront_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int casiofront_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOGE(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
		return 0;
}

static unsigned int read_casiofront_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != casiofront_eeprom_info[meta_id].meta)
		return -1;

	if (size != casiofront_eeprom_info[meta_id].size)
		return -1;

	addr = casiofront_eeprom_info[meta_id].start;
	readsize = casiofront_eeprom_info[meta_id].size;

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
	.addr_modinfoflag = 0x000F,

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

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
    kal_uint16 get_byte = 0;

    adaptor_i2c_rd_u8(ctx->i2c_client, CASIOFRONT_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
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
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, CASIOFRONT_EEPROM_WRITE_ID >> 1,
            addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xE000;
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, CASIOFRONT_EEPROM_WRITE_ID >> 1, reg, 0xA1);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, CASIOFRONT_EEPROM_WRITE_ID >> 1, reg, 0xA0);
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
        if ((pStereodata->uSensorId == CASIOFRONT_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == CASIOFRONT_STEREO_START_ADDR)) {
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
        } else if ((pStereodata->uSensorId == CASIOFRONT_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == 0xff)) {
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
/*             LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                read_cmos_eeprom_8(ctx, CASIOFRONT_AESYNC_START_ADDR),
                read_cmos_eeprom_8(ctx, CASIOFRONT_AESYNC_START_ADDR+1),
                read_cmos_eeprom_8(ctx, CASIOFRONT_AESYNC_START_ADDR+2),
                read_cmos_eeprom_8(ctx, CASIOFRONT_AESYNC_START_ADDR+3),
                read_cmos_eeprom_8(ctx, CASIOFRONT_AESYNC_START_ADDR+4),
                read_cmos_eeprom_8(ctx, CASIOFRONT_AESYNC_START_ADDR+5),
                read_cmos_eeprom_8(ctx, CASIOFRONT_AESYNC_START_ADDR+6),
                read_cmos_eeprom_8(ctx, CASIOFRONT_AESYNC_START_ADDR+7)); */
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("casiofront write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int casiofront_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        *len = (u32)-1; /*write eeprom failed*/
        LOG_INF("ret=%d\n", ret);
    }
    return 0;
}

static int casiofront_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	if(*len > CALI_DATA_SLAVE_LENGTH) {
		*len = CALI_DATA_SLAVE_LENGTH;
	}
	read_casiofront_eeprom_info(ctx, EEPROM_META_STEREO_DATA,
			(BYTE *)para, *len);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, CASIOFRONT_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static int casiofront_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int casiofront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int casiofront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}

static bool first_read = TRUE;
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
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
			if (*sensor_id == 0x3109) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					casiofront_read_eeprom_4cell_info(ctx);
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
 	i2c_table_write(ctx, casiofront_sensor_preinit_setting, ARRAY_SIZE(casiofront_sensor_preinit_setting));
	mdelay(3);
	casiofront_sensor_init(ctx);

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

static void casiofront_sensor_init(struct subdrv_ctx *ctx)
{
    casiofront_table_write_cmos_sensor(ctx, casiofront_sensor_init_setting_array1, ARRAY_SIZE(casiofront_sensor_init_setting_array1));
    casiofront_burst_write_cmos_sensor(ctx, 0x6F12, (kal_uint16 *)casiofront_sensor_init_setting_array2, ARRAY_SIZE(casiofront_sensor_init_setting_array2));
    casiofront_table_write_cmos_sensor(ctx, casiofront_sensor_init_setting_array3, ARRAY_SIZE(casiofront_sensor_init_setting_array3));
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
	return gain * 32 / BASEGAIN;
}

static int casiofront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode) {
		if (mode == 5) {
			subdrv_i2c_wr_u16(ctx, 0x0600, 0x0001); /*black*/
		} else {
			subdrv_i2c_wr_u16(ctx, 0x0600, mode); /*100% Color bar*/
		}
	}
	else if (ctx->test_pattern)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/

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

	// struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	/* Probe EEPROM device */
	if (!probe_eeprom(ctx))
		return;

	ctx->is_read_preload_eeprom = 1;
}

static void set_sensor_cali(void *arg)
{
	//struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	return;
}

int casiofront_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[0].addr[0],
		(rg_gain >> 8) & 0xFF);
	set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[0].addr[1],
		rg_gain & 0xFF);
	DRV_LOG(ctx, "gain[0x%x]\n", rg_gain);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	commit_i2c_buffer(ctx);
	/* group hold end */
	return 0;
}

static void casiofront_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
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
		subdrv_i2c_wr_u16(ctx, 0x0702, 0x0000);
		subdrv_i2c_wr_u16(ctx, 0x0704, 0x0000);
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

static int casiofront_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	casiofront_set_multi_shutter_frame_length(ctx, (u32 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

static void casiofront_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
	int i = 0;
	u32 values[3] = {0};

	if (shutters != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u32) *(shutters + i);
	}
	casiofront_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

static int casiofront_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	casiofront_set_hdr_tri_shutter(ctx, feature_data, 2);
	return 0;
}

static int casiofront_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	casiofront_set_hdr_tri_shutter(ctx, feature_data, 3);
	return 0;
}

static char four_cell_data[FOUR_CELL_SIZE + 2] = {0};
static int casiofront_get_eeprom_4cell_info(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u8 *data = (char *)(uintptr_t)(*(feature_data + 1));
	u16 type = (u16)(*feature_data);
	if (type == FOUR_CELL_CAL_TYPE_XTALK_CAL){
		DRV_LOG(ctx, "memcpy xtalk data!\n");
		memcpy(data, four_cell_data, FOUR_CELL_SIZE + 2);
	}
	// dump
	// for (int i = 0; i < FOUR_CELL_SIZE + 2; i++) {
	// 	LOG_INF("four_cell_data[%d] = 0x%02x\n", i, four_cell_data[i]);
	// }
	return 0;
}

static void casiofront_read_eeprom_4cell_info(struct subdrv_ctx *ctx)
{
	four_cell_data[0] = (FOUR_CELL_SIZE & 0xFF); /*Low*/
	four_cell_data[1] = ((FOUR_CELL_SIZE >> 8) & 0xFF); /*High*/

	if (!read_cmos_eeprom_p8(ctx, FOUR_CELL_ADDR, &four_cell_data[2], FOUR_CELL_SIZE)) {
		DRV_LOGE(ctx, "read SXTC_DATA fail!");
	}
}
