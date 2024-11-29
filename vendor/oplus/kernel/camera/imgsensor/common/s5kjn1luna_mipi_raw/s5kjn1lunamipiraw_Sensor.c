// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5kjn1lunamipiraw_Sensor.c
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
#include "s5kjn1lunamipiraw_Sensor.h"

#define S5KJN1LUNA_EEPROM_READ_ID	0xA2
#define S5KJN1LUNA_EEPROM_WRITE_ID 0xA3
#define S5KJN1LUNA_MAX_OFFSET		0x8000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "s5kjn1luna_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x8000
#define OTP_HW_GGC_SIZE       346
#define OTP_HW_GGC_ADDR       0x3DA0
#define OTP_HW_GGC_VALID_ADDR 0x3EFA
static u8 hwggc_is_valid = 0;
static u8 hwggc_setting[OTP_HW_GGC_SIZE];

static bool bNeedSetNormalMode = FALSE;
static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int s5kjn1luna_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int s5kjn1luna_get_cust_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_sensor_temperature(void *arg);
static void get_sensor_cali(void* arg);
static void set_sensor_cali(void *arg);
static int s5kjn1luna_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1luna_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);

/* STRUCT */

static BYTE s5kjn1luna_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };

static struct eeprom_map_info s5kjn1luna_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C1, 0x00C2, 17, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x4C80, 0x0000, 0x0000, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x2B00, 0x3199, 0x319A, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0x31C0, 0x3859, 0x385A, CALI_DATA_MASTER_LENGTH, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5kjn1luna_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, s5kjn1luna_set_test_pattern_data},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, s5kjn1luna_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, s5kjn1luna_get_sensor_sn},
	{SENSOR_FEATURE_SET_SENSOR_OTP, s5kjn1luna_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, s5kjn1luna_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, s5kjn1luna_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, s5kjn1luna_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, s5kjn1luna_streaming_resume},
	{SENSOR_FEATURE_SET_ESHUTTER, s5kjn1luna_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, s5kjn1luna_set_shutter_frame_length},
	{SENSOR_FEATURE_GET_CUST_PIXEL_RATE, s5kjn1luna_get_cust_pixel_rate},
	{SENSOR_FEATURE_SET_GAIN, s5kjn1luna_set_gain},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, s5kjn1luna_set_hdr_tri_shutter2},
	{SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, s5kjn1luna_set_hdr_tri_shutter3},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, s5kjn1luna_set_multi_shutter_frame_length_ctrl},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x005A0006,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA2,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX =  8,
	.i4OffsetY =  8,
	.i4PitchX  =  8,
	.i4PitchY  =  8,
	.i4PairNum  = 4,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{9, 8},{11, 11},{15, 12},{13, 15}},
	.i4PosR = {{8, 8},{10, 11},{14, 12},{12, 15}},
	.i4BlockNumX = 504, // Tail_X / (i4PitchX / i4SubBlkW)
	.i4BlockNumY = 382, // Tail_Y / 2 / (i4PitchY / i4SubBlkH)
	.iMirrorFlip = 3,
	// i4Crop = (fullRaw - imgSz) / 2 / Bin
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 388}, {120, 456}, {0, 0},
		{408, 312}, {0, 388}, {0, 0}, {1240, 936}
	},
	.i4FullRawW = 4080,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	//.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1, 0},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
	.i4OffsetX =  8,
	.i4OffsetY =  8,
	.i4PitchX  =  8,
	.i4PitchY  =  8,
	.i4PairNum  = 4,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{9, 8},{11, 11},{15, 12},{13, 15}},
	.i4PosR = {{8, 8},{10, 11},{14, 12},{12, 15}},
	.i4BlockNumX = 504,
	.i4BlockNumY = 286,
	.iMirrorFlip = 3,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 388}, {120, 456}, {0, 0},
		{408, 312}, {0, 388}, {0, 0}, {1240, 936}
	},
	.i4FullRawW = 4080, // out_size = 4080x2296
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	//.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1, 0},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_hs_video = {
	.i4OffsetX =  8,
	.i4OffsetY =  8,
	.i4PitchX  =  8,
	.i4PitchY  =  8,
	.i4PairNum  = 4,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{9, 8},{11, 11},{15, 12},{13, 15}},
	.i4PosR = {{8, 8},{10, 11},{14, 12},{12, 15}},
	.i4BlockNumX = 480,
	.i4BlockNumY = 268,
	.iMirrorFlip = 3,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 388}, {120, 456}, {0, 0},
		{408, 312}, {0, 388}, {0, 0}, {1240, 936}
	},
	.i4FullRawW = 4080, // out_size = 3840x2160
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	//.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1, 0 },
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_cust1 = {
	.i4OffsetX =  8,
	.i4OffsetY =  8,
	.i4PitchX  =  8,
	.i4PitchY  =  8,
	.i4PairNum  = 4,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{9, 8},{11, 11},{15, 12},{13, 15}},
	.i4PosR = {{8, 8},{10, 11},{14, 12},{12, 15}},
	.i4BlockNumX = 408,
	.i4BlockNumY = 304,
	.iMirrorFlip = 3,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 388}, {120, 456}, {0, 0},
		{408, 312}, {0, 388}, {0, 0}, {1240, 936}
	},
	.i4FullRawW = 4080, // 3264x2448
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	//.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1, 0 },
	},
};

// static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_cust4 = {
// 	.i4OffsetX =  8,
// 	.i4OffsetY =  8,
// 	.i4PitchX  =  8,
// 	.i4PitchY  =  8,
// 	.i4PairNum  = 4,
// 	.i4SubBlkW  = 8,
// 	.i4SubBlkH  = 2,
// 	.i4PosL = {{9, 8},{11, 11},{15, 12},{13, 15}},
// 	.i4PosR = {{8, 8},{10, 11},{14, 12},{12, 15}},
// 	.i4BlockNumX = 400,
// 	.i4BlockNumY = 74,
// 	.iMirrorFlip = 0,
// 	.i4Crop = {
// 		{0, 0}, {0, 0}, {0, 388}, {120, 456}, {0, 0},
// 		{408, 312}, {0, 388}, {0, 0}, {1240, 936}
// 	},
// 	.i4FullRawW = 4080, // out_size = 1600x1200
// 	.i4FullRawH = 3072,
// 	.i4VCPackNum = 1,
// 	//.i4ModeIndex = 0,
// 	.sPDMapInfo[0] = {
// 		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
// 		.i4PDPattern = 3,
// 		.i4PDRepetition = 2,
// 		.i4PDOrder = {1, 0},
// 	},
// };

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 504,
			.vsize = 3056,
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
			.hsize = 4080, //0xff0
			.vsize = 3072, //0xc00
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 504,
			.vsize = 3056,
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
			.hsize = 4080,
			.vsize = 2296,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 504,
			.vsize = 2288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3840,
			.vsize = 2160,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 480,
			.vsize = 2144,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 504,
			.vsize = 3056,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
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
			.hsize = 408,
			.vsize = 2432,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 4080,
			.vsize = 2296,
			.user_data_desc = VC_RAW_PROCESSED_DATA, //processed raw
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x12,
			.hsize = 1024,	// 1024 pixels data width (embedded data by byte)
			.vsize = 568,	// 568 data height
			.user_data_desc = VC_GENERAL_EMBEDDED,  // 3a_meta
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2e,
			.hsize = 504,
			.vsize = 2288,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 4080,
			.vsize = 2296,
			.user_data_desc = VC_RAW_PROCESSED_DATA, //processed raw
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 2296,
			.user_data_desc = VC_RAW_DATA,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x12,
			.hsize = 1024,	// 1024 pixels data width (embedded data by byte)
			.vsize = 568,	// 568 data height
			.user_data_desc = VC_GENERAL_EMBEDDED,  // 3a_meta
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2e,
			.hsize = 504,
			.vsize = 2288,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	// {
	// 	.bus.csi2 = {
	// 		.channel = 0,
	// 		.data_type = 0x30,
	// 		.hsize = 400,
	// 		.vsize = 596,
	// 		.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
	// 		.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
	// 	},
	// },
};
#endif

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = s5kjn1luna_preview_setting,
		.mode_setting_len = ARRAY_SIZE(s5kjn1luna_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 5632,
		.framelength = 3312,
		.max_framerate = 300,
		//.mipi_pixel_rate = 508800000,
		.mipi_pixel_rate = 544068000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4080,
			.full_h = 3072,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4080,
			.h0_size = 3072,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = s5kjn1luna_capture_setting,
		.mode_setting_len = ARRAY_SIZE(s5kjn1luna_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 5632,
		.framelength = 3312,
		.max_framerate = 300,
		//.mipi_pixel_rate = 508800000,
		.mipi_pixel_rate = 544068000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4080,
			.full_h = 3072,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4080,
			.h0_size = 3072,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = s5kjn1luna_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(s5kjn1luna_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.max_framerate = 300,
		//.mipi_pixel_rate = 508800000,
		.mipi_pixel_rate = 544068000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4080,
			.full_h = 3072,
			.x0_offset = 0,
			.y0_offset = 388,
			.w0_size = 4080,
			.h0_size = 2296,
			.scale_w = 4080,
			.scale_h = 2296,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2296,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = s5kjn1luna_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(s5kjn1luna_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 4096,
		.framelength = 2274,
		.max_framerate = 600,
		.mipi_pixel_rate = 724800000,
		// .mipi_pixel_rate = 776260800,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4080,
			.full_h = 3072,
			.x0_offset = 120,
			.y0_offset = 456,
			.w0_size = 3840,
			.h0_size = 2160,
			.scale_w = 3840,
			.scale_h = 2160,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3840,
			.h1_size = 2160,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3840,
			.h2_tg_size = 2160,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_hs_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = s5kjn1luna_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(s5kjn1luna_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3952,
		.max_framerate = 240,
		// .mipi_pixel_rate = 508800000,
		.mipi_pixel_rate = 544068000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4080,
			.full_h = 3072,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4080,
			.h0_size = 3072,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = s5kjn1luna_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(s5kjn1luna_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 6624,
		.framelength = 3520,
		.max_framerate = 240,
		//.mipi_pixel_rate = 432000000,
		.mipi_pixel_rate = 462672000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4080,
			.full_h = 3072,
			.x0_offset = 408,
			.y0_offset = 312,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_cust1,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = s5kjn1luna_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(s5kjn1luna_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.max_framerate = 300,
		//.mipi_pixel_rate = 508800000,
		.mipi_pixel_rate = (2800*3*16/7/10*1000000),
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4080,
			.full_h = 3072,
			.x0_offset = 0,
			.y0_offset = 388,
			.w0_size = 4080,
			.h0_size = 2296,
			.scale_w = 4080,
			.scale_h = 2296,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2296,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = s5kjn1luna_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(s5kjn1luna_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.max_framerate = 300,
		//.mipi_pixel_rate = 508800000,
		.mipi_pixel_rate = (2800*3*16/7/10*1000000),
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4080,
			.full_h = 3072,
			.x0_offset = 0,
			.y0_offset = 388,
			.w0_size = 4080,
			.h0_size = 2296,
			.scale_w = 4080,
			.scale_h = 2296,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2296,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = s5kjn1luna_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(s5kjn1luna_custom4_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 600000000,
		.linelength = 10848,
		.framelength = 2304,
		.max_framerate = 240,
		.mipi_pixel_rate = 320000000,
		// .mipi_pixel_rate = 342720000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4080,
			.full_h = 3072,
			.x0_offset = 440,
			.y0_offset = 336,
			.w0_size = 3200,
			.h0_size = 2400,
			.scale_w = 1600,
			.scale_h = 1200,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1600,
			.h1_size = 1200,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1600,
			.h2_tg_size = 1200,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = S5KJN1LUNA_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x5a, 0xff},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {4080, 3072},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 2, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 2,
	.ana_gain_table = s5kjn1luna_ana_gain_table,
	.ana_gain_table_size = sizeof(s5kjn1luna_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xffff * 128) - 4,
	.exposure_step = 1,
	.exposure_margin = 10,

	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.g_cali = get_sensor_cali,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = 0x0020,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = s5kjn1luna_sensor_init_setting,
	.init_setting_len =  ARRAY_SIZE(s5kjn1luna_sensor_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	//.chk_s_off_before_s_on = 1,
	//.chk_s_off_before_control = 1,
	//.chk_s_off_after_s_off = 0,

	.checksum_value = 0x350174bc,
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
	{HW_ID_DOVDD, 1804000, 0},
	//{HW_ID_DVDD, 1050000, 2},
	{HW_ID_DVDD, 1080000, 1},
	{HW_ID_AVDD, 2804000, 1},
	{HW_ID_AFVDD, 2804000, 1},
	{HW_ID_RST, 1, 2},
	//{HW_ID_MCLK, 24, 10},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 10},
};

const struct subdrv_entry s5kjn1luna_mipi_raw_entry = {
	.name = "s5kjn1luna_mipi_raw",
	.id = S5KJN1LUNA_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature >= 0x0 && temperature <= 0x78)
		temperature_convert = temperature;
	else
		temperature_convert = -1;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

static void s5kjn1luna_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	/* return; //for test */
	subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
	subdrv_i2c_wr_u16(ctx, 0x0342, ctx->line_length);
}				/*      set_dummy  */

static void s5kjn1luna_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	s5kjn1luna_set_dummy(ctx);
}

static void s5kjn1luna_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 *shutter, u32 frame_length, bool auto_extend_en)
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
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			s5kjn1luna_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			s5kjn1luna_set_max_framerate(ctx, 146, 0);
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

static int s5kjn1luna_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "shutter:%u, frame_length:%u\n", (u32)(*para), (u32) (*(para + 1)));
	s5kjn1luna_set_shutter_frame_length_convert(ctx, (u32 *)para, (u32) (*(para + 1)), (u16) (*(para + 2)));
	return 0;
}

static void s5kjn1luna_write_shutter(struct subdrv_ctx *ctx)
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
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			s5kjn1luna_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			s5kjn1luna_set_max_framerate(ctx, 146, 0);
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

		if (ctx->exposure[0] >= 3448275) {  //>32s
			ctx->exposure[0] = 3448275;
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

static void s5kjn1luna_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	s5kjn1luna_write_shutter(ctx);
}

static int s5kjn1luna_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *para);
	s5kjn1luna_set_shutter_convert(ctx, (u32 *)para);
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
		//if (ctx->s_ctx.chk_s_off_before_s_on) {
		//	DRV_LOG(ctx, "check_stream_off before stream on");
		//	check_stream_off(ctx);
		//}
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
		//if (ctx->s_ctx.chk_s_off_after_s_off)
			check_stream_off(ctx);
	}
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "enable:%u\n", enable);
}

static int s5kjn1luna_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			s5kjn1luna_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int s5kjn1luna_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
		return 0;
}

static int s5kjn1luna_get_cust_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
       u64 *feature_data = (u64 *) para;
       switch (*feature_data) {
                case SENSOR_SCENARIO_ID_CUSTOM2:
                        *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (2800*3*16/7/10*1000000);
                        break;
                case SENSOR_SCENARIO_ID_CUSTOM3:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1307200000;
                        break;
                default:
                       break;
       }
       return 0;
}

static unsigned int read_s5kjn1luna_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != s5kjn1luna_eeprom_info[meta_id].meta)
		return -1;

	if (size != s5kjn1luna_eeprom_info[meta_id].size)
		return -1;

	addr = s5kjn1luna_eeprom_info[meta_id].start;
	readsize = s5kjn1luna_eeprom_info[meta_id].size;

	if (!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static void read_module_data(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;

	read_s5kjn1luna_eeprom_info(ctx, EEPROM_META_MODULE_ID,
		&(s5kjn1luna_common_data[0]), 2);
	// ctx->s_ctx.module_id = (kal_uint16)(s5kjn1luna_common_data[1] << 8) |
	// 	s5kjn1luna_common_data[0];
	read_s5kjn1luna_eeprom_info(ctx, EEPROM_META_SENSOR_ID,
		&(s5kjn1luna_common_data[2]), 2);
	read_s5kjn1luna_eeprom_info(ctx, EEPROM_META_LENS_ID,
		&(s5kjn1luna_common_data[4]), 2);
	read_s5kjn1luna_eeprom_info(ctx, EEPROM_META_VCM_ID,
		&(s5kjn1luna_common_data[6]), 2);
	read_s5kjn1luna_eeprom_info(ctx, EEPROM_META_MODULE_SN,
		&(s5kjn1luna_common_data[8]), 17);
	read_s5kjn1luna_eeprom_info(ctx, EEPROM_META_AF_CODE,
		&(s5kjn1luna_common_data[25]), 6);
	read_s5kjn1luna_eeprom_info(ctx, EEPROM_META_AF_FLAG,
		&(s5kjn1luna_common_data[33]), 1);

	for (idx = 0; idx < 36; idx = idx + 4)
		DRV_LOG(ctx, "otp data1: %02x %02x %02x %02x\n", s5kjn1luna_common_data[idx],
			s5kjn1luna_common_data[idx + 1], s5kjn1luna_common_data[idx + 2],
			s5kjn1luna_common_data[idx + 3]);

	s5kjn1luna_common_data[25] = (kal_uint8)((s5kjn1luna_common_data[26] << 6) | (s5kjn1luna_common_data[25] >> 2));
	s5kjn1luna_common_data[26] = (kal_uint8)(s5kjn1luna_common_data[26] >> 2);
	s5kjn1luna_common_data[27] = (kal_uint8)((s5kjn1luna_common_data[28] << 6) | (s5kjn1luna_common_data[27] >> 2));
	s5kjn1luna_common_data[28] = (kal_uint8)(s5kjn1luna_common_data[28] >> 2);
	s5kjn1luna_common_data[29] = (kal_uint8)((s5kjn1luna_common_data[30] << 6) | (s5kjn1luna_common_data[29] >> 2));
	s5kjn1luna_common_data[30] = (kal_uint8)(s5kjn1luna_common_data[30] >> 2);

	for (idx = 0; idx < 36; idx = idx + 4)
		DRV_LOG(ctx, "otp data2: %02x %02x %02x %02x\n", s5kjn1luna_common_data[idx],
			s5kjn1luna_common_data[idx + 1], s5kjn1luna_common_data[idx + 2],
			s5kjn1luna_common_data[idx + 3]);

}

static int s5kjn1luna_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOG(ctx, "get otp data");
	memcpy(feature_return_para_32, s5kjn1luna_common_data,
		OPLUS_CAMERA_COMMON_DATA_LENGTH);
	*len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
	return 0;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
    kal_uint16 get_byte = 0;

    adaptor_i2c_rd_u8(ctx->i2c_client, S5KJN1LUNA_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
    return get_byte;
}

#define   WRITE_DATA_MAX_LENGTH     (16)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, S5KJN1LUNA_EEPROM_WRITE_ID >> 1,
            addr, para, WRITE_DATA_MAX_LENGTH);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xE000;
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, S5KJN1LUNA_EEPROM_WRITE_ID >> 1, reg, 0xA3);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, S5KJN1LUNA_EEPROM_WRITE_ID >> 1, reg, 0xA2);
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
    if(pStereodata != NULL) {
        LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
            pStereodata->uSensorId,
            pStereodata->uDeviceId,
            pStereodata->baseAddr,
            pStereodata->dataLength);

        data_base = pStereodata->baseAddr;
        data_length = pStereodata->dataLength;
        pData = pStereodata->uData;
        if ((pStereodata->uSensorId == S5KJN1LUNA_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == S5KJN1LUNA_STEREO_START_ADDR)) {
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);
            idx = data_length/WRITE_DATA_MAX_LENGTH;
            idy = data_length%WRITE_DATA_MAX_LENGTH;
            /* close write protect */
            write_eeprom_protect(ctx, 0);
            msleep(6);
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
        } else if ((pStereodata->uSensorId == S5KJN1LUNA_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == S5KJN1LUNA_AESYNC_START_ADDR)) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
            idx = data_length/WRITE_DATA_MAX_LENGTH;
            idy = data_length%WRITE_DATA_MAX_LENGTH;
            /* close write protect */
            write_eeprom_protect(ctx, 0);
            msleep(6);
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
                read_cmos_eeprom_8(ctx, S5KJN1LUNA_AESYNC_START_ADDR),
                read_cmos_eeprom_8(ctx, S5KJN1LUNA_AESYNC_START_ADDR+1),
                read_cmos_eeprom_8(ctx, S5KJN1LUNA_AESYNC_START_ADDR+2),
                read_cmos_eeprom_8(ctx, S5KJN1LUNA_AESYNC_START_ADDR+3),
                read_cmos_eeprom_8(ctx, S5KJN1LUNA_AESYNC_START_ADDR+4),
                read_cmos_eeprom_8(ctx, S5KJN1LUNA_AESYNC_START_ADDR+5),
                read_cmos_eeprom_8(ctx, S5KJN1LUNA_AESYNC_START_ADDR+6),
                read_cmos_eeprom_8(ctx, S5KJN1LUNA_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("s5kjn1luna write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int s5kjn1luna_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        LOG_INF("ret=%d\n", ret);
    }
    return 0;
}

static int s5kjn1luna_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	if(*len > CALI_DATA_SLAVE_LENGTH) {
		*len = CALI_DATA_SLAVE_LENGTH;
	}
	read_s5kjn1luna_eeprom_info(ctx, EEPROM_META_STEREO_DATA,
			(BYTE *)para, *len);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, S5KJN1LUNA_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "jn1 read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "jn1 read_otp_info end\n");
}

static int s5kjn1luna_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int s5kjn1luna_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
			if (*sensor_id == S5KJN1_SENSOR_ID) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_module_data(ctx);
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

	/* initail setting */
	subdrv_i2c_wr_u16(ctx, 0x6028, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x0003);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x38E1);
	subdrv_i2c_wr_u16(ctx, 0x001E, 0x0007);
	subdrv_i2c_wr_u16(ctx, 0x6028, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x6010, 0x0001);
	mdelay(5);  //delay 5ms
	subdrv_i2c_wr_u16(ctx, 0x6226, 0x0001);
	mdelay(10);  //delay 10ms
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
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return gain * 32 / BASEGAIN;
}

static int s5kjn1luna_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int s5kjn1luna_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_R >> 22) & 0x3ff;
	u16 Gb = (data->Channel_R >> 22) & 0x3ff;
	u16 B = (data->Channel_R >> 22) & 0x3ff;

	subdrv_i2c_wr_u16(ctx, 0x0602, R);
	subdrv_i2c_wr_u16(ctx, 0x0604, Gr);
	subdrv_i2c_wr_u16(ctx, 0x0606, Gb);
	subdrv_i2c_wr_u16(ctx, 0x0608, B);

	DRV_LOG(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);
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

	int i = 0;
	// struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	/* Probe EEPROM device */
	if (!probe_eeprom(ctx))
		return;

	hwggc_is_valid = i2c_read_eeprom(ctx, OTP_HW_GGC_VALID_ADDR);
	if (hwggc_is_valid != 0x01) {
		DRV_LOGE(ctx, "HWGGC data is invalid, flag(%02x)", hwggc_is_valid);
	} else {
		if (!read_cmos_eeprom_p8(ctx, OTP_HW_GGC_ADDR, hwggc_setting, OTP_HW_GGC_SIZE)) {
			DRV_LOGE(ctx, "preload HWGGC data failed");
		} else {
			DRV_LOG(ctx, "preload HWGGC data %u bytes", OTP_HW_GGC_SIZE);
		}
	}

	for(i = 0; i < OTP_HW_GGC_SIZE; ++i) {
		DRV_LOG(ctx, "HWGGC[%04x] = %02x", OTP_HW_GGC_ADDR + i, hwggc_setting[i]);
	}

	ctx->is_read_preload_eeprom = 1;
}

static void set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u8* pbuf = &hwggc_setting[0];

	if (hwggc_is_valid) {
		subdrv_i2c_wr_u16(ctx, 0x6028, 0x2400);
		subdrv_i2c_wr_u16(ctx, 0x602A, 0x0CFC);
		subdrv_i2c_wr_p8(ctx, 0x6F12, pbuf, OTP_HW_GGC_SIZE);
		DRV_LOG(ctx, "set HWGGC calibration data done.");
	} else {
		DRV_LOGE(ctx, "HWGGC calibration data error");
	}
}

int s5kjn1luna_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static void s5kjn1luna_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
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

static int s5kjn1luna_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	s5kjn1luna_set_multi_shutter_frame_length(ctx, (u32 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

static void s5kjn1luna_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
	int i = 0;
	u32 values[3] = {0};

	if (shutters != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u32) *(shutters + i);
	}
	s5kjn1luna_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

static int s5kjn1luna_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	s5kjn1luna_set_hdr_tri_shutter(ctx, feature_data, 2);
	return 0;
}

static int s5kjn1luna_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	s5kjn1luna_set_hdr_tri_shutter(ctx, feature_data, 3);
	return 0;
}
