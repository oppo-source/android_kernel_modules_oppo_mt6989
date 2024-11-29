// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 omegas3frontmipiraw_Sensor.c
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
#include "omegas3frontmipiraw_Sensor.h"

#define OMEGAS3FRONT_EEPROM_READ_ID	0xA9
#define OMEGAS3FRONT_EEPROM_WRITE_ID   0xA8
#define OMEGAS3FRONT_MAX_OFFSET		0x4000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "omegas3front_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x4000
#define OTP_QCOM_PDAF_DATA_LENGTH 0x468
#define OTP_QCOM_PDAF_DATA_START_ADDR 0x600

static bool bNeedSetNormalMode = FALSE;
static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static kal_uint8 otp_qcom_pdaf_data[OTP_QCOM_PDAF_DATA_LENGTH] = {0};
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int omegas3front_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int get_sensor_temperature(void *arg);
static void get_sensor_cali(void* arg);
static void set_sensor_cali(void *arg);
static int omegas3front_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int omegas3front_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3front_i2c_burst_wr_regs_u16(struct subdrv_ctx *ctx, u16 * list, u32 len);
static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx,
		u16 addr, u16 *list, u32 len);

/* STRUCT */

static struct eeprom_map_info omegas3front_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8, 23, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x51F0, 0x0000, 0x0000, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x2B00, 0x3199, 0x319A, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0x31C0, 0x3859, 0x385A, CALI_DATA_MASTER_LENGTH, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, omegas3front_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, omegas3front_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, omegas3front_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, omegas3front_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, omegas3front_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, omegas3front_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, omegas3front_streaming_resume},
	{SENSOR_FEATURE_SET_ESHUTTER, omegas3front_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, omegas3front_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_GAIN, omegas3front_set_gain},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, omegas3front_set_hdr_tri_shutter2},
	{SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, omegas3front_set_hdr_tri_shutter3},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, omegas3front_set_multi_shutter_frame_length_ctrl},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_DATA, omegas3front_get_otp_qcom_pdaf_data},
	{SENSOR_FEATURE_SET_AWB_GAIN, omegas3front_set_awb_gain},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, omegas3front_get_min_shutter_by_scenario_adapter},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01A8010A,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA8,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX =  0,
	.i4PitchY =  0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.iMirrorFlip = 0,
	// i4Crop = (fullRaw - imgSz) / 2 / Bin
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
		/* <cust1> <cust2> <cust3> <cust4> <cust5> */
		{0, 0}, {0, 0}, {0, 0}, {0, 384}, {0, 0},
		/* <cust6> <cust7> <cust8> <cust9> <cust10>*/
		{0, 0}, {0, 384}, {0, 384}, {0, 0}, {416, 618},
		/* <cust11> <cust12>*/
		{416, 618}, {416, 312},
	},
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x3,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,//all-pd
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, // R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_v2h2 = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX =  0,
	.i4PitchY =  0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.iMirrorFlip = 0,
	// i4Crop = (fullRaw - imgSz) / 2 / Bin
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 192},
		/* <cust1> <cust2> <cust3> <cust4> <cust5> */
		{0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cust6> <cust7> <cust8> <cust9> <cust10>*/
		{0, 192}, {0, 192}, {0, 0}, {0, 192}, {0, 0},
	},
	.i4FullRawW = 2048,
	.i4FullRawH = 1536,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x3,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,//all-pd
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, // R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_full = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX =  0,
	.i4PitchY =  0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.iMirrorFlip = 0,
	// i4Crop = (fullRaw - imgSz) / 2 / Bin
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
		/* <cust1> <cust2> <cust3> <cust4> <cust5> */
		{0, 384}, {0, 0}, {0, 0}, {0, 0}, {2048, 1536},
		/* <cust6> <cust7> <cust8> <cust9> <cust10>*/
		{0, 0}, {0, 384}, {0, 384}, {0, 0}, {0, 0},
	},
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x3,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,//all-pd
		.i4BinFacX = 4,
		.i4BinFacY = 8,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, // R=1, L=0
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
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
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
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
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
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
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
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
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
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
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 8192,
			.vsize = 6144,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096, //uint:Byte, width*10/8
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	/*{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},*/
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1836,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 3264,
			.vsize = 458,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus11[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1836,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 3264,
			.vsize = 458,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus12[] = {
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
			.hsize = 3264,
			.vsize = 612,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus13[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = omegas3front_preview_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = omegas3front_capture_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = omegas3front_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608 ,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = omegas3front_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 3204,
		.max_framerate = 600,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608 ,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 60,
		},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = omegas3front_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 1602,
		.max_framerate = 1200,
		.mipi_pixel_rate = 668800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 120,
		},
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = omegas3front_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 8008,
		.max_framerate = 240,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},
	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = omegas3front_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 668800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 2048,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = omegas3front_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 9600,
		.framelength = 6346,
		.max_framerate = 150,
		.mipi_pixel_rate = 902400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 16,
		.exposure_margin = 48,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = RMSC_MASK,
			.equivalent_fps = 15,
		},
	},
	{
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = omegas3front_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom4_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 6000,
		.framelength = 5110,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 2,
		.min_exposure_line = 8,
		.exposure_margin = 24,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = HDR_RAW_STAGGER_2EXP_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = omegas3front_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom5_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 9200,
		.framelength = 3332,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 16,
		.exposure_margin = 48,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1536,
			.w0_size = 8192,
			.h0_size = 3072,
			.scale_w = 8192,
			.scale_h = 3072,
			.x1_offset = 2048,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = INSENSORZOOM_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = omegas3front_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = omegas3front_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom7_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 2944,
		.framelength = 1302,
		.max_framerate = 2400,
		.mipi_pixel_rate = 796800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 240,
		},
	},
	{
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = omegas3front_custom8_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom8_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 12816,
		.max_framerate = 150,
		.mipi_pixel_rate = 668800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 2048,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = omegas3front_custom9_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom9_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4852,
		.framelength = 12640,
		.max_framerate = 150,
		.mipi_pixel_rate = 668800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_cus10,
		.num_entries = ARRAY_SIZE(frame_desc_cus10),
		.mode_setting_table = omegas3front_custom10_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom10_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4848,
		.framelength = 6309,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 416,
			.y1_offset = 234,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_cus11,
		.num_entries = ARRAY_SIZE(frame_desc_cus11),
		.mode_setting_table = omegas3front_custom11_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom11_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4848,
		.framelength = 12640,
		.max_framerate = 150,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 416,
			.y1_offset = 234,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_cus12,
		.num_entries = ARRAY_SIZE(frame_desc_cus12),
		.mode_setting_table = omegas3front_custom12_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom12_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4848,
		.framelength = 6312,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 416,
			.y1_offset = 312,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_cus13,
		.num_entries = ARRAY_SIZE(frame_desc_cus13),
		.mode_setting_table = omegas3front_custom13_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3front_custom13_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 2944,
		.framelength = 10412,
		.max_framerate = 300,
		.mipi_pixel_rate = 312000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 684,
			.y1_offset = 320,
			.w1_size = 680,
			.h1_size = 512,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 680,
			.h2_tg_size = 512,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 120,
		},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OMEGAS3FRONT_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x20, 0xff},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 80,
	.ana_gain_type = 2, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 2,
	.ana_gain_table = omegas3front_ana_gain_table,
	.ana_gain_table_size = sizeof(omegas3front_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xffff * 128) - 4,
	.exposure_step = 1,
	.exposure_margin = 10,

	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 929000,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
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

//	.init_setting_table = omegas3front_sensor_init_setting,
//	.init_setting_len =  ARRAY_SIZE(omegas3front_sensor_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
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
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DOVDD, 0, 3},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DOVDD, 0, 3},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD, 1024000, 1},
	{HW_ID_AVDD, 2204000, 1},
	{HW_ID_RST, 1, 2},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 10},
};

static struct subdrv_pw_seq_entry pw_off_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1800000, 0},
	{HW_ID_DVDD, 1024000, 1},
	{HW_ID_AVDD, 2204000, 1},
	{HW_ID_RST, 1, 2},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 10},
};

const struct subdrv_entry omegas3front_mipi_raw_entry = {
	.name = "omegas3front_mipi_raw",
	.id = OMEGAS3FRONT_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.pw_off_seq = pw_off_seq,
	.pw_off_seq_cnt = ARRAY_SIZE(pw_off_seq),
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

static void omegas3front_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	/* return; //for test */
	subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
	subdrv_i2c_wr_u16(ctx, 0x0342, ctx->line_length);
}				/*      set_dummy  */

static void omegas3front_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	omegas3front_set_dummy(ctx);
}

static void omegas3front_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 *shutter, u32 frame_length, bool auto_extend_en)
{
	//kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	u64 CintR = 0;
	u64 Time_Farme = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
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

	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		write_frame_length(ctx, ctx->frame_length);

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

static int omegas3front_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "shutter:%u, frame_length:%u\n", (u32)(*para), (u32) (*(para + 1)));
	omegas3front_set_shutter_frame_length_convert(ctx, (u32 *)para, (u32) (*(para + 1)), (u16) (*(para + 2)));
	return 0;
}

static void omegas3front_write_shutter(struct subdrv_ctx *ctx)
{
	kal_uint16 realtime_fps = 0;
	u64 CintR = 0;
	u64 Time_Farme = 0;
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
		if (realtime_fps > 592 && realtime_fps <= 607) {
			omegas3front_set_max_framerate(ctx, 592, 0);
		} else if (realtime_fps > 296 && realtime_fps <= 305) {
			omegas3front_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 246 && realtime_fps <= 253) {
			omegas3front_set_max_framerate(ctx, 246, 0);
		} else if (realtime_fps > 236 && realtime_fps <= 243) {
			omegas3front_set_max_framerate(ctx, 236, 0);
		} else if (realtime_fps > 146 && realtime_fps <= 153) {
			omegas3front_set_max_framerate(ctx, 146, 0);
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

static void omegas3front_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	omegas3front_write_shutter(ctx);
}

static int omegas3front_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *para);
	omegas3front_set_shutter_convert(ctx, (u32 *)para);
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

static int omegas3front_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			omegas3front_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int omegas3front_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
		return 0;
}

static unsigned int read_omegas3front_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != omegas3front_eeprom_info[meta_id].meta)
		return -1;

	if (size != omegas3front_eeprom_info[meta_id].size)
		return -1;

	addr = omegas3front_eeprom_info[meta_id].start;
	readsize = omegas3front_eeprom_info[meta_id].size;

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
	.addr_modinfoflag = 0x0010,

	.addr_af = 0x0092,
	.addr_afmacro = 0x0092,
	.addr_afinf = 0x0094,
	.addr_afflag = 0x0098,

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

    adaptor_i2c_rd_u8(ctx->i2c_client, OMEGAS3FRONT_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
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
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, OMEGAS3FRONT_EEPROM_WRITE_ID >> 1,
            addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xE000;
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAS3FRONT_EEPROM_WRITE_ID >> 1, reg, 0xA1);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAS3FRONT_EEPROM_WRITE_ID >> 1, reg, 0xA0);
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
        if ((pStereodata->uSensorId == OMEGAS3FRONT_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == OMEGAS3FRONT_STEREO_START_ADDR)) {
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
        } else if ((pStereodata->uSensorId == OMEGAS3FRONT_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == OMEGAS3FRONT_AESYNC_START_ADDR)) {
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
                read_cmos_eeprom_8(ctx, OMEGAS3FRONT_AESYNC_START_ADDR),
                read_cmos_eeprom_8(ctx, OMEGAS3FRONT_AESYNC_START_ADDR+1),
                read_cmos_eeprom_8(ctx, OMEGAS3FRONT_AESYNC_START_ADDR+2),
                read_cmos_eeprom_8(ctx, OMEGAS3FRONT_AESYNC_START_ADDR+3),
                read_cmos_eeprom_8(ctx, OMEGAS3FRONT_AESYNC_START_ADDR+4),
                read_cmos_eeprom_8(ctx, OMEGAS3FRONT_AESYNC_START_ADDR+5),
                read_cmos_eeprom_8(ctx, OMEGAS3FRONT_AESYNC_START_ADDR+6),
                read_cmos_eeprom_8(ctx, OMEGAS3FRONT_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("omegas3front write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int omegas3front_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        *len = (u32)-1; /*write eeprom failed*/
        LOG_INF("ret=%d\n", ret);
    }
    return 0;
}

static int omegas3front_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	if(*len > CALI_DATA_SLAVE_LENGTH) {
		*len = CALI_DATA_SLAVE_LENGTH;
	}
	read_omegas3front_eeprom_info(ctx, EEPROM_META_STEREO_DATA,
			(BYTE *)para, *len);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, OMEGAS3FRONT_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static int omegas3front_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;

	read_cmos_eeprom_p8(ctx, OTP_QCOM_PDAF_DATA_START_ADDR, otp_qcom_pdaf_data, OTP_QCOM_PDAF_DATA_LENGTH);

	memcpy(feature_return_para_32, (UINT32 *)otp_qcom_pdaf_data, sizeof(otp_qcom_pdaf_data));
	*len = sizeof(otp_qcom_pdaf_data);

	return 0;
}

static int omegas3front_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {

	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;

	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D82, awb_gain->ABS_GAIN_R * 2); //red 1024(1x)
	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D86, awb_gain->ABS_GAIN_B * 2); //blue

	LOG_INF("[test] ABS_GAIN_GR(%d) ABS_GAIN_R(%d) ABS_GAIN_B(%d) ABS_GAIN_GB(%d)", awb_gain->ABS_GAIN_GR, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B, awb_gain->ABS_GAIN_GB);
	LOG_INF("[test] 0x0D82(red) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D82));
	LOG_INF("[test] 0x0D84(green) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D84));
	LOG_INF("[test] 0x0D86(blue) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D86));
	return 0;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "jn1 read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "jn1 read_otp_info end\n");
}

static int omegas3front_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int omegas3front_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
			if (*sensor_id == 0x38E5) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
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
	u32 module_info = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	module_info = subdrv_i2c_rd_u16(ctx, 0x0010);
	DRV_LOG(ctx, "write init setting +");
	subdrv_i2c_wr_regs_u16(ctx, omegas3front_sensor_init_pre_setting1, ARRAY_SIZE(omegas3front_sensor_init_pre_setting1));
	subdrv_i2c_wr_regs_u16(ctx, omegas3front_sensor_init_pre_setting2, ARRAY_SIZE(omegas3front_sensor_init_pre_setting2));
	mdelay(5);
	if (module_info == 0x010F || module_info == 0x011F ||((module_info >> 8) == 0x02)){
		DRV_LOG(ctx, "modules with eeprom data, module_info:0x%x: \n",module_info);
		subdrv_i2c_wr_regs_u16(ctx, omegas3front_sensor_simple1_init_setting, ARRAY_SIZE(omegas3front_sensor_simple1_init_setting));
		omegas3front_i2c_burst_wr_regs_u16(ctx, omegas3front_sensor_simple2_init_setting, ARRAY_SIZE(omegas3front_sensor_simple2_init_setting));
		subdrv_i2c_wr_regs_u16(ctx, omegas3front_sensor_simple3_init_setting, ARRAY_SIZE(omegas3front_sensor_simple3_init_setting));
	} else {
		DRV_LOG(ctx, "modules without eeprom data, module_info:0x%x: \n",module_info);
		subdrv_i2c_wr_regs_u16(ctx, omegas3front_sensor_init_setting, ARRAY_SIZE(omegas3front_sensor_init_setting));
	}
	DRV_LOG(ctx, "write init setting -");
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

void omegas3front_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
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
			case HDR_RAW_STAGGER:
			case HDR_NONE:
 			case HDR_RAW_LBMF:
  			case HDR_RAW_DCG_RAW:
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
	}  else {
		DRV_LOG(ctx, "over sensor_mode_num[%d], use default", ctx->s_ctx.sensor_mode_num);
		*exposure_step = ctx->s_ctx.exposure_step;
		*min_shutter = ctx->s_ctx.exposure_min;
	}
	DRV_LOG(ctx, "scenario_id[%d] exposure_step[%llu] min_shutter[%llu]\n", scenario_id, *exposure_step, *min_shutter);
}

int omegas3front_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	omegas3front_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return 0;
}

static int omegas3front_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

int omegas3front_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static void omegas3front_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
		u32 *shutters, u16 exp_cnt,	u16 frame_length)
{
	int i = 0;
	u32 fine_integ_line = 0;
	u16 last_exp_cnt = 1;
	u32 calc_fl[3] = {0};
//	int readout_diff = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32 rg_shutters[3] = {0};
	u32 cit_step = 0;
  	u8 exposure_margin = ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin ? ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin : ctx->s_ctx.exposure_margin;

  	if(exp_cnt == 2) {
  		LOG_INF("shutter[0]:%u, shutter[1]:%u, exp_cnt:%d, frame_length:%u exposure_margin:%d\n",
  			shutters[0], shutters[1], exp_cnt, frame_length, exposure_margin);
  	} else {
  		LOG_INF("shutter[0]:%u, exp_cnt:%d, frame_length:%u, exposure_margin:%d\n",
  			shutters[0], exp_cnt, frame_length, exposure_margin);
  	}

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
//	calc_fl[0] = shutters[0];
//	for (i = 1; i < last_exp_cnt; i++)
//		calc_fl[0] += ctx->exposure[i];
//	calc_fl[0] += ctx->s_ctx.exposure_margin*exp_cnt*exp_cnt;

	/* - (2) current se + current me + current le */
	calc_fl[1] = shutters[0];
	for (i = 1; i < exp_cnt; i++)
		calc_fl[1] += shutters[i];
	calc_fl[1] += exposure_margin*exp_cnt;

//	/* - (3) readout time cannot be overlapped */
//	calc_fl[2] =
//		(ctx->s_ctx.mode[ctx->current_scenario_id].readout_length +
//		ctx->s_ctx.mode[ctx->current_scenario_id].read_margin);
//	if (last_exp_cnt == exp_cnt)
//		for (i = 1; i < exp_cnt; i++) {
//			readout_diff = ctx->exposure[i] - shutters[i];
//			calc_fl[2] += readout_diff > 0 ? readout_diff : 0;
//		}
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

static int omegas3front_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	omegas3front_set_multi_shutter_frame_length(ctx, (u32 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

static void omegas3front_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
	int i = 0;
	u32 values[3] = {0};

	if (shutters != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u32) *(shutters + i);
	}
	omegas3front_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

static int omegas3front_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	omegas3front_set_hdr_tri_shutter(ctx, feature_data, 2);
	return 0;
}

static int omegas3front_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	omegas3front_set_hdr_tri_shutter(ctx, feature_data, 3);
	return 0;
}

static bool dump_i2c_enable = false;

static void dump_i2c_buf(struct subdrv_ctx *ctx, u8 * buf, u32 length)
{
	int i;
	char *out_str = NULL;
	char *strptr = NULL;
	size_t buf_size = SUBDRV_I2C_BUF_SIZE * sizeof(char);
	size_t remind = buf_size;
	int num = 0;

	out_str = kzalloc(buf_size + 1, GFP_KERNEL);
	if (!out_str)
		return;

	strptr = out_str;
	memset(out_str, 0, buf_size + 1);

	num = snprintf(strptr, remind,"[ ");
	remind -= num;
	strptr += num;

	for (i = 0 ; i < length; i ++) {
		num = snprintf(strptr, remind,"0x%02x, ", buf[i]);

		if (num <= 0) {
			DRV_LOG(ctx, "snprintf return negative at line %d\n", __LINE__);
			kfree(out_str);
			return;
		}

		remind -= num;
		strptr += num;

		if (remind <= 20) {
			DRV_LOG(ctx, " write %s\n", out_str);
			memset(out_str, 0, buf_size + 1);
			strptr = out_str;
			remind = buf_size;
		}
	}

	num = snprintf(strptr, remind," ]");
	remind -= num;
	strptr += num;

	DRV_LOG(ctx, " write %s\n", out_str);
	strptr = out_str;
	remind = buf_size;

	kfree(out_str);
}

static int omegas3front_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx, u16 * list, u32 len)
{
	adapter_i2c_burst_wr_regs_u16(ctx, ctx->i2c_write_id >> 1, list, len);
	return 	0;
}

#define MAX_BUF_SIZE  4096
#define MAX_MSG_NUM_U16  MAX_BUF_SIZE/4

struct cache_wr_regs_u16 {
	struct i2c_msg msg[MAX_MSG_NUM_U16];
};

static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx ,
		u16 addr, u16 *list, u32 len)
{
	struct i2c_client *i2c_client = ctx->i2c_client;
	struct cache_wr_regs_u16 *pmem = NULL;
	struct i2c_msg *pmsg = NULL;
	u8 *pbuf = NULL;
	u16 *plist = NULL;
	u16 *plist_end = NULL;
	u8 * msg_buf = NULL;

	u32 sent = 0;
	u32 total = 0;
	u32 cnt = 0;
	int i, j, k;
	int ret;

	pmem = kmalloc(sizeof(*pmem), GFP_KERNEL);
	msg_buf = kmalloc(len * 2 * sizeof(*list), GFP_KERNEL);
	if (!pmem || !msg_buf) {
		if(pmem) {
			kfree(pmem);
		}
		if(msg_buf) {
			kfree(msg_buf);
		}
		LOG_INF("malloc error");
		return -ENOMEM;
	}
	memset(msg_buf, 0 ,len * sizeof(u16));

	/* each msg contains addr(u16) + val(u16 *) */
	sent = 0;
	total = len / 2;
	plist = list;
	plist_end = list + len - 2;

	DRV_LOG(ctx, "[test] len(%u)  total(%u)", len, total);
	pbuf = msg_buf;

	while (sent < total) {

		cnt = 0; //data length
		memset(pmem, 0 ,sizeof(*pmem));
		pmsg = pmem->msg;

		for (i = 0; i < MAX_MSG_NUM_U16; i++) {  //msg count
			pmsg->addr = addr;
			pmsg->flags = i2c_client->flags;
			pmsg->buf = pbuf;

			pbuf[0] = plist[0] >> 8;    //address
			pbuf[1] = plist[0] & 0xff;

			pbuf[2] = plist[1] >> 8;  //data 1
			pbuf[3] = plist[1] & 0xff;

			cnt +=1;
			pmsg->len = 4;
			pbuf += 4;

			for( j = 0;  j <  (MAX_BUF_SIZE - 4 ) / 2 ; j += 2)
			{
				if(plist == plist_end) {  //no next
					break;
				} else {
					if(plist[0] + 2 == plist[2] ) {  //Addresses are consecutive
						pbuf[0] = plist[3] >> 8;
						pbuf[1] = plist[3] & 0xff;

						cnt+=1;
						pbuf += 2;
						pmsg->len += 2;
						plist += 2;
					} else {
						break;
					}
				}
			}
			pbuf ++; //string terminator

			if(dump_i2c_enable) {
			DRV_LOG(ctx, "meg(%d)   pmsg->len(%d), buff: ", i, pmsg->len);
				dump_i2c_buf(ctx, pmsg->buf, pmsg->len);
			}

			if(plist == plist_end) {  //loop end
				i++;
				break;
			}
			plist += 2;
			pmsg++;
		}

		DRV_LOG(ctx, "i2c_transfer  msg_nummer(%d) cnt(%u) sent(%u)  total(%u)", i, cnt, sent, total);
		for(k = 0; k < i; k++)
		{
			ret = i2c_transfer(i2c_client->adapter, &(pmem->msg[k]), 1);

			if (ret < 0) {
				dev_info(&i2c_client->dev,
					"i2c transfer failed (%d)  cnt(%d)  i(%d)  k(%d)\n", ret, cnt, i, k);
				kfree(pmem);
				kfree(msg_buf);
				return -EIO;
			}
		}

		sent += cnt;
	}

	kfree(pmem);
	kfree(msg_buf);

	return 0;
}