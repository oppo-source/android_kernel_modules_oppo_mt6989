// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 casiomain2mipiraw_Sensor.c
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
#include "casiomain2mipiraw_Sensor.h"

#define CASIOMAIN2_EEPROM_READ_ID	0xA1
#define CASIOMAIN2_EEPROM_WRITE_ID	0xA0
#define CASIOMAIN2_MAX_OFFSET		0x8000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "casiomain2_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x8000

#define OTP_QSC_VALID_ADDR    0x2E10
#define QSC_IS_VALID_VAL      0x01
#define SENSOR_QSC_ENABLE_REG 0x3206

#define OTP_PDC_VALID_ADDR    0x2FA0
#define SPC_IS_VALID_VAL      0x01
#define SPC_OTP_ADDR_PART1    0xD200
#define SPC_OTP_ADDR_PART2    0xD300

//#define CASIOMAIN2_UNIQUE_SENSOR_ID 0x0A1F
//#define CASIOMAIN2_UNIQUE_SENSOR_ID_LENGHT 11

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int casiomain2_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//static void casiomain2_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int casiomain2_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void get_sensor_cali(void* arg);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);

static int casiomain2_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomain2_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);

/* STRUCT */

static BYTE casiomain2_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
//static BYTE casiomain2_unique_id[CASIOMAIN2_UNIQUE_SENSOR_ID_LENGHT] = { 0 };

/* Normal(Qbin) to Normal(Qbin) */
/* Normal(Qbin) to 2DOL(Qbin) */
static void comp_mode_tran_time_cal1(struct subdrv_ctx *ctx, u32 pre_scenario_id, u32* prsh);
typedef void (*cal_comp_mode_tran_time)(struct subdrv_ctx *ctx, u32 pre_scenario_id, u32* prsh);
struct comp_mode_tran_time_params {
	u8 enable;
	u32 clock_vtpxck;
	cal_comp_mode_tran_time cal_fn;
};
static struct comp_mode_tran_time_params casiomain2_comp_params[SENSOR_SCENARIO_ID_MAX] = {
	{ .enable = 0, }, /*pre*/
	{ .enable = 1, .clock_vtpxck = 1884, .cal_fn = comp_mode_tran_time_cal1, }, /*cap*/
	{ .enable = 0, }, /*vid*/
	{ .enable = 0, }, /*hvid*/
	{ .enable = 0, }, /*svid*/
	{ .enable = 0, }, /*cus1*/
	{ .enable = 0, }, /*cus2*/
	{ .enable = 0, }, /*csu3*/
};

static struct eeprom_map_info casiomain2_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008,0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8,23, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0x0000, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x2FBA, 0xFFFF, 0xFFFF, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0x3655, 0xFFFF, 0xFFFF, CALI_DATA_MASTER_LENGTH, false }
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, casiomain2_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, casiomain2_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, casiomain2_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_DATA, casiomain2_get_sensor_sn},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, casiomain2_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, casiomain2_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, casiomain2_get_otp_checksum_data},
	//{SENSOR_FEATURE_GET_UNIQUE_SENSORID, casiomain2_get_unique_sensorid},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, casiomain2_get_min_shutter_by_scenario_adapter},
	{SENSOR_FEATURE_SET_AWB_GAIN, casiomain2_set_awb_gain},

	{SENSOR_FEATURE_SET_GAIN, casiomain2_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, casiomain2_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, casiomain2_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, casiomain2_set_multi_shutter_frame_length_ctrl},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01A30115,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA0,

		.lrc_support = TRUE,
		.lrc_size = 384,
		.addr_lrc = 0x2220,
		.sensor_reg_addr_lrc = 0xC800, // useless

		.qsc_support = TRUE,
		.qsc_size = 3072,
		.addr_qsc = 0x2210, //QSC_EEPROM_ADDR
		.sensor_reg_addr_qsc = 0xC000, //QSC_OTP_ADDR

		.pdc_support = TRUE,
		.pdc_size = 384,
		.addr_pdc = 0x2E20, //SPC_EEPROM_ADDR
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 64,
	.i4OffsetY = 16,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{70, 18}, {78, 18}, {66, 22}, {74, 22}, {70, 26}, {78, 26}, {66, 30}, {74, 30}},
	.i4PosR = {{69, 18}, {77, 18}, {65, 22}, {73, 22}, {69, 26}, {77, 26}, {65, 30}, {73, 30}},
	.i4BlockNumX = 248,
	.i4BlockNumY = 190,
	.i4LeFirst = 0,
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {1088, 996},
		/* <cust1> <cust2> <cust3> <cust4> <cust5> */
		{0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cust6> <cust7> <cust8> <cust9> <cust10>*/
		{0, 0}, {0, 384}, {0, 384}, {0, 0}, {0, 0},
	},
	.iMirrorFlip = 0,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,//PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 2,//all-pd
		//.i4BinFacX = 2,
		//.i4BinFacY = 4,
		.i4PDRepetition = 4,
		.i4PDOrder = {1}, //R=1, L=0
	},
};
#if 0
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_v2h2 = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 192},
		/* <cust1> <cust2> <cust3> <cust4> <cust5> */
		{0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cust6> <cust7> <cust8> <cust9> <cust10>*/
		{0, 0}, {0, 384}, {0, 384}, {0, 0}, {0, 0},
	},
	.iMirrorFlip = 0,
	.i4FullRawW = 2048,
	.i4FullRawH = 1536,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,//PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x2,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,//all-pd
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, //R=1, L=0
	},
};
#endif

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_full = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
		/* <cust1> <cust2> <cust3> <cust4> <cust5> */
		{0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cust6> <cust7> <cust8> <cust9> <cust10>*/
		{0, 0}, {0, 384}, {0, 384}, {0, 0}, {0, 0}, {2048, 1536},
	},
	.iMirrorFlip = 0,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,//PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x2,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,//all-pd
		.i4BinFacX = 4,
		.i4BinFacY = 2,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, //R=1, L=0
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
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 992,//4096
			.vsize = 760,//768
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
			.hsize = 4096,//4096
			.vsize = 3072,//3072
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 992,//4096
			.vsize = 760,//768
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
			.hsize = 4096,//4096
			.vsize = 2304,//2304
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 992,//992
			.vsize = 576,//576
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
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    // partial pd
	{
	    .bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 992,
			.vsize = 576,
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
			.hsize = 1920,
			.vsize = 1080,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 960,
			.vsize = 540,
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
			.hsize = 1920,
			.vsize = 1080,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	/*{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},*/
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
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
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 992,
			.vsize = 760,
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

static struct subdrv_mode_struct mode_struct[] = {
    {/*Reg_B_4096x2304_30FPS**/
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = casiomain2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(casiomain2_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 7840,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 6,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 62,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{/*Reg_A_QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max*/
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = casiomain2_capture_setting,
		.mode_setting_len = ARRAY_SIZE(casiomain2_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 7840,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 6,
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
		.ae_binning_ratio = 1,//cc
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 62,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 30,
		},
	},
	{/*Reg_B_4096x2304_30FPS**/
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = casiomain2_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(casiomain2_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 7842,
		.max_framerate = 300,
		.mipi_pixel_rate = 672000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 6,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 62,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
    {/*Reg_B_4096x2304_60FPS**/
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = casiomain2_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(casiomain2_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 100000000,
		.linelength = 325,
		.framelength = 5128,
		.max_framerate = 600,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 6,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 15.5,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 60,
		},
	},
    {/*Reg_B_4096x2304_30FPS**/
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = casiomain2_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(casiomain2_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 100000000,
		.linelength = 325,
		.framelength = 2564,
		.max_framerate = 1200,
		.mipi_pixel_rate = 540000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 8,
		.coarse_integ_step = 8,
		.min_exposure_line = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 256,
			.y0_offset = 912,
			.w0_size = 7680,
			.h0_size = 4320,
			.scale_w = 1920,
			.scale_h = 1080,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 15.5,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 120,
		},
	},
    {/*Reg_B_4096x2304_30FPS**/
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = casiomain2_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(casiomain2_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 100000000,
		.linelength = 325,
		.framelength = 1280,
		.max_framerate = 2400,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 8,
		.coarse_integ_step = 8,
		.min_exposure_line = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 256,
			.y0_offset = 912,
			.w0_size = 7680,
			.h0_size = 4320,
			.scale_w = 1920,
			.scale_h = 1080,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 15.5,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 240,
		},
	},
    {/*Reg_A-1_QBIN(VBIN)_4096x3072_24FPS with PDAF VB_max**/
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = casiomain2_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(casiomain2_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 9804,
		.max_framerate = 240,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 6,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 62,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},
    {/*Reg_B_4096x2304_30FPS**/
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = casiomain2_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(casiomain2_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 100000000,
		.linelength = 700,
		.framelength = 2380,
		.max_framerate = 600,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 10,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 256,
			.y0_offset = 912,
			.w0_size = 7680,
			.h0_size = 4320,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1,
		.fine_integ_line = 359,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 62,
		.sensor_setting_info = {
			.sensor_scenario_usage = RMSC_MASK,
			.equivalent_fps = 15,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = CASIOMAIN2_SENSOR_ID,
	.reg_addr_sensor_id = {0x300a, 0x300b, 0x300c},
	.i2c_addr_table = {0x20, 0x34, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
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

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 62,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = casiomain2_ana_gain_table,
	.ana_gain_table_size = sizeof(casiomain2_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 6,
	.exposure_max = 128*(0x7fffff - 20), /* exposure reg is limited to 4x. max = max - margin */
	.exposure_step = 2,
	.exposure_margin = 20,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 4,

	.frame_length_max = 0x7fffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 0,

	.pdaf_type = PDAF_SUPPORT_CAMSV,//PDAF_SUPPORT_CAMSV_QPD,
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
			{0x3500, 0x3501, 0x3502},//Long exposure
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3160,
	.reg_addr_ana_gain = {
			{0x3508, 0x3509},//Long Gian
	},
	.reg_addr_frame_length = {0x3840, 0x380e, 0x380f},
/* 	.reg_addr_temp_en = 0x0138,
	.reg_addr_temp_read = 0x013A,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = 0x0005,
	.reg_addr_fast_mode = 0x3010, */

	.init_setting_table = casiomain2_init_setting,
	.init_setting_len = ARRAY_SIZE(casiomain2_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
	.checksum_value = 0xcd9966da,
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
	{HW_ID_PDN, 1, 3},
	{HW_ID_AVDD, 2825000, 3},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_AFVDD, 2800000, 3},
	{HW_ID_DVDD, 1200000, 4},
	//{HW_ID_DOVDD, 1800000, 3},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_RST, 1, 2},
};

static struct subdrv_pw_seq_entry pw_off_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1800000, 3},
	{HW_ID_PDN, 1, 3},
	{HW_ID_AVDD, 2825000, 3},
	{HW_ID_AFVDD, 2800000, 3},
	{HW_ID_DVDD, 1104000, 4},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry casiomain2_mipi_raw_entry = {
	.name = "casiomain2_mipi_raw",
	.id = CASIOMAIN2_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.pw_off_seq = pw_off_seq,
	.pw_off_seq_cnt = ARRAY_SIZE(pw_off_seq),
	.ops = &ops,
};


/* FUNCTION */

static unsigned int read_casiomain2_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);

	if (meta_id != casiomain2_eeprom_info[meta_id].meta)
		return -1;

	if (size != casiomain2_eeprom_info[meta_id].size)
		return -1;

	addr = casiomain2_eeprom_info[meta_id].start;
	readsize = casiomain2_eeprom_info[meta_id].size;

	if(!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

/* static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
	.i2c_read_id = 0xA1,
	.i2c_write_id = 0xA0,

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
}; */


static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 addr_sensorver = 0x0018;
	struct oplus_eeprom_info_struct* infoPtr;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	infoPtr = (struct oplus_eeprom_info_struct*)(para);
	*len = sizeof(oplus_eeprom_info);
	if (subdrv_i2c_rd_u8(ctx, addr_sensorver) != 0x00) {
		DRV_LOG(ctx, "need to convert to 10bit");
		infoPtr->afInfo[0] = (kal_uint8)((infoPtr->afInfo[1] << 6) | (infoPtr->afInfo[0] >> 2));
		infoPtr->afInfo[1] = (kal_uint8)(infoPtr->afInfo[1] >> 2);
		infoPtr->afInfo[2] = (kal_uint8)((infoPtr->afInfo[3] << 6) | (infoPtr->afInfo[2] >> 2));
		infoPtr->afInfo[3] = (kal_uint8)(infoPtr->afInfo[3] >> 2);
		infoPtr->afInfo[4] = (kal_uint8)((infoPtr->afInfo[5] << 6) | (infoPtr->afInfo[4] >> 2));
		infoPtr->afInfo[5] = (kal_uint8)(infoPtr->afInfo[5] >> 2);
	}

	return 0;
}

static int casiomain2_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOGE(ctx, "get otp data");
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	memcpy(feature_return_para_32, casiomain2_common_data,
		OPLUS_CAMERA_COMMON_DATA_LENGTH);
	*len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
	return 0;
}

/*
static void casiomain2_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	LOG_INF("get unique sensorid");
	memcpy(feature_return_para_32, casiomain2_unique_id,
		CASIOMAIN2_UNIQUE_SENSOR_ID_LENGHT);
	LOG_INF("para :%x, get unique sensorid", *para);
}
*/

/* static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, CASIOMAIN2_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
} */

#ifdef WRITE_DATA_MAX_LENGTH
#undef WRITE_DATA_MAX_LENGTH
#endif
#define   WRITE_DATA_MAX_LENGTH     (32)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, CASIOMAIN2_EEPROM_WRITE_ID >> 1,
            addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xE000;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, CASIOMAIN2_EEPROM_READ_ID >> 1, reg, (CASIOMAIN2_EEPROM_WRITE_ID & 0xFE) | 0x01);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, CASIOMAIN2_EEPROM_READ_ID >> 1, reg, CASIOMAIN2_EEPROM_WRITE_ID & 0xFE);
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
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
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
        if ((pStereodata->uSensorId == CASIOMAIN2_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
            /*&& (data_base == CASIOMAIN2_STEREO_START_ADDR )*/) {

                return ERROR_NONE;
           /* LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);
            write_eeprom_protect(ctx, 0);
            msleep(6);
            if (offset > 0) {
                ret = table_write_eeprom_30Bytes(ctx, data_base, &pData[0], offset);
                if (ret != ERROR_NONE) {
                    LOG_INF("write_eeprom error: offset\n");
                    // open write protect
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
                write_eeprom_protect(ctx, 1);
                msleep(6);
                return -1;
            }
            msleep(6);
            write_eeprom_protect(ctx, 1);
            msleep(6);
            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");
            */
        } else if ((pStereodata->uSensorId == CASIOMAIN2_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            /*&& (data_base == CASIOMAIN2_AESYNC_START_ADDR)*/) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
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
                write_eeprom_protect(ctx, 1);
                msleep(6);
                return -1;
            }
            msleep(6);
            write_eeprom_protect(ctx, 1);
            msleep(6);
            /* LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                read_cmos_eeprom_8(ctx, CASIOMAIN2_AESYNC_START_ADDR),
                read_cmos_eeprom_8(ctx, CASIOMAIN2_AESYNC_START_ADDR+1),
                read_cmos_eeprom_8(ctx, CASIOMAIN2_AESYNC_START_ADDR+2),
                read_cmos_eeprom_8(ctx, CASIOMAIN2_AESYNC_START_ADDR+3),
                read_cmos_eeprom_8(ctx, CASIOMAIN2_AESYNC_START_ADDR+4),
                read_cmos_eeprom_8(ctx, CASIOMAIN2_AESYNC_START_ADDR+5),
                read_cmos_eeprom_8(ctx, CASIOMAIN2_AESYNC_START_ADDR+6),
                read_cmos_eeprom_8(ctx, CASIOMAIN2_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n"); */
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("casiomain2 write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int casiomain2_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        LOG_INF("ret=%d\n", ret);
    }
	return 0;
}

static int casiomain2_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
    if(*len > CALI_DATA_MASTER_LENGTH)
        *len = CALI_DATA_MASTER_LENGTH;
    read_casiomain2_eeprom_info(ctx, EEPROM_META_STEREO_DATA,
            (BYTE *)para, *len);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	if (adaptor_i2c_rd_p8(ctx->i2c_client, CASIOMAIN2_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	DRV_LOGE(ctx, "casiomain2 read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "casiomain2 read_otp_info end\n");
}

static int casiomain2_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOGE(ctx, "get otp data");
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	if (otp_data_checksum[0] == 0) {
		read_otp_info(ctx);
	} else {
		DRV_LOG(ctx, "otp data has already read");
	}
	memcpy(feature_return_para_32, (UINT32 *)otp_data_checksum, sizeof(otp_data_checksum));
	*len = sizeof(otp_data_checksum);
	return 0;
}

static int casiomain2_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
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
	//LOG_INF("yuan get_imgsensor_id ov50d");
	//DRV_LOG(ctx, "yuan get_imgsensor_id ov50d");
	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			LOG_INF("i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x565044) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					//read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					first_read = KAL_FALSE;
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

static u16 casiomain2_feedback_awbgain[] = {
	0x0B8E, 0x01,
	0x0B8F, 0x00,
	0x0B90, 0x02,
	0x0B91, 0x28,
	0x0B92, 0x01,
	0x0B93, 0x77,
	0x0B94, 0x01,
	0x0B95, 0x00,
};

/*write AWB gain to sensor*/
static void feedback_awbgain(struct subdrv_ctx *ctx, kal_uint32 r_gain, kal_uint32 b_gain)
{
	UINT32 r_gain_int = 0;
	UINT32 b_gain_int = 0;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || // RMSC
			ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5) { // QRMSC
		DRV_LOG(ctx, "feedback_awbgain r_gain: %d, b_gain: %d\n", r_gain, b_gain);
		//LOG_INF("yuan in feedback %s ov50d", __FUNCTION__);
		r_gain_int = r_gain / 512;
		b_gain_int = b_gain / 512;
		casiomain2_feedback_awbgain[5] = r_gain_int;
		casiomain2_feedback_awbgain[7] = (r_gain - r_gain_int * 512) / 2;
		casiomain2_feedback_awbgain[9] = b_gain_int;
		casiomain2_feedback_awbgain[11] = (b_gain - b_gain_int * 512) / 2;
		subdrv_i2c_wr_regs_u8(ctx, casiomain2_feedback_awbgain,
			ARRAY_SIZE(casiomain2_feedback_awbgain));
	}
}

static int casiomain2_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;
	feedback_awbgain(ctx, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B);
	return 0;
}

static void casiomain2_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
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
	if (last_exp_cnt == exp_cnt) {
		for (i = 1; i < exp_cnt; i++) {
			readout_diff = ctx->exposure[i] - shutters[i];
			calc_fl[2] += readout_diff > 0 ? readout_diff : 0;
		}
	}
	for (i = 0; i < ARRAY_SIZE(calc_fl); i++)
		ctx->frame_length = max(ctx->frame_length, calc_fl[i]);
	ctx->frame_length =	max(ctx->frame_length, ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	for (i = 0; i < exp_cnt; i++)
		ctx->exposure[i] = shutters[i];
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
	for (i = 0; i < 3; i++) {
		if (rg_shutters[i]) {
			if (ctx->s_ctx.reg_addr_exposure[i].addr[2]) {
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 16) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					(rg_shutters[i] >> 8) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[2],
					(rg_shutters[i]) & 0xFF);
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

static int casiomain2_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	casiomain2_set_multi_shutter_frame_length(ctx, (u32 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

static void casiomain2_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		              ctx->dummy_line, ctx->dummy_pixel);
    set_i2c_buffer(ctx, 0x3840, ctx->frame_length >> 16);
    set_i2c_buffer(ctx, 0x380e, ctx->frame_length >>  8);
    set_i2c_buffer(ctx, 0x380f, ctx->frame_length & 0xFF);
}

static void casiomain2_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	if (ctx->frame_length > ctx->s_ctx.frame_length_max) {
		ctx->frame_length = ctx->s_ctx.frame_length_max;

		ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;

	casiomain2_set_dummy(ctx);
}

static u32 gain2reg_ov(const u32 gain)
{
	u32 iReg = 0x0000;

	//platform 1xgain = 64, sensor driver 1*gain = 0x100
	iReg = gain*256/BASEGAIN;

	if(iReg < 0x100)	//sensor 1xGain
	{
		iReg = 0x100;
	}
	if(iReg > 0x3e00)	//sensor 62xGain
	{
		iReg = 0x3e00;
	}
	return iReg;		/* sensorGlobalGain */

}

static int casiomain2_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u64* feature_data = (u64*)para;
    u32 gain = *feature_data;
    u32 reg_gain;
    int ret = 0;

    //unsigned long flags;
    /*if (videoGainDiffNormal && gain > imgsensor_info.video_max_gain) {
        gain = imgsensor_info.video_max_gain;
    }*/
	if (gain > ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max) {
		gain = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max;
	}
    DRV_LOG(ctx, "cdb_test read gain = %d %d\n ", gain, BASEGAIN);
    reg_gain = gain2reg_ov(gain);
    /*spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.gain = reg_gain;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);*/

    /* restore gain */
    memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
    ctx->ana_gain[0] = reg_gain;

    DRV_LOG(ctx, "cdb_test set  gain = %d\n ", reg_gain);
    ret = subdrv_i2c_wr_u8(ctx, 0x3508, (reg_gain >> 8));
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3508: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3508: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3509, (reg_gain&0xff));
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3509: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3509: Succeed !");
    }
    DRV_LOG(ctx, "cdb_test read gain = %x:%x\n ", subdrv_i2c_rd_u8(ctx, 0x3508), subdrv_i2c_rd_u8(ctx, 0x3509));
    //videoGainDiffNormal = false;
    return gain;
}

static void casiomain2_write_shutter(struct subdrv_ctx *ctx)
{
	uint16_t realtime_fps = 0;
	static u32 lastshutter = 0;
	int ret = 0;

	DRV_LOG(ctx, "===brad shutter:%d\n", ctx->exposure[0]);

	if (ctx->exposure[0] > ctx->min_frame_length - ctx->s_ctx.exposure_margin) {
		ctx->frame_length = ctx->exposure[0] + ctx->s_ctx.exposure_margin;
	} else {
		ctx->frame_length = ctx->min_frame_length;
	}
	if (ctx->frame_length > ctx->s_ctx.frame_length_max) {
		ctx->frame_length = ctx->s_ctx.frame_length_max;
	}

	if (ctx->exposure[0] < ctx->s_ctx.exposure_min) {
		ctx->exposure[0] = ctx->s_ctx.exposure_min;
	}
	ctx->exposure[0] = (ctx->exposure[0] >> 1) << 1;
	ctx->frame_length = (ctx->frame_length >> 1) << 1;

	if ((lastshutter <= ctx->frame_length) && (ctx->frame_length <= (lastshutter+6))) {
		ctx->frame_length = lastshutter+8;
	}

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps > 297 && realtime_fps <= 305) {
			casiomain2_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 147 && realtime_fps <= 150) {
			casiomain2_set_max_framerate(ctx, 146, 0);
		}
	}

    ret = subdrv_i2c_wr_u8(ctx, 0x3208, 0x01);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3840, ctx->frame_length >> 16);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3840: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3840: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x380e, ctx->frame_length >>  8);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x380e: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x380e: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x380f, ctx->frame_length & 0xFF);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x380f: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x380f: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3500, (ctx->exposure[0] >> 16) & 0xFF);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3500: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3500: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3501, (ctx->exposure[0] >>  8) & 0xFF);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3501: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3501: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3502, (ctx->exposure[0])  & 0xFF);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3502: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3502: Succeed !");
    }

    ret = subdrv_i2c_wr_u8(ctx, 0x3208, 0x11);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3208, 0xa1);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Succeed !");
    }
    lastshutter = ctx->exposure[0];

    DRV_LOG(ctx, "cdb_test set  shutter =%x:%x:%x\n", (ctx->exposure[0] >> 16) & 0xFF, (ctx->exposure[0] >>  8) & 0xFF, (ctx->exposure[0])  & 0xFF);
    DRV_LOG(ctx, "cdb_test read shutter  %x:%x:%x \n ", subdrv_i2c_rd_u8(ctx, 0x3500), subdrv_i2c_rd_u8(ctx, 0x3501), subdrv_i2c_rd_u8(ctx, 0x3502));

}	/*	write_shutter  */

static int casiomain2_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u32 *shutter = (u32*)para;
    ctx->exposure[0] = *shutter;
    casiomain2_write_shutter(ctx);
	return 0;
}

static void casiomain2_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 *shutter, u32 frame_length, bool auto_extend_en)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	static u32 lastshutter = 0;
	int ret = 0;
	ctx->exposure[0] = *shutter;

	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;

	ctx->frame_length = ctx->frame_length + dummy_line;

	if (ctx->exposure[0] > ctx->frame_length - ctx->s_ctx.exposure_margin)
		ctx->frame_length = ctx->exposure[0] + ctx->s_ctx.exposure_margin;

	if (ctx->frame_length > ctx->s_ctx.frame_length_max)
		ctx->frame_length = ctx->s_ctx.frame_length_max;

	ctx->exposure[0] = (ctx->exposure[0] < ctx->s_ctx.exposure_min)
			? ctx->s_ctx.exposure_min : ctx->exposure[0];

	if ((lastshutter <= ctx->frame_length) && (ctx->frame_length <= (lastshutter+6))) {
		ctx->frame_length = lastshutter+8;
	}

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps > 297 && realtime_fps <= 305) {
			casiomain2_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 147 && realtime_fps <= 150) {
			casiomain2_set_max_framerate(ctx, 146, 0);
		}
	}
    ret = subdrv_i2c_wr_u8(ctx, 0x3208, 0x01);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3840, ctx->frame_length >> 16);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3840: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3840: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x380e, ctx->frame_length >>  8);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x380e: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x380e: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x380f, ctx->frame_length & 0xFF);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x380f: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x380f: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3500, (ctx->exposure[0] >> 16) & 0xFF);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3500: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3500: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3501, (ctx->exposure[0] >>  8) & 0xFF);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3501: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3501: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3502, (ctx->exposure[0])  & 0xFF);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3502: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3502: Succeed !");
    }

    ret = subdrv_i2c_wr_u8(ctx, 0x3208, 0x11);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Succeed !");
    }
    ret = subdrv_i2c_wr_u8(ctx, 0x3208, 0xa1);
    if(ret < 0) {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Failed !");
    } else {
        DRV_LOG(ctx, " subdrv_i2c_wr_u8 0x3208: Succeed !");
    }
    lastshutter = ctx->exposure[0];

    DRV_LOG(ctx, "cdb_test set  shutter =%x:%x:%x\n", (ctx->exposure[0] >> 16) & 0xFF, (ctx->exposure[0] >>  8) & 0xFF, (ctx->exposure[0])  & 0xFF);
    DRV_LOG(ctx, "cdb_test read shutter  %x:%x:%x \n ", subdrv_i2c_rd_u8(ctx, 0x3500), subdrv_i2c_rd_u8(ctx, 0x3501), subdrv_i2c_rd_u8(ctx, 0x3502));

}	/* set_shutter_frame_length */

static int casiomain2_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "shutter:%u, frame_length:%u\n", (u32)(*para), (u32) (*(para + 1)));
	casiomain2_set_shutter_frame_length_convert(ctx, (u32 *)para, (u32) (*(para + 1)), (u16) (*(para + 2)));
	return 0;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	DRV_LOG(ctx, "yuan addr:0x0301 value:0x%x   line:%d", subdrv_i2c_rd_u8(ctx, 0x0301), __LINE__);
	subdrv_i2c_wr_u8(ctx, 0x0301, 0x01);
	DRV_LOG(ctx, "yuan addr:0x0301 value:0x%x   line:%d", subdrv_i2c_rd_u8(ctx, 0x0301), __LINE__);
	/* initail setting */
	DRV_LOG(ctx, "yuan open addr:0x0103 value:0x%x  line:%d", subdrv_i2c_rd_u8(ctx, 0x0103), __LINE__);
	subdrv_i2c_wr_u8(ctx, 0x0103, 0x01);
	sensor_init(ctx);
	DRV_LOG(ctx, "yuan open addr:0x0103 value:0x%x  line:%d", subdrv_i2c_rd_u8(ctx, 0x0103), __LINE__);
	DRV_LOG(ctx, "yuan addr:0x0301 value:0x%x   line:%d", subdrv_i2c_rd_u8(ctx, 0x0301), __LINE__);
	/*QSC&SPC setting*/
/* 	if (ctx->s_ctx.s_cali != NULL) {
		ctx->s_ctx.s_cali((void*)ctx);
	} else {
		write_sensor_Cali(ctx);
	} */

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

static void set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
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
			subdrv_i2c_wr_u8(ctx, SENSOR_QSC_ENABLE_REG, 0x01);
			DRV_LOG(ctx, "set QSC calibration data done.");
		} else {
			subdrv_i2c_wr_u8(ctx, SENSOR_QSC_ENABLE_REG, 0x00);
		}
	}

	/* SPC data */
	support = info[idx].pdc_support;
	pbuf = info[idx].preload_pdc_table;
	size = info[idx].pdc_size;
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			addr = SPC_OTP_ADDR_PART1;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size >> 1);
			addr = SPC_OTP_ADDR_PART2;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf + (size >> 1), size >> 1);
			DRV_LOG(ctx, "set SPC data done.");
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	DRV_LOG(ctx, "yuan addr:0x0103 value:0x%x", subdrv_i2c_rd_u8(ctx, 0x0103));
	DRV_LOG(ctx, "yuan addr:0x0102 value:0x%x", subdrv_i2c_rd_u8(ctx, 0x0102));
	DRV_LOG(ctx, "yuan addr:0x0301 value:0x%x", subdrv_i2c_rd_u8(ctx, 0x0301));



 	for(int i = 0x0304 ; i < 0x0308; i++ )
	{
		DRV_LOG(ctx, "yuan addr:0x%x value:0x%x", i, subdrv_i2c_rd_u8(ctx, i));
	} 


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
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	return (16384 - (16384 * BASEGAIN) / gain);
}

void casiomain2_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	u32 exp_cnt = 0;
	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	check_current_scenario_id_bound(ctx);
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	LOG_INF("sensor_mode_num[%d]", ctx->s_ctx.sensor_mode_num);
	if (scenario_id < ctx->s_ctx.sensor_mode_num) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
			case HDR_RAW_STAGGER:
				*exposure_step = ctx->s_ctx.exposure_step * exp_cnt;
				*min_shutter = ctx->s_ctx.exposure_min * exp_cnt;
				break;
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

int casiomain2_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	casiomain2_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return 0;
}

static int casiomain2_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM pre_seamless_scenario_id;
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 exp_cnt = 0;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return ERROR_INVALID_SCENARIO_ID;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
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

	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	ctx->is_seamless = TRUE;
	pre_seamless_scenario_id = ctx->current_scenario_id;
	update_mode_info(ctx, scenario_id);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x02);
	if (ctx->s_ctx.reg_addr_fast_mode_in_lbmf &&
		(ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_LBMF ||
		ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF))
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode_in_lbmf, 0x4);

	update_mode_info(ctx, scenario_id);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	DRV_LOG(ctx, "write seamless switch setting done\n");
	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_LBMF:
			set_multi_shutter_frame_length_in_lut(ctx,
				(u64 *)&ae_ctrl->exposure, exp_cnt, 0, frame_length_in_lut);
			set_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		default:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	common_get_prsh_length_lines(ctx, ae_ctrl, pre_seamless_scenario_id, scenario_id);

	if (ctx->s_ctx.seamless_switch_prsh_length_lc > 0) {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_prsh_mode, 0x01);

		subdrv_i2c_wr_u8(ctx,
				ctx->s_ctx.reg_addr_prsh_length_lines.addr[0],
				(ctx->s_ctx.seamless_switch_prsh_length_lc >> 16) & 0xFF);
		subdrv_i2c_wr_u8(ctx,
				ctx->s_ctx.reg_addr_prsh_length_lines.addr[1],
				(ctx->s_ctx.seamless_switch_prsh_length_lc >> 8)  & 0xFF);
		subdrv_i2c_wr_u8(ctx,
				ctx->s_ctx.reg_addr_prsh_length_lines.addr[2],
				(ctx->s_ctx.seamless_switch_prsh_length_lc) & 0xFF);

		DRV_LOG(ctx, "seamless switch pre-shutter set(%u)\n", ctx->s_ctx.seamless_switch_prsh_length_lc);
	} else
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_prsh_mode, 0x00);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}

static int casiomain2_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	if (mode) {
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
		switch (mode) {
		case 5:
			subdrv_i2c_wr_u8(ctx, 0x020E, 0x00); /* dig_gain = 0 */
			subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
			subdrv_i2c_wr_u8(ctx, 0x3015, 0x00);
			break;
		default:
			subdrv_i2c_wr_u8(ctx, 0x0601, mode);
			break;
		}
	} else if (ctx->test_pattern) {
		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, mode);
		//subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/
		//subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);
		//subdrv_i2c_wr_u8(ctx, 0x0218, 0x01);
		//subdrv_i2c_wr_u8(ctx, 0x3015, 0x40);
		subdrv_i2c_wr_u8(ctx, 0x430b, 0xff); /*No pattern*/
		subdrv_i2c_wr_u8(ctx, 0x430c, 0xff);
		subdrv_i2c_wr_u8(ctx, 0x4310, 0xff);
		subdrv_i2c_wr_u8(ctx, 0x4311, 0xff);
	}
	ctx->test_pattern = mode;
	return 0;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
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
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	ctx->sof_cnt = sof_cnt;
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, 0x3010, 0x00);
		set_i2c_buffer(ctx, 0x3036, 0x00);
		commit_i2c_buffer(ctx);
	}
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
	u8 pdc_is_valid = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
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

	/* SPC data */
	support = info[idx].pdc_support;
	size = info[idx].pdc_size;
	addr = info[idx].addr_pdc;
	buf = info[idx].pdc_table;
	if (support && size > 0) {
		/* Check pdc validation */
		pdc_is_valid = i2c_read_eeprom(ctx, OTP_PDC_VALID_ADDR);
		if (pdc_is_valid != SPC_IS_VALID_VAL) {
			DRV_LOGE(ctx, "SPC data is invalid, flag(%02x)", pdc_is_valid);
		} else if (info[idx].preload_pdc_table == NULL) {
			info[idx].preload_pdc_table = kmalloc(size, GFP_KERNEL);
			if (buf == NULL) {
				if (!read_cmos_eeprom_p8(ctx, addr, info[idx].preload_pdc_table, size)) {
					DRV_LOGE(ctx, "preload PDC data failed");
				}
			} else {
				memcpy(info[idx].preload_pdc_table, buf, size);
			}
			DRV_LOG(ctx, "preload PDC data %u bytes", size);
		} else {
			DRV_LOG(ctx, "PDC data is already preloaded %u bytes", size);
		}
	}

	ctx->is_read_preload_eeprom = 1;
}

static void comp_mode_tran_time_cal1(struct subdrv_ctx *ctx, u32 scenario_id, u32* prsh) {
	#define SYSTEM_USED_LINES1 (96UL)
	#define SYSTEM_DELAY1      (189UL)
	u64 frame_duration = 0;
	u64 data_delay = 0;
	u64 system_delay = 0;
	u64 current_tline = 0;
	u64 tline = 0;
	//LOG_INF("yuan %s ov50d", __FUNCTION__);
	if (!prsh) {
		DRV_LOGE(ctx, "prsh param is NULL");
		return;
	}

	*prsh = 0U;
	if (casiomain2_comp_params[ctx->current_scenario_id].clock_vtpxck == 0) {
		DRV_LOG(ctx, "invalid params");
		return;
	}

	frame_duration = 1000000000UL / ctx->current_fps * 10;
	current_tline = 1000000000UL * ctx->s_ctx.mode[ctx->current_scenario_id].linelength /
		ctx->s_ctx.mode[ctx->current_scenario_id].pclk;
	tline = 1000000000UL * ctx->s_ctx.mode[scenario_id].linelength /
		ctx->s_ctx.mode[scenario_id].pclk;
	data_delay = (ctx->s_ctx.mode[ctx->current_scenario_id].imgsensor_winsize_info.h2_tg_size +
		SYSTEM_USED_LINES1) * current_tline;
	system_delay = SYSTEM_DELAY1 * 1000 * 1000 * 10 /
		casiomain2_comp_params[ctx->current_scenario_id].clock_vtpxck;
	if (frame_duration <= data_delay + system_delay) {
		DRV_LOGE(ctx, "invalid parameter");
		return;
	}

	*prsh = (frame_duration - data_delay - system_delay) / tline;
	if (ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_STAGGER) {
		*prsh = *prsh / 2;
	}

	DRV_LOG(ctx, "frame_duration(%llu), current_tline(%llu), tline(%llu), "
		"data_delay(%llu) system_delay(%llu) prsh(%u)\n", frame_duration,
		current_tline, tline, data_delay, system_delay, *prsh);
}
