// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 alphahmainmipiraw_Sensor.c
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
#include "alphahmainmipiraw_Sensor.h"

#define ALPHAHMAIN_EEPROM_READ_ID	0xA1
#define ALPHAHMAIN_EEPROM_WRITE_ID	0xA0
#define ALPHAHMAIN_MAX_OFFSET		0x8000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "alphahmain_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE              0x8000
#define OTP_PDC_ADDR          0x1900
#define OTP_PDC_SIZE          0x1CA
#define SENSOR_PDC_ADDR       0x59F0
#define PDC_IS_VALID_VAL      0x01
#define PDC_TYPE              0x03
#define alphahmain_burst_write_cmos_sensor(...) subdrv_i2c_wr_p8(__VA_ARGS__)

#define OTP_QSC_VALID_ADDR    0x2E10
#define QSC_IS_VALID_VAL      0x01
#define SENSOR_QSC_ENABLE_REG 0x3206

#define OTP_PDC_VALID_ADDR    0x2FA0
#define SPC_IS_VALID_VAL      0x01
#define SPC_OTP_ADDR_PART1    0xD200
#define SPC_OTP_ADDR_PART2    0xD300

#define ALPHAHMAIN_UNIQUE_SENSOR_ID_ADDR    (0x7000)
#define ALPHAHMAIN_UNIQUE_SENSOR_ID_LENGTH  (16)
static BYTE alphahmain_unique_id[ALPHAHMAIN_UNIQUE_SENSOR_ID_LENGTH] = { 0 };

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int alphahmain_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int alphahmain_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void get_sensor_cali(void* arg);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static void alphahmain_write_shutter(struct subdrv_ctx *ctx);
static int alphahmain_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void alphahmain_write_frame_length(struct subdrv_ctx *ctx, u32 fll);
static int alphahmain_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_get_cloud_otp_info(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmain_set_cali_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
/* STRUCT */

static BYTE alphahmain_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };

/* Normal(Qbin) to Normal(Qbin) */
/* Normal(Qbin) to 2DOL(Qbin) */
static void comp_mode_tran_time_cal1(struct subdrv_ctx *ctx, u32 pre_scenario_id, u32* prsh);
typedef void (*cal_comp_mode_tran_time)(struct subdrv_ctx *ctx, u32 pre_scenario_id, u32* prsh);
struct comp_mode_tran_time_params {
	u8 enable;
	u32 clock_vtpxck;
	cal_comp_mode_tran_time cal_fn;
};
static struct comp_mode_tran_time_params alphahmain_comp_params[SENSOR_SCENARIO_ID_MAX] = {
	{ .enable = 0, }, /*pre*/
	{ .enable = 1, .clock_vtpxck = 1884, .cal_fn = comp_mode_tran_time_cal1, }, /*cap*/
	{ .enable = 0, }, /*vid*/
	{ .enable = 0, }, /*hvid*/
	{ .enable = 0, }, /*svid*/
	{ .enable = 0, }, /*cus1*/
	{ .enable = 0, }, /*cus2*/
	{ .enable = 0, }, /*csu3*/
};

static struct eeprom_map_info alphahmain_eeprom_info[] = {
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
	{SENSOR_FEATURE_SET_TEST_PATTERN, alphahmain_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, alphahmain_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, alphahmain_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_DATA, alphahmain_get_sensor_sn},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, alphahmain_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, alphahmain_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, alphahmain_get_otp_checksum_data},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, alphahmain_get_min_shutter_by_scenario_adapter},
	{SENSOR_FEATURE_SET_AWB_GAIN, alphahmain_set_awb_gain},
	{SENSOR_FEATURE_SET_GAIN, alphahmain_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, alphahmain_set_shutter},
	{SENSOR_FEATURE_SET_FRAMELENGTH, alphahmain_set_frame_length},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, alphahmain_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, alphahmain_set_multi_shutter_frame_length_ctrl},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, alphahmain_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, alphahmain_streaming_resume},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, alphahmain_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_GET_UNIQUE_SENSORID, alphahmain_get_unique_sensorid},
	{SENSOR_FEATURE_GET_CLOUD_OTP_INFO, alphahmain_get_cloud_otp_info},
	{SENSOR_FEATURE_SET_CALI_DATA, alphahmain_set_cali_data},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01C10115,
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
#endif

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
			.hsize = 3648,
			.vsize = 2736,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 912,
			.vsize = 684,
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2048,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 992,//4096
			.vsize = 512,//768
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
    {/*Reg_B_4096x2304_30FPS**/
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = alphahmain_preview_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmain_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 850,
		.framelength = 3920,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
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
		.mode_setting_table = alphahmain_capture_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmain_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 850,
		.framelength = 3920,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
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
		.mode_setting_table = alphahmain_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmain_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 850,
		.framelength = 3921,
		.max_framerate = 300,
		.mipi_pixel_rate = 672000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
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
		.mode_setting_table = alphahmain_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmain_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 650,
		.framelength = 2564,
		.max_framerate = 600,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
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
		.mode_setting_table = alphahmain_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmain_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 325,
		.framelength = 2564,
		.max_framerate = 1200,
		.mipi_pixel_rate = 540000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
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
		.mode_setting_table = alphahmain_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmain_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 325,
		.framelength = 1280,
		.max_framerate = 2400,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
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
		.mode_setting_table = alphahmain_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmain_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 850,
		.framelength = 4902,
		.max_framerate = 240,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
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
		.mode_setting_table = alphahmain_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmain_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 850,
		.framelength = 3920,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 6,
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
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1,
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
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = alphahmain_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmain_custom4_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 850,
		.framelength = 3920,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
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
			.y1_offset = 512,
			.w1_size = 4096,
			.h1_size = 2048,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2048,
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
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = ALPHAHMAIN_SENSOR_ID,
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
	.ana_gain_type = 1,
	.ana_gain_step = 1,
	.ana_gain_table = alphahmain_ana_gain_table,
	.ana_gain_table_size = sizeof(alphahmain_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 6,
	.exposure_max = (0x7ffffc - 20), /* exposure reg is limited to 4x. max = max - margin */
	.exposure_step = 2,
	.exposure_margin = 20,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 4,

	.frame_length_max = 0x7fffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 2293000,

	.pdaf_type = PDAF_SUPPORT_CAMSV,//PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
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
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {
			{0x3508, 0x3509},//Long Gian
	},
	.reg_addr_frame_length = {0x3840, 0x380e, 0x380f},
 	.reg_addr_temp_en = 0x4D12,
	.reg_addr_temp_read = 0x4D13,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x387e,
	.reg_addr_fast_mode = PARAM_UNDEFINED,

	.init_setting_table = alphahmain_init_setting,
	.init_setting_len = ARRAY_SIZE(alphahmain_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.chk_s_off_sta = 0,
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
	{HW_ID_RST, 0, 11},
	{HW_ID_AVDD, 2800000, 3},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_AFVDD, 2800000, 3},
	{HW_ID_DVDD, 1200000, 4},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_RST, 1, 5},
};

static struct subdrv_pw_seq_entry oplus_pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2800000, 3},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_AFVDD, 2800000, 3},
	{HW_ID_DVDD, 1200000, 4},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_RST, 1, 5},
};

const struct subdrv_entry alphahmain_mipi_raw_entry = {
	.name = "alphahmain_mipi_raw",
	.id = ALPHAHMAIN_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};


/* FUNCTION */

static unsigned int read_alphahmain_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != alphahmain_eeprom_info[meta_id].meta)
		return -1;

	if (size != alphahmain_eeprom_info[meta_id].size)
		return -1;

	addr = alphahmain_eeprom_info[meta_id].start;
	readsize = alphahmain_eeprom_info[meta_id].size;

	if(!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
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
};

static kal_uint8 otp_pdc_data[OTP_PDC_SIZE] = {0};

static void read_eeprom_pdc_data(struct subdrv_ctx *ctx)
{
	u32 pdVersion = 0, pdCompensation = 0;
	if(!read_cmos_eeprom_p8(ctx, OTP_PDC_ADDR, otp_pdc_data, OTP_PDC_SIZE)) {
		LOG_INF("read pdc data failed!\n");
	}
	pdVersion      = otp_pdc_data[3] << 24 | otp_pdc_data[2] << 16 | otp_pdc_data[1] << 8 | otp_pdc_data[0];
	pdCompensation = otp_pdc_data[7] << 24 | otp_pdc_data[6] << 16 | otp_pdc_data[5] << 8 | otp_pdc_data[4];
	LOG_INF("pdVersion = 0x%x, pdCompensation = 0x%x\n", pdVersion, pdCompensation);
}

static struct SENSOR_OTP_INFO_STRUCT cloud_otp_info[OPLUS_CAM_CAL_DATA_MAX] = {
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0000, 17}}, /*{addr_modinfo, addr_modinfolen}*/
	}, /*OPLUS_CAM_CAL_DATA_MODULE_VERSION*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0000, 17}}, /*{addr_modinfo, addr_modinfolen}*/
	}, /*OPLUS_CAM_CAL_DATA_PART_NUMBER*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x1d60, 1868}},
	}, /*OPLUS_CAM_CAL_DATA_SHADING_TABLE--LSC*/
	{
		.OtpInfoLen = 5,
		.OtpInfo = {{0x0020, 16}, {0x0044, 16}, {0x0060, 4}, {0x006c, 4}, {0x0092, 6}},
		.isAFCodeOffset = KAL_FALSE,
	}, /*OPLUS_CAM_CAL_DATA_3A_GAIN-awb5000\awb2850\awb5000Light\awb2850light\af*/
	{
		.OtpInfoLen = 2,
		.OtpInfo = {{0x1300, 496}, {0x1500, 1004}},
	}, /*OPLUS_CAM_CAL_DATA_PDAF*/
	{
		.OtpInfoLen = 8,
		.OtpInfo = {{0x0000, 17}, {0x0006, 2}, {0x0008, 2}, {0x000a, 2}, {0x0092, 7}, {0x0092, 2}, {0x0094, 2}, {0x00b0, 24}},
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
		.OtpInfoLen = 0,
	}, /*OPLUS_CAM_CAL_DATA_QSC*/
	{
		.OtpInfoLen = 0,
	}, /*OPLUS_CAM_CAL_DATA_LRC*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0000, 16384}},
	}, /*OPLUS_CAM_CAL_DATA_ALL*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x1900, 458}},
	}, /*OPLUS_CAM_CAL_DATA_PDC*/
};

static int alphahmain_get_cloud_otp_info(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	case OPLUS_CAM_CAL_DATA_PDC:
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
		subdrv_i2c_wr_u8(ctx, 0x0103, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x3d84, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x3d85, 0x1b);
		subdrv_i2c_wr_u8(ctx, 0x0100, 0x01);
		msleep(5);
		if (adaptor_i2c_rd_p8(ctx->i2c_client, ctx->i2c_write_id >> 1, ALPHAHMAIN_UNIQUE_SENSOR_ID_ADDR,
			&(alphahmain_unique_id[0]), ALPHAHMAIN_UNIQUE_SENSOR_ID_LENGTH) < 0) {
			LOG_INF("Read sensor unique sensorid fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
		}
		i++;
	}
}

static int alphahmain_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	*len = ALPHAHMAIN_UNIQUE_SENSOR_ID_LENGTH;
	memcpy(feature_return_para_32, alphahmain_unique_id,
		ALPHAHMAIN_UNIQUE_SENSOR_ID_LENGTH);
	LOG_INF("para :%x, get unique sensorid", *para);
	return 0;
}

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct oplus_eeprom_info_struct* infoPtr;
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	infoPtr = (struct oplus_eeprom_info_struct*)(para);
	*len = sizeof(oplus_eeprom_info);

	return 0;
}

static int alphahmain_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOGE(ctx, "get otp data");

	memcpy(feature_return_para_32, alphahmain_common_data,
		OPLUS_CAMERA_COMMON_DATA_LENGTH);
	*len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
	return 0;
}

/* static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, ALPHAHMAIN_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
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

    ret = adaptor_i2c_wr_p8(ctx->i2c_client, ALPHAHMAIN_EEPROM_WRITE_ID >> 1,
            addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xE000;

    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, ALPHAHMAIN_EEPROM_READ_ID >> 1, reg, (ALPHAHMAIN_EEPROM_WRITE_ID & 0xFE) | 0x01);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, ALPHAHMAIN_EEPROM_READ_ID >> 1, reg, ALPHAHMAIN_EEPROM_WRITE_ID & 0xFE);
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
        if ((pStereodata->uSensorId == ALPHAHMAIN_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
            /*&& (data_base == ALPHAHMAIN_STEREO_START_ADDR )*/) {

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
        } else if ((pStereodata->uSensorId == ALPHAHMAIN_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            /*&& (data_base == ALPHAHMAIN_AESYNC_START_ADDR)*/) {
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
                read_cmos_eeprom_8(ctx, ALPHAHMAIN_AESYNC_START_ADDR),
                read_cmos_eeprom_8(ctx, ALPHAHMAIN_AESYNC_START_ADDR+1),
                read_cmos_eeprom_8(ctx, ALPHAHMAIN_AESYNC_START_ADDR+2),
                read_cmos_eeprom_8(ctx, ALPHAHMAIN_AESYNC_START_ADDR+3),
                read_cmos_eeprom_8(ctx, ALPHAHMAIN_AESYNC_START_ADDR+4),
                read_cmos_eeprom_8(ctx, ALPHAHMAIN_AESYNC_START_ADDR+5),
                read_cmos_eeprom_8(ctx, ALPHAHMAIN_AESYNC_START_ADDR+6),
                read_cmos_eeprom_8(ctx, ALPHAHMAIN_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n"); */
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("alphahmain write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int alphahmain_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        LOG_INF("ret=%d\n", ret);
    }
	return 0;
}

static int alphahmain_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    if(*len > CALI_DATA_MASTER_LENGTH)
        *len = CALI_DATA_MASTER_LENGTH;
    read_alphahmain_eeprom_info(ctx, EEPROM_META_STEREO_DATA,
            (BYTE *)para, *len);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, ALPHAHMAIN_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "alphahmain read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "alphahmain read_otp_info end\n");
}

static int alphahmain_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static void streaming_ctrl(struct subdrv_ctx *ctx, bool enable)
{
	check_current_scenario_id_bound(ctx);

	if (enable) {
		if (ctx->s_ctx.chk_s_off_sta) {
			DRV_LOG(ctx, "check_stream_off before stream on");
			check_stream_off(ctx);
		}
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
	} else {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
	}
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static void alphahmain_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	alphahmain_write_shutter(ctx);
}

static int alphahmain_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOGE(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			alphahmain_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int alphahmain_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOGE(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
		return 0;
}

static int alphahmain_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	LOG_INF("rst delay = %d, func: %s, line: %d\n", pw_seq[1].delay, __FUNCTION__, __LINE__);
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
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					read_eeprom_pdc_data(ctx);
					read_unique_sensorid(ctx);
					memcpy(&pw_seq, &oplus_pw_seq, sizeof(oplus_pw_seq));
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

static u16 alphahmain_feedback_awbgain[] = {
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
	if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || // RMSC
			ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5) { // QRMSC
		DRV_LOG(ctx, "feedback_awbgain r_gain: %d, b_gain: %d\n", r_gain, b_gain);
		r_gain_int = r_gain / 512;
		b_gain_int = b_gain / 512;
		alphahmain_feedback_awbgain[5] = r_gain_int;
		alphahmain_feedback_awbgain[7] = (r_gain - r_gain_int * 512) / 2;
		alphahmain_feedback_awbgain[9] = b_gain_int;
		alphahmain_feedback_awbgain[11] = (b_gain - b_gain_int * 512) / 2;
		subdrv_i2c_wr_regs_u8(ctx, alphahmain_feedback_awbgain,
			ARRAY_SIZE(alphahmain_feedback_awbgain));
	}
}

static int alphahmain_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;
	feedback_awbgain(ctx, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B);
	return 0;
}

static void alphahmain_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
		u64 *shutter, u16 exp_cnt,	u16 frame_length)
{
	int i = 0;
	u32 fine_integ_line = 0;
	u16 last_exp_cnt = 1;
	u32 calc_fl[3] = {0};
	int readout_diff = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32 rg_shutters[3] = {0};
	u32 cit_step = 0;
	u32 *shutters = (u32 *)shutter;

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
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	// if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
	alphahmain_write_frame_length(ctx, ctx->frame_length);
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
			subdrv_i2c_wr_u8(ctx,	0x3500, ((rg_shutters[i] * 2) >> 16) & 0xFF);
			subdrv_i2c_wr_u8(ctx,	0x3501, ((rg_shutters[i] * 2) >> 8)  & 0xFF);
			subdrv_i2c_wr_u8(ctx,	0x3502, ((rg_shutters[i] * 2)) & 0xFF);
		}
	}
	DRV_LOG(ctx, "exp[0x%x/0x%x/0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		rg_shutters[0], rg_shutters[1], rg_shutters[2],
		frame_length, ctx->frame_length, ctx->autoflicker_en);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		// commit_i2c_buffer(ctx);
	}
	/* group hold end */
}

static int alphahmain_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	alphahmain_set_multi_shutter_frame_length(ctx, (u64 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

static void alphahmain_set_dummy(struct subdrv_ctx *ctx)
{
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		              ctx->dummy_line, ctx->dummy_pixel);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	if (ctx->current_scenario_id == 4 || ctx->current_scenario_id == 5) {
		subdrv_i2c_wr_u8(ctx, 0x3840, ctx->frame_length >> 16);
		subdrv_i2c_wr_u8(ctx, 0x380e, ctx->frame_length >>  8);
		subdrv_i2c_wr_u8(ctx, 0x380f, ctx->frame_length & 0xFF);
	} else {
		subdrv_i2c_wr_u8(ctx, 0x3840, (ctx->frame_length * 2) >> 16);
		subdrv_i2c_wr_u8(ctx, 0x380e, (ctx->frame_length * 2) >>  8);
		subdrv_i2c_wr_u8(ctx, 0x380f, (ctx->frame_length * 2) & 0xFF);
	}
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
}

static void alphahmain_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	alphahmain_set_dummy(ctx);
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

static int alphahmain_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u64* feature_data = (u64*)para;
    u32 gain = *feature_data;
    u32 reg_gain;
    bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
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

    if (gph && !ctx->ae_ctrl_gph_en)
        ctx->s_ctx.s_gph((void *)ctx, 1);
    ret = subdrv_i2c_wr_u8(ctx, 0x3508, (reg_gain >> 8));
    ret = subdrv_i2c_wr_u8(ctx, 0x3509, (reg_gain & 0xff));
    if (gph)
        ctx->s_ctx.s_gph((void *)ctx, 0);
    DRV_LOG(ctx, "cdb_test read gain = %x:%x\n ", subdrv_i2c_rd_u8(ctx, 0x3508), subdrv_i2c_rd_u8(ctx, 0x3509));
    //videoGainDiffNormal = false;
    return gain;
}

static void alphahmain_write_shutter(struct subdrv_ctx *ctx)
{
	uint16_t realtime_fps = 0;
	static u32 lastshutter = 0;
	int ret = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

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
	if (ctx->exposure[0] > ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin) {
		ctx->exposure[0] = ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin;
	}

	ctx->exposure[0] = (ctx->exposure[0] >> 1) << 1;
	ctx->frame_length = (ctx->frame_length >> 1) << 1;

	if (ctx->current_scenario_id == 4 || ctx->current_scenario_id == 5) {
		if ((lastshutter <= ctx->frame_length) && (ctx->frame_length <= (lastshutter + 6))) {
			ctx->frame_length = lastshutter + 8;
		}
	} else {
		if ((lastshutter <= ctx->frame_length) && (ctx->frame_length <= (lastshutter + 3))) {
			ctx->frame_length = lastshutter + 4;
		}
	}

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps > 297 && realtime_fps <= 305) {
			alphahmain_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 147 && realtime_fps <= 150) {
			alphahmain_set_max_framerate(ctx, 146, 0);
		}
	}
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);

	if (ctx->current_scenario_id == 4 || ctx->current_scenario_id == 5) {
		ret = subdrv_i2c_wr_u8(ctx, 0x3840, ctx->frame_length >> 16);
		ret = subdrv_i2c_wr_u8(ctx, 0x380e, ctx->frame_length >>  8);
		ret = subdrv_i2c_wr_u8(ctx, 0x380f, ctx->frame_length & 0xFF);

		ret = subdrv_i2c_wr_u8(ctx, 0x3500, (ctx->exposure[0] >> 16) & 0xFF);
		ret = subdrv_i2c_wr_u8(ctx, 0x3501, (ctx->exposure[0] >>  8) & 0xFF);
		ret = subdrv_i2c_wr_u8(ctx, 0x3502, (ctx->exposure[0])  & 0xFF);
	} else {
		ret = subdrv_i2c_wr_u8(ctx, 0x3840, (ctx->frame_length * 2) >> 16);
		ret = subdrv_i2c_wr_u8(ctx, 0x380e, (ctx->frame_length * 2) >>  8);
		ret = subdrv_i2c_wr_u8(ctx, 0x380f, (ctx->frame_length * 2) & 0xFF);

		ret = subdrv_i2c_wr_u8(ctx, 0x3500, ((ctx->exposure[0] * 2) >> 16) & 0xFF);
		ret = subdrv_i2c_wr_u8(ctx, 0x3501, ((ctx->exposure[0] * 2) >>  8) & 0xFF);
		ret = subdrv_i2c_wr_u8(ctx, 0x3502, ((ctx->exposure[0] * 2)) & 0xFF);
	}

	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		// commit_i2c_buffer(ctx);
	}
    lastshutter = ctx->exposure[0];

    DRV_LOG(ctx, "cdb_test set  shutter =%x:%x:%x\n", (ctx->exposure[0] >> 16) & 0xFF, (ctx->exposure[0] >>  8) & 0xFF, (ctx->exposure[0])  & 0xFF);
    DRV_LOG(ctx, "cdb_test read shutter  %x:%x:%x \n ", subdrv_i2c_rd_u8(ctx, 0x3500), subdrv_i2c_rd_u8(ctx, 0x3501), subdrv_i2c_rd_u8(ctx, 0x3502));

}	/*	write_shutter  */

static int alphahmain_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u32 *shutter = (u32*)para;
    ctx->exposure[0] = *shutter;
    alphahmain_write_shutter(ctx);
	return 0;
}

static void alphahmain_write_frame_length(struct subdrv_ctx *ctx, u32 fll)
{
	u32 fll_step = 0;
	u32 dol_cnt = 1;

	check_current_scenario_id_bound(ctx);
	fll_step = ctx->s_ctx.mode[ctx->current_scenario_id].framelength_step;
	if (fll_step)
		fll = roundup(fll, fll_step);
	ctx->frame_length = fll;

	if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_STAGGER)
		dol_cnt = ctx->s_ctx.mode[ctx->current_scenario_id].exp_cnt;

	fll = fll / dol_cnt;

	if (ctx->extend_frame_length_en == FALSE) {
		if (ctx->current_scenario_id == 4 || ctx->current_scenario_id == 5) {
			subdrv_i2c_wr_u8(ctx, 0x3840, ctx->frame_length >> 16);
			subdrv_i2c_wr_u8(ctx, 0x380e, ctx->frame_length >>  8);
			subdrv_i2c_wr_u8(ctx, 0x380f, ctx->frame_length & 0xFF);
		} else {
			subdrv_i2c_wr_u8(ctx, 0x3840, (ctx->frame_length * 2) >> 16);
			subdrv_i2c_wr_u8(ctx, 0x380e, (ctx->frame_length * 2) >>  8);
			subdrv_i2c_wr_u8(ctx, 0x380f, (ctx->frame_length * 2) & 0xFF);
		}
		/* update FL RG value after setting buffer for writting RG */
		ctx->frame_length_rg = ctx->frame_length;

		DRV_LOG(ctx,
			"ctx:(fl(RG):%u), fll[0x%x] multiply %u, fll_step:%u\n",
			ctx->frame_length_rg, fll, dol_cnt, fll_step);
	}
}

static int alphahmain_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u16 *feature_data = (u16*)para;
	u16 frame_length = *feature_data;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	if (frame_length)
		ctx->frame_length = frame_length;
	ctx->frame_length = max(ctx->frame_length, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	alphahmain_write_frame_length(ctx, ctx->frame_length);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);

	DRV_LOG(ctx, "fll(input/output/min):%u/%u/%u\n",
		frame_length, ctx->frame_length, ctx->min_frame_length);

	return 0;
}

static void alphahmain_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 *shutter, u32 frame_length, bool auto_extend_en)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	static u32 lastshutter = 0;
	int ret = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	ctx->exposure[0] = *shutter;

	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;

	ctx->frame_length = ctx->frame_length + dummy_line;

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
	if (ctx->exposure[0] > ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin) {
		ctx->exposure[0] = ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin;
	}

	ctx->exposure[0] = (ctx->exposure[0] >> 1) << 1;
	ctx->frame_length = (ctx->frame_length >> 1) << 1;

	if (ctx->current_scenario_id == 4 || ctx->current_scenario_id == 5) {
		if ((lastshutter <= ctx->frame_length) && (ctx->frame_length <= (lastshutter + 6))) {
			ctx->frame_length = lastshutter + 8;
		}
	} else {
		if ((lastshutter <= ctx->frame_length) && (ctx->frame_length <= (lastshutter + 3))) {
			ctx->frame_length = lastshutter + 4;
		}
	}

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps > 297 && realtime_fps <= 305) {
			alphahmain_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 147 && realtime_fps <= 150) {
			alphahmain_set_max_framerate(ctx, 146, 0);
		}
	}
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);

	if (ctx->current_scenario_id == 4 || ctx->current_scenario_id == 5) {
		ret = subdrv_i2c_wr_u8(ctx, 0x3840, ctx->frame_length >> 16);
		ret = subdrv_i2c_wr_u8(ctx, 0x380e, ctx->frame_length >>  8);
		ret = subdrv_i2c_wr_u8(ctx, 0x380f, ctx->frame_length & 0xFF);

		ret = subdrv_i2c_wr_u8(ctx, 0x3500, (ctx->exposure[0] >> 16) & 0xFF);
		ret = subdrv_i2c_wr_u8(ctx, 0x3501, (ctx->exposure[0] >>  8) & 0xFF);
		ret = subdrv_i2c_wr_u8(ctx, 0x3502, (ctx->exposure[0])  & 0xFF);
	} else {
		ret = subdrv_i2c_wr_u8(ctx, 0x3840, (ctx->frame_length * 2) >> 16);
		ret = subdrv_i2c_wr_u8(ctx, 0x380e, (ctx->frame_length * 2) >>  8);
		ret = subdrv_i2c_wr_u8(ctx, 0x380f, (ctx->frame_length * 2) & 0xFF);

		ret = subdrv_i2c_wr_u8(ctx, 0x3500, ((ctx->exposure[0] * 2) >> 16) & 0xFF);
		ret = subdrv_i2c_wr_u8(ctx, 0x3501, ((ctx->exposure[0] * 2) >>  8) & 0xFF);
		ret = subdrv_i2c_wr_u8(ctx, 0x3502, ((ctx->exposure[0] * 2))  & 0xFF);
	}

	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		// commit_i2c_buffer(ctx);
	}
    lastshutter = ctx->exposure[0];

    DRV_LOG(ctx, "cdb_test set  shutter =%x:%x:%x\n", (ctx->exposure[0] >> 16) & 0xFF, (ctx->exposure[0] >>  8) & 0xFF, (ctx->exposure[0])  & 0xFF);
    DRV_LOG(ctx, "cdb_test read shutter  %x:%x:%x \n ", subdrv_i2c_rd_u8(ctx, 0x3500), subdrv_i2c_rd_u8(ctx, 0x3501), subdrv_i2c_rd_u8(ctx, 0x3502));

}	/* set_shutter_frame_length */

static int alphahmain_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "shutter:%u, frame_length:%u\n", (u32)(*para), (u32) (*(para + 1)));
	alphahmain_set_shutter_frame_length_convert(ctx, (u32 *)para, (u32) (*(para + 1)), (u16) (*(para + 2)));
	return 0;
}

static void alphahmain_write_pdc_data(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E!\n");

	if (ctx->s_ctx.eeprom_info->preload_pdc_table) {
		alphahmain_burst_write_cmos_sensor(ctx, SENSOR_PDC_ADDR, &ctx->s_ctx.eeprom_info->preload_pdc_table[8], OTP_PDC_SIZE - 8);
		DRV_LOG(ctx, "Cloud burning Data Writing!\n");
	}
	else {
		alphahmain_burst_write_cmos_sensor(ctx, SENSOR_PDC_ADDR, &otp_pdc_data[8], OTP_PDC_SIZE - 8);
		DRV_LOG(ctx, "Otp Data Writing!\n");
	}

	DRV_LOG(ctx, "X!\n");
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	subdrv_i2c_wr_u8(ctx, 0x0301, 0x01);
	/* initail setting */
	subdrv_i2c_wr_u8(ctx, 0x0103, 0x01);
	sensor_init(ctx);
	/* PDC setting */
	alphahmain_write_pdc_data(ctx);
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
		subdrv_i2c_wr_u8(ctx, 0x3208, 0x01);
	} else {
		subdrv_i2c_wr_u8(ctx, 0x3208, 0x11);
		subdrv_i2c_wr_u8(ctx, 0x3208, 0xa1);
	}
}

static u16 get_gain2reg(u32 gain)
{
	return (16384 - (16384 * BASEGAIN) / gain);
}

void alphahmain_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	u32 exp_cnt = 0;
	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	check_current_scenario_id_bound(ctx);
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

int alphahmain_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	alphahmain_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return 0;
}

static int alphahmain_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM pre_seamless_scenario_id;
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 exp_cnt = 0;

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

static int alphahmain_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode) {
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
		switch (mode) {
		case 5:
			subdrv_i2c_wr_u8(ctx, 0x430b, 0x00); /* dig_gain = 0 */
			subdrv_i2c_wr_u8(ctx, 0x430c, 0x00);
			subdrv_i2c_wr_u8(ctx, 0x4310, 0x00);
			subdrv_i2c_wr_u8(ctx, 0x4311, 0x00);
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

	if (!prsh) {
		DRV_LOGE(ctx, "prsh param is NULL");
		return;
	}

	*prsh = 0U;
	if (alphahmain_comp_params[ctx->current_scenario_id].clock_vtpxck == 0) {
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
		alphahmain_comp_params[ctx->current_scenario_id].clock_vtpxck;
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

static int alphahmain_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 frame_length;
	u32 frame_length_step;
	u32 frame_length_min;
	u32 frame_length_max;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)(*para);
	u64 *feature_framerate = (u64 *)para;
	u32 framerate = *(u32 *)(feature_framerate + 1);

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}
	if (!framerate) {
		DRV_LOG(ctx, "framerate (%u) is invalid\n", framerate);
		return 0;
	}
	if (!ctx->s_ctx.mode[scenario_id].linelength) {
		DRV_LOG(ctx, "linelength (%u) is invalid\n",
			ctx->s_ctx.mode[scenario_id].linelength);
		return 0;
	}
	frame_length_step = ctx->s_ctx.mode[scenario_id].framelength_step;
	/* set on the step of frame length */
	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	frame_length = frame_length_step ?
		(frame_length - (frame_length % frame_length_step)) : frame_length;
	frame_length_min = ctx->s_ctx.mode[scenario_id].framelength;
	frame_length_max = ctx->s_ctx.frame_length_max;
	frame_length_max = frame_length_step ?
		(frame_length_max - (frame_length_max % frame_length_step)) : frame_length_max;
	/* set in the range of frame length */
	ctx->frame_length = max(frame_length, frame_length_min);
	ctx->frame_length = min(ctx->frame_length, frame_length_max);
	ctx->frame_length = frame_length_step ?
		roundup(ctx->frame_length,frame_length_step) : ctx->frame_length;

	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	DRV_LOG(ctx, "max_fps(input/output):%u/%u(sid:%u), min_fl_en:1, ctx->frame_length:%u\n",
		framerate, ctx->current_fps, scenario_id, ctx->frame_length);
	if (ctx->s_ctx.reg_addr_auto_extend ||
			(ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin))) {
		alphahmain_set_dummy(ctx);
	}
	return 0;
}

static int alphahmain_set_cali_data(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	u16 idx = 0;
	u8 support = FALSE;
	u8 *buf = NULL;
	u32 size = 0;
	u8 pdc_is_valid = 0;
	u8 type = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (ctx->is_read_preload_eeprom >= 2) {
        return 0;
	}
	type = para[(*len) + 1];
	if (type == PDC_TYPE) {
		/* PDC data */
		support = info[idx].pdc_support;
		info[idx].pdc_size = *len;
		size = *len;
		buf = para;
		if (support && size > 0 && buf != NULL) {
			/* Check PDC validation */
			pdc_is_valid = buf[*len];
			if (pdc_is_valid == PDC_IS_VALID_VAL) {
				DRV_LOG(ctx, "PDC data is valid, flag(%02x)", pdc_is_valid);
				if (info[idx].preload_pdc_table == NULL) {
					info[idx].preload_pdc_table = kmalloc(size, GFP_KERNEL);
				}
				memcpy(info[idx].preload_pdc_table, buf, size);
				ctx->is_read_preload_eeprom = 2;
				DRV_LOG(ctx, "preload PDC data %u bytes", size);
			} else {
				DRV_LOGE(ctx, "PDC data is invalid, flag(%02x)", pdc_is_valid);
			}
		}
	}

	return 0;
}
