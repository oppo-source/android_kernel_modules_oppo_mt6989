// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 nvwatelemipiraw_Sensor.c
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
#include "nvwatelemipiraw_Sensor.h"
#define NVWATELE_EEPROM_READ_ID	0xC3
#define NVWATELE_EEPROM_WRITE_ID  0xC3
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "nvwatele_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x4000
#define OTP_QCOM_PDAF_DATA_LENGTH 0x16A6
#define OTP_QCOM_PDAF_DATA_START_ADDR 0xC600

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static kal_uint8 otp_qcom_pdaf_data[OTP_QCOM_PDAF_DATA_LENGTH] = {0};
static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static void nvwatele_set_dummy(struct subdrv_ctx *ctx);
static int nvwatele_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static u16 get_gain2reg(u32 gain);
static int nvwatele_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwatele_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwatele_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwatele_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwatele_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwatele_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int nvwatele_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int nvwatele_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, nvwatele_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, nvwatele_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, nvwatele_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, nvwatele_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, nvwatele_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, nvwatele_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, nvwatele_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_DATA, nvwatele_get_otp_qcom_pdaf_data},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, nvwatele_get_min_shutter_by_scenario_adapter},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x00E90114,
		.addr_header_id = 0x0000C006,
		.i2c_write_id = 0xC3,

		.pdc_support = TRUE,
		.pdc_size = 720,
		.addr_pdc = 0xD90C,
		.sensor_reg_addr_pdc = 0x5F80,

		.xtalk_support = TRUE,
		.xtalk_size = 288,
		.addr_xtalk = 0xFA00,
		.sensor_reg_addr_xtalk = 0x5A40,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 40,
	.i4OffsetY = 16,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{47, 18}, {55, 18}, {43, 22}, {51, 22},
	           {47, 26}, {55, 26}, {43, 30}, {51, 30}},
	.i4PosR = {{46, 18}, {54, 18}, {42, 22}, {50, 22},
	           {46, 26}, {54, 26}, {42, 30}, {50, 30}},
	.i4BlockNumX = 284,
	.i4BlockNumY = 215,
	.i4Crop = {
		// <prev> <cap> <vid> <hs_vid> <slim_vid>
		{8, 8}, {8, 8}, {8, 440}, {8, 440}, {2320, 1744},
		// <cust1> <cust2> <cust3> <cust4> <cust5>
		{2320, 1744}, {8, 8}, {0, 4}, {8, 440}, {8, 8},
		// <cust6> <cust7><cust8>
		{0, 0}, {0, 0},{0, 2},
	},
	.iMirrorFlip = 2,
	.i4FullRawW = 4624,
	.i4FullRawH = 3472,
	.i4VCPackNum = 1,
	.sPDMapInfo[0] = {
		.i4PDPattern = 2,
		.i4PDRepetition = 2,
		.i4PDOrder = {1}, // R = 1, L = 0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_vid = {
	.i4OffsetX = 40,
	.i4OffsetY = 16,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{47, 18}, {55, 18}, {43, 22}, {51, 22},
	           {47, 26}, {55, 26}, {43, 30}, {51, 30}},
	.i4PosR = {{46, 18}, {54, 18}, {42, 22}, {50, 22},
	           {46, 26}, {54, 26}, {42, 30}, {50, 30}},
	.i4BlockNumX = 284,
	.i4BlockNumY = 162,
	.i4Crop = {
		// <prev> <cap> <vid> <hs_vid> <slim_vid>
		{8, 8}, {8, 8}, {8, 440}, {8, 440}, {2320, 1744},
		// <cust1> <cust2> <cust3> <cust4> <cust5>
		{2320, 1744}, {8, 8}, {0, 4}, {8, 440}, {8, 8},
		// <cust6> <cust7><cust8>
		{0, 0}, {0, 0},{0, 2},
	},
	.iMirrorFlip = 2,
	.i4FullRawW = 4624,
	.i4FullRawH = 3472,
	.i4VCPackNum = 1,
	.sPDMapInfo[0] = {
		.i4PDPattern = 2,
		.i4PDRepetition = 2,
		.i4PDOrder = {1}, // R = 1, L = 0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_full = {
	.i4OffsetX = 80,
	.i4OffsetY = 32,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 4,
	.i4SubBlkW = 16,
	.i4SubBlkH = 4,
	.i4PosL = {{93, 35}, {93, 37}, {85, 43}, {85, 45}},
	.i4PosR = {{92, 35}, {92, 37}, {84, 43}, {84, 45}},
	.i4BlockNumX = 568,
	.i4BlockNumY = 430,
	.i4Crop = {
		// <prev> <cap> <vid> <hs_vid> <slim_vid>
		{8, 8}, {8, 8}, {8, 440}, {8, 440}, {2320, 1744},
		// <cust1> <cust2> <cust3> <cust4> <cust5>
		{2320, 1744}, {8, 8}, {0, 4}, {8, 440}, {8, 8},
		// <cust6> <cust7><cust8>
		{0, 0}, {0, 0},{0, 2},
	},
	.iMirrorFlip = 2,
	.i4FullRawW = 9248,
	.i4FullRawH = 6944,
	.i4VCPackNum = 1,
	.sPDMapInfo[0] = {
		.i4PDPattern = 2,
		.i4PDRepetition = 4,
		.i4PDOrder = {1}, // R = 1, L = 0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_full_crop = {
	.i4OffsetX = 80,
	.i4OffsetY = 32,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 4,
	.i4SubBlkW = 16,
	.i4SubBlkH = 4,
	.i4PosL = {{93, 35}, {93, 37}, {85, 43}, {85, 45}},
	.i4PosR = {{92, 35}, {92, 37}, {84, 43}, {84, 45}},
	.i4BlockNumX = 288,
	.i4BlockNumY = 216,
	.i4Crop = {
		// <prev> <cap> <vid> <hs_vid> <slim_vid>
		{8, 8}, {8, 8}, {8, 440}, {8, 440}, {2320, 1744},
		// <cust1> <cust2> <cust3> <cust4> <cust5>
		{2320, 1744}, {8, 8}, {0, 4}, {8, 440}, {8, 8},
		// <cust6> <cust7><cust8>
		{0, 0}, {0, 0},{0, 2},
	},
	.iMirrorFlip = 2,
	.i4FullRawW = 9248,
	.i4FullRawH = 6944,
	.i4VCPackNum = 1,
	.sPDMapInfo[0] = {
		.i4PDPattern = 2,
		.i4PDRepetition = 1,
		.i4PDOrder = {1}, // R = 1, L = 0
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1200,
			.vsize = 0x0d80,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0470,
			.vsize = 0x035c,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1200,
			.vsize = 0x0d80,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0470,
			.vsize = 0x035c,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1200,
			.vsize = 0x0a20,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0470,
			.vsize = 0x0288,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1200,
			.vsize = 0x0a20,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0470,
			.vsize = 0x0288,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1200,
			.vsize = 0x0d80,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0240,
			.vsize = 0x0360,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1200,
			.vsize = 0x0d80,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0240,
			.vsize = 0x0360,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1200,
			.vsize = 0x0d80,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0470,
			.vsize = 0x035c,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2420,
			.vsize = 0x1b18,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0470,
			.vsize = 0x06b8,
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
			.hsize = 0x1200,
			.vsize = 0x0a20,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1200,
			.vsize = 0x0a20,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0470,
			.vsize = 0x0288,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1200,
			.vsize = 0x0d80,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0470,
			.vsize = 0x035c,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x780,
			.vsize = 0x438,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x500,
			.vsize = 0x2d0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0d8c,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0470,
			.vsize = 0x035c,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{	/*Reg-Mode0: 4608x3456@30fps*/
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = nvwatele_preview_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_preview_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = nvwatele_seamless_normal_preview,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwatele_seamless_normal_preview),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1008,
		.framelength = 3810,
		.max_framerate = 300,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 16,
			.y0_offset = 16,
			.w0_size = 9216,
			.h0_size = 6912,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1470,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Reg-Mode0: 4608x3456@30fps*/
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = nvwatele_capture_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_capture_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = nvwatele_seamless_normal_preview,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwatele_seamless_normal_preview),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1008,
		.framelength = 3810,
		.max_framerate = 300,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 16,
			.y0_offset = 16,
			.w0_size = 9216,
			.h0_size = 6912,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1470,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Reg-Mode6_4608x2592_30fps*/
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = nvwatele_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_normal_video_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = nvwatele_seamless_normal_video,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwatele_seamless_normal_video),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1008,
		.framelength = 3810,
		.max_framerate = 300,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 8,
			.y1_offset = 440,
			.w1_size = 4608,
			.h1_size = 2592,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4608,
			.h2_tg_size = 2592,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_vid,
		.ae_binning_ratio = 1470,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Reg-Mode5_4608x2592_60fps*/
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = nvwatele_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 696,
		.framelength = 2758,
		.max_framerate = 600,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 8,
			.y1_offset = 440,
			.w1_size = 4608,
			.h1_size = 2592,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4608,
			.h2_tg_size = 2592,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_vid,
		.ae_binning_ratio = 1470,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{	/*Reg-Mode1_Center_Crop_4608x3456_30fps_4C*/
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = nvwatele_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_slim_video_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = nvwatele_seamless_slim_video,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwatele_seamless_slim_video),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1008,
		.framelength = 3808,
		.max_framerate = 300,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 8,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 2320,
			.y0_offset = 1744,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_full_crop,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Reg-Mode2_Center_Crop_4608x3456_30fps_bayer*/
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = nvwatele_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_custom1_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = nvwatele_seamless_custom1,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwatele_seamless_custom1),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1008,
		.framelength = 3808,
		.max_framerate = 300,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 8,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 2320,
			.y0_offset = 1744,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_full_crop,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Reg-Mode8_4608x3456_24fps-max_vbank*/
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = nvwatele_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 696,
		.framelength = 6900,
		.max_framerate = 240,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 16,
			.y0_offset = 16,
			.w0_size = 9216,
			.h0_size = 6912,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1470,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Reg-Mode3_9248x6936_15.00fps*/
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = nvwatele_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_custom3_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = nvwatele_seamless_custom3,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwatele_seamless_custom3),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1080,
		.framelength = 7108,
		.max_framerate = 150,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 8,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 9248,
			.scale_h = 6944,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 9248,
			.h1_size = 6944,
			.x2_tg_offset = 0,
			.y2_tg_offset = 4,
			.w2_tg_size = 9248,
			.h2_tg_size = 6936,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Reg-Mode7_4608x2592_2sHDR_30fps*/
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = nvwatele_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_custom4_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = nvwatele_seamless_custom4,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(nvwatele_seamless_custom4),
		.hdr_mode = HDR_RAW_STAGGER,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 115200000,
		.linelength = 696,
		.framelength = 2758*2,
		.max_framerate = 300,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 2600*2,
		.read_margin = 10*2,
		.framelength_step = 2*2,
		.coarse_integ_step = 2*2,
		.min_exposure_line = 4*2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4*2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = 2592*2,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 8,
			.y1_offset = 440,
			.w1_size = 4608,
			.h1_size = 2592,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4608,
			.h2_tg_size = 2592,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_vid,
		.ae_binning_ratio = 1470,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Mode4_4608x3456_46fps*/
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = nvwatele_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_custom5_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 696,
		.framelength = 3600,
		.max_framerate = 460,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 16,
			.y0_offset = 16,
			.w0_size = 9216,
			.h0_size = 6912,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1470,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Reg_Mode11_1920x1080_240fps*/
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = nvwatele_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 336,
		.framelength = 1428,
		.max_framerate = 2400,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 784,
			.y0_offset = 1312,
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
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 2940,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*Reg_Mode12_1280x720_480fps*/
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = nvwatele_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_custom7_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 300,
		.framelength = 800,
		.max_framerate = 4800,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 2064,
			.y0_offset = 2032,
			.w0_size = 5120,
			.h0_size = 2880,
			.scale_w = 1280,
			.scale_h = 720,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1280,
			.h1_size = 720,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1280,
			.h2_tg_size = 720,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 2940,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
	{   /*old Reg-Mode0_4624x3468_30fps*/
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = nvwatele_custom8_setting,
		.mode_setting_len = ARRAY_SIZE(nvwatele_custom8_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1008,
		.framelength = 3810,
		.max_framerate = 300,
		.mipi_pixel_rate = 1158170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 4,
			.w0_size = 9248,
			.h0_size = 6936,
			.scale_w = 4624,
			.scale_h = 3468,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3468,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3468,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1470,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.dpc_enabled = TRUE,
		.pdc_enabled = TRUE,
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = NVWATELE_SENSOR_ID,
	.reg_addr_sensor_id = {0x300A, 0x300B, 0x300C},
	.i2c_addr_table = {0x20, 0xFF}, // TBD
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {9248, 6944},
	.mirror = IMAGE_V_MIRROR, // TBD

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 15.5,
	.ana_gain_type = 1,
	.ana_gain_step = 4,
	.ana_gain_table = nvwatele_ana_gain_table,
	.ana_gain_table_size = sizeof(nvwatele_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max = 0xFFFFFF - 31,
	.exposure_step = 2,
	.exposure_margin = 31,

	.frame_length_max = 0xFFFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 1640000,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,

	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED, // TBD
	.reg_addr_exposure = {{0x3500, 0x3501, 0x3502},
				{0x3580, 0x3581, 0x3582},
				{0x3540, 0x3541, 0x3542}},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x3508, 0x3509}, {0x3588, 0x3589}, {0x3548, 0x3549}},
	.reg_addr_frame_length = {0x3840, 0x380E, 0x380F},
	.reg_addr_temp_en = 0x4D12,
	.reg_addr_temp_read = 0x4D13,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x387f, // To be verified

	.init_setting_table = nvwatele_init_setting,
	.init_setting_len = ARRAY_SIZE(nvwatele_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 0,
	.chk_s_off_end = 0,

	.checksum_value = 0x37E5E8C5,
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
	.vsync_notify = vsync_notify,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 0},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 1},
	{HW_ID_DOVDD, 1804000, 5},
	{HW_ID_DVDD, 1104000, 0},
	{HW_ID_AVDD, 2804000, 0},
	{HW_ID_AVDD1, 2804000, 0},
	{HW_ID_AFVDD, 2804000, 3},
	{HW_ID_RST, 1, 5}
};

const struct subdrv_entry nvwatele_mipi_raw_entry = {
	.name = "nvwatele_mipi_raw",
	.id = NVWATELE_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
static struct eeprom_map_info nvwatele_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008,0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C1, 0x00C2, 17, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x009A, 0x009B, 6, true },
	{ EEPROM_META_AF_FLAG, 0x009A, 0x009A, 0x009B, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x0000, 0x0000, 0x0000, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0XDD00, 0x0000, 0x0000, CALI_DATA_SLAVE_TELE_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA_105CM, 0XE400, 0x0000, 0x0000, CALI_DATA_SLAVE_TELE_LENGTH, false },
};

static unsigned int read_nvwatele_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;
	if (meta_id != nvwatele_eeprom_info[meta_id].meta)
		return -1;

	if (size != nvwatele_eeprom_info[meta_id].size)
		return -1;

	addr = nvwatele_eeprom_info[meta_id].start;
	readsize = nvwatele_eeprom_info[meta_id].size;

	if (!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}


 static struct eeprom_addr_table_struct  oplus_eeprom_addr_table =
{
    .i2c_read_id = 0xC3,
	.i2c_write_id = 0xC2,

	.addr_modinfo = 0xC000,
	.addr_lens = 0xC008,
	.addr_vcm = 0xC00A,
    .addr_modinfoflag = 0xC010,

	.addr_af = 0xC092,
	.addr_afmacro = 0xC092,
	.addr_afinf = 0xC094,
	.addr_afflag = 0xC09A,

	.addr_qrcode = 0xC0B0,
	.addr_qrcodeflag = 0xC0C7,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};


static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct oplus_eeprom_info_struct* infoPtr;
  	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	infoPtr = (struct oplus_eeprom_info_struct*)(para);
	*len = sizeof(oplus_eeprom_info);
	infoPtr->afInfo[0] = (kal_uint8)((infoPtr->afInfo[1] << 4) | (infoPtr->afInfo[0] >> 4));
	infoPtr->afInfo[1] = (kal_uint8)(infoPtr->afInfo[1] >> 4);
	infoPtr->afInfo[2] = (kal_uint8)((infoPtr->afInfo[3] << 4) | (infoPtr->afInfo[2] >> 4));
	infoPtr->afInfo[3] = (kal_uint8)(infoPtr->afInfo[3] >> 4);
	infoPtr->afInfo[4] = (kal_uint8)((infoPtr->afInfo[7] << 4) | (infoPtr->afInfo[6] >> 4));
	infoPtr->afInfo[5] = (kal_uint8)(infoPtr->afInfo[7] >> 4);
	infoPtr->afInfo[6] = (kal_uint8)((infoPtr->afInfo[5] << 4) | (infoPtr->afInfo[4] >> 4));
	infoPtr->afInfo[7] = (kal_uint8)(infoPtr->afInfo[5] >> 4);
	return 0;
}


static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
    kal_uint16 get_byte = 0;

    adaptor_i2c_rd_u8(ctx->i2c_client, NVWATELE_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
    return get_byte;
}


#ifdef WRITE_DATA_MAX_LENGTH
#undef WRITE_DATA_MAX_LENGTH
#endif
#define   WRITE_DATA_MAX_LENGTH     (128)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, NVWATELE_EEPROM_WRITE_ID >> 1,
            addr, para, len);

	return ret;
}

int adaptor_i2c_wr_u32(struct i2c_client *i2c_client, u16 addr, u16 reg, u32 val)
{
	int ret;
	u8 buf[6];
	struct i2c_msg msg;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = (val >> 24) & 0xff;
	buf[3] = (val >> 16) & 0xff;
	buf[4] = (val >> 8) & 0xff;
	buf[5] = val & 0xff;

	msg.addr = addr;
	msg.flags = i2c_client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(i2c_client->adapter, &msg, 1);
	if (ret < 0)
		dev_info(&i2c_client->dev, "i2c transfer failed (%d)\n", ret);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    /*
    MOCHA SEM1217S Write Protection On / off Method:
    Write 0x23016745 to 0x4E04 : Write Protection Off.
    Write 0x00000000 to 0x4E04 : Write Protection On.
    The only way to write is to use 128 byte.
    Write in 128 units at the time of writing. The starting address must be 0xXX00 or 0xXX80.
    */
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg_eeprom = 0x4E04;
    kal_uint16 reg_ois = 0x0000;
    kal_uint16 reg_af = 0x0200;
    if ( enable ) {
        adaptor_i2c_wr_u32(ctx->i2c_client, NVWATELE_EEPROM_WRITE_ID >> 1, reg_eeprom, 0x00000000);
        adaptor_i2c_wr_u8(ctx->i2c_client, NVWATELE_EEPROM_WRITE_ID >> 1, reg_ois, 0x01);
        adaptor_i2c_wr_u8(ctx->i2c_client, NVWATELE_EEPROM_WRITE_ID >> 1, reg_af, 0x01);
    } else {
        adaptor_i2c_wr_u8(ctx->i2c_client, NVWATELE_EEPROM_WRITE_ID >> 1, reg_ois, 0x00); /* OIS off */
        adaptor_i2c_wr_u8(ctx->i2c_client, NVWATELE_EEPROM_WRITE_ID >> 1, reg_af, 0x00); /* AF off */
        adaptor_i2c_wr_u32(ctx->i2c_client, NVWATELE_EEPROM_WRITE_ID >> 1, reg_eeprom, 0x23016745); /* eeprom wr protect off */
    }
    return ret;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
    ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
    kal_int32  ret = ERROR_NONE;
    kal_uint16 data_base, data_length;
    kal_uint32 idx;
    kal_uint8 *pData;
    UINT32 i = 0;
    kal_uint8 remainder = 0;
    if(pStereodata != NULL) {
        LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
            pStereodata->uSensorId,
            pStereodata->uDeviceId,
            pStereodata->baseAddr,
            pStereodata->dataLength);

        data_base = pStereodata->baseAddr;
        data_length = pStereodata->dataLength;
        pData = pStereodata->uData;
        remainder = data_length % WRITE_DATA_MAX_LENGTH;
        if ((pStereodata->uSensorId == NVWATELE_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == NVWATELE_STEREO_START_ADDR || data_base == NVWATELE_STEREO_105CM_START_ADDR)) {
            data_length = remainder > 0 ? data_length + (WRITE_DATA_MAX_LENGTH - remainder) : data_length;
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);
            idx = data_length/WRITE_DATA_MAX_LENGTH;
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
            /* open write protect */
            write_eeprom_protect(ctx, 1);
            msleep(6);
            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");
        } else if ((pStereodata->uSensorId == NVWATELE_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == NVWATELE_AESYNC_START_ADDR)) {
            data_length = remainder > 0 ? data_length + (WRITE_DATA_MAX_LENGTH - remainder) : data_length;
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
            idx = data_length/WRITE_DATA_MAX_LENGTH;
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
            /* open write protect */
            write_eeprom_protect(ctx, 1);
            msleep(6);
            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                read_cmos_eeprom_8(ctx, NVWATELE_AESYNC_START_ADDR),
                read_cmos_eeprom_8(ctx, NVWATELE_AESYNC_START_ADDR+1),
                read_cmos_eeprom_8(ctx, NVWATELE_AESYNC_START_ADDR+2),
                read_cmos_eeprom_8(ctx, NVWATELE_AESYNC_START_ADDR+3),
                read_cmos_eeprom_8(ctx, NVWATELE_AESYNC_START_ADDR+4),
                read_cmos_eeprom_8(ctx, NVWATELE_AESYNC_START_ADDR+5),
                read_cmos_eeprom_8(ctx, NVWATELE_AESYNC_START_ADDR+6),
                read_cmos_eeprom_8(ctx, NVWATELE_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("nvwatele write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int nvwatele_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        *len = (u32)-1; /*write eeprom failed*/
        LOG_INF("ret=%d\n", ret);
    }
    return 0;
}

static int nvwatele_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	UINT16 *feature_data_16 = (UINT16 *) para;
	UINT32 *feature_return_para_32 = (UINT32 *) para;
	if(*len > CALI_DATA_SLAVE_LENGTH) {
		*len = CALI_DATA_SLAVE_LENGTH;
	}
	LOG_INF("feature_data mode: %d", *feature_data_16);
	switch (*feature_data_16) {
	case EEPROM_STEREODATA_MT_MAIN_105CM:
		read_nvwatele_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA_105CM,
				(BYTE *)feature_return_para_32, *len);
		break;
	case EEPROM_STEREODATA_MT_MAIN:
	default:
		read_nvwatele_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA,
				(BYTE *)feature_return_para_32, *len);
		break;
	}
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, NVWATELE_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static int nvwatele_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;

	read_cmos_eeprom_p8(ctx, OTP_QCOM_PDAF_DATA_START_ADDR, otp_qcom_pdaf_data, OTP_QCOM_PDAF_DATA_LENGTH);

	memcpy(feature_return_para_32, (UINT32 *)otp_qcom_pdaf_data, sizeof(otp_qcom_pdaf_data));
	*len = sizeof(otp_qcom_pdaf_data);

	return 0;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0xc000, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "read_otp_info end\n");
}

static int nvwatele_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	if (otp_data_checksum[0] == 0) {
		read_otp_info(ctx);
	} else {
		DRV_LOG(ctx, "otp data has already read");
	}
	DRV_LOGE(ctx, "get otp data");
	memcpy(feature_return_para_32, (UINT32 *)otp_data_checksum, sizeof(otp_data_checksum));
	*len = sizeof(otp_data_checksum);
	return 0;
}

static int nvwatele_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}

/* FUNCTION */
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
			DRV_LOG(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x566442) {
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

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

    /* initail setting */
	sensor_init(ctx);

	/*QSC setting*/
	if (ctx->s_ctx.s_cali != NULL) {
		ctx->s_ctx.s_cali((void*)ctx);
	} else {
		write_sensor_Cali(ctx);
	}

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

	/* PDC data */
	support = info[idx].pdc_support;
	if (support) {
		pbuf = info[idx].preload_pdc_table;
		if (pbuf != NULL) {
			size = 720;
			addr = 0x5F80;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			DRV_LOG(ctx, "set PDC calibration data done.");
		}
	}

	/* xtalk data */
	support = info[idx].xtalk_support;
	if (support) {
		pbuf = info[idx].preload_xtalk_table;
		if (pbuf != NULL) {
			size = 288;
			addr = 0x5A40;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			DRV_LOG(ctx, "set xtalk calibration data done.");
		}
	}

	/* xtalk setp2 data*/
	if (support) {
		pbuf = nvwatele_xtalk_setp2_setting;
		if (pbuf != NULL) {
			size = 864;
			addr = XTALK_STEP2_ADDR;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			DRV_LOG(ctx, "set xtalk setp2 calibration data done.");
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	int temperature = 0;

	/*TEMP_SEN_CTL */
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_temp_en, 0x01);
	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);
	temperature = (temperature > 0xC0) ? (temperature - 0x100) : temperature;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature);
	return temperature;
}

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en) {
		set_i2c_buffer(ctx, 0x3208, 0x00);
	} else {
		set_i2c_buffer(ctx, 0x3208, 0x10);
		set_i2c_buffer(ctx, 0x3208, 0xA0);
	}
}

static void nvwatele_set_dummy(struct subdrv_ctx *ctx)
{
	// bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	// if (gph)
	// ctx->s_ctx.s_gph((void *)ctx, 1);
	write_frame_length(ctx, ctx->frame_length);
	// if (gph)
	// ctx->s_ctx.s_gph((void *)ctx, 0);

	commit_i2c_buffer(ctx);
}

void nvwatele_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u set default\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = 0;
	}
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
	} else {
		DRV_LOG(ctx, "over sensor_mode_num[%d], use default", ctx->s_ctx.sensor_mode_num);
		*exposure_step = ctx->s_ctx.exposure_step;
		*min_shutter = ctx->s_ctx.exposure_min;
	}
	DRV_LOG(ctx, "scenario_id[%d] exposure_step[%llu] min_shutter[%llu]\n", scenario_id, *exposure_step, *min_shutter);
}

int nvwatele_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	nvwatele_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return 0;
}

static int nvwatele_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)*feature_data;
	u32 framerate = *(feature_data + 1);
	u32 frame_length, calc_fl, exp_cnt, i;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	if (framerate == 0) {
		DRV_LOG(ctx, "framerate should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->s_ctx.mode[scenario_id].linelength == 0) {
		DRV_LOG(ctx, "linelength should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->line_length == 0) {
		DRV_LOG(ctx, "ctx->line_length should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->frame_length == 0) {
		DRV_LOG(ctx, "ctx->frame_length should not be 0\n");
		return ERROR_NONE;
	}
	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	calc_fl = ctx->exposure[0];
	for (i = 1; i < exp_cnt; i++)
		calc_fl += ctx->exposure[i];
	calc_fl += ctx->s_ctx.exposure_margin*exp_cnt*exp_cnt;

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	DRV_LOG(ctx, "max_fps(input/output):%u/%u(sid:%u), frame_length:%u, calc_fl:%u, min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id, ctx->frame_length, calc_fl);
	if (ctx->frame_length > calc_fl)
		nvwatele_set_dummy(ctx);
	else
		ctx->frame_length = calc_fl;

	return ERROR_NONE;
}

static u16 get_gain2reg(u32 gain)
{
	return gain * 256 / BASEGAIN;
}

static int nvwatele_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	enum IMGSENSOR_HDR_MODE_ENUM scen1_hdr, scen2_hdr;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 exp_cnt = 0;

	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return ERROR_NONE;
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
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return ERROR_NONE;
	}

	scen1_hdr = ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode;
	scen2_hdr = ctx->s_ctx.mode[scenario_id].hdr_mode;
	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	ctx->is_seamless = TRUE;
	update_mode_info(ctx, scenario_id);

	i2c_table_write(ctx, addr_data_pair_seamless_switch_step1_nvwatele,
		ARRAY_SIZE(addr_data_pair_seamless_switch_step1_nvwatele));
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		default:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	i2c_table_write(ctx, addr_data_pair_seamless_switch_step2_nvwatele,
		ARRAY_SIZE(addr_data_pair_seamless_switch_step2_nvwatele));

	if (scen1_hdr == HDR_RAW_STAGGER) {
		i2c_table_write(ctx, addr_data_pair_seamless_switch_step3_HDR_nvwatele,
			ARRAY_SIZE(addr_data_pair_seamless_switch_step3_HDR_nvwatele));
			DRV_LOG(ctx, "do hdr to linear mode\n");
	} else {
		i2c_table_write(ctx, addr_data_pair_seamless_switch_step3_nvwatele,
			ARRAY_SIZE(addr_data_pair_seamless_switch_step3_nvwatele));
			DRV_LOG(ctx, "do linear to hdr/linear mode\n");
	}
	ctx->is_seamless = FALSE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->fast_mode_on = TRUE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}

static int nvwatele_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	if (mode != ctx->test_pattern)
		DRV_LOGE(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	else
		return ERROR_NONE;
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* 1:Solid Color 2:Color Bar 5:Black */
	switch (mode) {
	case 2:
		set_i2c_buffer(ctx, 0x50c1, 0x01);
		break;
	case 5:
		set_i2c_buffer(ctx, 0x350a, 0x00);
		set_i2c_buffer(ctx, 0x401a, 0x00);
		set_i2c_buffer(ctx, 0x3019, 0xf0);
		set_i2c_buffer(ctx, 0x4308, 0x01);
		break;
	default:
		break;
	}

	if (mode != ctx->test_pattern)
		switch (ctx->test_pattern) {
		case 2:
			set_i2c_buffer(ctx, 0x50c1, 0x00);
			break;
		case 5:
			set_i2c_buffer(ctx, 0x350a, 0x01);
			set_i2c_buffer(ctx, 0x401a, 0x40);
			set_i2c_buffer(ctx, 0x3019, 0xd2);
			set_i2c_buffer(ctx, 0x4308, 0x00);
			break;
		default:
			break;
		}
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	commit_i2c_buffer(ctx);
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
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
	}
	return 0;
}


