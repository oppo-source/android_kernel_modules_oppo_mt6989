// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 baikalmfrontmipiraw_Sensor.c
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
#include "baikalmfrontmipiraw_Sensor.h"

#define BAIKALMFRONT_EEPROM_READ_ID	0xA1
#define BAIKALMFRONT_EEPROM_WRITE_ID   0xA0
#define BAIKALMFRONT_MAX_OFFSET		0x4000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "baikalmfront_camera_sensor"
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
static int baikalmfront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_get_eeprom_4cell_info(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static void baikalmfront_write_preview_setting(struct subdrv_ctx *ctx);
static void baikalmfront_write_normal_video_setting(struct subdrv_ctx *ctx);
static void baikalmfront_write_hs_video_setting(struct subdrv_ctx *ctx);
static int baikalmfront_control(struct subdrv_ctx *ctx, enum SENSOR_SCENARIO_ID_ENUM scenario_id,
				MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data);
static void baikalmfront_sensor_init(struct subdrv_ctx *ctx);
static void baikalmfront_write_frame_length(struct subdrv_ctx *ctx, u32 fll);
static int open(struct subdrv_ctx *ctx);
static int close(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void set_sensor_cali(void *arg);
static int baikalmfront_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int baikalmfront_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int baikalmfront_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);

/* STRUCT */
//自定义
static void gc08a8_set_dummy(struct subdrv_ctx *ctx);
static int gc08a8_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static unsigned int read_baikalmfront_eeprom_4cell_info(struct subdrv_ctx *ctx,
	struct oplus_eeprom_4cell_info_struct* oplus_eeprom_4cell_info,
	struct eeprom_4cell_addr_table_struct oplus_eeprom_4cell_addr_table);

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, baikalmfront_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, baikalmfront_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, baikalmfront_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, baikalmfront_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, baikalmfront_streaming_resume},
	{SENSOR_FEATURE_SET_ESHUTTER, baikalmfront_set_shutter},
	{SENSOR_FEATURE_SET_FRAMELENGTH, baikalmfront_set_frame_length},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, baikalmfront_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_GAIN, baikalmfront_set_gain},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, baikalmfront_set_hdr_tri_shutter2},
	{SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, baikalmfront_set_hdr_tri_shutter3},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, baikalmfront_set_multi_shutter_frame_length_ctrl},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_DATA, baikalmfront_get_otp_qcom_pdaf_data},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, gc08a8_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_GET_4CELL_DATA, baikalmfront_get_eeprom_4cell_info},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x015F0131,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA0,
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

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = baikalmfront_preview_setting,
		.mode_setting_len = ARRAY_SIZE(baikalmfront_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000, //828*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
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
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.mode_setting_table = baikalmfront_capture_setting,
		.mode_setting_len = ARRAY_SIZE(baikalmfront_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000, //828*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
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
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.mode_setting_table = baikalmfront_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(baikalmfront_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000, //828*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 612,
			.w0_size = 3264,
			.h0_size = 1836,
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
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.mode_setting_table = baikalmfront_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(baikalmfront_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 1276,
		.max_framerate = 600,
		.mipi_pixel_rate = 134400000, //1600*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
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
		.pdaf_cap = false,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.mode_setting_table = baikalmfront_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(baikalmfront_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 1276,
		.max_framerate = 600,
		.mipi_pixel_rate = 134400000, //828*4/10,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
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
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = BAIKALMFRONT_SENSOR_ID,
	.reg_addr_sensor_id = {0x03f0, 0x03f1},
	.i2c_addr_table = {0x20, 0xff},
	.i2c_burst_write_support = false,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {3264, 2448},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 4, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 1,
	.ana_gain_table = baikalmfront_ana_gain_table,
	.ana_gain_table_size = sizeof(baikalmfront_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0x2420 - 16,//0x3FFE - 64
	.exposure_step = 1,
	.exposure_margin = 16,

	.frame_length_max = 0x2420,//0x3FFE
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 1000000,

	.pdaf_type = PDAF_SUPPORT_NA, //PDAF_SUPPORT_CAMSV,
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

	.init_setting_table = baikalmfront_sensor_init_setting,
	.init_setting_len =  ARRAY_SIZE(baikalmfront_sensor_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
	.checksum_value = 0xE5D32119,
};


static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = baikalmfront_control,
	.feature_control = common_feature_control,
	.close = close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 0},
	{HW_ID_RST, 0, 3},
	{HW_ID_DOVDD, 1800000, 3},
	{HW_ID_DVDD, 1200000, 1},
	{HW_ID_AVDD, 2800000, 3},
	{HW_ID_RST, 1, 1},
};

const struct subdrv_entry baikalmfront_mipi_raw_entry = {
	.name = "baikalmfront_mipi_raw",
	.id = BAIKALMFRONT_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */


static void baikalmfront_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
	 ctx->dummy_line, ctx->dummy_pixel);

	/* return; //for test */
	if (ctx->frame_length < 0xfffe)
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xfffe);
}				/*      set_dummy  */

static void baikalmfront_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	baikalmfront_set_dummy(ctx);
}

static int baikalmfront_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u16* featrue_data = (u16*)para;
	u16 frame_length = *featrue_data;
	if (frame_length)
		ctx->frame_length = frame_length;
	ctx->frame_length = max(ctx->frame_length, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	baikalmfront_write_frame_length(ctx, ctx->frame_length);

	DRV_LOG(ctx, "fll(input/output/min):%u/%u/%u\n",
		frame_length, ctx->frame_length, ctx->min_frame_length);
	return 0;
}

static int baikalmfront_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	kal_uint16 realtime_fps = 0;
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = *(feature_data + 1);
	u32 fine_integ_line = 0;
	DRV_LOG(ctx, "gc08a8_shutter = 0x%x ,frame_length = 0x%x\n", shutter,frame_length);
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
	realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
	/* write framelength&shutter */
	if (set_auto_flicker(ctx, 0) || ctx->frame_length) {
		baikalmfront_set_max_framerate(ctx, realtime_fps, 0);
	}
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],
		(ctx->exposure[0] >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],
		ctx->exposure[0] & 0xFF);

	DRV_LOG(ctx, "gc08a8 exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);

	return 0;
}

static void baikalmfront_write_shutter(struct subdrv_ctx *ctx)
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
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			baikalmfront_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			baikalmfront_set_max_framerate(ctx, 146, 0);
		} else {
			// Extend frame length
			subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xfffe);
		}
	} else {
		// Extend frame length
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xfffe);
	}
	subdrv_i2c_wr_u16(ctx, 0x0202, ctx->exposure[0] & 0xffff);
	DRV_LOG(ctx, "shutter =%d, framelength =%d\n", ctx->exposure[0], ctx->frame_length);
}	/*	write_shutter  */

static void baikalmfront_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	baikalmfront_write_shutter(ctx);
}

static int baikalmfront_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *para);
	baikalmfront_set_shutter_frame_length(ctx, para, len);
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
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		}
	}
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static int baikalmfront_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			baikalmfront_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int baikalmfront_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
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

static struct eeprom_4cell_addr_table_struct oplus_eeprom_4cell_addr_table = {
	.addr_sxtc = 0x2300,
	.addr_pdxtc = 0x2612,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};
static struct oplus_eeprom_4cell_info_struct  oplus_eeprom_4cell_info = {0};

static unsigned int read_baikalmfront_eeprom_4cell_info(struct subdrv_ctx *ctx,
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

static int baikalmfront_get_eeprom_4cell_info(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	if (adaptor_i2c_rd_p8(ctx->i2c_client, BAIKALMFRONT_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static int baikalmfront_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int baikalmfront_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int baikalmfront_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
			LOG_INF("i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x08A8) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_baikalmfront_eeprom_4cell_info(ctx, &oplus_eeprom_4cell_info, oplus_eeprom_4cell_addr_table);
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
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

/* SENSOR MIRROR FLIP INFO */
#define GC08A8_MIRROR_NORMAL    1
#define GC08A8_MIRROR_H         0
#define GC08A8_MIRROR_V         0
#define GC08A8_MIRROR_HV        0

#if GC08A8_MIRROR_NORMAL
#define GC08A8_MIRROR	        0x00
#elif GC08A8_MIRROR_H
#define GC08A8_MIRROR	        0x01
#elif GC08A8_MIRROR_V
#define GC08A8_MIRROR	        0x02
#elif GC08A8_MIRROR_HV
#define GC08A8_MIRROR	        0x03
#else
#define GC08A8_MIRROR	        0x00
#endif

static void baikalmfront_sensor_init(struct subdrv_ctx *ctx)
{
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0337, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0335, 0x51);
	subdrv_i2c_wr_u8(ctx, 0x0336, 0x70);
	subdrv_i2c_wr_u8(ctx, 0x0383, 0xbb);
	subdrv_i2c_wr_u8(ctx, 0x031a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0321, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0327, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0325, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x0326, 0x23);
	subdrv_i2c_wr_u8(ctx, 0x0314, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0315, 0xd6);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0334, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x0324, 0x42);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x039a, 0x43);
	subdrv_i2c_wr_u8(ctx, 0x0084, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x02b3, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0057, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x05c3, 0x50);
	subdrv_i2c_wr_u8(ctx, 0x0311, 0x90);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0074, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0059, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0070, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0101, GC08A8_MIRROR);
	subdrv_i2c_wr_u8(ctx, 0x0344, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0345, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0346, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0347, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0348, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0349, 0xd0);
	subdrv_i2c_wr_u8(ctx, 0x034a, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x034b, 0x9c);
	subdrv_i2c_wr_u8(ctx, 0x0202, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0203, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0340, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0341, 0xf4);
	subdrv_i2c_wr_u8(ctx, 0x0342, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0343, 0x1c);
	subdrv_i2c_wr_u8(ctx, 0x0219, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0226, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0227, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0e0a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e0b, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e24, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0e25, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0e26, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e27, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0e01, 0x74);
	subdrv_i2c_wr_u8(ctx, 0x0e03, 0x47);
	subdrv_i2c_wr_u8(ctx, 0x0e04, 0x33);
	subdrv_i2c_wr_u8(ctx, 0x0e05, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x0e06, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x0e0c, 0x1e);
	subdrv_i2c_wr_u8(ctx, 0x0e17, 0x3a);
	subdrv_i2c_wr_u8(ctx, 0x0e18, 0x3c);
	subdrv_i2c_wr_u8(ctx, 0x0e19, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x0e1a, 0x42);
	subdrv_i2c_wr_u8(ctx, 0x0e28, 0x21);
	subdrv_i2c_wr_u8(ctx, 0x0e2b, 0x68);
	subdrv_i2c_wr_u8(ctx, 0x0e2c, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0e2d, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0e34, 0xf4);
	subdrv_i2c_wr_u8(ctx, 0x0e35, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x0e36, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0e38, 0x39);
	subdrv_i2c_wr_u8(ctx, 0x0210, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0241, 0x88);
	subdrv_i2c_wr_u8(ctx, 0x0e32, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e33, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x0e42, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0e43, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0e44, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0e45, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e4f, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x057a, 0x20);
	subdrv_i2c_wr_u8(ctx, 0x0381, 0x7c);
	subdrv_i2c_wr_u8(ctx, 0x0382, 0x9b);
	subdrv_i2c_wr_u8(ctx, 0x0384, 0xfb);
	subdrv_i2c_wr_u8(ctx, 0x0389, 0x38);
	subdrv_i2c_wr_u8(ctx, 0x038a, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0390, 0x6a);
	subdrv_i2c_wr_u8(ctx, 0x0391, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0392, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0393, 0xc1);
	subdrv_i2c_wr_u8(ctx, 0x0396, 0x3f);
	subdrv_i2c_wr_u8(ctx, 0x0398, 0x22);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x0360, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0360, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0a67, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a53, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0a65, 0x17);
	subdrv_i2c_wr_u8(ctx, 0x0a68, 0xa1);
	subdrv_i2c_wr_u8(ctx, 0x0a58, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0ace, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x00a4, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00a5, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x00a7, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x00a8, 0x9c);
	subdrv_i2c_wr_u8(ctx, 0x00a9, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x00aa, 0xd0);
	subdrv_i2c_wr_u8(ctx, 0x0a8a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a8b, 0xe0);
	subdrv_i2c_wr_u8(ctx, 0x0a8c, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x0a8d, 0xe8);
	subdrv_i2c_wr_u8(ctx, 0x0a90, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0a91, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0a92, 0xf8);
	subdrv_i2c_wr_u8(ctx, 0x0a71, 0xf2);
	subdrv_i2c_wr_u8(ctx, 0x0a72, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x0a73, 0x64);
	subdrv_i2c_wr_u8(ctx, 0x0a75, 0x41);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x00a0, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0080, 0xd2);
	subdrv_i2c_wr_u8(ctx, 0x0081, 0x3f);
	subdrv_i2c_wr_u8(ctx, 0x0087, 0x51);
	subdrv_i2c_wr_u8(ctx, 0x0089, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x009b, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x0096, 0x81);
	subdrv_i2c_wr_u8(ctx, 0x0097, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0x82);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05ad, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x05ae, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0800, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0801, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0802, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0803, 0x34);
	subdrv_i2c_wr_u8(ctx, 0x0804, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0805, 0x33);
	subdrv_i2c_wr_u8(ctx, 0x0806, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0807, 0x8a);
	subdrv_i2c_wr_u8(ctx, 0x0808, 0x3e);
	subdrv_i2c_wr_u8(ctx, 0x0809, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x080a, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x080b, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080c, 0x1d);
	subdrv_i2c_wr_u8(ctx, 0x080d, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080e, 0x16);
	subdrv_i2c_wr_u8(ctx, 0x080f, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0810, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0811, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0812, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0813, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0814, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0815, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0816, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0817, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0818, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0819, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x081a, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x081b, 0x6c);
	subdrv_i2c_wr_u8(ctx, 0x081c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x081d, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x081e, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x081f, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0820, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0821, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0822, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0823, 0xd9);
	subdrv_i2c_wr_u8(ctx, 0x0824, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0825, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0826, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0827, 0xf0);
	subdrv_i2c_wr_u8(ctx, 0x0828, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0829, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x082a, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x082b, 0x94);
	subdrv_i2c_wr_u8(ctx, 0x082c, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x082d, 0x6e);
	subdrv_i2c_wr_u8(ctx, 0x082e, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x082f, 0xe6);
	subdrv_i2c_wr_u8(ctx, 0x0830, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0831, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0832, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0833, 0x2c);
	subdrv_i2c_wr_u8(ctx, 0x0834, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0835, 0xae);
	subdrv_i2c_wr_u8(ctx, 0x0836, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0837, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0838, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x0839, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x059a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059b, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059c, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0598, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0597, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x05ab, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x05a4, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x05a3, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0xc2);
	subdrv_i2c_wr_u8(ctx, 0x0207, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0208, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0209, 0x78);
	subdrv_i2c_wr_u8(ctx, 0x0204, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0205, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0040, 0x22);
	subdrv_i2c_wr_u8(ctx, 0x0041, 0x20);
	subdrv_i2c_wr_u8(ctx, 0x0043, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0044, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0046, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0047, 0xf0);
	subdrv_i2c_wr_u8(ctx, 0x0048, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x004b, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x004c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0050, 0x5c);
	subdrv_i2c_wr_u8(ctx, 0x0051, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x005b, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x00c0, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00c1, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x00c2, 0x31);
	subdrv_i2c_wr_u8(ctx, 0x00c3, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0460, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0462, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0464, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0466, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0468, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x046a, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x046c, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x046e, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0461, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0463, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0465, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0467, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0469, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x046b, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x046d, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x046f, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0470, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0472, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0474, 0x26);
	subdrv_i2c_wr_u8(ctx, 0x0476, 0x38);
	subdrv_i2c_wr_u8(ctx, 0x0478, 0x20);
	subdrv_i2c_wr_u8(ctx, 0x047a, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x047c, 0x38);
	subdrv_i2c_wr_u8(ctx, 0x047e, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0471, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0473, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0475, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0477, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0479, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x047b, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x047d, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x047f, 0x04);
}

static void baikalmfront_write_preview_setting(struct subdrv_ctx *ctx) {
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0337, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0335, 0x51);
	subdrv_i2c_wr_u8(ctx, 0x0336, 0x70);
	subdrv_i2c_wr_u8(ctx, 0x0383, 0xbb);
	subdrv_i2c_wr_u8(ctx, 0x031a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0321, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0327, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0325, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x0326, 0x23);
	subdrv_i2c_wr_u8(ctx, 0x0314, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0315, 0xd6);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0334, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x0324, 0x42);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x0344, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0345, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0346, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0347, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0348, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0349, 0xd0);
	subdrv_i2c_wr_u8(ctx, 0x034a, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x034b, 0x9c);
	subdrv_i2c_wr_u8(ctx, 0x0202, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0203, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0340, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0341, 0xf4);
	subdrv_i2c_wr_u8(ctx, 0x0342, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0343, 0x1c);
	subdrv_i2c_wr_u8(ctx, 0x0226, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0227, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0e38, 0x39);
	subdrv_i2c_wr_u8(ctx, 0x0210, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0241, 0x88);
	subdrv_i2c_wr_u8(ctx, 0x0392, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x00a2, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00a3, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00ab, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0x82);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05ad, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x05ae, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0800, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0801, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0802, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0803, 0x34);
	subdrv_i2c_wr_u8(ctx, 0x0804, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0805, 0x33);
	subdrv_i2c_wr_u8(ctx, 0x0806, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0807, 0x8a);
	subdrv_i2c_wr_u8(ctx, 0x0808, 0x3e);
	subdrv_i2c_wr_u8(ctx, 0x0809, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x080a, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x080b, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080c, 0x1d);
	subdrv_i2c_wr_u8(ctx, 0x080d, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080e, 0x16);
	subdrv_i2c_wr_u8(ctx, 0x080f, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0810, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0811, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0812, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0813, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0814, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0815, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0816, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0817, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0818, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0819, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x081a, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x081b, 0x6c);
	subdrv_i2c_wr_u8(ctx, 0x081c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x081d, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x081e, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x081f, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0820, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0821, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0822, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0823, 0xd9);
	subdrv_i2c_wr_u8(ctx, 0x0824, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0825, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0826, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0827, 0xf0);
	subdrv_i2c_wr_u8(ctx, 0x0828, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0829, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x082a, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x082b, 0x94);
	subdrv_i2c_wr_u8(ctx, 0x082c, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x082d, 0x6e);
	subdrv_i2c_wr_u8(ctx, 0x082e, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x082f, 0xe6);
	subdrv_i2c_wr_u8(ctx, 0x0830, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0831, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0832, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0833, 0x2c);
	subdrv_i2c_wr_u8(ctx, 0x0834, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0835, 0xae);
	subdrv_i2c_wr_u8(ctx, 0x0836, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0837, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0838, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x0839, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x059a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059b, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059c, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0598, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0597, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x05ab, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x05a4, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x05a3, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0xc2);
	subdrv_i2c_wr_u8(ctx, 0x0207, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0204, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0205, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0050, 0x5c);
	subdrv_i2c_wr_u8(ctx, 0x0051, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x009a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0351, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0352, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0353, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0354, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x034c, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x034d, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x034e, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x034f, 0x90);
	subdrv_i2c_wr_u8(ctx, 0x0114, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0180, 0x67);
	subdrv_i2c_wr_u8(ctx, 0x0181, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x0185, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0115, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x011b, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x011c, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x0121, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0122, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0123, 0x2f);
	subdrv_i2c_wr_u8(ctx, 0x0124, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0125, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x0126, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0129, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x012a, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x012b, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0a73, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00a4, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0a67, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0084, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0102, 0x09);
}

static void baikalmfront_write_normal_video_setting(struct subdrv_ctx *ctx) {
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0337, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0335, 0x51);
	subdrv_i2c_wr_u8(ctx, 0x0336, 0x70);
	subdrv_i2c_wr_u8(ctx, 0x0383, 0xbb);
	subdrv_i2c_wr_u8(ctx, 0x031a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0321, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0327, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0325, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x0326, 0x23);
	subdrv_i2c_wr_u8(ctx, 0x0314, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0315, 0xd6);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0334, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x0324, 0x42);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x0344, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0345, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0346, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0347, 0x36);
	subdrv_i2c_wr_u8(ctx, 0x0348, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0349, 0xd0);
	subdrv_i2c_wr_u8(ctx, 0x034a, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x034b, 0x38);
	subdrv_i2c_wr_u8(ctx, 0x0202, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0203, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0340, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0341, 0xf4);
	subdrv_i2c_wr_u8(ctx, 0x0342, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0343, 0x1c);
	subdrv_i2c_wr_u8(ctx, 0x0226, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0227, 0x8c);
	subdrv_i2c_wr_u8(ctx, 0x0e38, 0x39);
	subdrv_i2c_wr_u8(ctx, 0x0210, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0241, 0x88);
	subdrv_i2c_wr_u8(ctx, 0x0392, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x00a2, 0x32);
	subdrv_i2c_wr_u8(ctx, 0x00a3, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x00ab, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0x82);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05ad, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x05ae, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0800, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0801, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0802, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0803, 0x34);
	subdrv_i2c_wr_u8(ctx, 0x0804, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0805, 0x33);
	subdrv_i2c_wr_u8(ctx, 0x0806, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0807, 0x8a);
	subdrv_i2c_wr_u8(ctx, 0x0808, 0x3e);
	subdrv_i2c_wr_u8(ctx, 0x0809, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x080a, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x080b, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080c, 0x1d);
	subdrv_i2c_wr_u8(ctx, 0x080d, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080e, 0x16);
	subdrv_i2c_wr_u8(ctx, 0x080f, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0810, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0811, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0812, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0813, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0814, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0815, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0816, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0817, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0818, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0819, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x081a, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x081b, 0x6c);
	subdrv_i2c_wr_u8(ctx, 0x081c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x081d, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x081e, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x081f, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0820, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0821, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0822, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0823, 0xd9);
	subdrv_i2c_wr_u8(ctx, 0x0824, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0825, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0826, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0827, 0xf0);
	subdrv_i2c_wr_u8(ctx, 0x0828, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0829, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x082a, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x082b, 0x94);
	subdrv_i2c_wr_u8(ctx, 0x082c, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x082d, 0x6e);
	subdrv_i2c_wr_u8(ctx, 0x082e, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x082f, 0xe6);
	subdrv_i2c_wr_u8(ctx, 0x0830, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0831, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0832, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0833, 0x2c);
	subdrv_i2c_wr_u8(ctx, 0x0834, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0835, 0xae);
	subdrv_i2c_wr_u8(ctx, 0x0836, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0837, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0838, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x0839, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x059a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059b, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059c, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0598, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0597, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x05ab, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x05a4, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x05a3, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0xc2);
	subdrv_i2c_wr_u8(ctx, 0x0207, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0204, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0205, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0050, 0x5c);
	subdrv_i2c_wr_u8(ctx, 0x0051, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x009a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0351, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0352, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0353, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0354, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x034c, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x034d, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x034e, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x034f, 0x2c);
	subdrv_i2c_wr_u8(ctx, 0x0114, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0180, 0x67);
	subdrv_i2c_wr_u8(ctx, 0x0181, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x0185, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0115, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x011b, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x011c, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x0121, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0122, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0123, 0x2f);
	subdrv_i2c_wr_u8(ctx, 0x0124, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0125, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x0126, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0129, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x012a, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x012b, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0a73, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00a4, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0a67, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0084, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0102, 0x09);
}

static void baikalmfront_write_hs_video_setting(struct subdrv_ctx *ctx) {
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0337, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0335, 0x55);
	subdrv_i2c_wr_u8(ctx, 0x0336, 0x70);
	subdrv_i2c_wr_u8(ctx, 0x0383, 0xbb);
	subdrv_i2c_wr_u8(ctx, 0x031a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0321, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0327, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0325, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x0326, 0x23);
	subdrv_i2c_wr_u8(ctx, 0x0314, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0315, 0xd6);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0334, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x0324, 0x42);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x0344, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0345, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0346, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0347, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0348, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0349, 0xd0);
	subdrv_i2c_wr_u8(ctx, 0x034a, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x034b, 0x9c);
	subdrv_i2c_wr_u8(ctx, 0x0202, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0203, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0340, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0341, 0xfc);
	subdrv_i2c_wr_u8(ctx, 0x0342, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0343, 0x1c);
	subdrv_i2c_wr_u8(ctx, 0x0226, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0227, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0e38, 0x39);
	subdrv_i2c_wr_u8(ctx, 0x0210, 0x53);
	subdrv_i2c_wr_u8(ctx, 0x0218, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0241, 0x8c);
	subdrv_i2c_wr_u8(ctx, 0x0392, 0x3b);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x00a2, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00a3, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00ab, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0x82);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05ad, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x05ae, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0800, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0801, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0802, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0803, 0x34);
	subdrv_i2c_wr_u8(ctx, 0x0804, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0805, 0x33);
	subdrv_i2c_wr_u8(ctx, 0x0806, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0807, 0x8a);
	subdrv_i2c_wr_u8(ctx, 0x0808, 0x3e);
	subdrv_i2c_wr_u8(ctx, 0x0809, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x080a, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x080b, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080c, 0x1d);
	subdrv_i2c_wr_u8(ctx, 0x080d, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080e, 0x16);
	subdrv_i2c_wr_u8(ctx, 0x080f, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0810, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0811, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0812, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0813, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0814, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0815, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0816, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0817, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0818, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0819, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x081a, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x081b, 0x6c);
	subdrv_i2c_wr_u8(ctx, 0x081c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x081d, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x081e, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x081f, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0820, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0821, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0822, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0823, 0xd9);
	subdrv_i2c_wr_u8(ctx, 0x0824, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0825, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0826, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0827, 0xf0);
	subdrv_i2c_wr_u8(ctx, 0x0828, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0829, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x082a, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x082b, 0x94);
	subdrv_i2c_wr_u8(ctx, 0x082c, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x082d, 0x6e);
	subdrv_i2c_wr_u8(ctx, 0x082e, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x082f, 0xe6);
	subdrv_i2c_wr_u8(ctx, 0x0830, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0831, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0832, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0833, 0x2c);
	subdrv_i2c_wr_u8(ctx, 0x0834, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0835, 0xae);
	subdrv_i2c_wr_u8(ctx, 0x0836, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0837, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0838, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x0839, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x059a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059b, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059c, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0598, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0597, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x05ab, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x05a4, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x05a3, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0xc2);
	subdrv_i2c_wr_u8(ctx, 0x0207, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0204, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0205, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0050, 0x5c);
	subdrv_i2c_wr_u8(ctx, 0x0051, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x009a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0351, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0352, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0353, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0354, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x034c, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x034d, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x034e, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x034f, 0xc8);
	subdrv_i2c_wr_u8(ctx, 0x0114, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0180, 0x67);
	subdrv_i2c_wr_u8(ctx, 0x0181, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x0185, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0115, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x011b, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x011c, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x0121, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0122, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0123, 0x15);
	subdrv_i2c_wr_u8(ctx, 0x0124, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0125, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0126, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0129, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x012a, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x012b, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0a73, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00a4, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0a67, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0084, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0102, 0x09);
}

static int baikalmfront_control(struct subdrv_ctx *ctx,
			enum SENSOR_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	int ret = ERROR_NONE;
	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u64 time_boot_begin = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;
	struct adaptor_ctx *_adaptor_ctx = NULL;
	struct v4l2_subdev *sd = NULL;

	if (ctx->i2c_client)
		sd = i2c_get_clientdata(ctx->i2c_client);
	if (sd)
		_adaptor_ctx = to_ctx(sd);
	if (!_adaptor_ctx) {
		DRV_LOGE(ctx, "null _adaptor_ctx\n");
		return -ENODEV;
	}

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
		ret = ERROR_INVALID_SCENARIO_ID;
	}
	if (ctx->s_ctx.chk_s_off_sta)
		check_stream_off(ctx);
	update_mode_info(ctx, scenario_id);

	if (ctx->s_ctx.mode[scenario_id].mode_setting_table != NULL) {
		DRV_LOG_MUST(ctx, "E: sid:%u size:%u\n", scenario_id,
			ctx->s_ctx.mode[scenario_id].mode_setting_len);
		if (ctx->power_on_profile_en)
			time_boot_begin = ktime_get_boottime_ns();

		/* initail setting */
		if (ctx->s_ctx.aov_sensor_support) {
			if (ctx->s_ctx.mode[scenario_id].aov_mode &&
				ctx->s_ctx.s_pwr_seq_reset_view_to_sensing != NULL)
				ctx->s_ctx.s_pwr_seq_reset_view_to_sensing((void *) ctx);

		#ifndef OPLUS_FEATURE_CAMERA_COMMON
			if (!ctx->s_ctx.init_in_open)
				baikalmfront_sensor_init(ctx);
		#else /*OPLUS_FEATURE_CAMERA_COMMON*/
			if (!ctx->s_ctx.init_in_open && ctx->s_ctx.mode[scenario_id].aov_mode) {
				sensor_sensing_init(ctx);
			}
		#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
		}

		/* mode setting */
		#ifndef OPLUS_FEATURE_CAMERA_COMMON
		if (ctx->s_ctx.aov_sensor_support &&
			ctx->s_ctx.mode[scenario_id].aov_mode &&
			ctx->sensor_mode_ops == AOV_MODE_CTRL_OPS_MONTION_DETECTION_CTRL) {
		#else /*OPLUS_FEATURE_CAMERA_COMMON*/
		if (ctx->s_ctx.aov_sensor_support &&
			ctx->s_ctx.mode[scenario_id].aov_mode && ctx->s_ctx.mode[scenario_id].max_framerate < 50) {
		#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
			/* AOV MD setting */
			ret = pinctrl_select_state(
				_adaptor_ctx->pinctrl,
				_adaptor_ctx->state[STATE_EINT]);
			switch(scenario_id) {
				case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
				case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
					baikalmfront_write_preview_setting(ctx);
					break;
				case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
					baikalmfront_write_normal_video_setting(ctx);
					break;
				case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
				case SENSOR_SCENARIO_ID_SLIM_VIDEO:
					baikalmfront_write_hs_video_setting(ctx);
					break;
				default:
					break;
				}
		} else {
			switch(scenario_id) {
				case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
				case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
					baikalmfront_write_preview_setting(ctx);
					break;
				case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
					baikalmfront_write_normal_video_setting(ctx);
					break;
				case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
				case SENSOR_SCENARIO_ID_SLIM_VIDEO:
					baikalmfront_write_hs_video_setting(ctx);
					break;
				default:
					break;
			}
		}

		if (ctx->power_on_profile_en) {
			ctx->sensor_pw_on_profile.i2c_cfg_period =
					ktime_get_boottime_ns() - time_boot_begin;

			ctx->sensor_pw_on_profile.i2c_cfg_table_len =
					ctx->s_ctx.mode[scenario_id].mode_setting_len;
		}
		DRV_LOG(ctx, "X: sid:%u size:%u\n", scenario_id,
			ctx->s_ctx.mode[scenario_id].mode_setting_len);
	} else {
		DRV_LOGE(ctx, "please implement mode setting(sid:%u)!\n", scenario_id);
	}

	if (check_is_no_crop(ctx, scenario_id) && probe_eeprom(ctx)) {
		idx = ctx->eeprom_index;
		support = info[idx].xtalk_support;
		pbuf = info[idx].preload_xtalk_table;
		size = info[idx].xtalk_size;
		addr = info[idx].sensor_reg_addr_xtalk;
		if (support) {
			if (pbuf != NULL && addr > 0 && size > 0) {
				subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
				DRV_LOG(ctx, "set XTALK calibration data done.");
			}
		}
	}

	if (ctx->s_ctx.aov_sensor_support &&
		ctx->s_ctx.s_data_rate_global_timing_phy_ctrl != NULL)
		ctx->s_ctx.s_data_rate_global_timing_phy_ctrl((void *) ctx);

	set_mirror_flip(ctx, ctx->s_ctx.mirror);

	return ret;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	LOG_INF("open\n");
	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
//	i2c_table_write(ctx, baikalmfront_sensor_preinit_setting, ARRAY_SIZE(baikalmfront_sensor_preinit_setting));
//	mdelay(5);
	baikalmfront_sensor_init(ctx);

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

static int baikalmfront_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool enable = mode;
	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "gc08a8 test_pattern mode(%u->%u)\n", ctx->test_pattern, mode);
	if (enable) {
		subdrv_i2c_wr_u8(ctx, 0x008c, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x008d, 0x00);
	} else {
		subdrv_i2c_wr_u8(ctx, 0x008c, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x008d, 0x10);
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

static int baikalmfront_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *)para);
	u32 rg_gain;
	DRV_LOG(ctx, "gc08a8 platform_gain = 0x%x basegain = %d\n", gain, BASEGAIN);
	/* check boundary of gain */
	gain = max(gain, ctx->s_ctx.ana_gain_min);
	gain = min(gain, ctx->s_ctx.ana_gain_max);
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	/* write gain */
	// rg_gain = gain * (BASEGAIN / 16);
	rg_gain = gain;
	subdrv_i2c_wr_u16(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[0],
		rg_gain & 0xFFFF);
	DRV_LOG(ctx, "gc08a8 rg_gain = 0x%x \n", rg_gain);
	return 0;
}

static void baikalmfront_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
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
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		baikalmfront_write_frame_length(ctx, ctx->frame_length);
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
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure_lshift, 0);
	for (i = 0; i < 3; i++) {
		if (rg_shutters[i]) {
			if (ctx->s_ctx.reg_addr_exposure[i].addr[2]) {
				subdrv_i2c_wr_u8(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 16) & 0xFF);
				subdrv_i2c_wr_u8(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					(rg_shutters[i] >> 8) & 0xFF);
				subdrv_i2c_wr_u8(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[2],
					rg_shutters[i] & 0xFF);
			} else {
				subdrv_i2c_wr_u8(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 8) & 0xFF);
				subdrv_i2c_wr_u8(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					rg_shutters[i] & 0xFF);
			}
		}
	}
	DRV_LOG(ctx, "exp[0x%x/0x%x/0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		rg_shutters[0], rg_shutters[1], rg_shutters[2],
		frame_length, ctx->frame_length, ctx->autoflicker_en);

}

static int baikalmfront_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	baikalmfront_set_multi_shutter_frame_length(ctx, (u32 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

static void baikalmfront_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
	int i = 0;
	u32 values[3] = {0};

	if (shutters != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u32) *(shutters + i);
	}
	baikalmfront_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

static int baikalmfront_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	baikalmfront_set_hdr_tri_shutter(ctx, feature_data, 2);
	return 0;
}

static int baikalmfront_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	baikalmfront_set_hdr_tri_shutter(ctx, feature_data, 3);
	return 0;
}
static void baikalmfront_write_frame_length(struct subdrv_ctx *ctx, u32 fll)
{
	u32 addr_h = ctx->s_ctx.reg_addr_frame_length.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_frame_length.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_frame_length.addr[2];
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
		if (addr_ll) {
			subdrv_i2c_wr_u8(ctx,	addr_h,	(fll >> 16) & 0xFF);
			subdrv_i2c_wr_u8(ctx,	addr_l, (fll >> 8) & 0xFF);
			subdrv_i2c_wr_u8(ctx,	addr_ll, fll & 0xFF);
		} else {
			subdrv_i2c_wr_u8(ctx,	addr_h, (fll >> 8) & 0xFF);
			subdrv_i2c_wr_u8(ctx,	addr_l, fll & 0xFF);
		}
		/* update FL RG value after setting buffer for writting RG */
		ctx->frame_length_rg = ctx->frame_length;

		DRV_LOG(ctx,
			"ctx:(fl(RG):%u), fll[0x%x] multiply %u, fll_step:%u\n",
			ctx->frame_length_rg, fll, dol_cnt, fll_step);
	}
}
static void gc08a8_set_dummy(struct subdrv_ctx *ctx)
{
	baikalmfront_write_frame_length(ctx, ctx->frame_length);
}

static int gc08a8_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	DRV_LOG(ctx, "gc08a8 max_fps(input/output):%u/%u(sid:%u), min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id);
	if (ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin))
		gc08a8_set_dummy(ctx);

	return 0;
}
