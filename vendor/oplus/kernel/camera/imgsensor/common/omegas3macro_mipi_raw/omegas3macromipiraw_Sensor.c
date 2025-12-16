// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 omegas3macromipiraw_Sensor.c
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
#include "omegas3macromipiraw_Sensor.h"

#define OMEGAS3MACRO_EEPROM_READ_ID	0xA5
#define OMEGAS3MACRO_EEPROM_WRITE_ID   0xA4
#define OMEGAS3MACRO_MAX_OFFSET		0x1400
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "omegas3macro_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x1400

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static u16 get_gain2reg(u32 gain);
static int omegas3macro_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int close(struct subdrv_ctx *ctx);
static int omegas3macro_control(struct subdrv_ctx *ctx,
			enum SENSOR_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int get_sensor_temperature(void *arg);
static void get_sensor_cali(void* arg);
static void set_sensor_cali(void *arg);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int omegas3macro_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegas3macro_set_mirror_filp(struct subdrv_ctx *ctx, u8 image_mirror);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, omegas3macro_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, omegas3macro_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, omegas3macro_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_ESHUTTER, omegas3macro_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, omegas3macro_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, omegas3macro_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, omegas3macro_streaming_resume},
	{SENSOR_FEATURE_SET_GAIN, omegas3macro_set_gain},
	{SENSOR_FEATURE_SET_FRAMELENGTH, omegas3macro_set_frame_length},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, omegas3macro_set_hdr_tri_shutter2},
	{SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, omegas3macro_set_hdr_tri_shutter3},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, omegas3macro_set_multi_shutter_frame_length_ctrl},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x0000CA99,
		.addr_header_id = 0x00000005,
		.i2c_write_id = 0xA4,
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = omegas3macro_preview_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3macro_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1221,
		.max_framerate = 300,
		.mipi_pixel_rate = 66000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
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
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = omegas3macro_capture_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3macro_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1221,
		.max_framerate = 300,
		.mipi_pixel_rate = 66000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
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
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = omegas3macro_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3macro_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1221,
		.max_framerate = 300,
		.mipi_pixel_rate = 66000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
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
		.csi_param = {
			.dphy_trail = 0x98,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = omegas3macro_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3macro_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1221,
		.max_framerate = 300,
		.mipi_pixel_rate = 66000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
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
		.csi_param = {
			.dphy_trail = 0x54,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = omegas3macro_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3macro_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1221,
		.max_framerate = 300,
		.mipi_pixel_rate = 66000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
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
		.csi_param = {
			.dphy_trail = 0x54,
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = omegas3macro_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(omegas3macro_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1536,
		.max_framerate = 240,
		.mipi_pixel_rate = 66000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
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
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OMEGAS3MACRO_SENSOR_ID,
	.reg_addr_sensor_id = {0x0200, 0x0300},
	.i2c_addr_table = {0x7a, 0xff},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_8_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {1600, 1200},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_1_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 15,
	.ana_gain_type = 1, // 0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 2,
	.ana_gain_table = omegas3macro_ana_gain_table,
	.ana_gain_table_size = sizeof(omegas3macro_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xffff * 128) - 4,
	.exposure_step = 1,
	.exposure_margin = 10,

	.frame_length_max = 0x7fff,
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
	.s_gph = PARAM_UNDEFINED,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	//.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	//.reg_addr_ana_gain = {{0x0204, 0x0205},},
	//.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = 0x0020,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,

	.init_setting_table = omegas3macro_sensor_init_setting,
	.init_setting_len =  ARRAY_SIZE(omegas3macro_sensor_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 0,
	.chk_s_off_end = 0,
	.checksum_value = 0x350174bc,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = omegas3macro_control,
	.feature_control = common_feature_control,
	.close = close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
    {HW_ID_RST, 0, 1},
    {HW_ID_DOVDD, 1800000, 1},
    {HW_ID_AVDD, 2804000, 8},
    {HW_ID_MCLK, 24, 7},
    {HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
    {HW_ID_RST, 1, 10},
};

const struct subdrv_entry omegas3macro_mipi_raw_entry = {
	.name = "omegas3macro_mipi_raw",
	.id = OMEGAS3MACRO_SENSOR_ID,
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

static void omegas3macro_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	/* return; //for test */
	subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x14, (ctx->frame_length - 1220) >> 8);
	subdrv_i2c_wr_u8_u8(ctx, 0x15, (ctx->frame_length - 1220) & 0xFF);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);
}				/*      set_dummy  */

static void omegas3macro_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	omegas3macro_set_dummy(ctx);
}

static void omegas3macro_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 *shutter, u32 frame_length, bool auto_extend_en)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

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

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps > 592 && realtime_fps <= 607) {
			omegas3macro_set_max_framerate(ctx, 592, 0);
		} else if (realtime_fps > 296 && realtime_fps <= 305) {
			omegas3macro_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 246 && realtime_fps <= 253) {
			omegas3macro_set_max_framerate(ctx, 246, 0);
		} else if (realtime_fps > 236 && realtime_fps <= 243) {
			omegas3macro_set_max_framerate(ctx, 236, 0);
		} else if (realtime_fps > 146 && realtime_fps <= 153) {
			omegas3macro_set_max_framerate(ctx, 146, 0);
		} else {
			/* Extend frame length */
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
			subdrv_i2c_wr_u8_u8(ctx, 0x14, (ctx->frame_length - 1220) >> 8);
			subdrv_i2c_wr_u8_u8(ctx, 0x15, (ctx->frame_length - 1220) & 0xFF);
			subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);
		}
	} else {
		/* Extend frame length */
		subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
		subdrv_i2c_wr_u8_u8(ctx, 0x14, (ctx->frame_length - 1220) >> 8);
		subdrv_i2c_wr_u8_u8(ctx, 0x15, (ctx->frame_length - 1220) & 0xFF);
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);
	}
	subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x0f, (ctx->exposure[0]) & 0xFF);
	subdrv_i2c_wr_u8_u8(ctx, 0x0e, (ctx->exposure[0] >> 8) & 0xFF);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);

	DRV_LOG(ctx, "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, \n",
		ctx->exposure[0], ctx->frame_length, frame_length, dummy_line);
}	/* set_shutter_frame_length */

static int omegas3macro_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u16 fll = (u16) (*para);
	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "framelength =%d\n", fll);
	subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x14, (fll - 1220) >> 8);
	subdrv_i2c_wr_u8_u8(ctx, 0x15, (fll - 1220) & 0xFF);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);
	return 0;
}

static int omegas3macro_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "shutter:%u, frame_length:%u\n", (u32)(*para), (u32) (*(para + 1)));
	omegas3macro_set_shutter_frame_length_convert(ctx, (u32 *)para, (u32) (*(para + 1)), (u16) (*(para + 2)));
	return 0;
}

static void omegas3macro_write_shutter(struct subdrv_ctx *ctx)
{
	kal_uint16 realtime_fps = 0;
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

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps > 592 && realtime_fps <= 607) {
			omegas3macro_set_max_framerate(ctx, 592, 0);
		} else if (realtime_fps > 296 && realtime_fps <= 305) {
			omegas3macro_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 246 && realtime_fps <= 253) {
			omegas3macro_set_max_framerate(ctx, 246, 0);
		} else if (realtime_fps > 236 && realtime_fps <= 243) {
			omegas3macro_set_max_framerate(ctx, 236, 0);
		} else if (realtime_fps > 146 && realtime_fps <= 153) {
			omegas3macro_set_max_framerate(ctx, 146, 0);
		} else {
			// Extend frame length
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
			subdrv_i2c_wr_u8_u8(ctx, 0x14, (ctx->frame_length - 1220) >> 8);
			subdrv_i2c_wr_u8_u8(ctx, 0x15, (ctx->frame_length - 1220) & 0xFF);
			subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);
		}
	} else {
		// Extend frame length
		subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
		subdrv_i2c_wr_u8_u8(ctx, 0x14, (ctx->frame_length - 1220) >> 8);
		subdrv_i2c_wr_u8_u8(ctx, 0x15, (ctx->frame_length - 1220) & 0xFF);
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);
	}
	subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x0f, (ctx->exposure[0]) & 0xFF);
	subdrv_i2c_wr_u8_u8(ctx, 0x0e, (ctx->exposure[0] >> 8) & 0xFF);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);
	DRV_LOG(ctx, "shutter =%d, framelength =%d\n", ctx->exposure[0], ctx->frame_length);
}	/*	write_shutter  */

static void omegas3macro_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	omegas3macro_write_shutter(ctx);
}

static int omegas3macro_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *para);
	omegas3macro_set_shutter_convert(ctx, (u32 *)para);
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
		subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0X03);
		subdrv_i2c_wr_u8_u8(ctx, 0xc2, 0X01);
	} else {
		subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0X03);
		subdrv_i2c_wr_u8_u8(ctx, 0xc2, 0X00);
	}
	mdelay(70);
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static int omegas3macro_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			omegas3macro_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int omegas3macro_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
		return 0;
}

static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
	.i2c_read_id = 0xA5,
	.i2c_write_id = 0xA4,

	.addr_modinfo = 0x0000,
	.addr_sensorid = 0x0005,
	.addr_lens = 0x0006,
	.addr_modinfoflag = 0x000A,
	.addr_qrcode = 0x00E0,
	.addr_qrcodeflag = 0x00F1,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	*len = sizeof(oplus_eeprom_info);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, OMEGAS3MACRO_EEPROM_READ_ID >> 1,
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

static int omegas3macro_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int omegas3macro_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0X00);
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			DRV_LOG(ctx, "i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x002B) {
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

	/* get sensor id */
	mdelay(9);
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	sensor_init(ctx);
	omegas3macro_set_mirror_filp(ctx, ctx->s_ctx.mirror);

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

static int omegas3macro_control(struct subdrv_ctx *ctx,
			enum SENSOR_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	int ret = ERROR_NONE;
	u64 time_boot_begin = 0;
	struct adaptor_ctx *_adaptor_ctx = NULL;
	struct v4l2_subdev *sd = NULL;

	if (ctx->i2c_client)
		sd = i2c_get_clientdata(ctx->i2c_client);
	if (sd)
		_adaptor_ctx = to_ctx(sd);
	if (!_adaptor_ctx)
		return -ENODEV;

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
		/* mode setting */

		#ifndef OPLUS_FEATURE_CAMERA_COMMON
			i2c_table_write(ctx, ctx->s_ctx.mode[scenario_id].mode_setting_table,
				ctx->s_ctx.mode[scenario_id].mode_setting_len);
		#else /*OPLUS_FEATURE_CAMERA_COMMON*/
			i2c_table_rewrite(ctx, ctx->s_ctx.mode[scenario_id].mode_setting_table,
				ctx->s_ctx.mode[scenario_id].mode_setting_len);
		#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

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

	omegas3macro_set_mirror_filp(ctx, ctx->s_ctx.mirror);
	mdelay(10);
	return ret;
}

static u16 get_gain2reg(u32 gain)
{
	return gain * 16 / BASEGAIN;
}

static int omegas3macro_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode) {
		switch(mode) {
		case 5:
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x03);
			subdrv_i2c_wr_u8_u8(ctx, 0x8c, 0x00);
			subdrv_i2c_wr_u8_u8(ctx, 0x8e, 0x00);
			subdrv_i2c_wr_u8_u8(ctx, 0x90, 0x00);
			subdrv_i2c_wr_u8_u8(ctx, 0x92, 0x00);
			subdrv_i2c_wr_u8_u8(ctx, 0x9b, 0x00);
			subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);
			break;
		default:
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x03);
			subdrv_i2c_wr_u8_u8(ctx, 0x81, 0x01);
		}
	} else if (ctx->test_pattern) {
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x03);
			subdrv_i2c_wr_u8_u8(ctx, 0x81, 0x00);
			subdrv_i2c_wr_u8_u8(ctx, 0x8c, 0x40);
			subdrv_i2c_wr_u8_u8(ctx, 0x8e, 0x40);
			subdrv_i2c_wr_u8_u8(ctx, 0x90, 0x40);
			subdrv_i2c_wr_u8_u8(ctx, 0x92, 0x40);
			subdrv_i2c_wr_u8_u8(ctx, 0x9b, 0x46);
			subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x02);
	}
	ctx->test_pattern = mode;
	return 0;
}

static int omegas3macro_set_mirror_filp(struct subdrv_ctx *ctx, u8 image_mirror)
{
	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "set image_mirror %d\n", image_mirror);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
			subdrv_i2c_wr_u8_u8(ctx, 0x12, 0x00);
			subdrv_i2c_wr_u8_u8(ctx, 0x01, 0x01);
			break;
		case IMAGE_H_MIRROR:
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
			subdrv_i2c_wr_u8_u8(ctx, 0x12, 0x01);
			subdrv_i2c_wr_u8_u8(ctx, 0x01, 0x01);
			break;
		case IMAGE_V_MIRROR:
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
			subdrv_i2c_wr_u8_u8(ctx, 0x12, 0x02);
			subdrv_i2c_wr_u8_u8(ctx, 0x01, 0x01);
			break;
		case IMAGE_HV_MIRROR:
			subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x01);
			subdrv_i2c_wr_u8_u8(ctx, 0x12, 0x03);
			subdrv_i2c_wr_u8_u8(ctx, 0x01, 0x01);
			break;
		default:
			DRV_LOG(ctx, "Error image_mirror setting\n");
	}
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

int omegas3macro_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0X01);
	subdrv_i2c_wr_u8_u8(ctx, 0x22, (rg_gain & 0xFF));
	subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0X02);
	DRV_LOG(ctx, "gain[0x%x]\n", rg_gain);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	/* group hold end */
	return 0;
}

static void omegas3macro_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
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

static int omegas3macro_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	omegas3macro_set_multi_shutter_frame_length(ctx, (u32 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

static void omegas3macro_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
	int i = 0;
	u32 values[3] = {0};

	if (shutters != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u32) *(shutters + i);
	}
	omegas3macro_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

static int omegas3macro_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	omegas3macro_set_hdr_tri_shutter(ctx, feature_data, 2);
	return 0;
}

static int omegas3macro_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	omegas3macro_set_hdr_tri_shutter(ctx, feature_data, 3);
	return 0;
}
