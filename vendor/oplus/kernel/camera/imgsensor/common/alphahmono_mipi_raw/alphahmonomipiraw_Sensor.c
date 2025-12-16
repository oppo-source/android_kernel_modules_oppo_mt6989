// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 OPLUS. All rights reserved.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 alphahmonomipiraw_Sensor.c
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
#include "alphahmonomipiraw_Sensor.h"

#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "alphahmono_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

static int alphahmono_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmono_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmono_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static void alphahmono_sensor_init(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int alphahmono_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmono_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmono_set_frame_length(struct subdrv_ctx *ctx, u8 *frame_length, u32 *len);
static int alphahmono_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmono_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmono_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int alphahmono_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void alphahmono_write_frame_length(struct subdrv_ctx *ctx, u32 fll);
static bool alphahmono_set_auto_flicker(struct subdrv_ctx *ctx, bool min_framelength_en);
static int alphahmono_set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
		u8* feature_data, u32* len);
static int alphahmono_control(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, alphahmono_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, alphahmono_check_sensor_id},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, alphahmono_get_min_shutter_by_scenario_adapter},
	{SENSOR_FEATURE_SET_GAIN, alphahmono_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, alphahmono_set_shutter},
	{SENSOR_FEATURE_SET_FRAMELENGTH, alphahmono_set_frame_length},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, alphahmono_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, alphahmono_streaming_resume},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, alphahmono_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, alphahmono_set_multi_shutter_frame_length_ctrl},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, alphahmono_set_max_framerate_by_scenario},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_RAW_DATA,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_RAW_DATA,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_RAW_DATA,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_RAW_DATA,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_RAW_DATA,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_RAW_DATA,
		},
	}
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = alphahmono_preview_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmono_preview_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.max_framerate = 300,
		.mipi_pixel_rate = 67200000,
		.readout_length = 0,
		.read_margin = 16,
		.framelength_step = 1,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 12,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},

	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = alphahmono_capture_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmono_capture_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.max_framerate = 300,
		.mipi_pixel_rate = 67200000,
		.readout_length = 0,
		.read_margin = 16,
		.framelength_step = 1,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 12,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = alphahmono_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmono_normal_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.max_framerate = 300,
		.mipi_pixel_rate = 67200000,
		.readout_length = 0,
		.read_margin = 16,
		.framelength_step = 1,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 12,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},

	{
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = alphahmono_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmono_hs_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.max_framerate = 300,
		.mipi_pixel_rate = 67200000,
		.readout_length = 0,
		.read_margin = 16,
		.framelength_step = 1,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 12,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 180,
		},
	},

	{
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = alphahmono_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmono_slim_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.max_framerate = 300,
		.mipi_pixel_rate = 67200000,
		.readout_length = 0,
		.read_margin = 16,
		.framelength_step = 1,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 12,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = alphahmono_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(alphahmono_custom1_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1596,
		.max_framerate = 240,
		.mipi_pixel_rate = 67200000,
		.readout_length = 0,
		.read_margin = 16,
		.framelength_step = 1,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 12,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = ALPHAHMONO_SENSOR_ID,
	.reg_addr_sensor_id = {0xf0, 0xf1},
	.i2c_addr_table = {0x6e, 0x20, 0xFF},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_8_DATA_8,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {1600, 1200},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_1_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 12,
	.ana_gain_type = 4,
	.ana_gain_step = 1,
	.ana_gain_table = alphahmono_ana_gain_table,
	.ana_gain_table_size = sizeof(alphahmono_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3ED,
	.exposure_min = 4,
	.exposure_max = (0xffff * 128) - 16,
	.exposure_step = 1,
	.exposure_margin = 16,

	.frame_length_max = 0x3fff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.pdaf_type = PDAF_SUPPORT_NA,
	.g_gain2reg = PARAM_UNDEFINED,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x17,
	.reg_addr_exposure = {
			{0x03, 0x04},
	},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {
			{0xb1, 0xb2},
	},
	.reg_addr_frame_length = {0x41, 0x42},
	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0xf7375923,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = alphahmono_control,
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
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD, 1800000, 1},
	{HW_ID_AVDD, 2800000, 5},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_RST, 1, 2},
};

static struct subdrv_pw_seq_entry oplus_pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD, 1800000, 1},
	{HW_ID_AVDD, 2800000, 5},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry alphahmono_mipi_raw_entry = {
	.name = "alphahmono_mipi_raw",
	.id = ALPHAHMONO_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};
static void alphahmono_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		              ctx->dummy_line, ctx->dummy_pixel);

	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x41, (ctx->frame_length >> 8) & 0x3f);
	subdrv_i2c_wr_u8_u8(ctx, 0x42, ctx->frame_length & 0xff);
}				/*      set_dummy  */

static void alphahmono_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	alphahmono_set_dummy(ctx);
}
static void alphahmono_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 *shutter, u32 frame_length, bool auto_extend_en)
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

	ctx->exposure[0] = (ctx->exposure[0] > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
			? (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin) : ctx->exposure[0];

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			alphahmono_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			alphahmono_set_max_framerate(ctx, 146, 0);
		} else {
			/* Extend frame length */
			subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
			subdrv_i2c_wr_u8_u8(ctx, 0x41, (ctx->frame_length >> 8) & 0x3f);
			subdrv_i2c_wr_u8_u8(ctx, 0x42, ctx->frame_length & 0xff);
		}
	} else {
		/* Extend frame length */
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
		subdrv_i2c_wr_u8_u8(ctx, 0x41, (ctx->frame_length >> 8) & 0x3f);
		subdrv_i2c_wr_u8_u8(ctx, 0x42, ctx->frame_length & 0xff);
	}

	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x03, (ctx->exposure[0] >> 8) & 0x3f);
	subdrv_i2c_wr_u8_u8(ctx, 0x04, ctx->exposure[0] & 0xff);

	DRV_LOG(ctx, "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, \n",
		ctx->exposure[0], ctx->frame_length, frame_length, dummy_line);
}	/* set_shutter_frame_length */

static int alphahmono_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "shutter:%u, frame_length:%u\n", (u32)(*para), (u32) (*(para + 1)));
	alphahmono_set_shutter_frame_length_convert(ctx, (u32 *)para, (u32) (*(para + 1)), (u16) (*(para + 2)));
	return 0;
}
/* FUNCTION */
static void alphahmono_write_shutter(struct subdrv_ctx *ctx)
{
	uint16_t realtime_fps = 0;
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
	if (ctx->exposure[0] > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) {
		ctx->exposure[0] = ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin;
	}

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			alphahmono_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			alphahmono_set_max_framerate(ctx, 146, 0);
		} else {
			/* Extend frame length */
			subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
			subdrv_i2c_wr_u8_u8(ctx, 0x41, (ctx->frame_length >> 8) & 0x3f);
			subdrv_i2c_wr_u8_u8(ctx, 0x42, ctx->frame_length & 0xff);
		}
	} else {
		/* Extend frame length */
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
		subdrv_i2c_wr_u8_u8(ctx, 0x41, (ctx->frame_length >> 8) & 0x3f);
		subdrv_i2c_wr_u8_u8(ctx, 0x42, ctx->frame_length & 0xff);
	}

	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x03, (ctx->exposure[0] >> 8) & 0x3f);
	subdrv_i2c_wr_u8_u8(ctx, 0x04, ctx->exposure[0] & 0xff);

	DRV_LOG(ctx, "shutter =%d, framelength =%d\n", ctx->exposure[0], ctx->frame_length);
}	/*	write_shutter  */

static int alphahmono_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}
static void alphahmono_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	alphahmono_write_shutter(ctx);
}

static int alphahmono_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *para);
	alphahmono_set_shutter_convert(ctx, (u32 *)para);
	return 0;
}

static int alphahmono_set_frame_length(struct subdrv_ctx *ctx, u8 *feature_frame_length, u32 *len)
{
	u16 frame_length = *(u16 *)feature_frame_length;
	if (frame_length)
		ctx->frame_length = frame_length;
	ctx->frame_length = max(ctx->frame_length, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	alphahmono_write_frame_length(ctx, ctx->frame_length);
	DRV_LOG(ctx, "fll(input/output/min):%u/%u/%u\n", frame_length, ctx->frame_length, ctx->min_frame_length);
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
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
		subdrv_i2c_wr_u8_u8(ctx, 0x3e, 0x90);
	} else {;
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
		subdrv_i2c_wr_u8_u8(ctx, 0x3e, 0x00);
	}
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static int alphahmono_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			alphahmono_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int alphahmono_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
		return 0;
}

static void alphahmono_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
		u32 *shutters, u16 exp_cnt,	u16 frame_length)
{
	int i = 0;
	u32 fine_integ_line = 0;
	u16 last_exp_cnt = 1;
	u32 calc_fl[3] = {0};
	int readout_diff = 0;
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
	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		subdrv_i2c_wr_u8_u8(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (alphahmono_set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		alphahmono_write_frame_length(ctx, ctx->frame_length);
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
			subdrv_i2c_wr_u8_u8(ctx, ctx->s_ctx.reg_addr_exposure[i].addr[0],
				(rg_shutters[i] >> 8) & 0x3f);
			subdrv_i2c_wr_u8_u8(ctx, ctx->s_ctx.reg_addr_exposure[i].addr[1],
				rg_shutters[i] & 0xff);
		}
	}
	DRV_LOG(ctx, "exp[0x%x/0x%x/0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		rg_shutters[0], rg_shutters[1], rg_shutters[2],
		frame_length, ctx->frame_length, ctx->autoflicker_en);
}


static int alphahmono_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	alphahmono_set_multi_shutter_frame_length(ctx, (u32 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

static void alphahmono_write_frame_length(struct subdrv_ctx *ctx, u32 fll)
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
			subdrv_i2c_wr_u8_u8(ctx, addr_h,	(fll >> 16) & 0xFF);
			subdrv_i2c_wr_u8_u8(ctx, addr_l, (fll >> 8) & 0xFF);
			subdrv_i2c_wr_u8_u8(ctx, addr_ll, fll & 0xFF);
		} else {
			subdrv_i2c_wr_u8_u8(ctx, addr_h, (fll >> 8) & 0xFF);
			subdrv_i2c_wr_u8_u8(ctx, addr_l, fll & 0xFF);
		}
		/* update FL RG value after setting buffer for writting RG */
		ctx->frame_length_rg = ctx->frame_length;

		DRV_LOG(ctx,
			"ctx:(fl(RG):%u), fll[0x%x] multiply %u, fll_step:%u\n",
			ctx->frame_length_rg, fll, dol_cnt, fll_step);
	}
}

static bool alphahmono_set_auto_flicker(struct subdrv_ctx *ctx, bool min_framelength_en)
{
	u16 framerate = 0;
	bool ret = TRUE;

	if (!ctx->line_length) {
		DRV_LOGE(ctx, "line_length(%u) is invalid\n", ctx->line_length);
		return FALSE;
	}

	if (!ctx->frame_length) {
		DRV_LOGE(ctx, "frame_length(%u) is invalid\n", ctx->frame_length);
		return FALSE;
	}
	framerate = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;

	switch (ctx->autoflicker_en) {
	case 1:
		if (framerate > 592 && framerate <= 607)
			set_max_framerate(ctx, 592, min_framelength_en);
		else if (framerate > 296 && framerate <= 305)
			set_max_framerate(ctx, 296, min_framelength_en);
		else if (framerate > 246 && framerate <= 253)
			set_max_framerate(ctx, 246, min_framelength_en);
		else if (framerate > 236 && framerate <= 243)
			set_max_framerate(ctx, 236, min_framelength_en);
		else if (framerate > 146 && framerate <= 153)
			set_max_framerate(ctx, 146, min_framelength_en);
		else
			ret = FALSE;
		break;
	case 2:
		if (framerate > 592 && framerate <= 607)
			set_max_framerate(ctx, 592, min_framelength_en);
		else if (framerate > 299 && framerate <= 305)
			set_max_framerate(ctx, 299, min_framelength_en);
		else if (framerate > 246 && framerate <= 253)
			set_max_framerate(ctx, 246, min_framelength_en);
		else if (framerate > 236 && framerate <= 243)
			set_max_framerate(ctx, 236, min_framelength_en);
		else if (framerate > 146 && framerate <= 153)
			set_max_framerate(ctx, 146, min_framelength_en);
		else
			ret = FALSE;
		break;
	default:
		ret = FALSE;
	}

	DRV_LOG(ctx, "cur_fps:%u, flick_en:%d, min_fl_en:%u, new_fps:%llu\n",
				framerate, ctx->autoflicker_en, min_framelength_en,
				ctx->pclk / ctx->line_length * 10 / ctx->frame_length);

	return ret;
}

#define GC02M1B_SENSOR_GAIN_BASE             0x400
#define GC02M1B_SENSOR_GAIN_MAX              (12 * GC02M1B_SENSOR_GAIN_BASE)
#define GC02M1B_SENSOR_GAIN_MAX_VALID_INDEX  16
#define GC02M1B_SENSOR_DGAIN_BASE            0x400
static int alphahmono_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;
	u32 gain = *feature_data;
	uint32_t rg_gain;
	int16_t gain_index;
	uint32_t temp_gain;
	kal_uint16 GC02M1B_AGC_Param[GC02M1B_SENSOR_GAIN_MAX_VALID_INDEX][2] = {
		{  1024,  0 },
		{  1536,  1 },
		{  2035,  2 },
		{  2519,  3 },
		{  3165,  4 },
		{  3626,  5 },
		{  4147,  6 },
		{  4593,  7 },
		{  5095,  8 },
		{  5697,  9 },
		{  6270, 10 },
		{  6714, 11 },
		{  7210, 12 },
		{  7686, 13 },
		{  8214, 14 },
		{ 10337, 15 },
	};

	rg_gain = gain;
	if (rg_gain < GC02M1B_SENSOR_GAIN_BASE) {
        rg_gain = GC02M1B_SENSOR_GAIN_BASE;
	} else if (rg_gain > GC02M1B_SENSOR_GAIN_MAX) {
        rg_gain = GC02M1B_SENSOR_GAIN_MAX;
	}
	DRV_LOG(ctx, "Gain_Debug pass_gain= 0x%x\n", gain);
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = rg_gain;
	for (gain_index = GC02M1B_SENSOR_GAIN_MAX_VALID_INDEX - 1; gain_index >= 0; gain_index--)
		if (rg_gain >= GC02M1B_AGC_Param[gain_index][0])
			break;
	/* write gain */
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xb6, GC02M1B_AGC_Param[gain_index][1]);
	temp_gain = rg_gain * GC02M1B_SENSOR_DGAIN_BASE / GC02M1B_AGC_Param[gain_index][0];
	subdrv_i2c_wr_u8_u8(ctx, 0xb1, (temp_gain >> 8) & 0x1f);
	subdrv_i2c_wr_u8_u8(ctx, 0xb2, temp_gain & 0xff);

	DRV_LOG(ctx, "GC02M1B_AGC_Param[gain_index][1] = 0x%x, rg_gain = 0x%x, temp_gain = 0x%x\n", 	\
					GC02M1B_AGC_Param[gain_index][1], rg_gain, temp_gain);
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

	LOG_INF("rst delay = %d, func: %s, line: %d\n", pw_seq[1].delay, __FUNCTION__, __LINE__);
	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8_u8(ctx, addr_ll);
			LOG_INF("i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x02e0) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
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

	alphahmono_sensor_init(ctx);

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

void alphahmono_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E!\n");
	/*system*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xf4, 0x41);
	subdrv_i2c_wr_u8_u8(ctx, 0xf5, 0xe3);
	subdrv_i2c_wr_u8_u8(ctx, 0xf6, 0x44);
	subdrv_i2c_wr_u8_u8(ctx, 0xf8, 0x38);
	subdrv_i2c_wr_u8_u8(ctx, 0xf9, 0x82);
	subdrv_i2c_wr_u8_u8(ctx, 0xfa, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xfd, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x81);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x03);
	subdrv_i2c_wr_u8_u8(ctx, 0x01, 0x0b);
	subdrv_i2c_wr_u8_u8(ctx, 0xf7, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x8e);

	/*CISCTL*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x87, 0x09);
	subdrv_i2c_wr_u8_u8(ctx, 0xee, 0x72);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x8c, 0x90);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x90, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x03, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0x04, 0x7d);
	subdrv_i2c_wr_u8_u8(ctx, 0x41, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0x42, 0xf4);
	subdrv_i2c_wr_u8_u8(ctx, 0x05, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0x06, 0x48);
	subdrv_i2c_wr_u8_u8(ctx, 0x07, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x08, 0x18);
	subdrv_i2c_wr_u8_u8(ctx, 0x9d, 0x18);
	subdrv_i2c_wr_u8_u8(ctx, 0x09, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x0a, 0x02);
	subdrv_i2c_wr_u8_u8(ctx, 0x0d, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0x0e, 0xbc);
	subdrv_i2c_wr_u8_u8(ctx, 0x17, 0x83);
	subdrv_i2c_wr_u8_u8(ctx, 0x19, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0x24, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x56, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0x5b, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x5e, 0x01);

	/*analog Register width*/
	subdrv_i2c_wr_u8_u8(ctx, 0x21, 0x3c);
	subdrv_i2c_wr_u8_u8(ctx, 0x44, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0xcc, 0x01);

	/*analog mode*/
	subdrv_i2c_wr_u8_u8(ctx, 0x1a, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0x1f, 0x11);
	subdrv_i2c_wr_u8_u8(ctx, 0x27, 0x30);
	subdrv_i2c_wr_u8_u8(ctx, 0x2b, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x33, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x53, 0x90);
	subdrv_i2c_wr_u8_u8(ctx, 0xe6, 0x50);

	/*analog voltage*/
	subdrv_i2c_wr_u8_u8(ctx, 0x39, 0x07);
	subdrv_i2c_wr_u8_u8(ctx, 0x43, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0x46, 0x4a);
	subdrv_i2c_wr_u8_u8(ctx, 0x7c, 0xa0);
	subdrv_i2c_wr_u8_u8(ctx, 0xd0, 0xbe);
	subdrv_i2c_wr_u8_u8(ctx, 0xd1, 0x60);
	subdrv_i2c_wr_u8_u8(ctx, 0xd2, 0x40);
	subdrv_i2c_wr_u8_u8(ctx, 0xd3, 0xf3);
	subdrv_i2c_wr_u8_u8(ctx, 0xde, 0x1d);

	/*analog current*/
	subdrv_i2c_wr_u8_u8(ctx, 0xcd, 0x05);
	subdrv_i2c_wr_u8_u8(ctx, 0xce, 0x6f);

	/*CISCTL RESET*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x88);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x10);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x8e);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x88);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x10);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xfc, 0x8e);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0xe0, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);

	/*ISP*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x53, 0x54);
	subdrv_i2c_wr_u8_u8(ctx, 0x87, 0x53);
	subdrv_i2c_wr_u8_u8(ctx, 0x89, 0x03);

	/*Gain*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xb0, 0x74);
	subdrv_i2c_wr_u8_u8(ctx, 0xb1, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0xb2, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xb6, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0xd8, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x40);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x60);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xc0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x2a);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x40);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xa0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x90);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x19);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xc0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xD0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x2F);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x90);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x39);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x0f);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x40);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x1a);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x60);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x25);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xa0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x2c);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xa0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x32);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xc0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x38);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x60);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x3c);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x02);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0xa0);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x40);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x02);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x18);
	subdrv_i2c_wr_u8_u8(ctx, 0xc0, 0x5c);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x9f, 0x10);

	/*BLK*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x26, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x40, 0x22);
	subdrv_i2c_wr_u8_u8(ctx, 0x46, 0x7f);
	subdrv_i2c_wr_u8_u8(ctx, 0x49, 0x0f);
	subdrv_i2c_wr_u8_u8(ctx, 0x4a, 0xf0);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0x14, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0x15, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0x16, 0x80);
	subdrv_i2c_wr_u8_u8(ctx, 0x17, 0x80);

	/*ant _blooming*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x41, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0x4c, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x4d, 0x0c);
	subdrv_i2c_wr_u8_u8(ctx, 0x44, 0x08);
	subdrv_i2c_wr_u8_u8(ctx, 0x48, 0x03);

	/*Window 1600X1200*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x90, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x91, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x92, 0x06);
	subdrv_i2c_wr_u8_u8(ctx, 0x93, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x94, 0x06);
	subdrv_i2c_wr_u8_u8(ctx, 0x95, 0x04);
	subdrv_i2c_wr_u8_u8(ctx, 0x96, 0xb0);
	subdrv_i2c_wr_u8_u8(ctx, 0x97, 0x06);
	subdrv_i2c_wr_u8_u8(ctx, 0x98, 0x40);

	/*mipi*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x03);
	subdrv_i2c_wr_u8_u8(ctx, 0x01, 0x23);
	subdrv_i2c_wr_u8_u8(ctx, 0x03, 0xce);
	subdrv_i2c_wr_u8_u8(ctx, 0x04, 0x48);
	subdrv_i2c_wr_u8_u8(ctx, 0x15, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x21, 0x10);
	subdrv_i2c_wr_u8_u8(ctx, 0x22, 0x05);
	subdrv_i2c_wr_u8_u8(ctx, 0x23, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0x25, 0x20);
	subdrv_i2c_wr_u8_u8(ctx, 0x26, 0x08);
	subdrv_i2c_wr_u8_u8(ctx, 0x29, 0x06);
	subdrv_i2c_wr_u8_u8(ctx, 0x2a, 0x0a);
	subdrv_i2c_wr_u8_u8(ctx, 0x2b, 0x08);

	/*out*/
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_u8_u8(ctx, 0x8c, 0x10);
	subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_u8_u8(ctx, 0x3e, 0x00);
	DRV_LOG(ctx, "X!\n");
}

void alphahmono_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	u32 exp_cnt = 0;
	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
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

int alphahmono_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	alphahmono_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return 0;
}

static int alphahmono_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	if (mode) {
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x01);
		subdrv_i2c_wr_u8_u8(ctx, 0x8c, 0x11);
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
	} else if (ctx->test_pattern) {
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x01);
		subdrv_i2c_wr_u8_u8(ctx, 0x8c, 0x10);
		subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
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

static int alphahmono_set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
		u8* feature_data, u32* len)
{
	u32 frame_length;
	u32 frame_length_step;
	u32 frame_length_min;
	u32 frame_length_max;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)(*feature_data);
	u64 *feature_framerate = (u64 *)feature_data;
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
	if (ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_LBMF) {
		set_max_framerate_in_lut_by_scenario(ctx, scenario_id, framerate);
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
	ctx->frame_length = frame_length_step ? roundup(ctx->frame_length, frame_length_step) : ctx->frame_length;

	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	DRV_LOG(ctx, "max_fps(input/output):%u/%u(sid:%u), min_fl_en:1, ctx->frame_length:%u\n",
		framerate, ctx->current_fps, scenario_id, ctx->frame_length);
	if (ctx->s_ctx.reg_addr_auto_extend ||
			(ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin))) {
		if (ctx->s_ctx.aov_sensor_support &&
			ctx->s_ctx.mode[scenario_id].aov_mode &&
			!ctx->s_ctx.mode[scenario_id].s_dummy_support)
			DRV_LOG_MUST(ctx, "AOV mode not support set_dummy!\n");
		else
			alphahmono_set_dummy(ctx);
	}
	return 0;
}

static int alphahmono_control(struct subdrv_ctx *ctx,
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

		/* mode setting */
		if (scenario_id == SENSOR_SCENARIO_ID_CUSTOM1) {
			subdrv_i2c_wr_u8_u8(ctx, 0x41, 0x06);
			subdrv_i2c_wr_u8_u8(ctx, 0x42, 0x3c);
		} else {
			subdrv_i2c_wr_u8_u8(ctx, 0xfe, 0x00);
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
