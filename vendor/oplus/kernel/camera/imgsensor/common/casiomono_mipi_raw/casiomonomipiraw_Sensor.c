// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 OPLUS. All rights reserved.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 casiomonomipiraw_Sensor.c
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
#include "casiomonomipiraw_Sensor.h"

#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "casiomono_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

static void casiomono_sensor_init(struct subdrv_ctx *ctx);
static u16 get_gain2reg(u32 gain);
static int casiomono_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomono_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomono_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int casiomono_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomono_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomono_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomono_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomono_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int casiomono_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void casiomono_write_frame_length(struct subdrv_ctx *ctx, u32 fll);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, casiomono_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, casiomono_check_sensor_id},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, casiomono_get_min_shutter_by_scenario_adapter},
	{SENSOR_FEATURE_SET_GAIN, casiomono_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, casiomono_set_shutter},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, casiomono_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, casiomono_streaming_resume},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, casiomono_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, casiomono_set_multi_shutter_frame_length_ctrl},
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
		.mode_setting_table = casiomono_preview_setting,
		.mode_setting_len = ARRAY_SIZE(casiomono_preview_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 72000000,
		.linelength = 1920,
		.framelength = 1250,
		.max_framerate = 300,
		.mipi_pixel_rate = 72000000,
		.readout_length = 0,
		.read_margin = 6,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
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
		.csi_param = {
			.dphy_trail = 90,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},

	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = casiomono_capture_setting,
		.mode_setting_len = ARRAY_SIZE(casiomono_capture_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 72000000,
		.linelength = 1920,
		.framelength = 1250,
		.max_framerate = 300,
		.mipi_pixel_rate = 72000000,
		.readout_length = 0,
		.read_margin = 6,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
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
		.csi_param = {
			.dphy_trail = 90,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = casiomono_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(casiomono_normal_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 72000000,
		.linelength = 1920,
		.framelength = 1250,
		.max_framerate = 300,
		.mipi_pixel_rate = 72000000,
		.readout_length = 0,
		.read_margin = 6,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
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
		.delay_frame = 5,
		.csi_param = {
			.dphy_trail = 90,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},

	{
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = casiomono_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(casiomono_hs_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 72000000,
		.linelength = 1920,
		.framelength = 1250,
		.max_framerate = 300,
		.mipi_pixel_rate = 72000000,
		.readout_length = 0,
		.read_margin = 6,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
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
		.delay_frame = 5,
		.csi_param = {
			.dphy_trail = 90,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 180,
		},
	},

	{
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = casiomono_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(casiomono_slim_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 72000000,
		.linelength = 1920,
		.framelength = 1250,
		.max_framerate = 300,
		.mipi_pixel_rate = 72000000,
		.readout_length = 0,
		.read_margin = 6,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
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
		.delay_frame = 5,
		.csi_param = {
			.dphy_trail = 90,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = casiomono_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(casiomono_custom1_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 72000000,
		.linelength = 1920,
		.framelength = 1562,
		.max_framerate = 240,
		.mipi_pixel_rate = 72000000,
		.readout_length = 0,
		.read_margin = 6,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
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
		.csi_param = {
			.dphy_trail = 90,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = CASIOMONO_SENSOR_ID,
	.reg_addr_sensor_id = {0x3107, 0x3108},
	.i2c_addr_table = {0x6C, 0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
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
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 1,
	.ana_gain_step = 1,
	.ana_gain_table = casiomono_ana_gain_table,
	.ana_gain_table_size = sizeof(casiomono_ana_gain_table),
	.min_gain_iso = 50,
	.exposure_def = 0x3D0,
	.exposure_min = 2,
	.exposure_max = (0xffff * 128) - 6,
	.exposure_step = 1,
	.exposure_margin = 6,

	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 934000,
	.pdaf_type = PDAF_SUPPORT_NA,
	.g_gain2reg = get_gain2reg,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x3221,
	.reg_addr_exposure = {
			{0x3e00, 0x3e01, 0x3e02},
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3060,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {
			{0x3e07, 0x3e09},
	},
	.reg_addr_frame_length = {0x320e, 0x320f},
	.init_setting_table = casiomono_init_setting,
	.init_setting_len = ARRAY_SIZE(casiomono_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0x820e0cf,
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
	{HW_ID_RST, 0, 0},
	{HW_ID_DOVDD, 1800000, 3},
	{HW_ID_AVDD, 2812500, 1},
	{HW_ID_RST, 1, 1},
	{HW_ID_RST, 0, 1},
	{HW_ID_RST, 1, 4},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 4},
};

const struct subdrv_entry casiomono_mipi_raw_entry = {
	.name = "casiomono_mipi_raw",
	.id = CASIOMONO_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};
static void casiomono_set_dummy(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		              ctx->dummy_line, ctx->dummy_pixel);

	subdrv_i2c_wr_u8(ctx, 0x320e, ctx->frame_length >> 8);
	subdrv_i2c_wr_u8(ctx, 0x320f, ctx->frame_length & 0xff);
	subdrv_i2c_wr_u8(ctx, 0x320c, ctx->line_length >> 8);
	subdrv_i2c_wr_u8(ctx, 0x320d, ctx->line_length & 0xff);
}				/*      set_dummy  */

static void casiomono_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
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

	casiomono_set_dummy(ctx);
}
static void casiomono_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 *shutter, u32 frame_length, bool auto_extend_en)
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
		if (realtime_fps > 297 && realtime_fps <= 305) {
			casiomono_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 147 && realtime_fps <= 150) {
			casiomono_set_max_framerate(ctx, 146, 0);
		} else {
			/* Extend frame length */
			subdrv_i2c_wr_u8(ctx, 0x320e, ctx->frame_length >> 8);
			subdrv_i2c_wr_u8(ctx, 0x320f, ctx->frame_length & 0xff);
		}
	} else {
		/* Extend frame length */
		subdrv_i2c_wr_u8(ctx, 0x320e, ctx->frame_length >> 8);
		subdrv_i2c_wr_u8(ctx, 0x320f, ctx->frame_length & 0xff);
	}

	subdrv_i2c_wr_u8(ctx, 0x3e00, (ctx->exposure[0] >> 12) & 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x3e01, (ctx->exposure[0] >> 4) & 0xff);
	subdrv_i2c_wr_u8(ctx, 0x3e02, (ctx->exposure[0] << 4) & 0xf0);

	DRV_LOG(ctx, "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, \n",
		ctx->exposure[0], ctx->frame_length, frame_length, dummy_line);
}	/* set_shutter_frame_length */

static int casiomono_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "shutter:%u, frame_length:%u\n", (u32)(*para), (u32) (*(para + 1)));
	casiomono_set_shutter_frame_length_convert(ctx, (u32 *)para, (u32) (*(para + 1)), (u16) (*(para + 2)));
	return 0;
}
/* FUNCTION */
static void casiomono_write_shutter(struct subdrv_ctx *ctx)
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
		if (realtime_fps > 297 && realtime_fps <= 305) {
			casiomono_set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps > 147 && realtime_fps <= 150) {
			casiomono_set_max_framerate(ctx, 146, 0);
		} else {
			/* Extend frame length */
			subdrv_i2c_wr_u8(ctx, 0x320e, ctx->frame_length >> 8);
			subdrv_i2c_wr_u8(ctx, 0x320f, ctx->frame_length & 0xff);
		}
	} else {
		/* Extend frame length */
		subdrv_i2c_wr_u8(ctx, 0x320e, ctx->frame_length >> 8);
		subdrv_i2c_wr_u8(ctx, 0x320f, ctx->frame_length & 0xff);
	}

	subdrv_i2c_wr_u8(ctx, 0x3e00, (ctx->exposure[0] >> 12) & 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x3e01, (ctx->exposure[0] >> 4) & 0xff);
	subdrv_i2c_wr_u8(ctx, 0x3e02, (ctx->exposure[0] << 4) & 0xf0);

	DRV_LOG(ctx, "shutter =%d, framelength =%d\n", ctx->exposure[0], ctx->frame_length);
}	/*	write_shutter  */

static int casiomono_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}
static void casiomono_set_shutter_convert(struct subdrv_ctx *ctx, u32 *shutter)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *shutter);
	ctx->exposure[0] = *shutter;

	casiomono_write_shutter(ctx);
}

static int casiomono_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx, "set_shutter shutter =%d\n", *para);
	casiomono_set_shutter_convert(ctx, (u32 *)para);
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
		check_stream_on(ctx);
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

static int casiomono_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			casiomono_set_shutter_convert(ctx, (u32 *)para);
		streaming_ctrl(ctx, true);
		return 0;
}

static int casiomono_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
		return 0;
}

static void casiomono_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
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
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		casiomono_write_frame_length(ctx, ctx->frame_length);
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
				subdrv_i2c_wr_u8(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 12) & 0x0F);
				subdrv_i2c_wr_u8(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					(rg_shutters[i] >> 4) & 0xFF);
				subdrv_i2c_wr_u8(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[2],
					(rg_shutters[i] << 4) & 0xF0);
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

static int casiomono_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	casiomono_set_multi_shutter_frame_length(ctx, (u32 *)(*feature_data),
		(u16) (*(feature_data + 1)), (u16) (*(feature_data + 2)));
	return 0;
}

void casiomono_write_frame_length(struct subdrv_ctx *ctx, u32 fll)
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

#define SC202CS_SENSOR_GAIN_BASE             1024
#define SC202CS_SENSOR_GAIN_MAX              65024 /* (63.5 * SC202CS_SENSOR_GAIN_BASE) */
#define SC202CS_SENSOR_GAIN_MAX_VALID_INDEX  6
static int casiomono_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;
	u32 gain = *feature_data;
	uint32_t rg_gain;
	int16_t gain_index;
	uint32_t temp_gain;
	uint32_t SC202CS_AGC_Param[SC202CS_SENSOR_GAIN_MAX_VALID_INDEX][2] = {
		{  1024,  0x00 },
		{  2048,  0x01 },
		{  4096,  0x03 },
		{  8192,  0x07 },
		{ 16384,  0x0f },
		{ 32768,  0x1f },
	};

	rg_gain = gain;
	if (rg_gain < SC202CS_SENSOR_GAIN_BASE) {
        rg_gain = SC202CS_SENSOR_GAIN_BASE;
	} else if (rg_gain > SC202CS_SENSOR_GAIN_MAX) {
        rg_gain = SC202CS_SENSOR_GAIN_MAX;
	}
	DRV_LOG(ctx, "Gain_Debug pass_gain= 0x%x\n", gain);
	/* mapping of gain to register value */
	if (ctx->s_ctx.g_gain2reg != NULL)
		rg_gain = ctx->s_ctx.g_gain2reg(rg_gain);
	else
		rg_gain = gain2reg(rg_gain);
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = rg_gain;
	for (gain_index = SC202CS_SENSOR_GAIN_MAX_VALID_INDEX - 1; gain_index >= 0; gain_index--)
		if (rg_gain >= SC202CS_AGC_Param[gain_index][0])
			break;
	/* write gain */
	subdrv_i2c_wr_u8(ctx, 0x3e09, SC202CS_AGC_Param[gain_index][1]);
	temp_gain = rg_gain * SC202CS_SENSOR_GAIN_BASE / SC202CS_AGC_Param[gain_index][0];
	subdrv_i2c_wr_u8(ctx, 0x3e07, (temp_gain >> 3) & 0xff);
	DRV_LOG(ctx, "gain[0x%x], %x, %x\n", rg_gain, SC202CS_AGC_Param[gain_index][1], (temp_gain >> 3) & 0xff);

	return 0;
}

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
			LOG_INF("i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0xeb52) {
				*sensor_id = ctx->s_ctx.sensor_id;
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

	casiomono_sensor_init(ctx);

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

static void casiomono_sensor_init(struct subdrv_ctx *ctx)
{
	subdrv_i2c_wr_u8(ctx, 0x0103, 0x01);
	mdelay(10);
	subdrv_i2c_wr_u8(ctx, 0x0100, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x36e9, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x36e9, 0x24);
	subdrv_i2c_wr_u8(ctx, 0x301f, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x3301, 0xff);
	subdrv_i2c_wr_u8(ctx, 0x3304, 0x68);
	subdrv_i2c_wr_u8(ctx, 0x3306, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x3308, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x3309, 0xa8);
	subdrv_i2c_wr_u8(ctx, 0x330b, 0xb0);
	subdrv_i2c_wr_u8(ctx, 0x330c, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x330d, 0xff);
	subdrv_i2c_wr_u8(ctx, 0x330e, 0x20);
	subdrv_i2c_wr_u8(ctx, 0x331e, 0x59);
	subdrv_i2c_wr_u8(ctx, 0x331f, 0x99);
	subdrv_i2c_wr_u8(ctx, 0x3333, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x335e, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x335f, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x3364, 0x1f);
	subdrv_i2c_wr_u8(ctx, 0x337c, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x337d, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x338f, 0xa0);
	subdrv_i2c_wr_u8(ctx, 0x3390, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x3391, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x3392, 0x1f);
	subdrv_i2c_wr_u8(ctx, 0x3393, 0xff);
	subdrv_i2c_wr_u8(ctx, 0x3394, 0xff);
	subdrv_i2c_wr_u8(ctx, 0x3395, 0xff);
	subdrv_i2c_wr_u8(ctx, 0x33a2, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x33ad, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x33b1, 0x20);
	subdrv_i2c_wr_u8(ctx, 0x33b3, 0x38);
	subdrv_i2c_wr_u8(ctx, 0x33f9, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x33fb, 0x48);
	subdrv_i2c_wr_u8(ctx, 0x33fc, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x33fd, 0x1f);
	subdrv_i2c_wr_u8(ctx, 0x349f, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x34a6, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x34a7, 0x1f);
	subdrv_i2c_wr_u8(ctx, 0x34a8, 0x38);
	subdrv_i2c_wr_u8(ctx, 0x34a9, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x34ab, 0xb0);
	subdrv_i2c_wr_u8(ctx, 0x34ad, 0xb0);
	subdrv_i2c_wr_u8(ctx, 0x34f8, 0x1f);
	subdrv_i2c_wr_u8(ctx, 0x34f9, 0x20);
	subdrv_i2c_wr_u8(ctx, 0x3630, 0xa0);
	subdrv_i2c_wr_u8(ctx, 0x3631, 0x92);
	subdrv_i2c_wr_u8(ctx, 0x3632, 0x64);
	subdrv_i2c_wr_u8(ctx, 0x3633, 0x43);
	subdrv_i2c_wr_u8(ctx, 0x3637, 0x49);
	subdrv_i2c_wr_u8(ctx, 0x363a, 0x85);
	subdrv_i2c_wr_u8(ctx, 0x363c, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x3650, 0x31);
	subdrv_i2c_wr_u8(ctx, 0x3670, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x3674, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x3675, 0xa0);
	subdrv_i2c_wr_u8(ctx, 0x3676, 0xa0);
	subdrv_i2c_wr_u8(ctx, 0x3677, 0x92);
	subdrv_i2c_wr_u8(ctx, 0x3678, 0x96);
	subdrv_i2c_wr_u8(ctx, 0x3679, 0x9a);
	subdrv_i2c_wr_u8(ctx, 0x367c, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x367d, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x367e, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x367f, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x3698, 0x83);
	subdrv_i2c_wr_u8(ctx, 0x3699, 0x86);
	subdrv_i2c_wr_u8(ctx, 0x369a, 0x8c);
	subdrv_i2c_wr_u8(ctx, 0x369b, 0x94);
	subdrv_i2c_wr_u8(ctx, 0x36a2, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x36a3, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x36a4, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x36ae, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x36af, 0x1f);
	subdrv_i2c_wr_u8(ctx, 0x36bd, 0x22);
	subdrv_i2c_wr_u8(ctx, 0x36be, 0x22);
	subdrv_i2c_wr_u8(ctx, 0x36bf, 0x22);
	subdrv_i2c_wr_u8(ctx, 0x36d0, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x370f, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x3721, 0x6c);
	subdrv_i2c_wr_u8(ctx, 0x3722, 0x8d);
	subdrv_i2c_wr_u8(ctx, 0x3725, 0xc5);
	subdrv_i2c_wr_u8(ctx, 0x3727, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x3728, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x37b7, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x37b8, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x37b9, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x37bd, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x37be, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x3901, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x3903, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x3905, 0x8d);
	subdrv_i2c_wr_u8(ctx, 0x3907, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x3908, 0x41);
	subdrv_i2c_wr_u8(ctx, 0x391f, 0x41);
	subdrv_i2c_wr_u8(ctx, 0x3933, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x3934, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x3937, 0x6f);
	subdrv_i2c_wr_u8(ctx, 0x393a, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x393d, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x393e, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x39dd, 0x41);
	subdrv_i2c_wr_u8(ctx, 0x3e00, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x3e01, 0x4d);
	subdrv_i2c_wr_u8(ctx, 0x3e02, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x3e09, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x4509, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x450d, 0x61);
}

static u16 get_gain2reg(u32 gain)
{
	return gain;
}

void casiomono_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
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

int casiomono_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	casiomono_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return 0;
}

static int casiomono_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	if (mode) {
		switch(mode) {
		case 5:
			subdrv_i2c_wr_u8(ctx, 0x020E, 0x00);
			subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
			break;
		default:
			subdrv_i2c_wr_u8(ctx, 0x0601, mode);
			break;
		}
	} else if (ctx->test_pattern) {
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/
		subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x0218, 0x01);
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
