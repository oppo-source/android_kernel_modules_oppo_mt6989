// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#include "../mediatek/mediatek_v2/mtk_dsi.h"
#endif

#include "../mediatek/mediatek_v2/mtk_corner_pattern/oplus24705_data_hw_roundedpattern.h"
#include "nt37703a_fhdp_dsi_vdo_120hz_dphy.h"
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#include "../oplus/oplus_display_onscreenfingerprint.h"

static unsigned int nt37703a_vdo_dphy_buf_thresh[14] ={896, 1792, 2688, 3584, 4480,
	5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
static unsigned int nt37703a_vdo_dphy_range_min_qp[15] ={0, 4, 5, 5, 7, 7, 7, 7, 7,
	7, 9, 9, 9, 11, 23};
static unsigned int nt37703a_vdo_dphy_range_max_qp[15] ={8, 8, 9, 10, 11, 11, 11,
	12, 13, 14, 15, 16, 17, 17, 19};
static int nt37703a_vdo_dphy_range_bpg_ofs[15] ={2, 0, 0, -2, -4, -6, -8, -8, -8,
	-10, -10, -12, -12, -12, -12};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr1p2_enable_gpio;
	struct gpio_desc *vci3p0_enable_gpio;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	bool hbm_en;
	bool hbm_wait;
	int error;
};

extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern unsigned long seed_mode;
static int current_fps = 60;
static bool aod_state = false;

#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2
#define FINGER_HBM_BRIGHTNESS 3730

extern void lcdinfo_notify(unsigned long val, void *v);

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("debug for %s+\n", __func__);
	//OD optimization
	lcm_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x08);
	lcm_dcs_write_seq_static(ctx, 0xE0,0x31,0x01,0x01,0x01);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x06);
	lcm_dcs_write_seq_static(ctx, 0xE0,0x80,0x00,0x21,0x32,0x43,0x54,0x65,0x76,0x87,0x98,0x99,0x99,0x99);
	lcm_dcs_write_seq_static(ctx, 0xED,0x10,0x10,0x00);
	lcm_dcs_write_seq_static(ctx, 0xE1,0x00,0x00,0x0D,0x13,0x0F,0x07,0x0C,0x07,0x01,0x01,0x12,0x0C,0x07,0x08,0x0B,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x32);
	lcm_dcs_write_seq_static(ctx, 0xE1,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xE2,0x00,0x00,0x12,0x0B,0x0F,0x07,0x06,0x0A,0x00,0x10,0x03,0x08,0x05,0x04,0x04,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x32);
	lcm_dcs_write_seq_static(ctx, 0xE2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xE3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x29,0x17,0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x32);
	lcm_dcs_write_seq_static(ctx, 0xE3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xE4,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xE5,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xE6,0x00,0x00,0x00,0x00,0x00,0x00);
	//-----------------------------------CMD3-----------------------------------
	lcm_dcs_write_seq_static(ctx, 0xFF,0xAA,0x55,0xA5,0x80);
	//SDOptimize
	lcm_dcs_write_seq_static(ctx, 0x6F,0x46);
	lcm_dcs_write_seq_static(ctx, 0xF4,0x01,0x03);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x4A);
	lcm_dcs_write_seq_static(ctx, 0xF4,0x02,0x04);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x56);
	lcm_dcs_write_seq_static(ctx, 0xF4,0x22,0x22);
	lcm_dcs_write_seq_static(ctx, 0xFF,0xAA,0x55,0xA5,0x80);
	//isop
	lcm_dcs_write_seq_static(ctx, 0x6F,0x18);
	lcm_dcs_write_seq_static(ctx, 0xF4,0x73);
	//sd_optimize
	lcm_dcs_write_seq_static(ctx, 0x6F,0x2A);
	lcm_dcs_write_seq_static(ctx, 0xF4,0x08);
	//VGH_CLAMP_ON=0
	lcm_dcs_write_seq_static(ctx, 0x6F,0x32);
	lcm_dcs_write_seq_static(ctx, 0xF2,0x00);
	lcm_dcs_write_seq_static(ctx, 0xFF,0xAA,0x55,0xA5,0x81);
	//hs_drop_detect_en=1,hs_drop_detect_period[2:0]=4
	lcm_dcs_write_seq_static(ctx, 0x6F,0x3C);
	lcm_dcs_write_seq_static(ctx, 0xF5,0x84);
	//PWR_CP_OPT[7:0]=02h
	lcm_dcs_write_seq_static(ctx, 0x6F,0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF5,0x02);
	//-----------------------------------CMD1-----------------------------------
	////VideomodeAODsetting
	lcm_dcs_write_seq_static(ctx, 0x17,0x03);
	lcm_dcs_write_seq_static(ctx, 0x71,0x00);
	lcm_dcs_write_seq_static(ctx, 0x8D,0x00,0x00,0x04,0x37,0x00,0x00,0x09,0x57);
	//CommandmodeAODsettingMax.1080x1720
	//lcm_dcs_write_seq_static(ctx, 0x17,0x21}}
	//lcm_dcs_write_seq_static(ctx, 0x71,0x10}}
	//lcm_dcs_write_seq_static(ctx, 0x8D,0x00,0x00,0x04,0x37,0x00,0x00,0x06,0xB7}}
	//OldVideoDrop
	lcm_dcs_write_seq_static(ctx, 0xFF,0xAA,0x55,0xA5,0x81);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x3C);
	lcm_dcs_write_seq_static(ctx, 0xF5,0x87);
	//DataScalingOn
	//lcm_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x08}}
	//lcm_dcs_write_seq_static(ctx, 0xC1,0xAE,0xFF}}
	lcm_dcs_write_seq_static(ctx, 0x2A,0x00,0x00,0x04,0x37);
	lcm_dcs_write_seq_static(ctx, 0x2B,0x00,0x00,0x09,0x57);
	//VESASetting(DSCv1.1,2DEC,sliceheight=12,HxV=1080x2392}}-TMCC2X6850
	lcm_dcs_write_seq_static(ctx, 0x90,0x03);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x01);
	lcm_dcs_write_seq_static(ctx, 0x90,0x43);
	lcm_dcs_write_seq_static(ctx, 0x91,0xAB,0x28,0x00,0x08,0xC2,0x00,0x02,0x0E,0x00,0xBB,0x00,0x07,0x0D,0xB7,0x0C,0xB7,0x10,0xF0);
	//BC_CTRLDBV[13:0]isactive
	lcm_dcs_write_seq_static(ctx, 0x53,0x20);
	//VideoModeExt_VFP,VBPFSET0~2,IDLE
	lcm_dcs_write_seq_static(ctx, 0x3B,0x00,0x14,0x00,0x54,0x00,0x14,0x00,0x54,0x00,0x14,0x03,0x84,0x00,0x14,0x0A,0x14);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x10);
	lcm_dcs_write_seq_static(ctx, 0x3B,0x00,0x14,0x00,0x54);
	//TEon
	lcm_dcs_write_seq_static(ctx, 0x35,0x00);
	lcm_dcs_write_seq_static(ctx, 0x2F,0x00);
	//DBV
	lcm_dcs_write_seq_static(ctx, 0x51,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x6F,0x04);
	lcm_dcs_write_seq_static(ctx, 0x51,0x00,0x00);
	//demura ON
	lcm_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x04);
	lcm_dcs_write_seq_static(ctx, 0xCA,0x01);
	//Local HBM
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02);
	lcm_dcs_write_seq_static(ctx, 0xD1, 0x2A, 0xC3, 0x27, 0xB8, 0x32, 0x2C);

	lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x6F, 0x32);
	lcm_dcs_write_seq_static(ctx, 0xDF, 0x06);
	lcm_dcs_write_seq_static(ctx, 0x8B, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x87, 0x20); //0x20 Local HBM OFF,0x25 Local HBM ON
	lcm_dcs_write_seq_static(ctx, 0x88, 0x01, 0x02, 0x1C, 0x08, 0x7F);


	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	usleep_range(120*1000, 121*1000);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);

	pr_info("debug for %s-\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s:prepared=%d\n", __func__, ctx->prepared);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(10000, 11000);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(150*1000, 151*1000);

	ctx->error = 0;
	ctx->prepared = false;
	//ctx->hbm_en = false;
	pr_info("%s:success\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s:prepared=%d\n", __func__, ctx->prepared);
	if (ctx->prepared)
		return 0;

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
	pr_info("%s:success\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define FRAME_WIDTH             (1080)
#define FRAME_HEIGHT            (2392)
#define HFP                     (140)
#define HBP                     (40)
#define HSA                     (8)
#define VFP_60HZ                (2580)
#define VFP_90HZ                (900)
#define VFP_120HZ               (84)
#define VFP_144HZ               (84)
#define VBP                     (18)
#define VSA                     (2)

static const struct drm_display_mode disp_mode_60Hz = {
	.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_60HZ + VBP + VSA) * 60) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_60HZ,
	.vsync_end = FRAME_HEIGHT + VFP_60HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_60HZ + VSA + VBP,
};

static const struct drm_display_mode disp_mode_90Hz = {
	.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_90HZ + VBP + VSA) * 90) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_90HZ,
	.vsync_end = FRAME_HEIGHT + VFP_90HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_90HZ + VSA + VBP,
};

static const struct drm_display_mode disp_mode_120Hz = {
	.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_120HZ + VBP + VSA) * 120) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_120HZ,
	.vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
};

static struct mtk_panel_params ext_params_60Hz = {
	.pll_clk = 565,
	.data_rate = 1130,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x03}},
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 572,
		.data_rate = 1144,
		.hfp = 148,
		.vfp = VFP_60HZ,
		.vsa = VSA,
		.vbp = VBP,
		.hsa = HSA,
		.hbp = HBP,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 0,
	.esd_check_enable = 1,
	//.lcm_esd_check_table[0] = {
	//	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	//},
	//.lcm_esd_check_table[1] = {
	//	.cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00,
	//},
	.vdo_mix_mode_en = false,
	// .lcm_esd_check_table[1] = {
	// 	.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	// },



	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,


	.oplus_display_global_dre = 1,
	.oplus_custom_hdr_color_tmp = true,
	.oplus_custom_hdr_red = 950,
	.oplus_custom_hdr_green = 1024,
	.oplus_custom_hdr_blue = 800,
	.oplus_panel_use_rgb_gain = true,

	.vendor = "TMA_NT37703A",
	.manufacture = "TM_CASIOX",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2392,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
	},
};

static struct mtk_panel_params ext_params_90Hz = {
	.pll_clk = 565,
	.data_rate = 1130,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x02}},
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 572,
		.data_rate = 1144,
		.hfp = 148,
		.vfp = VFP_90HZ,
		.vsa = VSA,
		.vbp = VBP,
		.hsa = HSA,
		.hbp = HBP,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 0,
	.esd_check_enable = 1,
	//.lcm_esd_check_table[0] = {
	//	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	//},
	//.lcm_esd_check_table[1] = {
	//	.cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00,
	//},
	.vdo_mix_mode_en = false,
	// .lcm_esd_check_table[1] = {
	// 	.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	// },



	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,


	.oplus_display_global_dre = 1,
	.oplus_custom_hdr_color_tmp = true,
	.oplus_custom_hdr_red = 950,
	.oplus_custom_hdr_green = 1024,
	.oplus_custom_hdr_blue = 800,
	.oplus_panel_use_rgb_gain = true,

	.vendor = "TMA_NT37703A",
	.manufacture = "TM_CASIOX",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2392,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
	},
};


static struct mtk_panel_params ext_params_120Hz = {
	.pll_clk = 565,
	.data_rate = 1130,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		// .send_mode = 1,
		// .send_cmd_need_delay = 1,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x00}},
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 572,
		.data_rate = 1144,
		.hfp = 148,
		.vfp = VFP_120HZ,
		.vsa = VSA,
		.vbp = VBP,
		.hsa = HSA,
		.hbp = HBP,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 0,
	.esd_check_enable = 1,
	//.lcm_esd_check_table[0] = {
	//	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	//},
	//.lcm_esd_check_table[1] = {
	//	.cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00,
	//},
	.vdo_mix_mode_en = false,
	// .lcm_esd_check_table[1] = {
	// 	.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	// },


	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

	.oplus_display_global_dre = 1,
	.oplus_custom_hdr_color_tmp = true,
	.oplus_custom_hdr_red = 950,
	.oplus_custom_hdr_green = 1024,
	.oplus_custom_hdr_blue = 800,
	.oplus_panel_use_rgb_gain = true,

	.vendor = "TMA_NT37703A",
	.manufacture = "TM_CASIOX",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2392,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
	},
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	static unsigned int demura_offset = 0;
	int i = 0;
	char bl_tb0[] = {0x51, 0x00, 0x00};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	mapped_level = level;
	if (mapped_level > 0 && mapped_level < 8) {
		pr_info("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, mapped_level);
		return -EINVAL;
	}

	if (mapped_level > 1 && mapped_level <= BRIGHTNESS_MAX) {
		oplus_display_brightness = mapped_level;
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	if ((get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) && (mapped_level > 1))
		mapped_level = 1023;

	if(demura_offset != 1 && mapped_level <= 0x481) {
		demura_offset = 1;
		for (i = 0; i < sizeof(lcm_set_demura_offset1)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_set_demura_offset1[i].para_list, lcm_set_demura_offset1[i].count);
		}
	} else if (demura_offset != 2 && mapped_level > 0x481 && mapped_level <= 0xFFE) {
		demura_offset = 2;
		for (i = 0; i < sizeof(lcm_set_demura_offset2)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_set_demura_offset2[i].para_list, lcm_set_demura_offset2[i].count);
		}
	} else if (demura_offset != 3 && mapped_level == 0xFFF) {
		demura_offset = 3;
		for (i = 0; i < sizeof(lcm_set_demura_offset3)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_set_demura_offset3[i].para_list, lcm_set_demura_offset3[i].count);
		}
	}

	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	pr_info("%s,level = %d,", __func__, level);

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char esd_bl_level[] = {0x51, 0x03, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	esd_bl_level[1] = level >> 8;
	esd_bl_level[2] = level & 0xFF;
	cb(dsi, handle, esd_bl_level, ARRAY_SIZE(esd_bl_level));
	pr_info("esd_bl_level[1]=%x, esd_bl_level[2]=%x\n", esd_bl_level[1], esd_bl_level[2]);

	return 0;
}

static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;
	unsigned int level = 0;
	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	pr_info("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, hbm_mode);
	if (hbm_mode == 1) {
		level = FINGER_HBM_BRIGHTNESS;
	} else if (hbm_mode == 0) {
		level = oplus_display_brightness;
	}
	lcm_setbrightness_normal[0].para_list[1] = level >> 8;
	lcm_setbrightness_normal[0].para_list[2] = level & 0xFF;
	for (i = 0; i < sizeof(lcm_setbrightness_normal)/sizeof(struct LCM_setting_table); i++){
		cb(dsi, handle, lcm_setbrightness_normal[i].para_list, lcm_setbrightness_normal[i].count);
	}
	return 0;
}

static unsigned int getBeta(unsigned int level)
{
	unsigned int Beta = 0;
	if (level <= 0x481) {
		Beta = 0x00;
	} else if (level >= 0x482 && level < 0x74E) {
		Beta = 2048 * (level - 1154) / (1870 - 1154);
	} else if (level >= 0x74E && level <= 0xDBB) {
		Beta = (2304 - 2048) * (level - 1870) / (3515 - 1870) + 2048;
	} else if (level <= 0xFFE) {
		Beta = (2304 - 2048) * (level - 3515) / (4094 - 3515) + 2304;
	}
	return Beta;
}

static unsigned int getAlpha(unsigned int level)
{
	unsigned int Alpha = 0;
	if (level <= 0x481) {
		Alpha = 0xEF6;
	} else if (level >= 0x482 && level < 0xDBB) {
		Alpha = 0xE72;
	} else if (level <= 0xFFE) {
		Alpha = (4095 - 3698) * (level - 3515) / (4095 - 3515) + 3698;
	}
	return Alpha;
}

static int oplus_ofp_set_lhbm_pressed_icon_single(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int i = 0;
	unsigned int Beta = 0;
	unsigned int Alpha = 0;

	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	pr_info("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, en);
	if (en == 1) {
		Beta = getBeta(oplus_display_brightness);
		Alpha = getAlpha(oplus_display_brightness);
		if (oplus_display_brightness > 0x481) {
			lcm_finger_lhbm_on_setting[7].para_list[1] = 0x06;
			lcm_finger_lhbm_on_setting[8].para_list[1] = 0x00;
		} else {
			lcm_finger_lhbm_on_setting[7].para_list[1] = 0x16;
			lcm_finger_lhbm_on_setting[8].para_list[1] = 0x10;
		}
		lcm_finger_lhbm_on_setting[4].para_list[1] = Beta >> 8;
		lcm_finger_lhbm_on_setting[4].para_list[2] = Beta & 0xFF;
		lcm_finger_lhbm_on_setting[4].para_list[3] = Alpha >> 8;
		lcm_finger_lhbm_on_setting[4].para_list[4] = Alpha & 0xFF;
		if (current_fps == 60) {
			lcm_finger_lhbm_on_setting[10].para_list[1] = 0x03;
			lcm_finger_lhbm_on_setting[12].para_list[1] = 0x03;
		} else if (current_fps == 90) {
			lcm_finger_lhbm_on_setting[10].para_list[1] = 0x02;
			lcm_finger_lhbm_on_setting[12].para_list[1] = 0x02;
		} else if (current_fps == 120) {
			lcm_finger_lhbm_on_setting[10].para_list[1] = 0x00;
			lcm_finger_lhbm_on_setting[12].para_list[1] = 0x00;
		}
		for (i = 0; i < sizeof(lcm_finger_lhbm_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_lhbm_on_setting[i].para_list, lcm_finger_lhbm_on_setting[i].count);
		}
	} else if (en == 0) {
		for (i = 0; i < sizeof(lcm_lhbm_off_setbrightness_normal)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_lhbm_off_setbrightness_normal[i].para_list, lcm_lhbm_off_setbrightness_normal[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	ctx->hbm_en = en;
	ctx->hbm_wait = true;
	return 0;
}

static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*state = ctx->hbm_en;
}

static void panel_hbm_set_state(struct drm_panel *panel, bool state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->hbm_en = state;
}

static void panel_hbm_get_wait_state(struct drm_panel *panel, bool *wait)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*wait = ctx->hbm_wait;
}

static bool panel_hbm_set_wait_state(struct drm_panel *panel, bool wait)
{
	struct lcm *ctx = panel_to_lcm(panel);
	bool old = ctx->hbm_wait;

	ctx->hbm_wait = wait;
	return old;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;
	struct mtk_dsi *mtk_dsi = dsi;
	struct drm_crtc *crtc = NULL;

	if (!dsi || !cb || !mtk_dsi) {
		return -EINVAL;
	}

	crtc = mtk_dsi->encoder.crtc;

	if (!crtc) {
		OFP_ERR("Invalid crtc param\n");
		return -EINVAL;
	}

	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	for (i = 0; i < (sizeof(AOD_off_setting) / sizeof(struct LCM_setting_table)); i++) {

		cmd = AOD_off_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count * 1000, AOD_off_setting[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count, AOD_off_setting[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, AOD_off_setting[i].para_list, AOD_off_setting[i].count);
		}
	}
	aod_state = false;
	if(!oplus_ofp_backlight_filter(crtc, handle, oplus_display_brightness))
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	pr_info("%s:success\n", __func__);
	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
    unsigned int cmd;
	aod_state = true;
	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	for (i = 0; i < (sizeof(AOD_on_setting)/sizeof(struct LCM_setting_table)); i++) {
		cmd = AOD_on_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(AOD_on_setting[i].count * 1000, AOD_on_setting[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(AOD_on_setting[i].count, AOD_on_setting[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				{
					cb(dsi, handle, AOD_on_setting[i].para_list, AOD_on_setting[i].count);
				}
		}
	}

	pr_info("%s:success\n", __func__);
	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_bl_level[i].para_list, aod_high_bl_level[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_bl_level[i].para_list, aod_low_bl_level[i].count);
		}
	}
	pr_info("%s:success %d !\n", __func__, level);

	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("%s: tianma_nt37703a lcm ctx->prepared %d\n", __func__, ctx->prepared);

	ctx->vddr1p2_enable_gpio =
		devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	usleep_range(5000, 5100);
	ctx->vci3p0_enable_gpio =
		devm_gpiod_get(ctx->dev, "vci-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci3p0_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vci3p0_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci3p0_enable_gpio));
		return PTR_ERR(ctx->vci3p0_enable_gpio);
	}
	gpiod_set_value(ctx->vci3p0_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci3p0_enable_gpio);
	usleep_range(10000, 10010);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

    pr_info("%s:Successful\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("%s: TM_nt37703a lcm ctx->prepared %d\n", __func__, ctx->prepared);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);
	ctx->vci3p0_enable_gpio =
		devm_gpiod_get(ctx->dev, "vci-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci3p0_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vci3p0_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci3p0_enable_gpio));
		return PTR_ERR(ctx->vci3p0_enable_gpio);
	}
	gpiod_set_value(ctx->vci3p0_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vci3p0_enable_gpio);
	usleep_range(5000, 5100);
	ctx->vddr1p2_enable_gpio =
		devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	usleep_range(5000, 5100);
	usleep_range(70000, 70100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	pr_info("%s:Successful\n", __func__);

	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared)
		return 0;

	pr_info("[LCM]debug for lcd reset :%s, ctx->prepared:%d\n", __func__, ctx->prepared);

	if(IS_ERR(ctx->reset_gpio)){
		pr_err("cannot get reset-gpios %ld\n",PTR_ERR(ctx->reset_gpio));
	}

	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(15000, 15100);

	return 0;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);
	pr_info("%s: mode=%d, vrefresh=%d\n", __func__, mode, drm_mode_vrefresh(m));

	if (m_vrefresh == 60) {
		ext->params = &ext_params_60Hz;
		current_fps = 60;
	} else if (m_vrefresh == 90) {
		ext->params = &ext_params_90Hz;
		current_fps = 90;
	}else if (m_vrefresh == 120) {
		ext->params = &ext_params_120Hz;
		current_fps = 120;
	} else {
		ret = 1;
	}

	return ret;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.panel_reset = lcm_panel_reset,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	//.mode_switch = mode_switch,
	.set_hbm = lcm_set_hbm,
	//.hbm_set_cmdq = panel_hbm_set_cmdq,
	.oplus_ofp_set_lhbm_pressed_icon_single = oplus_ofp_set_lhbm_pressed_icon_single,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,

	.hbm_get_state = panel_hbm_get_state,
	.hbm_set_state = panel_hbm_set_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
};

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode[3];

	mode[0] = drm_mode_duplicate(connector->dev, &disp_mode_60Hz);
	if (!mode[0]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_60Hz.hdisplay, disp_mode_60Hz.vdisplay, drm_mode_vrefresh(&disp_mode_60Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[0]);
	pr_info("%s clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n", __func__, mode[0]->clock, mode[0]->htotal,
		mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

	mode[1] = drm_mode_duplicate(connector->dev, &disp_mode_90Hz);
	if (!mode[1]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_90Hz.hdisplay, disp_mode_90Hz.vdisplay, drm_mode_vrefresh(&disp_mode_90Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[1]);
	mode[1]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[1]);

	mode[2] = drm_mode_duplicate(connector->dev, &disp_mode_120Hz);
	if (!mode[2]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_120Hz.hdisplay, disp_mode_120Hz.vdisplay, drm_mode_vrefresh(&disp_mode_120Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[2]);
	mode[2]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[2]);


	connector->display_info.width_mm = 69;
	connector->display_info.height_mm = 155;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;

	pr_info("[LCM] TM_nt377_novatek %s START\n", __func__);


	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	usleep_range(5000, 5100);
	ctx->vddr1p2_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vddr1p2_enable_gpio);

	usleep_range(5000, 5100);

	ctx->vci3p0_enable_gpio = devm_gpiod_get(dev, "vci-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci3p0_enable_gpio)) {
		dev_err(dev, "%s: cannot get vci3p0_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci3p0_enable_gpio));
		return PTR_ERR(ctx->vci3p0_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vci3p0_enable_gpio);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_60Hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	register_device_proc("lcd", "TMA_NT37703A", "TM_CASIOX");
	ctx->hbm_en = false;
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	oplus_ofp_init(dev);


	pr_info("[LCM] %s- lcm, TM_nt377_novatek, END\n", __func__);


	return ret;
}

static void lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "nt37703a,fhdp,dsi,vdo,120hz,dphy", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "nt37703a_fhdp_dsi_vdo_120hz_dphy",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
