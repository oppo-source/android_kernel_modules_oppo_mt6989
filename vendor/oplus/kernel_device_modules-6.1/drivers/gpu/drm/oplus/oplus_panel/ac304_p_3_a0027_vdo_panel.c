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
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"

#include "../../oplus/oplus_display_mtk_debug.h"
#define CONFIG_MTK_PANEL_EXT
#include "../../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
//#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#include "../../mediatek/mediatek_v2/mtk_dsi.h"
#include "../../mediatek/mediatek_v2/mtk-cmdq-ext.h"

#ifdef OPLUS_FEATURE_DISPLAY
#include "../../oplus/oplus_drm_disp_panel.h"
#include "../../oplus/oplus_display_temp_compensation.h"
#include "../../oplus/oplus_display_mtk_debug.h"
#endif /* OPLUS_FEATURE_DISPLAY */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "../../oplus/oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/of_address.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include "ac304_p_3_a0027_vdo_panel.h"

static unsigned int epd8820_vdo_dphy_buf_thresh[14] ={896, 1792, 2688, 3584, 4480,
	5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
static unsigned int epd8820_vdo_dphy_range_min_qp[15] ={0, 4, 5, 5, 7, 7, 7, 7, 7,
	7, 9, 9, 9, 13, 16};
static unsigned int epd8820_vdo_dphy_range_max_qp[15] ={8, 8, 9, 10, 11, 11, 11,
	12, 13, 14, 14, 15, 15, 16, 17};
static int epd8820_vdo_dphy_range_bpg_ofs[15] ={2, 0, 0, -2, -4, -6, -8, -8, -8,
	-10, -10, -12, -12, -12, -12};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr1p2_enable_gpio;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	bool hbm_en;
	bool hbm_wait;
	int error;
};
static unsigned int temp_seed_mode = 0;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern unsigned long seed_mode;
static int current_fps = 120;
static bool aod_state = false;

#define MAX_NORMAL_BRIGHTNESS   3083
#define LCM_BRIGHTNESS_TYPE 2
#define FINGER_HBM_BRIGHTNESS 3730

extern void lcdinfo_notify(unsigned long val, void *v);
extern int oplus_serial_number_probe(struct device *dev);

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

    lcm_dcs_write_seq_static(ctx, 0xF0,0x5A,0x5A);
	//AOD off
	lcm_dcs_write_seq_static(ctx, 0x90,0x40);

	//DSC Setting(DSCv1.2 1080*2372分辨率、10Bit、1/2 Slice Width、4 Slice Height)
	lcm_dcs_write_seq_static(ctx, 0x03,0x11);
	lcm_dcs_write_seq_static(ctx, 0xA3,0x12,0x00,0x00,0xAB,0x30,0x80,0x09,0x44,0x04,0x38,0x00,0x08,0x04,0x38,0x04,0x38,0x02,0x00,0x03,0x1C,0x00,0x20,0x00,0xF6,0x00,0x0F,0x00,0x0C,0x0D,0xB7,0x06,0x5C,0x18,0x00,0x10,0xF0,0x07,0x10,0x20,0x00);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x28);
	lcm_dcs_write_seq_static(ctx, 0xA3,0x06,0x0F,0x0F,0x33,0x0E,0x1C,0x2A,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7B,0x7D,0x7E,0x02,0x02,0x22,0x00,0x2A,0x40,0x2A,0xBE,0x3A,0xFC,0x3A,0xFA,0x3A,0xF8,0x3B,0x38,0x3B,0x78,0x3B,0xB6,0x4B,0xB6);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x50);
	lcm_dcs_write_seq_static(ctx, 0xA3,0x4B,0xF4,0x4B,0xF4,0x6C,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

	lcm_dcs_write_seq_static(ctx, 0x2A,0x00,0x00,0x04,0x37);
	lcm_dcs_write_seq_static(ctx, 0x2B,0x00,0x00,0x09,0x43);

	lcm_dcs_write_seq_static(ctx, 0x43,0x02,0x01);
	lcm_dcs_write_seq_static(ctx, 0x44,0x09,0x44);
	lcm_dcs_write_seq_static(ctx, 0x35,0x00);
	lcm_dcs_write_seq_static(ctx, 0x53,0x20);
	//HBM off & backlight off
	lcm_dcs_write_seq_static(ctx, 0x51,0x00,0x00);
	//set fps
	if (current_fps == 60) {
		lcm_dcs_write_seq_static(ctx, 0x86,0x12);
	} else if (current_fps == 90) {
		lcm_dcs_write_seq_static(ctx, 0x86,0x11);
	} else if (current_fps == 120) {
		lcm_dcs_write_seq_static(ctx, 0x86,0x10);
	}
	lcm_dcs_write_seq_static(ctx, 0x11);

	usleep_range(120*1000, 121*1000);

	lcm_dcs_write_seq_static(ctx, 0xF0,0x5A,0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x0A);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x20,0xCF);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x0C);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x00,0xCF);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x0E);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x00,0xCF);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x10);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x00,0xCF);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x1E);
	//swire pulse宽度normal区1.25H, AOD是0.75*4=3H
	lcm_dcs_write_seq_static(ctx, 0xC0,0x00,0xF0);
	lcm_dcs_write_seq_static(ctx, 0xB3,0x51,0x00,0x00,0x1F,0x01,0x3F,0x00,0x05,0x00,0x30,0x03,0x28,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x00,0x7D,0x0F,0x12,0x14,0x00,0x00,0x02,0x09);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x00);
	lcm_dcs_write_seq_static(ctx, 0xC9,0x9F);
	lcm_dcs_write_seq_static(ctx, 0xB0,0xF5);
	//IRC ON
	lcm_dcs_write_seq_static(ctx, 0xC9,0x20);
	//long V TE
	lcm_dcs_write_seq_static(ctx, 0xB0,0x08);
	lcm_dcs_write_seq_static(ctx, 0xB2,0x70);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x34);
	lcm_dcs_write_seq_static(ctx, 0xB2,0x00,0x00);
	//MIPI tracking 1130M
	lcm_dcs_write_seq_static(ctx, 0xBF,0x29,0x00,0x43,0xFF,0x03,0x11,0xF8);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x0C);
	lcm_dcs_write_seq_static(ctx, 0xBF,0x20);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x14);
	lcm_dcs_write_seq_static(ctx, 0xBF,0x01);
	//LVD ON
	lcm_dcs_write_seq_static(ctx, 0xF7,0xFC,0xA2,0x02,0x77,0x01,0x7C,0x37,0x07);

	lcm_dcs_write_seq_static(ctx, 0xB0,0xE5);
	lcm_dcs_write_seq_static(ctx, 0xC9,0x1A,0x13,0x0B,0x07,0x1D,0x15,0x0D,0x06,0x17,0x12,0x0B,0x06);
	lcm_dcs_write_seq_static(ctx, 0xB0,0xE3);
	lcm_dcs_write_seq_static(ctx, 0xC9,0x7F,0x00);
	lcm_dcs_write_seq_static(ctx, 0xC9,0x1F,0xA2,0x2A);
	lcm_dcs_write_seq_static(ctx, 0xB0,0xF1);
	lcm_dcs_write_seq_static(ctx, 0xC9,0xFF,0xE6,0xC8,0x87);

	lcm_dcs_write_seq_static(ctx, 0xB0,0x5A);
	lcm_dcs_write_seq_static(ctx, 0xC5,0x03,0xB0);

	lcm_dcs_write_seq_static(ctx, 0xF0,0xA5,0xA5);
	lcm_dcs_write_seq_static(ctx, 0x29);

	pr_info("debug for %s-\n", __func__);
}

static struct regulator *mt6363_vcn13;
static int lcm_panel_mt6363_vcn13_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	mt6363_vcn13 = devm_regulator_get_optional(dev, "vddr");
	if (IS_ERR_OR_NULL(mt6363_vcn13)) { /* handle return value */
		ret = PTR_ERR(mt6363_vcn13);
		pr_err("get mt6363_vcn13 fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */
}

static int lcm_panel_mt6363_vcn13_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_mt6363_vcn13_regulator_init(dev);

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(mt6363_vcn13)) {
		ret = regulator_set_voltage(mt6363_vcn13, 1200000, 1200000);
		if (ret < 0)
			pr_err("set voltage mt6363_vcn13 fail, ret = %d\n", ret);
		retval |= ret;
	}
	/* enable regulator */
	if (!IS_ERR_OR_NULL(mt6363_vcn13)) {
		ret = regulator_enable(mt6363_vcn13);
		if (ret < 0)
			pr_err("enable regulator mt6363_vcn13 fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("%s, retval = %d\n", __func__, retval);
	return retval;
}

static int lcm_panel_mt6363_vcn13_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_mt6363_vcn13_regulator_init(dev);

	if (!IS_ERR_OR_NULL(mt6363_vcn13)) {
		ret = regulator_disable(mt6363_vcn13);
		if (ret < 0)
			pr_err("disable regulator mt6363_vcn13 fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("%s, retval = %d\n", __func__, retval);
	return retval;
}

static struct regulator *mt6369_vmch;
static int lcm_panel_mt6369_vmch_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	mt6369_vmch = devm_regulator_get(dev, "vci");
	if (IS_ERR_OR_NULL(mt6369_vmch)) { /* handle return value */
		ret = PTR_ERR(mt6369_vmch);
		pr_err("get mt6369_vmch fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */
}

static int lcm_panel_mt6369_vmch_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_mt6369_vmch_regulator_init(dev);

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(mt6369_vmch)) {
		ret = regulator_set_voltage(mt6369_vmch, 3000000, 3000000);
		if (ret < 0)
			pr_err("set voltage mt6369_vmch fail, ret = %d\n", ret);
		retval |= ret;
	}
	/* enable regulator */
	if (!IS_ERR_OR_NULL(mt6369_vmch)) {
		ret = regulator_enable(mt6369_vmch);
		if (ret < 0)
			pr_err("enable regulator mt6369_vmch fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("%s, retval = %d\n", __func__, retval);
	return retval;
}

static int lcm_panel_mt6369_vmch_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_mt6369_vmch_regulator_init(dev);

	if (!IS_ERR_OR_NULL(mt6369_vmch)) {
		ret = regulator_disable(mt6369_vmch);
		if (ret < 0)
			pr_err("disable regulator mt6369_vmch fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("%s, retval = %d\n", __func__, retval);
	return retval;
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
	usleep_range(20000, 21000);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(120*1000, 121*1000);

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
#define FRAME_HEIGHT            (2376)
#define HFP                     (164)
#define HBP                     (32)
#define HSA                     (4)
#define VFP_60HZ                (2492)
#define VFP_90HZ                (860)
#define VFP_120HZ               (44)
#define VBP                     (24)
#define VSA                     (4)

#define HFP_30HZ                (2092)
#define HBP_30HZ                (32)
#define HSA_30HZ                (4)
#define VFP_30HZ                (44)
#define VBP_30HZ                (24)
#define VSA_30HZ                (4)

static u32 fake_heigh = 2372;
static u32 fake_width = 1080;
static bool need_fake_resolution;

static struct drm_display_mode disp_mode_60Hz = {
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

static struct drm_display_mode disp_mode_90Hz = {
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

static struct drm_display_mode disp_mode_120Hz = {
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

static struct drm_display_mode disp_mode_30Hz = {
	.clock = ((FRAME_WIDTH + HFP_30HZ + HBP_30HZ + HSA_30HZ) * (FRAME_HEIGHT + VFP_30HZ + VBP_30HZ + VSA_30HZ) * 30) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP_30HZ,
	.hsync_end = FRAME_WIDTH + HFP_30HZ + HSA_30HZ,
	.htotal = FRAME_WIDTH + HFP_30HZ + HSA_30HZ + HBP_30HZ,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_30HZ,
	.vsync_end = FRAME_HEIGHT + VFP_30HZ + VSA_30HZ,
	.vtotal = FRAME_HEIGHT + VFP_30HZ + VSA_30HZ + VBP_30HZ,
};

static struct mtk_panel_params ext_params_60Hz = {
	.pll_clk = 565,
	.data_rate = 1130,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0x86, 0x12}},
	},
	.dyn = {
		.switch_en = 0,
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

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_repeatedly_cnt = 4,
	.lcm_esd_check_table[0] = {
		.cmd = 0x03, .count = 1, .para_list[0] = 0x11,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xA2, .count = 4, .para_list[0] = 0x12, .para_list[1] = 0x00, .para_list[2] = 0x00, .para_list[3] = 0xAB,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lcm_esd_check_table[3] = {
		.cmd = 0xB1, .count = 1, .para_list[0] = 0x06,
	},
	.color_vivid_status = true,
	.vendor = "AC304_A0027",
	.manufacture = "P_3",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.panel_bpp = 10,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 0,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2376,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 1080,
		.chunk_size = 1080,
		.xmit_delay = 512,
		.dec_delay = 796,
		.scale_value = 32,
		.increment_interval = 246,
		.decrement_interval = 15,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 1628,
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
			.rc_buf_thresh = epd8820_vdo_dphy_buf_thresh,
			.range_min_qp = epd8820_vdo_dphy_range_min_qp,
			.range_max_qp = epd8820_vdo_dphy_range_max_qp,
			.range_bpg_ofs = epd8820_vdo_dphy_range_bpg_ofs,
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
		.dfps_cmd_table[0] = {0, 2 , {0x86, 0x11}},
	},
	.dyn = {
		.switch_en = 0,
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

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_repeatedly_cnt = 4,
	.lcm_esd_check_table[0] = {
		.cmd = 0x03, .count = 1, .para_list[0] = 0x11,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xA2, .count = 4, .para_list[0] = 0x12, .para_list[1] = 0x00, .para_list[2] = 0x00, .para_list[3] = 0xAB,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lcm_esd_check_table[3] = {
		.cmd = 0xB1, .count = 1, .para_list[0] = 0x06,
	},
	.color_vivid_status = true,
	.vendor = "AC304_A0027",
	.manufacture = "P_3",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.panel_bpp = 10,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 0,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2376,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 1080,
		.chunk_size = 1080,
		.xmit_delay = 512,
		.dec_delay = 796,
		.scale_value = 32,
		.increment_interval = 246,
		.decrement_interval = 15,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 1628,
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
			.rc_buf_thresh = epd8820_vdo_dphy_buf_thresh,
			.range_min_qp = epd8820_vdo_dphy_range_min_qp,
			.range_max_qp = epd8820_vdo_dphy_range_max_qp,
			.range_bpg_ofs = epd8820_vdo_dphy_range_bpg_ofs,
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
		.dfps_cmd_table[0] = {0, 2 , {0x86, 0x10}},
	},
	.dyn = {
		.switch_en = 0,
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

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_repeatedly_cnt = 4,
	.lcm_esd_check_table[0] = {
		.cmd = 0x03, .count = 1, .para_list[0] = 0x11,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xA2, .count = 4, .para_list[0] = 0x12, .para_list[1] = 0x00, .para_list[2] = 0x00, .para_list[3] = 0xAB,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lcm_esd_check_table[3] = {
		.cmd = 0xB1, .count = 1, .para_list[0] = 0x06,
	},
	.color_vivid_status = true,
	.vendor = "AC304_A0027",
	.manufacture = "P_3",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.panel_bpp = 10,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 0,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2376,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 1080,
		.chunk_size = 1080,
		.xmit_delay = 512,
		.dec_delay = 796,
		.scale_value = 32,
		.increment_interval = 246,
		.decrement_interval = 15,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 1628,
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
			.rc_buf_thresh = epd8820_vdo_dphy_buf_thresh,
			.range_min_qp = epd8820_vdo_dphy_range_min_qp,
			.range_max_qp = epd8820_vdo_dphy_range_max_qp,
			.range_bpg_ofs = epd8820_vdo_dphy_range_bpg_ofs,
			},
	},
};

static struct mtk_panel_params ext_params_30Hz = {
	.pll_clk = 565,
	.data_rate = 1130,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 30,
	},
	.dyn = {
		.switch_en = 0,
		.pll_clk = 572,
		.data_rate = 1144,
		.hfp = 148,
		.vfp = VFP_30HZ,
		.vsa = VSA_30HZ,
		.vbp = VBP_30HZ,
		.hsa = HSA_30HZ,
		.hbp = HBP_30HZ,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.color_vivid_status = true,
	.vendor = "AC304_A0027",
	.manufacture = "P_3",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.panel_bpp = 10,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 0,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2376,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 1080,
		.chunk_size = 1080,
		.xmit_delay = 512,
		.dec_delay = 796,
		.scale_value = 32,
		.increment_interval = 246,
		.decrement_interval = 15,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 1628,
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
			.rc_buf_thresh = epd8820_vdo_dphy_buf_thresh,
			.range_min_qp = epd8820_vdo_dphy_range_min_qp,
			.range_max_qp = epd8820_vdo_dphy_range_max_qp,
			.range_bpg_ofs = epd8820_vdo_dphy_range_bpg_ofs,
			},
	},
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_get_virtual_heigh(void)
{
	return FRAME_HEIGHT;
}

static int lcm_get_virtual_width(void)
{
	return FRAME_WIDTH;
}

static void check_is_need_fake_resolution(struct device *dev)
{
	unsigned int ret = 0;

	ret = of_property_read_u32(dev->of_node, "fake-heigh", &fake_heigh);
	if (ret)
		need_fake_resolution = false;
	ret = of_property_read_u32(dev->of_node, "fake-width", &fake_width);
	if (ret)
		need_fake_resolution = false;
	if (fake_heigh > 0 && fake_heigh < FRAME_HEIGHT)
		need_fake_resolution = true;
	if (fake_width > 0 && fake_width < FRAME_WIDTH)
		need_fake_resolution = true;

	DISP_INFO("need_fake_resolution:%d,%s-\n", need_fake_resolution,__func__);
}

static void change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	int vtotal = mode->vtotal;
	int htotal = mode->htotal;
	int fps = mode->clock * 1000 / vtotal / htotal;

	if (fake_heigh > 0 && fake_heigh < FRAME_HEIGHT) {
		mode->vsync_start = mode->vsync_start - mode->vdisplay
					+ fake_heigh;
		mode->vsync_end = mode->vsync_end - mode->vdisplay + fake_heigh;
		mode->vtotal = mode->vtotal - mode->vdisplay + fake_heigh;
		mode->vdisplay = fake_heigh;
	}
	if (fake_width > 0 && fake_width < FRAME_WIDTH) {
		mode->hsync_start = mode->hsync_start - mode->hdisplay
					+ fake_width;
		mode->hsync_end = mode->hsync_end - mode->hdisplay + fake_width;
		mode->htotal = mode->htotal - mode->hdisplay + fake_width;
		mode->hdisplay = fake_width;
	}

	if (fps > 100)
		fps = 120;
	else if (fps > 70)
		fps = 90;
	else if (fps > 40)
		fps = 60;
	else
		fps = 30;
	mode->clock = fps * mode->vtotal * mode->htotal / 1000;
	mode->clock += 1;

	DISP_INFO("mode->vdisplay:%d,%s-\n",mode->vdisplay, __func__);
}

static int bl_demura_mode = 5;
static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};
	int index = 13;
	int i = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 0) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	}

	if (level == 1) {
		DISP_ERR("[%s:%d]skip set backlight lvl:%u\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4094) {
		level = 4094;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = 2047;
	}

	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	DISP_ERR("ac304_p_3_a0027 backlight = %d bl_level[1]=%x, bl_level[2]=%x\n", level, bl_level[1], bl_level[2]);
	oplus_display_brightness = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &level);

	/* demura switch */
	if ((level > 1) && (level < 1147) && (bl_demura_mode != 0)) {
		bl_demura_mode = 0;
		index = sizeof(dsi_demura0_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
		}
		pr_info("demura DBV switch to bl_demura_mode%d\n", bl_demura_mode);
	} else if ((level >= 1147) && (level <= 4094) && (bl_demura_mode != 1)) {
		bl_demura_mode = 1;
		index = sizeof(dsi_demura1_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
		}
		pr_info("demura DBV switch to bl_demura_mode%d\n", bl_demura_mode);
	}

	return 0;
}

static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int mode)
{
	unsigned int i = 0;
	pr_info("[DISP][INFO][%s: mode=%d\n", __func__, mode);
	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	temp_seed_mode = mode;

	switch(mode) {
		case NATURAL:
			for(i = 0; i < sizeof(dsi_set_seed_natural)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_natural[i].para_list, dsi_set_seed_natural[i].count);
			}
		break;
		case EXPERT:
			for(i = 0; i < sizeof(dsi_set_seed_expert)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_expert[i].para_list, dsi_set_seed_expert[i].count);
			}
		break;
		default:
		break;
	}
	return 0;
}


static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char esd_bl_level[] = {0x51, 0x03, 0xFF};
	int index = 13;
	int i = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	esd_bl_level[1] = level >> 8;
	esd_bl_level[2] = level & 0xFF;
	cb(dsi, handle, esd_bl_level, ARRAY_SIZE(esd_bl_level));
	pr_info("esd_bl_level[1]=%x, esd_bl_level[2]=%x\n", esd_bl_level[1], esd_bl_level[2]);

	if ((level > 1) && (level < 1147)) {
		bl_demura_mode = 0;
		index = sizeof(dsi_demura0_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
		}
	} else if ((level >= 1147) && (level <= 4094)) {
		bl_demura_mode = 1;
		index = sizeof(dsi_demura1_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
		}
	}
	pr_info("level=%d, bl_demura_mode=%d\n", level, bl_demura_mode);
	return 0;
}

static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;

	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	pr_info("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, hbm_mode);
	if (hbm_mode == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, hbm_on_cmd[i].para_list, hbm_on_cmd[i].count);
		}
	} else if (hbm_mode == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, hbm_off_cmd[i].para_list, hbm_off_cmd[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	return 0;
}

static int oplus_ofp_set_lhbm_pressed_icon_single(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	struct lcm *ctx = NULL;
	int i = 0;
	struct LCM_setting_table *lhbm_pressed_icon_cmd = NULL;
	//unsigned int Beta = 0;
	//unsigned int Alpha = 0;
	unsigned int reg_count = 0;

	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid ctx params\n");
	}

	pr_info("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, en);
	if (en) {
		reg_count = sizeof(lcm_lhbm_on_setting) / sizeof(struct LCM_setting_table);
		lhbm_pressed_icon_cmd = lcm_lhbm_on_setting;
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_cmd[i].para_list, lhbm_pressed_icon_cmd[i].count);
		}
	} else if (en == 0) {
		reg_count = sizeof(lcm_lhbm_off_setting) / sizeof(struct LCM_setting_table);
		lhbm_pressed_icon_cmd = lcm_lhbm_off_setting;
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_cmd[i].para_list, lhbm_pressed_icon_cmd[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
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
	if (temp_seed_mode)
		panel_set_seed(dsi, cb, handle, temp_seed_mode);
	pr_info("%s:success\n", __func__);
	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;
	unsigned int reg_count = 0;
	aod_state = true;
	struct LCM_setting_table *lhbm_pressed_icon_off_cmd = NULL;
	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	if(oplus_ofp_local_hbm_is_enabled() && oplus_ofp_get_hbm_state()) {
		reg_count = sizeof(lcm_lhbm_off_setting) / sizeof(struct LCM_setting_table);
		lhbm_pressed_icon_off_cmd = lcm_lhbm_off_setting;
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_off_cmd[i].para_list, lhbm_pressed_icon_off_cmd[i].count);
		}
		OFP_INFO("should off hbm\n");
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

static struct vdo_aod_params vdo_aod_on = {
	.porch_change_flag = 0x03,
	.dst_hfp = 2092,
	.dst_vfp = 44, //30fps
	.mode_idx = FHD_SDC30,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0x90,0x41}},
	.vdo_aod_cmd_table[1]={5, {0x51,0x00,0x00,0x0F,0xFE}},
};


static struct vdo_aod_params vdo_aod_to_120hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 164,
	.dst_vfp = 44,
	.mode_idx = 0,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0x90,0x40}},
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},

};

static struct vdo_aod_params vdo_aod_to_120hz = {
        .porch_change_flag = 0x03,
        .dst_hfp = 164,
        .dst_vfp = 44,
        .mode_idx = 0,
        .change_mmclk = true,
        .vdo_aod_cmd_table[0]={2, {0x90,0x40}},
};

static struct vdo_aod_params vdo_aod_to_90hz_unlocking = {
        .porch_change_flag = 0x03,
        .dst_hfp = 164,
        .dst_vfp = 860,
        .mode_idx = 1,
        .change_mmclk = true,
        .vdo_aod_cmd_table[0]={2, {0x90,0x40}},
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_90hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 164,
	.dst_vfp = 860,
	.mode_idx = 1,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0x90,0x40}},
};

static struct vdo_aod_params vdo_aod_to_60hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 164,
	.dst_vfp = 2492,
	.mode_idx = 2,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0x90,0x40}},

};

static struct vdo_aod_params vdo_aod_to_60hz_unlocking = {
        .porch_change_flag = 0x03,
        .dst_hfp = 164,
        .dst_vfp = 2492,
        .mode_idx = 2,
        .change_mmclk = true,
        .vdo_aod_cmd_table[0]={2, {0x90,0x40}},
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},
};

static int mtk_get_vdo_aod_param(int aod_en, struct vdo_aod_params **vdo_aod_param)
{
	static int mode_id_before_aod = 120;
	if (aod_en) {
		*vdo_aod_param = &vdo_aod_on;
		mode_id_before_aod = current_fps;
	} else {
		if (mode_id_before_aod == 60) {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_60hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_60hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		} else if (mode_id_before_aod == 90) {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_90hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_90hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		} else {
			if(oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_120hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_120hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		}
	}
	OFP_INFO("%s:aod_en %d, mode_id %d, unlocking =%d\n", __func__, aod_en, current_fps, oplus_ofp_get_aod_unlocking());
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

	DISP_ERR("%s: ac304_p_3_a0027 poweron Start\n", __func__);

	//enable ldo 3p0
	lcm_panel_mt6369_vmch_enable(ctx->dev);

	/* Wait no limits, actual 3ms */
	usleep_range(5000, 5100);

	/* vddr-oled 1p2 enable */
	lcm_panel_mt6363_vcn13_enable(ctx->dev);

	/* Wait > 10ms, actual 12ms */
	usleep_range(22000, 22100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	bl_demura_mode = 5;

	DISP_ERR("%s: ac304_p_3_a0027 poweron Successful\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("%s: ac304_p_3_a0027 lcm ctx->prepared %d\n", __func__, ctx->prepared);

	// set reset 0
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);

	/* vddr-oled 1p2 disable */
	lcm_panel_mt6363_vcn13_disable(ctx->dev);

	usleep_range(5000, 5100);

	/* vci-oled 3p0 disable */
	lcm_panel_mt6369_vmch_disable(ctx->dev);

	usleep_range(10000, 10100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	pr_info("%s:ac304_p_3_a0027 Successful\n", __func__);

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
	} else if (m_vrefresh == 30) {
		ext->params = &ext_params_30Hz;
		current_fps = 30;
	} else {
		ret = 1;
	}

	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	struct drm_display_mode *m = get_mode_by_id(connector, id);
	int m_vrefresh = 0;
	int ret = 0;

	if (m == NULL) {
		DISP_ERR("ac304_p_3_a0027 %s display_mode m = NULL!\n", __func__);
		return -EINVAL;
	}

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60) {
		*ext_param = &ext_params_60Hz;
	} else if (m_vrefresh == 90) {
		*ext_param = &ext_params_90Hz;
	} else if (m_vrefresh == 120) {
		*ext_param = &ext_params_120Hz;
	} else {
		*ext_param = &ext_params_60Hz;
	}

	if (*ext_param)
		DISP_DEBUG("ac304_p_3_a0027 data_rate:%d\n", (*ext_param)->data_rate);
	else
		DISP_ERR("ac304_p_3_a0027 ext_param is NULL;\n");

	return ret;
}

static int lcm_esd_check_multipage_pre(void *dsi, dcs_write_gce cb, void *handle, int i)
{
	int j = 0;

	if (i != 3)
		return 0;

	for (j = 0; j < sizeof(esd_check_multipage_pre)/sizeof(struct LCM_setting_table); j++) {
		cb(dsi, handle, esd_check_multipage_pre[j].para_list, esd_check_multipage_pre[j].count);
	}
	DISP_INFO("success.\n");
	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.panel_reset = lcm_panel_reset,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	//.mode_switch = mode_switch,
	.set_hbm = lcm_set_hbm,
	//.hbm_set_cmdq = panel_hbm_set_cmdq,
	.oplus_ofp_set_lhbm_pressed_icon_single = oplus_ofp_set_lhbm_pressed_icon_single,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.esd_check_multipage_pre = lcm_esd_check_multipage_pre,
	.hbm_get_state = panel_hbm_get_state,
	.hbm_set_state = panel_hbm_set_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
	.set_seed = panel_set_seed,
	.get_vdo_aod_param = mtk_get_vdo_aod_param,
};

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode[4];

	if (need_fake_resolution) {
		change_drm_disp_mode_params(&disp_mode_60Hz);
		change_drm_disp_mode_params(&disp_mode_90Hz);
		change_drm_disp_mode_params(&disp_mode_120Hz);
		change_drm_disp_mode_params(&disp_mode_30Hz);
	}

	mode[0] = drm_mode_duplicate(connector->dev, &disp_mode_60Hz);
	if (!mode[0]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_60Hz.hdisplay, disp_mode_60Hz.vdisplay, drm_mode_vrefresh(&disp_mode_60Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER;
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
	mode[2]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[2]);

	mode[3] = drm_mode_duplicate(connector->dev, &disp_mode_30Hz);
	if (!mode[3]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_30Hz.hdisplay, disp_mode_30Hz.vdisplay, drm_mode_vrefresh(&disp_mode_30Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[3]);
	mode[3]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[3]);

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

	pr_info("[LCM] ac304_p_3_a0027 %s START\n", __func__);


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

	//enable ldo 3p0
	lcm_panel_mt6369_vmch_enable(ctx->dev);

	usleep_range(5000, 5100);

	/* vddr-oled 1p2 enable */
	lcm_panel_mt6363_vcn13_enable(ctx->dev);

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

	oplus_serial_number_probe(dev);
	register_device_proc("lcd", "AC304_A0027", "P_3");
	ctx->hbm_en = false;
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	oplus_ofp_init(dev);

	check_is_need_fake_resolution(dev);

	pr_info("[LCM] %s- lcm, ac304_p_3_a0027, END\n", __func__);


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
	{
		.compatible = "ac304,p,3,a0027,vdo,panel",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "ac304_p_3_a0027_vdo_panel",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
