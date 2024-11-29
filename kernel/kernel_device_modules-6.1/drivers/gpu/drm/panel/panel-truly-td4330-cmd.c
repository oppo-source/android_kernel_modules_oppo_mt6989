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
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
    struct gpio_desc *vddr1p2_enable_gpio;

	bool prepared;
	bool enabled;

	unsigned int gate_ic;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
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
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static struct regulator *wl28681c1_ldo7a;

static int lcm_panel_wl28681c1_ldo7a_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	wl28681c1_ldo7a = regulator_get(dev, "vci");
	if (IS_ERR(wl28681c1_ldo7a)) { /* handle return value */
		ret = PTR_ERR(wl28681c1_ldo7a);
		pr_err("get wl28681c1_ldo7a fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_wl28681c1_ldo7a_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_wl28681c1_ldo7a_regulator_init(dev);

	/* set voltage with min & max*/
	ret = regulator_set_voltage(wl28681c1_ldo7a, 3004000, 3004000);
	if (ret < 0)
		pr_err("set voltage wl28681c1_ldo7a fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(wl28681c1_ldo7a);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static int lcm_panel_wl28681c1_ldo7a_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_wl28681c1_ldo7a_regulator_init(dev);

	ret = regulator_disable(wl28681c1_ldo7a);
	if (ret < 0)
		pr_err("disable regulator wl28681c1_ldo7a fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(1 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(10 * 1000);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	pr_err("%s +\n", __func__);
	/* Hsync/Vsync */
	lcm_dcs_write_seq_static(ctx,0xF0,0x55,0xAA,0x52,0x08,0x01);
	lcm_dcs_write_seq_static(ctx,0xD1,0x07,0x02,0x0A,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01);
	/* Delete ELVDD Peak During Power On */
	lcm_dcs_write_seq_static(ctx,0xF0,0x55,0xAA,0x52,0x08,0x01);
	lcm_dcs_write_seq_static(ctx,0x6F,0x18);
	lcm_dcs_write_seq_static(ctx,0xD8,0x18);
	/* Optimize bright line on power-up */
	lcm_dcs_write_seq_static(ctx,0xF0,0x55,0xAA,0x52,0x08,0x01);
	lcm_dcs_write_seq_static(ctx,0xCD,0x61);
	/* Page 1 */
	lcm_dcs_write_seq_static(ctx,0xF0,0x55,0xAA,0x52,0x08,0x01);
	/* LVD OFF */
	lcm_dcs_write_seq_static(ctx,0x6F,0x03);
	lcm_dcs_write_seq_static(ctx,0xC7,0x00);
	/* OSC Select:CMD_Page0 */
	lcm_dcs_write_seq_static(ctx,0xFF,0xAA,0x55,0xA5,0x80);
	/* DSP OSC=121.9MHz */
	lcm_dcs_write_seq_static(ctx,0x6F,0x1A);
	lcm_dcs_write_seq_static(ctx,0xF4,0x55);
	lcm_dcs_write_seq_static(ctx,0x6F,0x15);
	lcm_dcs_write_seq_static(ctx,0xF8,0x01,0x51);
	/* CMD3_Page1 */
	lcm_dcs_write_seq_static(ctx,0xFF,0xAA,0x55,0xA5,0x81);
	/* Porch LD Disable */
	lcm_dcs_write_seq_static(ctx,0x6F,0x05);
	lcm_dcs_write_seq_static(ctx,0xFE,0x3C);
	/* WB_PS_Build_DLY */
	lcm_dcs_write_seq_static(ctx,0x6F,0x02);
	lcm_dcs_write_seq_static(ctx,0xF9,0x04);
	/* BOIS CLK Gated Turn On */
	lcm_dcs_write_seq_static(ctx,0x6F,0x0F);
	lcm_dcs_write_seq_static(ctx,0xF5,0x20);
	/* CMD3_Page3 */
	lcm_dcs_write_seq_static(ctx,0xFF,0xAA,0x55,0xA5,0x83);
	/* VRAM */
	lcm_dcs_write_seq_static(ctx,0x6F,0x12);
	lcm_dcs_write_seq_static(ctx,0xFE,0x41);
	/* Demura Sync */
	lcm_dcs_write_seq_static(ctx,0x6F,0x13);
	lcm_dcs_write_seq_static(ctx,0xFD,0x21);
	/* CMD Disable */
	lcm_dcs_write_seq_static(ctx,0xFF,0xAA,0x55,0xA5,0x00);
	/* CMD1 */
	lcm_dcs_write_seq_static(ctx,0x35);
	/* Dimming OFF */
	lcm_dcs_write_seq_static(ctx,0x53,0x20);
	/* CASET 1079 */
	lcm_dcs_write_seq_static(ctx,0x2A,0x00,0x00,0x04,0x37);
	/* PASET 2411 */
	lcm_dcs_write_seq_static(ctx,0x2B,0x00,0x00,0x09,0x6B);
	/* Normal GMA */
	lcm_dcs_write_seq_static(ctx,0x26,0x00);
	/* CMD1,DPC Temperature */
	lcm_dcs_write_seq_static(ctx,0x81,0x01,0x00);
	/* DSC Enable */
	lcm_dcs_write_seq_static(ctx,0x03,0x01);
	/* PPS Setting */
	lcm_dcs_write_seq_static(ctx,0x90,0x03,0x03);
	/* PPS Setting(10bit_3X) */
	lcm_dcs_write_seq_static(ctx,0x91,0xAB,0x28,0x00,0x0C,0xD2,0x00,0x02,0x25,0x01,0x14,0x00,0x07,0x09,0x75,0x08,0x7A,0x10,0xF0);
	/* Frame Transition 60Hz */
        lcm_dcs_write_seq_static(ctx,0x1F,0x40);
	lcm_dcs_write_seq_static(ctx,0x2F,0x02);
	lcm_dcs_write_seq_static(ctx,0xF0,0x55,0xAA,0x52,0x08,0x0A);
	lcm_dcs_write_seq_static(ctx,0x6F,0x6B);
	lcm_dcs_write_seq_static(ctx,0xB0,0x00);
	/* Sleep Out */
	lcm_dcs_write_seq_static(ctx,0x11);
	/* Wait>60ms,Actual 80ms */
	usleep_range(80000, 80100);
	/* Display On */
	lcm_dcs_write_seq_static(ctx,0x29);
	pr_err("%s -\n", __func__);
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

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;
    pr_err("%s:28 10\n", __func__);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
    msleep(5);
	lcm_panel_wl28681c1_ldo7a_disable(ctx->dev);
    msleep(5);
	ctx->vddr1p2_enable_gpio =
		devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
    msleep(5);
    msleep(70);
    pr_err("%s: poweroff\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

	ctx->vddr1p2_enable_gpio =
		devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
    msleep(5);
	lcm_panel_wl28681c1_ldo7a_enable(ctx->dev);
    msleep(10);

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

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

static const struct drm_display_mode default_mode = {
	.clock = 166800,
	.hdisplay = 1080,
	.hsync_start = 1080 + 9,//HFP
	.hsync_end = 1080 + 9 + 2,//HSA
	.htotal = 1080 + 9 + 2 + 21,//HBP
	.vdisplay = 2412,
	.vsync_start = 2412 + 52,//VFP
	.vsync_end = 2412 + 52 + 14,//VSA
	.vtotal = 2412 + 52+ 14 + 22,//VBP
};

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xFF};

	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	pr_err("backlight=%d,bl_tb0[1]=0x%x,bl_tb0[2]=0x%x \n", level,bl_tb0[1] ,bl_tb0[2]);
	return 0;
}

static struct mtk_panel_params ext_params = {
	.pll_clk = 300,
	.vendor = "td4330",
	.manufacture = "truly",
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.dsc_params = {
		.enable = 1,
		.dsc_cfg_change = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 549,
		.scale_value = 32,
		.increment_interval = 276,
		.decrement_interval = 7,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 2421,
		.slice_bpg_offset = 2170,
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
	},
	.data_rate = 600,
};

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
};

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 70;
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

	pr_info("%s+\n", __func__);

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
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET
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
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	msleep(5);
	ctx->vddr1p2_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vddr1p2_enable_gpio);

	msleep(5);

    lcm_panel_wl28681c1_ldo7a_enable(ctx->dev);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	register_device_proc("lcd", "td4330", "truly");

	pr_info("%s-\n", __func__);

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
	{ .compatible = "truly,td4330,cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-truly-td4330-cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init truly_drv_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register jdi driver: %d\n",
			__func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit truly_drv_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}
module_init(truly_drv_init);
module_exit(truly_drv_exit);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("truly td4330 CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
