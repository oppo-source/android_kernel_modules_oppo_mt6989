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
#include <linux/mtk_disp_notify.h>
#include <soc/oplus/system/boot_mode.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#include "../mediatek/mediatek_v2/mtk_dsi.h"
#endif

#include "oplus25680_td4376b_fhdp_hx_dsi_vdo.h"
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#include "../oplus/oplus_display_onscreenfingerprint.h"
#include "../bias/oplus23661_aw37501_bias.h"

struct lcm {
    struct device *dev;
    struct drm_panel panel;
    struct backlight_device *backlight;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *bias_pos;
    struct gpio_desc *bias_neg;
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
extern unsigned int oplus_enhance_mipi_strength;
static int current_fps = 60;
static bool aod_state = false;
static int cabc_mode_backup = 3;
static int map_exp[4096] = {0};

#define MAX_NORMAL_BRIGHTNESS   2850
#define LCM_BRIGHTNESS_TYPE 2
#define FINGER_HBM_BRIGHTNESS 4050

/*TP define*/
#define LCD_CTL_TP_LOAD_FW 0x10
#define LCD_CTL_RST_ON 0x11
#define LCD_CTL_RST_OFF 0x12
#define LCD_CTL_CS_ON   0x19
#define LCD_CTL_CS_OFF  0x1A
#define LCD_CTL_IRQ_ON  0x1B
#define LCD_CTL_IRQ_OFF 0x1C
#define LCD_CTL_AOD_OFF 0x1D
#define MTK_DISP_EVENT_FOR_TOUCH     0x10

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
#endif
extern bool g_shutdown;

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
    pr_info("hx td4376b debug for %s+\n", __func__);



    lcm_dcs_write_seq_static(ctx, 0xB0, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x5E, 0x01, 0x30);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xBE, 0x01, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB8, 0x00, 0x51);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB8, 0x01, 0x59);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB8, 0x02, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB8, 0x03, 0xbe);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB8, 0x04, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB8, 0x05, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB8, 0x06, 0x0a);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB9, 0x00, 0x59);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB9, 0x01, 0x59);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB9, 0x02, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB9, 0x03, 0xbe);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB9, 0x04, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB9, 0x05, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xB9, 0x06, 0x0a);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xBA, 0x00, 0xa1);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xBA, 0x01, 0x65);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xBA, 0x02, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xBA, 0x03, 0xbe);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xBA, 0x04, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xBA, 0x05, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xBA, 0x06, 0x10);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x00, 0x77);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x01, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x02, 0x04);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x03, 0x10);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x04, 0x31);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x05, 0x33);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x06, 0x3f);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x07, 0x4b);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x08, 0x57);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x09, 0x63);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0A, 0x79);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0B, 0x87);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0C, 0x93);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0D, 0xa4);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0E, 0xb4);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0F, 0xc0);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x10, 0xcd);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x11, 0xd2);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x12, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x14, 0x40);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x15, 0x40);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x1B, 0x43);
    lcm_dcs_write_seq_static(ctx, 0xC0, 0x00, 0x4c, 0x01, 0x2c, 0x08, 0x09, 0x60, 0x00, 0x0c, 0x22, 0x00, 0x08, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x2c, 0x01, 0x2c, 0x01, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xC1, 0x30, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00, 0x22, 0x00, 0x05, 0x20, 0x00, 0x80, 0xfa, 0x40, 0x00, 0x84, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x2c, 0x08, 0x08, 0xbf, 0x0b, 0x54, 0x11, 0x94, 0x15, 0x7c, 0x18, 0x88, 0x19, 0xc8, 0x20, 0x6c, 0x81, 0x13, 0xb4, 0xff, 0xf5, 0x84, 0x7a, 0xa0, 0x43, 0xfe, 0xb3, 0x4c, 0x22, 0x06, 0x14, 0x1b, 0x62, 0xf2, 0x58, 0x83, 0xe0, 0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x64, 0xA1, 0x00, 0x01, 0x00, 0x0E, 0x37, 0xD0);
    lcm_dcs_write_seq_static(ctx, 0xD7, 0x21, 0x10, 0x52, 0x52, 0x00, 0x4c, 0x00, 0x0c, 0x00, 0x4c, 0x04, 0xfd, 0x01, 0x00, 0x03, 0x00, 0x05, 0x05, 0x00, 0x03, 0x04, 0x05, 0x00, 0x04, 0x00, 0x08, 0x02, 0x08, 0x06, 0x03, 0x08, 0x04, 0x08, 0x10, 0x0c, 0x0b, 0x0a, 0x0a, 0x0a, 0x07, 0x07, 0x06, 0x06, 0x00, 0x08, 0x08, 0x04, 0x05, 0x09, 0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x02, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x06, 0x06, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xDE, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x4c);
    lcm_dcs_write_seq_static(ctx, 0xEA, 0x01, 0x07, 0x11, 0x10, 0x17, 0x00, 0x00, 0x00, 0x09, 0x00, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xc2, 0x00, 0x12, 0x00, 0x50, 0x05, 0x00, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xEC, 0x08, 0x20, 0x00, 0x11, 0x1F, 0x17, 0x00, 0x00, 0x02, 0x3a);
    lcm_dcs_write_seq_static(ctx, 0xED, 0x01, 0x01, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xec, 0x10, 0x00);
    lcm_dcs_write_seq_static(ctx, 0x53, 0x2c);
    lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
    lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xD6, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xB0, 0x03);
    lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
    usleep_range(120000, 120100);
    lcm_dcs_write_seq_static(ctx, 0x29, 0x00);
    usleep_range(5000, 5100);

    pr_info("hx td4376b debug for %s-\n", __func__);
}

static void cabc_mode_retore(struct lcm *ctx)
{
    lcm_dcs_write_seq_static(ctx, 0x53, 0x2c);
    if (cabc_mode_backup == 1) {
        lcm_dcs_write_seq_static(ctx, 0x55, 0x01);
    } else if (cabc_mode_backup == 2) {
        lcm_dcs_write_seq_static(ctx, 0x55, 0x02);
    } else if (cabc_mode_backup == 3) {
        lcm_dcs_write_seq_static(ctx, 0x55, 0x03);
    } else if (cabc_mode_backup == 0) {
        lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
    }
    pr_info("%s- cabc_mode_backup=%d\n", __func__, cabc_mode_backup);
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
    pr_info("%s:hx prepared=%d\n", __func__, ctx->prepared);

    if (!ctx->prepared)
        return 0;

    lcm_dcs_write_seq_static(ctx, 0x28);
    usleep_range(10000, 11000);
    lcm_dcs_write_seq_static(ctx, 0x10);
    usleep_range(150*1000, 151*1000);

    ctx->error = 0;
    ctx->prepared = false;
    //ctx->hbm_en = false;
    pr_info("%s:hx success\n", __func__);

    return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
    struct lcm *ctx = panel_to_lcm(panel);
    int ret;

    pr_info("%s:hx prepared=%d\n", __func__, ctx->prepared);
    if (ctx->prepared)
        return 0;

    lcm_panel_init(ctx);
    cabc_mode_retore(ctx);

    ret = ctx->error;
    if (ret < 0)
        lcm_unprepare(panel);

    ctx->prepared = true;
    pr_info("%s:hx success\n", __func__);
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
#define FRAME_HEIGHT            (2400)
#define HFP                     (198)
#define HFP_144                 (96)
#define HBP                     (32)
#define HSA                     (4)
#define VFP_30HZ                (7744)
#define VFP_45HZ                (4352)
#define VFP_48HZ                (3928)
#define VFP_50HZ                (3674)
#define VFP_60HZ                (2656)
#define VFP_90HZ                (960)
#define VFP_120HZ               (112)
#define VFP_144HZ               (64)
#define VBP                     (28)
#define VSA                     (4)
#define PLL_CLOCK               (624)
#define DATA_RATE               (1248)

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

static const struct drm_display_mode disp_mode_144Hz = {
    .clock = ((FRAME_WIDTH + HFP_144 + HBP + HSA) * (FRAME_HEIGHT + VFP_144HZ + VBP + VSA) * 144) / 1000,
    .hdisplay = FRAME_WIDTH,
    .hsync_start = FRAME_WIDTH + HFP_144,
    .hsync_end = FRAME_WIDTH + HFP_144 + HSA,
    .htotal = FRAME_WIDTH + HFP_144 + HSA + HBP,
    .vdisplay = FRAME_HEIGHT,
    .vsync_start = FRAME_HEIGHT + VFP_144HZ,
    .vsync_end = FRAME_HEIGHT + VFP_144HZ + VSA,
    .vtotal = FRAME_HEIGHT + VFP_144HZ + VSA + VBP,
};

static struct mtk_panel_params ext_params_60Hz = {
    .pll_clk = PLL_CLOCK,
    .data_rate = DATA_RATE,
    .change_fps_by_vfp_send_cmd_need_delay = 1,
    .dyn_fps = {
        .switch_en = 1,
        .vact_timing_fps = 60,
    },
    .output_mode = MTK_PANEL_DSC_SINGLE_PORT,

    .vdo_mix_mode_en = false,

    .oplus_display_global_dre = 1,
    .oplus_custom_hdr_color_tmp = true,
    .oplus_custom_hdr_red = 950,
    .oplus_custom_hdr_green = 1024,
    .oplus_custom_hdr_blue = 800,
    .oplus_panel_use_rgb_gain = true,

    .vendor = "HX_TD4376B",
    .manufacture = "HX_SUZUKI",

    .oplus_ofp_need_keep_apart_backlight = true,
    .oplus_ofp_hbm_on_delay = 11,
    .oplus_ofp_pre_hbm_off_delay = 2,
    .oplus_ofp_hbm_off_delay = 11,
    .oplus_uiready_before_time = 17,
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
    .oplus_display_lcd_tp_aod = 1,

    .dsc_params = {
        .enable = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 34,
        .rct_on = 1,
        .bit_per_channel = 8,
        .dsc_line_buf_depth = 9,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2400,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 3,
        .flatness_maxqp = 12,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 11,
        .rc_quant_incr_limit1 = 11,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
    },
};

static struct mtk_panel_params ext_params_90Hz = {
    .pll_clk = PLL_CLOCK,
    .data_rate = DATA_RATE,
    .change_fps_by_vfp_send_cmd_need_delay = 1,
    .dyn_fps = {
        .switch_en = 1,
        .vact_timing_fps = 90,
    },
    .output_mode = MTK_PANEL_DSC_SINGLE_PORT,

    .vdo_mix_mode_en = false,

    .oplus_display_global_dre = 1,
    .oplus_custom_hdr_color_tmp = true,
    .oplus_custom_hdr_red = 950,
    .oplus_custom_hdr_green = 1024,
    .oplus_custom_hdr_blue = 800,
    .oplus_panel_use_rgb_gain = true,

    .vendor = "HX_TD4376B",
    .manufacture = "HX_SUZUKI",

    .oplus_ofp_need_keep_apart_backlight = true,
    .oplus_ofp_hbm_on_delay = 11,
    .oplus_ofp_pre_hbm_off_delay = 2,
    .oplus_ofp_hbm_off_delay = 11,
    .oplus_uiready_before_time = 17,
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
    .oplus_display_lcd_tp_aod = 1,

    .dsc_params = {
        .enable = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 34,
        .rct_on = 1,
        .bit_per_channel = 8,
        .dsc_line_buf_depth = 9,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2400,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 3,
        .flatness_maxqp = 12,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 11,
        .rc_quant_incr_limit1 = 11,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
    },
};

static struct mtk_panel_params ext_params_120Hz = {
    .pll_clk = PLL_CLOCK,
    .data_rate = DATA_RATE,
    .change_fps_by_vfp_send_cmd_need_delay = 1,
    .dyn_fps = {
        .switch_en = 1,
        .vact_timing_fps = 120,
    },
    .output_mode = MTK_PANEL_DSC_SINGLE_PORT,

    .vdo_mix_mode_en = false,

    .oplus_display_global_dre = 1,
    .oplus_custom_hdr_color_tmp = true,
    .oplus_custom_hdr_red = 950,
    .oplus_custom_hdr_green = 1024,
    .oplus_custom_hdr_blue = 800,
    .oplus_panel_use_rgb_gain = true,

    .vendor = "HX_TD4376B",
    .manufacture = "HX_SUZUKI",
    .cust_esd_check = 1,
    .esd_check_enable = 1,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
    },

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
        .dsc_cfg = 34,
        .rct_on = 1,
        .bit_per_channel = 8,
        .dsc_line_buf_depth = 9,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2400,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 3,
        .flatness_maxqp = 12,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 11,
        .rc_quant_incr_limit1 = 11,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
    },
};

static struct mtk_panel_params ext_params_144Hz = {
    .pll_clk = PLL_CLOCK,
    .data_rate = DATA_RATE,
    .change_fps_by_vfp_send_cmd_need_delay = 1,
    .dyn_fps = {
        .switch_en = 1,
        .vact_timing_fps = 144,
    },

    .output_mode = MTK_PANEL_DSC_SINGLE_PORT,

    .vdo_mix_mode_en = false,

    .oplus_display_global_dre = 1,
    .oplus_custom_hdr_color_tmp = true,
    .oplus_custom_hdr_red = 950,
    .oplus_custom_hdr_green = 1024,
    .oplus_custom_hdr_blue = 800,
    .oplus_panel_use_rgb_gain = true,

    .vendor = "HX_TD4376B",
    .manufacture = "HX_SUZUKI",

    .oplus_ofp_need_keep_apart_backlight = true,
    .oplus_ofp_hbm_on_delay = 11,
    .oplus_ofp_pre_hbm_off_delay = 2,
    .oplus_ofp_hbm_off_delay = 11,
    .oplus_uiready_before_time = 17,
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
    .oplus_display_lcd_tp_aod = 1,

    .dsc_params = {
        .enable = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 34,
        .rct_on = 1,
        .bit_per_channel = 8,
        .dsc_line_buf_depth = 9,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2400,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 3,
        .flatness_maxqp = 12,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 11,
        .rc_quant_incr_limit1 = 11,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
    },
};

static int panel_ata_check(struct drm_panel *panel)
{
    /* Customer test by own ATA tool */
    return 1;
}

static void init_global_exp_backlight(void)
{
    int lut_index[41] = {0, 4, 99, 144, 187, 227, 264, 300, 334, 366, 397, 427, 456, 484, 511, 537, 563, 587, 611, 635, 658, 680,
                        702, 723, 744, 764, 784, 804, 823, 842, 861, 879, 897, 915, 933, 950, 967, 984, 1000, 1016, 1023};
    int lut_value1[41] = {0, 4, 6, 14, 24, 37, 52, 69, 87, 107, 128, 150, 173, 197, 222, 248, 275, 302, 330, 358, 387, 416, 446,
                        479, 509, 541, 572, 604, 636, 669, 702, 735, 769, 803, 837, 871, 905, 938, 973, 1008, 1023};
    int index_start = 0, index_end = 0;
    int value1_start = 0, value1_end = 0;
    int i, j;
    int index_len = sizeof(lut_index) / sizeof(int);
    int value_len = sizeof(lut_value1) / sizeof(int);
    if (index_len == value_len) {
        for (i = 0; i < index_len - 1; i++) {
            index_start = lut_index[i] * MAX_NORMAL_BRIGHTNESS / 1023;
            index_end = lut_index[i+1] * MAX_NORMAL_BRIGHTNESS / 1023;
            value1_start = lut_value1[i] * MAX_NORMAL_BRIGHTNESS / 1023;
            value1_end = lut_value1[i+1] * MAX_NORMAL_BRIGHTNESS / 1023;
            for (j = index_start; j <= index_end; j++) {
                map_exp[j] = value1_start + (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
            }
        }
    }
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
    char bl_tb0[] = {0x51, 0x0F, 0xFF};
    //char bl_tb1[] = {0x53, 0x24};
    //char bl_tb2[] = {0x28};
    //char bl_tb3[] = {0x29};
    int bl_map = 0;

    if (!dsi || !cb) {
        return -EINVAL;
    }

    bl_map = (level > BRIGHTNESS_MAX) ? BRIGHTNESS_MAX:level;

    if (bl_map > 0 && bl_map < 8) {
        pr_info("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, bl_map);
        return -EINVAL;
    }

    if ((get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) && (bl_map > 1))
        bl_map = 1023;

    if (bl_map > 0 && bl_map <= BRIGHTNESS_MAX) {
        lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &bl_map);
        oplus_display_brightness = bl_map;
    }

    if ((bl_map > 0) && (level < MAX_NORMAL_BRIGHTNESS)) {
        bl_map = map_exp[bl_map];
    }

    bl_tb0[1] = bl_map >> 8;
    bl_tb0[2] = bl_map & 0xFF;
    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
    pr_info("%s,level = %d, bl_map = %d, oplus_display_brightness = %d\n", __func__, level, bl_map, oplus_display_brightness);

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
    int blank;
    int mode;
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

    AOD_off_setting[1].para_list[1] = cabc_mode_backup;

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

    mode = get_boot_mode();
    pr_info("[TP] in dis_panel_power_on,mode = %d\n", mode);
    if ((mode != MSM_BOOT_MODE__FACTORY) &&(mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
        blank = LCD_CTL_CS_ON;
        mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
        pr_err("[TP]TP CS will chang to spi mode and high\n");
        usleep_range(5000, 5100);
        blank = LCD_CTL_TP_LOAD_FW;
        mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
        pr_info("[TP] start to load fw!\n");
        blank = LCD_CTL_AOD_OFF;
        mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
        pr_info("[TP] EXIT AOD success!\n");
    }
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
    int ret = 0;
    int blank = 0;

    if (ctx->prepared)
        return 0;

    pr_info("%s: hx_td4376b lcm ctx->prepared %d\n", __func__, ctx->prepared);

    lcm_i2c_write_bytes(0x0, 0xf);
    lcm_i2c_write_bytes(0x1, 0xf);

    ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
    gpiod_set_value(ctx->bias_pos, 1);
    devm_gpiod_put(ctx->dev, ctx->bias_pos);

    usleep_range(5000, 5010);
    ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
    gpiod_set_value(ctx->bias_neg, 1);
    devm_gpiod_put(ctx->dev, ctx->bias_neg);

    usleep_range(3000, 3100);

    blank = LCD_CTL_IRQ_OFF;
    mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
    usleep_range(5000, 5100);
    blank = LCD_CTL_RST_ON;
    mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
    usleep_range(5000, 5100);

    ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
    gpiod_set_value(ctx->reset_gpio, 0);
    usleep_range(3100, 3101);
    gpiod_set_value(ctx->reset_gpio, 1);
    devm_gpiod_put(ctx->dev, ctx->reset_gpio);
    usleep_range(22000, 22100);

    ret = ctx->error;
    if (ret < 0)
        lcm_unprepare(panel);

    blank = LCD_CTL_IRQ_ON;
    mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
    pr_err("[TP]TP LCD_CTL_IRQ_ON will chang to spi mode and high\n");
    usleep_range(5000, 5100);
    blank = LCD_CTL_TP_LOAD_FW;
    mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
    pr_info("%s:Successful\n", __func__);
    return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
    struct lcm *ctx = panel_to_lcm(panel);
    int ret = 0;
    int flag_poweroff = 1;
    int mode;

    if (ctx->prepared)
        return 0;

    pr_info("%s:hx_td4376b lcm ctx->prepared %d\n", __func__, ctx->prepared);

    mode = get_boot_mode();
	if ((mode != MSM_BOOT_MODE__FACTORY) && (mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
        if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0) && (g_shutdown == 0)) {
            flag_poweroff = 0;
            pr_err("[TP] tp gesture is enable,Display not to poweroff\n");
        } else {
            flag_poweroff = 1;
            pr_err("[TP] set poweroff to 1\n");
        }
    }

    if(flag_poweroff == 1) {
        if (g_shutdown == 1) {
            pr_err("[TP] g_shutdown ==1, Display goto power off , And TP reset will low\n");
        }

    ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
    gpiod_set_value(ctx->reset_gpio, 0);
    devm_gpiod_put(ctx->dev, ctx->reset_gpio);
    usleep_range(2000, 2001);

    ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
    gpiod_set_value(ctx->bias_neg, 0);
    devm_gpiod_put(ctx->dev, ctx->bias_neg);

    usleep_range(2000, 2001);
    ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
    gpiod_set_value(ctx->bias_pos, 0);
    devm_gpiod_put(ctx->dev, ctx->bias_pos);
    usleep_range(5000, 5100);
    }
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
    } else if (m_vrefresh == 120) {
        ext->params = &ext_params_120Hz;
        current_fps = 120;
    } else if (m_vrefresh == 144) {
        ext->params = &ext_params_144Hz;
        current_fps = 144;
    } else {
        ret = 1;
    }

    return ret;
}

static void cabc_mode_switch(void *dsi, dcs_write_gce cb, void *handle, unsigned int cabc_mode)
{
    char bl_tb0[] = {0x53, 0x2c};
    char bl_tb1[] = {0x55, 0x00};

    pr_err("%s cabc_mode = %d\n", __func__, cabc_mode);
    if (cabc_mode > 3) {
        pr_err("%s: Invaild params skiped!\n", __func__);
        return;
    }

    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
    if (cabc_mode == 1) {
        bl_tb1[1] = 1;
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    } else if (cabc_mode == 2) {
        bl_tb1[1] = 2;
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    } else if (cabc_mode == 3) {
        bl_tb1[1] = 3;
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    } else if (cabc_mode == 0) {
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    }
    cabc_mode_backup = cabc_mode;
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
    .doze_disable = panel_doze_disable,
    .doze_enable = panel_doze_enable,
    .set_aod_light_mode = panel_set_aod_light_mode,
    .cabc_switch = cabc_mode_switch,
    .esd_backlight_recovery = oplus_esd_backlight_recovery,

    .hbm_get_state = panel_hbm_get_state,
    .hbm_set_state = panel_hbm_set_state,
    .hbm_get_wait_state = panel_hbm_get_wait_state,
    .hbm_set_wait_state = panel_hbm_set_wait_state,
};

static int lcm_get_modes(struct drm_panel *panel,
                    struct drm_connector *connector)
{
    struct drm_display_mode *mode[5];

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

    mode[3] = drm_mode_duplicate(connector->dev, &disp_mode_144Hz);
    if (!mode[3]) {
        pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_144Hz.hdisplay, disp_mode_144Hz.vdisplay, drm_mode_vrefresh(&disp_mode_144Hz));
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

    pr_info("[LCM] hx td4376b %s START\n", __func__);


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
    dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;

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
    ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->bias_pos)) {
        dev_err(dev, "%s: cannot get bias_pos 0 %ld\n",
            __func__, PTR_ERR(ctx->bias_pos));
        return PTR_ERR(ctx->bias_pos);
    }
    devm_gpiod_put(dev, ctx->bias_pos);

    usleep_range(5000, 5100);
    ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->bias_neg)) {
        dev_err(dev, "%s: cannot get bias_neg 1 %ld\n",
            __func__, PTR_ERR(ctx->bias_neg));
        return PTR_ERR(ctx->bias_neg);
    }
    devm_gpiod_put(dev, ctx->bias_neg);

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
    register_device_proc("lcd", "HX_TD4376B", "HX_SUZUKI");
    ctx->hbm_en = false;
    oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
    oplus_enhance_mipi_strength = 4;
    init_global_exp_backlight();

    pr_info("[LCM] %s- lcm, hx td4376b, END\n", __func__);


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
    { .compatible = "oplus25680,td4376b,fhdp,hx,dsi,vdo", },
    { }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
    .probe = lcm_probe,
    .remove = lcm_remove,
    .driver = {
        .name = "oplus25680_td4376b_fhdp_hx_dsi_vdo",
        .owner = THIS_MODULE,
        .of_match_table = lcm_of_match,
    },
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
