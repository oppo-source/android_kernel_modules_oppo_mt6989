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

#include "ae174_p_1_a0037_cmd_panel.h"
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "../oplus/oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
#include "../mediatek/mediatek_v2/mtk_dsi.h"

struct lcm {
    struct device *dev;
    struct drm_panel panel;
    struct backlight_device *backlight;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *vddr1p2_enable_gpio;
    struct gpio_desc *vdd_vss_eswire_enable_gpio;
    struct regulator *ldo_vci3p0;
    struct drm_display_mode *m;
    bool prepared;
    bool enabled;
    int error;
};

extern unsigned int last_backlight;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern int oplus_display_panel_dbv_probe(struct device *dev);
extern void lcdinfo_notify(unsigned long val, void *v);
static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle);
extern char regs1[AC178_GAMMA_COMPENSATION_READ_LENGTH];
extern char regs2[AC178_GAMMA_COMPENSATION_READ_LENGTH];
extern char regs3[AC178_GAMMA_COMPENSATION_READ_LENGTH];
extern char regs4[AE174_ELVSS_READ_LENGTH];
extern atomic_t esd_pending;
extern unsigned int m_db;

#define MAX_NORMAL_BRIGHTNESS   3515
#define BRIGHTNESS_MAX          3805
#define LCM_BRIGHTNESS_TYPE 2

#define FRAME_WIDTH             (1272)
#define FRAME_HEIGHT            (2772)
#define FRAME_WIDTH_VIR         (1080)
#define FRAME_HEIGHT_VIR        (2354)
#define HFP                     (132)
#define HBP                     (20)
#define HSA                     (4)
//#define VFP_60HZ                (2904)
//#define VFP_90HZ                (1004)
//#define VFP_120HZ               (56)
//#define VFP_144HZ               (72)
#define VFP                     (56)
#define VBP                     (20)
#define VSA                     (2)
#define DSI_PLL_CLK             (576)
#define DSI_DATA_RATE           (1152)
#define DSI_PLL_CLK_DYN         (566)
#define DSI_DATA_RATE_DYN       (1132)

#define PHYSICAL_WIDTH          (71)
#define PHYSICAL_HEIGHT         (156)

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

static unsigned int last_fps_mode = 120;
static unsigned int nt37708s_cmd_dphy_buf_thresh[14] ={896, 1792, 2688, 3584, 4480,
                                                       5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
static unsigned int nt37708s_cmd_dphy_range_min_qp[15] ={0, 4, 5, 5, 7, 7, 7, 7, 7,
                                                         7, 9, 9, 9, 13, 16};
static unsigned int nt37708s_cmd_dphy_range_max_qp[15] ={8, 8, 9, 10, 11, 11, 11,
                                                         12, 13, 14, 14, 15, 15, 16, 17};
static int nt37708s_cmd_dphy_range_bpg_ofs[15] ={2, 0, 0, -2, -4, -6, -8, -8, -8,
                                                 -10, -10, -12, -12, -12, -12};

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
    return container_of(panel, struct lcm, panel);
}

static enum PANEL_ES inline get_panel_es_ver(void)
{
    enum PANEL_ES panel_es_ver = ES_DVT;
    switch (m_db) {
        case 1:
            panel_es_ver = ES_T0;
            break;
        case 2:
            panel_es_ver = ES_EVT;
            break;
        case 3:
            panel_es_ver = ES_DVT;
            break;
        case 4:
            panel_es_ver = ES_PVT;
            break;
        case 6:
            panel_es_ver = ES_MP;
            break;
        default:
            panel_es_ver = ES_DVT;
    }
    return panel_es_ver;
}

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle)
{
    unsigned int i = 0;
    struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

    if (lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
        pr_err("out of mtk_ddic_dsi_cmd \n");
        return 0;
    }

    for (i = 0; i < lcm_cmd_count; i++) {
        send_cmd_to_ddic.mtk_ddic_cmd_table[i].cmd_num = table[i].count;
        send_cmd_to_ddic.mtk_ddic_cmd_table[i].para_list = table[i].para_list;
    }
    send_cmd_to_ddic.is_hs = 1;
    send_cmd_to_ddic.is_package = 1;
    send_cmd_to_ddic.cmd_count = lcm_cmd_count;
    cb(dsi, handle, &send_cmd_to_ddic);

    return 0;
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

static void push_table(struct lcm *ctx, struct LCM_setting_table *table,
                       unsigned int count)
{
    unsigned int i;
    unsigned int cmd;

    for (i = 0; i < count; i++) {
        cmd = table[i].cmd;
        switch (cmd) {
            case REGFLAG_DELAY:
                usleep_range(table[i].count * 1000, table[i].count * 1000 + 100);
                break;
            case REGFLAG_UDELAY:
                usleep_range(table[i].count, table[i].count + 1000);
                break;
            case REGFLAG_END_OF_TABLE:
                break;
            default:
                lcm_dcs_write(ctx, table[i].para_list,
                              table[i].count);
                break;
        }
    }
}

static void lcm_panel_init(struct lcm *ctx)
{
    enum PANEL_ES panel_es_ver = get_panel_es_ver();

    pr_info("sdc_nt37708s %s +, last_fps_mode=%d panel_es_ver = %d\n", __func__, last_fps_mode, panel_es_ver);
    if (panel_es_ver == ES_T0) {
        push_table(ctx, init_T0, sizeof(init_T0)/sizeof(struct LCM_setting_table));
    } else if (panel_es_ver == ES_EVT) {
        push_table(ctx, init_EVT, sizeof(init_EVT)/sizeof(struct LCM_setting_table));
    } else if (panel_es_ver == ES_DVT) {
        push_table(ctx, init_DVT, sizeof(init_DVT)/sizeof(struct LCM_setting_table));
    }
    switch (last_fps_mode) {
        case FHD_SDC60:
            if (panel_es_ver == ES_T0) {
                push_table(ctx, mode_switch_to_60, sizeof(mode_switch_to_60) / sizeof(struct LCM_setting_table));
            } else if (panel_es_ver == ES_EVT) {
                push_table(ctx, mode_switch_to_60_EVT, sizeof(mode_switch_to_60_EVT) / sizeof(struct LCM_setting_table));
            } else if (panel_es_ver == ES_DVT) {
                push_table(ctx, mode_switch_to_60_DVT, sizeof(mode_switch_to_60_DVT) / sizeof(struct LCM_setting_table));
            }
            break;
        case FHD_SDC90:
            if (panel_es_ver == ES_DVT) {
                push_table(ctx, mode_switch_to_90_DVT,
                           sizeof(mode_switch_to_90_DVT) / sizeof(struct LCM_setting_table));
            } else {
                push_table(ctx, mode_switch_to_90,
                           sizeof(mode_switch_to_90) / sizeof(struct LCM_setting_table));
            }
            break;
        case FHD_SDC120:
            if (panel_es_ver == ES_DVT) {
                push_table(ctx, mode_switch_to_120_DVT,
                           sizeof(mode_switch_to_120_DVT) / sizeof(struct LCM_setting_table));
            } else {
                push_table(ctx, mode_switch_to_120,
                           sizeof(mode_switch_to_120) / sizeof(struct LCM_setting_table));
            }
            break;
        case FHD_SDC144:
            if (panel_es_ver == ES_DVT) {
                push_table(ctx, mode_switch_to_144_DVT,
                           sizeof(mode_switch_to_144_DVT) / sizeof(struct LCM_setting_table));
            } else {
                push_table(ctx, mode_switch_to_144,
                           sizeof(mode_switch_to_144) / sizeof(struct LCM_setting_table));
            }
            break;
        default:
            push_table(ctx, mode_switch_to_120_DVT,
                       sizeof(mode_switch_to_120_DVT) / sizeof(struct LCM_setting_table));
    }
    pr_info("sdc_nt37708s %s -\n", __func__);
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
    pr_info("sdc_nt37708s %s:prepared=%d\n", __func__, ctx->prepared);

    if (!ctx->prepared)
        return 0;

    lcm_dcs_write_seq_static(ctx, 0x28);
    usleep_range(10000, 11000);
    lcm_dcs_write_seq_static(ctx, 0x10);
    usleep_range(150*1000, 151*1000);

    ctx->error = 0;
    ctx->prepared = false;
    pr_info("sdc_nt37708s %s:success\n", __func__);

    return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
    struct lcm *ctx = panel_to_lcm(panel);
    int ret;

    pr_info("sdc_nt37708s %s:prepared=%d\n", __func__, ctx->prepared);
    if (ctx->prepared)
        return 0;

    lcm_panel_init(ctx);

    ret = ctx->error;
    if (ret < 0)
        lcm_unprepare(panel);

    ctx->prepared = true;
    pr_info("sdc_nt37708s %s:success\n", __func__);
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

static const struct drm_display_mode display_mode[MODE_NUM * RES_NUM] = {
        //60HZ
        {
                .clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP + VBP + VSA) * 60) / 1000,
                .hdisplay = FRAME_WIDTH,
                .hsync_start = FRAME_WIDTH + HFP,
                .hsync_end = FRAME_WIDTH + HFP + HSA,
                .htotal = FRAME_WIDTH + HFP + HSA + HBP,
                .vdisplay = FRAME_HEIGHT,
                .vsync_start = FRAME_HEIGHT + VFP,
                .vsync_end = FRAME_HEIGHT + VFP + VSA,
                .vtotal = FRAME_HEIGHT + VFP + VSA + VBP,

        },
        //90HZ
        {
                .clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP + VBP + VSA) * 90) / 1000,
                .hdisplay = FRAME_WIDTH,
                .hsync_start = FRAME_WIDTH + HFP,
                .hsync_end = FRAME_WIDTH + HFP + HSA,
                .htotal = FRAME_WIDTH + HFP + HSA + HBP,
                .vdisplay = FRAME_HEIGHT,
                .vsync_start = FRAME_HEIGHT + VFP,
                .vsync_end = FRAME_HEIGHT + VFP + VSA,
                .vtotal = FRAME_HEIGHT + VFP + VSA + VBP,
        },
        //120HZ
        {
                .clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP + VBP + VSA) * 120) / 1000,
                .hdisplay = FRAME_WIDTH,
                .hsync_start = FRAME_WIDTH + HFP,
                .hsync_end = FRAME_WIDTH + HFP + HSA,
                .htotal = FRAME_WIDTH + HFP + HSA + HBP,
                .vdisplay = FRAME_HEIGHT,
                .vsync_start = FRAME_HEIGHT + VFP,
                .vsync_end = FRAME_HEIGHT + VFP + VSA,
                .vtotal = FRAME_HEIGHT + VFP + VSA + VBP,
        },
        //140HZ
        {
                .clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP + VBP + VSA) * 144) / 1000,
                .hdisplay = FRAME_WIDTH,
                .hsync_start = FRAME_WIDTH + HFP,
                .hsync_end = FRAME_WIDTH + HFP + HSA,
                .htotal = FRAME_WIDTH + HFP + HSA + HBP,
                .vdisplay = FRAME_HEIGHT,
                .vsync_start = FRAME_HEIGHT + VFP,
                .vsync_end = FRAME_HEIGHT + VFP + VSA,
                .vtotal = FRAME_HEIGHT + VFP + VSA + VBP,
        },
        //60HZ_VIR
        {
                .clock = ((FRAME_WIDTH_VIR + HFP + HBP + HSA) * (FRAME_HEIGHT_VIR + VFP + VBP + VSA) * 60) / 1000,
                .hdisplay = FRAME_WIDTH_VIR,
                .hsync_start = FRAME_WIDTH_VIR + HFP,
                .hsync_end = FRAME_WIDTH_VIR + HFP + HSA,
                .htotal = FRAME_WIDTH_VIR + HFP + HSA + HBP,
                .vdisplay = FRAME_HEIGHT_VIR,
                .vsync_start = FRAME_HEIGHT_VIR + VFP,
                .vsync_end = FRAME_HEIGHT_VIR + VFP + VSA,
                .vtotal = FRAME_HEIGHT_VIR + VFP + VSA + VBP,
        },
        //90HZ_VIR
        {
                .clock = ((FRAME_WIDTH_VIR + HFP + HBP + HSA) * (FRAME_HEIGHT_VIR + VFP + VBP + VSA) * 90) / 1000,
                .hdisplay = FRAME_WIDTH_VIR,
                .hsync_start = FRAME_WIDTH_VIR + HFP,
                .hsync_end = FRAME_WIDTH_VIR + HFP + HSA,
                .htotal = FRAME_WIDTH_VIR + HFP + HSA + HBP,
                .vdisplay = FRAME_HEIGHT_VIR,
                .vsync_start = FRAME_HEIGHT_VIR + VFP,
                .vsync_end = FRAME_HEIGHT_VIR + VFP + VSA,
                .vtotal = FRAME_HEIGHT_VIR + VFP + VSA + VBP,
        },
        //120HZ_VIR
        {
                .clock = ((FRAME_WIDTH_VIR + HFP + HBP + HSA) * (FRAME_HEIGHT_VIR + VFP + VBP + VSA) * 120) / 1000,
                .hdisplay = FRAME_WIDTH_VIR,
                .hsync_start = FRAME_WIDTH_VIR + HFP,
                .hsync_end = FRAME_WIDTH_VIR + HFP + HSA,
                .htotal = FRAME_WIDTH_VIR + HFP + HSA + HBP,
                .vdisplay = FRAME_HEIGHT_VIR,
                .vsync_start = FRAME_HEIGHT_VIR + VFP,
                .vsync_end = FRAME_HEIGHT_VIR + VFP + VSA,
                .vtotal = FRAME_HEIGHT_VIR + VFP + VSA + VBP,
        },
        //144HZ_VIR
        {
                .clock = ((FRAME_WIDTH_VIR + HFP + HBP + HSA) * (FRAME_HEIGHT_VIR + VFP + VBP + VSA) * 144) / 1000,
                .hdisplay = FRAME_WIDTH_VIR,
                .hsync_start = FRAME_WIDTH_VIR + HFP,
                .hsync_end = FRAME_WIDTH_VIR + HFP + HSA,
                .htotal = FRAME_WIDTH_VIR + HFP + HSA + HBP,
                .vdisplay = FRAME_HEIGHT_VIR,
                .vsync_start = FRAME_HEIGHT_VIR + VFP,
                .vsync_end = FRAME_HEIGHT_VIR + VFP + VSA,
                .vtotal = FRAME_HEIGHT_VIR + VFP + VSA + VBP,
        },
};

static struct mtk_panel_params ext_params_60Hz = {
        .pll_clk = DSI_PLL_CLK,
        .data_rate = DSI_DATA_RATE,

        .cust_esd_check = 1,
        .esd_check_enable = 1,
        .lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
        },

        .vendor = "A0037",
        .manufacture = "P_1",

        .phy_timcon = {
                .lpx = 10,
        },

        .lane_swap_en = 0,
        .lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
        .lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
        .lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
        .lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
        .lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
        .lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
        .lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
        .lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
        .lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
        .lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
        .lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
        .lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,

        .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
        .oplus_display_global_dre = 1,

        .dsc_params = {
                .enable = 1,
                .ver = 18,
                .slice_mode = 1,
                .rgb_swap = 0,
                .dsc_cfg = 40,
                .rct_on = 1,
                .bit_per_channel = 10,
                .dsc_line_buf_depth = 11,
                .bp_enable = 1,
                .bit_per_pixel = 128,
                .pic_height = 2772,
                .pic_width = 1272,
                .slice_height = 12,
                .slice_width = 636,
                .chunk_size = 636,
                .xmit_delay = 512,
                .dec_delay = 574,
                .scale_value = 32,
                .increment_interval = 311,
                .decrement_interval = 8,
                .line_bpg_offset = 12,
                .nfl_bpg_offset = 2235,
                .slice_bpg_offset = 1842,
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
                        .rc_buf_thresh = nt37708s_cmd_dphy_buf_thresh,
                        .range_min_qp = nt37708s_cmd_dphy_range_min_qp,
                        .range_max_qp = nt37708s_cmd_dphy_range_max_qp,
                        .range_bpg_ofs = nt37708s_cmd_dphy_range_bpg_ofs,
                },
        },

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
        .oplus_ofp_need_keep_apart_backlight = false,
		.oplus_ofp_hbm_on_delay = 0,
		.oplus_ofp_pre_hbm_off_delay = 2,
		.oplus_ofp_hbm_off_delay = 0,
		.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
		.oplus_ofp_aod_off_insert_black = 1,
		.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

        .dyn_fps = {
                .switch_en = 1,
                .vact_timing_fps = 60,
                .apollo_limit_superior_us = 10540,
                .apollo_limit_inferior_us = 14200,
                .apollo_transfer_time_us = 8200,
        },
        .dyn = {
                .switch_en = 1,
                .pll_clk = DSI_PLL_CLK_DYN,
                .data_rate = DSI_DATA_RATE_DYN,
        },
        .panel_bpp = 10,
        .vdo_mix_mode_en = false,
        .before_power_down = true,
};

static struct mtk_panel_params ext_params_90Hz = {
        .pll_clk = DSI_PLL_CLK,
        .data_rate = DSI_DATA_RATE,

        .cust_esd_check = 1,
        .esd_check_enable = 1,
        .lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
        },

        .vendor = "A0037",
        .manufacture = "P_1",

        .phy_timcon = {
                .lpx = 10,
        },

        .lane_swap_en = 0,
        .lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
        .lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
        .lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
        .lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
        .lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
        .lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
        .lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
        .lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
        .lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
        .lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
        .lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
        .lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,

        .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
        .oplus_display_global_dre = 1,

        .dsc_params = {
                .enable = 1,
                .ver = 18,
                .slice_mode = 1,
                .rgb_swap = 0,
                .dsc_cfg = 40,
                .rct_on = 1,
                .bit_per_channel = 10,
                .dsc_line_buf_depth = 11,
                .bp_enable = 1,
                .bit_per_pixel = 128,
                .pic_height = 2772,
                .pic_width = 1272,
                .slice_height = 12,
                .slice_width = 636,
                .chunk_size = 636,
                .xmit_delay = 512,
                .dec_delay = 574,
                .scale_value = 32,
                .increment_interval = 311,
                .decrement_interval = 8,
                .line_bpg_offset = 12,
                .nfl_bpg_offset = 2235,
                .slice_bpg_offset = 1842,
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
                        .rc_buf_thresh = nt37708s_cmd_dphy_buf_thresh,
                        .range_min_qp = nt37708s_cmd_dphy_range_min_qp,
                        .range_max_qp = nt37708s_cmd_dphy_range_max_qp,
                        .range_bpg_ofs = nt37708s_cmd_dphy_range_bpg_ofs,
                },
        },

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
        .oplus_ofp_need_keep_apart_backlight = false,
		.oplus_ofp_hbm_on_delay = 0,
		.oplus_ofp_pre_hbm_off_delay = 2,
		.oplus_ofp_hbm_off_delay = 0,
		.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
		.oplus_ofp_aod_off_insert_black = 1,
		.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

        .dyn_fps = {
                .switch_en = 1,
                .vact_timing_fps = 90,
                .apollo_limit_superior_us = 7880,
                .apollo_limit_inferior_us = 10000,
                .apollo_transfer_time_us = 8800,
        },
        .dyn = {
                .switch_en = 1,
                .pll_clk = DSI_PLL_CLK_DYN,
                .data_rate = DSI_DATA_RATE_DYN,
        },
        .panel_bpp = 10,
        .vdo_mix_mode_en = false,
        .before_power_down = true,
};

static struct mtk_panel_params ext_params_120Hz = {
        .pll_clk = DSI_PLL_CLK,
        .data_rate = DSI_DATA_RATE,

        .cust_esd_check = 1,
        .esd_check_enable = 1,
        .lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
        },

        .vendor = "A0037",
        .manufacture = "P_1",

        .phy_timcon = {
                .lpx = 10,
        },

        .lane_swap_en = 0,
        .lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
        .lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
        .lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
        .lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
        .lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
        .lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
        .lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
        .lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
        .lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
        .lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
        .lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
        .lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,

        .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
        .oplus_display_global_dre = 1,

        .dsc_params = {
                .enable = 1,
                .ver = 18,
                .slice_mode = 1,
                .rgb_swap = 0,
                .dsc_cfg = 40,
                .rct_on = 1,
                .bit_per_channel = 10,
                .dsc_line_buf_depth = 11,
                .bp_enable = 1,
                .bit_per_pixel = 128,
                .pic_height = 2772,
                .pic_width = 1272,
                .slice_height = 12,
                .slice_width = 636,
                .chunk_size = 636,
                .xmit_delay = 512,
                .dec_delay = 574,
                .scale_value = 32,
                .increment_interval = 311,
                .decrement_interval = 8,
                .line_bpg_offset = 12,
                .nfl_bpg_offset = 2235,
                .slice_bpg_offset = 1842,
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
                        .rc_buf_thresh = nt37708s_cmd_dphy_buf_thresh,
                        .range_min_qp = nt37708s_cmd_dphy_range_min_qp,
                        .range_max_qp = nt37708s_cmd_dphy_range_max_qp,
                        .range_bpg_ofs = nt37708s_cmd_dphy_range_bpg_ofs,
                },
        },

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
        .oplus_ofp_need_keep_apart_backlight = false,
		.oplus_ofp_hbm_on_delay = 0,
		.oplus_ofp_pre_hbm_off_delay = 2,
		.oplus_ofp_hbm_off_delay = 0,
		.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
		.oplus_ofp_aod_off_insert_black = 1,
		.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

        .dyn_fps = {
                .switch_en = 1,
                .vact_timing_fps = 120,
                .apollo_limit_superior_us = 4900,
                .apollo_limit_inferior_us = 7700,
                .apollo_transfer_time_us = 6200,
        },
        .dyn = {
                .switch_en = 1,
                .pll_clk = DSI_PLL_CLK_DYN,
                .data_rate = DSI_DATA_RATE_DYN,
        },
        .panel_bpp = 10,
        .vdo_mix_mode_en = false,
        .before_power_down = true,
};

static struct mtk_panel_params ext_params_144Hz = {
        .pll_clk = DSI_PLL_CLK,
        .data_rate = DSI_DATA_RATE,

        .cust_esd_check = 1,
        .esd_check_enable = 1,
        .lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
        },

        .vendor = "A0037",
        .manufacture = "P_1",

        .phy_timcon = {
                .lpx = 10,
        },

        .lane_swap_en = 0,
        .lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
        .lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
        .lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
        .lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
        .lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
        .lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
        .lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
        .lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
        .lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
        .lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
        .lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
        .lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,

        .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
        .oplus_display_global_dre = 1,

        .dsc_params = {
                .enable = 1,
                .ver = 18,
                .slice_mode = 1,
                .rgb_swap = 0,
                .dsc_cfg = 40,
                .rct_on = 1,
                .bit_per_channel = 10,
                .dsc_line_buf_depth = 11,
                .bp_enable = 1,
                .bit_per_pixel = 128,
                .pic_height = 2772,
                .pic_width = 1272,
                .slice_height = 12,
                .slice_width = 636,
                .chunk_size = 636,
                .xmit_delay = 512,
                .dec_delay = 574,
                .scale_value = 32,
                .increment_interval = 311,
                .decrement_interval = 8,
                .line_bpg_offset = 12,
                .nfl_bpg_offset = 2235,
                .slice_bpg_offset = 1842,
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
                        .rc_buf_thresh = nt37708s_cmd_dphy_buf_thresh,
                        .range_min_qp = nt37708s_cmd_dphy_range_min_qp,
                        .range_max_qp = nt37708s_cmd_dphy_range_max_qp,
                        .range_bpg_ofs = nt37708s_cmd_dphy_range_bpg_ofs,
                },
        },

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
        .oplus_ofp_need_keep_apart_backlight = false,
		.oplus_ofp_hbm_on_delay = 0,
		.oplus_ofp_pre_hbm_off_delay = 2,
		.oplus_ofp_hbm_off_delay = 0,
		.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
		.oplus_ofp_aod_off_insert_black = 1,
		.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

        .dyn_fps = {
                .switch_en = 1,
                .vact_timing_fps = 144,
                .apollo_limit_superior_us = 4900,
                .apollo_limit_inferior_us = 7700,
                .apollo_transfer_time_us = 6200,
        },
        .dyn = {
                .switch_en = 1,
                .pll_clk = DSI_PLL_CLK_DYN,
                .data_rate = DSI_DATA_RATE_DYN,
        },
        .panel_bpp = 10,
        .vdo_mix_mode_en = false,
        .before_power_down = true,
};

static int panel_ata_check(struct drm_panel *panel)
{
    /* Customer test by own ATA tool */
    return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
    unsigned int mapped_level = 0;
    unsigned char bl_level[] = {0x51, 0x03, 0xFF};

    if (!dsi || !cb) {
        return -EINVAL;
    }

    if (level == 0) {
        pr_info("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
    }

    if (level == 1) {
        pr_info("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
        return 0;
    } else if (level > BRIGHTNESS_MAX) {
        level = BRIGHTNESS_MAX;
    }

    if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
        level = 2047;
    }

    mapped_level = level;
    if (mapped_level > 1) {
        lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
    }

    bl_level[1] = mapped_level >> 8;
    bl_level[2] = mapped_level & 0xFF;
    cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
    pr_info("sdc_nt37708s bl_level[1]=%x, bl_level[2]=%x, backlight = %d\n", bl_level[1], bl_level[2], mapped_level);
    oplus_display_brightness = mapped_level;
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
    oplus_ofp_lhbm_setbacklight(dsi);
#endif
    return 0;
}

static int oplus_display_panel_set_hbm_max(void *dsi, dcs_write_gce_pack cb, dcs_write_gce cb2, void *handle, unsigned int en)
{
    unsigned int lcm_cmd_count = 0;
    unsigned int level = oplus_display_brightness;
    static char r_reg1 = 0;
    static char r_reg2 = 0;
    static char g_reg1 = 0;
    static char g_reg2 = 0;
    static char b_reg1 = 0;
    static char b_reg2 = 0;
    static char elvss_reg = 0;

    if (!dsi || !cb || !cb2) {
        pr_err("Invalid params\n");
        return -EINVAL;
    }

    r_reg1 = ((((regs1[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
                | regs1[AC178_GAMMA_COMPENSATION_REG_INDEX2])) >> 8) & 0xFF;
    r_reg2 = (((regs1[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
               | regs1[AC178_GAMMA_COMPENSATION_REG_INDEX2])) & 0xFF;
    g_reg1 = ((((regs2[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
                | regs2[AC178_GAMMA_COMPENSATION_REG_INDEX2])) >> 8) & 0xFF;
    g_reg2 = (((regs2[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
               | regs2[AC178_GAMMA_COMPENSATION_REG_INDEX2])) & 0xFF;
    b_reg1 = ((((regs3[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
                | regs3[AC178_GAMMA_COMPENSATION_REG_INDEX2])) >> 8) & 0xFF;
    b_reg2 = (((regs3[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
               | regs3[AC178_GAMMA_COMPENSATION_REG_INDEX2])) & 0xFF;
    elvss_reg = regs4[AC178_ELVSS_REG_INDEX];
    pr_info("Enter hbm_max APL mode regs1=[%02X %02X], regs2=[%02X %02X], regs3=[%02X %02X] ,regs4=[%02X]\n",
            r_reg1, r_reg2, g_reg1, g_reg2, b_reg1, b_reg2, elvss_reg);

    if (en) {
        lcm_cmd_count = sizeof(dsi_switch_hbm_apl_on) / sizeof(struct LCM_setting_table);
        panel_send_pack_hs_cmd(dsi, dsi_switch_hbm_apl_on, lcm_cmd_count, cb, handle);
        pr_info("Enter hbm_max APL mode\n");
    } else if (!en) {
        lcm_cmd_count = sizeof(dsi_switch_hbm_apl_off) / sizeof(struct LCM_setting_table);
        dsi_switch_hbm_apl_off[0].para_list[1] = level >> 8;
        dsi_switch_hbm_apl_off[0].para_list[2] = level & 0xFF;
        dsi_switch_hbm_apl_off[6].para_list[1] = r_reg1;
        dsi_switch_hbm_apl_off[6].para_list[2] = r_reg2;
        dsi_switch_hbm_apl_off[8].para_list[1] = g_reg1;
        dsi_switch_hbm_apl_off[8].para_list[2] = g_reg2;
        dsi_switch_hbm_apl_off[10].para_list[1] = b_reg1;
        dsi_switch_hbm_apl_off[10].para_list[2] = b_reg2;
        dsi_switch_hbm_apl_off[19].para_list[1] = elvss_reg;
        panel_send_pack_hs_cmd(dsi, dsi_switch_hbm_apl_off, lcm_cmd_count, cb, handle);
        pr_info("hbm_max APL off, restore backlight:%d\n", level);
    }
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
    pr_info("sdc_nt37708s esd_bl_level[1]=%x, esd_bl_level[2]=%x\n", esd_bl_level[1], esd_bl_level[2]);
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
    oplus_ofp_lhbm_setbacklight(dsi);
#endif

    return 0;
}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
static void set_lhbm_pressed_icon(void *dsi_drv, dcs_write_gce_pack cb, void *handle, bool lhbm_pressed_icon_on)
{
    static char r_reg1 = 0;
    static char r_reg2 = 0;
    static char g_reg1 = 0;
    static char g_reg2 = 0;
    static char b_reg1 = 0;
    static char b_reg2 = 0;
    unsigned int r_fpr_ratio = 11234;
    unsigned int g_fpr_ratio = 11624;
    unsigned int b_fpr_ratio = 11394;
    unsigned int reg_count = 0;
    unsigned int level = oplus_display_brightness;
    enum PANEL_ES panel_es_ver = get_panel_es_ver();

    OFP_INFO("sdc_nt37708s lhbm_pressed_icon_on:%u,bl_lvl:%u,panel_es_ver = %d\n", lhbm_pressed_icon_on, level, panel_es_ver);

    if (lhbm_pressed_icon_on) {
        if (panel_es_ver == ES_EVT) {
            for (int i = 0; i < sizeof(lhbm_fpr_ratio_EVT) / sizeof(lhbm_fpr_ratio_EVT[0]); i++) {
                if (oplus_display_brightness >= lhbm_fpr_ratio_EVT[i].min &&
                    oplus_display_brightness < lhbm_fpr_ratio_EVT[i].max) {
                    r_fpr_ratio = lhbm_fpr_ratio_EVT[i].r_ratio;
                    g_fpr_ratio = lhbm_fpr_ratio_EVT[i].g_ratio;
                    b_fpr_ratio = lhbm_fpr_ratio_EVT[i].b_ratio;
                    break;
                }
            }
        } else {
            for (int i = 0; i < sizeof(lhbm_fpr_ratio_DVT) / sizeof(lhbm_fpr_ratio_DVT[0]); i++) {
                if (oplus_display_brightness >= lhbm_fpr_ratio_DVT[i].min &&
                    oplus_display_brightness < lhbm_fpr_ratio_DVT[i].max) {
                    r_fpr_ratio = lhbm_fpr_ratio_DVT[i].r_ratio;
                    g_fpr_ratio = lhbm_fpr_ratio_DVT[i].g_ratio;
                    b_fpr_ratio = lhbm_fpr_ratio_DVT[i].b_ratio;
                    break;
                }
            }
        }
        OFP_INFO("sdc_nt37708s r_fpr_ratio=%d, g_fpr_ratio=%d, b_fpr_ratio=%d\n",
                 r_fpr_ratio, g_fpr_ratio, b_fpr_ratio);
        r_reg1 = ((((regs1[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
                    | regs1[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * r_fpr_ratio * 4 / 10000U) >> 8) & 0xFF;
        r_reg2 = (((regs1[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
                   | regs1[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * r_fpr_ratio * 4  / 10000U) & 0xFF;
        g_reg1 = ((((regs2[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
                    | regs2[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * g_fpr_ratio * 4  / 10000U) >> 8) & 0xFF;
        g_reg2 = (((regs2[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
                   | regs2[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * g_fpr_ratio * 4  / 10000U) & 0xFF;
        b_reg1 = ((((regs3[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
                    | regs3[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * b_fpr_ratio * 4  / 10000U) >> 8) & 0xFF;
        b_reg2 = (((regs3[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
                   | regs3[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * b_fpr_ratio * 4  / 10000U) & 0xFF;
        OFP_INFO("sdc_nt37708s compensation regs1=[%02X %02X], regs2=[%02X %02X], regs3=[%02X %02X]\n",
                 r_reg1, r_reg2, g_reg1, g_reg2, b_reg1, b_reg2);

        lhbm_pressed_icon_on_cmd[4].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX1 + 1] = r_reg1;
        lhbm_pressed_icon_on_cmd[4].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX2 + 1] = r_reg2;
        lhbm_pressed_icon_on_cmd[4].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX3 + 1] = g_reg1;
        lhbm_pressed_icon_on_cmd[4].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX4 + 1] = g_reg2;
        lhbm_pressed_icon_on_cmd[4].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX5 + 1] = b_reg1;
        lhbm_pressed_icon_on_cmd[4].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX6 + 1] = b_reg2;

        reg_count = sizeof(lhbm_pressed_icon_on_cmd) / sizeof(struct LCM_setting_table);
        panel_send_pack_hs_cmd(dsi_drv, lhbm_pressed_icon_on_cmd, reg_count, cb, handle);
    } else {
        reg_count = sizeof(lhbm_pressed_icon_off_cmd) / sizeof(struct LCM_setting_table);
        panel_send_pack_hs_cmd(dsi_drv, lhbm_pressed_icon_off_cmd, reg_count, cb, handle);
    }
}

static int lcm_set_hbm(void *dsi, dcs_write_gce_pack cb,
                       void *handle, unsigned int hbm_mode)
{
    OFP_DEBUG("sdc_nt37708s start\n");

    if (!dsi || !cb) {
        OFP_ERR("Invalid params\n");
        return -EINVAL;
    }

    set_lhbm_pressed_icon(dsi, cb, handle, hbm_mode);

    OFP_DEBUG("sdc_nt37708s end\n");

    return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
                              dcs_write_gce_pack cb, void *handle, bool en)
{
    unsigned int level = oplus_display_brightness;
    unsigned int lcm_cmd_count = 0;

    if (!panel || !dsi || !cb) {
        pr_err("Invalid params\n");
        return -EINVAL;
    }

    pr_info("sdc_nt37708s panel_hbm_set_cmdq oplus_display_brightness=%d, hbm_mode=%u\n", oplus_display_brightness, en);

    if(en == 1) {
        if (last_backlight == BRIGHTNESS_MAX) {
            OFP_INFO("sdc_nt37708s Enter hbm mode, set last_backlight as %d", last_backlight);
        }
        lcm_cmd_count = sizeof(hbm_on_cmd) / sizeof(struct LCM_setting_table);
        panel_send_pack_hs_cmd(dsi, hbm_on_cmd, lcm_cmd_count, cb, handle);
        lcdinfo_notify(1, &en);
    } else if (en == 0) {
        hbm_off_cmd[1].para_list[1] = level >> 8;
        hbm_off_cmd[1].para_list[2] = level & 0xFF;
        lcm_cmd_count = sizeof(hbm_off_cmd) / sizeof(struct LCM_setting_table);
        panel_send_pack_hs_cmd(dsi, hbm_off_cmd, lcm_cmd_count, cb, handle);
        lcdinfo_notify(1, &en);
        pr_info("sdc_nt37708s level %x\n", level);
    }
    return 0;
}

static int oplus_ofp_set_lhbm_pressed_icon(struct drm_panel *panel, void *dsi_drv, dcs_write_gce_pack cb, void *handle, bool lhbm_pressed_icon_on)
{
    unsigned int vrefresh_rate = 0;
    struct lcm *ctx = NULL;

    OFP_DEBUG("sdc_nt37708s start\n");

    if (!oplus_ofp_local_hbm_is_enabled()) {
        OFP_DEBUG("sdc_nt37708s local hbm is not enabled, should not set lhbm pressed icon\n");
    }

    if (!panel || !dsi_drv || !cb) {
        OFP_ERR("sdc_nt37708s Invalid input params\n");
        return -EINVAL;
    }

    ctx = panel_to_lcm(panel);
    if (!ctx) {
        OFP_ERR("sdc_nt37708s Invalid ctx params\n");
    }

    if (!ctx->m) {
        vrefresh_rate = 120;
        OFP_INFO("sdc_nt37708s default refresh rate is 120hz\n");
    } else {
        vrefresh_rate = drm_mode_vrefresh(ctx->m);
    }

    set_lhbm_pressed_icon(dsi_drv, cb, handle, lhbm_pressed_icon_on);

    OFP_DEBUG("sdc_nt37708s end\n");

    return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
    unsigned int i = 0;
    unsigned int cmd;
    unsigned int reg_count = 0;
    struct mtk_dsi *mtk_dsi = dsi;
    struct drm_crtc *crtc = NULL;
    struct mtk_crtc_state *mtk_state = NULL;
    struct LCM_setting_table *aod_off_cmd_set = NULL;

    if (!panel || !mtk_dsi) {
        OFP_ERR("sdc_nt37708s Invalid mtk_dsi params\n");
    }

    crtc = mtk_dsi->encoder.crtc;

    if (!crtc || !crtc->state) {
        OFP_ERR("sdc_nt37708s Invalid crtc param\n");
        return -EINVAL;
    }

    mtk_state = to_mtk_crtc_state(crtc->state);
    if (!mtk_state) {
        OFP_ERR("sdc_nt37708s Invalid mtk_state param\n");
        return -EINVAL;
    }

    aod_off_cmd_set = aod_off_cmd;
    reg_count = sizeof(aod_off_cmd) / sizeof(struct LCM_setting_table);

    for (i = 0; i < reg_count; i++) {
        cmd = aod_off_cmd_set[i].cmd;

        switch (cmd) {
            case REGFLAG_DELAY:
                if (handle == NULL) {
                    usleep_range(aod_off_cmd_set[i].count * 1000, aod_off_cmd_set[i].count * 1000 + 100);
                } else {
                    cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd_set[i].count * 1000), CMDQ_GPR_R14);
                }
                break;
            case REGFLAG_UDELAY:
                if (handle == NULL) {
                    usleep_range(aod_off_cmd_set[i].count, aod_off_cmd_set[i].count + 100);
                } else {
                    cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd_set[i].count), CMDQ_GPR_R14);
                }
                break;
            case REGFLAG_END_OF_TABLE:
                break;
            default:
                cb(dsi, handle, aod_off_cmd_set[i].para_list, aod_off_cmd_set[i].count);
        }
    }
    if(!oplus_ofp_backlight_filter(crtc, handle, oplus_display_brightness))
        lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
    OFP_INFO("sdc_nt37708s send aod off cmd\n");
    atomic_set(&esd_pending, 0);

    return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
    unsigned int i = 0;
    struct mtk_dsi *mtk_dsi = dsi;
    struct drm_crtc *crtc = NULL;
    struct mtk_crtc_state *mtk_state = NULL;

    if (!panel || !mtk_dsi) {
        OFP_ERR("sdc_nt37708s Invalid mtk_dsi params\n");
    }

    crtc = mtk_dsi->encoder.crtc;

    if (!crtc || !crtc->state) {
        OFP_ERR("sdc_nt37708s Invalid crtc param\n");
        return -EINVAL;
    }

    mtk_state = to_mtk_crtc_state(crtc->state);
    if (!mtk_state) {
        OFP_ERR("sdc_nt37708s Invalid mtk_state param\n");
        return -EINVAL;
    }

    OFP_INFO("sdc_nt37708s %s crtc_active:%d, doze_active:%llu\n", __func__, crtc->state->active, mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE]);
    atomic_set(&esd_pending, 1);
    for (i = 0; i < (sizeof(aod_on_cmd)/sizeof(struct LCM_setting_table)); i++) {
        unsigned int cmd;
        cmd = aod_on_cmd[i].cmd;

        switch (cmd) {
            case REGFLAG_DELAY:
                usleep_range(aod_on_cmd[i].count * 1000, aod_on_cmd[i].count * 1000 + 100);
                break;
            case REGFLAG_UDELAY:
                usleep_range(aod_on_cmd[i].count, aod_on_cmd[i].count + 100);
                break;
            case REGFLAG_END_OF_TABLE:
                break;
            default:
                cb(dsi, handle, aod_on_cmd[i].para_list, aod_on_cmd[i].count);
        }
    }
    OFP_INFO("sdc_nt37708s send aod on cmd\n");

    return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
    unsigned int i = 0;
    if (level == 0) {
        for (i = 0; i < sizeof(aod_high_bl_level)/sizeof(struct LCM_setting_table); i++) {
            cb(dsi, handle, aod_high_bl_level[i].para_list, aod_high_bl_level[i].count);
        }
    } else {
        for (i = 0; i < sizeof(aod_low_bl_level)/sizeof(struct LCM_setting_table); i++) {
            cb(dsi, handle, aod_low_bl_level[i].para_list, aod_low_bl_level[i].count);
        }
    }
    OFP_INFO("sdc_nt37708s level = %d\n", level);

    return 0;
}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

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

static unsigned int lcm_enable_ldo_vci3p0(struct lcm *ctx, int en)
{
    unsigned int ret = 0;

    pr_info("[lcd_info] sdc_nt37708s %s +\n", __func__);
    if(!ctx->ldo_vci3p0) {
        pr_err("%s error return -1\n", __func__);
        return -1;
    }

    if(en) {
        ret = regulator_set_voltage(ctx->ldo_vci3p0, 3000000, 3000000);
        ret = regulator_enable(ctx->ldo_vci3p0);
        pr_info("[lcd_info] sdc_nt37708s %s vddio enable\n", __func__);
    } else {
        ret = regulator_disable(ctx->ldo_vci3p0);
        pr_info("[lcd_info] sdc_nt37708s %s vddio disable\n", __func__);
    }
    pr_info("[lcd_info] sdc_nt37708s %s -\n", __func__);

    return ret;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
    struct lcm *ctx = panel_to_lcm(panel);
    int ret;
    enum PANEL_ES panel_es_ver = get_panel_es_ver();

    if (ctx->prepared)
        return 0;

    pr_info("sdc_nt37708s %s:lcm ctx->prepared %d panel_es_ver = %d\n", __func__, ctx->prepared, panel_es_ver);
    //enable vddr 1.2v
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
    if (panel_es_ver == ES_T0) {
        //enable vdd-vss-eswire
        ctx->vdd_vss_eswire_enable_gpio =
                devm_gpiod_get(ctx->dev, "vdd-vss-eswire", GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->vdd_vss_eswire_enable_gpio)) {
            dev_err(ctx->dev, "%s: cannot get vdd_vss_eswire_enable_gpio %ld\n",
                    __func__, PTR_ERR(ctx->vdd_vss_eswire_enable_gpio));
            return PTR_ERR(ctx->vdd_vss_eswire_enable_gpio);
        }
        gpiod_set_value(ctx->vdd_vss_eswire_enable_gpio, 1);
        devm_gpiod_put(ctx->dev, ctx->vdd_vss_eswire_enable_gpio);
        usleep_range(5000, 5100);
    }
    //enable vci 3.0v
    ret = lcm_enable_ldo_vci3p0(ctx, 1);
    if(ret) {
        pr_err("[lcd_info]%s: set vddio on failed! ret=%d\n", __func__, ret);
    }

    usleep_range(10000, 10010);

    ret = ctx->error;
    if (ret < 0)
        lcm_unprepare(panel);

    pr_info("sdc_nt37708s %s:Successful\n", __func__);
    return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
    struct lcm *ctx = panel_to_lcm(panel);
    int ret;
    enum PANEL_ES panel_es_ver = get_panel_es_ver();

    if (ctx->prepared)
        return 0;

    pr_info("sdc_nt37708s %s:lcm ctx->prepared %d panel_es_ver = %d\n", __func__, ctx->prepared, panel_es_ver);

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
    //disable vci 3.0V
    ret = lcm_enable_ldo_vci3p0(ctx, 0);
    if(ret) {
        pr_err("[lcd_info]%s: set vddio off failed! ret=%d\n", __func__, ret);
    }
    usleep_range(5000, 5100);
    if (panel_es_ver == ES_T0) {
        //disable vdd-vss-eswire
        ctx->vdd_vss_eswire_enable_gpio =
                devm_gpiod_get(ctx->dev, "vdd-vss-eswire", GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->vdd_vss_eswire_enable_gpio)) {
            dev_err(ctx->dev, "%s: cannot get vdd_vss_eswire_enable_gpio %ld\n",
                    __func__, PTR_ERR(ctx->vdd_vss_eswire_enable_gpio));
            return PTR_ERR(ctx->vdd_vss_eswire_enable_gpio);
        }
        gpiod_set_value(ctx->vdd_vss_eswire_enable_gpio, 0);
        devm_gpiod_put(ctx->dev, ctx->vdd_vss_eswire_enable_gpio);
        usleep_range(5000, 5100);
    }
    //disable vddr 1.0V
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
    pr_info("sdc_nt37708s %s:Successful\n", __func__);

    return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
    struct lcm *ctx = panel_to_lcm(panel);

    if (ctx->prepared)
        return 0;

    pr_info("[LCM]sdc_nt37708s debug for lcd reset :%s, ctx->prepared:%d\n", __func__, ctx->prepared);

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
    pr_info("sdc_nt37708s %s: mode=%d, vrefresh=%d\n", __func__, mode, drm_mode_vrefresh(m));

    if (m_vrefresh == 60) {
        ext->params = &ext_params_60Hz;
    } else if (m_vrefresh == 90) {
        ext->params = &ext_params_90Hz;
    } else if (m_vrefresh == 120) {
        ext->params = &ext_params_120Hz;
    } else if (m_vrefresh == 144) {
        ext->params = &ext_params_144Hz;
    } else {
        ret = 1;
    }

    return ret;
}

enum RES_SWITCH_TYPE mtk_get_res_switch_type(void)
{
    pr_info("sdc_nt37708s res_switch_type: %d\n", res_switch_type);
    return res_switch_type;
}

int mtk_scaling_mode_mapping(int mode_idx)
{
    return MODE_MAPPING_RULE(mode_idx);
}

static int mode_switch(struct drm_panel *panel,
                       struct drm_connector *connector, unsigned int cur_mode,
                       unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
    int ret = 0;
    struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
    struct lcm *ctx = panel_to_lcm(panel);
    enum PANEL_ES panel_es_ver = get_panel_es_ver();
    int target_vrefresh = 0;

    target_vrefresh = drm_mode_vrefresh(m);
    pr_info("sdc_nt37708s %s cur_mode = %d dst_mode = %d target_vrefresh = %d last_fps_mode = %d panel_es_ver =%d\n",
            __func__, cur_mode, dst_mode, target_vrefresh, last_fps_mode, panel_es_ver);
    if (cur_mode == dst_mode || target_vrefresh == last_fps_mode)
        return ret;

    switch (target_vrefresh) {
        case FHD_SDC60:
            if (panel_es_ver == ES_T0) {
                push_table(ctx, mode_switch_to_60, sizeof(mode_switch_to_60) / sizeof(struct LCM_setting_table));
            } else if (panel_es_ver == ES_EVT) {
                push_table(ctx, mode_switch_to_60_EVT, sizeof(mode_switch_to_60_EVT) / sizeof(struct LCM_setting_table));
            } else if (panel_es_ver == ES_DVT) {
                if(last_fps_mode == FHD_SDC90) {
                    push_table(ctx, mode_switch_90_to_60_DVT, sizeof(mode_switch_90_to_60_DVT) / sizeof(struct LCM_setting_table));
                } else if(last_fps_mode == FHD_SDC120) {
                    push_table(ctx, mode_switch_120_to_60_DVT, sizeof(mode_switch_120_to_60_DVT) / sizeof(struct LCM_setting_table));
                } else if(last_fps_mode == FHD_SDC144) {
                    push_table(ctx, mode_switch_144_to_60_DVT, sizeof(mode_switch_144_to_60_DVT) / sizeof(struct LCM_setting_table));
                }
            }
            break;
        case FHD_SDC90:
            if (panel_es_ver == ES_DVT) {
                push_table(ctx, mode_switch_to_90_DVT,
                           sizeof(mode_switch_to_90_DVT) / sizeof(struct LCM_setting_table));
            } else {
                push_table(ctx, mode_switch_to_90,
                           sizeof(mode_switch_to_90) / sizeof(struct LCM_setting_table));
            }
            break;
        case FHD_SDC120:
            if (panel_es_ver == ES_DVT) {
                push_table(ctx, mode_switch_to_120_DVT,
                           sizeof(mode_switch_to_120_DVT) / sizeof(struct LCM_setting_table));
            } else {
                push_table(ctx, mode_switch_to_120,
                           sizeof(mode_switch_to_120) / sizeof(struct LCM_setting_table));
            }
            break;
        case FHD_SDC144:
            if (panel_es_ver == ES_DVT) {
                push_table(ctx, mode_switch_to_144_DVT,
                           sizeof(mode_switch_to_144_DVT) / sizeof(struct LCM_setting_table));
            } else {
                push_table(ctx, mode_switch_to_144,
                           sizeof(mode_switch_to_144) / sizeof(struct LCM_setting_table));
            }
            break;
        default:
            push_table(ctx, mode_switch_to_120_DVT,
                       sizeof(mode_switch_to_120_DVT) / sizeof(struct LCM_setting_table));
    }

    pr_info("sdc_nt37708s %s timing switch to %d\n", __func__, target_vrefresh);

    ret = 1;
    last_fps_mode = target_vrefresh;
    ctx->m = m;
    return ret;
}

static struct mtk_panel_funcs ext_funcs = {
        .reset = panel_ext_reset,
        .set_backlight_cmdq = lcm_setbacklight_cmdq,
        .lcm_set_hbm_max = oplus_display_panel_set_hbm_max,
        .panel_poweron = lcm_panel_poweron,
        .panel_poweroff = lcm_panel_poweroff,
        .panel_reset = lcm_panel_reset,
        .ata_check = panel_ata_check,
        .ext_param_set = mtk_panel_ext_param_set,
        .get_res_switch_type = mtk_get_res_switch_type,
        .scaling_mode_mapping = mtk_scaling_mode_mapping,
        .mode_switch = mode_switch,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
        .oplus_set_hbm = lcm_set_hbm,
        .oplus_hbm_set_cmdq = panel_hbm_set_cmdq,
        .doze_disable = panel_doze_disable,
        .doze_enable = panel_doze_enable,
        .set_aod_light_mode = panel_set_aod_light_mode,
        .oplus_ofp_set_lhbm_pressed_icon = oplus_ofp_set_lhbm_pressed_icon,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
        .esd_backlight_recovery = oplus_esd_backlight_recovery,
};

static int lcm_get_modes(struct drm_panel *panel,
                         struct drm_connector *connector)
{
    struct drm_display_mode *mode[MODE_NUM * RES_NUM];
    int i = 0;

    for (i = 0; i < MODE_NUM * RES_NUM; i++) {
        mode[i] = drm_mode_duplicate(connector->dev, &display_mode[i]);
        if (!mode[i]) {
            pr_info("%s failed to add mode %ux%ux@%u\n", __func__, display_mode[i].hdisplay, display_mode[i].vdisplay, drm_mode_vrefresh(&display_mode[i]));
            return -ENOMEM;
        }
        drm_mode_set_name(mode[i]);
        mode[i]->type = DRM_MODE_TYPE_DRIVER;
        if (i == 2) {
            mode[i]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        }
        drm_mode_probed_add(connector, mode[i]);
    }

    connector->display_info.width_mm = PHYSICAL_WIDTH;
    connector->display_info.height_mm = PHYSICAL_HEIGHT;

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
    unsigned int res_switch;
    enum PANEL_ES panel_es_ver = get_panel_es_ver();

    pr_info("[LCM] sdc_nt37708s %s panel_es_ver = %d START\n", __func__ ,panel_es_ver);

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
    dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

    ret = of_property_read_u32(dev->of_node, "res-switch", &res_switch);
    if (ret < 0)
        res_switch = 0;
    else
        res_switch_type = (enum RES_SWITCH_TYPE)res_switch;
    pr_info("sdc_nt37708s lcm probe res_switch_type:%d\n", res_switch);

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

    //enable vddr 1.2v
    ctx->vddr1p2_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
        dev_err(dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
                __func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
        return PTR_ERR(ctx->vddr1p2_enable_gpio);
    }
    devm_gpiod_put(dev, ctx->vddr1p2_enable_gpio);

    usleep_range(5000, 5100);

    if (panel_es_ver == ES_T0) {
        //enable vdd-vss-eswire
        ctx->vdd_vss_eswire_enable_gpio = devm_gpiod_get(dev, "vdd-vss-eswire", GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->vdd_vss_eswire_enable_gpio)) {
            dev_err(dev, "%s: cannot get vdd_vss_eswire_enable_gpio %ld\n",
                    __func__, PTR_ERR(ctx->vdd_vss_eswire_enable_gpio));
            return PTR_ERR(ctx->vdd_vss_eswire_enable_gpio);
        }
        devm_gpiod_put(dev, ctx->vdd_vss_eswire_enable_gpio);

        usleep_range(5000, 5100);
    }

    //enable vci 3.0v
    ctx->ldo_vci3p0 = regulator_get(ctx->dev, "vci3p0");
    if (IS_ERR(ctx->ldo_vci3p0)) {
        pr_err("[lcd_info]cannot get ldo_vci3p0 %ld\n",
               PTR_ERR(ctx->ldo_vci3p0));
        return -517;
    } else {
        pr_info("[lcd_info]get ldo_vci3p0 success\n");
    }
    pr_info("[lcd_info]sdc_nt37708s %s ldo_vci3p0 LINE=%d\n", __func__, __LINE__);
    ret = lcm_enable_ldo_vci3p0(ctx, 1);

    ctx->prepared = true;
    ctx->enabled = true;

    drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

    drm_panel_add(&ctx->panel);

    ret = mipi_dsi_attach(dsi);
    if (ret < 0)
        drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
    mtk_panel_tch_handle_reg(&ctx->panel);
    ret = mtk_panel_ext_create(dev, &ext_params_120Hz, &ext_funcs, &ctx->panel);
    if (ret < 0)
        return ret;
#endif
    oplus_display_panel_dbv_probe(dev);
    register_device_proc("lcd", "A0037", "P_1");
    oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
    oplus_ofp_init(dev);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
    pr_info("[LCM] sdc_nt37708s %s END\n", __func__);

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
    { .compatible = "ae174,p,1,a0037,cmd,panel", },
    { }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
    .probe = lcm_probe,
    .remove = lcm_remove,
    .driver = {
        .name = "ae174_p_1_a0037_cmd_panel",
        .owner = THIS_MODULE,
        .of_match_table = lcm_of_match,
    },
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");