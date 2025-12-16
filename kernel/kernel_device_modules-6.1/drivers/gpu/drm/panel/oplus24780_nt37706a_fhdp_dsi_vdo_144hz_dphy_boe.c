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
#include "oplus24780_nt37706a_fhdp_dsi_vdo_144hz_dphy_boe.h"
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#include "../oplus/oplus_display_onscreenfingerprint.h"

static unsigned int nt37706a_vdo_dphy_buf_thresh[14] ={896, 1792, 2688, 3584, 4480,
    5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
static unsigned int nt37706a_vdo_dphy_range_min_qp[15] ={0, 4, 5, 5, 7, 7, 7, 7, 7,
    7, 9, 9, 9, 13, 16};
static unsigned int nt37706a_vdo_dphy_range_max_qp[15] ={8, 8, 9, 10, 11, 11, 11,
    12, 13, 14, 14, 15, 15, 16, 17};
static int nt37706a_vdo_dphy_range_bpg_ofs[15] ={2, 0, 0, -2, -4, -6, -8, -8, -8,
    -10, -10, -12, -12, -12, -12};

enum PANEL_ES {
    ES_T0    = 1,
    ES_EVT   = 2,
    ES_DV3_1 = 3,
    ES_DV3_2 = 4,
    ES_MP_1  = 5,
    ES_MP_2  = 6,
};

struct lcm {
    struct device *dev;
    struct drm_panel panel;
    struct backlight_device *backlight;
    struct gpio_desc *pxvdd_enable_gpio;
    struct gpio_desc *px_reset_gpio;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *vddr1p2_enable_gpio;
    struct regulator *ldo_vci3p0;
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
extern unsigned int m_db;
extern int oplus_display_panel_dbv_probe(struct device *dev);
static int current_fps = 60;
static bool aod_state = false;

#define MAX_NORMAL_BRIGHTNESS   3598
#define LCM_BRIGHTNESS_TYPE 2
#define FINGER_HBM_BRIGHTNESS 3840
#define AOD_INIT_BRIGHTNESS 938

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

static enum PANEL_ES inline get_panel_es_ver(void)
{
    enum PANEL_ES panel_es_ver = ES_T0;
    switch (m_db) {
        case 1:
        panel_es_ver = ES_T0;
        break;
        case 2:
        panel_es_ver = ES_EVT;
        break;
        case 3:
        panel_es_ver = ES_DV3_1;
        break;
        case 4:
        panel_es_ver = ES_DV3_2;
        break;
        case 5:
        panel_es_ver = ES_MP_1;
        break;
        case 6:
        panel_es_ver = ES_MP_2;
        break;
        default:
        panel_es_ver = ES_T0;
    }
    return panel_es_ver;
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
    enum PANEL_ES panel_es_ver = get_panel_es_ver();
    pr_info("debug for %s+,current_fps = %d, panel_es_ver = %d\n", __func__, current_fps, panel_es_ver);
    // Setting SPR panel boundary decolor
    //SPR panel boundary decolor
    if (panel_es_ver == ES_T0) {
        lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x07);
        lcm_dcs_write_seq_static(ctx, 0xB4, 0xC0);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x01);
        lcm_dcs_write_seq_static(ctx, 0xB4, 0x60, 0x80, 0x80);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x04);
        lcm_dcs_write_seq_static(ctx, 0xB4, 0x60, 0x20, 0x80);
        lcm_dcs_write_seq_static(ctx, 0xB4, 0x60, 0x60, 0x80);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x07);
        lcm_dcs_write_seq_static(ctx, 0xB4, 0x80, 0x80, 0x80);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x0A);
        lcm_dcs_write_seq_static(ctx, 0xB4, 0x80, 0x50, 0x80);
    }

    if (panel_es_ver == ES_T0) {
        //NVideo trim OSC2 to 167Mhz,
        //For other setting refer OP manual Ch 4.3
        lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x04);
        lcm_dcs_write_seq_static(ctx, 0xC3, 0xFF);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x09);
        lcm_dcs_write_seq_static(ctx, 0xC3, 0xFF);
        lcm_dcs_write_seq_static(ctx, 0xEA, 0xC0);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x11);
        lcm_dcs_write_seq_static(ctx, 0xEA, 0xC0);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x07);
        lcm_dcs_write_seq_static(ctx, 0xEA, 0x01, 0x02, 0x01, 0x34, 0x01, 0x34, 0x01, 0x34, 0x04, 0xD1);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x18);
        lcm_dcs_write_seq_static(ctx, 0xEA, 0x01, 0x8A, 0x01, 0xD7, 0x01, 0xD7, 0x01, 0xD7, 0x07, 0x5E);
        //VGXP by pad cap
        lcm_dcs_write_seq_static(ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x80);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x1F);
        lcm_dcs_write_seq_static(ctx, 0xF4, 0x0B);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x20);
        lcm_dcs_write_seq_static(ctx, 0xF4, 0x3F);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x08);
        lcm_dcs_write_seq_static(ctx, 0xFC, 0x03);
        lcm_dcs_write_seq_static(ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x80);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x24);
        lcm_dcs_write_seq_static(ctx, 0xF8, 0xFF);
    } else {
        //NVideo trim OSC2 to 160Mhz
        //For other setting refer OP manual Ch 4.3
        lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x04);
        lcm_dcs_write_seq_static(ctx, 0xC3, 0x0A);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x09);
        lcm_dcs_write_seq_static(ctx, 0xC3, 0x0A);
        lcm_dcs_write_seq_static(ctx, 0xEA, 0xC0);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x11);
        lcm_dcs_write_seq_static(ctx, 0xEA, 0xC0);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x07);
        lcm_dcs_write_seq_static(ctx, 0xEA, 0x01, 0x02, 0x01, 0x34, 0x01, 0x34, 0x01, 0x34, 0x04, 0xD1);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x18);
        lcm_dcs_write_seq_static(ctx, 0xEA, 0x01, 0x79, 0x01, 0xC4, 0x01, 0xC4, 0x01, 0xC4, 0x07, 0x0F);
        //VGXP by pad cap
        lcm_dcs_write_seq_static(ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x80);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x1F);
        lcm_dcs_write_seq_static(ctx, 0xF4, 0x0F);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x20);
        lcm_dcs_write_seq_static(ctx, 0xF4, 0x0F);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x08);
        lcm_dcs_write_seq_static(ctx, 0xFC, 0x01);
        lcm_dcs_write_seq_static(ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x80);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x24);
        lcm_dcs_write_seq_static(ctx, 0xF8, 0xFF);
    }

    //For ldle enter BIST
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xC0, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01);
    lcm_dcs_write_seq_static(ctx, 0x6F, 0x0B);
    lcm_dcs_write_seq_static(ctx, 0xD2, 0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x84);
    lcm_dcs_write_seq_static(ctx, 0x6F, 0x10);
    lcm_dcs_write_seq_static(ctx, 0xF8, 0x02);
    lcm_dcs_write_seq_static(ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x84);
    lcm_dcs_write_seq_static(ctx, 0xF2, 0x15);

    //LCTC Constraint For ESD Improve
    lcm_dcs_write_seq_static(ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x84);
    lcm_dcs_write_seq_static(ctx, 0x6F, 0xA9);
    lcm_dcs_write_seq_static(ctx, 0xF4, 0xF3);

    //FOD 光斑大小位置
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00);

    //RND START_X = 0x21C =540
    lcm_dcs_write_seq_static(ctx, 0x6F, 0x07);
    lcm_dcs_write_seq_static(ctx, 0xDF, 0x02, 0x1C);

    //RND START_Y = 0x994 = 2452
    lcm_dcs_write_seq_static(ctx, 0x6F, 0x09);
    lcm_dcs_write_seq_static(ctx, 0xDF, 0x09, 0x94);

    // RND END_X =0x2E2= 738
    lcm_dcs_write_seq_static(ctx, 0x6F, 0x0B);
    lcm_dcs_write_seq_static(ctx, 0xDF, 0x02, 0xE2);

    //RND END_Y = 0xA43 = 2651
    lcm_dcs_write_seq_static(ctx, 0x6F, 0x0D);
    lcm_dcs_write_seq_static(ctx, 0xDF, 0x0A, 0x5B);

    //CUC OFF
    if (panel_es_ver == ES_T0) {
        lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x08);
        lcm_dcs_write_seq_static(ctx, 0xB1, 0x02);
    }

    //Resolution Setting
    lcm_dcs_write_seq_static(ctx, 0x2A, 0x00, 0x00, 0x04, 0xFF);
    lcm_dcs_write_seq_static(ctx, 0x2B, 0x00, 0x00, 0x0A, 0xEF);

    //VESA1.2 10 bit_3.75倍压缩 DSC Setting
    //For other refer OP manual Ch 4.4
    lcm_dcs_write_seq_static(ctx, 0x90, 0x03);
    lcm_dcs_write_seq_static(ctx, 0x6F, 0x01);
    lcm_dcs_write_seq_static(ctx, 0x90, 0x43);
    lcm_dcs_write_seq_static(ctx, 0x91, 0xAB, 0xA8, 0x00, 0x28, 0xC2, 0x00, 0x02, 0x41, 0x04, 0x33, 0x00, 0x08, 0x02, 0x77, 0x02, 0x20, 0x10, 0xE0);

    //update PMIC SC6010 setting
    if (panel_es_ver == ES_DV3_1 || panel_es_ver == ES_MP_1) {
        lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x06);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0xFF, 0x00, 0x36, 0x00, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x0C);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x00, 0x36, 0x00, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x11);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x36, 0x36, 0x36, 0x36, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x18);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x01, 0x19, 0x00, 0x00, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x1D);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x01, 0x19, 0x00, 0x00, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x27);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x01, 0x01, 0x01, 0x01, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x2D);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x79, 0x71, 0x5D, 0x5D);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x3A);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x79, 0x71, 0x5D, 0x5D);
    } else {
        lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x06);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0xFF, 0x00, 0x36, 0x00, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x0C);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x00, 0x36, 0x00, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x11);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x36, 0x36, 0x36, 0x36, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x18);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x05, 0x19, 0x00, 0x00, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x1D);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x05, 0x19, 0x00, 0x00, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x27);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x05, 0x05, 0x05, 0x05, 0x00);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x2D);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x79, 0x71, 0x5D, 0x5D);
        lcm_dcs_write_seq_static(ctx, 0x6F, 0x3A);
        lcm_dcs_write_seq_static(ctx, 0xB5, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x79, 0x71, 0x5D, 0x5D);
    }

    if (panel_es_ver == ES_DV3_1 || panel_es_ver == ES_MP_1) {
        lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01);
        lcm_dcs_write_seq_static(ctx, 0xB0, 0x39, 0x39);
        lcm_dcs_write_seq_static(ctx, 0xB2, 0xAA, 0x22, 0x55, 0x01);
        lcm_dcs_write_seq_static(ctx, 0xB7, 0x2F, 0x2F, 0x2F, 0x2F, 0x2F, 0x00, 0x2F);
    }

    //Dimming OFF
    lcm_dcs_write_seq_static(ctx, 0x53, 0x20);

    //Video Mode Ext_VFP,VBPF Setting
    lcm_dcs_write_seq_static(ctx, 0x3B, 0x00, 0x1C, 0x00, 0x74, 0x00, 0x1C, 0x00, 0x7C, 0x00, 0x1C, 0x04, 0x54, 0x00, 0x1C, 0x00, 0x7C);
    lcm_dcs_write_seq_static(ctx, 0x6F, 0x10);
    lcm_dcs_write_seq_static(ctx, 0x3B, 0x00, 0x1C, 0x00, 0x7C);

    //DPC Temperature Setting
    lcm_dcs_write_seq_static(ctx, 0x81, 0x01, 0x19);

    //FPR1 EN
    lcm_dcs_write_seq_static(ctx, 0x88, 0x01);

    //120Hz Sleep Out
    //For other refer OP manual Ch 5.4.3
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x08);
    switch (current_fps) {
        case 60:
        lcm_dcs_write_seq_static(ctx, 0xB1, 0x02);
        lcm_dcs_write_seq_static(ctx, 0x2F, 0x03);
        break;
        case 90:
        lcm_dcs_write_seq_static(ctx, 0xB1, 0x02);
        lcm_dcs_write_seq_static(ctx, 0x2F, 0x02);
        break;
        case 120:
        lcm_dcs_write_seq_static(ctx, 0xB1, 0x02);
        lcm_dcs_write_seq_static(ctx, 0x2F, 0x01);
        break;
        case 144:
        lcm_dcs_write_seq_static(ctx, 0xB1, 0x03);
        lcm_dcs_write_seq_static(ctx, 0x2F, 0x00);
        break;
        default:
        lcm_dcs_write_seq_static(ctx, 0xB1, 0x02);
        lcm_dcs_write_seq_static(ctx, 0x2F, 0x03);
    }

    lcm_dcs_write_seq_static(ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x80);
    lcm_dcs_write_seq_static(ctx, 0x6F, 0x47);
    lcm_dcs_write_seq_static(ctx, 0xF2, 0x21);

    //TE ON
    lcm_dcs_write_seq_static(ctx, 0x35, 0x00);

    //Setting Loading Effect x1.0
    lcm_dcs_write_seq_static(ctx, 0x5F, 0x00, 0x00);

    //corner ON
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x07);
    lcm_dcs_write_seq_static(ctx, 0xC0, 0x07);
    lcm_dcs_write_seq_static(ctx, 0xC1, 0x1F, 0x00);

    //Switch DBV to 0x0000
    lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);

    //Sleep out
    lcm_dcs_write_seq_static(ctx, 0x11, 0x00);

    usleep_range(120*1000, 121*1000);

    //Display out
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

#define FRAME_WIDTH             (1280)
#define FRAME_HEIGHT            (2800)
#define FRAME_WIDTH_VIR         (1080)
#define FRAME_HEIGHT_VIR        (2362)
#define HFP_144HZ               (20)
#define HFP                     (132)
#define HBP                     (20)
#define HSA                     (4)
#define VFP_60HZ                (3076)
#define VFP_90HZ                (1108)
#define VFP_120HZ               (124)
#define VFP_144HZ               (116)
#define VBP                     (26)
#define VSA                     (2)

static const struct drm_display_mode display_mode[MODE_NUM * RES_NUM] = {
    // sdc_144hz_mode
    {
        .clock = ((FRAME_WIDTH + HFP_144HZ + HBP + HSA) * (FRAME_HEIGHT + VFP_144HZ + VBP + VSA) * 144) / 1000,
        .hdisplay = FRAME_WIDTH,
        .hsync_start = FRAME_WIDTH + HFP_144HZ,
        .hsync_end = FRAME_WIDTH + HFP_144HZ + HSA,
        .htotal = FRAME_WIDTH + HFP_144HZ + HSA + HBP,
        .vdisplay = FRAME_HEIGHT,
        .vsync_start = FRAME_HEIGHT + VFP_144HZ,
        .vsync_end = FRAME_HEIGHT + VFP_144HZ + VSA,
        .vtotal = FRAME_HEIGHT + VFP_144HZ + VSA + VBP,
    },
    // sdc_120hz_mode
    {
        .clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_120HZ + VBP + VSA) * 120) / 1000,
        .hdisplay = FRAME_WIDTH,
        .hsync_start = FRAME_WIDTH + HFP,
        .hsync_end = FRAME_WIDTH + HFP + HSA,
        .htotal = FRAME_WIDTH + HFP + HSA + HBP,
        .vdisplay = FRAME_HEIGHT,
        .vsync_start = FRAME_HEIGHT + VFP_120HZ,
        .vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
        .vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
    },

    // sdc_90hz_mode
    {
        .clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_90HZ + VBP + VSA) * 90) / 1000,
        .hdisplay = FRAME_WIDTH,
        .hsync_start = FRAME_WIDTH + HFP,
        .hsync_end = FRAME_WIDTH + HFP + HSA,
        .htotal = FRAME_WIDTH + HFP + HSA + HBP,
        .vdisplay = FRAME_HEIGHT,
        .vsync_start = FRAME_HEIGHT + VFP_90HZ,
        .vsync_end = FRAME_HEIGHT + VFP_90HZ + VSA,
        .vtotal = FRAME_HEIGHT + VFP_90HZ + VSA + VBP,
    },

    // sdc_60hz_mode
    {
        .clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_60HZ + VBP + VSA) * 60) / 1000,
        .hdisplay = FRAME_WIDTH,
        .hsync_start = FRAME_WIDTH + HFP,
        .hsync_end = FRAME_WIDTH + HFP + HSA,
        .htotal = FRAME_WIDTH + HFP + HSA + HBP,
        .vdisplay = FRAME_HEIGHT,
        .vsync_start = FRAME_HEIGHT + VFP_60HZ,
        .vsync_end = FRAME_HEIGHT + VFP_60HZ + VSA,
        .vtotal = FRAME_HEIGHT + VFP_60HZ + VSA + VBP,
    },

    // vir_fhd_sdc_144hz_mode
    {
        .clock = ((FRAME_WIDTH_VIR + HFP_144HZ + HBP + HSA) * (FRAME_HEIGHT_VIR + VFP_144HZ + VBP + VSA) * 144) / 1000,
        .hdisplay = FRAME_WIDTH_VIR,
        .hsync_start = FRAME_WIDTH_VIR + HFP_144HZ,
        .hsync_end = FRAME_WIDTH_VIR + HFP_144HZ + HSA,
        .htotal = FRAME_WIDTH_VIR + HFP_144HZ + HSA + HBP,
        .vdisplay = FRAME_HEIGHT_VIR,
        .vsync_start = FRAME_HEIGHT_VIR + VFP_144HZ,
        .vsync_end = FRAME_HEIGHT_VIR + VFP_144HZ + VSA,
        .vtotal = FRAME_HEIGHT_VIR + VFP_144HZ + VSA + VBP,
    },

    // vir_fhd_sdc_120hz_mode
    {
        .clock = ((FRAME_WIDTH_VIR + HFP + HBP + HSA) * (FRAME_HEIGHT_VIR + VFP_120HZ + VBP + VSA) * 120) / 1000,
        .hdisplay = FRAME_WIDTH_VIR,
        .hsync_start = FRAME_WIDTH_VIR + HFP,
        .hsync_end = FRAME_WIDTH_VIR + HFP + HSA,
        .htotal = FRAME_WIDTH_VIR + HFP + HSA + HBP,
        .vdisplay = FRAME_HEIGHT_VIR,
        .vsync_start = FRAME_HEIGHT_VIR + VFP_120HZ,
        .vsync_end = FRAME_HEIGHT_VIR + VFP_120HZ + VSA,
        .vtotal = FRAME_HEIGHT_VIR + VFP_120HZ + VSA + VBP,
    },

    // vir_fhd_sdc_90hz_mode
    {
        .clock = ((FRAME_WIDTH_VIR + HFP + HBP + HSA) * (FRAME_HEIGHT_VIR + VFP_90HZ + VBP + VSA) * 90) / 1000,
        .hdisplay = FRAME_WIDTH_VIR,
        .hsync_start = FRAME_WIDTH_VIR + HFP,
        .hsync_end = FRAME_WIDTH_VIR + HFP + HSA,
        .htotal = FRAME_WIDTH_VIR + HFP + HSA + HBP,
        .vdisplay = FRAME_HEIGHT_VIR,
        .vsync_start = FRAME_HEIGHT_VIR + VFP_90HZ,
        .vsync_end = FRAME_HEIGHT_VIR + VFP_90HZ + VSA,
        .vtotal = FRAME_HEIGHT_VIR + VFP_90HZ + VSA + VBP,
    },

    // vir_fhd_sdc_60hz_mode
    {
        .clock = ((FRAME_WIDTH_VIR + HFP + HBP + HSA) * (FRAME_HEIGHT_VIR + VFP_60HZ + VBP + VSA) * 60) / 1000,
        .hdisplay = FRAME_WIDTH_VIR,
        .hsync_start = FRAME_WIDTH_VIR + HFP,
        .hsync_end = FRAME_WIDTH_VIR + HFP + HSA,
        .htotal = FRAME_WIDTH_VIR + HFP + HSA + HBP,
        .vdisplay = FRAME_HEIGHT_VIR,
        .vsync_start = FRAME_HEIGHT_VIR + VFP_60HZ,
        .vsync_end = FRAME_HEIGHT_VIR + VFP_60HZ + VSA,
        .vtotal = FRAME_HEIGHT_VIR + VFP_60HZ + VSA + VBP,
    },
};

static struct mtk_panel_params ext_params_60Hz = {
    .pll_clk = 723,
    .data_rate = 1446,
    .change_fps_by_vfp_send_cmd_need_delay = 1,
    .dyn_fps = {
        .switch_en = 1,
        .vact_timing_fps = 60,
        .dfps_cmd_table[0] = {0, 6 , {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x08}},
        .dfps_cmd_table[1] = {0, 2 , {0xB1, 0x02}},
        .dfps_cmd_table[2] = {0, 2 , {0x2F, 0x03}},
    },
    .dyn = {
        .switch_en = 1,
        .pll_clk = 701,
        .data_rate = 1402,
        .hfp = 112,
        .vfp = VFP_60HZ,
        .vsa = VSA,
        .vbp = VBP,
        .hsa = HSA,
        .hbp = HBP,
    },
    .output_mode = MTK_PANEL_DSC_SINGLE_PORT,

    .cust_esd_check = 1,
    .esd_check_enable = 1,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
    },
    .lcm_esd_check_table[1] = {
        .cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00,
    },
    .vdo_mix_mode_en = false,
    // .lcm_esd_check_table[1] = {
    //     .cmd = 0x05, .count = 1, .para_list[0] = 0x00,
    // },



    //.round_corner_en = 1,
    //.corner_pattern_height = ROUND_CORNER_H_TOP,
    //.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    //.corner_pattern_tp_size = sizeof(top_rc_pattern),
    //.corner_pattern_lt_addr = (void *)top_rc_pattern,


    .oplus_display_global_dre = 1,
    .oplus_custom_hdr_color_tmp = true,
    .oplus_custom_hdr_red = 950,
    .oplus_custom_hdr_green = 1024,
    .oplus_custom_hdr_blue = 800,
    .oplus_panel_use_rgb_gain = true,

    .vendor = "BOE_NT37706A",
    .manufacture = "BOE_CASIOY",

    .oplus_ofp_need_keep_apart_backlight = true,
    .oplus_ofp_hbm_on_delay = 11,
    .oplus_ofp_pre_hbm_off_delay = 2,
    .oplus_ofp_hbm_off_delay = 11,
    .oplus_uiready_before_time = 17,
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

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
        .pic_height = 2800,
        .pic_width = 1280,
        .slice_height = 40,
        .slice_width = 640,
        .chunk_size = 640,
        .xmit_delay = 512,
        .dec_delay = 577,
        .scale_value = 32,
        .increment_interval = 1075,
        .decrement_interval = 8,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 631,
        .slice_bpg_offset = 544,
        .initial_offset = 6144,
        .final_offset = 4320,
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
            .rc_buf_thresh = nt37706a_vdo_dphy_buf_thresh,
            .range_min_qp = nt37706a_vdo_dphy_range_min_qp,
            .range_max_qp = nt37706a_vdo_dphy_range_max_qp,
            .range_bpg_ofs = nt37706a_vdo_dphy_range_bpg_ofs,
        },
    },
};

static struct mtk_panel_params ext_params_90Hz = {
    .pll_clk = 723,
    .data_rate = 1446,
    .change_fps_by_vfp_send_cmd_need_delay = 1,
    .dyn_fps = {
        .switch_en = 1,
        .vact_timing_fps = 90,
        .dfps_cmd_table[0] = {0, 6 , {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x08}},
        .dfps_cmd_table[1] = {0, 2 , {0xB1, 0x02}},
        .dfps_cmd_table[2] = {0, 2 , {0x2F, 0x02}},
    },
    .dyn = {
        .switch_en = 1,
        .pll_clk = 701,
        .data_rate = 1402,
        .hfp = 112,
        .vfp = VFP_90HZ,
        .vsa = VSA,
        .vbp = VBP,
        .hsa = HSA,
        .hbp = HBP,
    },
    .output_mode = MTK_PANEL_DSC_SINGLE_PORT,

    .cust_esd_check = 1,
    .esd_check_enable = 1,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
    },
    .lcm_esd_check_table[1] = {
        .cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00,
    },
    .vdo_mix_mode_en = false,
    // .lcm_esd_check_table[1] = {
    //     .cmd = 0x05, .count = 1, .para_list[0] = 0x00,
    // },



    //.round_corner_en = 1,
    //.corner_pattern_height = ROUND_CORNER_H_TOP,
    //.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    //.corner_pattern_tp_size = sizeof(top_rc_pattern),
    //.corner_pattern_lt_addr = (void *)top_rc_pattern,


    .oplus_display_global_dre = 1,
    .oplus_custom_hdr_color_tmp = true,
    .oplus_custom_hdr_red = 950,
    .oplus_custom_hdr_green = 1024,
    .oplus_custom_hdr_blue = 800,
    .oplus_panel_use_rgb_gain = true,

    .vendor = "BOE_NT37706A",
    .manufacture = "BOE_CASIOY",

    .oplus_ofp_need_keep_apart_backlight = true,
    .oplus_ofp_hbm_on_delay = 11,
    .oplus_ofp_pre_hbm_off_delay = 2,
    .oplus_ofp_hbm_off_delay = 11,
    .oplus_uiready_before_time = 17,
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

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
        .pic_height = 2800,
        .pic_width = 1280,
        .slice_height = 40,
        .slice_width = 640,
        .chunk_size = 640,
        .xmit_delay = 512,
        .dec_delay = 577,
        .scale_value = 32,
        .increment_interval = 1075,
        .decrement_interval = 8,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 631,
        .slice_bpg_offset = 544,
        .initial_offset = 6144,
        .final_offset = 4320,
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
            .rc_buf_thresh = nt37706a_vdo_dphy_buf_thresh,
            .range_min_qp = nt37706a_vdo_dphy_range_min_qp,
            .range_max_qp = nt37706a_vdo_dphy_range_max_qp,
            .range_bpg_ofs = nt37706a_vdo_dphy_range_bpg_ofs,
        },
    },
};


static struct mtk_panel_params ext_params_120Hz = {
    .pll_clk = 723,
    .data_rate = 1446,
    .change_fps_by_vfp_send_cmd_need_delay = 1,
    .dyn_fps = {
        .switch_en = 1,
        .vact_timing_fps = 120,
        // .send_mode = 1,
        // .send_cmd_need_delay = 1,
        .dfps_cmd_table[0] = {0, 6 , {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x08}},
        .dfps_cmd_table[1] = {0, 2 , {0xB1, 0x02}},
        .dfps_cmd_table[2] = {0, 2 , {0x2F, 0x01}},
    },
    .dyn = {
        .switch_en = 1,
        .pll_clk = 701,
        .data_rate = 1402,
        .hfp = 112,
        .vfp = VFP_120HZ,
        .vsa = VSA,
        .vbp = VBP,
        .hsa = HSA,
        .hbp = HBP,
    },
    .output_mode = MTK_PANEL_DSC_SINGLE_PORT,

    .cust_esd_check = 1,
    .esd_check_enable = 1,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
    },
    .lcm_esd_check_table[1] = {
        .cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00,
    },
    .vdo_mix_mode_en = false,
    // .lcm_esd_check_table[1] = {
    //     .cmd = 0x05, .count = 1, .para_list[0] = 0x00,
    // },


    //.round_corner_en = 1,
    //.corner_pattern_height = ROUND_CORNER_H_TOP,
    //.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    //.corner_pattern_tp_size = sizeof(top_rc_pattern),
    //.corner_pattern_lt_addr = (void *)top_rc_pattern,

    .oplus_display_global_dre = 1,
    .oplus_custom_hdr_color_tmp = true,
    .oplus_custom_hdr_red = 950,
    .oplus_custom_hdr_green = 1024,
    .oplus_custom_hdr_blue = 800,
    .oplus_panel_use_rgb_gain = true,

    .vendor = "BOE_NT37706A",
    .manufacture = "BOE_CASIOY",

    .oplus_ofp_need_keep_apart_backlight = true,
    .oplus_ofp_hbm_on_delay = 11,
    .oplus_ofp_pre_hbm_off_delay = 2,
    .oplus_ofp_hbm_off_delay = 11,
    .oplus_uiready_before_time = 17,
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

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
        .pic_height = 2800,
        .pic_width = 1280,
        .slice_height = 40,
        .slice_width = 640,
        .chunk_size = 640,
        .xmit_delay = 512,
        .dec_delay = 577,
        .scale_value = 32,
        .increment_interval = 1075,
        .decrement_interval = 8,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 631,
        .slice_bpg_offset = 544,
        .initial_offset = 6144,
        .final_offset = 4320,
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
            .rc_buf_thresh = nt37706a_vdo_dphy_buf_thresh,
            .range_min_qp = nt37706a_vdo_dphy_range_min_qp,
            .range_max_qp = nt37706a_vdo_dphy_range_max_qp,
            .range_bpg_ofs = nt37706a_vdo_dphy_range_bpg_ofs,
        },
    },
};

static struct mtk_panel_params ext_params_144Hz = {
    .pll_clk = 723,
    .data_rate = 1446,
    .change_fps_by_vfp_send_cmd_need_delay = 1,
    .dyn_fps = {
        .switch_en = 1,
        .vact_timing_fps = 144,
        // .send_mode = 1,
        // .send_cmd_need_delay = 1,
        .dfps_cmd_table[0] = {0, 6 , {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x08}},
        .dfps_cmd_table[1] = {0, 2 , {0xB1, 0x03}},
        .dfps_cmd_table[2] = {0, 2 , {0x2F, 0x00}},
    },
    .output_mode = MTK_PANEL_DSC_SINGLE_PORT,

    .cust_esd_check = 1,
    .esd_check_enable = 1,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
    },
    .lcm_esd_check_table[1] = {
        .cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00,
    },
    .vdo_mix_mode_en = false,
    // .lcm_esd_check_table[1] = {
    //     .cmd = 0x05, .count = 1, .para_list[0] = 0x00,
    // },


    //.round_corner_en = 1,
    //.corner_pattern_height = ROUND_CORNER_H_TOP,
    //.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    //.corner_pattern_tp_size = sizeof(top_rc_pattern),
    //.corner_pattern_lt_addr = (void *)top_rc_pattern,

    .oplus_display_global_dre = 1,
    .oplus_custom_hdr_color_tmp = true,
    .oplus_custom_hdr_red = 950,
    .oplus_custom_hdr_green = 1024,
    .oplus_custom_hdr_blue = 800,
    .oplus_panel_use_rgb_gain = true,

    .vendor = "BOE_NT37706A",
    .manufacture = "BOE_CASIOY",

    .oplus_ofp_need_keep_apart_backlight = true,
    .oplus_ofp_hbm_on_delay = 11,
    .oplus_ofp_pre_hbm_off_delay = 2,
    .oplus_ofp_hbm_off_delay = 11,
    .oplus_uiready_before_time = 17,
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

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
        .pic_height = 2800,
        .pic_width = 1280,
        .slice_height = 40,
        .slice_width = 640,
        .chunk_size = 640,
        .xmit_delay = 512,
        .dec_delay = 577,
        .scale_value = 32,
        .increment_interval = 1075,
        .decrement_interval = 8,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 631,
        .slice_bpg_offset = 544,
        .initial_offset = 6144,
        .final_offset = 4320,
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
            .rc_buf_thresh = nt37706a_vdo_dphy_buf_thresh,
            .range_min_qp = nt37706a_vdo_dphy_range_min_qp,
            .range_max_qp = nt37706a_vdo_dphy_range_max_qp,
            .range_bpg_ofs = nt37706a_vdo_dphy_range_bpg_ofs,
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
    static unsigned int last_brightness = 0;
    char bl_tb0[] = {0x51, 0x00, 0x00};
    char bl_tb1[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
    char bl_tb2[] = {0x6F, 0x0B};
    char bl_tb3_1[] = {0xC0, 0x64, 0x00};
    char bl_tb3_2[] = {0xC0, 0xE4, 0x00};

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

    if ((get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) && (mapped_level > 1)) {
        mapped_level = 1023;
    }

    if (last_brightness == AOD_INIT_BRIGHTNESS && oplus_display_brightness == AOD_INIT_BRIGHTNESS) {
        pr_info("%s,repeat aod init level = %d,", __func__, oplus_display_brightness);
        return -EINVAL;
    }

    if (last_brightness >= 1088 && oplus_display_brightness < 1088) {
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
        cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
        cb(dsi, handle, bl_tb3_1, ARRAY_SIZE(bl_tb3_1));
    } else if (last_brightness < 1088 && oplus_display_brightness >= 1088) {
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
        cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
        cb(dsi, handle, bl_tb3_2, ARRAY_SIZE(bl_tb3_2));
    }

    last_brightness = oplus_display_brightness;
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

static int oplus_ofp_set_lhbm_pressed_icon_single(struct drm_panel *panel, void *dsi,
        dcs_write_gce cb, void *handle, bool en)
{
    struct lcm *ctx = panel_to_lcm(panel);
    enum PANEL_ES panel_es_ver = get_panel_es_ver();
    int i = 0;

    if (!dsi || !cb) {
        pr_err("Invalid params\n");
        return -EINVAL;
    }

    pr_info("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, en);
    if (panel_es_ver == ES_T0) {
        if (en == 1) {
            for (i = 0; i < sizeof(lcm_finger_lhbm_on_setting_T0)/sizeof(struct LCM_setting_table); i++){
                cb(dsi, handle, lcm_finger_lhbm_on_setting_T0[i].para_list, lcm_finger_lhbm_on_setting_T0[i].count);
            }
        } else if (en == 0) {
            for (i = 0; i < sizeof(lcm_lhbm_off_setbrightness_normal_T0)/sizeof(struct LCM_setting_table); i++){
                cb(dsi, handle, lcm_lhbm_off_setbrightness_normal_T0[i].para_list, lcm_lhbm_off_setbrightness_normal_T0[i].count);
            }
            lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
        }
    } else {
        if (en == 1) {
            for (i = 0; i < sizeof(lcm_finger_lhbm_on_setting_EVT)/sizeof(struct LCM_setting_table); i++){
                cb(dsi, handle, lcm_finger_lhbm_on_setting_EVT[i].para_list, lcm_finger_lhbm_on_setting_EVT[i].count);
            }
        } else if (en == 0) {
            for (i = 0; i < sizeof(lcm_lhbm_off_setbrightness_normal_EVT)/sizeof(struct LCM_setting_table); i++){
                cb(dsi, handle, lcm_lhbm_off_setbrightness_normal_EVT[i].para_list, lcm_lhbm_off_setbrightness_normal_EVT[i].count);
            }
            lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
        }
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

static int oplus_display_panel_set_hbm_max(void *dsi, dcs_write_gce_pack cb1, dcs_write_gce cb2, void *handle, unsigned int en) {
    unsigned int i = 0;

    pr_info("en=%d\n", en);

    if (!dsi || !cb1 || !cb2) {
        pr_info("Invalid params\n");
        return -EINVAL;
    }

    if (en) {
        for (i = 0; i < sizeof(dsi_switch_hbm_apl_on) / sizeof(struct LCM_setting_table); i++) {
            cb2(dsi, handle, dsi_switch_hbm_apl_on[i].para_list, dsi_switch_hbm_apl_on[i].count);
        }
        pr_info("Enter hbm_max mode");
    } else if (!en) {
        dsi_switch_hbm_apl_off[1].para_list[1] = oplus_display_brightness >> 8;
        dsi_switch_hbm_apl_off[1].para_list[2] = oplus_display_brightness & 0xFF;
        for (i = 0; i < sizeof(dsi_switch_hbm_apl_off) / sizeof(struct LCM_setting_table); i++) {
            cb2(dsi, handle, dsi_switch_hbm_apl_off[i].para_list, dsi_switch_hbm_apl_off[i].count);
        }
        pr_info("hbm_max off, restore bl:%d\n", oplus_display_brightness);
    }

    return 0;
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

    if (oplus_display_brightness > BRIGHTNESS_HALF) {
        AOD_on_setting[0].cmd = REGFLAG_CMD;
        AOD_on_setting[0].count = 3;
        AOD_on_setting[0].para_list[0] = 0x51;
        AOD_on_setting[0].para_list[1] = 0x0D;
        AOD_on_setting[0].para_list[2] = 0xBB;

        AOD_on_setting[1].cmd = REGFLAG_DELAY;
        AOD_on_setting[1].count = 20;
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

static unsigned int lcm_enable_ldo_vci3p0(struct lcm *ctx, int en)
{
    unsigned int ret = 0;

    pr_info("[lcd_info]%s +\n", __func__);
    if(!ctx->ldo_vci3p0) {
        pr_err("%s error return -1\n", __func__);
        return -1;
    }

    if(en) {
        ret = regulator_set_voltage(ctx->ldo_vci3p0, 3000000, 3000000);
        ret = regulator_enable(ctx->ldo_vci3p0);
        pr_info("[lcd_info]%s vddio enable\n", __func__);
    } else {
        ret = regulator_disable(ctx->ldo_vci3p0);
        pr_info("[lcd_info]%s vddio disable\n", __func__);
    }
    pr_info("[lcd_info]%s -\n", __func__);

    return ret;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
    struct lcm *ctx = panel_to_lcm(panel);
    int ret;

    if (ctx->prepared)
        return 0;

    pr_info("%s: boe_nt37706a lcm ctx->prepared %d\n", __func__, ctx->prepared);

    ctx->pxvdd_enable_gpio =
        devm_gpiod_get(ctx->dev, "pxvdd-enable", GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->pxvdd_enable_gpio)) {
        dev_err(ctx->dev, "%s: cannot get pxvdd_enable_gpio %ld\n",
            __func__, PTR_ERR(ctx->pxvdd_enable_gpio));
        return PTR_ERR(ctx->pxvdd_enable_gpio);
    }
    gpiod_set_value(ctx->pxvdd_enable_gpio, 1);
    devm_gpiod_put(ctx->dev, ctx->pxvdd_enable_gpio);
    usleep_range(2000, 2100);
    ctx->px_reset_gpio =
        devm_gpiod_get(ctx->dev, "px-reset", GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->px_reset_gpio)) {
        dev_err(ctx->dev, "%s: cannot get px_reset_gpio %ld\n",
            __func__, PTR_ERR(ctx->px_reset_gpio));
        return PTR_ERR(ctx->px_reset_gpio);
    }
    gpiod_set_value(ctx->px_reset_gpio, 1);
    devm_gpiod_put(ctx->dev, ctx->px_reset_gpio);
    usleep_range(2000, 2100);
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
    //enable vci 3.0v
    ret = lcm_enable_ldo_vci3p0(ctx, 1);
    if(ret) {
        pr_err("[lcd_info]%s: set vddio on failed! ret=%d\n", __func__, ret);
    }

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

    pr_info("%s: boe_nt37706a lcm ctx->prepared %d\n", __func__, ctx->prepared);

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
    //disable 3.0V
    ret = lcm_enable_ldo_vci3p0(ctx, 0);
    if(ret) {
        pr_err("[lcd_info]%s: set vddio off failed! ret=%d\n", __func__, ret);
    }
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
    ctx->pxvdd_enable_gpio =
        devm_gpiod_get(ctx->dev, "pxvdd-enable", GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->pxvdd_enable_gpio)) {
        dev_err(ctx->dev, "%s: cannot get pxvdd_enable_gpio %ld\n",
            __func__, PTR_ERR(ctx->pxvdd_enable_gpio));
        return PTR_ERR(ctx->pxvdd_enable_gpio);
    }
    gpiod_set_value(ctx->pxvdd_enable_gpio, 0);
    devm_gpiod_put(ctx->dev, ctx->pxvdd_enable_gpio);
    usleep_range(5000, 5100);
    ctx->px_reset_gpio =
        devm_gpiod_get(ctx->dev, "px-reset", GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->px_reset_gpio)) {
        dev_err(ctx->dev, "%s: cannot get px_reset_gpio %ld\n",
            __func__, PTR_ERR(ctx->px_reset_gpio));
        return PTR_ERR(ctx->px_reset_gpio);
    }
    gpiod_set_value(ctx->px_reset_gpio, 0);
    devm_gpiod_put(ctx->dev, ctx->px_reset_gpio);

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

enum RES_SWITCH_TYPE mtk_get_res_switch_type(void)
{
    pr_info("res_switch_type: %d\n", res_switch_type);
    return res_switch_type;
}

int mtk_scaling_mode_mapping(int mode_idx)
{
    return MODE_MAPPING_RULE(mode_idx);
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
    }else if (m_vrefresh == 144) {
        ext->params = &ext_params_144Hz;
        current_fps = 144;
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
    .get_res_switch_type = mtk_get_res_switch_type,
    .scaling_mode_mapping = mtk_scaling_mode_mapping,
    //.mode_switch = mode_switch,
    .set_hbm = lcm_set_hbm,
    //.hbm_set_cmdq = panel_hbm_set_cmdq,
    .oplus_ofp_set_lhbm_pressed_icon_single = oplus_ofp_set_lhbm_pressed_icon_single,
    .doze_disable = panel_doze_disable,
    .doze_enable = panel_doze_enable,
    .lcm_set_hbm_max = oplus_display_panel_set_hbm_max,
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
        if (i == 3) {
            mode[i]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        }
        drm_mode_probed_add(connector, mode[i]);
    }

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
    unsigned int res_switch;

    pr_info("[LCM] boe_nt37706a %s START\n", __func__);


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
             | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET;

    ret = of_property_read_u32(dev->of_node, "res-switch", &res_switch);
    if (ret < 0)
        res_switch = 0;
    else
        res_switch_type = (enum RES_SWITCH_TYPE)res_switch;
    pr_info("lcm probe res_switch_type:%d\n", res_switch);

    backlight = of_parse_phandle(dev->of_node, "backlight", 0);
    if (backlight) {
        ctx->backlight = of_find_backlight_by_node(backlight);
        of_node_put(backlight);

        if (!ctx->backlight)
            return -EPROBE_DEFER;
    }

    ctx->pxvdd_enable_gpio = devm_gpiod_get(dev, "pxvdd-enable", GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->pxvdd_enable_gpio)) {
        dev_info(dev, "%s: cannot get pxvdd-enable-gpios %ld\n",
            __func__, PTR_ERR(ctx->pxvdd_enable_gpio));
        return PTR_ERR(ctx->pxvdd_enable_gpio);
    }
    devm_gpiod_put(dev, ctx->pxvdd_enable_gpio);

    ctx->px_reset_gpio = devm_gpiod_get(dev, "px-reset", GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->px_reset_gpio)) {
        dev_info(dev, "%s: cannot get px-reset-gpios %ld\n",
            __func__, PTR_ERR(ctx->px_reset_gpio));
        return PTR_ERR(ctx->px_reset_gpio);
    }
    devm_gpiod_put(dev, ctx->px_reset_gpio);

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

    //enable vci 3.0v
    ctx->ldo_vci3p0 = regulator_get(ctx->dev, "vci3p0");
    if (IS_ERR(ctx->ldo_vci3p0)) {
        pr_err("[lcd_info]cannot get ldo_vci3p0 %ld\n",
            PTR_ERR(ctx->ldo_vci3p0));
        return -517;
    } else {
        pr_info("[lcd_info]get ldo_vci3p0 success\n");
    }
    pr_info("[lcd_info]%s ldo_vci3p0 LINE=%d\n", __func__, __LINE__);
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
    ret = mtk_panel_ext_create(dev, &ext_params_60Hz, &ext_funcs, &ctx->panel);
    if (ret < 0)
        return ret;
#endif
    oplus_display_panel_dbv_probe(dev);
    register_device_proc("lcd", "BOE_NT37706A", "BOE_CASIOY");
    ctx->hbm_en = false;
    oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
    oplus_ofp_init(dev);


    pr_info("[LCM] %s- lcm, boe_nt37706a, END\n", __func__);


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
    { .compatible = "oplus24780,nt37706a,fhdp,dsi,vdo,144hz,dphy,boe", },
    { }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
    .probe = lcm_probe,
    .remove = lcm_remove,
    .driver = {
        .name = "oplus24780_nt37706a_fhdp_dsi_vdo_144hz_dphy_boe",
        .owner = THIS_MODULE,
        .of_match_table = lcm_of_match,
    },
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");