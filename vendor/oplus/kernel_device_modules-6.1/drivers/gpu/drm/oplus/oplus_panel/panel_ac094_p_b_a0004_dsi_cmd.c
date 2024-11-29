// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 OPPO Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <soc/oplus/device_info.h>
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "panel_ac094_p_b_a0004_dsi_cmd.h"

#ifdef OPLUS_FEATURE_DISPLAY
#include "../../oplus/oplus_drm_disp_panel.h"
#include "../../oplus/oplus_display_temp_compensation.h"
#include "../../oplus/oplus_display_mtk_debug.h"
#endif /* OPLUS_FEATURE_DISPLAY */

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "../../oplus/oplus_adfr_ext.h"
#endif

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "../../oplus/oplus_display_onscreenfingerprint.h"
#include "../../mediatek/mediatek_v2/mtk-cmdq-ext.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#ifdef OPLUS_FEATURE_DISPLAY_HPWM
#include "../../oplus/oplus_display_high_frequency_pwm.h"
#endif /* OPLUS_FEATURE_DISPLAY_HPWM */

#include "../mtk_round_corner/data_hw_roundedpattern_ac094.h"

#define PHYSICAL_WIDTH 71289
#define PHYSICAL_HEIGHT 156792
#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2
#define DRM_PANEL_EVENT_PWM_TURBO  0x14
DEFINE_MUTEX(oplus_pwm_lock);

#define RES_NUM                 (2)
#define MODE_MAPPING_RULE(x)    ((x) % (MODE_NUM))

#define FHD_FRAME_WIDTH    (1080)
#define FHD_HFP            (9)
#define FHD_HSA            (2)
#define FHD_HBP            (21)
#define FHD_HTOTAL         (FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP)
#define FHD_FRAME_HEIGHT   (2376)
#define FHD_VFP            (66)
#define FHD_VSA            (24)
#define FHD_VBP            (36)
#define FHD_VTOTAL         (FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP)
#define FHD_FRAME_TOTAL    (FHD_HTOTAL * FHD_VTOTAL)

#define FHD_VREFRESH_120   (120)
#define FHD_VREFRESH_60    (60)
#define FHD_VREFRESH_90    (90)

#define FHD_CLK_120_X10    ((FHD_FRAME_TOTAL * FHD_VREFRESH_120) / 100)
#define FHD_CLK_60_X10     ((FHD_FRAME_TOTAL * FHD_VREFRESH_60) / 100)
#define FHD_CLK_90_X10     ((FHD_FRAME_TOTAL * FHD_VREFRESH_90) / 100)

#define FHD_CLK_120		(((FHD_CLK_120_X10 % 10) != 0) ?             \
			(FHD_CLK_120_X10 / 10 + 1) : (FHD_CLK_120_X10 / 10))
#define FHD_CLK_90		(((FHD_CLK_90_X10 % 10) != 0) ?              \
			(FHD_CLK_90_X10 / 10 + 1) : (FHD_CLK_90_X10 / 10))
#define FHD_CLK_60		(((FHD_CLK_60_X10 % 10) != 0) ?              \
			(FHD_CLK_60_X10 / 10 + 1) : (FHD_CLK_60_X10 / 10))

static enum RES_SWITCH_TYPE res_switch_type = RES_SWITCH_NO_USE;


static struct regulator *vmc_ldo;
static struct regulator *vrfio18_aif;
static ktime_t mode_switch_begin_time = 0;
static unsigned int panel_mode_id = FHD_SDC120;// default 120fps


extern unsigned int last_backlight;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern unsigned int oplus_enhance_mipi_strength;
extern unsigned int m_db;
extern unsigned int m_id;

extern void lcdinfo_notify(unsigned long val, void *v);
extern int oplus_serial_number_probe(struct device *dev);

extern inline void set_pwm_turbo_switch_state(enum PWM_SWITCH_STATE state);
extern void set_pwm_turbo_power_on(bool en);

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle);

enum oplus_adfr_manual_tianma_min_fps_value {
	OPLUS_ADFR_MANAUL_MIN_FPS_MAX = 0x00,
	OPLUS_ADFR_MANAUL_MIN_FPS_60HZ = 0x01,
	OPLUS_ADFR_MANAUL_MIN_FPS_40HZ = 0x02,
	OPLUS_ADFR_MANAUL_MIN_FPS_30HZ = 0x03,
	OPLUS_ADFR_MANAUL_MIN_FPS_20HZ = 0x05,
	OPLUS_ADFR_MANAUL_MIN_FPS_10HZ = 0x0B,
	OPLUS_ADFR_MANAUL_MIN_FPS_5HZ  = 0x17,
	OPLUS_ADFR_MANAUL_MIN_FPS_1HZ  = 0x77,
	OPLUS_ADFR_MANAUL_MIN_FPS90_45HZ = 0x01,
	OPLUS_ADFR_MANAUL_MIN_FPS90_30HZ = 0x02,
	OPLUS_ADFR_MANAUL_MIN_FPS90_15HZ = 0x05,
	OPLUS_ADFR_MANAUL_MIN_FPS90_10HZ = 0x08,
	OPLUS_ADFR_MANAUL_MIN_FPS90_5HZ  = 0x11,
	OPLUS_ADFR_MANAUL_MIN_FPS90_1HZ  = 0x59,
};

enum PANEL_ES {
	ES_DV1 = 1,
	ES_DV2 = 2,
};

struct lcm_pmic_info {
	struct regulator *reg_vrfio18_aif;
	struct regulator *reg_vmc3p0;
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *vddr1p2_enable_gpio;
	struct gpio_desc *vddr_aod_enable_gpio;
	struct gpio_desc *te_switch_gpio,*te_out_gpio;
	/* ADFR:save display mode for panel init */
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,                          \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static int inline get_panel_es_ver(void)
{
	int ret = 0;
	if (m_db ==1) {
		if (m_id == 0)
			ret = ES_DV1;
		else if(m_id == 1)
			ret = ES_DV2;
	} else if (m_db >= 2)
		ret = ES_DV2;
	return ret;
}


#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}
static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

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

static int lcm_panel_vrfio18_aif_regulator_init(struct device *dev)
{
        static int regulator_vufs_inited;
        int ret = 0;

        if (regulator_vufs_inited)
                return ret;
        pr_err("[LCM] lcm_panel_vrfio18_aif_regulator_init\n");

        /* please only get regulator once in a driver */
        vrfio18_aif = devm_regulator_get(dev, "1p8");
        if (IS_ERR(vrfio18_aif)) { /* handle return value */
                ret = PTR_ERR(vrfio18_aif);
                pr_err("get vrfio18_aif fail, error: %d\n", ret);
                //return ret;
        }
        regulator_vufs_inited = 1;
        return ret; /* must be 0 */

}

static int lcm_panel_vrfio18_aif_enable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_vrfio18_aif_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vrfio18_aif)) {
		ret = regulator_set_voltage(vrfio18_aif, 1800000, 1800000);
		if (ret < 0)
			pr_err("[LCM] set voltage vrfio18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vrfio18_aif)) {
		ret = regulator_enable(vrfio18_aif);
		if (ret < 0)
			pr_err("[LCM] enable regulator vrfio18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_err("[LCM] lcm_panel_vrfio18_aif_enable\n");

        return retval;
}

static int lcm_panel_vrfio18_aif_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_vrfio18_aif_regulator_init(dev);

	if (IS_ERR_OR_NULL(vrfio18_aif)) {
		pr_err("[LCM] disable regulator fail, vrfio18_aif is null\n");
	}

	if (!IS_ERR_OR_NULL(vrfio18_aif)) {
		ret = regulator_disable(vrfio18_aif);
		pr_err("[LCM] disable regulator vrfio18_aif 1.8v\n");
		if (ret < 0)
			pr_err("[LCM] disable regulator vrfio18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
	return ret;
}

static int lcm_panel_vmc_ldo_regulator_init(struct device *dev)
{
	static int regulator_vmc_inited;
	int ret = 0;

	if (regulator_vmc_inited)
		return ret;
	pr_err("[LCM] lcm_panel_vmc_ldo_regulator_init\n");

	/* please only get regulator once in a driver */
	vmc_ldo = devm_regulator_get(dev, "3p0");
	if (IS_ERR(vmc_ldo)) { /* handle return value */
		ret = PTR_ERR(vmc_ldo);
		pr_err("[LCM] vmc_ldo fail, error: %d\n", ret);
		//return ret;
	}
	regulator_vmc_inited = 1;
	return ret; /* must be 0 */
}

static int lcm_panel_vmc_ldo_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_vmc_ldo_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vmc_ldo)) {
		ret = regulator_set_voltage(vmc_ldo, 3000000, 3000000);
		if (ret < 0)
			pr_err("[LCM] voltage vmc_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vmc_ldo)) {
		ret = regulator_enable(vmc_ldo);
		if (ret < 0)
			pr_err("[LCM] enable regulator vmc_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_err("[LCM] lcm_panel_vmc_ldo_enable\n");

        return retval;
}

static int lcm_panel_vmc_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_vmc_ldo_regulator_init(dev);

	if (IS_ERR_OR_NULL(vmc_ldo)) {
		pr_err("[LCM] disable regulator fail, vmc_ldo is null\n");
	}

	if (!IS_ERR_OR_NULL(vmc_ldo)) {
		ret = regulator_disable(vmc_ldo);
		pr_err("[LCM] disable regulator vmc_ldo 3v\n");
		if (ret < 0)
			pr_err("[LCM] disable regulator vmc_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	return ret;
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
    	pr_info("get_mode_enum drm_display_mode *m is null, default 120fps\n");
    	ret = FHD_SDC120;
    	return ret;
	}

	m_vrefresh = drm_mode_vrefresh(m);

    if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
    	ret = FHD_SDC60;
    } else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
    	ret = FHD_SDC90;
    } else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
    	ret = FHD_SDC120;
    } else if (m_vrefresh == 120 && m->hskew == OPLUS_ADFR) {
    	ret = FHD_OPLUS120;
    }

	panel_mode_id = ret;


	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	struct drm_display_mode *m = ctx->m;

	mode_id = get_mode_enum(m);
	pr_info("%s panel_es_ver=%d, mode id=%d\n", __func__, get_panel_es_ver(), mode_id);

	switch (mode_id) {
	case FHD_SDC60:
		pr_info("fhd_dsi_on_cmd_sdc60\n");
		if (get_panel_es_ver() == ES_DV1)
			push_table(ctx, dsi_on_cmd_sdc60_sv1, sizeof(dsi_on_cmd_sdc60_sv1) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc60_sv2, sizeof(dsi_on_cmd_sdc60_sv2) / sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC90:
		pr_info("fhd_dsi_on_cmd_sdc90\n");
			if (get_panel_es_ver() == ES_DV1)
				push_table(ctx, dsi_on_cmd_sdc90_sv1, sizeof(dsi_on_cmd_sdc90_sv1) / sizeof(struct LCM_setting_table));
			else if (get_panel_es_ver() == ES_DV2)
				push_table(ctx, dsi_on_cmd_sdc90_sv2, sizeof(dsi_on_cmd_sdc90_sv2) / sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC120:
		pr_info("fhd_dsi_on_cmd_sdc120\n");
			if (get_panel_es_ver() == ES_DV1)
				push_table(ctx, dsi_on_cmd_sdc120_sv1, sizeof(dsi_on_cmd_sdc120_sv1) / sizeof(struct LCM_setting_table));
			else if (get_panel_es_ver() == ES_DV2)
				push_table(ctx, dsi_on_cmd_sdc120_sv2, sizeof(dsi_on_cmd_sdc120_sv2) / sizeof(struct LCM_setting_table));
		break;
	case FHD_OPLUS120:
		pr_info("fhd_dsi_on_cmd_oplus120\n");
			if (get_panel_es_ver() == ES_DV1)
				push_table(ctx, dsi_on_cmd_oa120_sv1, sizeof(dsi_on_cmd_oa120_sv1) / sizeof(struct LCM_setting_table));
			else if (get_panel_es_ver() == ES_DV2)
				push_table(ctx, dsi_on_cmd_oa120_sv2, sizeof(dsi_on_cmd_oa120_sv2) / sizeof(struct LCM_setting_table));
		break;
	default:
		pr_info(" default mode_id\n");
			if (get_panel_es_ver() == ES_DV1)
				push_table(ctx, dsi_on_cmd_sdc120_sv1, sizeof(dsi_on_cmd_sdc120_sv1) / sizeof(struct LCM_setting_table));
			else if (get_panel_es_ver() == ES_DV2)
				push_table(ctx, dsi_on_cmd_sdc120_sv2, sizeof(dsi_on_cmd_sdc120_sv2) / sizeof(struct LCM_setting_table));
		break;
	}

	set_pwm_turbo_power_on(true);
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	if (oplus_adfr_is_support()) {
		// reset adfr auto mode status as auto mode will be change after power on
		oplus_adfr_status_reset(NULL, m);
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

	pr_info("%s,successful\n", __func__);
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
	int vrefresh_rate = 0;
	struct lcm *ctx = panel_to_lcm(panel);
	pr_err("[LCM]%s+\n", __func__);

	if (!ctx->prepared)
		return 0;

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	if (oplus_ofp_get_aod_state() == true) {
		if (vrefresh_rate == 60) {
			push_table(ctx, aod_off_cmd_60hz, sizeof(aod_off_cmd_60hz) / sizeof(struct LCM_setting_table));
		} else if (vrefresh_rate == 120) {
			push_table(ctx, aod_off_cmd_120hz, sizeof(aod_off_cmd_120hz) / sizeof(struct LCM_setting_table));
		} else {
			push_table(ctx, aod_off_cmd_90hz, sizeof(aod_off_cmd_90hz) / sizeof(struct LCM_setting_table));
		}
		usleep_range(9000, 9100);
		OFP_INFO("send aod off cmd\n");
	}

	usleep_range(5000, 5100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(25000, 25100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(125000, 125100);

	ctx->error = 0;
	ctx->prepared = false;
	pr_err("[LCM]%s-\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_err("[LCM]%s +\n", __func__);
	if (ctx->prepared)
		return 0;

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	pr_info("%s-\n", __func__);
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
	//sdc_120_mode
	{
		.clock = 446031,
		.hdisplay = 1264,
		.hsync_start = 1264 + 9,//HFP
		.hsync_end = 1264 + 9 + 2,//HSA
		.htotal = 1264 + 9 + 2 + 21,//HBP
		.vdisplay = 2780,
		.vsync_start = 2780 + 52,//VFP
		.vsync_end = 2780 + 52 + 14,//VSA
		.vtotal = 2780 + 52+ 14 + 22,//VBP
		.hskew = SDC_ADFR,
	},
	//sdc_60_mode
	{
		.clock = 223015,
		.hdisplay = 1264,
		.hsync_start = 1264 + 9,//HFP
		.hsync_end = 1264 + 9 + 2,//HSA
		.htotal = 1264 + 9 + 2 + 21,//HBP
		.vdisplay = 2780,
		.vsync_start = 2780 + 52,//VFP
		.vsync_end = 2780 + 52 + 14,//VSA
		.vtotal = 2780 + 52+ 14 + 22,//VBP
		.hskew = SDC_MFR,
	},
	//sdc_90_mode
	{
		.clock = 334523,
		.hdisplay = 1264,
		.hsync_start = 1264 + 9,//HFP
		.hsync_end = 1264 + 9 + 2,//HSA
		.htotal = 1264 + 9 + 2 + 21,//HBP
		.vdisplay = 2780,
		.vsync_start = 2780 + 52,//VFP
		.vsync_end = 2780 + 52 + 14,//VSA
		.vtotal = 2780 + 52+ 14 + 22,//VBP
		.hskew = SDC_ADFR,
	},
	//oa_120_mode
	{
		.clock = 446031,
		.hdisplay = 1264,
		.hsync_start = 1264 + 9,//HFP
		.hsync_end = 1264 + 9 + 2,//HSA
		.htotal = 1264 + 9 + 2 + 21,//HBP
		.vdisplay = 2780,
		.vsync_start = 2780 + 52,//VFP
		.vsync_end = 2780 + 52 + 14,//VSA
		.vtotal = 2780 + 52+ 14 + 22,//VBP
		.hskew = OPLUS_ADFR,
	},
	//vir_fhd_sdc_120_mode
	{
		.clock = FHD_CLK_120,
		.hdisplay = FHD_FRAME_WIDTH,
		.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,//HFP
		.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,//HSA
		.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//HBP
		.vdisplay = FHD_FRAME_HEIGHT,
		.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,//VFP
		.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,//VSA
		.vtotal = FHD_FRAME_HEIGHT + FHD_VFP+ FHD_VSA + FHD_VBP,//VBP
		.hskew = SDC_ADFR,
	},
	//vir_fhd_sdc_60_mode
	{
		.clock = FHD_CLK_60,
		.hdisplay = FHD_FRAME_WIDTH,
		.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,//HFP
		.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,//HSA
		.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//HBP
		.vdisplay = FHD_FRAME_HEIGHT,
		.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,//VFP
		.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,//VSA
		.vtotal = FHD_FRAME_HEIGHT + FHD_VFP+ FHD_VSA + FHD_VBP,//VBP
		.hskew = SDC_MFR,
	},
	//vir_fhd_sdc_90_mode
	{
		.clock = FHD_CLK_90,
		.hdisplay = FHD_FRAME_WIDTH,
		.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,//HFP
		.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,//HSA
		.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//HBP
		.vdisplay = FHD_FRAME_HEIGHT,
		.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,//VFP
		.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,//VSA
		.vtotal = FHD_FRAME_HEIGHT + FHD_VFP+ FHD_VSA + FHD_VBP,//VBP
		.hskew = SDC_ADFR,
	},
	//vir_fhd_oa_120_mode
	{
		.clock = FHD_CLK_120,
		.hdisplay = FHD_FRAME_WIDTH,
		.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,//HFP
		.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,//HSA
		.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//HBP
		.vdisplay = FHD_FRAME_HEIGHT,
		.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,//VFP
		.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,//VSA
		.vtotal = FHD_FRAME_HEIGHT + FHD_VFP+ FHD_VSA + FHD_VBP,//VBP
		.hskew = OPLUS_ADFR,
	},
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params[MODE_NUM] = {
	//fhd_sdc_120_mode
	{
	.pll_clk = 556,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	//.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9c,
        },
	.cmd_null_pkt_en = 0,
	.cmd_null_pkt_len = 0,
	.merge_trig_offset = 6500,  /* 250us, 1us = 26tick */
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_samsung_status = false,
	.color_loading_status = false,
	.color_2nit_status = true,
	.color_samsung_status = false,
	.color_nature_profession_status = true,
	.vendor = "A0004",
	.manufacture = "P_B",
	.panel_type = 3,
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
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
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
			.pic_height = 2780,
			.pic_width = 1264,
			.slice_height = 20,
			.slice_width = 632,
			.chunk_size = 632,
			.xmit_delay = 512,
			.dec_delay = 599,
			.scale_value = 32,
			.increment_interval = 504,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 1402,
			.slice_bpg_offset = 1103,
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
		},
	.data_rate = 1112,
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,

	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 0,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
		.apollo_limit_superior_us = 4900,
		.apollo_limit_inferior_us = 7700,
		.apollo_transfer_time_us = 6200,
		},
	.panel_bpp = 10,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	},

	//fhd_sdc_60_mode
	{
	.pll_clk = 556,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	//.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9c,
        },
	.cmd_null_pkt_en = 0,
	.cmd_null_pkt_len = 0,
	.merge_trig_offset = 15600,  /* 600us */
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_samsung_status = false,
	.color_loading_status = false,
	.color_2nit_status = true,
	.color_samsung_status = false,
	.color_nature_profession_status = true,
	.vendor = "A0004",
	.manufacture = "P_B",
	.panel_type = 3,
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
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
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
			.pic_height = 2780,
			.pic_width = 1264,
			.slice_height = 20,
			.slice_width = 632,
			.chunk_size = 632,
			.xmit_delay = 512,
			.dec_delay = 599,
			.scale_value = 32,
			.increment_interval = 504,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 1402,
			.slice_bpg_offset = 1103,
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
		},
	.data_rate = 1112,
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 0,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 2,
	.oplus_ofp_aod_off_black_frame_total_time = 59,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 60,
		.apollo_limit_superior_us = 10540,
		.apollo_limit_inferior_us = 14200,
		.apollo_transfer_time_us = 8200,
		},
	//.prefetch_offset = 466,
	.panel_bpp = 10,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	},

	//fhd 90hz
    {
	.pll_clk = 414,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	//.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9c,
        },
	.cmd_null_pkt_en = 0,
	.cmd_null_pkt_len = 0,
	.merge_trig_offset = 7800,  /* 300us */
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_samsung_status = false,
	.color_loading_status = false,
	.color_2nit_status = true,
	.color_samsung_status = false,
	.color_nature_profession_status = true,
	.vendor = "A0004",
	.manufacture = "P_B",
	.panel_type = 3,
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
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
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
			.pic_height = 2780,
			.pic_width = 1264,
			.slice_height = 20,
			.slice_width = 632,
			.chunk_size = 632,
			.xmit_delay = 512,
			.dec_delay = 599,
			.scale_value = 32,
			.increment_interval = 504,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 1402,
			.slice_bpg_offset = 1103,
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
		},
	.data_rate = 828,
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 0,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 45,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
		.apollo_limit_superior_us = 7880, .apollo_limit_inferior_us = 10000,
		.apollo_transfer_time_us = 8800,
		},
	//.prefetch_offset = 211,
	.panel_bpp = 10,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	},

	//fhd_oa_120_mode
	{
	.pll_clk = 556,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	//.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9c,
        },
	.cmd_null_pkt_en = 0,
	.cmd_null_pkt_len = 0,
	.merge_trig_offset = 6500,  /* 250us */
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_samsung_status = false,
	.color_loading_status = false,
	.color_2nit_status = true,
	.color_samsung_status = false,
	.color_nature_profession_status = true,
	.vendor = "A0004",
	.manufacture = "P_B",
	.panel_type = 3,
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
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
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
			.pic_height = 2780,
			.pic_width = 1264,
			.slice_height = 20,
			.slice_width = 632,
			.chunk_size = 632,
			.xmit_delay = 512,
			.dec_delay = 599,
			.scale_value = 32,
			.increment_interval = 504,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 1402,
			.slice_bpg_offset = 1103,
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
		},
	.data_rate = 1112,
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 0,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
		.apollo_limit_superior_us = 4900,
		.apollo_limit_inferior_us = 7700,
		.apollo_transfer_time_us = 6200,

		},
	.panel_bpp = 10,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	},
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

#ifdef OPLUS_FEATURE_DISPLAY_HPWM
void oplus_display_panel_set_frequency_pwm(void *dsi, dcs_write_gce cb, void *handle, unsigned int bl_lvl)
{
	unsigned int i = 0;
	int plus_bl = get_pwm_turbo_plus_bl();

	if (bl_lvl < plus_bl) {
		pr_info("pwm_turbo dsi_pwm_switch_ac_hstv backlight level=%d\n", bl_lvl);
		if (get_panel_es_ver() == ES_DV1) {
			for (i = 0; i < sizeof(dsi_pwm_switch_ac_hstv)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_pwm_switch_ac_hstv[i].para_list, dsi_pwm_switch_ac_hstv[i].count);
			}
		}
		set_pwm_turbo_switch_state(PWM_SWITCH_HPWM_STATE);
	} else if (bl_lvl >= plus_bl) {
		pr_info("pwm_turbo dsi_pwm_switch_dc_hstv backlight level=%d\n", bl_lvl);
		if (get_panel_es_ver() == ES_DV1) {
			for (i = 0; i < sizeof(dsi_pwm_switch_dc_hstv)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_pwm_switch_dc_hstv[i].para_list, dsi_pwm_switch_dc_hstv[i].count);
			}
		}
		set_pwm_turbo_switch_state(PWM_SWITCH_DC_STATE);
	}
}

static int oplus_display_panel_set_pwm_plus_bl(void *dsi, dcs_write_gce_pack cb, void *handle, unsigned int bl_lvl)
{
	int plus_bl = get_pwm_turbo_plus_bl();

	if (bl_lvl <= plus_bl) {
		pr_info("pwm_turbo dsi_pwm_switch_ac_hstv backlight level=%d\n", bl_lvl);
		if (get_panel_es_ver() == ES_DV1) {
			panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_ac_hstv,
				sizeof(dsi_pwm_switch_ac_hstv) / sizeof(struct LCM_setting_table), cb, handle);
		}
		set_pwm_turbo_switch_state(PWM_SWITCH_HPWM_STATE);
	} else if (bl_lvl > plus_bl) {
		pr_info("pwm_turbo dsi_pwm_switch_dc_hstv backlight level=%d\n", bl_lvl);
		if (get_panel_es_ver() == ES_DV1) {
			panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_dc_hstv,
				sizeof(dsi_pwm_switch_dc_hstv) / sizeof(struct LCM_setting_table), cb, handle);
		}
		set_pwm_turbo_switch_state(PWM_SWITCH_DC_STATE);
	}

	dsi_set_backlight[0].para_list[1] = bl_lvl >> 8;
	dsi_set_backlight[0].para_list[2] = bl_lvl & 0xFF;
	panel_send_pack_hs_cmd(dsi, dsi_set_backlight, sizeof(dsi_set_backlight) / sizeof(struct LCM_setting_table), cb, handle);
	last_backlight = bl_lvl;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &last_backlight);


	return 0;
}
#endif

static int oplus_display_panel_set_hstv(void *dsi, dcs_write_gce cb, void *handle, unsigned int bl_lvl)
{
	int i = 0;
	int plus_bl = get_pwm_turbo_plus_bl();

	if (bl_lvl < plus_bl) {
		pr_info("dsi_pwm_switch_ac_hstv backlight level=%d\n", bl_lvl);
		if (get_panel_es_ver() == ES_DV1) {
			for (i = 0; i < sizeof(dsi_pwm_switch_ac_hstv)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_pwm_switch_ac_hstv[i].para_list, dsi_pwm_switch_ac_hstv[i].count);
			}
		}
	} else if (bl_lvl >= plus_bl) {
		pr_info("dsi_pwm_switch_dc_hstv backlight level=%d\n", bl_lvl);
		if (get_panel_es_ver() == ES_DV1) {
			for (i = 0; i < sizeof(dsi_pwm_switch_dc_hstv)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_pwm_switch_dc_hstv[i].para_list, dsi_pwm_switch_dc_hstv[i].count);
			}
		}
	}
	dsi_set_backlight[0].para_list[1] = bl_lvl >> 8;
	dsi_set_backlight[0].para_list[2] = bl_lvl & 0xFF;
	for (i = 0; i < sizeof(dsi_set_backlight)/sizeof(struct LCM_setting_table); i++) {
		cb(dsi, handle, dsi_set_backlight[i].para_list, dsi_set_backlight[i].count);
	}
	last_backlight = bl_lvl;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &last_backlight);

	return 0;
}


static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (last_backlight == 0 || level == 0) {
		DISP_INFO("[INFO][%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	} else {
		DISP_BACKLIGHT("[INFO][%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	}

	last_backlight = level;
	mapped_level = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);

	if (level == 1) {
		pr_info("[DISP][INFO][%s:%d]filter backlight %u setting\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = 2047;
	}

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	oplus_display_brightness = level;

	return 0;
}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	unsigned int i = 0;

	OFP_DEBUG("start\n");

	if (!dsi || !cb) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd_60hz)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_on_cmd_60hz[i].para_list, hbm_on_cmd_60hz[i].count);
		}
	} else if (hbm_mode == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd_60hz)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_off_cmd_60hz[i].para_list, hbm_off_cmd_60hz[i].count);
		}
		if (oplus_panel_pwm_turbo_is_enabled()) {
			oplus_display_panel_set_frequency_pwm(dsi, cb, handle, oplus_display_brightness);
		}
		oplus_display_panel_set_hstv(dsi, cb, handle, oplus_display_brightness);
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}

	lcdinfo_notify(1, &hbm_mode);

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	unsigned int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *hbm_cmd = NULL;

	OFP_DEBUG("start\n");

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	OFP_INFO("hbm_en:%u,bl_lvl:%u,refresh_rate:%u\n", en, oplus_display_brightness, vrefresh_rate);

	if (vrefresh_rate == 60) {
		if (en) {
			hbm_cmd = hbm_on_cmd_60hz;
			reg_count = sizeof(hbm_on_cmd_60hz) / sizeof(struct LCM_setting_table);
		} else {
			hbm_cmd = hbm_off_cmd_60hz;
			reg_count = sizeof(hbm_off_cmd_60hz) / sizeof(struct LCM_setting_table);
		}
	} else if (vrefresh_rate == 90) {
		if (en) {
			hbm_cmd = hbm_on_cmd_90hz;
			reg_count = sizeof(hbm_on_cmd_90hz) / sizeof(struct LCM_setting_table);
		} else {
			hbm_cmd = hbm_off_cmd_90hz;
			reg_count = sizeof(hbm_off_cmd_90hz) / sizeof(struct LCM_setting_table);
		}
	} else if (vrefresh_rate == 120) {
		if (en) {
			hbm_cmd = hbm_on_cmd_120hz;
			reg_count = sizeof(hbm_on_cmd_120hz) / sizeof(struct LCM_setting_table);
		} else {
			hbm_cmd = hbm_off_cmd_120hz;
			reg_count = sizeof(hbm_off_cmd_120hz) / sizeof(struct LCM_setting_table);
		}
	}

	for (i = 0; i < reg_count; i++) {
		unsigned int cmd;
		cmd = hbm_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (!handle) {
					usleep_range(hbm_cmd[i].count * 1000, hbm_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(hbm_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (!handle) {
					usleep_range(hbm_cmd[i].count, hbm_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(hbm_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, hbm_cmd[i].para_list, hbm_cmd[i].count);
		}
	}

	if (!en) {
		if (oplus_panel_pwm_turbo_is_enabled()) {
			oplus_display_panel_set_frequency_pwm(dsi, cb, handle, oplus_display_brightness);
		}
		oplus_display_panel_set_hstv(dsi, cb, handle, oplus_display_brightness);
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}

	lcdinfo_notify(1, &en);

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *aod_off_cmd = NULL;

	if (!panel || !dsi) {
		OFP_ERR("Invalid dsi params\n");
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		pr_info("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}
	if (vrefresh_rate == 60) {
		aod_off_cmd = aod_off_cmd_60hz;
		reg_count = sizeof(aod_off_cmd_60hz) / sizeof(struct LCM_setting_table);
	} else if (vrefresh_rate == 120) {
		aod_off_cmd = aod_off_cmd_120hz;
		reg_count = sizeof(aod_off_cmd_120hz) / sizeof(struct LCM_setting_table);
	} else {
		aod_off_cmd = aod_off_cmd_90hz;
		reg_count = sizeof(aod_off_cmd_90hz) / sizeof(struct LCM_setting_table);
	}
	for (i = 0; i < reg_count; i++) {
		unsigned int cmd;
		cmd = aod_off_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count * 1000, aod_off_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count, aod_off_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_off_cmd[i].para_list, aod_off_cmd[i].count);
		}
	}

	oplus_display_panel_set_hstv(dsi, cb, handle, oplus_display_brightness);
	lcm_setbacklight_cmdq(dsi, cb, handle, last_backlight);

	OFP_INFO("send aod off cmd\n");

	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;

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

	OFP_INFO("send aod on cmd\n");

	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_mode[i].para_list, aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_mode[i].para_list, aod_low_mode[i].count);
		}
	}
	OFP_INFO("level = %d\n", level);

	return 0;
}

static int panel_set_ultra_low_power_aod(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *ultra_low_power_aod_cmd = NULL;

	if (!panel || !dsi) {
		OFP_ERR("Invalid dsi params\n");
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		pr_info("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}
	if (vrefresh_rate == 60) {
		if (level == 1) {
			ultra_low_power_aod_cmd = ultra_low_power_aod_on_cmd_60hz;
			reg_count = sizeof(ultra_low_power_aod_on_cmd_60hz) / sizeof(struct LCM_setting_table);
		} else {
			ultra_low_power_aod_cmd = ultra_low_power_aod_off_cmd_60hz;
			reg_count = sizeof(ultra_low_power_aod_off_cmd_60hz) / sizeof(struct LCM_setting_table);
		}
	} else if (vrefresh_rate == 120) {
		if (level == 1) {
			ultra_low_power_aod_cmd = ultra_low_power_aod_on_cmd_120hz;
			reg_count = sizeof(ultra_low_power_aod_on_cmd_120hz) / sizeof(struct LCM_setting_table);
		} else {
			ultra_low_power_aod_cmd = ultra_low_power_aod_off_cmd_120hz;
			reg_count = sizeof(ultra_low_power_aod_off_cmd_120hz) / sizeof(struct LCM_setting_table);
		}
	} else {
		if (level == 1) {
			ultra_low_power_aod_cmd = ultra_low_power_aod_on_cmd_90hz;
			reg_count = sizeof(ultra_low_power_aod_on_cmd_90hz) / sizeof(struct LCM_setting_table);
		} else {
			ultra_low_power_aod_cmd = ultra_low_power_aod_off_cmd_90hz;
			reg_count = sizeof(ultra_low_power_aod_off_cmd_90hz) / sizeof(struct LCM_setting_table);
		}
	}
	for (i = 0; i < reg_count; i++) {
		unsigned int cmd;
		cmd = ultra_low_power_aod_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(ultra_low_power_aod_cmd[i].count * 1000, ultra_low_power_aod_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(ultra_low_power_aod_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(ultra_low_power_aod_cmd[i].count, ultra_low_power_aod_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(ultra_low_power_aod_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, ultra_low_power_aod_cmd[i].para_list, ultra_low_power_aod_cmd[i].count);
		}
	}

	OFP_INFO("level = %d\n", level);

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

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_err("[LCM]debug %s, ctx->prepared %d\n", __func__, ctx->prepared);

	lcm_panel_vrfio18_aif_enable(ctx->dev);
	usleep_range(5000, 5010);
	ctx->vddr1p2_enable_gpio = devm_gpiod_get(ctx->dev, "pm-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	usleep_range(5000, 5010);
	lcm_panel_vmc_ldo_enable(ctx->dev);
	usleep_range(21000, 21100);


	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(1000, 1100);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_err("[LCM]debug lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);
	usleep_range(10000, 10100);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);
	/* set vddi 3.0v */
	lcm_panel_vmc_ldo_disable(ctx->dev);
	usleep_range(10000, 10100);;
	ctx->vddr1p2_enable_gpio = devm_gpiod_get(ctx->dev, "pm-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	usleep_range(10000, 10100);;
	lcm_panel_vrfio18_aif_disable(ctx->dev);
	usleep_range(2000, 2100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(70000, 70100);
	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared)
               return 0;

	pr_info("[LCM]debug for lcd reset :%s, ctx->prepared:%d\n", __func__, ctx->prepared);

	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(3000, 3100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(25000, 25100);

	return 0;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	mode = MODE_MAPPING_RULE(mode);

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
		*ext_param = &ext_params[1];
	}
	else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
		*ext_param = &ext_params[0];
	} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
		*ext_param = &ext_params[2];
	} else if (m_vrefresh == 120 && m->hskew == OPLUS_ADFR) {
		*ext_param = &ext_params[3];
	} else {
		*ext_param = &ext_params[0];
	}

	if (*ext_param)
		pr_debug("[LCM] data_rate:%d\n", (*ext_param)->data_rate);
	else
		pr_err("[LCM] ext_param is NULL;\n");

	return ret;
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
	pr_err("%s, mode=%d, vrefresh=%d, hskew=%d\n", __func__, mode, drm_mode_vrefresh(m), m->hskew);

	if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
		ext->params = &ext_params[1];
	} else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
		ext->params = &ext_params[0];
	} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
		ext->params = &ext_params[2];
	} else if (m_vrefresh == 120 && m->hskew == OPLUS_ADFR) {
		ext->params = &ext_params[3];
	} else {
		ext->params = &ext_params[0];
	}

	return ret;
}

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle)
{
	unsigned int i = 0;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
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

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
static int panel_minfps_check(int mode_id, int extend_frame)
{
	if (mode_id == FHD_SDC90) {
		if (extend_frame < OPLUS_ADFR_MANAUL_MIN_FPS_MAX || extend_frame > OPLUS_ADFR_MANAUL_MIN_FPS90_1HZ)
			extend_frame = OPLUS_ADFR_MANAUL_MIN_FPS_MAX;
	} else if (mode_id == FHD_SDC60) {
		if (extend_frame < OPLUS_ADFR_MANAUL_MIN_FPS_60HZ || extend_frame > OPLUS_ADFR_MANAUL_MIN_FPS_1HZ)
			extend_frame = OPLUS_ADFR_MANAUL_MIN_FPS_60HZ;
	} else {
		if (extend_frame < OPLUS_ADFR_MANAUL_MIN_FPS_MAX || extend_frame > OPLUS_ADFR_MANAUL_MIN_FPS_1HZ)
			extend_frame = OPLUS_ADFR_MANAUL_MIN_FPS_MAX;
	}
	return extend_frame;
}

static int panel_set_minfps(void *dsi, struct drm_panel *panel, dcs_write_gce_pack cb, void *handle,
	void *minfps, struct drm_display_mode *m)
{
	unsigned int mode_id = 0;
	unsigned int vrefresh_rate = 0;
	unsigned int ext_frame = 0;
	unsigned int lcm_cmd_count = 0;
	struct oplus_minfps *min_fps = (struct oplus_minfps *)minfps;

	if (!dsi || !cb || !minfps || !m) {
		ADFR_ERR("Invalid params\n");
		return -EINVAL;
	}

	mode_id = get_mode_enum(m);
	vrefresh_rate = drm_mode_vrefresh(m);
	ADFR_INFO("mode_id:%u,refresh_rate:%u,minfps_flag:%u,extern_frame:%u\n",
				mode_id, vrefresh_rate, min_fps->minfps_flag, min_fps->extend_frame);

	/*update min fps cmd */
	if (!min_fps->minfps_flag) {
		/* update manual min fps */
		ext_frame = panel_minfps_check(mode_id, min_fps->extend_frame);
		switch (vrefresh_rate) {
		case 120:
			auto_off_minfps_cmd_120hz[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
			lcm_cmd_count = sizeof(auto_off_minfps_cmd_120hz) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_off_minfps_cmd_120hz, ext_frame:%u\n", ext_frame);
			panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_120hz, lcm_cmd_count, cb, handle);
			break;
		case 90:
			auto_off_minfps_cmd_90hz[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
			lcm_cmd_count = sizeof(auto_off_minfps_cmd_90hz) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_off_minfps_cmd_90hz, ext_frame:%u\n", ext_frame);
			panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_90hz, lcm_cmd_count, cb, handle);
			break;
		case 60:
			auto_off_minfps_cmd_60hz[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = (ext_frame+1)/2 -1;
			lcm_cmd_count = sizeof(auto_off_minfps_cmd_60hz) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_off_minfps_cmd_60hz, ext_frame:%u\n", ext_frame);
			panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_60hz, lcm_cmd_count, cb, handle);
			break;
		default:
			break;
		}
	}

	return 0;
}

static int panel_set_multite(void *dsi, struct drm_panel *panel, dcs_write_gce_pack cb, void *handle, bool enable)
{
	unsigned int lcm_cmd_count = 0;

	/*enable or disable multi-te cmds */
	if (enable) {
		ADFR_INFO("multite enabled\n");
		/* enable multi TE */
		lcm_cmd_count = sizeof(multi_te_enable) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, multi_te_enable, lcm_cmd_count, cb, handle);
	} else {
		ADFR_INFO("multite disabled\n");
		/* disable multi TE */
		lcm_cmd_count = sizeof(multi_te_disable) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, multi_te_disable, lcm_cmd_count, cb, handle);
	}

	return 0;
}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

static int oplus_update_time(void)
{
	mode_switch_begin_time = ktime_get();
	return 0;
}

static unsigned int get_mode_switch_delay(void)
{
	ktime_t time_gap = 0;
	unsigned int sleep_time = 0;
	time_gap = ktime_to_us(ktime_sub(ktime_get(), mode_switch_begin_time));
	DISP_DEBUG("time_gap: %lld, mode_switch_begin_time: %lld\n", time_gap, mode_switch_begin_time);
	if (time_gap > 8300)
		time_gap = 8300;
	sleep_time = (unsigned int)(8300 - time_gap);
	DISP_DEBUG("sleep_time =  %d\n", sleep_time);
	return sleep_time;
}

static int dsi_pwm_switch_cmd_update(struct drm_display_mode *m)
{
	int vrefresh = -1;
	int plus_bl = 0x388;
	if (!m)
		return 0;
	vrefresh = drm_mode_vrefresh(m);


	if (vrefresh == 60) {
		if (oplus_display_brightness < plus_bl) {
			pr_info("sdc60 switch to dsi_pwm_switch_ac_hstv\n");
			timing_switch_cmd_sdc60_sv1[18].para_list[1] = 0x00;
			timing_switch_cmd_sdc60_sv1[19].para_list[1] = 0x0B;
			timing_switch_cmd_sdc60_sv1[20].para_list[1] = 0x4A;
			timing_switch_cmd_sdc60_sv1[21].para_list[1] = 0xBB;
			timing_switch_cmd_sdc60_sv1[21].para_list[2] = 0x2D;
			timing_switch_cmd_sdc60_sv1[21].para_list[3] = 0x2D;
			timing_switch_cmd_sdc60_sv1[22].para_list[1] = 0x00;
			timing_switch_cmd_sdc60_sv1[22].para_list[2] = 0x04;
			timing_switch_cmd_sdc60_sv1[22].para_list[3] = 0x04;
		} else if (oplus_display_brightness >= plus_bl) {
			pr_info("sdc60 switch to dsi_pwm_switch_dc_hstv\n");
			timing_switch_cmd_sdc60_sv1[18].para_list[1] = 0x10;
			timing_switch_cmd_sdc60_sv1[19].para_list[1] = 0x00;
			timing_switch_cmd_sdc60_sv1[20].para_list[1] = 0x28;
			timing_switch_cmd_sdc60_sv1[21].para_list[1] = 0x00;
			timing_switch_cmd_sdc60_sv1[21].para_list[2] = 0x27;
			timing_switch_cmd_sdc60_sv1[21].para_list[3] = 0x27;
			timing_switch_cmd_sdc60_sv1[22].para_list[1] = 0x00;
			timing_switch_cmd_sdc60_sv1[22].para_list[2] = 0x04;
			timing_switch_cmd_sdc60_sv1[22].para_list[3] = 0x00;
		}
	} else if (vrefresh == 90) {
		if (oplus_display_brightness < plus_bl) {
			pr_info("sdc90 switch to dsi_pwm_switch_ac_hstv\n");
			timing_switch_cmd_sdc90_sv1[18].para_list[1] = 0x00;
			timing_switch_cmd_sdc90_sv1[19].para_list[1] = 0x00;
			timing_switch_cmd_sdc90_sv1[20].para_list[1] = 0x28;
			timing_switch_cmd_sdc90_sv1[21].para_list[1] = 0x00;
			timing_switch_cmd_sdc90_sv1[21].para_list[2] = 0x27;
			timing_switch_cmd_sdc90_sv1[21].para_list[3] = 0x27;
			timing_switch_cmd_sdc90_sv1[22].para_list[1] = 0x00;
			timing_switch_cmd_sdc90_sv1[22].para_list[2] = 0x04;
			timing_switch_cmd_sdc90_sv1[22].para_list[3] = 0x00;
		} else if (oplus_display_brightness >= plus_bl) {
			pr_info("sdc90 switch to dsi_pwm_switch_dc_hstv\n");
			timing_switch_cmd_sdc90_sv1[18].para_list[1] = 0x10;
			timing_switch_cmd_sdc90_sv1[19].para_list[1] = 0x00;
			timing_switch_cmd_sdc90_sv1[20].para_list[1] = 0x28;
			timing_switch_cmd_sdc90_sv1[21].para_list[1] = 0x00;
			timing_switch_cmd_sdc90_sv1[21].para_list[2] = 0x27;
			timing_switch_cmd_sdc90_sv1[21].para_list[3] = 0x27;
			timing_switch_cmd_sdc90_sv1[22].para_list[1] = 0x00;
			timing_switch_cmd_sdc90_sv1[22].para_list[2] = 0x04;
			timing_switch_cmd_sdc90_sv1[22].para_list[3] = 0x00;
		}
	} else if (vrefresh == 120 && (m->hskew == SDC_MFR || m->hskew == SDC_ADFR)) {
		if (oplus_display_brightness < plus_bl) {
			pr_info("sdc120 switch to dsi_pwm_switch_ac_hstv\n");
			timing_switch_cmd_sdc120_sv1[18].para_list[1] = 0x00;
			timing_switch_cmd_sdc120_sv1[19].para_list[1] = 0x00;
			timing_switch_cmd_sdc120_sv1[20].para_list[1] = 0x28;
			timing_switch_cmd_sdc120_sv1[21].para_list[1] = 0x00;
			timing_switch_cmd_sdc120_sv1[21].para_list[2] = 0x27;
			timing_switch_cmd_sdc120_sv1[21].para_list[3] = 0x27;
			timing_switch_cmd_sdc120_sv1[22].para_list[1] = 0x00;
			timing_switch_cmd_sdc120_sv1[22].para_list[2] = 0x04;
			timing_switch_cmd_sdc120_sv1[22].para_list[3] = 0x00;
		} else if (oplus_display_brightness >= plus_bl) {
			pr_info("sdc120 switch to dsi_pwm_switch_dc_hstv\n");
			timing_switch_cmd_sdc120_sv1[18].para_list[1] = 0x10;
			timing_switch_cmd_sdc120_sv1[19].para_list[1] = 0x00;
			timing_switch_cmd_sdc120_sv1[20].para_list[1] = 0x28;
			timing_switch_cmd_sdc120_sv1[21].para_list[1] = 0x00;
			timing_switch_cmd_sdc120_sv1[21].para_list[2] = 0x27;
			timing_switch_cmd_sdc120_sv1[21].para_list[3] = 0x27;
			timing_switch_cmd_sdc120_sv1[22].para_list[1] = 0x00;
			timing_switch_cmd_sdc120_sv1[22].para_list[2] = 0x04;
			timing_switch_cmd_sdc120_sv1[22].para_list[3] = 0x00;
		}
	} else if (vrefresh == 120 && (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR)) {
		if (oplus_display_brightness < plus_bl) {
			pr_info("oa120 switch to dsi_pwm_switch_ac_hstv\n");
			timing_switch_cmd_oa120_sv1[18].para_list[1] = 0x00;
			timing_switch_cmd_oa120_sv1[19].para_list[1] = 0x00;
			timing_switch_cmd_oa120_sv1[20].para_list[1] = 0x28;
			timing_switch_cmd_oa120_sv1[21].para_list[1] = 0x00;
			timing_switch_cmd_oa120_sv1[21].para_list[2] = 0x27;
			timing_switch_cmd_oa120_sv1[21].para_list[3] = 0x27;
			timing_switch_cmd_oa120_sv1[22].para_list[1] = 0x00;
			timing_switch_cmd_oa120_sv1[22].para_list[2] = 0x04;
			timing_switch_cmd_oa120_sv1[22].para_list[3] = 0x00;
		} else if (oplus_display_brightness >= plus_bl) {
			pr_info("oa120 switch to dsi_pwm_switch_dc_hstv\n");
			timing_switch_cmd_oa120_sv1[18].para_list[1] = 0x10;
			timing_switch_cmd_oa120_sv1[19].para_list[1] = 0x00;
			timing_switch_cmd_oa120_sv1[20].para_list[1] = 0x28;
			timing_switch_cmd_oa120_sv1[21].para_list[1] = 0x00;
			timing_switch_cmd_oa120_sv1[21].para_list[2] = 0x27;
			timing_switch_cmd_oa120_sv1[21].para_list[3] = 0x27;
			timing_switch_cmd_oa120_sv1[22].para_list[1] = 0x00;
			timing_switch_cmd_oa120_sv1[22].para_list[2] = 0x04;
			timing_switch_cmd_oa120_sv1[22].para_list[3] = 0x00;
		}
	}
	return 0;
}

static int mode_switch_hs(struct drm_panel *panel, struct drm_connector *connector,
		void *dsi_drv, unsigned int cur_mode, unsigned int dst_mode,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage, dcs_write_gce_pack cb)
{
	int ret = 0;
	int m_vrefresh = 0;
	int src_vrefresh = 0;
	unsigned int lcm_cmd_count = 0;
	/* lk mipi setting is 830 */
	static int last_data_rate = 900;
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct drm_display_mode *src_m = get_mode_by_id(connector, cur_mode);
	struct lcm *ctx = panel_to_lcm(panel);
	unsigned int sleep_time = 0;

	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	dsi_pwm_switch_cmd_update(m);

	if (stage == BEFORE_DSI_POWERDOWN) {
		m_vrefresh = drm_mode_vrefresh(m);
		src_vrefresh = drm_mode_vrefresh(src_m);
		pr_info("mode_switch_hs,cur_mode:%d,dst_mode:%d,hdisplay:%d->%d,vrefresh:%d->%d,hskew:%d->%d\n",
			cur_mode, dst_mode, src_m->hdisplay, m->hdisplay, src_vrefresh, m_vrefresh, src_m->hskew, m->hskew);

		if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
			if (m_vrefresh == 60) {
				pr_info("timing switch to sdc60\n");
				if (get_panel_es_ver() == ES_DV1) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60_sv1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60_sv1, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() == ES_DV2) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60_sv2) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60_sv2, lcm_cmd_count, cb, NULL);
				}
			} else if ((src_vrefresh == 60) && (m_vrefresh == 90)) {
				pr_info("timing switch from sdc60 to sdc90, return 1\n");
				ret = 1;
			} else if ((src_vrefresh == 120) && (m_vrefresh == 90)) {
				pr_info("timing switch from sdc120 to sdc90\n");
				if (get_panel_es_ver() == ES_DV1) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_sv1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_sv1, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() == ES_DV2) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_sv2) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_sv2, lcm_cmd_count, cb, NULL);
				}
				sleep_time = get_mode_switch_delay();
				usleep_range(sleep_time, (sleep_time + 100));
			} else if (m_vrefresh == 120) {
				pr_info("timing switch to sdc120 return 1\n");
				ret = 1;
			}
			//oplus_adfr_set_multite_state(false);
		} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
			if (m_vrefresh == 120) {
				/* send OA120 timing-switch cmd */
				pr_info("timing switch to oa120, return 1\n");
				ret = 1;
			}
		}

		//if (oplus_adfr_is_support()) {
			// reset adfr auto mode status as panel mode will be change after timing switch
			//oplus_adfr_status_reset(src_m, m);
		//}
	} else if (stage == AFTER_DSI_POWERON) {
		ctx->m = m;
		m_vrefresh = drm_mode_vrefresh(m);
		src_vrefresh = drm_mode_vrefresh(src_m);
		pr_info("mode_switch_hs,AFTER_DSI_POWERON,cur_mode:%d,dst_mode:%d,hdisplay:%d->%d,vrefresh:%d->%d,hskew:%d->%d\n",
			cur_mode, dst_mode, src_m->hdisplay, m->hdisplay, src_vrefresh, m_vrefresh, src_m->hskew, m->hskew);
		if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
			if ((src_vrefresh == 60) && (m_vrefresh == 90)) {
				pr_info("timing switch from sdc60 to sdc90\n");
					if (get_panel_es_ver() == ES_DV1) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_sv1) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_sv1, lcm_cmd_count, cb, NULL);
					} else if (get_panel_es_ver() == ES_DV2) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_sv2) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_sv2, lcm_cmd_count, cb, NULL);
					}
			} else if (m_vrefresh == 120) {
				pr_info("timing switch to sdc120\n");
				if (get_panel_es_ver() == ES_DV1) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120_sv1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120_sv1, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() == ES_DV2) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120_sv2) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120_sv2, lcm_cmd_count, cb, NULL);
				}
			}
			//oplus_adfr_set_multite_state(false);
		} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
			if (m_vrefresh == 120) {
				/* send OA120 timing-switch cmd */
				pr_info("timing switch to oa120\n");
				if (get_panel_es_ver() == ES_DV1) {
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120_sv1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120_sv1, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() == ES_DV2) {
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120_sv2) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120_sv2, lcm_cmd_count, cb, NULL);
				}
			}
		}
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
		if (oplus_adfr_is_support()) {
			// reset adfr auto mode status as panel mode will be change after timing switch
			oplus_adfr_status_reset(src_m, m);
		}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
	}

	panel_mode_id = src_vrefresh;

	if (ext->params->data_rate != last_data_rate) {
		ret = 1;
		pr_info("need to change mipi clk, data_rate=%d,last_data_rate=%d\n",ext->params->data_rate,last_data_rate);
		last_data_rate = ext->params->data_rate;
	}

	return ret;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 1) {
		pr_info("[DISP][INFO][%s:%d]filter backlight %u setting\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	pr_info("[DISP][INFO][%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.panel_reset = lcm_panel_reset,
	.ata_check = panel_ata_check,
	.ext_param_get = mtk_panel_ext_param_get,
	.ext_param_set = mtk_panel_ext_param_set,
	.get_res_switch_type = mtk_get_res_switch_type,
	.scaling_mode_mapping = mtk_scaling_mode_mapping,
	.mode_switch_hs = mode_switch_hs,
	.update_time = oplus_update_time,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	.set_minfps = panel_set_minfps,
	.set_multite = panel_set_multite,
#endif
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.set_hbm = lcm_set_hbm,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.set_ultra_low_power_aod = panel_set_ultra_low_power_aod,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
#ifdef OPLUS_FEATURE_DISPLAY_HPWM
	.lcm_high_pwm_set_plus_bl = oplus_display_panel_set_pwm_plus_bl,
#endif /* OPLUS_FEATURE_DISPLAY_HPWM */
};
#endif

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode[MODE_NUM * RES_NUM];
	int i = 0;

	mode[0] = drm_mode_duplicate(connector->dev, &display_mode[0]);
	if (!mode[0]) {
		pr_err("failed to add mode %ux%ux@%u\n",
			display_mode[0].hdisplay, display_mode[0].vdisplay,
			 drm_mode_vrefresh(&display_mode[0]));
		return -ENOMEM;
	}

	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[0]);

	for (i = 1; i < MODE_NUM * RES_NUM; i++) {
		mode[i] = drm_mode_duplicate(connector->dev, &display_mode[i]);
		if (!mode[i]) {
			pr_info("not enough memory\n");
			return -ENOMEM;
		}

		drm_mode_set_name(mode[i]);
		mode[i]->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode[i]);
	}

	connector->display_info.width_mm = PHYSICAL_WIDTH / 1000;
	connector->display_info.height_mm = PHYSICAL_HEIGHT / 1000;

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
	unsigned int res_switch;
	int ret;
	int rc = 0;
	u32 config = 0;

	pr_info("[LCM] ac094_p_b_a0004 %s START\n", __func__);

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
	pr_info("remote_node name:%s, dev->of_node name:%s\n", remote_node->name, dev->of_node->name);
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);
	pr_err("[LCM]%s %d\n",__func__,__LINE__);
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_u32(dev->of_node, "res-switch", &res_switch);
	if (ret < 0)
		res_switch = 0;
	else
		res_switch_type = (enum RES_SWITCH_TYPE)res_switch;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	lcm_panel_vrfio18_aif_enable(ctx->dev);
	usleep_range(5000, 5100);
	ctx->vddr1p2_enable_gpio = devm_gpiod_get(dev, "pm-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vddr1p2_enable_gpio);

	usleep_range(5000, 5100);

	lcm_panel_vmc_ldo_enable(ctx->dev);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params[0], &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	rc = of_property_read_u32(dev->of_node, "oplus-adfr-config", &config);
	if (rc == 0) {
		oplus_adfr_config = config;
		pr_info("config=%d, adfrconfig=%d\n", config, oplus_adfr_config);
	} else {
		oplus_adfr_config = 0;
		pr_info("adfrconfig=%d\n", oplus_adfr_config);
	}

	oplus_serial_number_probe(dev);
	oplus_pwm_turbo_probe(dev);

	register_device_proc("lcd", "A0004", "P_B");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	oplus_enhance_mipi_strength = 2;
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	oplus_adfr_get_test_te_gpio(dev);
#endif
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	oplus_ofp_init(dev);
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	pr_info("[LCM] %s- lcm, AC094_P_B_A0004, END\n", __func__);

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
	    .compatible = "ac094,p,b,a0004,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel_ac094_p_b_a0004_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init ac094_p_b_a0004_drv_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register ac094_p_b_a0004_drv_init driver: %d\n",
			__func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit ac094_p_b_a0004_drv_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}
module_init(ac094_p_b_a0004_drv_init);
module_exit(ac094_p_b_a0004_drv_exit);


MODULE_AUTHOR("LiKe");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
