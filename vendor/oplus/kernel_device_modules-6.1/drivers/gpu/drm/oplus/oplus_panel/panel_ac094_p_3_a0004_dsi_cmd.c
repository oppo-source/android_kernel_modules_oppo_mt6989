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
#include"../../mediatek/mediatek_v2/mtk_dsi.h"

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "panel_ac094_p_3_a0004_dsi_cmd.h"

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

#define oplus_disp_trace_begin(fmt, args...) do { \
	if (g_trace_log) { \
		mtk_drm_print_trace("B|%d|"fmt"\n", current->tgid, ##args); \
	} \
} while (0)

#define oplus_disp_trace_end(fmt, args...) do { \
	if (g_trace_log) { \
		mtk_drm_print_trace("E|%d|"fmt"\n", current->tgid, ##args); \
	} \
} while (0)

#define oplus_disp_trace_c(fmt, args...) do { \
	if (g_trace_log) { \
		mtk_drm_print_trace("C|"fmt"\n", ##args); \
	} \
} while (0)

static enum RES_SWITCH_TYPE res_switch_type = RES_SWITCH_NO_USE;

bool aod_off_swtich_pulse = false;
ktime_t aod_off_timestamp;

static struct regulator *vmc_ldo;
static struct regulator *vrfio18_aif;
static unsigned int panel_mode_id = FHD_SDC120;    // default 120fps
static unsigned int temp_seed_mode = 0;
static unsigned int motify_once_gamma_oa120 = 1;
static unsigned int motify_once_gamma_120 = 1;
static unsigned int motify_once_gamma_90 = 1;
static unsigned int motify_once_gamma_60 = 1;
static unsigned int panel_last_brightness = 0;
unsigned int gamma93_rgb[3];
unsigned int gamma94_rgb[3];
unsigned int rgb_data[3];
unsigned int gamma95_120[5];
unsigned int gamma96_120[5];
unsigned int gamma97_120[5];
unsigned int gamma95_90[5];
unsigned int gamma96_90[5];
unsigned int gamma97_90[5];

extern unsigned int last_backlight;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern unsigned int oplus_enhance_mipi_strength;
extern unsigned int m_db;
extern unsigned int m_dc;
extern atomic_t oplus_pcp_handle_lock;
extern atomic_t oplus_pcp_num;
extern int g_last_mode_idx;
extern struct mutex oplus_pcp_lock;
extern bool pulse_flg;
extern unsigned int silence_mode;
extern unsigned char gamma93_120[5];
extern unsigned char gamma94_120[5];
extern unsigned char gamma93_90[5];
extern unsigned char gamma94_90[5];
extern atomic_t esd_pending;
extern bool ofp_aod_off_swtich_pulse;

extern unsigned int g_trace_log;
extern void mtk_drm_print_trace(char *fmt, ...);

extern void lcdinfo_notify(unsigned long val, void *v);
extern void oplus_pcp_handle(bool cmd_is_pcp,  void *handle);
extern int oplus_serial_number_probe(struct device *dev);

extern inline void set_pwm_turbo_switch_state(enum PWM_SWITCH_STATE state);
extern void set_pwm_turbo_power_on(bool en);
extern u16 mtk_get_gpr(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle);
extern struct oplus_pwm_turbo_params *oplus_pwm_turbo_get_params(void);
extern inline bool oplus_panel_pwm_onepulse_is_enabled(void);
extern int oplus_display_panel_dbv_probe(struct device *dev);

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle);
void oplus_display_panel_pwm_switch_prepare(unsigned int level);
static int oplus_display_panel_set_pwm_pul_lp(void *dsi, dcs_write_gce cb, void *handle, unsigned int mode);

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
	ES_DV2 = 2,
	ES_DV3 = 3,
	ES_DV4 = 4,
	ES_DV5 = 5,
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
	if (m_db > 0 && m_db <=2)
		ret = ES_DV2;
	else if (m_db == 3)
		ret = ES_DV3;
	else if (m_db >= 4) {
		if (m_db == 5 && m_dc == 0) {
			ret = ES_DV5;
		} else {
			ret = ES_DV4;
		}
	}
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

bool oplus_display_is_pcp(struct LCM_setting_table *table, unsigned int lcm_cmd_count)
{
	unsigned int i;
	struct LCM_setting_table *tb = table;

	for (i = 0; i < lcm_cmd_count; i++) {
		if (tb[i].count == 2) {
			if (tb[i].para_list[0] == 0x88 && tb[i].para_list[1] == 0x78) {
				return true;
			}
		}
	}

	return false;
}

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	unsigned int count;
	int i;
	unsigned char val;
	bool en = oplus_panel_pwm_onepulse_is_enabled();
	struct drm_display_mode *m = ctx->m;
	mode_id = get_mode_enum(m);
	pr_info("%s panel_es_ver=%d, mode id=%d\n", __func__, get_panel_es_ver(), mode_id);

	switch (mode_id) {
	case FHD_SDC60:
		pr_info("fhd_dsi_on_cmd_sdc60\n");
		if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc60_dv2, sizeof(dsi_on_cmd_sdc60_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_sdc60_dv3, sizeof(dsi_on_cmd_sdc60_dv3) / sizeof(struct LCM_setting_table));
		else if (m_db == 7 && m_dc == 0) {
			count = sizeof(dsi_on_cmd_sdc60_dvgamma) / sizeof(struct LCM_setting_table);
			if (motify_once_gamma_60 == 1) {
				gamma93_rgb[0] = ((gamma93_120[0] & 0x0F) << 8) | gamma93_120[2];
				gamma93_rgb[1] = ((gamma93_120[1] & 0xF0) << 4) | gamma93_120[3];
				gamma93_rgb[2] = ((gamma93_120[1] & 0x0F) << 8) | gamma93_120[4];

				gamma94_rgb[0] = ((gamma94_120[0] & 0x0F) << 8) | gamma94_120[2];
				gamma94_rgb[1] = ((gamma94_120[1] & 0xF0) << 4) | gamma94_120[3];
				gamma94_rgb[2] = ((gamma94_120[1] & 0x0F) << 8) | gamma94_120[4];

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 21U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma95_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma95_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma95_120[2] = rgb_data[0] & 0xFF;
				gamma95_120[3] = rgb_data[1] & 0xFF;
				gamma95_120[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 42U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma96_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma96_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma96_120[2] = rgb_data[0] & 0xFF;
				gamma96_120[3] = rgb_data[1] & 0xFF;
				gamma96_120[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 50U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma97_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma97_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma97_120[2] = rgb_data[0] & 0xFF;
				gamma97_120[3] = rgb_data[1] & 0xFF;
				gamma97_120[4] = rgb_data[2] & 0xFF;

				for (i = 4; i >= 0; i--) {
					dsi_on_cmd_sdc60_dvgamma[count-6].para_list[i+1] = gamma95_120[i];
					dsi_on_cmd_sdc60_dvgamma[count-5].para_list[i+1] = gamma96_120[i];
					dsi_on_cmd_sdc60_dvgamma[count-4].para_list[i+1] = gamma97_120[i];
				}
				motify_once_gamma_60 = 0;
			}
			push_table(ctx, dsi_on_cmd_sdc60_dvgamma, count);
		} else if (get_panel_es_ver() == ES_DV4) {
			if (m_db == 7 && m_dc == 1) {
				push_table(ctx, dsi_on_cmd_sdc60_dv7, sizeof(dsi_on_cmd_sdc60_dv7) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc60, sizeof(dsi_on_cmd_sdc60) / sizeof(struct LCM_setting_table));
			}
		} else if (get_panel_es_ver() == ES_DV5) {
			push_table(ctx, dsi_on_cmd_sdc60_dv5, sizeof(dsi_on_cmd_sdc60_dv5) / sizeof(struct LCM_setting_table));
		}
		break;
	case FHD_SDC90:
		pr_info("fhd_dsi_on_cmd_sdc90\n");
		if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc90_dv2, sizeof(dsi_on_cmd_sdc90_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_sdc90_dv3, sizeof(dsi_on_cmd_sdc90_dv3) / sizeof(struct LCM_setting_table));
		else if (m_db == 7 && m_dc == 0) {
			count = sizeof(dsi_on_cmd_sdc90_dvgamma) / sizeof(struct LCM_setting_table);
			if (motify_once_gamma_90 == 1) {
				gamma93_rgb[0] = ((gamma93_90[0] & 0x0F) << 8) | gamma93_90[2];
				gamma93_rgb[1] = ((gamma93_90[1] & 0xF0) << 4) | gamma93_90[3];
				gamma93_rgb[2] = ((gamma93_90[1] & 0x0F) << 8) | gamma93_90[4];

				gamma94_rgb[0] = ((gamma94_90[0] & 0x0F) << 8) | gamma94_90[2];
				gamma94_rgb[1] = ((gamma94_90[1] & 0xF0) << 4) | gamma94_90[3];
				gamma94_rgb[2] = ((gamma94_90[1] & 0x0F) << 8) | gamma94_90[4];

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 21U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma95_90[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma95_90[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma95_90[2] = rgb_data[0] & 0xFF;
				gamma95_90[3] = rgb_data[1] & 0xFF;
				gamma95_90[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 42U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma96_90[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma96_90[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma96_90[2] = rgb_data[0] & 0xFF;
				gamma96_90[3] = rgb_data[1] & 0xFF;
				gamma96_90[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 50U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma97_90[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma97_90[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma97_90[2] = rgb_data[0] & 0xFF;
				gamma97_90[3] = rgb_data[1] & 0xFF;
				gamma97_90[4] = rgb_data[2] & 0xFF;
				for (i = 4; i >= 0; i--) {
					dsi_on_cmd_sdc90_dvgamma[count-6].para_list[i+1] = gamma95_90[i];
					dsi_on_cmd_sdc90_dvgamma[count-5].para_list[i+1] = gamma96_90[i];
					dsi_on_cmd_sdc90_dvgamma[count-4].para_list[i+1] = gamma97_90[i];
				}
				motify_once_gamma_90 = 0;
			}
			push_table(ctx, dsi_on_cmd_sdc90_dvgamma, count);
		} else if (get_panel_es_ver() == ES_DV4) {
			if (m_db == 7 && m_dc == 1) {
				push_table(ctx, dsi_on_cmd_sdc90_dv7, sizeof(dsi_on_cmd_sdc90_dv7) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc90, sizeof(dsi_on_cmd_sdc90) / sizeof(struct LCM_setting_table));
			}
		} else if (get_panel_es_ver() == ES_DV5) {
			push_table(ctx, dsi_on_cmd_sdc90_dv5, sizeof(dsi_on_cmd_sdc90_dv5) / sizeof(struct LCM_setting_table));
		}
		break;
	case FHD_SDC120:
		pr_info("fhd_dsi_on_cmd_sdc120\n");
		if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc120_dv2, sizeof(dsi_on_cmd_sdc120_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_sdc120_dv3, sizeof(dsi_on_cmd_sdc120_dv3) / sizeof(struct LCM_setting_table));
		else if (m_db == 7 && m_dc == 0) {
			count = sizeof(dsi_on_cmd_sdc120_dvgamma) / sizeof(struct LCM_setting_table);
			if (motify_once_gamma_120 == 1) {
				gamma93_rgb[0] = ((gamma93_120[0] & 0x0F) << 8) | gamma93_120[2];
				gamma93_rgb[1] = ((gamma93_120[1] & 0xF0) << 4) | gamma93_120[3];
				gamma93_rgb[2] = ((gamma93_120[1] & 0x0F) << 8) | gamma93_120[4];

				gamma94_rgb[0] = ((gamma94_120[0] & 0x0F) << 8) | gamma94_120[2];
				gamma94_rgb[1] = ((gamma94_120[1] & 0xF0) << 4) | gamma94_120[3];
				gamma94_rgb[2] = ((gamma94_120[1] & 0x0F) << 8) | gamma94_120[4];

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 21U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma95_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma95_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma95_120[2] = rgb_data[0] & 0xFF;
				gamma95_120[3] = rgb_data[1] & 0xFF;
				gamma95_120[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 42U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma96_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma96_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma96_120[2] = rgb_data[0] & 0xFF;
				gamma96_120[3] = rgb_data[1] & 0xFF;
				gamma96_120[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 50U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma97_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma97_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma97_120[2] = rgb_data[0] & 0xFF;
				gamma97_120[3] = rgb_data[1] & 0xFF;
				gamma97_120[4] = rgb_data[2] & 0xFF;
				for (i = 4; i >= 0; i--) {
					dsi_on_cmd_sdc120_dvgamma[count-6].para_list[i+1] = gamma95_120[i];
					dsi_on_cmd_sdc120_dvgamma[count-5].para_list[i+1] = gamma96_120[i];
					dsi_on_cmd_sdc120_dvgamma[count-4].para_list[i+1] = gamma97_120[i];
				}
				motify_once_gamma_120 = 0;
			}
			push_table(ctx, dsi_on_cmd_sdc120_dvgamma, count);
		} else if (get_panel_es_ver() == ES_DV4) {
			if (m_db == 7 && m_dc == 1) {
				push_table(ctx, dsi_on_cmd_sdc120_dv7, sizeof(dsi_on_cmd_sdc120_dv7) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120) / sizeof(struct LCM_setting_table));
			}
		} else if (get_panel_es_ver() == ES_DV5) {
			push_table(ctx, dsi_on_cmd_sdc120_dv5, sizeof(dsi_on_cmd_sdc120_dv5) / sizeof(struct LCM_setting_table));
		}
		break;
	case FHD_OPLUS120:
		pr_info("fhd_dsi_on_cmd_oplus120\n");
		if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_oa120_dv2, sizeof(dsi_on_cmd_oa120_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_oa120_dv3, sizeof(dsi_on_cmd_oa120_dv3) / sizeof(struct LCM_setting_table));
		else if (m_db == 7 && m_dc == 0) {
			count = sizeof(dsi_on_cmd_oa120_dvgamma) / sizeof(struct LCM_setting_table);
			if (motify_once_gamma_oa120 == 1) {
				gamma93_rgb[0] = ((gamma93_120[0] & 0x0F) << 8) | gamma93_120[2];
				gamma93_rgb[1] = ((gamma93_120[1] & 0xF0) << 4) | gamma93_120[3];
				gamma93_rgb[2] = ((gamma93_120[1] & 0x0F) << 8) | gamma93_120[4];

				gamma94_rgb[0] = ((gamma94_120[0] & 0x0F) << 8) | gamma94_120[2];
				gamma94_rgb[1] = ((gamma94_120[1] & 0xF0) << 4) | gamma94_120[3];
				gamma94_rgb[2] = ((gamma94_120[1] & 0x0F) << 8) | gamma94_120[4];

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 21U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma95_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma95_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma95_120[2] = rgb_data[0] & 0xFF;
				gamma95_120[3] = rgb_data[1] & 0xFF;
				gamma95_120[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 42U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma96_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma96_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma96_120[2] = rgb_data[0] & 0xFF;
				gamma96_120[3] = rgb_data[1] & 0xFF;
				gamma96_120[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 50U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma97_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma97_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma97_120[2] = rgb_data[0] & 0xFF;
				gamma97_120[3] = rgb_data[1] & 0xFF;
				gamma97_120[4] = rgb_data[2] & 0xFF;
				for (i = 4; i >= 0; i--) {
					dsi_on_cmd_oa120_dvgamma[count-6].para_list[i+1] = gamma95_120[i];
					dsi_on_cmd_oa120_dvgamma[count-5].para_list[i+1] = gamma96_120[i];
					dsi_on_cmd_oa120_dvgamma[count-4].para_list[i+1] = gamma97_120[i];
				}
				motify_once_gamma_oa120 = 0;
			}
			push_table(ctx, dsi_on_cmd_oa120_dvgamma, count);
		} else if (get_panel_es_ver() == ES_DV4) {
			if (m_db == 7 && m_dc == 1) {
				push_table(ctx, dsi_on_cmd_oa120_dv7, sizeof(dsi_on_cmd_oa120_dv7) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_oa120, sizeof(dsi_on_cmd_oa120) / sizeof(struct LCM_setting_table));
			}
		} else if (get_panel_es_ver() == ES_DV5) {
			push_table(ctx, dsi_on_cmd_oa120_dv5, sizeof(dsi_on_cmd_oa120_dv5) / sizeof(struct LCM_setting_table));
		}
		break;
	default:
		pr_info(" default mode_id\n");
		if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc120_dv2, sizeof(dsi_on_cmd_sdc120_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_sdc120_dv3, sizeof(dsi_on_cmd_sdc120_dv3) / sizeof(struct LCM_setting_table));
		else if (m_db == 7 && m_dc == 0) {
			count = sizeof(dsi_on_cmd_sdc120_dvgamma) / sizeof(struct LCM_setting_table);
			if (motify_once_gamma_120 == 1) {
				gamma93_rgb[0] = ((gamma93_120[0] & 0x0F) << 8) | gamma93_120[2];
				gamma93_rgb[1] = ((gamma93_120[1] & 0xF0) << 4) | gamma93_120[3];
				gamma93_rgb[2] = ((gamma93_120[1] & 0x0F) << 8) | gamma93_120[4];

				gamma94_rgb[0] = ((gamma94_120[0] & 0x0F) << 8) | gamma94_120[2];
				gamma94_rgb[1] = ((gamma94_120[1] & 0xF0) << 4) | gamma94_120[3];
				gamma94_rgb[2] = ((gamma94_120[1] & 0x0F) << 8) | gamma94_120[4];

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 21U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma95_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma95_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma95_120[2] = rgb_data[0] & 0xFF;
				gamma95_120[3] = rgb_data[1] & 0xFF;
				gamma95_120[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 42U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma96_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma96_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma96_120[2] = rgb_data[0] & 0xFF;
				gamma96_120[3] = rgb_data[1] & 0xFF;
				gamma96_120[4] = rgb_data[2] & 0xFF;

				for(i = 0; i < 3; i++) {
					val = gamma94_rgb[i] - gamma93_rgb[i];
					rgb_data[i] = (val * 1024U * 50U / 32U + gamma94_rgb[i] * 1024U + 512U) / 1024U;
				}
				gamma97_120[0] = (rgb_data[0] >> 8) & 0xFF;
				gamma97_120[1] = ((rgb_data[1] & 0xF00) >> 4) | ((rgb_data[2] & 0xF00) >> 8);
				gamma97_120[2] = rgb_data[0] & 0xFF;
				gamma97_120[3] = rgb_data[1] & 0xFF;
				gamma97_120[4] = rgb_data[2] & 0xFF;
				for (i = 4; i >= 0; i--) {
					dsi_on_cmd_sdc120_dvgamma[count-6].para_list[i+1] = gamma95_120[i];
					dsi_on_cmd_sdc120_dvgamma[count-5].para_list[i+1] = gamma96_120[i];
					dsi_on_cmd_sdc120_dvgamma[count-4].para_list[i+1] = gamma97_120[i];
				}
				motify_once_gamma_120 = 0;
			}
			push_table(ctx, dsi_on_cmd_sdc120_dvgamma, count);
		} else if (get_panel_es_ver() == ES_DV4) {
			if (m_db == 7 && m_dc == 1) {
				push_table(ctx, dsi_on_cmd_sdc120_dv7, sizeof(dsi_on_cmd_sdc120_dv7) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120) / sizeof(struct LCM_setting_table));
			}
		} else if (get_panel_es_ver() == ES_DV5) {
			push_table(ctx, dsi_on_cmd_sdc120_dv5, sizeof(dsi_on_cmd_sdc120_dv5) / sizeof(struct LCM_setting_table));
		}
		break;
	}
	if (m_db >= ES_DV5) {
		if (en) {
			pr_info("%s, restore 1pulse timing, id:%d, backlight:%d\n", __func__, mode_id, oplus_display_brightness);
			if (mode_id == FHD_SDC60) {
				count = sizeof(dsi_power_on_sdc60_1pul) / sizeof(struct LCM_setting_table);
				push_table(ctx, dsi_power_on_sdc60_1pul, count);
			} else if (mode_id == FHD_SDC90) {
				count = sizeof(dsi_power_on_sdc90_1pul) / sizeof(struct LCM_setting_table);
				push_table(ctx, dsi_power_on_sdc90_1pul, count);
			} else if (mode_id == FHD_SDC120 || mode_id ==  FHD_OPLUS120) {
				count = sizeof(dsi_power_on_sdc120_1pul) / sizeof(struct LCM_setting_table);
				push_table(ctx, dsi_power_on_sdc120_1pul, count);
			}
		}
		pr_info("%s, restore seed mode, id:%d\n", __func__, temp_seed_mode);
		if (temp_seed_mode == NATURAL) {
			push_table(ctx, dsi_set_seed_natural, sizeof(dsi_set_seed_natural) / sizeof(struct LCM_setting_table));
		} else if (temp_seed_mode == EXPERT) {
			push_table(ctx, dsi_set_seed_expert, sizeof(dsi_set_seed_expert) / sizeof(struct LCM_setting_table));
		} else if (temp_seed_mode == VIVID) {
			push_table(ctx, dsi_set_seed_vivid, sizeof(dsi_set_seed_vivid) / sizeof(struct LCM_setting_table));
		}

	}
	set_pwm_turbo_power_on(true);

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		if (lhbm_pressed_icon_gamma_cmd[2].count == 6) {
			if (oplus_ofp_lhbm_pressed_icon_gamma_update(NULL, lhbm_pressed_icon_gamma_cmd) == 1) {
				push_table(ctx, lhbm_pressed_icon_gamma_cmd, sizeof(lhbm_pressed_icon_gamma_cmd) / sizeof(struct LCM_setting_table));
			}
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

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
	bool is_pcp = false;
	int vrefresh_rate = 0;
	unsigned int lcm_cmd_count = 0;
	struct lcm *ctx = panel_to_lcm(panel);
	struct LCM_setting_table *aod_off_cmd = NULL;
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
			aod_off_cmd = aod_off_cmd_60hz;
			lcm_cmd_count = sizeof(aod_off_cmd_60hz) / sizeof(struct LCM_setting_table);
		} else if (vrefresh_rate == 120) {
			aod_off_cmd = aod_off_cmd_120hz;
			lcm_cmd_count = sizeof(aod_off_cmd_120hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_cmd_90hz;
			lcm_cmd_count = sizeof(aod_off_cmd_90hz) / sizeof(struct LCM_setting_table);
		}
		is_pcp = oplus_display_is_pcp(aod_off_cmd, lcm_cmd_count);
		oplus_pcp_handle(is_pcp, NULL);
		push_table(ctx, aod_off_cmd, lcm_cmd_count);
		if (is_pcp) {
			atomic_inc(&oplus_pcp_handle_lock);
		}
		usleep_range(9000, 9100);
		OFP_INFO("send aod off cmd\n");
		atomic_set(&esd_pending, 0);
	}

	usleep_range(5000, 5100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(25000, 25100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(125000, 125100);
	pr_info("[LCM]%s, clear pulse status flag\n", __func__);
	pulse_flg = false;

	while (atomic_read(&oplus_pcp_handle_lock) > 0) {
		pr_info("%s atommic ++ %d\n", __func__, atomic_read(&oplus_pcp_handle_lock));
		atomic_dec(&oplus_pcp_handle_lock);
		pr_info("%s atommic -- %d\n", __func__, atomic_read(&oplus_pcp_handle_lock));
		mutex_unlock(&oplus_pcp_lock);
		atomic_dec(&oplus_pcp_num);
		oplus_disp_trace_end("M_LOCK_PCP");
		pr_info("%s oplus_pcp_unlock\n", __func__);
	}
	pr_info("%s oplus_pcp_handle_lock = %d, oplus_pcp_num = %d\n",
		__func__, atomic_read(&oplus_pcp_handle_lock),
		atomic_read(&oplus_pcp_num));

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
//#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

//#endif
	.cmd_null_pkt_en = 0,
	.cmd_null_pkt_len = 0,
	.merge_trig_offset = 8658,  /* 333us, 1us = 26tick */
	.oplus_vidle_te_duration = 7900,
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
	.manufacture = "P_3",
	.panel_type = 0,
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
	.oplus_ofp_pre_hbm_off_delay = 6,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 2,
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
//#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

//#endif
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 400,
	.merge_trig_offset = 19916,  /* 766us */
	.oplus_vidle_te_duration = 15800,
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
	.manufacture = "P_3",
	.panel_type = 0,
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
	.oplus_ofp_pre_hbm_off_delay = 12,
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
//#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

//#endif
	.cmd_null_pkt_en = 0,
	.cmd_null_pkt_len = 0,
	.merge_trig_offset = 11986,  /* 461us */
	.oplus_vidle_te_duration = 10500,
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
	.manufacture = "P_3",
	.panel_type = 0,
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
	.oplus_ofp_pre_hbm_off_delay = 9,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 2,
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
//#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

//#endif
	.cmd_null_pkt_en = 0,
	.cmd_null_pkt_len = 0,
	.merge_trig_offset = 8658,  /* 333us */
	.oplus_vidle_te_duration = 7900,
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
	.manufacture = "P_3",
	.panel_type = 0,
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
	.oplus_ofp_pre_hbm_off_delay = 6,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 2,
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

ktime_t aod_off_te_timestamp(void)
{
	ktime_t aod_off_time;

	aod_off_time = ktime_get();
	return aod_off_time;
}

int oplus_panel_pwm_compatible_mode_aodoff_handle(void)
{
	ktime_t cur_time;
	u32 interval;
	u32 delay;
	int rc = 0;

	if (!aod_off_swtich_pulse)
		return rc;

	/* first power on backlight */
	cur_time = ktime_get();
	interval = ktime_to_us(ktime_sub(cur_time, aod_off_timestamp));
	if (interval < 35000) {
		/* delay cmd to the next frame of aod_off for preventing screen flash*/
		delay = 35000 - interval;
		pr_info("need to delay for the interval between last aod off and cur time was %d\n", delay);
		usleep_range(delay, delay + 200);
		aod_off_swtich_pulse = false;
	}
	return rc;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int i = 0;
	unsigned int mapped_level = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	if (silence_mode) {
		pr_info("pwm_turbo silence_mode is %d, set backlight to 0\n", silence_mode);
		level = 0;
	}

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (last_backlight == 0 || level == 0) {
		DISP_INFO("[INFO][%s:%d]backlight lvl:%u, panel_last_brightness:%u\n", __func__, __LINE__, level, panel_last_brightness);
	} else {
		DISP_BACKLIGHT("[INFO][%s:%d]backlight lvl:%u, panel_last_brightness:%u\n", __func__, __LINE__, level, panel_last_brightness);
	}


#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		if (lhbm_pressed_icon_grayscale_cmd[1].count == 7) {
			if (oplus_ofp_lhbm_pressed_icon_grayscale_update(lhbm_pressed_icon_grayscale_cmd[1].para_list, level) == 1) {
				for(i = 0; i < (sizeof(lhbm_pressed_icon_grayscale_cmd) / sizeof(struct LCM_setting_table)); i++) {
					cb(dsi, handle, lhbm_pressed_icon_grayscale_cmd[i].para_list, lhbm_pressed_icon_grayscale_cmd[i].count);
				}
			}
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	last_backlight = level;
	mapped_level = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);

	if (level == 1) {
		pr_info("[DISP][INFO][%s:%d]filter backlight %u setting\n", __func__, __LINE__, level);
		panel_last_brightness = 1;
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = 2047;
	}

	if(oplus_panel_pwm_onepulse_is_enabled() && (level > 1)) {
		if (panel_last_brightness == 0 || panel_last_brightness == 1 || ofp_aod_off_swtich_pulse == true) {
			set_pwm_turbo_power_on(true);
			ofp_aod_off_swtich_pulse = false;
		}

		oplus_display_panel_pwm_switch_prepare(level);
		pr_info("%s pwm switch, pulse_flg: %d", __func__, pulse_flg);
		if((m_db >= ES_DV5) && pulse_flg) {
			if (atomic_read(&esd_pending) == 1) {
				aod_off_swtich_pulse = true;
				oplus_panel_pwm_compatible_mode_aodoff_handle();
			}
			pr_info("%s need up pwm switch, pwm_params->pwm_pul_cmd_id:%d", __func__, pwm_params->pwm_pul_cmd_id);
			oplus_display_panel_set_pwm_pul_lp(dsi, cb, handle, pwm_params->pwm_pul_cmd_id);
			set_pwm_turbo_power_on(false);
			pulse_flg = false;
		} else {
			bl_level[1] = level >> 8;
			bl_level[2] = level & 0xFF;
			cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
		}
	} else {
		bl_level[1] = level >> 8;
		bl_level[2] = level & 0xFF;
		cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		if (oplus_ofp_lhbm_dbv_alpha_update(lhbm_dbv_alpha_cmd, sizeof(lhbm_dbv_alpha_cmd) / sizeof(struct LCM_setting_table), false, level) > 0) {
			for(i = 0; i < (sizeof(lhbm_dbv_alpha_cmd) / sizeof(struct LCM_setting_table)); i++) {
				cb(dsi, handle, lhbm_dbv_alpha_cmd[i].para_list, lhbm_dbv_alpha_cmd[i].count);
			}
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	panel_last_brightness = level;

	return 0;
}

#ifdef OPLUS_FEATURE_DISPLAY_HPWM
void oplus_display_panel_pwm_switch_prepare(unsigned int level) {
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	bool condition1 = (last_backlight <= get_pwm_turbo_plus_bl()) && (level > get_pwm_turbo_plus_bl());
	bool condition2 = (last_backlight > get_pwm_turbo_plus_bl()) && (level <= get_pwm_turbo_plus_bl());
	bool condition3 = pwm_params->pwm_power_on && (level > get_pwm_turbo_plus_bl());
	bool condition4 = pwm_params->pwm_power_on && (level <= get_pwm_turbo_plus_bl());
	if (condition1
			|| condition2
			|| condition3
			|| condition4) {
		pulse_flg = true;
	}

	if (oplus_panel_pwm_onepulse_is_enabled()) {
		if (condition1 || condition3) {
			pwm_params->pwm_pul_cmd_id = PWM_SWITCH_18TO1;
			set_pwm_turbo_switch_state(PWM_SWITCH_ONEPULSE_STATE);
		}
		if (condition2 || condition4) {
			pwm_params->pwm_pul_cmd_id = PWM_SWITCH_1TO18;
			set_pwm_turbo_switch_state(PWM_SWITCH_HPWM_STATE);
		}
	} else {
		if (condition1 || condition3) {
			pwm_params->pwm_pul_cmd_id = PWM_SWITCH_18TO3;
			set_pwm_turbo_switch_state(PWM_SWITCH_DC_STATE);
		}
		if (condition2 || condition4) {
			pwm_params->pwm_pul_cmd_id = PWM_SWITCH_3TO18;
			set_pwm_turbo_switch_state(PWM_SWITCH_HPWM_STATE);
		}
	}
}

static int oplus_display_panel_set_pwm_pul(void *dsi, dcs_write_gce_pack cb, void *handle, unsigned int mode)
{
	unsigned int level = oplus_display_brightness;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	if (silence_mode) {
		pr_info("pwm_turbo silence_mode is %d, set backlight to 0\n", silence_mode);
		level = 0;
	}

	switch (mode) {
		case PWM_SWITCH_18TO3: {
			pr_info("pwm_turbo dsi_pwm_switch 18to3\n");
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				dsi_pwm_switch_18to3pul_120hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to3pul_120hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to3pul_120hz, sizeof(dsi_pwm_switch_18to3pul_120hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				dsi_pwm_switch_18to3pul_90hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to3pul_90hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to3pul_90hz, sizeof(dsi_pwm_switch_18to3pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
				dsi_pwm_switch_18to3pul_60hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to3pul_60hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to3pul_60hz, sizeof(dsi_pwm_switch_18to3pul_60hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			}
		}
		break;
		case PWM_SWITCH_3TO18: {
			pr_info("pwm_turbo dsi_pwm_switch 3to18\n");
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				dsi_pwm_switch_3to18pul_120hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_3to18pul_120hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_3to18pul_120hz, sizeof(dsi_pwm_switch_3to18pul_120hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				dsi_pwm_switch_3to18pul_90hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_3to18pul_90hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_3to18pul_90hz, sizeof(dsi_pwm_switch_3to18pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
				dsi_pwm_switch_3to18pul_60hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_3to18pul_60hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_3to18pul_60hz, sizeof(dsi_pwm_switch_3to18pul_60hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			}
		}
		break;
		case PWM_SWITCH_1TO3: {
			pr_info("pwm_turbo onepulse dsi_pwm_switch 1to3 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_1to3pul_120hz,
					sizeof(dsi_pwm_switch_1to3pul_120hz) /sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_1to3pul_90hz, sizeof(dsi_pwm_switch_1to3pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_1to3pul_60hz, sizeof(dsi_pwm_switch_1to3pul_60hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			}
		}
		break;
		case PWM_SWITCH_3TO1: {
			pr_info("pwm_turbo onepulse dsi_pwm_switch 3to1 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_3to1pul_120hz,
					sizeof(dsi_pwm_switch_3to1pul_120hz) /sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_3to1pul_90hz, sizeof(dsi_pwm_switch_3to1pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_3to1pul_60hz, sizeof(dsi_pwm_switch_3to1pul_60hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			}
		}
		break;
		case PWM_SWITCH_18TO1: {
			if (pwm_params->lhbm_off_wait) {
				pr_info("pwm_turbo onepulse dsi_pwm_switch 18to1 wait 17ms\n");
				usleep_range(17000, 17100);
			}
			pr_info("pwm_turbo onepulse dsi_pwm_switch 18to1 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				dsi_pwm_switch_18to1pul_120hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to1pul_120hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to1pul_120hz, sizeof(dsi_pwm_switch_18to1pul_120hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				dsi_pwm_switch_18to1pul_90hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to1pul_90hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to1pul_90hz, sizeof(dsi_pwm_switch_18to1pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
				dsi_pwm_switch_18to1pul_60hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to1pul_60hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to1pul_60hz, sizeof(dsi_pwm_switch_18to1pul_60hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			}
		}
		break;
		case PWM_SWITCH_1TO18: {
			if (pwm_params->lhbm_off_wait) {
				pr_info("pwm_turbo onepulse dsi_pwm_switch 1to18 wait 17ms\n");
				usleep_range(17000, 17100);
			}
			pr_info("pwm_turbo onepulse dsi_pwm_switch 1to18 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				dsi_pwm_switch_1to18pul_120hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_1to18pul_120hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_1to18pul_120hz, sizeof(dsi_pwm_switch_1to18pul_120hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				dsi_pwm_switch_1to18pul_90hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_1to18pul_90hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_1to18pul_90hz, sizeof(dsi_pwm_switch_1to18pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
				dsi_pwm_switch_1to18pul_60hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_1to18pul_60hz[1].para_list[2] = level & 0xFF;
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_1to18pul_60hz, sizeof(dsi_pwm_switch_1to18pul_60hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			}
		}
		break;
		default:
		break;
	}

	pwm_params->lhbm_off_wait = false;

	return 0;
}

static int oplus_display_panel_set_pwm_pul_lp(void *dsi, dcs_write_gce cb, void *handle, unsigned int mode)
{
	unsigned int i = 0;
	unsigned int level = oplus_display_brightness;

	if (silence_mode) {
		pr_info("pwm_turbo silence_mode is %d, set backlight to 0\n", silence_mode);
		level = 0;
	}

	switch (mode) {
		case PWM_SWITCH_18TO3: {
			pr_info("pwm_turbo dsi_pwm_switch 18to3\n");
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				dsi_pwm_switch_18to3pul_120hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to3pul_120hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_18to3pul_120hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_18to3pul_120hz[i].para_list, dsi_pwm_switch_18to3pul_120hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC90) {
				dsi_pwm_switch_18to3pul_90hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to3pul_90hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_18to3pul_90hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_18to3pul_90hz[i].para_list, dsi_pwm_switch_18to3pul_90hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC60) {
				dsi_pwm_switch_18to3pul_60hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to3pul_60hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_18to3pul_60hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_18to3pul_60hz[i].para_list, dsi_pwm_switch_18to3pul_60hz[i].count);
				}
			}
		}
		break;
		case PWM_SWITCH_3TO18: {
			pr_info("pwm_turbo dsi_pwm_switch 3to18\n");
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				dsi_pwm_switch_3to18pul_120hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_3to18pul_120hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_3to18pul_120hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_3to18pul_120hz[i].para_list, dsi_pwm_switch_3to18pul_120hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC90) {
				dsi_pwm_switch_3to18pul_90hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_3to18pul_90hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_3to18pul_90hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_3to18pul_90hz[i].para_list, dsi_pwm_switch_3to18pul_90hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC60) {
				dsi_pwm_switch_3to18pul_60hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_3to18pul_60hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_3to18pul_60hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_3to18pul_60hz[i].para_list, dsi_pwm_switch_3to18pul_60hz[i].count);
				}
			}
		}
		break;
		case PWM_SWITCH_1TO3: {
			pr_info("pwm_turbo onepulse dsi_pwm_switch 1to3 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				for(i = 0; i < sizeof(dsi_pwm_switch_1to3pul_120hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_1to3pul_120hz[i].para_list, dsi_pwm_switch_1to3pul_120hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC90) {
				for(i = 0; i < sizeof(dsi_pwm_switch_1to3pul_90hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_1to3pul_90hz[i].para_list, dsi_pwm_switch_1to3pul_90hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC60) {
				for(i = 0; i < sizeof(dsi_pwm_switch_1to3pul_60hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_1to3pul_60hz[i].para_list, dsi_pwm_switch_1to3pul_60hz[i].count);
				}
			}
		}
		break;
		case PWM_SWITCH_3TO1: {
			pr_info("pwm_turbo onepulse dsi_pwm_switch 3to1 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				for(i = 0; i < sizeof(dsi_pwm_switch_3to1pul_120hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_3to1pul_120hz[i].para_list, dsi_pwm_switch_3to1pul_120hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC90) {
				for(i = 0; i < sizeof(dsi_pwm_switch_3to1pul_90hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_3to1pul_90hz[i].para_list, dsi_pwm_switch_3to1pul_90hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC60) {
				for(i = 0; i < sizeof(dsi_pwm_switch_3to1pul_60hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_3to1pul_60hz[i].para_list, dsi_pwm_switch_3to1pul_60hz[i].count);
				}
			}
		}
		break;
		case PWM_SWITCH_18TO1: {
			pr_info("pwm_turbo onepulse dsi_pwm_switch 18to1 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				dsi_pwm_switch_18to1pul_120hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to1pul_120hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_18to1pul_120hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_18to1pul_120hz[i].para_list, dsi_pwm_switch_18to1pul_120hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC90) {
				dsi_pwm_switch_18to1pul_90hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to1pul_90hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_18to1pul_90hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_18to1pul_90hz[i].para_list, dsi_pwm_switch_18to1pul_90hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC60) {
				dsi_pwm_switch_18to1pul_60hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_18to1pul_60hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_18to1pul_60hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_18to1pul_60hz[i].para_list, dsi_pwm_switch_18to1pul_60hz[i].count);
				}
			}
		}
		break;
		case PWM_SWITCH_1TO18: {
			pr_info("pwm_turbo onepulse dsi_pwm_switch 1to18 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				dsi_pwm_switch_1to18pul_120hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_1to18pul_120hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_1to18pul_120hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_1to18pul_120hz[i].para_list, dsi_pwm_switch_1to18pul_120hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC90) {
				dsi_pwm_switch_1to18pul_90hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_1to18pul_90hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_1to18pul_90hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_1to18pul_90hz[i].para_list, dsi_pwm_switch_1to18pul_90hz[i].count);
				}
			} else if (panel_mode_id == FHD_SDC60) {
				dsi_pwm_switch_1to18pul_60hz[1].para_list[1] = level >> 8;
				dsi_pwm_switch_1to18pul_60hz[1].para_list[2] = level & 0xFF;
				for(i = 0; i < sizeof(dsi_pwm_switch_1to18pul_60hz)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, dsi_pwm_switch_1to18pul_60hz[i].para_list, dsi_pwm_switch_1to18pul_60hz[i].count);
				}
			}
		}
		break;
		default:
		break;
	}

	return 0;
}

static int oplus_display_panel_set_pwm_plus_bl(void *dsi, dcs_write_gce_pack cb, void *handle, unsigned int level)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (last_backlight == 0 || level == 0) {
		DISP_INFO("[INFO][%s:%d]backlight lvl:%u, panel_last_brightness:%u\n", __func__, __LINE__, level, panel_last_brightness);
	} else {
		DISP_BACKLIGHT("[INFO][%s:%d]backlight lvl:%u, panel_last_brightness:%u\n", __func__, __LINE__, level, panel_last_brightness);
	}

	if (level > 4095) {
		level = 4095;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = 2047;
	}

	if (silence_mode) {
		pr_info("pwm_turbo silence_mode is %d, set backlight to 0\n", silence_mode);
		level = 0;
	}

	if (m_db >= ES_DV5) {
		if ((panel_last_brightness == 0 || panel_last_brightness == 1) && level > 1)
			set_pwm_turbo_power_on(true);
		oplus_display_panel_pwm_switch_prepare(level);
	}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
		if (oplus_ofp_is_supported()) {
			if (lhbm_pressed_icon_grayscale_cmd[1].count == 7) {
				if (oplus_ofp_lhbm_pressed_icon_grayscale_update(lhbm_pressed_icon_grayscale_cmd[1].para_list, level) == 1) {
					panel_send_pack_hs_cmd(dsi, lhbm_pressed_icon_grayscale_cmd, sizeof(lhbm_pressed_icon_grayscale_cmd) / sizeof(struct LCM_setting_table), cb, handle);
				}
			}
		}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	last_backlight = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &last_backlight);

	if (level == 1) {
		pr_info("[DISP][INFO][%s:%d]filter backlight %u setting\n", __func__, __LINE__, level);
		panel_last_brightness = 1;
		return 0;
	}

	if (pulse_flg && m_db >= ES_DV5) {
		oplus_panel_pwm_compatible_mode_aodoff_handle();
		pr_info("%s filter backlight when pwm switch", __func__);
		oplus_display_panel_set_pwm_pul(dsi, cb, handle, pwm_params->pwm_pul_cmd_id);
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
		if (oplus_ofp_is_supported()) {
			if (oplus_ofp_lhbm_dbv_alpha_update(lhbm_dbv_alpha_cmd, sizeof(lhbm_dbv_alpha_cmd) / sizeof(struct LCM_setting_table), false, level) > 0) {
				panel_send_pack_hs_cmd(dsi, lhbm_dbv_alpha_cmd, sizeof(lhbm_dbv_alpha_cmd) / sizeof(struct LCM_setting_table), cb, handle);
			}
		}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
		set_pwm_turbo_power_on(false);
		pulse_flg = false;
		panel_last_brightness = level;
		return 0;
	}

	dsi_set_backlight[1].para_list[1] = level >> 8;
	dsi_set_backlight[1].para_list[2] = level & 0xFF;
	panel_send_pack_hs_cmd(dsi, dsi_set_backlight, sizeof(dsi_set_backlight) / sizeof(struct LCM_setting_table), cb, handle);

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		if (oplus_ofp_lhbm_dbv_alpha_update(lhbm_dbv_alpha_cmd, sizeof(lhbm_dbv_alpha_cmd) / sizeof(struct LCM_setting_table), false, level) > 0) {
			panel_send_pack_hs_cmd(dsi, lhbm_dbv_alpha_cmd, sizeof(lhbm_dbv_alpha_cmd) / sizeof(struct LCM_setting_table), cb, handle);
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	panel_last_brightness = level;

	return 0;
}
#endif

static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int mode)
{
	unsigned int i = 0;
	if (m_db < ES_DV5) {
		pr_info("[DISP][INFO]%s: not support",__func__);
		return 0;
	}

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
		case VIVID:
			for(i = 0; i < sizeof(dsi_set_seed_vivid)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_vivid[i].para_list, dsi_set_seed_vivid[i].count);
			}
		break;
		default:
		break;
	}
	return 0;

}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
static int lcm_set_hbm(void *dsi, dcs_write_gce_pack cb,
		void *handle, unsigned int hbm_mode)
{
	unsigned int lcm_cmd_count = 0;
	OFP_DEBUG("start\n");

	if (!dsi || !cb) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if(hbm_mode == 1) {
		lcm_cmd_count = sizeof(hbm_on_cmd_60hz) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, hbm_on_cmd_60hz, lcm_cmd_count, cb, handle);
		last_backlight = 4095;
		OFP_INFO("Enter hbm mode, set last_backlight as %d", last_backlight);
		lcdinfo_notify(1, &hbm_mode);
	} else if (hbm_mode == 0) {
		lcm_cmd_count = sizeof(hbm_off_cmd_60hz) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, hbm_off_cmd_60hz, lcm_cmd_count, cb, handle);
		lcdinfo_notify(1, &hbm_mode);
		oplus_display_panel_set_pwm_plus_bl(dsi, cb, handle, oplus_display_brightness);
	}

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
		dcs_write_gce_pack cb, void *handle, bool en)
{
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

	if (en) {
		last_backlight = 4095;
		OFP_INFO("Enter hbm mode, set last_backlight as %d", last_backlight);
	}

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
	panel_send_pack_hs_cmd(dsi, hbm_cmd, reg_count, cb, handle);
	lcdinfo_notify(1, &en);

	if (!en) {
		set_pwm_turbo_power_on(true);
		oplus_display_panel_set_pwm_plus_bl(dsi, cb, handle, oplus_display_brightness);
	}

	OFP_DEBUG("end\n");

	return 0;
}

static int oplus_ofp_set_lhbm_pressed_icon(struct drm_panel *panel, void *dsi_drv, dcs_write_gce_pack cb, void *handle, bool lhbm_pressed_icon_on)
{
	unsigned int reg_count = 0;
	unsigned int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *lhbm_pressed_icon_cmd = NULL;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, should not set lhbm pressed icon\n");
	}

	if (!panel || !dsi_drv || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid ctx params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	OFP_INFO("lhbm_pressed_icon_on:%u,bl_lvl:%u,refresh_rate:%u\n", lhbm_pressed_icon_on, oplus_display_brightness, vrefresh_rate);

	if (lhbm_pressed_icon_on) {
		lhbm_pressed_icon_cmd = lhbm_pressed_icon_on_cmd;
		reg_count = sizeof(lhbm_pressed_icon_on_cmd) / sizeof(struct LCM_setting_table);
	} else {
		lhbm_pressed_icon_cmd = lhbm_pressed_icon_off_cmd;
		reg_count = sizeof(lhbm_pressed_icon_off_cmd) / sizeof(struct LCM_setting_table);
	}

	panel_send_pack_hs_cmd(dsi_drv, lhbm_pressed_icon_cmd, reg_count, cb, handle);

	if (!lhbm_pressed_icon_on) {
		pwm_params->lhbm_off_wait = true;
		oplus_display_panel_set_pwm_plus_bl(dsi_drv, cb, handle, oplus_display_brightness);
	} else {
		if (oplus_ofp_lhbm_dbv_alpha_update(lhbm_dbv_alpha_cmd, sizeof(lhbm_dbv_alpha_cmd) / sizeof(struct LCM_setting_table), true, oplus_display_brightness) > 0) {
			panel_send_pack_hs_cmd(dsi_drv, lhbm_dbv_alpha_cmd, sizeof(lhbm_dbv_alpha_cmd) / sizeof(struct LCM_setting_table), cb, handle);
		}

		if (oplus_display_brightness > 0x0DBB) {
			OFP_INFO("set backlight level to 0x0DBB after pressed icon on\n");
			oplus_display_panel_set_pwm_plus_bl(dsi_drv, cb, handle, 0x0DBB);
		}
	}

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	bool is_pcp = false;
	bool icon_layer_status = false;
	uint64_t hbm_enable = 0;
	unsigned int i = 0;
	unsigned int reg_count = 0;
	int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct mtk_dsi *mtk_dsi = dsi;
	struct LCM_setting_table *aod_off_cmd = NULL;
	struct drm_crtc *crtc = NULL;
	struct mtk_crtc_state *mtk_state = NULL;

	if (!panel || !mtk_dsi) {
		OFP_ERR("Invalid mtk_dsi params\n");
	}

	crtc = mtk_dsi->encoder.crtc;

	if (!crtc || !crtc->state) {
		OFP_ERR("Invalid crtc param\n");
		return -EINVAL;
	}

	mtk_state = to_mtk_crtc_state(crtc->state);
	if (!mtk_state) {
		OFP_ERR("Invalid mtk_state param\n");
		return -EINVAL;
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
	hbm_enable = oplus_ofp_get_hbm_enable();
	icon_layer_status = hbm_enable & OPLUS_OFP_PROPERTY_ICON_LAYER;
	OFP_INFO("icon:%u\n", icon_layer_status);
	if (vrefresh_rate == 60) {
		if (!icon_layer_status) {
			aod_off_cmd = aod_off_cmd_60hz;
			reg_count = sizeof(aod_off_cmd_60hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_no_insert_black_cmd_60hz;
			reg_count = sizeof(aod_off_no_insert_black_cmd_60hz) / sizeof(struct LCM_setting_table);
		}
	} else if (vrefresh_rate == 120) {
		if (!icon_layer_status) {
			aod_off_cmd = aod_off_cmd_120hz;
			reg_count = sizeof(aod_off_cmd_120hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_no_insert_black_cmd_120hz;
			reg_count = sizeof(aod_off_no_insert_black_cmd_120hz) / sizeof(struct LCM_setting_table);
		}
	} else {
		if (!icon_layer_status) {
			aod_off_cmd = aod_off_cmd_90hz;
			reg_count = sizeof(aod_off_cmd_90hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_no_insert_black_cmd_90hz;
			reg_count = sizeof(aod_off_no_insert_black_cmd_90hz) / sizeof(struct LCM_setting_table);
		}
	}

	OFP_INFO("%s crtc_active:%d, doze_active:%llu\n", __func__, crtc->state->active, mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE]);
	is_pcp = oplus_display_is_pcp(aod_off_cmd, reg_count);
	oplus_pcp_handle(is_pcp, handle);
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
	aod_off_timestamp = aod_off_te_timestamp();

	if (is_pcp) {
		atomic_inc(&oplus_pcp_handle_lock);
	}

	if(!oplus_ofp_backlight_filter(crtc, handle, oplus_display_brightness))
			lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);

	if (temp_seed_mode)
		panel_set_seed(dsi, cb, handle, temp_seed_mode);

	OFP_INFO("send aod off cmd\n");
	atomic_set(&esd_pending, 0);
	aod_off_swtich_pulse = true;

	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	bool is_pcp = false;
	unsigned int i = 0;
	struct mtk_dsi *mtk_dsi = dsi;
	struct drm_crtc *crtc = NULL;
	struct mtk_crtc_state *mtk_state = NULL;

	if (!panel || !mtk_dsi) {
		OFP_ERR("Invalid mtk_dsi params\n");
	}

	crtc = mtk_dsi->encoder.crtc;

	if (!crtc || !crtc->state) {
		OFP_ERR("Invalid crtc param\n");
		return -EINVAL;
	}

	mtk_state = to_mtk_crtc_state(crtc->state);
	if (!mtk_state) {
		OFP_ERR("Invalid mtk_state param\n");
		return -EINVAL;
	}

	OFP_INFO("%s crtc_active:%d, doze_active:%llu\n", __func__, crtc->state->active, mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE]);

	atomic_set(&esd_pending, 1);

	is_pcp = oplus_display_is_pcp(aod_on_cmd, sizeof(aod_on_cmd)/sizeof(struct LCM_setting_table));
	oplus_pcp_handle(is_pcp, handle);
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

	if (is_pcp) {
		atomic_inc(&oplus_pcp_handle_lock);
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
	struct mtk_dsi *mtk_dsi = dsi;
	struct LCM_setting_table *ultra_low_power_aod_cmd = NULL;
	struct drm_crtc *crtc = NULL;
	struct mtk_crtc_state *mtk_state = NULL;

	if (!panel || !mtk_dsi) {
		OFP_ERR("Invalid mtk_dsi params\n");
	}

	crtc = mtk_dsi->encoder.crtc;

	if (!crtc || !crtc->state) {
		OFP_ERR("Invalid crtc param\n");
		return -EINVAL;
	}

	mtk_state = to_mtk_crtc_state(crtc->state);
	if (!mtk_state) {
		OFP_ERR("Invalid mtk_state param\n");
		return -EINVAL;
	}

	if (!mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
		OFP_INFO("not in aod mode, not set ultra_low_power_aod_mode\n");
		return 0;
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

	while (atomic_read(&oplus_pcp_handle_lock) > 0) {
		pr_info("%s atommic ++ %d\n", __func__, atomic_read(&oplus_pcp_handle_lock));
		atomic_dec(&oplus_pcp_handle_lock);
		pr_info("%s atommic -- %d\n", __func__, atomic_read(&oplus_pcp_handle_lock));
		mutex_unlock(&oplus_pcp_lock);
		atomic_dec(&oplus_pcp_num);
		oplus_disp_trace_end("M_LOCK_PCP");
		pr_info("%s oplus_pcp_unlock\n", __func__);
	}
	pr_info("%s oplus_pcp_handle_lock = %d, oplus_pcp_num = %d\n",
		__func__, atomic_read(&oplus_pcp_handle_lock),
		atomic_read(&oplus_pcp_num));

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
	bool is_pcp = false;
	unsigned int i = 0;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		pr_err("out of mtk_ddic_dsi_cmd \n");
		return 0;
	}

	is_pcp = oplus_display_is_pcp(table, lcm_cmd_count);
	if (table == lhbm_pressed_icon_off_cmd) {
		is_pcp = true;
		pr_info("%s, lhbm_pressed_icon_off, is_pcp:%d\n", __func__, is_pcp);
	}
	oplus_pcp_handle(is_pcp, handle);

	for (i = 0; i < lcm_cmd_count; i++) {
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].cmd_num = table[i].count;
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].para_list = table[i].para_list;
	}

	send_cmd_to_ddic.is_hs = 1;
	send_cmd_to_ddic.is_package = 1;
	send_cmd_to_ddic.cmd_count = lcm_cmd_count;
	cb(dsi, handle, &send_cmd_to_ddic);

	if (is_pcp) {
		atomic_inc(&oplus_pcp_handle_lock);
	}
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
		if (get_panel_es_ver() == ES_DV2) {
			switch (vrefresh_rate) {
			case 120:
				auto_off_minfps_cmd_120hz_dv2[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
				lcm_cmd_count = sizeof(auto_off_minfps_cmd_120hz_dv2) / sizeof(struct LCM_setting_table);
				ADFR_INFO("auto_off_minfps_cmd_120hz_dv2, ext_frame:%u\n", ext_frame);
				panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_120hz_dv2, lcm_cmd_count, cb, handle);
				break;
			case 90:
				auto_off_minfps_cmd_90hz_dv2[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
				lcm_cmd_count = sizeof(auto_off_minfps_cmd_90hz_dv2) / sizeof(struct LCM_setting_table);
				ADFR_INFO("auto_off_minfps_cmd_90hz_dv2, ext_frame:%u\n", ext_frame);
				panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_90hz_dv2, lcm_cmd_count, cb, handle);
				break;
			case 60:
				auto_off_minfps_cmd_60hz_dv2[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
				lcm_cmd_count = sizeof(auto_off_minfps_cmd_60hz_dv2) / sizeof(struct LCM_setting_table);
				ADFR_INFO("auto_off_minfps_cmd_60hz_dv2, ext_frame:%u\n", ext_frame);
				panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_60hz_dv2, lcm_cmd_count, cb, handle);
				break;
			default:
				break;
			}
		} else {
			switch (vrefresh_rate) {
			case 120:
				if (ext_frame > 3) {
					auto_off_minfps_cmd_120hz_1hz[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
					lcm_cmd_count = sizeof(auto_off_minfps_cmd_120hz_1hz) / sizeof(struct LCM_setting_table);
					ADFR_INFO("auto_off_minfps_cmd_120hz_1hz, ext_frame:%u\n", ext_frame);
					panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_120hz_1hz, lcm_cmd_count, cb, handle);
				} else {
					auto_off_minfps_cmd_120hz[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
					lcm_cmd_count = sizeof(auto_off_minfps_cmd_120hz) / sizeof(struct LCM_setting_table);
					ADFR_INFO("auto_off_minfps_cmd_120hz, ext_frame:%u\n", ext_frame);
					panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_120hz, lcm_cmd_count, cb, handle);
				}
				break;
			case 90:
				if (ext_frame > 2) {
					auto_off_minfps_cmd_90hz_1hz[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
					lcm_cmd_count = sizeof(auto_off_minfps_cmd_90hz_1hz) / sizeof(struct LCM_setting_table);
					ADFR_INFO("auto_off_minfps_cmd_90hz_1hz, ext_frame:%u\n", ext_frame);
					panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_90hz_1hz, lcm_cmd_count, cb, handle);
				} else {
					auto_off_minfps_cmd_90hz[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
					lcm_cmd_count = sizeof(auto_off_minfps_cmd_90hz) / sizeof(struct LCM_setting_table);
					ADFR_INFO("auto_off_minfps_cmd_90hz, ext_frame:%u\n", ext_frame);
					panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_90hz, lcm_cmd_count, cb, handle);
				}
				break;
			case 60:
				if (ext_frame > 3) {
					auto_off_minfps_cmd_60hz_1hz[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
					lcm_cmd_count = sizeof(auto_off_minfps_cmd_60hz_1hz) / sizeof(struct LCM_setting_table);
					ADFR_INFO("auto_off_minfps_cmd_60hz_1hz, ext_frame:%u\n", ext_frame);
					panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_60hz_1hz, lcm_cmd_count, cb, handle);
				} else {
					auto_off_minfps_cmd_60hz[MIN_FPS_CMD_ROW].para_list[MIN_FPS_CMD_COL] = ext_frame;
					lcm_cmd_count = sizeof(auto_off_minfps_cmd_60hz) / sizeof(struct LCM_setting_table);
					ADFR_INFO("auto_off_minfps_cmd_60hz, ext_frame:%u\n", ext_frame);
					panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_60hz, lcm_cmd_count, cb, handle);
				}
				break;
			default:
				break;
			}
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
	bool en = oplus_panel_pwm_onepulse_is_enabled();
	int plus_bl = get_pwm_turbo_plus_bl();
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	g_last_mode_idx = cur_mode;

	if (stage == BEFORE_DSI_POWERDOWN) {
		m_vrefresh = drm_mode_vrefresh(m);
		src_vrefresh = drm_mode_vrefresh(src_m);
		pr_info("mode_switch_hs,cur_mode:%d,dst_mode:%d,hdisplay:%d->%d,vrefresh:%d->%d,hskew:%d->%d\n",
			cur_mode, dst_mode, src_m->hdisplay, m->hdisplay, src_vrefresh, m_vrefresh, src_m->hskew, m->hskew);

		if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
			if (m_vrefresh == 60) {
				pr_info("timing switch to sdc60\n");
				if (get_panel_es_ver() == ES_DV2) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60_dv2) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60_dv2, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() == ES_DV3) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60_dv3) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60_dv3, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() >= ES_DV4) {
					if (en && oplus_display_brightness > plus_bl) {
						if (m_db >= ES_DV5) {
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60_1pul,
								sizeof(timing_switch_cmd_sdc60_1pul) / sizeof(struct LCM_setting_table), cb, NULL);
						} else {
							lcm_cmd_count = sizeof(timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table);
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60, lcm_cmd_count, cb, NULL);
						}
					} else if (get_panel_es_ver() == ES_DV4) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60, lcm_cmd_count, cb, NULL);
					} else if (get_panel_es_ver() == ES_DV5) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc60_dv5) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60_dv5, lcm_cmd_count, cb, NULL);
					}
				}
				panel_mode_id = FHD_SDC60;
				pwm_params->pwm_fps_mode = 60;
			} else if ((src_vrefresh == 60) && (m_vrefresh == 90)) {
				pr_info("timing switch from sdc60 to sdc90, return 1\n");
				ret = 1;
			} else if ((src_vrefresh == 120) && (m_vrefresh == 90)) {
				pr_info("timing switch from sdc120 to sdc90\n");
				if (get_panel_es_ver() == ES_DV2) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_dv2) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_dv2, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() == ES_DV3) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_dv3) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_dv3, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() >= ES_DV4) {
					if (en && oplus_display_brightness > plus_bl) {
						if (m_db >= ES_DV5) {
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_1pul,
								sizeof(timing_switch_cmd_sdc90_1pul) / sizeof(struct LCM_setting_table), cb, NULL);
						} else {
							lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
						}
					} else if (get_panel_es_ver() == ES_DV4) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
					} else if (get_panel_es_ver() == ES_DV5) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_dv5) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_dv5, lcm_cmd_count, cb, NULL);
					}
				}

				panel_mode_id = FHD_SDC90;
				pwm_params->pwm_fps_mode = 90;
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
				if (get_panel_es_ver() == ES_DV2) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_dv2) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_dv2, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() == ES_DV3) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_dv3) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_dv3, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() >= ES_DV4) {
					if (en && oplus_display_brightness > plus_bl) {
						if (m_db >= ES_DV5) {
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_1pul,
								sizeof(timing_switch_cmd_sdc90_1pul) / sizeof(struct LCM_setting_table), cb, NULL);
						} else {
							lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
						}
					} else if (get_panel_es_ver() == ES_DV4) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
					} else if (get_panel_es_ver() == ES_DV5) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_dv5) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_dv5, lcm_cmd_count, cb, NULL);
					}
				}
				panel_mode_id = FHD_SDC90;
				pwm_params->pwm_fps_mode = 90;
			} else if (m_vrefresh == 120) {
				pr_info("timing switch to sdc120\n");
				if (get_panel_es_ver() == ES_DV2) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120_dv2) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120_dv2, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() == ES_DV3) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120_dv3) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120_dv3, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() >= ES_DV4) {
					if (en && oplus_display_brightness > plus_bl) {
						if (m_db >= ES_DV5) {
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120_1pul,
								sizeof(timing_switch_cmd_sdc120_1pul) / sizeof(struct LCM_setting_table), cb, NULL);
						} else {
							lcm_cmd_count = sizeof(timing_switch_cmd_sdc120) / sizeof(struct LCM_setting_table);
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120, lcm_cmd_count, cb, NULL);
						}
					} else if (get_panel_es_ver() == ES_DV4) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc120) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120, lcm_cmd_count, cb, NULL);
					} else if (get_panel_es_ver() == ES_DV5) {
						lcm_cmd_count = sizeof(timing_switch_cmd_sdc120_dv5) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120_dv5, lcm_cmd_count, cb, NULL);
					}
				}
				panel_mode_id = FHD_SDC120;
				pwm_params->pwm_fps_mode = 120;
			}
			//oplus_adfr_set_multite_state(false);
		} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
			if (m_vrefresh == 120) {
				/* send OA120 timing-switch cmd */
				pr_info("timing switch to oa120\n");
				if (get_panel_es_ver() == ES_DV2) {
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120_dv2) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120_dv2, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() == ES_DV3) {
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120_dv3) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120_dv3, lcm_cmd_count, cb, NULL);
				} else if (get_panel_es_ver() >= ES_DV4) {
					if (en && oplus_display_brightness > plus_bl) {
						if (m_db >= ES_DV5) {
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120_1pul,
								sizeof(timing_switch_cmd_oa120_1pul) / sizeof(struct LCM_setting_table), cb, NULL);
						} else {
							lcm_cmd_count = sizeof(timing_switch_cmd_oa120) / sizeof(struct LCM_setting_table);
							panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120, lcm_cmd_count, cb, NULL);
						}
					} else if (get_panel_es_ver() == ES_DV4) {
						lcm_cmd_count = sizeof(timing_switch_cmd_oa120) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120, lcm_cmd_count, cb, NULL);
					} else if (get_panel_es_ver() == ES_DV5) {
						lcm_cmd_count = sizeof(timing_switch_cmd_oa120_dv5) / sizeof(struct LCM_setting_table);
						panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120_dv5, lcm_cmd_count, cb, NULL);
					}
				}
				panel_mode_id = FHD_OPLUS120;
				pwm_params->pwm_fps_mode = 120;
			}
		}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
		if (oplus_adfr_is_support()) {
			// reset adfr auto mode status as panel mode will be change after timing switch
			oplus_adfr_status_reset(src_m, m);
		}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
	}


	if (ext->params->data_rate != last_data_rate) {
		ret = 1;
		pr_info("need to change mipi clk, data_rate=%d,last_data_rate=%d\n",ext->params->data_rate,last_data_rate);
		last_data_rate = ext->params->data_rate;
	}

	return ret;
}

static int oplus_display_panel_set_hbm_max(void *dsi, dcs_write_gce_pack cb1, dcs_write_gce cb2, void *handle, unsigned int en)
{
	unsigned int lcm_cmd_count = 0;
	unsigned int i = 0;
	struct LCM_setting_table *table = NULL;
	bool enable = oplus_panel_pwm_onepulse_is_enabled();

	pr_info("[DISP][INFO][%s: en=%d\n", __func__, en);
	if (!dsi || !cb1 || !cb2) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	if (enable) {
		pr_info("[DISP][INFO][%s: 1pulse en=%d, skip set apl\n", __func__, enable);
		return -EINVAL;
	}

	if (en) {
		table = dsi_switch_hbm_apl_on;
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_on) / sizeof(struct LCM_setting_table);
		for (i = 0; i < lcm_cmd_count; i++) {
			cb2(dsi, handle, table[i].para_list, table[i].count);
		}
		last_backlight = 4095;
		pr_info("[DISP][INFO]Enter hbm max mode, set last_backlight as %d", last_backlight);
	} else if (!en) {
		table = dsi_switch_hbm_apl_off;
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_off) / sizeof(struct LCM_setting_table);
		for (i = 0; i < lcm_cmd_count; i++) {
			cb2(dsi, handle, table[i].para_list, table[i].count);
		}
		pr_info("[DISP][INFO][%s: hbm_max off, restore bl:%d\n", __func__, oplus_display_brightness);
		oplus_display_panel_set_pwm_plus_bl(dsi, cb1, handle, oplus_display_brightness);
	}

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (!dsi || !cb) {
		return -EINVAL;
	}

	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &level);

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

	panel_last_brightness = level;

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
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	.set_minfps = panel_set_minfps,
	.set_multite = panel_set_multite,
#endif
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_set_hbm = lcm_set_hbm,
	.oplus_hbm_set_cmdq = panel_hbm_set_cmdq,
	.oplus_ofp_set_lhbm_pressed_icon = oplus_ofp_set_lhbm_pressed_icon,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.set_ultra_low_power_aod = panel_set_ultra_low_power_aod,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
#ifdef OPLUS_FEATURE_DISPLAY_HPWM
	.lcm_high_pwm_set_plus_bl = oplus_display_panel_set_pwm_plus_bl,
#endif /* OPLUS_FEATURE_DISPLAY_HPWM */
#ifdef OPLUS_FEATURE_DISPLAY
	.lcm_set_hbm_max = oplus_display_panel_set_hbm_max,
	.set_seed = panel_set_seed,
#endif
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

	pr_info("[LCM] ac094_p_3_a0004 %s START\n", __func__);

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
	oplus_display_panel_dbv_probe(dev);

	register_device_proc("lcd", "A0004", "P_3");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	oplus_enhance_mipi_strength = 1;
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	oplus_adfr_get_test_te_gpio(dev);
#endif
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	oplus_ofp_init(dev);
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	mutex_init(&oplus_pcp_lock);

	pr_info("[LCM] %s- lcm, AC094_P_3_A0004, END\n", __func__);

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
	mutex_destroy(&oplus_pcp_lock);
}

static const struct of_device_id lcm_of_match[] = {
	{
	    .compatible = "ac094,p,3,a0004,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel_ac094_p_3_a0004_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init ac094_p_3_a0004_drv_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register ac094_p_3_a0004_drv_init driver: %d\n",
			__func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit ac094_p_3_a0004_drv_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}
module_init(ac094_p_3_a0004_drv_init);
module_exit(ac094_p_3_a0004_drv_exit);


MODULE_AUTHOR("LiPing-m");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
