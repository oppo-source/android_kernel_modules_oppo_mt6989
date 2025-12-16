// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 OPPO Inc.
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

#ifdef OPLUS_FEATURE_DISPLAY
#include "../../oplus/oplus_drm_disp_panel.h"
#include "../../oplus/oplus_display_temp_compensation.h"
#include "../../oplus/oplus_display_mtk_debug.h"
#endif /* OPLUS_FEATURE_DISPLAY */

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "../../oplus/oplus_adfr_ext.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "../../oplus/oplus_display_onscreenfingerprint.h"
#include "../../mediatek/mediatek_v2/mtk-cmdq-ext.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#ifdef OPLUS_FEATURE_DISPLAY_HPWM
#include "../../oplus/oplus_display_high_frequency_pwm.h"
#endif /* OPLUS_FEATURE_DISPLAY_HPWM */

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mtk_round_corner/data_hw_roundedpattern_ab946.h"
#endif
#include "panel_ab946_p_3_a0020_dsi_cmd_bcp.h"

#define LCM_ERR(fmt, arg...)	\
	do {	\
		DISP_ERR("[%s] "pr_fmt(fmt), LCM_TAG, ##arg);	\
	} while (0)
#define LCM_WARN(fmt, arg...)	\
	do {	\
		DISP_WARN("[%s] "pr_fmt(fmt), LCM_TAG, ##arg);	\
	} while (0)
#define LCM_INFO(fmt, arg...)	\
	do {	\
		DISP_INFO("[%s] "pr_fmt(fmt), LCM_TAG, ##arg);	\
	} while (0)
#define LCM_DEBUG(fmt, arg...)	\
	do {	\
		DISP_DEBUG("[%s] "pr_fmt(fmt), LCM_TAG, ##arg);	\
	} while (0)
#define LCM_BACKLIGHT(fmt, arg...)	\
	do {	\
		DISP_BACKLIGHT("[%s] "pr_fmt(fmt), LCM_TAG, ##arg);	\
	} while (0)

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
extern void lcdinfo_notify(unsigned long val, void *v);
extern void oplus_pcp_handle(bool cmd_is_pcp,  void *handle);
extern int oplus_serial_number_probe(struct device *dev);
extern inline void set_pwm_turbo_switch_state(enum PWM_SWITCH_STATE state);
extern void set_pwm_turbo_power_on(bool en);
extern struct oplus_pwm_turbo_params *oplus_pwm_turbo_get_params(void);
extern inline bool oplus_panel_pwm_onepulse_is_enabled(void);
DEFINE_MUTEX(oplus_pwm_lock);

static struct regulator *vci_ldo;
static struct regulator *vddi_ldo;
static unsigned int panel_mode_id = FHD_SDC120;    // default 120fps
static unsigned int temp_seed_mode = 0;
static enum RES_SWITCH_TYPE res_switch_type = RES_SWITCH_NO_USE;
static int panel_send_pack_hs_cmd(void *dsi,
		struct LCM_setting_table *table,
		unsigned int lcm_cmd_count,
		dcs_write_gce_pack cb,
		void *handle);
void oplus_display_panel_pwm_switch_prepare(unsigned int level);

enum PANEL_ES {
	ES_DV2 = 2,
	ES_DV3 = 3,
	ES_DV4 = 4,
	ES_DV5 = 5,
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr_en_gpio;
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
	if (m_db > 0 && m_db <4)
		ret = ES_DV2;
	else if (m_db >= 4)
		ret = ES_DV3;
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

	LCM_INFO("Execute\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		LCM_INFO("0x%08X\n", buffer[0] | (buffer[1] << 8));
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

	LCM_INFO("cmd count=%d\n", count);

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

static int lcm_panel_power_init(struct lcm *ctx)
{
	int ret = 0;

	LCM_INFO("Execute\n");

	vddi_ldo = devm_regulator_get(ctx->dev, "vddi");
	if (IS_ERR(vddi_ldo)) {
		LCM_ERR("cannot get vddi_ldo, ret=%ld\n", PTR_ERR(vddi_ldo));
		return PTR_ERR(vddi_ldo);
	}

	ctx->vddr_en_gpio = devm_gpiod_get(ctx->dev, "vddr_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr_en_gpio)) {
		LCM_ERR("cannot get vddr_en_gpio, ret=%ld\n", PTR_ERR(ctx->vddr_en_gpio));
		return PTR_ERR(ctx->vddr_en_gpio);
	}

	vci_ldo = devm_regulator_get(ctx->dev, "vci");
	if (IS_ERR(vci_ldo)) {
		LCM_ERR("cannot get vci_ldo, ret=%ld\n", PTR_ERR(vci_ldo));
		return PTR_ERR(vci_ldo);
	}

	return ret;
}

static int lcm_panel_vddi_ldo_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	LCM_INFO("Execute\n");

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vddi_ldo)) {
		ret = regulator_set_voltage(vddi_ldo, 1800000, 1800000);
		if (ret < 0)
			LCM_ERR("set voltage vddi_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	/* enable regulator */
	if (!IS_ERR_OR_NULL(vddi_ldo)) {
		LCM_INFO("enable regulator vddi_ldo\n");
		ret = regulator_enable(vddi_ldo);
		if (ret < 0)
			LCM_ERR("enable regulator vddi_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}

	return retval;
}

static int lcm_panel_vddi_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	LCM_INFO("Execute\n");

	if (IS_ERR_OR_NULL(vddi_ldo)) {
		LCM_ERR("disable regulator fail, vddi_ldo is null\n");
	}

	if (!IS_ERR_OR_NULL(vddi_ldo)) {
		LCM_INFO("disable regulator vddi_ldo\n");
		ret = regulator_disable(vddi_ldo);
		if (ret < 0)
			LCM_ERR("disable regulator vddi_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}

	return ret;
}

static int lcm_panel_vci_ldo_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	LCM_INFO("Execute\n");

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vci_ldo)) {
		ret = regulator_set_voltage(vci_ldo, 3000000, 3000000);
		if (ret < 0)
			LCM_ERR("voltage vci_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	/* enable regulator */
	if (!IS_ERR_OR_NULL(vci_ldo)) {
		LCM_INFO("enable regulator vci_ldo\n");
		ret = regulator_enable(vci_ldo);
		if (ret < 0)
			LCM_ERR("enable regulator vci_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}

	return retval;
}

static int lcm_panel_vci_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	LCM_INFO("Execute\n");

	if (IS_ERR_OR_NULL(vci_ldo)) {
		LCM_ERR("disable regulator fail, vci_ldo is null\n");
	}

	if (!IS_ERR_OR_NULL(vci_ldo)) {
		LCM_INFO("disable regulator vci_ldo\n");
		ret = regulator_disable(vci_ldo);
		if (ret < 0)
			LCM_ERR("disable regulator vci_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}

	return ret;
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		LCM_WARN("get_mode_enum drm_display_mode *m is null, default 120fps\n");
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
	struct drm_display_mode *m = ctx->m;

	mode_id = get_mode_enum(m);
	LCM_INFO("mode id=%d\n", mode_id);

	switch (mode_id) {
	case FHD_SDC60:
		LCM_INFO("fhd_dsi_on_cmd_sdc60\n");
		push_table(ctx, dsi_on_cmd_sdc60, sizeof(dsi_on_cmd_sdc60) / sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC90:
		LCM_INFO("fhd_dsi_on_cmd_sdc90\n");
		push_table(ctx, dsi_on_cmd_sdc90, sizeof(dsi_on_cmd_sdc90) / sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC120:
		LCM_INFO("fhd_dsi_on_cmd_sdc120\n");
		push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120) / sizeof(struct LCM_setting_table));
		break;
	case FHD_OPLUS120:
		LCM_INFO("fhd_dsi_on_cmd_oplus120\n");
		push_table(ctx, dsi_on_cmd_oa120, sizeof(dsi_on_cmd_oa120) / sizeof(struct LCM_setting_table));
		break;
	default:
		LCM_INFO(" default mode_id\n");
		push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120) / sizeof(struct LCM_setting_table));
		break;
	}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	if (oplus_adfr_is_support()) {
		// reset adfr auto mode status as auto mode will be change after power on
		oplus_adfr_status_reset(NULL, m);
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	LCM_INFO("Execute\n");

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

	if (!ctx->prepared)
		return 0;

	LCM_INFO("Start\n");

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
	}

	usleep_range(5000, 5100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(25000, 25100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(125000, 125100);
	LCM_INFO("clear pulse status flag\n");
	pulse_flg = false;

	while (atomic_read(&oplus_pcp_handle_lock) > 0) {
		LCM_INFO("atommic ++ %d\n", atomic_read(&oplus_pcp_handle_lock));
		atomic_dec(&oplus_pcp_handle_lock);
		LCM_INFO("atommic -- %d\n", atomic_read(&oplus_pcp_handle_lock));
		mutex_unlock(&oplus_pcp_lock);
		atomic_dec(&oplus_pcp_num);
//		oplus_disp_trace_end("M_LOCK_PCP");
		LCM_INFO("oplus_pcp_unlock\n");
	}
	LCM_INFO("oplus_pcp_handle_lock = %d, oplus_pcp_num = %d\n",
		atomic_read(&oplus_pcp_handle_lock),
		atomic_read(&oplus_pcp_num));

	ctx->error = 0;
	ctx->prepared = false;
	LCM_INFO("End\n");

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	LCM_INFO("Start\n");

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	LCM_INFO("End\n");

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	LCM_INFO("Execute\n");

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
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + 9,//HFP
		.hsync_end = FRAME_WIDTH + 9 + 2,//HSA
		.htotal = FRAME_WIDTH + 9 + 2 + 21,//HBP
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + 52,//VFP
		.vsync_end = FRAME_HEIGHT + 52 + 14,//VSA
		.vtotal = FRAME_HEIGHT + 52+ 14 + 22,//VBP
		.hskew = SDC_ADFR,
	},
	//sdc_60_mode
	{
		.clock = 223015,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + 9,//HFP
		.hsync_end = FRAME_WIDTH + 9 + 2,//HSA
		.htotal = FRAME_WIDTH + 9 + 2 + 21,//HBP
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + 52,//VFP
		.vsync_end = FRAME_HEIGHT + 52 + 14,//VSA
		.vtotal = FRAME_HEIGHT + 52+ 14 + 22,//VBP
		.hskew = SDC_MFR,
	},
	//sdc_90_mode
	{
		.clock = 334523,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + 9,//HFP
		.hsync_end = FRAME_WIDTH + 9 + 2,//HSA
		.htotal = FRAME_WIDTH + 9 + 2 + 21,//HBP
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + 52,//VFP
		.vsync_end = FRAME_HEIGHT + 52 + 14,//VSA
		.vtotal = FRAME_HEIGHT + 52+ 14 + 22,//VBP
		.hskew = SDC_ADFR,
	},
	//oa_120_mode
	{
		.clock = 446031,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + 9,//HFP
		.hsync_end = FRAME_WIDTH + 9 + 2,//HSA
		.htotal = FRAME_WIDTH + 9 + 2 + 21,//HBP
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + 52,//VFP
		.vsync_end = FRAME_HEIGHT + 52 + 14,//VSA
		.vtotal = FRAME_HEIGHT + 52+ 14 + 22,//VBP
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
	.pll_clk = 498,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

#endif
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
	.color_nature_profession_status = false,
	.vendor = VERSION,
	.manufacture = MANUFACTURE,
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
			.enable               = DSC_ENABLE,
			.ver                  = DSC_VER,
			.slice_mode           = DSC_SLICE_MODE,
			.rgb_swap             = DSC_RGB_SWAP,
			.dsc_cfg              = DSC_DSC_CFG,
			.rct_on               = DSC_RCT_ON,
			.bit_per_channel      = DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth   = DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable            = DSC_BP_ENABLE,
			.bit_per_pixel        = DSC_BIT_PER_PIXEL,
			.pic_height           = FHD_FRAME_HEIGHT,
			.pic_width            = FHD_FRAME_WIDTH,
			.slice_height         = DSC_SLICE_HEIGHT,
			.slice_width          = DSC_SLICE_WIDTH,
			.chunk_size           = DSC_CHUNK_SIZE,
			.xmit_delay           = DSC_XMIT_DELAY,
			.dec_delay            = DSC_DEC_DELAY,
			.scale_value          = DSC_SCALE_VALUE,
			.increment_interval   = DSC_INCREMENT_INTERVAL,
			.decrement_interval   = DSC_DECREMENT_INTERVAL,
			.line_bpg_offset      = DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset       = DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset     = DSC_SLICE_BPG_OFFSET,
			.initial_offset       = DSC_INITIAL_OFFSET,
			.final_offset         = DSC_FINAL_OFFSET,
			.flatness_minqp       = DSC_FLATNESS_MINQP,
			.flatness_maxqp       = DSC_FLATNESS_MAXQP,
			.rc_model_size        = DSC_RC_MODEL_SIZE,
			.rc_edge_factor       = DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0 = DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1 = DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi     = DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo     = DSC_RC_TGT_OFFSET_LO,
		},
	.data_rate = 997,
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
	.pll_clk = 498,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

#endif
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
	.color_nature_profession_status = false,
	.vendor = VERSION,
	.manufacture = MANUFACTURE,
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
			.enable               = DSC_ENABLE,
			.ver                  = DSC_VER,
			.slice_mode           = DSC_SLICE_MODE,
			.rgb_swap             = DSC_RGB_SWAP,
			.dsc_cfg              = DSC_DSC_CFG,
			.rct_on               = DSC_RCT_ON,
			.bit_per_channel      = DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth   = DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable            = DSC_BP_ENABLE,
			.bit_per_pixel        = DSC_BIT_PER_PIXEL,
			.pic_height           = FHD_FRAME_HEIGHT,
			.pic_width            = FHD_FRAME_WIDTH,
			.slice_height         = DSC_SLICE_HEIGHT,
			.slice_width          = DSC_SLICE_WIDTH,
			.chunk_size           = DSC_CHUNK_SIZE,
			.xmit_delay           = DSC_XMIT_DELAY,
			.dec_delay            = DSC_DEC_DELAY,
			.scale_value          = DSC_SCALE_VALUE,
			.increment_interval   = DSC_INCREMENT_INTERVAL,
			.decrement_interval   = DSC_DECREMENT_INTERVAL,
			.line_bpg_offset      = DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset       = DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset     = DSC_SLICE_BPG_OFFSET,
			.initial_offset       = DSC_INITIAL_OFFSET,
			.final_offset         = DSC_FINAL_OFFSET,
			.flatness_minqp       = DSC_FLATNESS_MINQP,
			.flatness_maxqp       = DSC_FLATNESS_MAXQP,
			.rc_model_size        = DSC_RC_MODEL_SIZE,
			.rc_edge_factor       = DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0 = DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1 = DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi     = DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo     = DSC_RC_TGT_OFFSET_LO,
		},
	.data_rate = 997,
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
	.pll_clk = 498,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

#endif
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
	.color_nature_profession_status = false,
	.vendor = VERSION,
	.manufacture = MANUFACTURE,
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
			.enable               = DSC_ENABLE,
			.ver                  = DSC_VER,
			.slice_mode           = DSC_SLICE_MODE,
			.rgb_swap             = DSC_RGB_SWAP,
			.dsc_cfg              = DSC_DSC_CFG,
			.rct_on               = DSC_RCT_ON,
			.bit_per_channel      = DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth   = DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable            = DSC_BP_ENABLE,
			.bit_per_pixel        = DSC_BIT_PER_PIXEL,
			.pic_height           = FHD_FRAME_HEIGHT,
			.pic_width            = FHD_FRAME_WIDTH,
			.slice_height         = DSC_SLICE_HEIGHT,
			.slice_width          = DSC_SLICE_WIDTH,
			.chunk_size           = DSC_CHUNK_SIZE,
			.xmit_delay           = DSC_XMIT_DELAY,
			.dec_delay            = DSC_DEC_DELAY,
			.scale_value          = DSC_SCALE_VALUE,
			.increment_interval   = DSC_INCREMENT_INTERVAL,
			.decrement_interval   = DSC_DECREMENT_INTERVAL,
			.line_bpg_offset      = DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset       = DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset     = DSC_SLICE_BPG_OFFSET,
			.initial_offset       = DSC_INITIAL_OFFSET,
			.final_offset         = DSC_FINAL_OFFSET,
			.flatness_minqp       = DSC_FLATNESS_MINQP,
			.flatness_maxqp       = DSC_FLATNESS_MAXQP,
			.rc_model_size        = DSC_RC_MODEL_SIZE,
			.rc_edge_factor       = DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0 = DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1 = DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi     = DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo     = DSC_RC_TGT_OFFSET_LO,
		},
	.data_rate = 997,
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
	.pll_clk = 498,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

#endif
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
	.color_nature_profession_status = false,
	.vendor = VERSION,
	.manufacture = MANUFACTURE,
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
			.enable               = DSC_ENABLE,
			.ver                  = DSC_VER,
			.slice_mode           = DSC_SLICE_MODE,
			.rgb_swap             = DSC_RGB_SWAP,
			.dsc_cfg              = DSC_DSC_CFG,
			.rct_on               = DSC_RCT_ON,
			.bit_per_channel      = DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth   = DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable            = DSC_BP_ENABLE,
			.bit_per_pixel        = DSC_BIT_PER_PIXEL,
			.pic_height           = FHD_FRAME_HEIGHT,
			.pic_width            = FHD_FRAME_WIDTH,
			.slice_height         = DSC_SLICE_HEIGHT,
			.slice_width          = DSC_SLICE_WIDTH,
			.chunk_size           = DSC_CHUNK_SIZE,
			.xmit_delay           = DSC_XMIT_DELAY,
			.dec_delay            = DSC_DEC_DELAY,
			.scale_value          = DSC_SCALE_VALUE,
			.increment_interval   = DSC_INCREMENT_INTERVAL,
			.decrement_interval   = DSC_DECREMENT_INTERVAL,
			.line_bpg_offset      = DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset       = DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset     = DSC_SLICE_BPG_OFFSET,
			.initial_offset       = DSC_INITIAL_OFFSET,
			.final_offset         = DSC_FINAL_OFFSET,
			.flatness_minqp       = DSC_FLATNESS_MINQP,
			.flatness_maxqp       = DSC_FLATNESS_MAXQP,
			.rc_model_size        = DSC_RC_MODEL_SIZE,
			.rc_edge_factor       = DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0 = DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1 = DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi     = DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo     = DSC_RC_TGT_OFFSET_LO,
		},
	.data_rate = 997,
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

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};
	unsigned int i = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (last_backlight == 0 || level == 0) {
		LCM_INFO("backlight level:%u\n", level);
	} else {
		LCM_BACKLIGHT("backlight level:%u\n", level);
	}

	if (oplus_panel_pwm_onepulse_is_enabled()) {
		oplus_display_panel_pwm_switch_prepare(level);
	}

	if (level <= 10 && last_backlight > BRIGHTNESS_HALF) {
		LCM_INFO("Vinit2/ELVSS dimming setting.\n");
		for(i = 0; i < (sizeof(dimming_setting_cmd) / sizeof(struct LCM_setting_table)); i++) {
			cb(dsi, handle, dimming_setting_cmd[i].para_list, dimming_setting_cmd[i].count);
		}
	}

	last_backlight = level;
	mapped_level = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);

	if (level == 1) {
		LCM_INFO("filter backlight %u setting\n", level);
		return 0;
	} else if (level > BRIGHTNESS_LCD_MAX) {
		level = BRIGHTNESS_LCD_MAX;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = BRIGHTNESS_HALF;
	}

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));

	return 0;
}

#ifdef OPLUS_FEATURE_DISPLAY_HPWM
void oplus_display_panel_pwm_switch_prepare(unsigned int level) {
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	bool condition1 = (last_backlight <= get_pwm_turbo_plus_bl()) && (level > get_pwm_turbo_plus_bl());
	bool condition2 = (last_backlight > get_pwm_turbo_plus_bl()) && (level <= get_pwm_turbo_plus_bl());

	if (!oplus_panel_pwm_onepulse_is_enabled() && (last_backlight > 10) && (level > 10)) {
		if (condition1) {
			pulse_flg = true;
			pwm_params->pwm_pul_cmd_id = PWM_SWITCH_18TO3;
			set_pwm_turbo_switch_state(PWM_SWITCH_DC_STATE);
		} else if (condition2) {
			pulse_flg = true;
			pwm_params->pwm_pul_cmd_id = PWM_SWITCH_3TO18;
			set_pwm_turbo_switch_state(PWM_SWITCH_HPWM_STATE);
		}
	} else if (oplus_panel_pwm_onepulse_is_enabled() && (last_backlight <= 1) && (level > 1)) {
		LCM_INFO("Send pwm_switch_3to1 when power-on\n");
		pulse_flg = true;
		pwm_params->pwm_pul_cmd_id = PWM_SWITCH_3TO1;
		set_pwm_turbo_switch_state(PWM_SWITCH_ONEPULSE_STATE);
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
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to3pul_120hz, sizeof(dsi_pwm_switch_18to3pul_120hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to3pul_90hz, sizeof(dsi_pwm_switch_18to3pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to3pul_60hz, sizeof(dsi_pwm_switch_18to3pul_60hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			}
		}
		break;
		case PWM_SWITCH_3TO18: {
			pr_info("pwm_turbo dsi_pwm_switch 3to18\n");
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_3to18pul_120hz, sizeof(dsi_pwm_switch_3to18pul_120hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_3to18pul_90hz, sizeof(dsi_pwm_switch_3to18pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
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
			pr_info("pwm_turbo onepulse dsi_pwm_switch 18to1 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to1pul_120hz, sizeof(dsi_pwm_switch_18to1pul_120hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to1pul_90hz, sizeof(dsi_pwm_switch_18to1pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_18to1pul_60hz, sizeof(dsi_pwm_switch_18to1pul_60hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			}
		}
		break;
		case PWM_SWITCH_1TO18: {
			pr_info("pwm_turbo onepulse dsi_pwm_switch 1to18 panel_mode_id:%d\n", panel_mode_id);
			if (panel_mode_id == FHD_SDC120 || panel_mode_id == FHD_OPLUS120) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_1to18pul_120hz, sizeof(dsi_pwm_switch_1to18pul_120hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC90) {
				panel_send_pack_hs_cmd(dsi, dsi_pwm_switch_1to18pul_90hz, sizeof(dsi_pwm_switch_1to18pul_90hz) /
					sizeof(struct LCM_setting_table), cb, handle);
			} else if (panel_mode_id == FHD_SDC60) {
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

static int oplus_display_panel_set_pwm_plus_bl(void *dsi, dcs_write_gce_pack cb, void *handle, unsigned int level)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (last_backlight == 0 || level == 0) {
		LCM_INFO("backlight level:%u\n", level);
	} else {
		LCM_BACKLIGHT("backlight level:%u\n", level);
	}

	if (level > BRIGHTNESS_LCD_MAX) {
		level = BRIGHTNESS_LCD_MAX;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = BRIGHTNESS_HALF;
	}

	if (silence_mode) {
		LCM_INFO("pwm_turbo silence_mode is %d, set backlight to 0\n", silence_mode);
		level = 0;
	}

	if (level <= 10 && last_backlight > BRIGHTNESS_HALF) {
		LCM_INFO("Vinit2/ELVSS dimming setting.\n");
		panel_send_pack_hs_cmd(dsi, dimming_setting_cmd, sizeof(dimming_setting_cmd) / sizeof(struct LCM_setting_table), cb, handle);
	}

	oplus_display_panel_pwm_switch_prepare(level);

	last_backlight = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &last_backlight);

	if (level == 1) {
		LCM_INFO("filter backlight %u setting\n", level);
		return 0;
	}

	if (pulse_flg) {
		oplus_display_panel_set_pwm_pul(dsi, cb, handle, pwm_params->pwm_pul_cmd_id);
		pulse_flg = false;
	}

	dsi_set_backlight[1].para_list[1] = level >> 8;
	dsi_set_backlight[1].para_list[2] = level & 0xFF;
	panel_send_pack_hs_cmd(dsi, dsi_set_backlight, sizeof(dsi_set_backlight) / sizeof(struct LCM_setting_table), cb, handle);

	return 0;
}

#endif

static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int mode)
{
	unsigned int i = 0;

	LCM_INFO("mode=%d\n", mode);

	if (!dsi || !cb) {
		LCM_ERR("Invalid params\n");
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
		last_backlight = BRIGHTNESS_LCD_MAX;
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
		last_backlight = BRIGHTNESS_LCD_MAX;
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
		oplus_display_panel_set_pwm_plus_bl(dsi_drv, cb, handle, oplus_display_brightness);
	}

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	bool is_pcp = false;
	unsigned int i = 0;
	unsigned int reg_count = 0;
	int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct mtk_dsi *mtk_dsi = dsi;
	struct LCM_setting_table *aod_off_cmd = NULL;
	struct drm_crtc *crtc = NULL;

	if (!panel || !mtk_dsi) {
		OFP_ERR("Invalid mtk_dsi params\n");
	}

	crtc = mtk_dsi->encoder.crtc;

	if (!crtc) {
		OFP_ERR("Invalid crtc param\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		LCM_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}
	if (oplus_panel_pwm_onepulse_is_enabled() && (last_backlight > 1) && (oplus_display_brightness > 1)) {
	if (vrefresh_rate == 60) {
			aod_off_cmd = aod_off_onepulse_cmd_60hz;
			reg_count = sizeof(aod_off_onepulse_cmd_60hz) / sizeof(struct LCM_setting_table);
		} else if (vrefresh_rate == 120) {
			aod_off_cmd = aod_off_onepulse_cmd_120hz;
			reg_count = sizeof(aod_off_onepulse_cmd_120hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_onepulse_cmd_90hz;
			reg_count = sizeof(aod_off_onepulse_cmd_90hz) / sizeof(struct LCM_setting_table);
		}
	} else {
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
	}
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

	OFP_INFO("send aod off cmd, onepulse_is_enabled: %d, last_backlight: %d, oplus_display_brightness: %d.\n",
		oplus_panel_pwm_onepulse_is_enabled(), last_backlight, oplus_display_brightness);

	if (is_pcp) {
		atomic_inc(&oplus_pcp_handle_lock);
	}

	if(!oplus_ofp_backlight_filter(crtc, handle, oplus_display_brightness))
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);

	if (temp_seed_mode)
		panel_set_seed(dsi, cb, handle, temp_seed_mode);

	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	bool is_pcp = false;
	unsigned int i = 0;

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
		LCM_INFO("default refresh rate is 120hz\n");
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

	LCM_INFO("Execute\n");

	gpiod_set_value(ctx->reset_gpio, on);

	return 0;
}

static int lcm_panel_poweron_sub(struct lcm *ctx)
{
	LCM_INFO("Execute\n");

	/* enable vddi */
	lcm_panel_vddi_ldo_enable(ctx->dev);
	usleep_range(5000, 5100);

	/* enable vddr */
	gpiod_set_value(ctx->vddr_en_gpio, 1);
	usleep_range(5000, 5100);

	/* enable vci */
	lcm_panel_vci_ldo_enable(ctx->dev);

	return ctx->error;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	LCM_INFO("Execute\n");

	ret = lcm_panel_poweron_sub(ctx);
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(1000, 1100);

	return 0;
}

static int lcm_panel_poweroff_sub(struct lcm *ctx)
{
	LCM_INFO("Execute\n");

	usleep_range(5000, 5100);
	/* disable reset */
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 5100);

	/* disable vci */
	lcm_panel_vci_ldo_disable(ctx->dev);
	usleep_range(5000, 5100);

	/* disable vddr */
	gpiod_set_value(ctx->vddr_en_gpio, 0);
	usleep_range(15000, 15100);

	/* disable vddi */
	lcm_panel_vddi_ldo_disable(ctx->dev);

	return ctx->error;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	LCM_INFO("Execute\n");

	ret = lcm_panel_poweroff_sub(ctx);
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

	LCM_INFO("Execute\n");

	usleep_range(25000, 25100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(1000, 1100);
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
		LCM_DEBUG("data_rate:%d\n", (*ext_param)->data_rate);
	else
		LCM_ERR("ext_param is NULL;\n");

	return ret;
}

enum RES_SWITCH_TYPE mtk_get_res_switch_type(void)
{
	LCM_INFO("res_switch_type: %d\n", res_switch_type);
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
	LCM_INFO("mode=%d, vrefresh=%d, hskew=%d\n", mode, drm_mode_vrefresh(m), m->hskew);

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

	LCM_DEBUG("Execute\n");

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		LCM_ERR("out of mtk_ddic_dsi_cmd \n");
		return 0;
	}

	is_pcp = oplus_display_is_pcp(table, lcm_cmd_count);
	if (table == lhbm_pressed_icon_off_cmd) {
		is_pcp = true;
		LCM_INFO("lhbm_pressed_icon_off, is_pcp:%d\n", is_pcp);
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
		switch (vrefresh_rate) {
		case 120:
			auto_off_minfps_cmd_120hz[MIN_FPS_CMD_ROW0].para_list[MIN_FPS_CMD_COL0] = ext_frame;
			auto_off_minfps_cmd_120hz[MIN_FPS_CMD_ROW1].para_list[MIN_FPS_CMD_COL1] = ext_frame;
			lcm_cmd_count = sizeof(auto_off_minfps_cmd_120hz) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_off_minfps_cmd_120hz, ext_frame:%u\n", ext_frame);
			panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_120hz, lcm_cmd_count, cb, handle);
			break;
		case 90:
			auto_off_minfps_cmd_90hz[MIN_FPS_CMD_ROW0].para_list[MIN_FPS_CMD_COL0] = ext_frame;
			auto_off_minfps_cmd_90hz[MIN_FPS_CMD_ROW1].para_list[MIN_FPS_CMD_COL1] = ext_frame;
			lcm_cmd_count = sizeof(auto_off_minfps_cmd_90hz) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_off_minfps_cmd_90hz, ext_frame:%u\n", ext_frame);
			panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_90hz, lcm_cmd_count, cb, handle);
			break;
		case 60:
			auto_off_minfps_cmd_60hz[MIN_FPS_CMD_ROW0].para_list[MIN_FPS_CMD_COL0] = ext_frame;
			auto_off_minfps_cmd_60hz[MIN_FPS_CMD_ROW1].para_list[MIN_FPS_CMD_COL1] = ext_frame;
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
	bool enable = oplus_panel_pwm_onepulse_is_enabled();
//	int plus_bl = get_pwm_turbo_plus_bl();
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	LCM_INFO("cur_mode = %d dst_mode %d\n", cur_mode, dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	if ((m->hskew == SDC_MFR || m->hskew == SDC_ADFR)
			&& (src_m->hskew == OPLUS_MFR || src_m->hskew == OPLUS_ADFR)) {
		LCM_INFO("OA to SA, send multi_te_disable\n");
		lcm_cmd_count = sizeof(multi_te_disable) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi_drv, multi_te_disable, lcm_cmd_count, cb, NULL);
	}

	g_last_mode_idx = cur_mode;

	if (stage == BEFORE_DSI_POWERDOWN) {
		m_vrefresh = drm_mode_vrefresh(m);
		src_vrefresh = drm_mode_vrefresh(src_m);
		LCM_INFO("mode_switch_hs,cur_mode:%d,dst_mode:%d,hdisplay:%d->%d,vrefresh:%d->%d,hskew:%d->%d\n",
			cur_mode, dst_mode, src_m->hdisplay, m->hdisplay, src_vrefresh, m_vrefresh, src_m->hskew, m->hskew);

		if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
			if (m_vrefresh == 60) {
				LCM_INFO("timing switch to sdc60 %dpulse\n", enable ? 1 : 3);
				if (enable) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60_1pulse) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60_1pulse, lcm_cmd_count, cb, NULL);
				} else {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60, lcm_cmd_count, cb, NULL);
				}
				panel_mode_id = FHD_SDC60;
				pwm_params->pwm_fps_mode = 60;
			} else if ((src_vrefresh == 60) && (m_vrefresh == 90)) {
				LCM_INFO("timing switch from sdc60 to sdc90, return 1\n");
				ret = 1;
			} else if ((src_vrefresh == 120) && (m_vrefresh == 90)) {
				LCM_INFO("timing switch from sdc120 to sdc90 %dpulse\n", enable ? 1 : 3);
				if (enable) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_1pulse) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_1pulse, lcm_cmd_count, cb, NULL);
				} else {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
				}
				panel_mode_id = FHD_SDC90;
				pwm_params->pwm_fps_mode = 90;
			} else if (m_vrefresh == 120) {
				LCM_INFO("timing switch to sdc120 return 1\n");
				ret = 1;
			}
			//oplus_adfr_set_multite_state(false);
		} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
			if (m_vrefresh == 120) {
				/* send OA120 timing-switch cmd */
				LCM_INFO("timing switch to oa120, return 1\n");
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
		LCM_INFO("mode_switch_hs,AFTER_DSI_POWERON,cur_mode:%d,dst_mode:%d,hdisplay:%d->%d,vrefresh:%d->%d,hskew:%d->%d\n",
			cur_mode, dst_mode, src_m->hdisplay, m->hdisplay, src_vrefresh, m_vrefresh, src_m->hskew, m->hskew);
		if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
			if ((src_vrefresh == 60) && (m_vrefresh == 90)) {
				LCM_INFO("timing switch from sdc60 to sdc90 %dpulse\n", enable ? 1 : 3);
				if (enable) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_1pulse) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_1pulse, lcm_cmd_count, cb, NULL);
				} else {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
				}
				panel_mode_id = FHD_SDC90;
				pwm_params->pwm_fps_mode = 90;
			} else if (m_vrefresh == 120) {
				LCM_INFO("timing switch to sdc120 %dpulse\n", enable ? 1 : 3);
				if (enable) {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120_1pulse) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120_1pulse, lcm_cmd_count, cb, NULL);
				} else {
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120, lcm_cmd_count, cb, NULL);
				}
				panel_mode_id = FHD_SDC120;
				pwm_params->pwm_fps_mode = 120;
			}
			//oplus_adfr_set_multite_state(false);
		} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
			if (m_vrefresh == 120) {
				/* send OA120 timing-switch cmd */
				LCM_INFO("timing switch to oa120 %dpulse\n", enable ? 1 : 3);
				if (enable) {
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120_1pulse) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120_1pulse, lcm_cmd_count, cb, NULL);
				} else {
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120, lcm_cmd_count, cb, NULL);
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
		LCM_INFO("need to change mipi clk, data_rate=%d,last_data_rate=%d\n",ext->params->data_rate,last_data_rate);
		last_data_rate = ext->params->data_rate;
	}

	return ret;
}

static int oplus_display_panel_set_hbm_max(void *dsi, dcs_write_gce_pack cb1, dcs_write_gce cb2, void *handle, unsigned int en)
{
	unsigned int lcm_cmd_count = 0;
	unsigned int i = 0;
	struct LCM_setting_table *table = NULL;
//	bool enable = oplus_panel_pwm_onepulse_is_enabled();

	LCM_INFO("en=%d\n", en);

	if (!dsi || !cb1 || !cb2) {
		LCM_ERR("Invalid params\n");
		return -EINVAL;
	}

//	if (enable) {
//		LCM_INFO("1pulse en=%d, skip set apl\n", enable);
//		return -EINVAL;
//	}

	if (en) {
		table = dsi_switch_hbm_apl_on;
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_on) / sizeof(struct LCM_setting_table);
		for (i = 0; i < lcm_cmd_count; i++) {
			cb2(dsi, handle, table[i].para_list, table[i].count);
		}
		last_backlight = BRIGHTNESS_LCD_MAX;
		LCM_INFO("Enter hbm max mode, set last_backlight as %d", last_backlight);
	} else if (!en) {
		table = dsi_switch_hbm_apl_off;
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_off) / sizeof(struct LCM_setting_table);
		for (i = 0; i < lcm_cmd_count; i++) {
			cb2(dsi, handle, table[i].para_list, table[i].count);
		}
		LCM_INFO("hbm_max off, restore bl:%d\n", oplus_display_brightness);
		oplus_display_panel_set_pwm_plus_bl(dsi, cb1, handle, oplus_display_brightness);
	}

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	LCM_BACKLIGHT("Execute\n");

	if (!dsi || !cb) {
		return -EINVAL;
	}

	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &level);

	if (level == 1) {
		LCM_INFO("filter backlight %u setting\n", level);
		return 0;
	} else if (level > BRIGHTNESS_LCD_MAX) {
		level = BRIGHTNESS_LCD_MAX;
	}

	LCM_INFO("backlight level=%u\n", level);

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
		LCM_ERR("failed to add mode %ux%ux@%u\n",
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
			LCM_ERR("not enough memory\n");
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

	LCM_INFO("Start\n");

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				LCM_ERR("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			LCM_INFO("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		LCM_ERR("skip probe due to not current lcm\n");
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

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		LCM_ERR("cannot get reset_gpio, ret=%ld\n",
				PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	/* power init */
	ret = lcm_panel_power_init(ctx);
	if (ret) {
		LCM_ERR("lcm_panel_power_init failed, ret=%d\n", ret);
		return ret;
	}
	/* power on */
	ret = lcm_panel_poweron_sub(ctx);
	if (ret)
		LCM_ERR("lcm_panel_poweron_sub failed, ret=%d\n", ret);

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

	register_device_proc("lcd", VERSION, MANUFACTURE);
	oplus_max_normal_brightness = BRIGHTNESS_NORMAL_MAX;
	oplus_enhance_mipi_strength = 1;

	oplus_serial_number_probe(dev);
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	rc = of_property_read_u32(dev->of_node, "oplus-adfr-config", &config);
	if (rc == 0) {
		oplus_adfr_config = config;
		LCM_INFO("config=%d, adfrconfig=%d\n", config, oplus_adfr_config);
	} else {
		oplus_adfr_config = 0;
		LCM_INFO("adfrconfig=%d\n", oplus_adfr_config);
	}
	oplus_adfr_get_test_te_gpio(dev);
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
#ifdef OPLUS_FEATURE_DISPLAY_HPWM
	oplus_pwm_turbo_probe(dev);
#endif /* OPLUS_FEATURE_DISPLAY_HPWM */
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	oplus_ofp_init(dev);
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	mutex_init(&oplus_pcp_lock);

	LCM_INFO("End\n");

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
		.compatible = PANEL_NAME,
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = PANEL_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	int ret = 0;

	LCM_INFO("Start\n");
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		LCM_INFO("Failed to register panel driver: %d\n", ret);

	mtk_panel_unlock();
	LCM_INFO("End, ret=%d\n", ret);

	return 0;
}

static void __exit lcm_drv_exit(void)
{
	LCM_INFO("Start\n");
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	LCM_INFO("End\n");
}
module_init(lcm_drv_init);
module_exit(lcm_drv_exit);


MODULE_AUTHOR("Six.Xu");
MODULE_DESCRIPTION("OPLUS AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
