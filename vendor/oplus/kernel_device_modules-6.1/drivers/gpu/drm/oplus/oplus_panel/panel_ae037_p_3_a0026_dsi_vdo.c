/***************************************************************
** Copyright (C), 2024, OPLUS Mobile Comm Corp., Ltd
**
** File : panel_ae037_p_3_a0026_dsi_vdo.c
** Description : oplus panel driver
** Version : 1.0
** Date : 2024/05/20
** Author : Display
******************************************************************/
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
#include "../../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#include "../../mediatek/mediatek_v2/mtk_dsi.h"
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "../../oplus/oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
#include "../../mediatek/mediatek_v2/mtk-cmdq-ext.h"

#include "panel_ae037_p_3_a0026_dsi_vdo.h"
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "../../oplus/oplus_adfr_ext.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

#define BRIGHTNESS_MAX    4094
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3648
#define LCM_BRIGHTNESS_TYPE 2
#define FHD_LCM_WIDTH 1264
#define FHD_LCM_HEIGHT 2780
#define FP_TYPE                 0x210

#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
extern unsigned int silence_mode;
extern unsigned int last_backlight;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
/* extern unsigned int oplus_max_brightness; */
extern unsigned int oplus_enhance_mipi_strength;
extern unsigned int m_db;
extern unsigned int m_dc;
static unsigned int panel_mode_id = FHD_SDC120;    /* default 120fps */
extern int g_last_mode_idx;
extern int oplus_display_panel_dbv_probe(struct device *dev);

#ifdef OPLUS_FEATURE_DISPLAY
/* extern int oplus_panel_parse(struct device_node *node, bool is_primary); */
#endif /* OPLUS_FEATURE_DISPLAY */
extern void lcdinfo_notify(unsigned long val, void *v);
static bool panel_power_on = false;
static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle);


static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	/*HBM ON*/
	{REGFLAG_CMD, 3, {0x51, 0x0E, 0x88}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


enum PANEL_ES {
	ES_DV1 = 1,
	ES_DV2 = 2,
	ES_DV3 = 3,
	ES_DV4 = 4,
	ES_DV5 = 5,
};

struct lcm_pmic_info {
	struct regulator *reg_vufs18;
	struct regulator *reg_vmch3p0;
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
	struct gpio_desc *vci_enable_gpio;
	struct gpio_desc *vddi_enable_gpio;
	struct drm_display_mode *m;
	struct gpio_desc *te_switch_gpio, *te_out_gpio;
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
	if (m_db == 1) {
		ret = ES_DV1;
	} else if (m_db == 2) {
		ret = ES_DV2;
	} else if (m_db == 5) {
		ret = ES_DV3;
	} else {
		ret = ES_DV4;
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
		pr_err("error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("+\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("0x%08x\n", buffer[0] | (buffer[1] << 8));
		pr_info("return %d data(0x%08x) to dsi engine\n",
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
		pr_err("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static struct regulator *vrf18_ldo;
static int lcm_panel_1p8_ldo_regulator_init(struct device *dev)
{
	static int regulator_1p8_inited;
	int ret = 0;

	if (regulator_1p8_inited)
		return ret;

	/* please only get regulator once in a driver */
	vrf18_ldo = devm_regulator_get(dev, "1p8");
	if (IS_ERR_OR_NULL(vrf18_ldo)) { /* handle return value */
		ret = PTR_ERR(vrf18_ldo);
		pr_err("get vrf18_ldo fail, error: %d\n", ret);
	}
	regulator_1p8_inited = 1;
	pr_info("get lcm_panel_1p8_ldo_regulator_init\n");
	return ret; /* must be 0 */
}

static int lcm_panel_1p8_ldo_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_1p8_ldo_regulator_init(dev);

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vrf18_ldo)) {
		ret = regulator_set_voltage(vrf18_ldo, 1800000, 1800000);
		if (ret < 0)
			pr_err("set voltage vrf18_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}

	/* enable regulator */
	if (!IS_ERR_OR_NULL(vrf18_ldo)) {
		ret = regulator_enable(vrf18_ldo);
		if (ret < 0)
			pr_err("enable regulator vrf18_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("get lcm_panel_1p8_ldo_enable\n");

	return retval;
}

static int lcm_panel_1p8_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_1p8_ldo_regulator_init(dev);

	if (!IS_ERR_OR_NULL(vrf18_ldo)) {
		ret = regulator_disable(vrf18_ldo);
		if (ret < 0)
			pr_err("disable regulator vrf18_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	return retval;
}

static struct regulator *wl2868c_ldo;
static int lcm_panel_wl2868c_ldo_regulator_init(struct device *dev)
{
	static int regulator_wl2868c_inited;
	int ret = 0;

	if (regulator_wl2868c_inited)
			return ret;
	pr_info("get lcm_panel_wl2868c_ldo_regulator_init\n");

	/* please only get regulator once in a driver */
	wl2868c_ldo = devm_regulator_get(dev, "3p0");
	if (IS_ERR_OR_NULL(wl2868c_ldo)) { /* handle return value */
			ret = PTR_ERR(wl2868c_ldo);
			pr_err("get wl2868c_ldo fail, error: %d\n", ret);
	}
	regulator_wl2868c_inited = 1;
	return ret; /* must be 0 */
}

static int lcm_panel_wl2868c_ldo_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_wl2868c_ldo_regulator_init(dev);

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(wl2868c_ldo)) {
		ret = regulator_set_voltage(wl2868c_ldo, 3000000, 3000000);
		if (ret < 0)
			pr_err("set voltage wl2868c_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	/* enable regulator */
	if (!IS_ERR_OR_NULL(wl2868c_ldo)) {
		ret = regulator_enable(wl2868c_ldo);
		if (ret < 0)
			pr_err("enable regulator wl2868c_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("get lcm_panel_wl2868c_ldo_enable\n");

	return retval;
}

static int lcm_panel_wl2868c_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_wl2868c_ldo_regulator_init(dev);

	if (!IS_ERR_OR_NULL(wl2868c_ldo)) {
		ret = regulator_disable(wl2868c_ldo);
		if (ret < 0)
			pr_err("disable regulator wl2868c_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	return retval;
}

static void push_table(struct lcm *ctx, struct LCM_setting_table *table, unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			usleep_range(table[i].count*1000, table[i].count*1000 + 100);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 100);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write(ctx, table[i].para_list, table[i].count);
			break;
		}
	}
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

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		pr_err("get_mode_enum drm_display_mode *m is null, default 120fps\n");
		ret = FHD_SDC120;
		return ret;
	}

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60) {
		ret = FHD_SDC60;
	} else if (m_vrefresh == 90) {
		ret = FHD_SDC90;
	} else if (m_vrefresh == 120) {
		ret = FHD_SDC120;
	} else if (m_vrefresh == 144) {
		ret = FHD_SDC144;
	} else {
		ret = FHD_SDC120;
	}

	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	panel_power_on = true;
	switch (panel_mode_id) {
	case FHD_SDC60:
		pr_info("fhd_dsi_on_cmd_sdc60\n");
		if (get_panel_es_ver() == ES_DV1)
			push_table(ctx, dsi_on_cmd_sdc60_dv1, sizeof(dsi_on_cmd_sdc60_dv1) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc60_dv2, sizeof(dsi_on_cmd_sdc60_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_sdc60_v2, sizeof(dsi_on_cmd_sdc60_v2) / sizeof(struct LCM_setting_table));
		else
			push_table(ctx, dsi_on_cmd_sdc60, sizeof(dsi_on_cmd_sdc60) / sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC90:
		pr_info("fhd_dsi_on_cmd_sdc90\n");
		if (get_panel_es_ver() == ES_DV1)
			push_table(ctx, dsi_on_cmd_sdc90_dv1, sizeof(dsi_on_cmd_sdc90_dv1) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc90_dv2, sizeof(dsi_on_cmd_sdc90_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_sdc90_v2, sizeof(dsi_on_cmd_sdc90_v2) / sizeof(struct LCM_setting_table));
		else
			push_table(ctx, dsi_on_cmd_sdc90, sizeof(dsi_on_cmd_sdc90) / sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC120:
		pr_info("fhd_dsi_on_cmd_sdc120\n");
		if (get_panel_es_ver() == ES_DV1)
			push_table(ctx, dsi_on_cmd_sdc120_dv1, sizeof(dsi_on_cmd_sdc120_dv1) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc120_dv2, sizeof(dsi_on_cmd_sdc120_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_sdc120_v2, sizeof(dsi_on_cmd_sdc120_v2) / sizeof(struct LCM_setting_table));
		else
			push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120) / sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC144:
		pr_info("fhd_dsi_on_cmd_sdc144\n");
		if (get_panel_es_ver() == ES_DV1)
			push_table(ctx, dsi_on_cmd_sdc144_dv1, sizeof(dsi_on_cmd_sdc144_dv1) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc144_dv2, sizeof(dsi_on_cmd_sdc144_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_sdc144_v2, sizeof(dsi_on_cmd_sdc144_v2) / sizeof(struct LCM_setting_table));
		else
			push_table(ctx, dsi_on_cmd_sdc144, sizeof(dsi_on_cmd_sdc144) / sizeof(struct LCM_setting_table));
		break;
	default:
		pr_info("fhd_dsi_on_cmd_sdc120_default\n");
		if (get_panel_es_ver() == ES_DV1)
			push_table(ctx, dsi_on_cmd_sdc120_dv1, sizeof(dsi_on_cmd_sdc120_dv1) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV2)
			push_table(ctx, dsi_on_cmd_sdc120_dv2, sizeof(dsi_on_cmd_sdc120_dv2) / sizeof(struct LCM_setting_table));
		else if (get_panel_es_ver() == ES_DV3)
			push_table(ctx, dsi_on_cmd_sdc120_v2, sizeof(dsi_on_cmd_sdc120_v2) / sizeof(struct LCM_setting_table));
		else
			push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120) / sizeof(struct LCM_setting_table));
		break;
	}
	pr_info("mode_id=%d\n", panel_mode_id);
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

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int vrefresh_rate = 0;
	unsigned int lcm_cmd_count = 0;
	struct LCM_setting_table *aod_off_cmd = NULL;

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

		push_table(ctx, aod_off_cmd, lcm_cmd_count);

		usleep_range(9000, 9100);
		OFP_INFO("send aod off cmd\n");
	}

	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	/* Wait 5ms */
	usleep_range(5000, 5100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	/* Wait 120ms */
	usleep_range(120000, 120100);

	ctx->error = 0;
	ctx->prepared = false;
	pr_info("%s:success\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

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

	pr_info("%s:success\n", __func__);
	return ret;
}

static const struct drm_display_mode display_mode[MODE_NUM * RES_NUM] = {
        {
		.clock = 495936,
		.hdisplay = 1280,
		.hsync_start = 1280 + 112,/* HFP */
		.hsync_end = 1280 + 112 + 4,/* HSA */
		.htotal = 1280 + 112 + 4 + 4,/* HBP */
		.vdisplay = 2800,
		.vsync_start = 2800 + 3076,/* VFP */
		.vsync_end = 2800 + 3076 + 2,/* VSA */
		.vtotal = 2800 + 3076 + 2 + 26,/* VBP */
		.hskew = SDC_MFR,
        },
        {
		.clock = 495936,
		.hdisplay = 1280,
		.hsync_start = 1280 + 112,/* HFP */
		.hsync_end = 1280 + 112 + 4,/* HSA */
		.htotal = 1280 + 112 + 4 + 4,/* HBP */
		.vdisplay = 2800,
		.vsync_start = 2800 + 1108,/* VFP */
		.vsync_end = 2800 + 1108 + 2,/* VSA */
		.vtotal = 2800 + 1108 + 2 + 26,/* VBP */
		.hskew = SDC_ADFR,
        },
        {
		.clock = 495936,
		.hdisplay = 1280,
		.hsync_start = 1280 + 112,/* HFP */
		.hsync_end = 1280 + 112 + 4,/* HSA */
		.htotal = 1280 + 112 + 4 + 4,/* HBP */
		.vdisplay = 2800,
		.vsync_start = 2800 + 124,/* VFP */
		.vsync_end = 2800 + 124 + 2,/* VSA */
		.vtotal = 2800 + 124 + 2 + 26,/* VBP */
		.hskew = SDC_ADFR,
        },
        {
		.clock = 554932,
		.hdisplay = 1280,
		.hsync_start = 1280 + 21,/* HFP */
		.hsync_end = 1280 + 21 + 4,/* HSA */
		.htotal = 1280 + 21 + 4 + 4,/* HBP */
		.vdisplay = 2800,
		.vsync_start = 2800 + 116,/* VFP */
		.vsync_end = 2800 + 116 + 2,/* VSA */
		.vtotal = 2800 + 116+ 2 + 26,/* VBP */
		.hskew = SDC_ADFR,
        },
        {
		.clock = 393552,
		.hdisplay = 1080,
		.hsync_start = 1080 + 112,/* HFP */
		.hsync_end = 1080 + 112 + 4,/* HSA */
		.htotal = 1080 + 112 + 4 + 4,/* HBP */
		.vdisplay = 2362,
		.vsync_start = 2362 + 3076,/* VFP */
		.vsync_end = 2362 + 3076 + 2,/* VSA */
		.vtotal = 2362 + 3076 + 2 + 26,/* VBP */
		.hskew = SDC_MFR,
        },
        {
		.clock = 377784,
		.hdisplay = 1080,
		.hsync_start = 1080 + 112,/* HFP */
		.hsync_end = 1080 + 112 + 4,/* HSA */
		.htotal = 1080 + 112 + 4 + 4,/* HBP */
		.vdisplay = 2362,
		.vsync_start = 2362 + 1108,/* VFP */
		.vsync_end = 2362 + 1108 + 2,/* VSA */
		.vtotal = 2362 + 1108 + 2 + 26,/* VBP */
		.hskew = SDC_ADFR,
        },
        {
		.clock = 362016,
		.hdisplay = 1080,
		.hsync_start = 1080 + 112,/* HFP */
		.hsync_end = 1080 + 112 + 4,/* HSA */
		.htotal = 1080 + 112 + 4 + 4,/* HBP */
		.vdisplay = 2362,
		.vsync_start = 2362 + 124,/* VFP */
		.vsync_end = 2362 + 124 + 2,/* VSA */
		.vtotal = 2362 + 124 + 2 + 26,/* VBP */
		.hskew = SDC_ADFR,
        },
        {
		.clock = 400198,
		.hdisplay = 1080,
		.hsync_start = 1080 + 21,/* HFP */
		.hsync_end = 1080 + 21 + 4,/* HSA */
		.htotal = 1080 + 21 + 4 + 4,/* HBP */
		.vdisplay = 2362,
		.vsync_start = 2362 + 116,/* VFP */
		.vsync_end = 2362 + 116 + 2,/* VSA */
		.vtotal = 2362 + 116+ 2 + 26,/* VBP */
		.hskew = SDC_ADFR,
        },
};

static struct mtk_panel_params ext_params[MODE_NUM] = {
	{
		.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 61 , {0xA9, 0x02, 0x08, 0xE0, 0x00, 0x00, 0x40, 0x02, 0x08, 0xE1, 0x01, 0x01, 0x20, 0x02, 0x08, 0xE2, 0x01, 0x01, 0x20, 0x02, 0x08,
				0xE1, 0x10, 0x10, 0x0B, 0x02, 0x08, 0xE2, 0x10, 0x10, 0x0B, 0x02, 0x08, 0xE1, 0x1E, 0x1E, 0x12, 0x02, 0x08, 0xE2, 0x1E, 0x1E, 0x12, 0x02, 0x08, 0xE1,
				0x2C, 0x2C, 0x08, 0x02, 0x08, 0xE2, 0x2C, 0x2C, 0x08, 0x02, 0x08, 0xE1, 0x3A, 0x3A, 0x00}},
		.dfps_cmd_table[1] = {0, 61 , {0xA9, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x00, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x06, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x06, 0x02, 0x08,
				0xE1, 0x56, 0x56, 0x04, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x04, 0x02, 0x08, 0xE1, 0x64, 0x64, 0x02, 0x02, 0x08, 0xE2, 0x64, 0x64, 0x02, 0x02, 0x08, 0xE1,
				0x72, 0x72, 0x01, 0x02, 0x08, 0xE2, 0x72, 0x72, 0x01, 0x02, 0x08, 0xE0, 0x00, 0x00, 0x41}},
		.dfps_cmd_table[2] = {0, 7 , {0xA9, 0x01, 0x00, 0x2F, 0x00, 0x00, 0x03}},
		.apollo_limit_superior_us = 10000,
		.apollo_limit_inferior_us = 13596,
		.apollo_transfer_time_us = 8200,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
	{
	.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 61 , {0xA9, 0x02, 0x08, 0xE0, 0x00, 0x00, 0x40, 0x02, 0x08, 0xE1, 0x01, 0x01, 0x30, 0x02, 0x08, 0xE2, 0x01, 0x01, 0x30, 0x02, 0x08,
				0xE1, 0x10, 0x10, 0x20, 0x02, 0x08, 0xE2, 0x10, 0x10, 0x20, 0x02, 0x08, 0xE1, 0x1E, 0x1E, 0x1B, 0x02, 0x08, 0xE2, 0x1E, 0x1E, 0x1B, 0x02, 0x08, 0xE1,
				0x2C, 0x2C, 0x11, 0x02, 0x08, 0xE2, 0x2C, 0x2C, 0x11, 0x02, 0x08, 0xE1, 0x3A, 0x3A, 0x0B}},
		.dfps_cmd_table[1] = {0, 61 , {0xA9, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x0B, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x09, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x09, 0x02, 0x08,
				0xE1, 0x56, 0x56, 0x08, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x08, 0x02, 0x08, 0xE1, 0x64, 0x64, 0x03, 0x02, 0x08, 0xE2, 0x64, 0x64, 0x03, 0x02, 0x08, 0xE1,
				0x72, 0x72, 0x02, 0x02, 0x08, 0xE2, 0x72, 0x72, 0x02, 0x02, 0x08, 0xE0, 0x00, 0x00, 0x41}},
		.dfps_cmd_table[2] = {0, 7 , {0xA9, 0x01, 0x00, 0x2F, 0x00, 0x00, 0x02}},
		.apollo_limit_superior_us = 2910,
		.apollo_limit_inferior_us = 10000,
		.apollo_transfer_time_us = 8400,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
	{
	.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 61 , {0xA9, 0x02, 0x08, 0xE0, 0x00, 0x00, 0x40, 0x02, 0x08, 0xE1, 0x01, 0x01, 0x30, 0x02, 0x08, 0xE2, 0x01, 0x01, 0x30, 0x02, 0x08,
				0xE1, 0x10, 0x10, 0x20, 0x02, 0x08, 0xE2, 0x10, 0x10, 0x20, 0x02, 0x08, 0xE1, 0x1E, 0x1E, 0x1B, 0x02, 0x08, 0xE2, 0x1E, 0x1E, 0x1B, 0x02, 0x08, 0xE1,
				0x2C, 0x2C, 0x11, 0x02, 0x08, 0xE2, 0x2C, 0x2C, 0x11, 0x02, 0x08, 0xE1, 0x3A, 0x3A, 0x0B}},
		.dfps_cmd_table[1] = {0, 61 , {0xA9, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x0B, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x09, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x09, 0x02, 0x08,
				0xE1, 0x56, 0x56, 0x08, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x08, 0x02, 0x08, 0xE1, 0x64, 0x64, 0x03, 0x02, 0x08, 0xE2, 0x64, 0x64, 0x03, 0x02, 0x08, 0xE1,
				0x72, 0x72, 0x02, 0x02, 0x08, 0xE2, 0x72, 0x72, 0x02, 0x02, 0x08, 0xE0, 0x00, 0x00, 0x41}},
		.dfps_cmd_table[2] = {0, 7 , {0xA9, 0x01, 0x00, 0x2F, 0x00, 0x00, 0x01}},
		.apollo_limit_superior_us = 0,
		.apollo_limit_inferior_us = 6898,
		.apollo_transfer_time_us = 6200,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
	{
	.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 144,
		.dfps_cmd_table[0] = {0, 61 , {0xA9, 0x02, 0x08, 0xE0, 0x00, 0x00, 0x40, 0x02, 0x08, 0xE1, 0x01, 0x01, 0x30, 0x02, 0x08, 0xE2, 0x01, 0x01, 0x30, 0x02, 0x08,
				0xE1, 0x10, 0x10, 0x20, 0x02, 0x08, 0xE2, 0x10, 0x10, 0x20, 0x02, 0x08, 0xE1, 0x1E, 0x1E, 0x1B, 0x02, 0x08, 0xE2, 0x1E, 0x1E, 0x1B, 0x02, 0x08, 0xE1,
				0x2C, 0x2C, 0x11, 0x02, 0x08, 0xE2, 0x2C, 0x2C, 0x11, 0x02, 0x08, 0xE1, 0x3A, 0x3A, 0x0B}},
		.dfps_cmd_table[1] = {0, 61 , {0xA9, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x0B, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x09, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x09, 0x02, 0x08,
				0xE1, 0x56, 0x56, 0x08, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x08, 0x02, 0x08, 0xE1, 0x64, 0x64, 0x03, 0x02, 0x08, 0xE2, 0x64, 0x64, 0x03, 0x02, 0x08, 0xE1,
				0x72, 0x72, 0x02, 0x02, 0x08, 0xE2, 0x72, 0x72, 0x02, 0x02, 0x08, 0xE0, 0x00, 0x00, 0x41}},
		.dfps_cmd_table[2] = {0, 7 , {0xA9, 0x01, 0x00, 0x2F, 0x00, 0x00, 0x00}},
		.apollo_limit_superior_us = 0,
		.apollo_limit_inferior_us = 6898,
		.apollo_transfer_time_us = 6200,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
};

static struct mtk_panel_params ext_params_dv1[MODE_NUM] = {
	{
		.pll_clk = 584,
		.phy_timcon = {
			.hs_trail = 14,
			.clk_trail = 15,
		},
		.vdo_per_frame_lp_enable = 1,
		.cust_esd_check = 0,
		.esd_check_enable = 1,
		.lcm_esd_check_table[0] = {
			.cmd = 0x0A,
			.count = 1,
			.para_list[0] = 0x9C,
			.mask_list[0] = 0x9C,
		},
		.lcm_esd_check_table[1] = {
			.cmd = 0xAB,
			.count = 2,
			.para_list[0] = 0x00,
			.mask_list[0] = 0x0F,
			.para_list[1] = 0x00,
			.mask_list[1] = 0x07,
		},
		.color_vivid_status = true,
		.color_srgb_status = true,
		.color_softiris_status = false,
		.color_dual_panel_status = false,
		.color_dual_brightness_status = true,
		.color_oplus_calibrate_status = true,
		.color_2nit_status = true,
		.cmd_null_pkt_en = 1,
		.cmd_null_pkt_len = 0,
		.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x03}},
		.dfps_cmd_table[1] = {0, 53 , {0xA9, 0x02, 0x08, 0xE1, 0x00, 0x02, 0x10, 0x10, 0x06, 0x02, 0x08, 0xE2, 0x00, 0x02, 0x06, 0x09, 0x05, 0x02, 0x08,
				0xE1, 0x3A, 0x3A, 0x01, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x01, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x01, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x01, 0x02,
				0x08, 0xE1, 0x56, 0x56, 0x01, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x01}},
		.apollo_limit_superior_us = 10000,
		.apollo_limit_inferior_us = 13596,
		.apollo_transfer_time_us = 8200,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
	{
	.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x02}},
		.dfps_cmd_table[1] = {0, 53 , {0xA9, 0x02, 0x08, 0xE1, 0x00, 0x02, 0x10, 0x10, 0x0C, 0x02, 0x08, 0xE2, 0x00, 0x02, 0x06, 0x09, 0x0A, 0x02, 0x08,
				0xE1, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x03, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x03, 0x02,
				0x08, 0xE1, 0x56, 0x56, 0x02, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x02}},
		.apollo_limit_superior_us = 2910,
		.apollo_limit_inferior_us = 10000,
		.apollo_transfer_time_us = 8400,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
	{
	.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x01}},
		.dfps_cmd_table[1] = {0, 53 , {0xA9, 0x02, 0x08, 0xE1, 0x00, 0x02, 0x10, 0x10, 0x0C, 0x02, 0x08, 0xE2, 0x00, 0x02, 0x06, 0x09, 0x0A, 0x02, 0x08,
				0xE1, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x03, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x03, 0x02,
				0x08, 0xE1, 0x56, 0x56, 0x02, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x02}},
		.apollo_limit_superior_us = 0,
		.apollo_limit_inferior_us = 6898,
		.apollo_transfer_time_us = 6200,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
	{
	.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 144,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x00}},
		.dfps_cmd_table[1] = {0, 53 , {0xA9, 0x02, 0x08, 0xE1, 0x00, 0x02, 0x10, 0x10, 0x0C, 0x02, 0x08, 0xE2, 0x00, 0x02, 0x06, 0x09, 0x0A, 0x02, 0x08,
				0xE1, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x03, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x03, 0x02,
				0x08, 0xE1, 0x56, 0x56, 0x02, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x02}},
		.apollo_limit_superior_us = 0,
		.apollo_limit_inferior_us = 6898,
		.apollo_transfer_time_us = 6200,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
};

static struct mtk_panel_params ext_params_dv2[MODE_NUM] = {
	{
		.pll_clk = 584,
		.phy_timcon = {
			.hs_trail = 14,
			.clk_trail = 15,
		},
		.vdo_per_frame_lp_enable = 1,
		.cust_esd_check = 0,
		.esd_check_enable = 1,
		.lcm_esd_check_table[0] = {
			.cmd = 0x0A,
			.count = 1,
			.para_list[0] = 0x9C,
			.mask_list[0] = 0x9C,
		},
		.lcm_esd_check_table[1] = {
			.cmd = 0xAB,
			.count = 2,
			.para_list[0] = 0x00,
			.mask_list[0] = 0x0F,
			.para_list[1] = 0x00,
			.mask_list[1] = 0x07,
		},
		.color_vivid_status = true,
		.color_srgb_status = true,
		.color_softiris_status = false,
		.color_dual_panel_status = false,
		.color_dual_brightness_status = true,
		.color_oplus_calibrate_status = true,
		.color_2nit_status = true,
		.cmd_null_pkt_en = 1,
		.cmd_null_pkt_len = 0,
		.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 59 , {0xA9, 0x02, 0x08, 0xB1, 0x00, 0x27, 0x03, 0x15, 0x9A, 0x19, 0x01, 0x01, 0x9A, 0x9A, 0x01, 0x01, 0x19, 0x19, 0x20,
				0x20, 0x4E, 0x30, 0x01, 0x82, 0x43, 0x82, 0x40, 0x84, 0x40, 0x00, 0x00, 0x00, 0x00, 0x10, 0xF1, 0xE2, 0xF1, 0xE2, 0xFF, 0x44, 0x4A, 0x45,
				0x59, 0x00, 0x13, 0x10, 0x02, 0x08, 0xB1, 0x28, 0x2F, 0x64, 0x64, 0x5F, 0x6B, 0x6A, 0x67, 0x6C, 0x6B}},
		.dfps_cmd_table[1] = {0, 59 , {0xA9, 0x02, 0x08, 0xB1, 0x30, 0x57, 0x6B, 0x69, 0x69, 0x67, 0x5C, 0x5C, 0x56, 0x61, 0x5E, 0x5B, 0x64, 0x62, 0x5F,
				0x60, 0x5E, 0x5C, 0x5A, 0x58, 0x55, 0x5C, 0x5B, 0x56, 0x5C, 0x5B, 0x57, 0x56, 0x54, 0x51, 0x55, 0x53, 0x4D, 0x55, 0x52, 0x50, 0x54, 0x52,
				0x4E, 0x5D, 0x5C, 0x54, 0x02, 0x08, 0xB1, 0x58, 0x5F, 0x53, 0x51, 0x49, 0x57, 0x57, 0x4E, 0x5B, 0x5B}},
		.dfps_cmd_table[2] = {0, 30 , {0xA9, 0x02, 0x08, 0xB1, 0x60, 0x6F, 0x53, 0x57, 0x58, 0x4F, 0x4F, 0x51, 0x44, 0x57, 0x57, 0x4B, 0x58, 0x58, 0x4C,
				0x54, 0x54, 0x48, 0x02, 0x08, 0xB1, 0x70, 0x72, 0x00, 0x00, 0x00}},
		.dfps_cmd_table[3] = {0, 59 , {0xA9, 0x02, 0x08, 0xB2, 0x00, 0x27, 0x1F, 0x1A, 0x21, 0x20, 0x1C, 0x26, 0x24, 0x1F, 0x26, 0x27, 0x22, 0x29, 0x14,
				0x0E, 0x17, 0x13, 0x0E, 0x14, 0x10, 0x09, 0x13, 0x09, 0x03, 0x0C, 0x1D, 0x19, 0x21, 0x23, 0x21, 0x21, 0x22, 0x1F, 0x23, 0x24, 0x22, 0x21,
				0x1F, 0x18, 0x1E, 0x20, 0x02, 0x08, 0xB2, 0x28, 0x2F, 0x1A, 0x1F, 0x21, 0x1D, 0x22, 0x20, 0x1A, 0x1E}},
		.dfps_cmd_table[4] = {0, 38 , {0xA9, 0x02, 0x08, 0xB2, 0x30, 0x47, 0x21, 0x1C, 0x1F, 0x21, 0x1F, 0x1D, 0x23, 0x22, 0x1F, 0x22, 0x1F, 0x1C, 0x27,
				0x26, 0x1F, 0x29, 0x29, 0x1F, 0x23, 0x24, 0x19, 0x25, 0x25, 0x1B, 0x02, 0x08, 0xB2, 0x48, 0x4A, 0x00, 0x00, 0x00}},
		.dfps_cmd_table[5] = {0, 2 , {0x2F, 0x03}},
		.dfps_cmd_table[6] = {0, 53 , {0xA9, 0x02, 0x08, 0xE1, 0x00, 0x02, 0x10, 0x10, 0x06, 0x02, 0x08, 0xE2, 0x00, 0x02, 0x06, 0x09, 0x05, 0x02, 0x08,
				0xE1, 0x3A, 0x3A, 0x01, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x01, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x01, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x01, 0x02,
				0x08, 0xE1, 0x56, 0x56, 0x01, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x01}},
		.apollo_limit_superior_us = 10000,
		.apollo_limit_inferior_us = 13596,
		.apollo_transfer_time_us = 8200,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
	{
	.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 59 , {0xA9, 0x02, 0x08, 0xB1, 0x00, 0x27, 0x03, 0x15, 0x9A, 0x19, 0x01, 0x01, 0x9A, 0x9A, 0x01, 0x01, 0x19, 0x19, 0x20,
				0x20, 0x36, 0x30, 0x01, 0x82, 0x43, 0x82, 0x40, 0x84, 0x40, 0x00, 0x00, 0x00, 0x00, 0x10, 0xF1, 0xE2, 0xF1, 0xE2, 0xFF, 0x44, 0x4A, 0x45,
				0x59, 0x00, 0x13, 0x10, 0x02, 0x08, 0xB1, 0x28, 0x2F, 0x4B, 0x50, 0x43, 0x50, 0x55, 0x48, 0x4D, 0x4F}},
		.dfps_cmd_table[1] = {0, 59 , {0xA9, 0x02, 0x08, 0xB1, 0x30, 0x57, 0x46, 0x4B, 0x4F, 0x45, 0x4F, 0x53, 0x47, 0x4E, 0x52, 0x46, 0x4B, 0x4E, 0x45,
				0x4F, 0x51, 0x48, 0x4B, 0x4D, 0x44, 0x4E, 0x51, 0x46, 0x47, 0x4B, 0x41, 0x47, 0x4B, 0x3F, 0x46, 0x4C, 0x3D, 0x42, 0x44, 0x3C, 0x47, 0x49,
				0x3F, 0x4B, 0x4D, 0x43, 0x02, 0x08, 0xB1, 0x58, 0x5F, 0x4C, 0x4E, 0x42, 0x47, 0x4B, 0x3F, 0x4D, 0x52}},
		.dfps_cmd_table[2] = {0, 30 , {0xA9, 0x02, 0x08, 0xB1, 0x60, 0x6F, 0x43, 0x4A, 0x4F, 0x42, 0x4A, 0x4E, 0x41, 0x4D, 0x53, 0x41, 0x4E, 0x52, 0x42,
				0x4B, 0x50, 0x3F, 0x02, 0x08, 0xB1, 0x70, 0x72, 0x00, 0x00, 0x00}},
		.dfps_cmd_table[3] = {0, 59 , {0xA9, 0x02, 0x08, 0xB2, 0x00, 0x27, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33,
				0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
				0x36, 0x36, 0x36, 0x36, 0x02, 0x08, 0xB2, 0x28, 0x2F, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36}},
		.dfps_cmd_table[4] = {0, 38 , {0xA9, 0x02, 0x08, 0xB2, 0x30, 0x47, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
				0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x02, 0x08, 0xB2, 0x48, 0x4A, 0x00, 0x00, 0x00}},
		.dfps_cmd_table[5] = {0, 2 , {0x2F, 0x02}},
		.dfps_cmd_table[6] = {0, 53 , {0xA9, 0x02, 0x08, 0xE1, 0x00, 0x02, 0x10, 0x10, 0x0C, 0x02, 0x08, 0xE2, 0x00, 0x02, 0x06, 0x09, 0x0A, 0x02, 0x08,
				0xE1, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x03, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x03, 0x02,
				0x08, 0xE1, 0x56, 0x56, 0x02, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x02}},
		.apollo_limit_superior_us = 2910,
		.apollo_limit_inferior_us = 10000,
		.apollo_transfer_time_us = 8400,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
	{
	.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 59 , {0xA9, 0x02, 0x08, 0xB1, 0x00, 0x27, 0x02, 0x15, 0x9A, 0x19, 0x01, 0x01, 0x9A, 0x9A, 0x01, 0x01, 0x19, 0x19, 0x20,
				0x20, 0x44, 0x43, 0xBF, 0x40, 0xD4, 0x40, 0xBB, 0xFF, 0xFE, 0xFF, 0x00, 0x00, 0x00, 0x10, 0xF1, 0xE2, 0xF1, 0xE2, 0xFF, 0x41, 0xFD, 0x00,
				0x1C, 0x00, 0x13, 0x10, 0x02, 0x08, 0xB1, 0x28, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
		.dfps_cmd_table[1] = {0, 59 , {0xA9, 0x02, 0x08, 0xB1, 0x30, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x02, 0x08, 0xB1, 0x58, 0x5F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
		.dfps_cmd_table[2] = {0, 30 , {0xA9, 0x02, 0x08, 0xB1, 0x60, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x02, 0x08, 0xB1, 0x70, 0x72, 0x00, 0x00, 0x00}},
		.dfps_cmd_table[3] = {0, 59 , {0xA9, 0x02, 0x08, 0xB2, 0x00, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x02, 0x08, 0xB2, 0x28, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
		.dfps_cmd_table[4] = {0, 38 , {0xA9, 0x02, 0x08, 0xB2, 0x30, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x08, 0xB2, 0x48, 0x4A, 0x00, 0x00, 0x00}},
		.dfps_cmd_table[5] = {0, 2 , {0x2F, 0x01}},
		.dfps_cmd_table[6] = {0, 53 , {0xA9, 0x02, 0x08, 0xE1, 0x00, 0x02, 0x10, 0x10, 0x0C, 0x02, 0x08, 0xE2, 0x00, 0x02, 0x06, 0x09, 0x0A, 0x02, 0x08,
				0xE1, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x03, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x03, 0x02,
				0x08, 0xE1, 0x56, 0x56, 0x02, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x02}},
		.apollo_limit_superior_us = 0,
		.apollo_limit_inferior_us = 6898,
		.apollo_transfer_time_us = 6200,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
	{
	.pll_clk = 584,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
		.mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.mask_list[0] = 0x0F,
		.para_list[1] = 0x00,
		.mask_list[1] = 0x07,
	},
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.color_2nit_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "A0026",
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
			.pic_height = 2800,
			.pic_width = 1280,
			.slice_height = 40,
			.slice_width = 640,
			.chunk_size = 640,
			.xmit_delay = 512,
			.dec_delay = 604,
			.scale_value = 32,
			.increment_interval = 1030,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 683,
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
		},
	.data_rate = 1168,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 144,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x00}},
		.dfps_cmd_table[1] = {0, 53 , {0xA9, 0x02, 0x08, 0xE1, 0x00, 0x02, 0x10, 0x10, 0x0C, 0x02, 0x08, 0xE2, 0x00, 0x02, 0x06, 0x09, 0x0A, 0x02, 0x08,
				0xE1, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE2, 0x3A, 0x3A, 0x02, 0x02, 0x08, 0xE1, 0x48, 0x48, 0x03, 0x02, 0x08, 0xE2, 0x48, 0x48, 0x03, 0x02,
				0x08, 0xE1, 0x56, 0x56, 0x02, 0x02, 0x08, 0xE2, 0x56, 0x56, 0x02}},
		.apollo_limit_superior_us = 0,
		.apollo_limit_inferior_us = 6898,
		.apollo_transfer_time_us = 6200,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.panel_bpp = 10,
	},
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static unsigned int demura_tap = 0;
static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned int i = 0;
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
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		level = 2047;
	}

	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	pr_info("ae037_A0026_P_3 backlight = %d bl_level[1]=%x, bl_level[2]=%x\n", level, bl_level[1], bl_level[2]);
	oplus_display_brightness = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &level);

	if (mapped_level < 1087 && demura_tap != 1) {
		demura_tap = 1;
		pr_info("ae037_A0026_P_3 backlight send demura0\n");
		for (i = 0; i < sizeof(dsi_demura0_bl)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
		}
	} else if (mapped_level >= 1087 && demura_tap != 2) {
		demura_tap = 2;
		pr_info("ae037_A0026_P_3 backlight send demura1\n");
		for (i = 0; i < sizeof(dsi_demura1_bl)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
		}
	}

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned int i = 0;
	unsigned int mapped_level = 0;
	unsigned char esd_bl_level[] = {0x51, 0x03, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	esd_bl_level[1] = level >> 8;
	esd_bl_level[2] = level & 0xFF;
	cb(dsi, handle, esd_bl_level, ARRAY_SIZE(esd_bl_level));

	pr_info("esd_bl_level[1]=%x, esd_bl_level[2]=%x backlight = %d\n", esd_bl_level[1], esd_bl_level[2], level);

	if (mapped_level < 1087 && demura_tap != 1) {
		demura_tap = 1;
		pr_info("ae037_A0026_P_3 backlight send demura0\n");
		for (i = 0; i < sizeof(dsi_demura0_bl)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
		}
	} else if (mapped_level >= 1087 && demura_tap != 2) {
		demura_tap = 2;
		pr_info("ae037_A0026_P_3 backlight send demura1\n");
		for (i = 0; i < sizeof(dsi_demura1_bl)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
		}
	}

	pr_info("ae037_A0026_P_3 esd_backlight_recovery finish\n");
	return 0;
}

static int oplus_display_panel_set_hbm_max(void *dsi, dcs_write_gce_pack cb1, dcs_write_gce cb2, void *handle, unsigned int en)
{
	unsigned int lcm_cmd_count = 0;
	unsigned int i = 0;
	struct LCM_setting_table *table = NULL;

	pr_info("en=%d\n", en);

	if (!dsi || !cb1 || !cb2) {
		pr_info("hbm max Invalid params\n");
		return -EINVAL;
	}

	if (en) {
		table = dsi_switch_hbm_apl_on;
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_on) / sizeof(struct LCM_setting_table);
		for (i = 0; i < lcm_cmd_count; i++) {
			cb2(dsi, handle, table[i].para_list, table[i].count);
		}
		last_backlight = MAX_NORMAL_BRIGHTNESS;
		pr_info("Enter hbm max mode, set last_backlight as %d", last_backlight);
	} else if (!en) {
		table = dsi_switch_hbm_apl_off;
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_off) / sizeof(struct LCM_setting_table);
		for (i = 0; i < lcm_cmd_count; i++) {
			cb2(dsi, handle, table[i].para_list, table[i].count);
	}
		pr_info("hbm_max off, restore bl:%d\n", oplus_display_brightness);
	}

	return 0;
}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;

	OFP_DEBUG("start\n");
	if (!dsi || !cb) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	pr_err("debug for %s oplus_display_brightness= %d, hbm_mode=%u\n",
			__func__, oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
		last_backlight = 3773;
		OFP_INFO("Enter hbm mode, set last_backlight as %d", last_backlight);
		lcdinfo_notify(1, &hbm_mode);
	} else if (hbm_mode == 0) {
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
		lcdinfo_notify(1, &hbm_mode);
		printk("debug for %s : %d ! backlight %d !\n", __func__, hbm_mode, oplus_display_brightness);
	}

	OFP_DEBUG("end\n");
	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
				  dcs_write_gce cb, void *handle, bool en)
{
	struct lcm *ctx = NULL;
	int i = 0;

	OFP_DEBUG("start\n");

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (en) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
		last_backlight = 3773;
		OFP_INFO("Enter hbm mode, set last_backlight as %d", last_backlight);
	} else if (en == 0) {
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}

	lcdinfo_notify(1, &en);

	OFP_DEBUG("end\n");
	return 0;
}

static int oplus_ofp_set_lhbm_pressed_icon(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	unsigned int reg_count = 0;
	unsigned int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *lhbm_pressed_icon_cmd = NULL;
	int i = 0;
	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, should not set lhbm pressed icon\n");
	}

	if (!panel || !dsi || !cb) {
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

	OFP_INFO("%s,oplus_display_brightness=%d, hbm_mode=%d, refresh_rate:%u\n", __func__, oplus_display_brightness, en, vrefresh_rate);
	if (en) {
		if (get_panel_es_ver() == ES_DV1) {
			reg_count = sizeof(lhbm_pressed_icon_on_cmd_dv1) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_on_cmd_dv1;
			OFP_INFO("LHBM DV1 ON\n");
		} else if (get_panel_es_ver() == ES_DV2) {
			reg_count = sizeof(lhbm_pressed_icon_on_cmd_dv2) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_on_cmd_dv2;
			OFP_INFO("LHBM DV2 ON\n");
		} else {
			reg_count = sizeof(lhbm_pressed_icon_on_cmd) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_on_cmd;
			OFP_INFO("LHBM PVT ON\n");
		}

		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_cmd[i].para_list, lhbm_pressed_icon_cmd[i].count);
		}

	} else if (en == 0) {
		if (get_panel_es_ver() == ES_DV1) {
			reg_count = sizeof(lhbm_pressed_icon_off_cmd_dv1) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_off_cmd_dv1;
			OFP_INFO("LHBM DV1 Off\n");
		} else if (get_panel_es_ver() == ES_DV2) {
			reg_count = sizeof(lhbm_pressed_icon_off_cmd_dv2) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_off_cmd_dv2;
			OFP_INFO("LHBM DV2 Off\n");
		} else {
			reg_count = sizeof(lhbm_pressed_icon_off_cmd) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_off_cmd;
			OFP_INFO("LHBM PVT Off\n");
		}
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_cmd[i].para_list, lhbm_pressed_icon_cmd[i].count);
		}

		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
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

	OFP_INFO("%s crtc_active:%d, doze_active:%llu\n", __func__, crtc->state->active, mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE]);

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

	if(!oplus_ofp_backlight_filter(crtc, handle, oplus_display_brightness))
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);

	OFP_INFO("send aod off cmd\n");

	return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
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


#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s:on=%d\n", __func__, on);

	gpiod_set_value(ctx->reset_gpio, on);

	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared) {
		pr_info("ctx->prepared:%d return! \n", ctx->prepared);
		return 0;
	}
	usleep_range(5000, 5100);
	/* lcd reset H -> L -> H */
	if (IS_ERR(ctx->reset_gpio)) {
		pr_err("cannot get reset-gpios %ld\n", PTR_ERR(ctx->reset_gpio));
	}
	gpiod_set_value(ctx->reset_gpio, 1);
	/* Wait > 1ms, actual 3ms */
	usleep_range(3000, 3100);
	gpiod_set_value(ctx->reset_gpio, 0);
	/* Wait > 10us, actual 2ms */
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	/* Wait > 20ms, actual 25ms */
	usleep_range(25000, 25100);
	pr_info("%s:Successful\n", __func__);

	return 0;
}
static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	/* iovcc enable 1.8V */
	lcm_panel_1p8_ldo_enable(ctx->dev);
	/* Wait > 1ms, actual 5ms */
	usleep_range(5000, 5100);

	/* enable ldo 3p0 */
	lcm_panel_wl2868c_ldo_enable(ctx->dev);
	usleep_range(5000, 5100);

	/* enable vcore 1p2 for boe is high 1.22v */
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);
	/* Wait > 10ms, actual 12ms */
	usleep_range(22000, 22100);

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

	usleep_range(10000, 10100);
	gpiod_set_value(ctx->reset_gpio, 0);
	/* Wait > 1ms, actual 5ms */
	usleep_range(5000, 5100);

	/* disable vcore1.2V*/
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 0);
	/* Wait > 1ms, actual 5ms */
	usleep_range(10000, 10100);

	/* disable ldo 3p0*/
	lcm_panel_wl2868c_ldo_disable(ctx->dev);
	/* Wait no limits, actual 5ms */
	usleep_range(10000, 10100);

	/* set vddi 1.8v*/
	lcm_panel_1p8_ldo_disable(ctx->dev);
	/* power off Foolproof, actual 70ms*/
	usleep_range(72000, 72100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	demura_tap = 0;
	pr_info("%s: Successful\n", __func__);
	return 0;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector, unsigned int mode)
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

static int mtk_panel_ext_param_set(struct drm_panel *panel, struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);
	pr_info("%s: mode=%d, vrefresh=%d\n", __func__, mode, drm_mode_vrefresh(m));

	if (get_panel_es_ver() == ES_DV1) {
		if (m_vrefresh == 60) {
			ext->params = &ext_params_dv1[0];
		} else if (m_vrefresh == 90) {
			ext->params = &ext_params_dv1[1];
		} else if (m_vrefresh == 120) {
			ext->params = &ext_params_dv1[2];
		} else if (m_vrefresh == 144) {
			ext->params = &ext_params_dv1[3];
		} else {
			ext->params = &ext_params_dv1[2];
			ret = 1;
		}
	} else if (get_panel_es_ver() == ES_DV2) {
		if (m_vrefresh == 60) {
			ext->params = &ext_params_dv2[0];
		} else if (m_vrefresh == 90) {
			ext->params = &ext_params_dv2[1];
		} else if (m_vrefresh == 120) {
			ext->params = &ext_params_dv2[2];
		} else if (m_vrefresh == 144) {
			ext->params = &ext_params_dv2[3];
		} else {
			ext->params = &ext_params_dv2[2];
			ret = 1;
		}
	} else {
		if (m_vrefresh == 60) {
			ext->params = &ext_params[0];
		} else if (m_vrefresh == 90) {
			ext->params = &ext_params[1];
		} else if (m_vrefresh == 120) {
			ext->params = &ext_params[2];
		} else if (m_vrefresh == 144) {
			ext->params = &ext_params[3];
		} else {
			ext->params = &ext_params[2];
			ret = 1;
		}
	}

	return ret;
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

	if (get_panel_es_ver() == ES_DV1) {
		if (m_vrefresh == 60) {
			*ext_param = &ext_params_dv1[0];
		} else if (m_vrefresh == 120) {
			*ext_param = &ext_params_dv1[2];
		} else if (m_vrefresh == 144) {
			*ext_param = &ext_params_dv1[3];
		} else if (m_vrefresh == 90) {
			*ext_param = &ext_params_dv1[1];
		} else {
			*ext_param = &ext_params_dv1[2];
		}
	} else if (get_panel_es_ver() == ES_DV2) {
		if (m_vrefresh == 60) {
			*ext_param = &ext_params_dv2[0];
		} else if (m_vrefresh == 120) {
			*ext_param = &ext_params_dv2[2];
		} else if (m_vrefresh == 144) {
			*ext_param = &ext_params_dv2[3];
		} else if (m_vrefresh == 90) {
			*ext_param = &ext_params_dv2[1];
		} else {
			*ext_param = &ext_params_dv2[2];
		}
	} else {
		if (m_vrefresh == 60) {
			*ext_param = &ext_params[0];
		} else if (m_vrefresh == 120) {
			*ext_param = &ext_params[2];
		} else if (m_vrefresh == 144) {
			*ext_param = &ext_params[3];
		} else if (m_vrefresh == 90) {
			*ext_param = &ext_params[1];
		} else {
			*ext_param = &ext_params[2];
		}
	}

	if (*ext_param)
		pr_debug("[LCM] data_rate:%d\n", (*ext_param)->data_rate);
	else
		pr_err("[LCM] ext_param is NULL;\n");

	return ret;
}

static int mode_switch_hs(struct drm_panel *panel, struct drm_connector *connector,
		void *dsi_drv, unsigned int cur_mode, unsigned int dst_mode,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage, dcs_write_gce_pack cb)
{
	int ret = 0;
	int m_vrefresh = 0;
	unsigned int lcm_cmd_count = 0;
	static int last_data_rate = 900;
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("lcm cur_mode = %d dst_mode %d.\n", cur_mode, dst_mode);

	if (cur_mode == dst_mode) {
		return ret;
	}

	if (stage == BEFORE_DSI_POWERDOWN) {
		m_vrefresh = drm_mode_vrefresh(m);

		if (m_vrefresh == 60) {
			lcm_cmd_count = sizeof(timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60, lcm_cmd_count, cb, NULL);
			pr_info("lcm timing switch to 60\n");
		} else if (m_vrefresh == 90) {
			lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
			pr_info("lcm timing switch to 90\n");
		} else if (m_vrefresh == 120) {
			lcm_cmd_count = sizeof(timing_switch_cmd_sdc120) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120, lcm_cmd_count, cb, NULL);
			pr_info("lcm timing switch to 120\n");
		} else if (m_vrefresh == 144) {
			lcm_cmd_count = sizeof(timing_switch_cmd_sdc144) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc144, lcm_cmd_count, cb, NULL);
			pr_info("lcm timing switch to 144\n");
		}
	}

	ctx->m = m;

	if (ext->params->data_rate != last_data_rate) {
		ret = 1;
		pr_info("need to change mipi clk, data_rate=%d, last_data_rate=%d\n", ext->params->data_rate, last_data_rate);
		last_data_rate = ext->params->data_rate;
	}

	return ret;
}

static int mode_switch_update_for_vdo(struct drm_connector *connector, unsigned int cur_mode, unsigned int dst_mode)
{
	int ret = 0;
	int m_vrefresh = 0;
	int src_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct drm_display_mode *src_m = get_mode_by_id(connector, cur_mode);

	pr_info("cur_mode=%d, dst_mode=%d\n", cur_mode, dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	g_last_mode_idx = cur_mode;
	panel_mode_id = get_mode_enum(m);
	m_vrefresh = drm_mode_vrefresh(m);
	src_vrefresh = drm_mode_vrefresh(src_m);

	pr_info("update panel_mode_id:%d->%d, hdisplay:%d->%d, hskew:%d->%d, vrefresh:%d->%d\n",
			get_mode_enum(src_m), panel_mode_id, src_m->hdisplay, m->hdisplay,
			src_m->hskew, m->hskew, src_vrefresh, m_vrefresh);

	return ret;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_reset = lcm_panel_reset,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.mode_switch_hs = mode_switch_hs,
	.mode_switch_update_for_vdo = mode_switch_update_for_vdo,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.set_hbm = lcm_set_hbm,
	.get_res_switch_type = mtk_get_res_switch_type,
	/* .scaling_mode_mapping = mtk_scaling_mode_mapping, */
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	/* .oplus_set_hbm = oplus_lcm_set_hbm, */
	/* .oplus_hbm_set_cmdq = oplus_panel_hbm_set_cmdq, */
	.oplus_ofp_set_lhbm_pressed_icon_single = oplus_ofp_set_lhbm_pressed_icon,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	/* .oplus_get_info = lcm_read_info, */
	.lcm_set_hbm_max = oplus_display_panel_set_hbm_max,
};

static int lcm_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode[MODE_NUM * RES_NUM];
        int i = 0;
	struct lcm *ctx = panel_to_lcm(panel);

	mode[0] = drm_mode_duplicate(connector->dev, &display_mode[0]);
	if (!mode[0]) {
		pr_err("failed to add mode %ux%ux@%u\n", display_mode[0].hdisplay, display_mode[0].vdisplay, drm_mode_vrefresh(&display_mode[0]));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[0]);
	pr_info("clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n", mode[0]->clock, mode[0]->htotal,
		mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

	for (i = 1; i < MODE_NUM * RES_NUM; i++) {
		mode[i] = drm_mode_duplicate(connector->dev, &display_mode[i]);
		if (!mode[i]) {
			pr_err("lcm_get_modes not enough memory\n");
		return -ENOMEM;
	}
		drm_mode_set_name(mode[i]);
		mode[i]->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode[i]);
	}

	connector->display_info.width_mm = PHYSICAL_WIDTH / 1000;
	connector->display_info.height_mm = PHYSICAL_HEIGHT / 1000;

	if (!ctx->m) {
		ctx->m = get_mode_by_id(connector, 0);
		pr_info("ctx->m init: mode_id %d\n", get_mode_enum(ctx->m));
	}
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

	pr_info("[LCM] %s+ ae037_p_3 a0026 Start\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_err("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_err("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_err("skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
		| MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

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

		if (!ctx->backlight) {
			pr_err("skip probe due to lcm backlight null\n");
			return -EPROBE_DEFER;
		}
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(ctx->reset_gpio)) {
		pr_err("cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 1);

	lcm_panel_1p8_ldo_enable(ctx->dev);

	usleep_range(5000, 5100);

	ctx->vddr1p2_enable_gpio = devm_gpiod_get(dev, "1p2", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(ctx->vddr1p2_enable_gpio)) {
		pr_err("cannot get vddr1p2_enable_gpio %ld\n",
			 PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);

	usleep_range(5000, 5100);

	lcm_panel_wl2868c_ldo_enable(ctx->dev);

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

	register_device_proc("lcd", "A0026", "P_3");
#ifdef OPLUS_FEATURE_DISPLAY
	/* oplus_panel_parse(dev->of_node, TRUE); */
#endif /* OPLUS_FEATURE_DISPLAY */
	/* flag_silky_panel = BL_SETTING_DELAY_60HZ; */
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;

	oplus_ofp_init(dev);
	oplus_display_panel_dbv_probe(dev);

	/* oplus_max_brightness = BRIGHTNESS_MAX; */
	oplus_enhance_mipi_strength = 0;
	/* g_is_silky_panel = true; */

	pr_info("ae037_A0026_P_3 lcm probe End.\n");
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
		.compatible = "panel_ae037_p_3_a0026_dsi_vdo",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel_ae037_p_3_a0026_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register lcm driver: %d\n", __func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit lcm_drv_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}

module_init(lcm_drv_init);
module_exit(lcm_drv_exit);

MODULE_AUTHOR("Oplus Display");
MODULE_DESCRIPTION("panel_ae037_p_3_a0026_dsi_vdo Driver");
MODULE_LICENSE("GPL v2");
