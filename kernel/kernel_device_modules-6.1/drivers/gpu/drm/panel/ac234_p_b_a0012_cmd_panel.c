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

#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/of_address.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include "include/ac234_p_b_a0012_cmd_panel.h"
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/ac234_p_b_a0012_data_hw_roundedpattern.h"
#include "../oplus/oplus_display_onscreenfingerprint.h"

#define LPX_60_90 (4)
#define LPX_120 (8)
extern unsigned int get_PCB_Version(void);
static unsigned int pcb_version = 0;

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr1p2_enable_gpio;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	int error;
};

static DEFINE_MUTEX(gVsvoterLocker);
static int hw_vsvoter_cnt = 0;
static struct hw_vsvoter *hw_vsvoter = NULL;

extern void lcdinfo_notify(unsigned long val, void *v);
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern int oplus_serial_number_probe(struct device *dev);
extern atomic_t oplus_pcp_handle_lock;
extern void oplus_pcp_handle(bool cmd_is_pcp,  void *handle);
extern int oplus_display_panel_dbv_probe(struct device *dev);

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle);
#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2

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

static int lcm_panel_vsvoter_parse(struct device_node *node)
{
	struct of_phandle_args args;
	struct platform_device *pdev;
	int ret;

	/* vs vote function is optional */
	ret = of_property_read_bool(node, "mediatek,vs-voter");
	if (ret != 1){
		pr_err("[LCM]node:(mediatek,vs-voter) not found\n");
		return 0;
	}

	ret = of_parse_phandle_with_fixed_args(node,
		"mediatek,vs-voter", 3, 0, &args);
	if (ret) {
		pr_err("[LCM]:of_parse_phandle_with_fixed_args-node:(mediatek,vs-voter) failed!\n");
		return ret;
	}

	if (!args.np) {
		pr_err("[LCM]:args.np not found\n");
		return -ENODEV;
	}

	pdev = of_find_device_by_node(args.np->child);
	if (!pdev) {
		pr_err("[LCM]:of_find_device_by_node-node:(args.np->child) not found\n");
		return -ENODEV;
	}

	hw_vsvoter = kmalloc(sizeof(struct hw_vsvoter), GFP_KERNEL);
	if (!hw_vsvoter){
		pr_err("[LCM]:hw_vsvoter kmalloc failed\n");
		kfree(hw_vsvoter);
		return -ENOMEM;
	}

	hw_vsvoter->vsv = dev_get_regmap(pdev->dev.parent, NULL);
	if (!hw_vsvoter->vsv) {
		pr_err("[LCM]:dev_get_regmap-node:(dev.parent) not found\n");
		kfree(hw_vsvoter);
		return -ENODEV;
	}

	hw_vsvoter->vsv_reg = args.args[0];
	hw_vsvoter->vsv_mask = args.args[1];
	hw_vsvoter->vsv_vers = args.args[2];
	pr_info("[LCM]:vsv - reg:0x%x, mask:0x%x, version:%d\n",
			hw_vsvoter->vsv_reg, hw_vsvoter->vsv_mask, hw_vsvoter->vsv_vers);

	return PTR_ERR_OR_ZERO(hw_vsvoter->vsv);
}

static void lcm_hw_vsvoter_set(void)
{
	u32 reg, msk, val;
	if (IS_ERR_OR_NULL(hw_vsvoter))
		return;

	/* write 1 to set and clr, update reg address */
	reg = hw_vsvoter->vsv_reg + VS_VOTER_EN_LO_SET;
	msk = hw_vsvoter->vsv_mask;
	val = hw_vsvoter->vsv_mask;

	if (IS_ERR_OR_NULL(hw_vsvoter->vsv)){
		return;
	} else {
		regmap_update_bits(hw_vsvoter->vsv, reg, msk, val);
		pr_info("[LCM]:vsv - reg:0x%x, mask:0x%x, version:%d\n", reg, msk, val);
	}
}

static void lcm_hw_vsvoter_clr(void)
{
	u32 reg, msk, val;
	unsigned int regval_a, regval_d;
	if (IS_ERR_OR_NULL(hw_vsvoter))
		return;

	/* write 1 to set and clr, update reg address */
	reg = hw_vsvoter->vsv_reg + VS_VOTER_EN_LO_CLR;
	msk = hw_vsvoter->vsv_mask;
	val = hw_vsvoter->vsv_mask;

	if (IS_ERR_OR_NULL(hw_vsvoter->vsv)){
		return;
	} else {
		regmap_update_bits(hw_vsvoter->vsv, reg, msk, val);
		regmap_read(hw_vsvoter->vsv, 0x149a, &regval_a);
		regmap_read(hw_vsvoter->vsv, 0x149d, &regval_d);
		pr_info("[LCM]:vsv - reg:0x%x, mask:0x%x, version:%d, reg0x149a:%d, reg0x149d:%d\n" ,reg, msk, val, regval_a, regval_d);
	}
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

static int oplus_display_panel_set_hbm_max(void *dsi, dcs_write_gce_pack cb, dcs_write_gce cb2, void *handle, unsigned int en)
{
	unsigned int lcm_cmd_count = 0;
	unsigned int level = oplus_display_brightness;

	if (!dsi || !cb || !cb2) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	if (en) {
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_on) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, dsi_switch_hbm_apl_on, lcm_cmd_count, cb, handle);
		pr_info("Enter hbm max APL mode\n");
	} else if (!en) {
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_off) / sizeof(struct LCM_setting_table);
		dsi_switch_hbm_apl_off[lcm_cmd_count-1].para_list[1] = level >> 8;
		dsi_switch_hbm_apl_off[lcm_cmd_count-1].para_list[2] = level & 0xFF;
		panel_send_pack_hs_cmd(dsi, dsi_switch_hbm_apl_off, lcm_cmd_count, cb, handle);
		pr_info("hbm max APL off, restore backlight:%d\n", level);
	}

	return 0;
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		return -EINVAL;
	}

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60) {
		ret = FHD_SDC60;
	} else if (m_vrefresh == 90) {
		ret = FHD_SDC90;
	} else if (m_vrefresh == 120) {
			ret = FHD_SDC120;
	} else {
		ret = FHD_SDC60;
	}
	return ret;

}

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	struct drm_display_mode *m = ctx->m;
	mode_id = get_mode_enum(m);
	pr_info("%s +\n", __func__);
	switch (mode_id) {
		pr_info("%s:last_fps_mode=%d\n", __func__, mode_id);
		case FHD_SDC120:
			push_table(ctx, init_setting_120hz, sizeof(init_setting_120hz) / sizeof(struct LCM_setting_table));
			break;
		case FHD_SDC90:
			push_table(ctx, init_setting_90hz, sizeof(init_setting_90hz) / sizeof(struct LCM_setting_table));
			break;
		case FHD_SDC60:
			push_table(ctx, init_setting_60hz, sizeof(init_setting_60hz) / sizeof(struct LCM_setting_table));
			break;
		default:
			push_table(ctx, init_setting_60hz, sizeof(init_setting_60hz) / sizeof(struct LCM_setting_table));
			break;
	}

	pr_info("%s -\n", __func__);
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
	pr_info("%s:prepared=%d\n", __func__, ctx->prepared);

	if (!ctx->prepared)
		return 0;

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}
	if( oplus_ofp_get_aod_state() == true) {
		if (vrefresh_rate == 60) {
			push_table(ctx, AOD_off_setting_60hz, sizeof(AOD_off_setting_60hz) / sizeof(struct LCM_setting_table));
		} else if (vrefresh_rate == 120) {
			push_table(ctx, AOD_off_setting_120hz, sizeof(AOD_off_setting_120hz) / sizeof(struct LCM_setting_table));
		} else {
			push_table(ctx, AOD_off_setting_90hz, sizeof(AOD_off_setting_90hz) / sizeof(struct LCM_setting_table));
		}
		usleep_range(9000, 9100);
		OFP_INFO("send aod off cmd\n");
	}

	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(21000, 21100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(121000, 121100);

	ctx->error = 0;
	ctx->prepared = false;
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

static const struct drm_display_mode disp_mode_60Hz = {
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

static const struct drm_display_mode disp_mode_90Hz = {
	.clock = 250200,
	.hdisplay = 1080,
	.hsync_start = 1080 + 9,//HFP
	.hsync_end = 1080 + 9 + 2,//HSA
	.htotal = 1080 + 9 + 2 + 21,//HBP
	.vdisplay = 2412,
	.vsync_start = 2412 + 52,//VFP
	.vsync_end = 2412 + 52 + 14,//VSA
	.vtotal = 2412 + 52+ 14 + 22,//VBP
};

static const struct drm_display_mode disp_mode_120Hz = {
	.clock = 333600,
	.hdisplay = 1080,
	.hsync_start = 1080 + 9,//HFP
	.hsync_end = 1080 + 9 + 2,//HSA
	.htotal = 1080 + 9 + 2 + 21,//HBP
	.vdisplay = 2412,
	.vsync_start = 2412 + 52,//VFP
	.vsync_end = 2412 + 52 + 14,//VSA
	.vtotal = 2412 + 52+ 14 + 22,//VBP
};

static struct mtk_panel_params ext_params_60Hz = {
	.pll_clk = 300,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.phy_timcon = {
		.hs_trail = 11,
		.clk_trail = 11,
		.lpx = LPX_60_90, //53.32 ns
		.ta_go = 4 * LPX_60_90,
		.ta_get = 5 * LPX_60_90,
		.ta_sure = 3 * LPX_60_90 / 2,
	},
//	.esd_check_multi = 1,
//	.esd_te_check_gpio = 1,
//	.move_esd_readdate_back = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},

	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

//	.cmd_null_pkt_en = 1,
//	.cmd_null_pkt_len = 0,
//	.skip_unnecessary_switch = true,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.panel_type = 3,
	.vendor = "AC234_A0012",
	.manufacture = "P_B",
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
		.dsc_cfg_change = 0,
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
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 287,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
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
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
		.oplus_ofp_aod_off_insert_black = 2,
		.oplus_ofp_aod_off_black_frame_total_time = 67,
		.oplus_ofp_need_keep_apart_backlight = true,
		.oplus_ofp_hbm_on_delay = 0,
		.oplus_ofp_pre_hbm_off_delay = 0,
		.oplus_ofp_hbm_off_delay = 0,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 60,
		.apollo_limit_superior_us = 10540,
		.apollo_limit_inferior_us = 14200,
		.apollo_transfer_time_us = 8200,
	},
	.panel_bpp = 10,
};

static struct mtk_panel_params ext_params_90Hz = {
	.pll_clk = 300,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.phy_timcon = {
		.hs_trail = 11,
		.clk_trail = 11,
		.lpx = LPX_60_90, //53.32 ns
		.ta_go = 4 * LPX_60_90,
		.ta_get = 5 * LPX_60_90,
		.ta_sure = 3 * LPX_60_90 / 2,
	},
//	.esd_check_multi = 1,
//	.esd_te_check_gpio = 1,
//	.move_esd_readdate_back = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},

	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

//	.cmd_null_pkt_en = 1,
//	.cmd_null_pkt_len = 0,
//	.skip_unnecessary_switch = true,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.panel_type = 3,
	.vendor = "AC234_A0012",
	.manufacture = "P_B",
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
		.dsc_cfg_change = 0,
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
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 287,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
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
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
		.oplus_ofp_aod_off_insert_black = 2,
		.oplus_ofp_aod_off_black_frame_total_time = 56,
		.oplus_ofp_need_keep_apart_backlight = true,
		.oplus_ofp_hbm_on_delay = 0,
		.oplus_ofp_pre_hbm_off_delay = 0,
		.oplus_ofp_hbm_off_delay = 0,
	.dyn_fps = {
	.switch_en = 1, .vact_timing_fps = 90,
	.apollo_limit_superior_us = 7880,
	.apollo_limit_inferior_us = 10000,
	.apollo_transfer_time_us = 8800,
	},
	.panel_bpp = 10,
};

static struct mtk_panel_params ext_params_120Hz = {
	.pll_clk = 553,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
		.lpx = LPX_120, //57.84 ns
		.ta_go = 4 * LPX_120,
		.ta_get = 5 * LPX_120,
		.ta_sure = 3 * LPX_120 / 2,
	},
//	.esd_check_multi = 1,
//	.esd_te_check_gpio = 1,
//	.move_esd_readdate_back = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},

	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,

	.cmd_null_pkt_en = 0,
	.cmd_null_pkt_len = 105,
	//.skip_unnecessary_switch = true,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.panel_type = 3,
	.vendor = "AC234_A0012",
	.manufacture = "P_B",
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
		.dsc_cfg_change = 0,
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
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 287,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
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

	.data_rate = 1106,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 2,
	.oplus_ofp_aod_off_black_frame_total_time = 50,
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
		.apollo_limit_superior_us = 4900,
		.apollo_limit_inferior_us = 7700,
		.apollo_transfer_time_us = 6200,
	},
	.panel_bpp = 10,
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
	} else if (level > 4095) {
		level = 4095;
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
	pr_info("bl_level[1]=%x, bl_level[2]=%x\n", bl_level[1], bl_level[2]);
	oplus_display_brightness = level;

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

static int lcm_set_hbm(void *dsi, dcs_write_gce_pack cb,
		void *handle, unsigned int hbm_mode)
{
	unsigned int level = oplus_display_brightness;
	unsigned int lcm_cmd_count = 0;

	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	pr_info("oplus_display_brightness=%d, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		lcm_cmd_count = sizeof(HBM_on_setting) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, HBM_on_setting, lcm_cmd_count, cb, handle);
	} else if (hbm_mode == 0) {
		HBM_off_setting[1].para_list[1] = level >> 8;
		HBM_off_setting[1].para_list[2] = level & 0xFF;
		lcm_cmd_count = sizeof(HBM_off_setting) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, HBM_off_setting, lcm_cmd_count, cb, handle);
		pr_info("level %x\n", level);
	}
	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	unsigned int i = 0;
	unsigned int cmd;

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	pr_info("oplus_display_brightness=%d, en=%u\n", oplus_display_brightness, en);
	if(en == 1) {
		for (i = 0; i < sizeof(HBM_on_setting)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, HBM_on_setting[i].para_list, HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		for (i = 0; i < sizeof(HBM_off_setting)/sizeof(struct LCM_setting_table); i++) {
			cmd = HBM_off_setting[i].cmd;
			switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(HBM_off_setting[i].count * 1000, HBM_off_setting[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(HBM_off_setting[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(HBM_off_setting[i].count, HBM_off_setting[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(HBM_off_setting[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, HBM_off_setting[i].para_list, HBM_off_setting[i].count);
		}
	}
	pr_info("paralist[1]=%x, paralist[2]=%x\n", HBM_off_setting[1].para_list[1], HBM_off_setting[1].para_list[2]);
	lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;
	unsigned int reg_count = 0;
	int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *aod_off_cmd = NULL;
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

	if (vrefresh_rate == 60) {
		aod_off_cmd = AOD_off_setting_60hz;
		reg_count = sizeof(AOD_off_setting_60hz) / sizeof(struct LCM_setting_table);
	} else if (vrefresh_rate == 90) {
		aod_off_cmd = AOD_off_setting_90hz;
		reg_count = sizeof(AOD_off_setting_90hz) / sizeof(struct LCM_setting_table);
	} else {
		aod_off_cmd = AOD_off_setting_120hz;
		reg_count = sizeof(AOD_off_setting_120hz) / sizeof(struct LCM_setting_table);
	}


	for (i = 0; i < reg_count; i++) {

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
	pr_info("%s:success\n", __func__);
	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;
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
				cb(dsi, handle, AOD_on_setting[i].para_list, AOD_on_setting[i].count);
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

	pr_info("%s: p_b_a0012 lcm ctx->prepared %d\n", __func__, ctx->prepared);
	/* vddr-oled 1p2 enable */
	if ((pcb_version < 3) || ((pcb_version >= 8) && (pcb_version <= 10)) || (pcb_version == 16) || (pcb_version == 17)) {
		ctx->vddr1p2_enable_gpio =
			devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
			dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
				__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
			return PTR_ERR(ctx->vddr1p2_enable_gpio);
		}
		gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);
		devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	} else {
		mutex_lock(&gVsvoterLocker);
		lcm_hw_vsvoter_set();
		hw_vsvoter_cnt++;
		pr_info("hw_vsvoter_cnt = %d\n", hw_vsvoter_cnt);
		mutex_unlock(&gVsvoterLocker);
		usleep_range(3000, 3100);
		lcm_panel_mt6363_vcn13_enable(ctx->dev);
	}
	usleep_range(5000, 5100);
	/* vci-oled 3p0 enable */
	lcm_panel_mt6369_vmch_enable(ctx->dev);
	usleep_range(10000, 10010);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	pr_info("%s: Successful, pcb_version=%d\n", __func__, pcb_version);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("%s: p_b_a0012 lcm ctx->prepared %d\n", __func__, ctx->prepared);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(3000, 3100);
	/* vci-oled 3p0 disable */
	lcm_panel_mt6369_vmch_disable(ctx->dev);
	usleep_range(3000, 3100);
	/* vddr-oled 1p2 disable */
	if ((pcb_version < 3) || ((pcb_version >= 8) && (pcb_version <= 10)) || (pcb_version == 16) || (pcb_version == 17)) {
		ctx->vddr1p2_enable_gpio =
			devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
			dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
				__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
			return PTR_ERR(ctx->vddr1p2_enable_gpio);
		}
		gpiod_set_value(ctx->vddr1p2_enable_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	} else {
		mutex_lock(&gVsvoterLocker);
		hw_vsvoter_cnt--;
		if(hw_vsvoter_cnt == 0){
			lcm_hw_vsvoter_clr();
			pr_info("hw_vsvoter_cnt = %d\n", hw_vsvoter_cnt);
		}
		mutex_unlock(&gVsvoterLocker);
		usleep_range(3000, 3100);
		lcm_panel_mt6363_vcn13_disable(ctx->dev);
	}
	usleep_range(15000, 15100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	pr_info("%s: Successful, pcb_version=%d\n", __func__, pcb_version);

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
	if (!m) {
		pr_err("%s:%d invalid display_mode\n", __func__, __LINE__);
		return ret;
	}

	m_vrefresh = drm_mode_vrefresh(m);
	pr_info("%s: mode=%d, vrefresh=%d\n", __func__, mode, drm_mode_vrefresh(m));

	if (m_vrefresh == 60) {
		ext->params = &ext_params_60Hz;
	} else if (m_vrefresh == 90) {
		ext->params = &ext_params_90Hz;
	} else if (m_vrefresh == 120) {
		ext->params = &ext_params_120Hz;
	} else {
		ret = 1;
	}

	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel, struct drm_connector *connector,
		struct mtk_panel_params **ext_param, unsigned int id)
{
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);
	if (!m) {
		pr_err("%s:%d invalid display_mode\n", __func__, __LINE__);
		return ret;
	}

	m_vrefresh = drm_mode_vrefresh(m);
	pr_info("%s:[LCM] vrefresh=%d\n", __func__, drm_mode_vrefresh(m));

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
		pr_info("[LCM] ac234_p_b_a0012 data_rate:%d\n", (*ext_param)->data_rate);
	else
		pr_err("[LCM] ac234_p_b_a0012 ext_param is NULL;\n");

	return ret;
}

static unsigned int last_fps_mode = 60;
static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);
	if (oplus_ofp_get_aod_state() == true){
		pr_info("no need mode switch\n");
		return 0;
	}
	if (cur_mode == dst_mode)
		return ret;
	if (drm_mode_vrefresh(m) == 60) {
		if (stage == BEFORE_DSI_POWERDOWN){
			push_table(ctx, mode_switch_to_60, sizeof(mode_switch_to_60) / sizeof(struct LCM_setting_table));
			if (last_fps_mode == 120) {
				usleep_range(8300, 8400);
			}
			last_fps_mode = 60;
			pr_info("%s timing switch to 60 success\n", __func__);
			ret = 1;
		}
	} else if (drm_mode_vrefresh(m) == 90) {
		if (stage == BEFORE_DSI_POWERDOWN){
			push_table(ctx, mode_switch_to_90, sizeof(mode_switch_to_90) / sizeof(struct LCM_setting_table));
			if (last_fps_mode == 120) {
				usleep_range(8300, 8400);
			}
			last_fps_mode = 90;
			pr_info("%s timing switch to 90 success\n", __func__);
			ret = 1;
		}
	} else if (drm_mode_vrefresh(m) == 120) {
		if (stage == AFTER_DSI_POWERON){
			push_table(ctx, mode_switch_to_120, sizeof(mode_switch_to_120) / sizeof(struct LCM_setting_table));
			last_fps_mode = 120;
			pr_info("%s timing switch to 120 success\n", __func__);
			ret = 1;
		}
	}
	ctx->m = m;
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
	.ext_param_get = mtk_panel_ext_param_get,
	.mode_switch = mode_switch,
	.oplus_set_hbm = lcm_set_hbm,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.lcm_set_hbm_max = oplus_display_panel_set_hbm_max,
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

	pr_info("[LCM] ac234_p_b_a0012 %s START\n", __func__);


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

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	lcm_panel_vsvoter_parse(dev->of_node);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	usleep_range(5000, 5100);
	pcb_version = get_PCB_Version();
	pr_info("%s: get PCB Version = %u\n", __func__, pcb_version);
	if ((pcb_version < 3) || ((pcb_version >= 8) && (pcb_version <= 10)) || (pcb_version == 16) || (pcb_version == 17)) {
		ctx->vddr1p2_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
			dev_err(dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
				__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
			return PTR_ERR(ctx->vddr1p2_enable_gpio);
		}
		devm_gpiod_put(dev, ctx->vddr1p2_enable_gpio);
	} else {
		mutex_lock(&gVsvoterLocker);
		lcm_hw_vsvoter_set();
		hw_vsvoter_cnt++;
		pr_info("hw_vsvoter_cnt = %d\n", hw_vsvoter_cnt);
		mutex_unlock(&gVsvoterLocker);
		usleep_range(3000, 3100);
		lcm_panel_mt6363_vcn13_enable(ctx->dev);
	}
	usleep_range(5000, 5100);
	lcm_panel_mt6369_vmch_enable(ctx->dev);

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
	oplus_serial_number_probe(dev);
	register_device_proc("lcd", "AC234_A0012", "P_B");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	oplus_ofp_init(dev);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
	pr_info("[LCM] %s- lcm, AC234_P_B_A0012, END\n", __func__);


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
	{ .compatible = "ac234,p,b,a0012,cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "ac234_p_b_a0012_cmd_panel",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
