// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <video/mipi_display.h>
#include <drm/drm_modes.h>

#include "mtk_drm_mmp.h"
#include "drm_internal.h"
#include "mtk_drm_debugfs.h"
#include "mtk_dsi.h"
#include "mtk_drm_crtc.h"

#include "dsi_iris_api.h"
#include "dsi_iris_mtk_api.h"
#include "dsi_iris_lightup.h"
#include "pw_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "pw_iris_lp.h"
#include "pw_iris_pq.h"
#include "pw_iris_ioctl.h"
#include "pw_iris_lut.h"
#include "pw_iris_loop_back.h"
#include "pw_iris_gpio.h"
//#include "pw_iris_emv_i7.h"
#include "dsi_iris_dual.h"
#include "pw_iris_timing_switch.h"
#include "pw_iris_log.h"
#include "pw_iris_memc.h"
#include "dsi_iris_memc.h"
#include "pw_iris_i3c.h"
#include "pw_iris_i2c.h"
#include "dsi_iris_cmpt.h"
#include "pw_iris_dts_fw.h"
#include "pw_iris_lightup.h"


#define IRIS_CHIP_VER_0   0
#define IRIS_CHIP_VER_1   1
#define IRIS_OCP_HEADER_ADDR_LEN  8

#define calc_space_left(x, y) (x - y%x)
#define NON_EMBEDDED_BUF_SIZE (512*1024)  //512k

static struct iris_vendor_cfg gcfg_ext = {0};

//static int _iris_dbgfs_cont_splash_init(void);
static void _iris_send_cont_splash_pkt(uint32_t type);
//static int _iris_update_pq_seq(struct iris_update_ipopt *popt, int len);
//static void _iris_update_desc_last(struct iris_cmd_desc *pcmd,
//		int count, bool last_cmd);
//static int _iris_set_pkt_last(struct iris_cmd_desc *cmd, int32_t cmd_cnt, uint32_t add_last_flag);
/*
static int _iris_get_vreg(void)
{
	int rc = 0;
	int i;
	struct regulator *vreg = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_panel *panel = pcfg->panel;

	for (i = 0; i < pcfg->iris_power_info.count; i++) {
		vreg = devm_regulator_get(panel->parent,
				pcfg->iris_power_info.vregs[i].vreg_name);
		rc = IS_ERR(vreg);
		if (rc) {
			IRIS_LOGE("failed to get %s regulator",
					pcfg->iris_power_info.vregs[i].vreg_name);
			goto error_put;
		}
		pcfg->iris_power_info.vregs[i].vreg = vreg;
	}

	return rc;
error_put:
	for (i = i - 1; i >= 0; i--) {
		devm_regulator_put(pcfg->iris_power_info.vregs[i].vreg);
		pcfg->iris_power_info.vregs[i].vreg = NULL;
	}
	return rc;
}

static int _iris_put_vreg(void)
{
	int rc = 0;
	int i;
	struct iris_cfg *pcfg = iris_get_cfg();

	for (i = pcfg->iris_power_info.count - 1; i >= 0; i--)
		devm_regulator_put(pcfg->iris_power_info.vregs[i].vreg);

	return rc;
}

bool iris_virtual_connector(struct drm_connector *c)
{
	struct mtk_dsi *dsi;
	if (c == NULL) {
		IRIS_LOGE("drm_connector is NULL");
		return false;
	}
	dsi = container_of(c, struct mtk_dsi, conn);

}
*/
void iris_esd_init(void)
{
	u32 index = 0;
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct mtk_panel_params *params = NULL;

	if (pcfg_ven->panel_ext && pcfg_ven->panel_ext->params) {
		params = pcfg_ven->panel_ext->params;

		for (index = 0; index < ESD_CHK_NUM; index++) {
			if (params->lcm_esd_check_table[index].cmd == 0)
				break;

			pcfg_ven->esd_chk_val[index] = params->lcm_esd_check_table[index].para_list[0];
			IRIS_LOGV("%s() esd_chk_val[%d] = 0x%x", __func__, index, pcfg_ven->esd_chk_val[index]);
		}

		iris_set_esd_check_num(index);
		pcfg_ven->esd_read_flag = false;
		pcfg_ven->esd_read_index = 0;
		pcfg_ven->is_esd_check_ongoing = false;
	} else
		IRIS_LOGE("[%s](%d) panel is null!", __func__, __LINE__);
}

void iris_ddp_mutex_lock(void)
{
	struct drm_crtc *crtc1 = iris_get_vendor_cfg()->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc1);

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	mtk_drm_set_idlemgr(crtc1, 0, 0);
}

void iris_ddp_mutex_unlock(void)
{
	struct drm_crtc *crtc1 = iris_get_vendor_cfg()->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc1);

	mtk_drm_set_idlemgr(crtc1, 1, 0);
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
}

int dsi_iris_obtain_cur_timing_info(struct iris_mode_info *timing_info)
{
	int ret = -EINVAL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (timing_info) {
		dsi_mode_to_iris_mode(timing_info, &pcfg->timing);
		ret = 0;
	}
	return ret;
}

int dsi_iris_get_panel_mode(void)
{
	int mode = IRIS_CMD_MODE;
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	if (pcfg_ven->mtk_comp)
		mode = mtk_dsi_is_cmd_mode(pcfg_ven->mtk_comp)
			? IRIS_CMD_MODE: IRIS_VIDEO_MODE;
	return mode;
}

void iris_send_pwil_cmd(struct iris_cmd_set *pcmdset, u32 addr, u32 meta)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->force_i2c_type)
		pcfg->iris_i2c_write(addr, meta);
	else
		iris_ocp_write_val(addr, meta);
}

struct iris_vendor_cfg *iris_get_vendor_cfg(void)
{
	return &gcfg_ext;
}

void iris_init(struct drm_panel *panel, struct mtk_panel_ext *panel_ext, struct drm_connector *conn)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	// Dynamic Compatibility.
	iris_query_capability();
	iris_global_var_init();

	if (pcfg->iris_chip_type == CHIP_IRIS7P)
		iris_init_i7p(panel, panel_ext, conn);
	else
		IRIS_LOGE("[%s](%d) iris chip type error!", __func__, __LINE__);

	iris_esd_init();
}

void iris_deinit(struct drm_connector *conn)
{
	int i;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_is_chip_supported())
		return;

	if (iris_virtual_connector(conn))
		return;

#if 0  //def IRIS_EXT_CLK // skip ext clk
	if (pcfg->ext_clk) {
		devm_clk_put(&display->pdev->dev, pcfg->ext_clk);
		pcfg->ext_clk = NULL;
	}
#endif

	for (i = 0; i < iris_get_cmd_list_cnt(); i++)
		iris_free_ipopt_buf(i);
	iris_free_ipopt_buf(IRIS_LUT_PIP_IDX);

	if (pcfg->pq_update_cmd.update_ipopt_array) {
		kfree(pcfg->pq_update_cmd.update_ipopt_array);
		pcfg->pq_update_cmd.update_ipopt_array = NULL;
		pcfg->pq_update_cmd.array_index = 0;
	}

	iris_free_seq_space();

	//_iris_put_vreg();
	iris_sysfs_status_deinit();
	iris_deinit_timing_switch();
	iris_driver_unregister();
	iris_pure_i2c_bus_exit();
	iris_i2c_bus_exit();
}

void iris_control_pwr_regulator(bool on)
{
#if 0
	int rc = 0;
	struct iris_cfg *pcfg = NULL;

	if (!iris_is_chip_supported())
		return;

	pcfg = iris_get_cfg();
	//rc = dsi_pwr_enable_regulator(&pcfg->iris_power_info, on);
	if (rc)
		IRIS_LOGE("failed to power %s iris", on ? "on" : "off");
#endif
}

void iris_power_on(void)
{
	if (!iris_is_chip_supported())
		return;
#if 0
	IRIS_LOGI("%s(), for [%s] %s, secondary: %s",
			__func__,
			panel->name, panel->type,
			panel->is_secondary ? "true" : "false");

	if (panel->is_secondary)
		return;
#endif

#ifdef IRIS_HDK_DEV // skip power control
	return;
#endif

	if (iris_vdd_valid()) {
		iris_enable_vdd();
	} else { // No need to control vdd and clk
		IRIS_LOGW("%s(), vdd does not valid, use pmic", __func__);
		iris_control_pwr_regulator(true);
	}

	usleep_range(5000, 5000);
}
EXPORT_SYMBOL(iris_power_on);

void iris_power_off(void)
{
	if (!iris_is_chip_supported())
		return;
	if (iris_vdd_valid())
		iris_disable_vdd();
	else
		iris_control_pwr_regulator(false);
}
EXPORT_SYMBOL(iris_power_off);

bool iris_check_dsc_enable(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	if (pcfg_ven->panel_ext && pcfg_ven->panel_ext->params)
		return pcfg_ven->panel_ext->params->dsc_params.enable;

	return false;
}

bool iris_check_2nd_dsc_enable(void)
{
	struct mtk_panel_params *ext_panel_params = NULL;

	ext_panel_params = iris_get_ext_panel_params();
	if (ext_panel_params)
		return ext_panel_params->dsc_params.enable;

	return false;
}

int iris_get_vtotal(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	return pcfg_ven->dsp_mode->vtotal;
}

int iris_get_htotal(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	return pcfg_ven->dsp_mode->htotal;
}

#if 0
bool iris_virtual_display(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct drm_connector *conn = pcfg_ven->conn;

	if (conn)
		return iris_virtual_connector(conn);

	return false;
}
#endif

static u32 iris_get_clkrate(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	if (pcfg_ven->panel_ext && pcfg_ven->panel_ext->params)
		return pcfg_ven->panel_ext->params->data_rate;

	return 0;
}

static u32 iris_get_2nd_clkrate(void)
{
	struct mtk_panel_params *ext_panel_params = NULL;

	ext_panel_params = iris_get_ext_panel_params();
	if (ext_panel_params)
		return ext_panel_params->data_rate;

	return 0;
}
#if 0
bool iris_is_virtual_encoder_phys(void *phys_enc)
{

	struct sde_encoder_phys *phys_encoder = phys_enc;
	struct sde_connector *c_conn = NULL;

	if (phys_encoder == NULL)
		return false;

	if (phys_encoder->connector == NULL)
		return false;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return false;

	display = c_conn->display;
	if (display == NULL)
		return false;

	if (!iris_virtual_display())
		return false;


	return true;
}
#endif

void iris_sync_timing(struct iris_mode_info *ptiming,
		struct drm_display_mode *dsp_mode)
{
	if (!ptiming || !dsp_mode)
		return;

	ptiming->refresh_rate = drm_mode_vrefresh(dsp_mode);
	ptiming->h_active = dsp_mode->hdisplay;
	ptiming->v_active = dsp_mode->vdisplay;
	ptiming->h_back_porch = dsp_mode->htotal - dsp_mode->hsync_end;
	ptiming->h_sync_width = dsp_mode->hsync_end - dsp_mode->hsync_start;
	ptiming->h_front_porch = dsp_mode->hsync_start - dsp_mode->hdisplay;
	ptiming->v_back_porch = dsp_mode->vtotal - dsp_mode->hsync_end;
	ptiming->v_sync_width = dsp_mode->vsync_end - dsp_mode->vsync_start;
	ptiming->v_front_porch = dsp_mode->vsync_start - dsp_mode->vdisplay;
	ptiming->clk_rate_hz = iris_get_clkrate();
	ptiming->dsc_enabled = iris_check_dsc_enable();
}

void iris_sync_aux_timing(struct iris_mode_info *ptiming,
		struct drm_display_mode *dsp_mode)
{
	if (!ptiming || !dsp_mode)
		return;

	ptiming->refresh_rate = drm_mode_vrefresh(dsp_mode);
	ptiming->h_active = dsp_mode->hdisplay;
	ptiming->v_active = dsp_mode->vdisplay;
	ptiming->h_back_porch = dsp_mode->htotal - dsp_mode->hsync_end;
	ptiming->h_sync_width = dsp_mode->hsync_end - dsp_mode->hsync_start;
	ptiming->h_front_porch = dsp_mode->hsync_start - dsp_mode->hdisplay;
	ptiming->v_back_porch = dsp_mode->vtotal - dsp_mode->hsync_end;
	ptiming->v_sync_width = dsp_mode->vsync_end - dsp_mode->vsync_start;
	ptiming->v_front_porch = dsp_mode->vsync_start - dsp_mode->vdisplay;
	ptiming->clk_rate_hz = iris_get_2nd_clkrate();
	ptiming->dsc_enabled = iris_check_2nd_dsc_enable();
}

void iris_sync_cur_timing(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	iris_sync_timing(&pcfg->timing, pcfg_ven->dsp_mode);
}

void iris_get_panel_params(struct drm_device *drm, struct drm_display_mode **dsp_mode)
{
	u8 *name = NULL;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct mtk_ddp_comp *mtk_comp;
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	IRIS_LOGI("crtc num1=%d num =%d", drm->num_crtcs, drm->mode_config.num_crtc);
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(drm)->mode_config.crtc_list,
			typeof(*crtc), head);
	if (!crtc) {
		IRIS_LOGE("find crtc fail\n");
		return;
	}
	pcfg_ven->crtc = crtc;
	mtk_crtc = to_mtk_crtc(crtc);

	mtk_comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_DSI0);
	if (!mtk_comp) {
		IRIS_LOGE("can not mtk comp\n");
		return;
	}
	pcfg_ven->mtk_comp = mtk_comp;

	//obtain panel name
	mtk_ddp_comp_io_cmd(mtk_comp, NULL, GET_PANEL_NAME, &name);
	IRIS_LOGI("panel name = %s", name);
	//pcfg->panel_name = name;

	//obtain panel timing
	mtk_ddp_comp_io_cmd(mtk_comp, NULL, DSI_GET_TIMING, dsp_mode);
}

static int iris_obtain_panel_name(u8 *panel_name)
{
	int i = 0;
	u8 *ptr = NULL;
	int len = 0;
	u8 head[] = "panel-";
	int head_len = strlen(head);

	if (!panel_name)
		return -EINVAL;

	mtk_ddp_comp_io_cmd(iris_get_vendor_cfg()->mtk_comp, NULL,
			GET_PANEL_NAME, &ptr);
	/*remove panel-*/
	if (strncmp(ptr, head, head_len))
		len = sprintf(panel_name, "%s", &ptr[0]);
	else
		len = sprintf(panel_name, "%s", &ptr[head_len]);

	for (i = 0; i < len; i++)
		if (panel_name[i] == '-')
			panel_name[i] = '_';
	IRIS_LOGI("dts panel_name:%s", panel_name);
	return 0;
}

static struct device_node *_iris_find_lightup_node(void)
{
	struct device_node *lightup_node = NULL;
	struct device_node *chosen_node = NULL;
	char config[256] = "pxlw,iris_lightup_config_";
	char panel_name[256] = {};

	iris_obtain_panel_name(panel_name);
	strcat(config, panel_name);
	IRIS_LOGI("light up node name: %s", config);

	chosen_node = of_find_node_by_path("/chosen");
	if (!chosen_node) {
		IRIS_LOGE("chosen node is null");
		return chosen_node;
	}

	lightup_node = of_find_node_by_name(chosen_node, config);
	if (!lightup_node)
		IRIS_LOGE("%s(), failed to find %s node", __func__, config);
	return lightup_node;
}

static int32_t _iris_parse_tx_mode(
		struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u8 tx_mode;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return rc;

	pcfg->rx_mode = iris_dsi_get_mode();
	pcfg->tx_mode = pcfg->rx_mode;

	IRIS_LOGI("%s, panel_mode = %d", __func__, pcfg->rx_mode);
	rc = p_dts_ops->read_u8(np, "pxlw,iris-tx-mode", &tx_mode);
	if (!rc) {
		IRIS_LOGI("get property: pxlw, iris-tx-mode: %d", tx_mode);
		//pcfg->tx_mode = tx_mode;
	}
	if (pcfg->rx_mode == pcfg->tx_mode)
		pcfg->pwil_mode = PT_MODE;
	else
		pcfg->pwil_mode = RFB_MODE;

	IRIS_LOGI("%s(), pwil mode: %d", __func__, pcfg->pwil_mode);
	return 0;
}

int iris_parse_param(void *dev)
{
	int32_t ret = 0;
	struct drm_device *drm = (struct drm_device *)dev;
	struct device_node *lightup_node = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	pcfg->valid = PARAM_EMPTY;	/* empty */
	pcfg_ven->drm = drm;

	spin_lock_init(&pcfg->iris_1w_lock);
	init_completion(&pcfg->frame_ready_completion);
	//parse panel information
	iris_get_panel_params(drm, &pcfg_ven->dsp_mode);
	iris_sync_cur_timing();

	lightup_node = _iris_find_lightup_node();
	if (lightup_node) {
		iris_set_dts_ops(DTS_CTX_FROM_IMG);
		iris_query_capability();
		_iris_parse_tx_mode(lightup_node, pcfg);
		ret = pw_iris_parse_param(lightup_node);
	}
	return ret;
}

static uint32_t _iris_calculate_delay_us(uint32_t payload_size, uint32_t cmd_num)
{
	uint32_t delay_us = 0;
	uint32_t panel_mbit_clk = 0;
	uint32_t lane_num = 4;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGV("%s(%d), clk_rate_hz is %llu"/*, num_data_lanes is %d"*/, __func__, __LINE__,
		pcfg->timing.clk_rate_hz/*,
		pcfg->panel->host_config.num_data_lanes*/);

	if (pcfg->timing.clk_rate_hz)
		panel_mbit_clk = pcfg->timing.clk_rate_hz / 1000000;

	if (!panel_mbit_clk) {
		IRIS_LOGE("%s(%d), panel_mbit_clk is 0, default set to 50MHz.",
			__func__, __LINE__);
		panel_mbit_clk = 50;
	}
#if 0
	lane_num = pcfg->panel->host_config.num_data_lanes;
	if (!lane_num) {
		/*default set to 4 lanes*/
		lane_num = 4;
	}
#endif
	/* follow:
	 *	8*(total_payload_size + total_command_num*6)*(1+inclk/pclk)/(lane_num*bitclk)
	 *	assume inclk/pclk = 2, this is the max value
	 */
	delay_us = 8 * (payload_size + cmd_num * 6) * (1+2) / (lane_num * panel_mbit_clk);

	return delay_us;
}

void iris_insert_delay_us(uint32_t payload_size, uint32_t cmd_num)
{
	uint32_t delay_us = 0;

	IRIS_LOGD("%s, payload_size is %d, cmd_num is %d",
		__func__, payload_size, cmd_num);

	if ((!payload_size) || (!cmd_num))
		return;

	if ((payload_size > 4096) || (cmd_num > 128))
		IRIS_LOGE("%s, it is risky to send such packets, payload_size %d, cmd_num %d",
			__func__, payload_size, cmd_num);
	/*embedded size is 240, non-embedded size is 256, use 240 default*/
	delay_us = _iris_calculate_delay_us(payload_size, payload_size/240 + 1);

	IRIS_LOGD("%s(%d): delay_us is %d", __func__, __LINE__, delay_us);

	if (delay_us)
		udelay(delay_us);
}

void iris_change_header(void *comp)
{
	int i = 0;
	struct iris_cmd_comp *pcmd_comp = comp;

	for (i = 0; i < pcmd_comp->cnt; i++) {
		u32 *ptr = (u32 *)pcmd_comp->cmd[i].msg.tx_buf;

		/*ocp burst and directbus need to change*/
		if ((ptr[0] & 0xFF) == 0x00 || (ptr[0] & 0xFF) == 0x0c)
			ptr[0] |= 0xF0;
	}
}

int iris_lightup(void)
{
	ktime_t ktime0;
	ktime_t ktime1;
	uint32_t timeus0 = 0;
	uint32_t timeus1 = 0;
	uint8_t type = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(), start +++, cmd list index: %u",
			__func__,
			iris_get_cmd_list_index());

	ktime0 = ktime_get();
	_iris_pre_lightup();
	_iris_load_mcu();

	type = iris_get_cont_splash_type();

	/*use to debug cont splash*/
	if (type == IRIS_CONT_SPLASH_LK) {
		IRIS_LOGI("%s(%d), enter cont splash", __func__, __LINE__);
		_iris_send_cont_splash_pkt(IRIS_CONT_SPLASH_LK);
	} else {
		_iris_send_lightup_pkt();
		iris_update_gamma();
		iris_ioinc_filter_ratio_send();
	}


	ktime1 = ktime_get();
	if (type == IRIS_CONT_SPLASH_LK)
		IRIS_LOGI("%s(), exit cont splash", __func__);
	else
		/*continuous splahs should not use dma setting low power*/
		iris_lp_enable_post();

	iris_tx_buf_to_vc_set(pcfg->vc_ctrl.vc_arr[VC_PT]);

	iris_update_last_pt_timing();
	iris_sdr2hdr_set_img_size(pcfg->timing.h_active,
			pcfg->timing.v_active);

	timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
	timeus1 = (u32) ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime1);
	IRIS_LOGI("%s() spend time0 %d us, time1 %d us.",
			__func__, timeus0, timeus1);

#ifdef IRIS_MIPI_TEST
	_iris_read_power_mode(panel);
#endif
	pcfg->abyp_ctrl.preloaded = true;
	IRIS_LOGI("%s(), end +++", __func__);

	return 0;
}

void iris_core_lightup(void)
{
	iris_lightup();
}

int iris_enable(struct iris_cmd_set *on_cmds)
{
	int rc = 0;
	int abyp_status_gpio;
	int prev_mode;
	int lightup_opt = iris_lightup_opt_get();
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 value = 0;
	ktime_t ktime0 = 0;
	ktime_t ktime1 = 0;
	ktime_t ktime2 = 0;
	ktime_t ktime3 = 0;
	ktime_t ktime4 = 0;
	uint32_t timeus0 = 0;
	uint32_t timeus1 = 0;
	uint32_t timeus2 = 0;
	uint32_t timeus3 = 0;
	uint32_t timeus4 = 0;

#ifdef IRIS_EXT_CLK
	iris_clk_enable(false);
#endif

	if ((pcfg->valid == PARAM_EMPTY) || (pcfg->valid < PARAM_PREPARED && iris_is_sleep_abyp_mode())) {
		if (on_cmds != NULL)
			rc = iris_abyp_send_panel_cmd(on_cmds);
		goto end;
	}

	if (IRIS_IF_LOGI())
		ktime0 = ktime_get();

	//SDE_ATRACE_BEGIN("iris_enable");
	iris_update_panel_timing(&pcfg->timing);
	iris_lp_enable_pre();

	/* Force Iris work in ABYP mode */
	if (iris_is_abyp_timing(&pcfg->timing))
		pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
	if (!(lightup_opt & 0x1)) {
		if ((pcfg->iris_chip_type == CHIP_IRIS7P) &&
				(pcfg->iris_i2c_read))
			pcfg->iris_i2c_read(0xF00000CC, &value);
		IRIS_LOGI("chip id value 0x%8x", value);
		if ((value >> 16) != 0x7777) {
			lightup_opt |= 0x1;
			iris_lightup_opt_set(lightup_opt);
		}
	}

	IRIS_LOGI("%s(), mode:%d, rate: %d, v: %d, on_opt:0x%x",
			__func__,
			pcfg->abyp_ctrl.abypass_mode,
			pcfg->timing.refresh_rate,
			pcfg->timing.v_active,
			lightup_opt);

	/* support lightup_opt */
	if (lightup_opt & 0x1) {
		if (on_cmds != NULL)
			rc = iris_abyp_send_panel_cmd(on_cmds);
		IRIS_LOGI("%s(), force ABYP lightup.", __func__);
		//SDE_ATRACE_END("iris_enable");
		goto end;
	}

	if (IRIS_IF_LOGI())
		ktime1 = ktime_get();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_bulksram_power_domain_proc_i7();
	else if (pcfg->iris_chip_type == CHIP_IRIS7P)
		iris_bulksram_power_domain_proc_i7p();

	if (iris_is_sleep_abyp_mode()) {
		if (pcfg->iris_chip_type == CHIP_IRIS7)
			iris_disable_temp_sensor();
		iris_sleep_abyp_power_down();

		if (IRIS_IF_LOGI())
			ktime2 = ktime_get();
		if (IRIS_IF_LOGI()) {
			timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
			timeus1 = (u32) ktime_to_us(ktime2) - (u32)ktime_to_us(ktime1);
		}
		IRIS_LOGI("%s(), iris takes total %d us, prepare %d us, low power %d us",
				__func__, timeus0 + timeus1, timeus0, timeus1);
	} else {
		prev_mode = pcfg->abyp_ctrl.abypass_mode;
		if (pcfg->iris_i2c_preload)
			goto _iris_lightup;
		abyp_status_gpio = iris_exit_abyp(true);
		if (abyp_status_gpio == 1) {
			IRIS_LOGE("%s(), failed to exit abyp!", __func__);
			//sSDE_ATRACE_END("iris_enable");
			goto end;
		}

		if (IRIS_IF_LOGI())
			ktime2 = ktime_get();

#if defined(CONFIG_PXLW_FPGA_IRIS)
		if (iris_platform_get() == IRIS_FPGA)
			iris_fpga_type_get();
#endif
_iris_lightup:
		rc = iris_lightup();
		pcfg->abyp_ctrl.abypass_mode = PASS_THROUGH_MODE;
		pcfg->iris_initialized = true;
		if (pcfg->iris_i2c_preload) {
			pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
			if (IRIS_IF_LOGI()) {
				ktime3 = ktime_get();
				ktime4 = ktime_get();
			}
			goto iris_enable_exit;
		}
		if (IRIS_IF_LOGI())
			ktime3 = ktime_get();

		if (on_cmds != NULL) {
			//SDE_ATRACE_BEGIN("iris_pt_send_panel_cmd");
			rc = iris_pt_send_panel_cmd(on_cmds);
			//SDE_ATRACE_END("iris_pt_send_panel_cmd");
		}

		if (IRIS_IF_LOGI())
			ktime4 = ktime_get();

		//Switch back to ABYP mode if need
		if ((iris_platform_get() != IRIS_FPGA) && !(iris_lightup_opt_get() & 0x2)) {
			if (prev_mode == ANALOG_BYPASS_MODE)
				iris_abyp_switch_proc(ANALOG_BYPASS_MODE);
		}
iris_enable_exit:
		if (IRIS_IF_LOGI()) {
			timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
			timeus1 = (u32) ktime_to_us(ktime2) - (u32)ktime_to_us(ktime1);
			timeus2 = (u32) ktime_to_us(ktime3) - (u32)ktime_to_us(ktime2);
			timeus3 = (u32) ktime_to_us(ktime4) - (u32)ktime_to_us(ktime3);
			timeus4 = (u32) ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime4);
		}
		IRIS_LOGI("%s(), iris takes total %d us, prepare %d us, enter PT %d us,"
				" light up %d us, exit PT %d us.",
				__func__,
				timeus0 + timeus1 + timeus2 + timeus4,
				timeus0, timeus1, timeus2, timeus4);
		if (on_cmds != NULL) {
			IRIS_LOGI("Send panel cmd takes %d us.", timeus3);
		}
	}
	//SDE_ATRACE_END("iris_enable");

end:
#ifdef IRIS_EXT_CLK
	iris_clk_disable(false);
#endif
	return rc;
}
EXPORT_SYMBOL(iris_enable);


struct mtk_panel_params *iris_get_ext_panel_params(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct mtk_panel_funcs *panel_funcs;
	struct mtk_panel_params *ext_panel_params = NULL;
	struct mtk_crtc_state *mtk_state;

	if (pcfg_ven->mtk_dsi_2nd) {
		if (pcfg_ven->mtk_dsi_2nd->encoder.crtc) {
			panel_funcs = mtk_drm_get_lcm_ext_funcs(pcfg_ven->mtk_dsi_2nd->encoder.crtc);
			if (panel_funcs && panel_funcs->ext_param_get) {
				mtk_state = to_mtk_crtc_state(pcfg_ven->mtk_dsi_2nd->encoder.crtc->state);
				panel_funcs->ext_param_get(
					pcfg_ven->mtk_dsi_2nd->panel, &pcfg_ven->mtk_dsi_2nd->conn,
					&ext_panel_params, mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);
			}
		}
	}

	return ext_panel_params;

}
int iris_enable_secondary(struct drm_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct drm_display_mode *dsp_mode = NULL;
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct mtk_panel_params *ext_panel_params = NULL;
	struct mtk_drm_crtc *mtk_crtc = NULL;

	if (pcfg_ven->mtk_dsi_2nd) {
		if (pcfg_ven->mtk_dsi_2nd->encoder.crtc) {
			ext_panel_params = iris_get_ext_panel_params();
			mtk_crtc = container_of(pcfg_ven->mtk_dsi_2nd->encoder.crtc, struct mtk_drm_crtc, base);
			if (mtk_crtc) {
				struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
				if (comp) {
					mtk_ddp_comp_io_cmd(comp, NULL, DSI_GET_TIMING, &dsp_mode);
				}
			}
		}
	}
	iris_enable_memc(true);
	if (dsp_mode) {
		pcfg->frc_setting.hres_2nd = dsp_mode->hdisplay;
		pcfg->frc_setting.vres_2nd = dsp_mode->vdisplay;
		pcfg->frc_setting.refresh_rate_2nd = drm_mode_vrefresh(dsp_mode);
	} else {
		IRIS_LOGI("failed to get 2nd display timing!");
		pcfg->frc_setting.hres_2nd = 1080;
		pcfg->frc_setting.vres_2nd = 2400;
		pcfg->frc_setting.refresh_rate_2nd = 120;
	}
	if (ext_panel_params)
		pcfg->frc_setting.dsc_2nd = ext_panel_params->dsc_params.enable;
	pcfg->ap_mipi1_power_st = true;
	IRIS_LOGI("%s, %d*%d@%d - %s", __func__, pcfg->frc_setting.hres_2nd,
		pcfg->frc_setting.vres_2nd, pcfg->frc_setting.refresh_rate_2nd,
		pcfg->frc_setting.dsc_2nd ? "DSC" : "RAW");

	// disable idlemgr in eMV case or until FRC entry in dual-iMV
	iris_set_idlemgr(3, 0, 0);
	return 0;
}
EXPORT_SYMBOL(iris_enable_secondary);

int iris_disable_secondary(struct drm_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	pcfg->ap_mipi1_power_st = false;
	IRIS_LOGI("%s!", __func__);
	return 0;
}
EXPORT_SYMBOL(iris_disable_secondary);

void iris_update_2nd_active_timing_mtk(int hdisplay, int vdisplay, int vrefresh, bool dsc)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_mode_info timing = {
		.h_active = hdisplay,
		.v_active = vdisplay,
		.refresh_rate = vrefresh,
		.dsc_enabled = dsc,
	};

	if (pcfg == NULL)
		return;

	iris_update_2nd_active_timing(&timing);
	pcfg->ap_mipi1_power_st = true; // change to true
	iris_enable_memc(true);

	// disable idlemgr in eMV case or until FRC entry in dual-iMV
	iris_set_idlemgr(3, 0, 0);
}
EXPORT_SYMBOL(iris_update_2nd_active_timing_mtk);

#if 0
int iris_set_aod(struct dsi_panel *panel, bool aod)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg)
		return rc;

	if (panel->is_secondary)
		return rc;

	IRIS_LOGI("%s(%d), aod: %d", __func__, __LINE__, aod);
	if (pcfg->aod == aod) {
		IRIS_LOGI("[%s:%d] aod: %d no change", __func__, __LINE__, aod);
		return rc;
	}

	if (aod) {
		if (!pcfg->fod) {
			pcfg->abyp_prev_mode = pcfg->abyp_ctrl.abypass_mode;
			if (iris_get_abyp_mode() == PASS_THROUGH_MODE)
				iris_abyp_switch_proc(ANALOG_BYPASS_MODE);
		}
	} else {
		if (!pcfg->fod) {
			if (iris_get_abyp_mode() == ANALOG_BYPASS_MODE &&
					pcfg->abyp_prev_mode == PASS_THROUGH_MODE &&
					!pcfg->fod) {
				iris_abyp_switch_proc(PASS_THROUGH_MODE);
			}
		}
	}

	if (pcfg->fod_pending)
		pcfg->fod_pending = false;
	pcfg->aod = aod;

	return rc;
}

int iris_set_fod( bool fod)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg)
		return rc;

	if (panel->is_secondary)
		return rc;

	IRIS_LOGD("%s(%d), fod: %d", __func__, __LINE__, fod);
	if (pcfg->fod == fod) {
		IRIS_LOGD("%s(%d), fod: %d no change", __func__, __LINE__, fod);
		return rc;
	}

	if (!dsi_panel_initialized(panel)) {
		IRIS_LOGD("%s(%d), panel is not initialized fod: %d", __func__, __LINE__, fod);
		pcfg->fod_pending = true;
		atomic_set(&pcfg->fod_cnt, 1);
		pcfg->fod = fod;
		return rc;
	}

	if (fod) {
		if (!pcfg->aod) {
			pcfg->abyp_prev_mode = pcfg->abyp_ctrl.abypass_mode;
			if (iris_get_abyp_mode() == PASS_THROUGH_MODE)
				iris_abyp_switch_proc(ANALOG_BYPASS_MODE);
		}
	} else {
		/* pending until hbm off cmds sent in update_hbm 1->0 */
		pcfg->fod_pending = true;
		atomic_set(&pcfg->fod_cnt, 1);
	}

	pcfg->fod = fod;

	return rc;
}

int iris_post_fod(struct dsi_panel *panel)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg)
		return rc;

	if (panel->is_secondary)
		return rc;

	if (atomic_read(&pcfg->fod_cnt) > 0) {
		IRIS_LOGD("%s(%d), fod delay %d", __func__, __LINE__, atomic_read(&pcfg->fod_cnt));
		atomic_dec(&pcfg->fod_cnt);
		return rc;
	}

	IRIS_LOGD("%s(%d), fod: %d", __func__, __LINE__, pcfg->fod);

	if (pcfg->fod) {
		if (!pcfg->aod) {
			pcfg->abyp_prev_mode = pcfg->abyp_ctrl.abypass_mode;
			if (iris_get_abyp_mode() == PASS_THROUGH_MODE)
				iris_abyp_switch_proc(ANALOG_BYPASS_MODE);
		}
	} else {
		if (!pcfg->aod) {
			if (iris_get_abyp_mode() == ANALOG_BYPASS_MODE &&
					pcfg->abyp_prev_mode == PASS_THROUGH_MODE) {
				iris_abyp_switch_proc(PASS_THROUGH_MODE);
			}
		}
	}

	pcfg->fod_pending = false;

	return rc;
}

bool iris_get_aod(struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!panel || !pcfg)
		return false;

	return pcfg->aod;
}

static void _iris_clear_aod_state(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->aod) {
		pcfg->aod = false;
		pcfg->abyp_ctrl.abypass_mode = pcfg->abyp_prev_mode;
	}
}
#endif

static void _iris_send_cont_splash_pkt(uint32_t type)
{
	int seq_cnt = 0;
	uint32_t size = 0;
	const int iris_max_opt_cnt = 30;
	struct iris_ctrl_opt *opt_arr = NULL;
	struct iris_cfg *pcfg = NULL;
	struct iris_ctrl_seq *pseq_cs = NULL;

	size = IRIS_IP_CNT * iris_max_opt_cnt * sizeof(struct iris_ctrl_opt);
	opt_arr = vmalloc(size);
	if (opt_arr == NULL) {
		IRIS_LOGE("%s(), failed to malloc buffer!", __func__);
		return;
	}

	pcfg = iris_get_cfg();
	memset(opt_arr, 0xff, size);

	if (type == IRIS_CONT_SPLASH_LK) {
		pseq_cs = _iris_get_ctrl_seq_cs(pcfg);
		iris_send_assembled_pkt(pseq_cs->ctrl_opt, pseq_cs->cnt);
	} else if (type == IRIS_CONT_SPLASH_KERNEL) {
		iris_lp_enable_pre();
		seq_cnt = _iris_select_cont_splash_ipopt(type, opt_arr);
		iris_send_assembled_pkt(opt_arr, seq_cnt);
		iris_lp_enable_post();
		_iris_read_chip_id();
	} else if (type == IRIS_CONT_SPLASH_BYPASS_PRELOAD) {
		iris_enable(NULL);
	}

	vfree(opt_arr);
}

void iris_send_cont_splash(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int lightup_opt = iris_lightup_opt_get();
	uint32_t type;
	static bool first_enter;
	int rc = 0;

	if (!iris_is_chip_supported())
		return;

	rc = iris_set_pinctrl_state(true);
	if (rc) {
		IRIS_LOGE("%s() failed to set iris pinctrl, rc=%d\n", __func__, rc);
		return;
	}

	if (!first_enter) {
		type = iris_get_cont_type_with_timing_switch(&pcfg->timing);

		if (lightup_opt & 0x1)
			type = IRIS_CONT_SPLASH_NONE;

		if ((dsi_iris_get_panel_mode() == IRIS_VIDEO_MODE)
					&& (type == IRIS_CONT_SPLASH_BYPASS_PRELOAD)
					&& (pcfg->valid >= PARAM_PARSED)) {
			schedule_work(&pcfg->iris_i2c_preload_work);
			pcfg->valid = PARAM_LIGHTUP;
			first_enter = true;
			return;
		}

		pcfg->lightup_ops.acquire_panel_lock();
		//iris_dsi_pre_cmd(pcfg_ven->mtk_comp, pcfg->crtc);
		_iris_send_cont_splash_pkt(type);
		//iris_dsi_pos_cmd(pcfg_ven->mtk_comp, pcfg->crtc);
		pcfg->lightup_ops.release_panel_lock();

		pcfg->valid = PARAM_LIGHTUP;
		first_enter = true;
	}
}

int iris_lightoff(bool dead,
		struct iris_cmd_set *off_cmds)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7P)
		rc = iris_lightoff_i7p(dead, off_cmds);
	else
		IRIS_LOGE("[%s](%d) iris chip type error!", __func__, __LINE__);

	return rc;
}

int iris_in_self_recovery(void)
{
#if 0  /* TODO */
	return iris_emv_in_self_recovery();
#else
	return 0;
#endif
}

int iris_disable(bool dead, struct iris_cmd_set *off_cmds)
{
	return iris_lightoff(dead, off_cmds);
}
EXPORT_SYMBOL(iris_disable);

uint32_t iris_schedule_line_no_get(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t schedule_line_no, panel_vsw_vbp;

	if ((pcfg->frc_enabled) || (pcfg->pwil_mode == FRC_MODE))
		schedule_line_no = pcfg->ovs_delay_frc;
	else
		schedule_line_no = pcfg->ovs_delay;

	panel_vsw_vbp = pcfg->timing.v_back_porch +
			pcfg->timing.v_sync_width;

	if (pcfg->vsw_vbp_delay > panel_vsw_vbp)
		schedule_line_no += pcfg->vsw_vbp_delay - panel_vsw_vbp;

	if (pcfg->dtg_eco_enabled)
		schedule_line_no += pcfg->vsw_vbp_delay;

	return schedule_line_no;
}

#ifdef IRIS_EXT_CLK
void iris_clk_enable(bool is_secondary)
{
	struct iris_cfg *pcfg = iris_get_cfg();

#if 0
	if (is_secondary) {
		IRIS_LOGD("%s(), %d, skip enable clk in virtual channel", __func__, __LINE__);
		return;
	}
#endif
	if (pcfg->ext_clk && !pcfg->clk_enable_flag) {
		IRIS_LOGI("%s(), %d, enable ext clk", __func__, __LINE__);
		clk_prepare_enable(pcfg->ext_clk);
		pcfg->clk_enable_flag = true;
		usleep_range(5000, 5001);
	} else {
		if (!pcfg->ext_clk)
			IRIS_LOGE("%s(), %d, ext clk not exist!", __func__, __LINE__);
		if (pcfg->clk_enable_flag)
			IRIS_LOGI("%s(), %d, ext clk has enabled", __func__, __LINE__);
	}
}

void iris_clk_disable(bool is_secondary)
{
	struct iris_cfg *pcfg = iris_get_cfg();

#if 0
	if (is_secondary) {
		IRIS_LOGD("%s(), %d, skip disable clk in virtual channel", __func__, __LINE__);
		return;
	}
#endif
	if (pcfg->ext_clk && pcfg->clk_enable_flag) {
		IRIS_LOGI("%s(), %d, disable ext clk", __func__, __LINE__);
		clk_disable_unprepare(pcfg->ext_clk);
		pcfg->clk_enable_flag = false;
	} else {
		if (!pcfg->ext_clk)
			IRIS_LOGE("%s(), %d, ext clk not exist!", __func__, __LINE__);
		if (!pcfg->clk_enable_flag)
			IRIS_LOGI("%s(), %d, ext clk not enabled", __func__, __LINE__);
	}
}

void iris_core_clk_set(bool enable, bool is_secondary)
{
	if (enable)
		iris_clk_enable(is_secondary);
	else
		iris_clk_disable(is_secondary);

}
#endif

static ssize_t _iris_cont_splash_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	_iris_set_cont_splash_type(val);

	if ((val == IRIS_CONT_SPLASH_BYPASS_PRELOAD) &&
		(dsi_iris_get_panel_mode() == IRIS_VIDEO_MODE)) {
		schedule_work(&pcfg->iris_i2c_preload_work);
		return count;
	}

	if (val == IRIS_CONT_SPLASH_KERNEL) {
		//struct iris_cfg *pcfg = iris_get_cfg();
		pcfg->lightup_ops.acquire_panel_lock();
		_iris_send_cont_splash_pkt(val);
		pcfg->lightup_ops.release_panel_lock();
	} else if (val != IRIS_CONT_SPLASH_LK &&
			val != IRIS_CONT_SPLASH_NONE) {
		IRIS_LOGE("the value is %zu, need to be 1 or 2 3", val);
	}

	return count;
}

static ssize_t _iris_cont_splash_read(
		struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	uint8_t type;
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	type = iris_get_cont_splash_type();
	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", type);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_cont_splash_fops = {
	.open = simple_open,
	.write = _iris_cont_splash_write,
	.read = _iris_cont_splash_read,
};

int iris_wait_vsync(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	if (pcfg_ven->drm != NULL)
		drm_wait_one_vblank(pcfg_ven->drm, 0);

	return 0;
}

int iris_sync_panel_brightness(int32_t step, void *phys_enc)
{
	int rc = 0;
#if 0
	struct sde_encoder_phys *phys_encoder = phys_enc;
	struct sde_connector *c_conn = NULL;
	struct iris_cfg *pcfg;
	int rc = 0;

	if (phys_encoder == NULL)
		return -EFAULT;
	if (phys_encoder->connector == NULL)
		return -EFAULT;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return -EFAULT;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	pcfg = iris_get_cfg();

	if (pcfg->panel_pending == step) {
		IRIS_LOGI("sync pending panel %d %d,%d,%d",
				step, pcfg->panel_pending, pcfg->panel_delay,
				pcfg->panel_level);
		//SDE_ATRACE_BEGIN("sync_panel_brightness");
		if (step <= 2) {
			rc = c_conn->ops.set_backlight(&c_conn->base,
					display, pcfg->panel_level);
			if (pcfg->panel_delay > 0)
				usleep_range(pcfg->panel_delay, pcfg->panel_delay + 1);
		} else {
			if (pcfg->panel_delay > 0)
				usleep_range(pcfg->panel_delay, pcfg->panel_delay + 1);
			rc = c_conn->ops.set_backlight(&c_conn->base,
					display, pcfg->panel_level);
		}
		if (c_conn->bl_device)
			c_conn->bl_device->props.brightness = pcfg->panel_level;
		pcfg->panel_pending = 0;
		//SDE_ATRACE_END("sync_panel_brightness");
	}
#endif
	return rc;
}

int iris_dbgfs_cont_splash_init(void *display)
{
	int ret = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}
	if (debugfs_create_file("iris_cont_splash", 0644, pcfg->dbg_root, display,
				&iris_cont_splash_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	ret = pw_dbgfs_cont_splash_init(display);

	return ret;
}

void iris_prepare(void)
{
	static bool is_boot = true;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	char name[256] = {};
	u32 *payload = NULL;

	if (!iris_is_chip_supported())
		return;
#if 0
	if (display->panel->is_secondary)
		return;
#endif
	if (is_boot) {
		is_boot = false;

		if (pcfg->valid == PARAM_PARSED) {
			iris_obtain_panel_name(name);
			if (_iris_fw_parse_dts(name)) {
				pcfg->valid = PARAM_EMPTY;
				pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
				return;
			}
			iris_parse_memc_param();
			iris_init_panel_timing(pcfg_ven->conn);
			iris_frc_setting_init();
			iris_parse_lut_cmds(LOAD_GOLDEN_ONLY);
			iris_alloc_seq_space();
			iris_alloc_update_ipopt_space();
			payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, pcfg->id_sys_dma_gen_ctrl, 4);
			if (payload)
				pcfg->default_dma_gen_ctrl = payload[0];
			payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, ID_SYS_DMA_GEN_CTRL2, 4);
			if (payload)
				pcfg->default_dma_gen_ctrl_2 = payload[0];
			pcfg->valid = PARAM_PREPARED;	/* prepare ok */

			if (pcfg->iris_chip_type == CHIP_IRIS7)
				iris_bulksram_power_domain_proc_i7();
			else if (pcfg->iris_chip_type == CHIP_IRIS7P)
				iris_bulksram_power_domain_proc_i7p();

			if (iris_is_sleep_abyp_mode()) {
				if (pcfg->iris_chip_type == CHIP_IRIS7)
					iris_disable_temp_sensor();
				iris_sleep_abyp_power_down();
			}

			iris_send_cont_splash();
		}
	}
}

int iris_find_secondary_name(char *name)
{
	struct device_node *dsi0_node, *remote_node = NULL, *endpoint = NULL;
	const char *panel_name = NULL;
	char name_suffix[5] = ",2nd";

	dsi0_node = of_find_compatible_node(NULL, NULL, "mediatek,disp1_dsi0");
	if (dsi0_node) {
		endpoint = of_graph_get_next_endpoint(dsi0_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				IRIS_LOGW("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			IRIS_LOGI("dsi0 device node name:%s\n", remote_node->name);
		}
	}
	of_property_read_string(remote_node, "compatible", &panel_name);
	IRIS_LOGI("panel name %s", panel_name);
	strlcpy(name, panel_name, strlen(panel_name) + 1);
	strlcat(name, name_suffix, 64);
	IRIS_LOGI("panel 2nd name %s", name);

	return 0;
}

//module_platform_driver(iris_driver);
