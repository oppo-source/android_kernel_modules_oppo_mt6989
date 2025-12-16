// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>

#include "mtk_dsi.h"

#include "dsi_iris_api.h"
#include "dsi_iris_mtk_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "pw_iris_lightup_ocp.h"
#include "pw_iris_loop_back.h"
#include "dsi_iris_lp.h"
#include "pw_iris_lp.h"
#include "pw_iris_pq.h"
#include "pw_iris_ioctl.h"
#include "pw_iris_lut.h"
#include "pw_iris_gpio.h"
#include "pw_iris_timing_switch.h"
#include "pw_iris_log.h"
#include "pw_iris_memc.h"
#include "pw_iris_memc_helper.h"
#include "pw_iris_i2c.h"
#include "pw_iris_dts_fw.h"
#include "dsi_iris_memc.h"
#include "dsi_iris_dual.h"
#include "dsi_iris_cmpt.h"

static void _iris_i2c_preload_work(struct work_struct *work)
{
        struct iris_cfg *pcfg = iris_get_cfg();
        struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

        if (!pcfg || !pcfg_ven || !pcfg_ven->panel_ext)
                return;

        pcfg->iris_i2c_preload = true;
		iris_reset_sys_domain();
        iris_enable(NULL);
        pcfg->iris_i2c_preload = false;
}

static void _iris_i2c_preload_work_init(void)
{
        struct iris_cfg *pcfg = iris_get_cfg();
        struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

        if (pcfg && pcfg_ven && pcfg_ven->panel_ext && pcfg->valid >= PARAM_PARSED
                        && (dsi_iris_get_panel_mode() == IRIS_VIDEO_MODE))
                INIT_WORK(&pcfg->iris_i2c_preload_work, _iris_i2c_preload_work);
}

void iris_init_i7p(struct drm_panel *panel, struct mtk_panel_ext *panel_ext, struct drm_connector *conn)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	void *display = NULL;

	if (iris_virtual_connector(conn)) {
		//pcfg_ven->drm_panel_2nd = panel;
		pcfg_ven->mtk_panel_ext_2nd = panel_ext;
		//pcfg_ven->drm_connector_2nd = conn;
		pcfg_ven->mtk_dsi_2nd = container_of(conn, struct mtk_dsi, conn);
		IRIS_LOGI("%s, 2nd panel return", __func__);
		return;
	}
	IRIS_LOGI("%s()", __func__);
	iris_driver_ops_init_i7p();
	pcfg_ven->conn = conn;
	pcfg_ven->panel_ext = panel_ext;
	if (panel->dev)
		pcfg->dsi_dev = to_mipi_dsi_device(panel->dev);

	pcfg->iris_initialized = false;
	pcfg->iris_i2c_read = iris_pure_i2c_single_read;
	pcfg->iris_i2c_write = iris_pure_i2c_single_write;
	pcfg->iris_i2c_burst_write = iris_pure_i2c_burst_write;
	pcfg->lightup_ops.acquire_panel_lock = iris_ddp_mutex_lock;
	pcfg->lightup_ops.release_panel_lock = iris_ddp_mutex_unlock;
	pcfg->lightup_ops.transfer = iris_dsi_send_cmds;
	pcfg->lightup_ops.obtain_cur_timing_info = dsi_iris_obtain_cur_timing_info;
	pcfg->lightup_ops.get_display_info = iris_debug_display_info_get;
	pcfg->lightup_ops.wait_vsync = iris_wait_vsync;
	pcfg->lightup_ops.send_pwil_cmd = iris_send_pwil_cmd;
	pcfg->lightup_ops.change_header = iris_change_header;
	pcfg->lightup_ops.vdo_mode_send_cmd_with_handle = iris_vdo_mode_send_cmd_with_handle;
	pcfg->get_panel_mode = dsi_iris_get_panel_mode;
	pcfg->ioctl_ops.get_selected_configure = NULL;

	pcfg->iris_memc_ops.iris_memc_get_main_panel_timing_info = iris_get_main_panel_timing_info;
	pcfg->iris_memc_ops.iris_memc_get_main_panel_dsc_en_info = iris_get_main_panel_curr_mode_dsc_en;

	pcfg->iris_memc_ops.iris_memc_get_aux_panel_timing_info = NULL;
	pcfg->iris_memc_ops.iris_memc_get_aux_panel_dsc_en_info = NULL;
	pcfg->iris_memc_ops.iris_memc_try_panel_lock = NULL;
	pcfg->iris_memc_ops.iris_register_osd_irq = NULL;
	pcfg->iris_memc_ops.iris_memc_create_pps_buf_cmd = NULL;
	pcfg->iris_memc_ops.iris_memc_aux_panel_initialized = NULL;
	pcfg->iris_memc_ops.iris_set_idlemgr = iris_set_idlemgr;
	pcfg->iris_memc_ops.iris_set_idle_check_interval = iris_set_idle_check_interval;
	pcfg->wait_pre_framedone = NULL;
	pcfg->iris_core_lightup = iris_core_lightup;
	pcfg->wait_cmdq_done = iris_wait_for_bypass_cmdq_done;
	pcfg->cmdq_empty = iris_is_cmdq_empty;
#ifdef IRIS_EXT_CLK
	pcfg->iris_clk_set = iris_core_clk_set;
#endif
	pcfg->platform_ops.fill_desc_para = iris_cmd_desc_para_fill;
	pcfg->set_esd_status = iris_set_esd_status;
	pcfg->iris_is_read_cmd = iris_is_read_cmd;
	pcfg->iris_is_last_cmd = NULL;
	pcfg->iris_is_curmode_cmd_mode = iris_is_curmode_cmd_mode;
	pcfg->iris_is_curmode_vid_mode = iris_is_curmode_vid_mode;
	pcfg->iris_set_msg_flags = NULL;
	pcfg->iris_switch_cmd_type = iris_switch_cmd_type;
	pcfg->iris_set_msg_ctrl = NULL;

	pcfg->aod = false;
	pcfg->fod = false;
	pcfg->fod_pending = false;
	pcfg->platform_type = 1; //need to use ASIC
	pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE; //default abyp

	pcfg->n2m_ratio = 1;
	pcfg->dtg_ctrl_pt = 0;
	pcfg->iris_pwil_mode_state = 2;

	pcfg->frc_pq_guided_level = 1;
	pcfg->frc_pq_dejaggy_level = 1;
	pcfg->frc_pq_peaking_level = 1;
	pcfg->frc_pq_DLTI_level = 1;

	pcfg->frc_label = 0;
	pcfg->frc_demo_window = 0;
	pcfg->dev = panel->dev;
	pcfg->memc_chain = false;
	atomic_set(&pcfg->fod_cnt, 0);

	iris_init_memc();
	iris_init_timing_switch();
	iris_lp_init();
	pcfg->lp_ctrl.force_exit_ulps_during_switching = false;
	pcfg->iris_mipi1_power_on_pending_en = false;
	pcfg->memc_chain_en = false;

#ifdef IRIS_EXT_CLK // skip ext clk
	//pcfg->ext_clk = devm_clk_get(&display->pdev->dev, "pw_bb_clk2");
#endif

	if (!iris_virtual_connector(conn)) {
		pw_iris_dbgfs_lp_init(display);
		iris_dbgfs_pq_init();
		iris_dbgfs_cont_splash_init(display);
		iris_dbgfs_memc_init();
		iris_dbgfs_loop_back_init(display);
		iris_dbgfs_adb_type_init(display);
		iris_dbgfs_fw_calibrate_status_init();
		iris_dbgfs_status_init(display);
		iris_dbgfs_scl_init();
		iris_dbg_gpio_init();
	}
	//_iris_get_vreg();
	mutex_init(&pcfg->gs_mutex);
	mutex_init(&pcfg->ioctl_mutex);
	mutex_init(&pcfg->i2c_read_mutex);
	iris_driver_register();
	iris_pure_i2c_bus_init();
	iris_i2c_bus_init();
	iris_register_osd_irq();
	pcfg->iris_i2c_preload = false;
	_iris_i2c_preload_work_init();
}

int iris_lightoff_i7p(bool dead,
		struct iris_cmd_set *off_cmds)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int lightup_opt = iris_lightup_opt_get();
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	if (pcfg->valid < PARAM_PREPARED) {
		pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
		if (iris_virtual_connector(pcfg_ven->conn) && off_cmds)
			iris_abyp_send_panel_cmd(off_cmds);
		return 0;
	}
	// using iris_send_noncmdq_cmds to send dsi commands
	iris_set_valid(PARAM_PARSED);

	pcfg->metadata = 0; // clean metadata
	pcfg->dtg_ctrl_pt = 0;

	if (iris_virtual_connector(pcfg_ven->conn)) {
		IRIS_LOGD("no need to light off for 2nd panel.");
		return 0;
	}

	if ((lightup_opt & 0x10) == 0)
		pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE; //clear to ABYP mode

	IRIS_LOGI("%s(%d), panel %s, mode: %s(%d) ---", __func__, __LINE__,
			dead ? "dead" : "alive",
			pcfg->abyp_ctrl.abypass_mode == PASS_THROUGH_MODE ? "PT" : "ABYP",
			pcfg->abyp_ctrl.abypass_mode);
	if (off_cmds && (!dead)) {
		if (pcfg->abyp_ctrl.abypass_mode == PASS_THROUGH_MODE)
			iris_pt_send_panel_cmd(off_cmds);
		else
			iris_abyp_send_panel_cmd(off_cmds);
	}
	iris_lightoff_memc();
	iris_quality_setting_off();
	iris_lp_setting_off();
	iris_memc_setting_off();
	iris_dtg_update_reset();
	iris_clear_aod_state();
	pcfg->panel_pending = 0;
	#if 0
	iris_set_pinctrl_state(false);
	#endif
	if (pcfg->crtc0_old_interval != 0
		&& pcfg->iris_memc_ops.iris_set_idle_check_interval)
		pcfg->iris_memc_ops.iris_set_idle_check_interval(0, pcfg->crtc0_old_interval);

	IRIS_LOGI("%s(%d) ---", __func__, __LINE__);

	return 0;
}

