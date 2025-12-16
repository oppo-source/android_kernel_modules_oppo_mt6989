// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2024.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "pw_iris_api.h"
#include "pw_iris_dts_fw.h"
#include "pw_iris_gpio.h"
#include "pw_iris_i2c.h"
#include "pw_iris_ioctl.h"
#include "pw_iris_lightup.h"
#include "pw_iris_lightup_ocp.h"
#include "pw_iris_log.h"
#include "pw_iris_loop_back.h"
#include "pw_iris_lp.h"
#include "pw_iris_lut.h"
#include "pw_iris_memc.h"
#include "pw_iris_memc_helper.h"
#include "pw_iris_pq.h"
#include "pw_iris_timing_switch.h"

static int iris_driver_initialized;

static int iris_catalog_init_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (iris_driver_initialized > 0) {
		IRIS_LOGI("%s() is already called! %d", __func__, iris_driver_initialized);
		iris_driver_initialized++;
		return 0;
	}
	iris_driver_initialized++;
	pcfg->pw_chip_func_ops.iris_memc_func_init_ = iris_memc_func_init_i7p;
	pcfg->pw_chip_func_ops.iris_memc_helper_setup_ = NULL;
	pcfg->pw_chip_func_ops.iris_timing_switch_setup_ = iris_timing_switch_setup_i7p;
	pcfg->pw_chip_func_ops.iris_debug_display_mode_get_ = iris_debug_display_mode_get_i7p;
	pcfg->pw_chip_func_ops.iris_debug_pq_info_get_ = NULL;
	pcfg->pw_chip_func_ops.iris_send_lut_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_send_lut_ = iris_send_lut_i7p;
	pcfg->pw_chip_func_ops.iris_configure_ = iris_configure_i7p;
	pcfg->pw_chip_func_ops.iris_configure_ex_ = iris_configure_ex_i7p;
	pcfg->pw_chip_func_ops.iris_configure_get_ = iris_configure_get_i7p;
	pcfg->pw_chip_func_ops.iris_dbgfs_adb_type_init_ = iris_dbgfs_adb_type_init_i7p;
	pcfg->pw_chip_func_ops.iris_ocp_write_mult_vals_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_parse_frc_setting_ = NULL;
	pcfg->pw_chip_func_ops.iris_mult_addr_pad_ = iris_mult_addr_pad_i7p;
	pcfg->pw_chip_func_ops.iris_get_dbc_lut_index_ = NULL;
	pcfg->pw_chip_func_ops.iris_clean_frc_status_ = NULL;
	pcfg->pw_chip_func_ops.iris_loop_back_validate_ = iris_loop_back_validate_i7p;
	pcfg->pw_chip_func_ops.iris_mipi_rx0_validate_ = iris_mipi_rx0_validate_i7p;
	pcfg->pw_chip_func_ops.iris_set_loopback_flag_ = iris_set_loopback_flag_i7p;
	pcfg->pw_chip_func_ops.iris_get_loopback_flag_ = iris_get_loopback_flag_i7p;
	pcfg->pw_chip_func_ops.iris_lp_preinit_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_lp_init_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_dynamic_power_set_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_disable_ulps_ = NULL;
	pcfg->pw_chip_func_ops.iris_enable_ulps_ = NULL;
	pcfg->pw_chip_func_ops.iris_dphy_itf_check_ = NULL;
	pcfg->pw_chip_func_ops.iris_pmu_bsram_set_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_dma_gen_ctrl_ = iris_dma_gen_ctrl_i7p;
	pcfg->pw_chip_func_ops.iris_global_var_init_ = iris_global_var_init_i7p;
	pcfg->pw_chip_func_ops.iris_pwil_update_i7p_ = iris_pwil_update_i7p;
	pcfg->pw_chip_func_ops.iris_abypass_switch_proc_ = NULL;
	pcfg->pw_chip_func_ops.iris_esd_check_ = iris_esd_check_i7p;
	pcfg->pw_chip_func_ops.iris_qsync_set_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_change_dpp_lutrgb_type_addr_i7_ = NULL;
	pcfg->pw_chip_func_ops.iris_parse_lut_cmds_ = iris_parse_lut_cmds_i7p;
	pcfg->pw_chip_func_ops.iris_get_hdr_enable_ = NULL;
	pcfg->pw_chip_func_ops.iris_quality_setting_off_ = iris_quality_setting_off_i7p;
	pcfg->pw_chip_func_ops.iris_end_dpp_ = iris_end_dpp_i7p;
	pcfg->pw_chip_func_ops.iris_pq_parameter_init_ = iris_pq_parameter_init_i7p;
	pcfg->pw_chip_func_ops.iris_cm_ratio_set_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_cm_ratio_set_ = iris_cm_ratio_set_i7p;
	pcfg->pw_chip_func_ops.iris_cm_color_gamut_set_ = iris_cm_color_gamut_set_i7p;
	pcfg->pw_chip_func_ops.iris_dpp_gamma_set_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_dpp_precsc_enable_ = iris_dpp_precsc_enable_i7p;
	pcfg->pw_chip_func_ops.iris_lux_set_ = iris_lux_set_i7p;
	pcfg->pw_chip_func_ops.iris_al_enable_ = iris_al_enable_i7p;
	pcfg->pw_chip_func_ops.iris_pwil_dport_disable_ = iris_pwil_dport_disable_i7p;
	pcfg->pw_chip_func_ops.iris_hdr_ai_input_bl_ = NULL;
	pcfg->pw_chip_func_ops.iris_dom_set_ = iris_dom_set_i7p;
	pcfg->pw_chip_func_ops.iris_csc2_para_set_ = iris_csc2_para_set_i7p;
	pcfg->pw_chip_func_ops.iris_pwil_dpp_en_ = iris_pwil_dpp_en_i7p;
	pcfg->pw_chip_func_ops.iris_dpp_en_ = NULL;
	pcfg->pw_chip_func_ops.iris_EDR_backlight_ctrl_i7_ = NULL;
	pcfg->pw_chip_func_ops.iris_frc_prepare_i5_ = NULL;
	pcfg->pw_chip_func_ops.iris_convert_dsi_to_i2c = iris_convert_dsi_to_i2c_i7p;

	iris_memc_func_init();
	iris_memc_helper_setup();
	iris_global_var_init();
	iris_timing_switch_setup();

	return 0;
}

int iris_driver_ops_init_i7p(void)
{
	IRIS_LOGI("%s(), init state: %d", __func__, iris_driver_initialized);
	if (iris_driver_initialized == 0)
		iris_catalog_init_i7p();
	return iris_driver_initialized;
}
