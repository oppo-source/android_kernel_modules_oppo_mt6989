// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <video/mipi_display.h>
#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>
#include "pw_iris_api.h"
#include "pw_iris_lightup.h"
#include "pw_iris_lightup_ocp.h"
#include "pw_iris_lp.h"
#include "pw_iris_timing_switch.h"
#include "pw_iris_pq.h"
#include "pw_iris_gpio.h"
#include "pw_iris_log.h"
#include "pw_iris_i3c.h"
#include "pw_iris_dts_fw.h"
#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include "pw_iris_def.h"

#define DUMP_BUFFER_LENGTH (2560)

static int _debug_lp_opt;
extern uint8_t iris_pq_update_path;

/* abyp debug option
 * bit[0]: 0 -- light up with PT, 1 -- light up with ABYP
 * bit[1]: 0 -- switch to ABYP after light up, 1 -- switch to PT after light up
 * bit[3]: 0 -- non force, 1 -- force abyp during panel switch
 * bit[6]: 0 -- default, 1 -- use dbp instead of abyp
 * bit[7]: 0 -- do nothing, 1 -- disable FORCE_MIPI_PWRDN_EN
 */
static int _debug_on_opt;

bool _need_update_iris_for_qsync_in_pt;
static bool _iris_hdr_power;
static bool _iris_bsram_power; /* BSRAM domain power status */
static bool _iris_frc_power;
static bool _dpg_temp_disable;
static bool _ulps_temp_disable;
static bool _flfp_temp_disable;
int _abyp_mode_config;

#define IRIS_TRACE_FPS       0x01
#define IRIS_TRACE_CADENCE   0X02
#define IRIS_RECORD_FPS	0x04
#define CHECK_KICKOFF_FPS_DURATION 3 /*EVERY 3s*/
#define IRIS_FPS_RECORD_LENGTH 5
#define IRIS_EFIFO_BW_128    0
#define IRIS_EFIFO_BW_256    1

static int mainFpsNextIndex;
static int auxFpsNextIndex;
static int last_kickoff_fps_main[IRIS_FPS_RECORD_LENGTH];
static int last_kickoff_fps_aux[IRIS_FPS_RECORD_LENGTH];
static ktime_t ktime_last_kickoff_main;
static ktime_t ktime_last_kickoff_aux;
static int debug_trace_opt = IRIS_RECORD_FPS;

static u32 _regs_dump_i5[] = {
		0xf0000000, //sys
		0xf0000004,
		0xf0000008,
		0xf0000018,
		0xf000001c,
		0xf0000020,
		0xf0000024,
		0xf0000034,
		0xf0000040,
		0xf0000044,
		0xf0000048,
		0xf000004c,
		0xf0000060,
		0xf0000094,
		0xf00000b0,
		0xf00000b4,
		0xf00000d0,
		0xf00000f4,
		0xf000028c,
		0xf0000290,
		0xf0010000,
		0xf0010004,
		0xf0010008,
		0xf001ffe4,
		0xf1800000, //rx
		0xf1800004,
		0xf180000c,
		0xf1800034,
		0xf1800300,
		0xf1800304,
		0xf1800420,
		0xf1800440,
		0xf1800444,
		0xf1801444,
		0xf1801494,
		0xf120ff08, //DTG
		0xf120ff0c,
		0xf120ff10,
		0xf1220000, //dport
		0xf123ffe4,
		0xf1240000, //pwil
		0xf1240008,
		0xf1240030,
		0xf1241028,
		0xf124102c,
		0xf1241030,
		0xf1241034,
		0xf1241038,
		0xf1241040,
		0xf1241050,
		0xf1250000,
		0xf125ffe4,
		0xf1040028, //sram_ctrl
		0xf105ffe4,
		0xf155ffe4, //blending
		0xf169ffe4, //dsc_dec_main
		0xf1620064, //DSC_ENC_FRC
		0xf1620068,
		0xf162006c,
		0xf1620070,
		0xf1620074,
		0xf163ffe4,
		0xf165ffe4, //dsc_enc_disp
		0xf16a000c, //DSC_DEC_FRC
		0xf16a0010,
		0xf16a0014,
		0xf16a0018,
		0xf16a001c,
		0xf16bffe4,
		0xf16c000c, //DSC_DEC2_FRC
		0xf16c0010,
		0xf16c0014,
		0xf16c0018,
		0xf16c001c,
		0xf16dffe4,
		0xf1a20008, //scaler_input
		0xf1a2000c,
		0xf1a2002c,
		0xf1a20030,
		0xf203ffe4, //FRC_MIF
};

static u32 _regs_dump_i7p[] = {
	0xf0000000, //sys
	0xf0000004,
	0xf0000008,
	0xf0000018,
	0xf0000020,
	0xf0000024,
	0xf0000028,
	0xf0000038,
	0xf0000048,
	0xf000004c,
	0xf0000064,
	0xf0000068,
	0xf000006c,
	0xf00000a4,
	0xf00000d0,
	0xf00000e4,
	0xf00000e8,
	0xf0000120,
	0xf0000144,
	0xf000028c,
	0xf0000290,
	0xf0010004,
	0xf0000210,
	0xf1640000, //mipi_rx
	0xf1640004,
	0xf164000c,
	0xf164002c,
	0xf1640034,
	0xf1640054,
	0xf1640080,
	0xf1640204,
	0xf1640300,
	0xf1640440,
	0xf1640444,
	0xf1640450,
	0xf1640454,
	0xf1641444,
	0xf1641460,
	0xf1641464,
	0xf1641468,
	0xf164146c,
	0xf1641494,
	0xf1600000, //pwil_0
	0xf1600008,
	0xf1600010,
	0xf160015c,
	0xf160007c,
	0xf16000a0,
	0xf1600228,
	0xf1600230,
	0xf1600234,
	0xf1601000,
	0xf161ffe4,
	0xf13dffe4, //input_dsc_dec_pwil0
	0xf1140008, //pbi_status
	0xf121ffe4, //dtg
	0xf1180000, //dport
	0xf1180048,
	0xf119ffe4,
	0xf131ffe4, //dsc_encoder_dport
	0xf1240000, //mipi_tx
	0xf1240020,
	0xf1240070,
	0xf1240074,
	0xf1240078,
	0xf125ffe4,
	0xf15c0014, // scaler1d_input
	0xf15c0038,
	0xf15dffe4,
	0xf1580000, // srcnn
	0xf159ffe4,
	0xf108002c, //sram_ctrl
	0xf109ffe4,
	0xf10dffe4, //mcu_code_ctrl
	0xf12dffe4, //dsc_enc_0
	0xf13400e0, //dsc_dec_0
	0xf13400e4,
	0xf135ffe4,
	0xf13800e0, //dsc_dec_1
	0xf13800e4,
	0xf139ffe4,
	0xf155ffe4, //blending
};

static u32 _regs_dump_i7[] = {
	0xf0000000, //sys
	0xf0000004,
	0xf0000008,
	0xf0000018,
	0xf0000020,
	0xf0000024,
	0xf0000028,
	0xf0000038,
	0xf0000048,
	0xf000004c,
	0xf0000064,
	0xf0000068,
	0xf000006c,
	0xf00000a4,
	0xf00000d0,
	0xf00000e8,
	0xf0000120,
	0xf0000144,
	0xf000028c,
	0xf0000290,
	0xf0010004,
	0xf1a00000, //mipi_rx
	0xf1a00004,
	0xf1a0000c,
	0xf1a0002c,
	0xf1a00034,
	0xf1a00080,
	0xf1a00204,
	0xf1a00300,
	0xf1a00304,
	0xf1a00440,
	0xf1a00444,
	0xf1a00450,
	0xf1a00454,
	0xf1a01444,
	0xf1a01460,
	0xf1a01464,
	0xf1a01468,
	0xf1a0146c,
	0xf1a01494,
	0xf1940000, //pwil_0
	0xf1940008,
	0xf1940010,
	0xf1940014,
	0xf194015c,
	0xf1940228,
	0xf1941000,
	0xf195ffe4,
	0xf1a9ffe4, //input_dsc_dec_pwil0
	0xf1140008, //pbi_status
	0xf135ffe4, //dtg
	0xf12c0000, //dport
	0xf12c0048,
	0xf12dffe4,
	0xf149ffe4, //dsc_encoder_dport
	0xf13c0000, //mipi_tx
	0xf13c0020,
	0xf13c0070,
	0xf13c0074,
	0xf13c0078,
	0xf13dffe4,
	0xf171ffe4, //hdr
	0xf108002c, //sram_ctrl
	0xf109ffe4,
	0xf10dffe4, //mcu_code_ctrl
	0xf1980000, //pwil_1
	0xf1980004,
	0xf1980100,
	0xf199ffe4,
	0xf145ffe4, //dsc_enc_0
	0xf14dffe4, //dsc_dec_0
	0xf151ffe4, //dsc_dec_1
	0xf17dffe4, //blending
	0xf18dffe4, //osd_comp
	0xf191ffe4, //osd_decomp
	0xf1adffe4, //input_dsc_dec_pwil1
	0xf225ffe4, //dsc_encoder_tnr
	0xf1a40000, //mipi_rx1
	0xf1a40004,
	0xf1a40008,
	0xf1a4000c,
	0xf1a40010,
	0xf1a4002c,
	0xf1a40034,
	0xf1a40300,
	0xf1a40304,
	0xf1a40354,
};
static u32 _regs_dump_i8[] = {
	0xf0000000, //sys
	0xf0000004,
	0xf0000008,
	0xf0000040,
	0xf0000044,
	0xf0000048,
	0xf0000088,
	0xf000008c,
	0xf0000090,
	0xf00000c0,
	0xf00000c4,
	0xf00000c8,
	0xf0000100,
	0xf0000144,
	0xf0000148,
	0xf0000184,
	0xf0000188,
	0xf000018c,
	0xf0000190,
	0xf00001c0,
	0xf00001cc,
	0xf00002c0,
	0xf00002c4,
	0xf0010008,
	0xff0c0000, //mipi_rx_main
	0xff0c0004,
	0xff0c000c,
	0xff0c002c,
	0xff0c0034,
	0xff0c0080,
	0xff0c0204,
	0xff0c0300,
	0xff0c0304,
	0xff0c0440,
	0xff0c0444,
	0xff0c0450,
	0xff0c0454,
	0xff0c1444,
	0xff0c1460,
	0xff0c1464,
	0xff0c1468,
	0xff0c146c,
	0xff0c1494,
	0xff120000, //pwil_main
	0xff120008,
	0xff120010,
	0xff12007c,
	0xff1200a0,
	0xff12015c,
	0xff120228,
	0xff120230,
	0xff120234,
	0xff1202a4,
	0xff121000,
	0xff13ffe4,
	0xff49ffe4, //input_decoder_main
	0xff520008, //pbi_status
	0xff09ffe4, //dtg
	0xff4c0000, //dport
	0xff4c0048,
	0xff4dffe4,
	0xff4bffe4, //dport_encoder
	0xff100000, //mipi_tx
	0xff100020,
	0xff100070,
	0xff100074,
	0xff100078,
	0xff11ffe4,
	0xff0a0020, //sram_ctrl
	0xff0bffe4,
	0xff05ffe4, //mcu_code_ctrl
	0xff140000, //pwil_aux
	0xff140004,
	0xff140100,
	0xff15ffe4,
	0xff440014, // input_scaler
	0xff440038,
	0xff45ffe4,
	0xff500000, // srcnn
	0xff51ffe4,
	0xff95ffe4, //dsc_unit_encoder
	0xff9600e0, //dsc_unit_decoder0
	0xff9600e4,
	0xff97ffe4,
	0xff9800e0, //dsc_unit_decoder1
	0xff9800e4,
	0xff99ffe4,
	0xff43ffe4, //blending
	0xff0e0000, //mipi_rx_aux
	0xff0e0004,
	0xff0e0008,
	0xff0e000c,
	0xff0e0010,
	0xff0e002c,
	0xff0e0034,
	0xff0e0300,
	0xff0e0304,
	0xff0e0354,
};
static void _iris_abyp_ctrl_init(bool chain);
static void _iris_flfp_set(bool enable, bool chain);
static int _iris_print_esd_mesg_to_buffer(char *p_buff);

static void _iris_abyp_stop(void);

bool iris_is_pt_mode(bool is_secondary)
{
	if (is_secondary) {
		IRIS_LOGI("%s:%d is secondary panel", __func__, __LINE__);
		return false;
	}

	return iris_get_abyp_mode() == PASS_THROUGH_MODE;
}
EXPORT_SYMBOL(iris_is_pt_mode);

int32_t iris_parse_lp_ctrl(struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u8 vals[3];
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	pcfg->lp_ctrl.esd_ctrl = 1;
	rc = p_dts_ops->read_u32(np, "pxlw,esd-ctrl", &(pcfg->lp_ctrl.esd_ctrl));
	if (rc) {
		IRIS_LOGW("%s, failed to parse pxlw esd-ctrl, return: %d", __func__, rc);
	}
	IRIS_LOGI("%s: pxlw esd-ctrl %#x", __func__, pcfg->lp_ctrl.esd_ctrl);

	pcfg->lp_ctrl.te_swap = p_dts_ops->read_bool(np, "pxlw,te-mask-swaped");
	IRIS_LOGI("%s: pxlw,te-mask-swaped %d", __func__, pcfg->lp_ctrl.te_swap);

	rc = p_dts_ops->read_u8_array(np, "pxlw,low-power", vals, 3);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse low power property, return: %d", __func__, rc);
		return 0;
	}

	pcfg->lp_ctrl.dynamic_power = (bool)vals[0];
	pcfg->lp_ctrl.ulps_lp = (bool)vals[1];
	pcfg->lp_ctrl.abyp_lp = (u8)vals[2];
	_abyp_mode_config = pcfg->lp_ctrl.abyp_lp;
	IRIS_LOGI("%s(), parse low power info: %d %d %d", __func__, vals[0], vals[1], vals[2]);

	return rc;
}

void iris_lp_init(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	pcfg->read_path = PATH_DSI;
#if defined(IRIS_HDK_DEV)
	if (iris_platform_get() == IRIS_FPGA) {// FPGA can work in PT mode only
		pcfg->abyp_ctrl.abypass_mode = PASS_THROUGH_MODE;
		if ((pcfg->iris_chip_type == CHIP_IRIS7P)||(pcfg->iris_chip_type == CHIP_IRIS8))
			_debug_on_opt = 0x10;
	}

	_debug_on_opt |= 0x80; // disable FORCE_MIPI_PWRDN_EN on HDK
#endif
	pcfg->abyp_ctrl.pending_mode = MAX_MODE;
	pcfg->abyp_ctrl.lightup_sys_powerdown = false;
	pcfg->abyp_ctrl.preloaded = false;
	_abyp_mode_config = pcfg->lp_ctrl.abyp_lp;
	mutex_init(&pcfg->abyp_ctrl.abypass_mutex);
	pcfg->iris_pq_disable = 0;
	pcfg->abyp_ctrl.wait_cmdq = false;
	if (pcfg->wait_cmdq_done)
		init_completion(&pcfg->abyp_ctrl.bypass_switch);


	IRIS_LOGI("%s:%d abypass_mode %d", __func__, __LINE__, pcfg->abyp_ctrl.abypass_mode);
}

void iris_lp_enable_pre(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (_debug_on_opt & 0x1)
		pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE;

	pcfg->abyp_ctrl.abyp_failed = false;
	if (pcfg->iris_chip_type == CHIP_IRIS5) {
		if (pcfg->pw_chip_func_ops.iris_lp_preinit_i5_)
			pcfg->pw_chip_func_ops.iris_lp_preinit_i5_();
	}
}

void iris_lp_enable_post(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->rx_mode == IRIS_CMD_MODE && pcfg->tx_mode == IRIS_CMD_MODE)
		pcfg->lp_ctrl.flfp_enable = true;
	else
		pcfg->lp_ctrl.flfp_enable = false;

	iris_init_update_ipopt_t(IRIS_IP_SYS, pcfg->id_sys_dma_ctrl, pcfg->id_sys_dma_ctrl, 1);

	iris_dynamic_power_set(pcfg->lp_ctrl.dynamic_power, true);
	_iris_abyp_ctrl_init(false);

	iris_ulps_enable(pcfg->lp_ctrl.ulps_lp, false);

	IRIS_LOGI("%s dynamic_power %d, ulps_lp %d, abyp_lp %d flfp %d", __func__,
			  pcfg->lp_ctrl.dynamic_power, pcfg->lp_ctrl.ulps_lp,
			  pcfg->lp_ctrl.abyp_lp, pcfg->lp_ctrl.flfp_enable);

}

/*== PMU related APIs ==*/

/* dynamic power gating set */
void iris_dynamic_power_set(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS5) {
		if (pcfg->pw_chip_func_ops.iris_dynamic_power_set_i5_)
			pcfg->pw_chip_func_ops.iris_dynamic_power_set_i5_(enable, chain);
		IRIS_LOGI("%s_i5 %d, chain %d", __func__, enable, chain);
		return;
	}

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = pcfg->id_sys_dpg;
	regval.mask = 0x00000001;
	regval.value = (enable ? 0x1 : 0x0);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(PATH_DSI, true);

	pcfg->lp_ctrl.dynamic_power = enable;
	IRIS_LOGI("%s %d, chain %d", __func__, enable, chain);
}

/* dynamic power gating get */
bool iris_dynamic_power_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	return pcfg->lp_ctrl.dynamic_power;
}

int iris_pmu_power_set(enum iris_pmu_domain domain_id, bool on, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;
	bool is_ulps_enable = 0;
	uint8_t path = iris_pq_update_path;

	pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = pcfg->id_sys_mpg;
	regval.mask = domain_id;
	regval.value = (on ? domain_id : 0x0);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_SYS, regval.opt_id, regval.opt_id, chain);
	if (pcfg->iris_chip_type == CHIP_IRIS5) {
		if (pcfg->pw_chip_func_ops.iris_disable_ulps_)
			is_ulps_enable = pcfg->pw_chip_func_ops.iris_disable_ulps_(path);
	}
	if (!chain)
		iris_update_pq_opt(PATH_DSI, true);
	if (pcfg->iris_chip_type == CHIP_IRIS5) {
		if (pcfg->pw_chip_func_ops.iris_enable_ulps_)
			pcfg->pw_chip_func_ops.iris_enable_ulps_(path, is_ulps_enable);
	}

	return 0;
}

/* power on & off mipi2 domain */
int iris_pmu_mipi2_set(bool on, bool chain)
{
	int rt = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	if (((_debug_lp_opt & 0x1) == 0x1) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	if (pcfg->iris_chip_type == CHIP_IRIS7 ||
	    pcfg->iris_chip_type == CHIP_IRIS8) {
		rt = iris_pmu_power_set(MIPI2_PWR, on, chain);
	} else if (pcfg->iris_chip_type == CHIP_IRIS5) {
			rt = iris_pmu_power_set(MIPI2_PWR_I5, on, chain);
			if (on && pcfg->pw_chip_func_ops.iris_dphy_itf_check_)
				pcfg->pw_chip_func_ops.iris_dphy_itf_check_(true);
	}

	IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	return rt;
}

/* power on & off pq domain */
int iris_pmu_pq_set(bool on)
{
	int rt = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (((_debug_lp_opt & 0x1) == 0x1) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	rt = iris_pmu_power_set(pcfg->pq_pwr, on, 0);
	IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	return rt;
}

/* power on & off bulksram domain */
int iris_pmu_bsram_set(bool on, bool chain)
{
	int rt = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (((_debug_lp_opt & 0x2) == 0x2) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	if (pcfg->iris_chip_type == CHIP_IRIS5) {
		if (pcfg->pw_chip_func_ops.iris_pmu_bsram_set_i5_)
			return pcfg->pw_chip_func_ops.iris_pmu_bsram_set_i5_(on);
	}

	if (on != _iris_bsram_power) {

		if (on)
			rt = iris_pmu_power_set(pcfg->bsram_pwr, on, 0);
		else
			rt = iris_pmu_power_set(pcfg->bsram_pwr, on, chain);

		if (on) {
			udelay(500);
			if (chain)
				iris_init_update_ipopt_t(IRIS_IP_SRAM, 0xa0, 0xa0, 1);
			else
				iris_send_ipopt_cmds(IRIS_IP_SRAM, 0xa0);
		}
	}
	IRIS_LOGI("%s: cur - %d, on - %d, rt - %d", __func__, _iris_bsram_power, on, rt);
	_iris_bsram_power = on;

	return rt;
}
static void _iris_linelock_set(bool enable, bool send)
{
	struct iris_update_regval regval;
	int len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->rx_mode == IRIS_VIDEO_MODE && pcfg->tx_mode == IRIS_VIDEO_MODE) {
		regval.ip = IRIS_IP_DTG;
		regval.opt_id = ID_DTG_TE_SEL;
		regval.mask = 0x04000000;
		regval.value = (enable ? 0x04000000 : 0x00000000);
		iris_update_bitmask_regval_nonread(&regval, false);
		len = iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);
	}
	regval.ip = IRIS_IP_DTG;
	regval.opt_id = 0xF0;
	regval.mask = 0x0000000F;
	regval.value = 0x2;
	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DTG, 0xF0, 0xF0, 0);

	if (send)
		iris_update_pq_opt(PATH_DSI, true);

	IRIS_LOGI("%s: %s, send: %d", __func__, (enable ? "Line Lock enable" : "Line Lock disable"), send);
}

void iris_linelock_set(bool enable, bool chain)
{
	_iris_linelock_set(enable, chain);
}

/* power on & off frc domain */
int iris_pmu_frc_set(bool on, bool chain)
{
	int rt = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (((_debug_lp_opt & 0x4) == 0x4) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	rt = iris_pmu_power_set(pcfg->frc_pwr, on, chain);
	_iris_frc_power = on;
	IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	return rt;
}

bool iris_is_pmu_frc_on(void)
{
	return _iris_frc_power;
}

/* power on & off dsc unit domain */
int iris_pmu_dscu_set(bool on)
{
	int rt = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (((_debug_lp_opt & 0x8) == 0x8) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	if (pcfg->iris_chip_type == CHIP_IRIS7) {
		rt = iris_pmu_power_set(DSCU_PWR, on, 0);
		IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	} else if (pcfg->iris_chip_type == CHIP_IRIS5) {
		rt = iris_pmu_power_set(DSCU_PWR_I5, on, 0);
		IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	}

	return rt;
}

bool iris_is_pmu_dscu_on(void)
{
	uint32_t mask = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	mask = iris_get_regval_bitmask(IRIS_IP_SYS, pcfg->id_sys_mpg);

	IRIS_LOGD("%s: mask: %x", __func__, mask);
	return (mask & DSCU_PWR) == DSCU_PWR;
}

/* control dma channels trigger
	input: channels -- bit 0, ch0; bit 1, ch1; bit 2, ch2; bit 3, ch3
	source -- trigger source selection
	chain -- send command with chain or not
*/

void iris_dma_gen_ctrl(int channels, int source, bool chain)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->pw_chip_func_ops.iris_dma_gen_ctrl_)
		pcfg->pw_chip_func_ops.iris_dma_gen_ctrl_(channels, source, chain);
}
/* control dma trigger
	input: channels -- dma channels. Bit [0] - pq, 1 - hdr, 2 - frc, 3 - dsc_unit, 4 - bulk_sram
		5 - mipi2, 6 - mipi, .. 10 ~ 15: sw channels
	chain -- send command with chain or not
*/
void iris_dma_trig(int channels, bool chain)
{
	iris_set_ipopt_payload_data(IRIS_IP_DMA, 0xe2, 4, channels);
	iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, chain);

	IRIS_LOGD("%s: channels 0x%x, chain %d", __func__, channels, chain);
}

void iris_ulps_enable(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = pcfg->id_sys_ulps;
	regval.mask = pcfg->ulps_mask_value;
	regval.value = enable ? 0x100 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_SYS, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(PATH_DSI, true);

	pcfg->lp_ctrl.ulps_lp = enable;

	IRIS_LOGD("%s %d, chain %d", __func__, enable, chain);
}

void iris_force_mipi_pwrdn_enable(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = pcfg->id_sys_pmu_ctrl;
	regval.mask = 0x80000;
	regval.value = enable ? 0x80000 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_SYS, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(PATH_DSI, true);

	IRIS_LOGD("%s %d, chain %d", __func__, enable, chain);
}

bool iris_ulps_enable_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	IRIS_LOGI("iris:%d", pcfg->lp_ctrl.ulps_lp);

	if (pcfg->lp_ctrl.ulps_lp)
		return true;
	else
		return false;
}

/*== Analog bypass related APIs ==*/

void iris_abyp_mode_set(int mode)
{
	if (mode == 1 || mode == 2) {
		_abyp_mode_config = mode;
		IRIS_LOGI("%s() %s ABP mode", __func__, mode == 1 ? "Standby" : "Sleep");
	} else
		IRIS_LOGW("%s() mode %d is not supported!", __func__, mode);
}

void iris_abyp_mode_get(u32 count, u32 *values)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	values[0] = pcfg->abyp_ctrl.abypass_mode;
	if (count >= 3) {
		values[1] = pcfg->lp_ctrl.abyp_lp << 4;
		values[2] = _abyp_mode_config << 4;
	}
}

bool iris_is_sleep_abyp_mode(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	IRIS_LOGI("%s() abyp_mode: %d, lp_mode: %d", __func__, pcfg->abyp_ctrl.abypass_mode, _abyp_mode_config);

	if ((pcfg->abyp_ctrl.abypass_mode == ANALOG_BYPASS_MODE) && (_abyp_mode_config == 2))
		return true;
	return false;
}

void iris_sleep_abyp_power_down(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	iris_send_one_wired_cmd(IRIS_POWER_DOWN_SYS);

	pcfg->lp_ctrl.abyp_lp = 2;
	_abyp_mode_config = 2;
	pcfg->abyp_ctrl.lightup_sys_powerdown = true;
	pcfg->abyp_ctrl.preloaded = false;
}

static void _iris_abyp_ctrl_init(bool chain)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;

	pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = pcfg->id_sys_abyp_ctrl;
	regval.mask = 0x0CD00000;
	if (pcfg->lp_ctrl.abyp_lp == 1)
		regval.value = 0x00800000;
	else if (pcfg->lp_ctrl.abyp_lp == 2)
		regval.value = 0x00400000;
	else
		regval.value = 0x00000000;

	/* Set digital_bypass_a2i_en/digital_bypass_i2a_en = 1 for video mode */
	if (pcfg->rx_mode == IRIS_VIDEO_MODE) {
		regval.value |= 0x0C100000;
		if (pcfg->lp_ctrl.abyp_lp == 0)
			regval.value |= 0x00100000;
	}

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(PATH_DSI, true);
	IRIS_LOGD("%s() abyp_lp mode %d", __func__, pcfg->lp_ctrl.abyp_lp);
}

/* Switch ABYP by GRCP commands
 * enter_abyp: true -- Enter ABYP, false -- Exit ABYP
*/
static void _iris_send_grcp_abyp(bool enter_abyp)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint8_t vc_id_bak = pcfg->vc_ctrl.to_iris_vc_id;

	if (enter_abyp) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, pcfg->id_sys_enter_abyp);
		IRIS_LOGI("%s, Enter ABYP.", __func__);
	} else {
		if (pcfg->iris_chip_type != CHIP_IRIS5)
			pcfg->vc_ctrl.to_iris_vc_id = pcfg->vc_ctrl.to_panel_hs_vc_id;/*iris7 set vc to 2 when exit abyp by grcp*/
		//first need to power on mipi
		iris_send_ipopt_cmds(IRIS_IP_SYS, pcfg->id_sys_exit_abyp);
		pcfg->vc_ctrl.to_iris_vc_id = vc_id_bak;
		IRIS_LOGI("%s, Exit ABYP.", __func__);
	}
}

/* set Two Wire 0 interface enable */
void iris_set_two_wire0_enable(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint8_t vc_id_bak = pcfg->vc_ctrl.to_iris_vc_id;

	pcfg->vc_ctrl.to_iris_vc_id = 2;/*iris7 set vc to 2 when send grcp cmd*/
	//first need to power on mipi
	iris_send_ipopt_cmds(IRIS_IP_SYS, 0xF5);
	pcfg->vc_ctrl.to_iris_vc_id = vc_id_bak;
	IRIS_LOGI("%s, enable Two Wire 0 interface", __func__);
}

static int _iris_set_max_return_size(void)
{
	int rc = -1;
	struct iris_cfg *pcfg;
	static char max_pktsize[2] = {0x01, 0x00}; /* LSB tx first, 2 bytes */
	static struct iris_cmd_desc pkt_size_cmd = {
		{
			.channel = 0,
			.type = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
			.flags =  MIPI_DSI_MSG_REQ_ACK,
			.tx_len = sizeof(max_pktsize),
			.tx_buf = max_pktsize,
		}, 1, 0};
	struct iris_cmd_set cmdset = {
		.state = IRIS_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &pkt_size_cmd,
	};

	pcfg = iris_get_cfg();

	IRIS_LOGD("%s", __func__);

	if (pcfg->lightup_ops.transfer)
		rc = pcfg->lightup_ops.transfer(cmdset.cmds, cmdset.count,
							cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);
	if (rc)
		IRIS_LOGE("failed to send max return size packet, rc=%d", rc);

	return rc;
}

static int _iris_lp_check_gpio_status(int cnt, int target_status)
{
	int i;
	int abyp_status_gpio;

	if (cnt <= 0) {
		IRIS_LOGE("invalid param, cnt is %d", cnt);
		return -EINVAL;
	}

	IRIS_LOGD("%s, cnt = %d, target_status = %d", __func__, cnt, target_status);

	/* check abyp gpio status */
	for (i = 0; i < cnt; i++) {
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGI("%s, %d, ABYP status: %d.", __func__, i, abyp_status_gpio);
		if (abyp_status_gpio == target_status)
			break;
		usleep_range(3000, 3001);
	}

	return abyp_status_gpio;
}

static void _iris_abyp_stop(void)
{
	int i = 0;
	if (_debug_on_opt & 0x20) {
		while (1) {
			IRIS_LOGI("ABYP switch stop here! cnt: %d", i++);
			msleep(3000);
			if (!(_debug_on_opt & 0x20)) {
				IRIS_LOGI("ABYP switch stop Exit!");
				break;
			}
		}
	} else {
		IRIS_LOGI("ABYP debug option not enable.");
	}
}

void iris_global_var_init(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	pcfg->iris_chip_type = iris_get_chip_type();
	if (pcfg->pw_chip_func_ops.iris_global_var_init_)
		pcfg->pw_chip_func_ops.iris_global_var_init_();
}

static int _iris_abyp_enter_precheck(void)
{
	struct iris_cfg *pcfg;
	u32 value = 0;
	int rc = 0;

	pcfg = iris_get_cfg();

	if (pcfg->iris_i2c_read) {
		if (pcfg->iris_i2c_read(pcfg->status_reg_addr, &value) < 0) {
			IRIS_LOGI("%s(): iris i2c read fail", __func__);
			goto exit;
		}
		IRIS_LOGD("%s(), value = 0x%08x", __func__, value);
		rc = 0;
	}
	if (value & 0x7) {
		iris_dump_status();
		rc = -1;
		IRIS_LOGE("%s(), data path check failed! value = 0x%08x", __func__, value);
	}
exit:
	return rc;
}

int iris_lp_abyp_enter(void)
{
	struct iris_cfg *pcfg;
	int abyp_status_gpio, toler_cnt;
	int rc = 0;
	ktime_t lp_ktime0;
	int check_cnt;

	pcfg = iris_get_cfg();

	lp_ktime0 = ktime_get();
	toler_cnt = RETRY_MAX_CNT;

	if (_iris_abyp_enter_precheck() != 0) {
		pcfg->abyp_ctrl.abyp_failed = true;
		IRIS_LOGE("precheck err, can't enter abyp!\n");
		rc = -1;
		return rc;
	}

	if (pcfg->lp_ctrl.abyp_lp != _abyp_mode_config) {
		pcfg->lp_ctrl.abyp_lp = _abyp_mode_config;
		_iris_abyp_ctrl_init(false);
	}
	IRIS_LOGI("Enter abyp mode %d start", pcfg->lp_ctrl.abyp_lp);
#ifdef IRIS_WA_FOR_IRIS8_A0
	if ((pcfg->iris_chip_type == CHIP_IRIS8) && (pcfg->lp_ctrl.abyp_lp == 1)) {
		if (iris_platform_get() == IRIS_ASIC) {
			iris_send_ipopt_cmds(IRIS_IP_SYS, 0x30);
			iris_send_ipopt_cmds(IRIS_IP_TX, 0x13);
			IRIS_LOGI("PHY PLL WA for iris8 A1 stand by ABYP!");
		}
	}
#endif
	pcfg->abyp_ctrl.wait_cmdq = true;
	_iris_send_grcp_abyp(true);

	if (pcfg->wait_cmdq_done)
		rc = pcfg->wait_cmdq_done();
enter_abyp_begin:
	check_cnt = (pcfg->rx_mode == IRIS_VIDEO_MODE) ? 80 : 50;
	abyp_status_gpio = _iris_lp_check_gpio_status(check_cnt, 1);
	if (abyp_status_gpio == 1)
		pcfg->abyp_ctrl.abypass_mode = IRIS_ABYP_MODE;
	if (_debug_lp_opt & 0x04) {
		// for error process debug
		_debug_lp_opt &= ~0x04;
		abyp_status_gpio = 0;
	}
	if (abyp_status_gpio != 1) {
		iris_dump_status();
		if (toler_cnt > 0) {
			IRIS_LOGW("Enter abyp retry %d", toler_cnt);
			iris_reset_chip();
			pcfg->abyp_ctrl.preloaded = false;
			toler_cnt--;
			goto enter_abyp_begin;
		} else {
			_iris_abyp_stop();
			pcfg->abyp_ctrl.abyp_failed = true;
			IRIS_LOGE("Enter abyp failed!");
			rc = -1;
			return rc;
		}
	}
#ifdef IRIS_WA_FOR_IRIS8_A0
	if (pcfg->iris_chip_type == CHIP_IRIS8) {
		if (pcfg->abyp_ctrl.lightup_sys_powerdown == false) {
			if (pcfg->lp_ctrl.abyp_lp == 2) { //sleep mode
				pcfg->abyp_ctrl.lightup_sys_powerdown = true;
				IRIS_LOGI("power down sys");
			}
		}
	}
#endif
	if (abyp_status_gpio == 1)
		iris_adjust_panel_te_scanline(false);

	IRIS_LOGI("Enter abyp done, spend time %d us",
			(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
#ifdef IRIS_EXT_CLK
	if (pcfg->iris_clk_set)
		pcfg->iris_clk_set(false, false);
#endif
	pcfg->iris_pq_disable = 0;
	pcfg->abyp_ctrl.abyp_failed = false;
	pcfg->abyp_ctrl.wait_cmdq = false;
	return rc;
}

int iris_sys_pll_tx_phy_wa(void)
{
	struct iris_cfg *pcfg;
	int rc = 0;

	pcfg = iris_get_cfg();

	IRIS_LOGI("%s(), enable WA for iris8 A0", __func__);

	rc = pcfg->iris_i2c_write(0xF0000040, 0x0000057C);
	if (rc < 0) {
		IRIS_LOGE("%s(),%d: failed to power up sys pll", __func__, __LINE__);
		return rc;
	}
	rc = pcfg->iris_i2c_write(0xFF102018, 0x05422194);
	if (rc < 0) {
		IRIS_LOGE("%s(),%d: failed to reset mipi tx pll trim", __func__, __LINE__);
		return rc;
	}
	rc = pcfg->iris_i2c_write(0xFF102018, 0x054221a4);
	if (rc < 0) {
		IRIS_LOGE("%s(),%d: failed to enable mipi tx pll trim", __func__, __LINE__);
		return rc;
	}

	return rc;
}

static int _iris7p_lp_abyp_exit_video_pre(void)
{
	struct iris_cfg *pcfg;
	int rc = 0;
	int abp_ctrl = 0x0C907033;
	struct iris_mode_info timing_info;

	pcfg = iris_get_cfg();

	IRIS_LOGI("%s", __func__);

	if (!pcfg || !pcfg->lightup_ops.obtain_cur_timing_info) {
		IRIS_LOGE("Invalid inparms!, %s", __func__);
		return 0;
	}

	if (pcfg->lightup_ops.obtain_cur_timing_info(&timing_info))
		return 0;


	// Update ABP switch parameter
	if (timing_info.refresh_rate == 144)
		abp_ctrl = 0x0C907053; // VDO_HS_LATENCY = 83, LP_WIDTH = 1,HS_WIDTH 2
	else if (timing_info.refresh_rate == 120)//for 120Hz
		abp_ctrl = 0x0C907033; // VDO_HS_LATENCY = 52, LP_WIDTH = 1,HS_WIDTH 2
	else {
		IRIS_LOGE("%s(),%d: FPS(%d) not match(144/120).", __func__, __LINE__, timing_info.refresh_rate);
		return -1;
	}
	if (pcfg->iris_i2c_write) {
		rc = pcfg->iris_i2c_write(0xF0010004, abp_ctrl);
		if (rc < 0) {
			IRIS_LOGE("%s(),%d: failed to update ABP setting.", __func__, __LINE__);
			return rc;
		}
	}
	// Update DTG parameter
	// Todo iris_update_dtg_settings();
	if (pcfg->iris_chip_type == CHIP_IRIS7P) {
		if (pcfg->rx_mode == IRIS_VIDEO_MODE) {
			iris_exit_abyp_update_panel_ap_te(NULL, timing_info.refresh_rate);
		}
	}

	return rc;
}

int iris_lp_abyp_exit(void)
{
	struct iris_cfg *pcfg;
	int abyp_status_gpio;
	int toler_cnt = RETRY_MAX_CNT;
	int rc = 0;
	ktime_t lp_ktime0;

	pcfg = iris_get_cfg();


#ifdef IRIS_EXT_CLK
	if (pcfg->iris_clk_set)
		pcfg->iris_clk_set(true, false);
#endif
	IRIS_LOGI("Exit abyp mode %d start", pcfg->lp_ctrl.abyp_lp);
	lp_ktime0 = ktime_get();

exit_abyp_loop:
	if (pcfg->lp_ctrl.abyp_lp == 2 || !pcfg->abyp_ctrl.preloaded) {
		if (pcfg->lightup_ops.acquire_panel_lock && pcfg->lightup_ops.release_panel_lock
				&& pcfg->wait_pre_framedone) {
			pcfg->lightup_ops.release_panel_lock();
			pcfg->wait_pre_framedone();
			pcfg->lightup_ops.acquire_panel_lock();
		}
	}
	if (pcfg->abyp_ctrl.lightup_sys_powerdown == true) {
		iris_send_one_wired_cmd(IRIS_POWER_UP_SYS);
		IRIS_LOGI("power up sys");
		udelay(2000);
		pcfg->abyp_ctrl.lightup_sys_powerdown = false;
#ifdef IRIS_WA_FOR_IRIS8_A0
		if (pcfg->iris_chip_type == CHIP_IRIS8) {
			bool retry = false;

			rc = iris_sys_pll_tx_phy_wa();
			if (rc < 0)
				retry = true;

			if (retry && toler_cnt > 0) {
				IRIS_LOGW("Exit abyp retry %d", toler_cnt);
				if (toler_cnt <= RETRY_MAX_CNT) {
					iris_reset_chip();
					pcfg->abyp_ctrl.preloaded = false;
					pcfg->abyp_ctrl.lightup_sys_powerdown = true;
				} /* send one wired cmd again*/
				toler_cnt--;
				goto exit_abyp_loop;
			}
		}
#endif
	}

	/*restore default dma setting when switch to PT from ABYP*/
	if (pcfg->iris_chip_type == CHIP_IRIS8 || pcfg->iris_chip_type == CHIP_IRIS7P) {
		iris_set_ipopt_payload_data(IRIS_IP_SYS, pcfg->id_sys_dma_gen_ctrl, 4, pcfg->default_dma_gen_ctrl);
		iris_set_ipopt_payload_data(IRIS_IP_SYS, ID_SYS_DMA_GEN_CTRL2, 4, pcfg->default_dma_gen_ctrl_2);
		IRIS_LOGD("Restore default setting %x  %x",  pcfg->default_dma_gen_ctrl, pcfg->default_dma_gen_ctrl_2);
	}

	if ((pcfg->rx_mode == IRIS_VIDEO_MODE) && (pcfg->iris_chip_type == CHIP_IRIS7P))
		_iris7p_lp_abyp_exit_video_pre();

	/* exit analog bypass */
	iris_send_one_wired_cmd(IRIS_EXIT_ANALOG_BYPASS);
	PW_IRIS_ATRACE_BEGIN("iris_abyp_exit_cmd");
	if (pcfg->rx_mode == IRIS_CMD_MODE
		/* && pcfg_ven->display->panel->ulps_feature_enabled */
		&& iris_platform_get() != IRIS_FPGA) {
		udelay(1000);
		if (pcfg->lp_ctrl.force_exit_ulps_during_switching)
			_iris_set_max_return_size();
	}
	PW_IRIS_ATRACE_END("iris_abyp_exit_cmd");

	abyp_status_gpio = _iris_lp_check_gpio_status(50, 0);
	if (abyp_status_gpio == 0)
		pcfg->abyp_ctrl.abypass_mode = IRIS_PT_MODE;

	if (_debug_lp_opt & 0x04) {
		// for error process debug
		_debug_lp_opt &= ~0x04;
		abyp_status_gpio = 1;
	}

	if (abyp_status_gpio != 0) {
		iris_dump_status();
		if (toler_cnt > 0) {
			IRIS_LOGW("Exit abyp retry %d", toler_cnt);
			if (toler_cnt <= RETRY_MAX_CNT) {
				iris_reset_chip();
				pcfg->abyp_ctrl.preloaded = false;
#ifdef IRIS_WA_FOR_IRIS8_A0
				if (pcfg->iris_chip_type == CHIP_IRIS8)
					pcfg->abyp_ctrl.lightup_sys_powerdown = true;
#endif
			} /* send one wired cmd again*/
			toler_cnt--;
			goto exit_abyp_loop;
		} else {
			IRIS_LOGE("Exit abyp failed!");
			_iris_abyp_stop();
			pcfg->abyp_ctrl.abyp_failed = true;
			rc = -1;
			return rc;
		}

	} else {
		iris_adjust_panel_te_scanline(false);
		if (pcfg->lp_ctrl.abyp_lp == 2 || !pcfg->abyp_ctrl.preloaded) {
			IRIS_LOGI("abyp light up iris");
			if (pcfg->iris_core_lightup)
				pcfg->iris_core_lightup();
			IRIS_LOGI("Light up time %d us",
				(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
		} else {
			iris_switch_from_abyp_to_pt();
		}
	}

	IRIS_LOGI("Exit abyp done, spend time %d us",
		(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));

	if (pcfg->iris_chip_type == CHIP_IRIS7P) {
		if (pcfg->pw_chip_func_ops.iris_pwil_update_i7p_)
			pcfg->pw_chip_func_ops.iris_pwil_update_i7p_();
		if (_debug_on_opt & 0x80)
			iris_force_mipi_pwrdn_enable(false, false);
	}

	pcfg->abyp_ctrl.abyp_failed = false;
	return rc;
}

int iris_dbp_switch(bool enter, bool chain)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;
	int rc = 0;

	if (pcfg->lp_ctrl.dbp_mode == enter) {
		IRIS_LOGW("%s same mode:%d!", __func__, enter);
		return rc;
	}
	IRIS_LOGI("%s, enter: %d, chain: %d", __func__, enter, chain);

	regval.ip = IRIS_IP_TX;
	regval.opt_id = pcfg->id_tx_bypass_ctrl;
	regval.mask = 0x2;
	regval.value = enter ? 0x2 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(PATH_DSI, true);
	pcfg->lp_ctrl.dbp_mode = enter;

	return rc;
}

/* Return: true is PT, false is Bypass */
bool iris_abyp_switch_proc(int mode)
{
	struct iris_cfg *pcfg;
	int rc = 0;
	ktime_t ktime0 = 0;
	int prev_mode = 0;

	pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS5) {
		if (pcfg->pw_chip_func_ops.iris_abypass_switch_proc_)
			return pcfg->pw_chip_func_ops.iris_abypass_switch_proc_(mode, false, true);
	}

	prev_mode = pcfg->abyp_ctrl.abypass_mode;
	if ((mode & BIT(0)) == ANALOG_BYPASS_MODE)
		atomic_set(&pcfg->iris_esd_flag, 0);
	if (pcfg->rx_mode != pcfg->tx_mode) {
		IRIS_LOGE("abyp can't be supported! rx_mode != tx_mode!");
		return 1;
	}

	if ((mode & 0x7F) == pcfg->abyp_ctrl.abypass_mode) {
		IRIS_LOGW("%s same mode:%d!", __func__, mode);
		// return rc;
	}

	if (_debug_on_opt & 0x40) {
		IRIS_LOGW("%s use dbp instead of abyp", __func__);
		if ((mode & BIT(0)) == ANALOG_BYPASS_MODE) {
			rc = iris_dbp_switch(true, false);
			pcfg->abyp_ctrl.abypass_mode = IRIS_ABYP_MODE;
		} else if ((mode & BIT(0)) == PASS_THROUGH_MODE) {
			rc = iris_dbp_switch(false, false);
			pcfg->abyp_ctrl.abypass_mode = IRIS_PT_MODE;
		} else
			IRIS_LOGE("%s: switch mode: %d not supported!", __func__, mode);
		return rc;
	}

	if (_debug_on_opt & 0x1000)
		ktime0 = ktime_get();

	mutex_lock(&pcfg->abyp_ctrl.abypass_mutex);

	// Check GPIO or mipi inside abyp_enter, abyp_exit
	if ((mode & BIT(0)) == ANALOG_BYPASS_MODE) {
		PW_IRIS_ATRACE_BEGIN("iris_abyp_enter");
		rc = iris_lp_abyp_enter();
		PW_IRIS_ATRACE_END("iris_abyp_enter");
	} else if ((mode & BIT(0)) == PASS_THROUGH_MODE) {
		PW_IRIS_ATRACE_BEGIN("iris_abyp_exit");
		rc = iris_lp_abyp_exit();
		PW_IRIS_ATRACE_END("iris_abyp_exit");
		if (rc == 0) {
			if (_need_update_iris_for_qsync_in_pt) {
				if (pcfg->qsync_mode > 0)
					iris_qsync_set(true);
				else
					iris_qsync_set(false);
				IRIS_LOGD("Update iris for QSYNC, qsync_mode: %d", pcfg->qsync_mode);
			}
		}
	} else
		IRIS_LOGE("%s: switch mode: %d not supported!", __func__, mode);
	mutex_unlock(&pcfg->abyp_ctrl.abypass_mutex);

	if (_debug_on_opt & 0x1000) {
		IRIS_LOGI("%s mode: %d -> %d spend time %d", __func__, prev_mode, mode,
			(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime0));
	}

	return rc;
}

int iris_exit_abyp(bool one_wired)
{
	int i = 0;
	int abyp_status_gpio;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS5) {
		if (pcfg->pw_chip_func_ops.iris_dphy_itf_check_)
			pcfg->pw_chip_func_ops.iris_dphy_itf_check_(false);
	}

	/* try to exit abyp */
	if (one_wired) {
		iris_send_one_wired_cmd(IRIS_EXIT_ANALOG_BYPASS);
		udelay(2000);
	} else {
		_iris_send_grcp_abyp(false); /* switch by MIPI command */
		udelay(100);
	}
	IRIS_LOGI("send exit abyp, one_wired:%d.", one_wired);

	/* check abyp gpio status */
	for (i = 0; i < 50; i++) {
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGI("%s, ABYP status: %d.", __func__, abyp_status_gpio);
		if (abyp_status_gpio == 0)
			break;
		udelay(3 * 1000);
	}

	if (abyp_status_gpio == 1) {
		IRIS_LOGE("%s(), failed to exit abyp!", __func__);
		_iris_abyp_stop();
	}

	return abyp_status_gpio;
}

int iris_lightup_opt_get(void)
{
	return _debug_on_opt;
}

void iris_lightup_opt_set(int val)
{
	_debug_on_opt = val;
}

#if defined(CHECK_KICKOFF_FPS_CADNENCE)
static int getCadenceDiff60(long timeDiff)
{
	int cadDiff = 0;

	while (1) {
		if (timeDiff < (((cadDiff + 1) * 100 / 6) - 8))
			break;
		cadDiff++;
		if (cadDiff >= 15)
			break;
	}
	return cadDiff;
}

static int getCadenceDiff90(long timeDiff)
{
	int cadDiff = 0;

	while (1) {
		if (timeDiff < (((cadDiff + 1) * 100 / 9) - 5))
			break;
		cadDiff++;
		if (cadDiff >= 15)
			break;
	}
	return cadDiff;
}

static int getCadenceDiff120(long timeDiff)
{
	int cadDiff = 0;

	while (1) {
		if (timeDiff < (((cadDiff + 1) * 100 / 12) - 3))
			break;
		cadDiff++;
		if (cadDiff >= 15)  // 4-bit
			break;
	}
	return cadDiff;
}

static int getFrameDiff(long timeDiff)
{
	int panel_rate = 60;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_mode_info timing_info;

	if (!pcfg || !pcfg->lightup_ops.obtain_cur_timing_info) {
		IRIS_LOGE("Invalid inparms!, %s", __func__);
		return 0;
	}

	if (pcfg->lightup_ops.obtain_cur_timing_info(&timing_info))
		return 0;

	panel_rate = timing_info.refresh_rate;
	if (panel_rate == 90)
		return getCadenceDiff90(timeDiff);
	else if (panel_rate == 120)
		return getCadenceDiff120(timeDiff);
	else
		return getCadenceDiff60(timeDiff);

}

void iris_dump_ap_kickoff_fps(void)
{
	IRIS_LOGI("iris: last main fps: [%d, %d, %d, %d, %d][nextId: %d][last kick at: %lld ns]",
		last_kickoff_fps_main[0],
		last_kickoff_fps_main[1],
		last_kickoff_fps_main[2],
		last_kickoff_fps_main[3],
		last_kickoff_fps_main[4],
		mainFpsNextIndex, ktime_last_kickoff_main);
	IRIS_LOGI("iris: last aux fps: [%d, %d, %d, %d, %d][nextId: %d][last kick at: %lld ns]",
		last_kickoff_fps_aux[0],
		last_kickoff_fps_aux[1],
		last_kickoff_fps_aux[2],
		last_kickoff_fps_aux[3],
		last_kickoff_fps_aux[4],
		auxFpsNextIndex, ktime_last_kickoff_aux);
}

void iris_check_kickoff_fps_cadence(void)
{
	static u32 kickoff_cnt;
	u32 timeusDelta = 0;
	static ktime_t ktime_kickoff_start;
	static u32 us_last_kickoff;
	ktime_t ktime_kickoff;
	static u32 cadence[10];
	static int cdIndex;
	u32 us_timediff;

	if (!debug_trace_opt)
		return;

	ktime_last_kickoff_main = ktime_get();

	if (kickoff_cnt == 0) {
		kickoff_cnt++;
		ktime_kickoff_start = ktime_get();
		memset(cadence, 0, sizeof(cadence));
		cdIndex = 0;
		cadence[cdIndex++] = 0;
		us_last_kickoff = (u32)ktime_to_us(ktime_kickoff_start);
	} else {
		kickoff_cnt++;
		ktime_kickoff = ktime_get();
		timeusDelta = (u32)ktime_to_us(ktime_kickoff) - (u32)ktime_to_us(ktime_kickoff_start);
		us_timediff = (u32)ktime_to_us(ktime_kickoff) - us_last_kickoff;
		us_last_kickoff = (u32)ktime_to_us(ktime_kickoff);
		if (cdIndex > 9)
			cdIndex = 0;

		cadence[cdIndex++] = getFrameDiff((us_timediff+500)/1000);//16667
		if (timeusDelta >= 1000000*CHECK_KICKOFF_FPS_DURATION) {
			if ((debug_trace_opt&IRIS_RECORD_FPS) == IRIS_RECORD_FPS) {
				int fps = kickoff_cnt/CHECK_KICKOFF_FPS_DURATION;

				if (mainFpsNextIndex >= IRIS_FPS_RECORD_LENGTH)
					mainFpsNextIndex = 0;
				if (mainFpsNextIndex < IRIS_FPS_RECORD_LENGTH)
					last_kickoff_fps_main[mainFpsNextIndex] = fps;
				mainFpsNextIndex ++;
			}
			if ((debug_trace_opt&IRIS_TRACE_FPS) == IRIS_TRACE_FPS) {
				IRIS_LOGI("iris: kickoff fps %d", kickoff_cnt/CHECK_KICKOFF_FPS_DURATION);
				if ((debug_trace_opt&IRIS_RECORD_FPS) == IRIS_RECORD_FPS)
					iris_dump_ap_kickoff_fps();
			}
			if ((debug_trace_opt&IRIS_TRACE_CADENCE) == IRIS_TRACE_CADENCE)
				IRIS_LOGI("iris: Latest cadence: %d %d %d %d %d, %d %d %d %d %d",
						cadence[0], cadence[1], cadence[2], cadence[3], cadence[4],
						cadence[5], cadence[6], cadence[7], cadence[8], cadence[9]);
			kickoff_cnt = 0;
		}
	}
}

void iris_check_kickoff_fps_cadence_2nd(void)
{
	static u32 kickoff_cnt;
	u32 timeusDelta = 0;
	static ktime_t ktime_kickoff_start;
	static u32 us_last_kickoff;
	ktime_t ktime_kickoff;
	static u32 cadence[10];
	static int cdIndex;
	u32 us_timediff;

	if (!debug_trace_opt)
		return;

	ktime_last_kickoff_aux = ktime_get();

	if (kickoff_cnt == 0) {
		kickoff_cnt++;
		ktime_kickoff_start = ktime_get();
		memset(cadence, 0, sizeof(cadence));
		cdIndex = 0;
		cadence[cdIndex++] = 0;
		us_last_kickoff = (u32)ktime_to_us(ktime_kickoff_start);
	} else {
		kickoff_cnt++;
		ktime_kickoff = ktime_get();
		timeusDelta = (u32)ktime_to_us(ktime_kickoff) - (u32)ktime_to_us(ktime_kickoff_start);
		us_timediff = (u32)ktime_to_us(ktime_kickoff) - us_last_kickoff;
		us_last_kickoff = (u32)ktime_to_us(ktime_kickoff);
		if (cdIndex > 9)
			cdIndex = 0;

		cadence[cdIndex++] = getFrameDiff((us_timediff+500)/1000);//16667
		if (timeusDelta >= 1000000*CHECK_KICKOFF_FPS_DURATION) {
			if ((debug_trace_opt&IRIS_RECORD_FPS) == IRIS_RECORD_FPS) {
				int fps = kickoff_cnt/CHECK_KICKOFF_FPS_DURATION;

				if (auxFpsNextIndex >= IRIS_FPS_RECORD_LENGTH)
					auxFpsNextIndex = 0;
				if (auxFpsNextIndex < IRIS_FPS_RECORD_LENGTH)
					last_kickoff_fps_aux[auxFpsNextIndex] = fps;
				auxFpsNextIndex ++;
			}
			if ((debug_trace_opt&IRIS_TRACE_FPS) == IRIS_TRACE_FPS)
				IRIS_LOGI("iris:2nd kickoff fps % d", kickoff_cnt/CHECK_KICKOFF_FPS_DURATION);
			if ((debug_trace_opt&IRIS_TRACE_CADENCE) == IRIS_TRACE_CADENCE)
				IRIS_LOGI("iris:2nd Latest cadence: %d %d %d %d %d, %d %d %d %d %d",
						cadence[0], cadence[1], cadence[2], cadence[3], cadence[4],
						cadence[5], cadence[6], cadence[7], cadence[8], cadence[9]);
			kickoff_cnt = 0;
		}
	}
}
#endif

void iris_set_metadata(bool panel_lock)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t dport_meta = pcfg->metadata & 0x3;
	uint32_t osd_meta = (pcfg->metadata >> 2) & 0x3;
	uint32_t dpp_meta = (pcfg->metadata >> 4) & 0x3;
	uint32_t pwil_meta = (pcfg->metadata >> 6) & 0x3;
	uint32_t framecount_meta = (pcfg->metadata >> 8) & 0xf;
	uint32_t dpp_only_meta = (pcfg->metadata >> 12) & 0x3;

	if (framecount_meta == 0)
		framecount_meta = 2;

	if (pcfg->metadata == 0)
		return;
	if (pcfg->abyp_ctrl.abypass_mode == ANALOG_BYPASS_MODE) {
		pcfg->metadata = 0;
		IRIS_LOGI("clean metadata when iris not initialized");
		return;
	}
	IRIS_LOGD("dport_meta: %x, osd_meta: %x, dpp_meta: %x", dport_meta, osd_meta, dpp_meta);
	pcfg->metadata = 0;

	if (panel_lock && pcfg->lightup_ops.acquire_panel_lock)
		pcfg->lightup_ops.acquire_panel_lock();
	// bit: 0x: nothing, 10: disable dport, 11: enable dport
	switch (dport_meta) {
	case 0x2:
		iris_dom_set(0);
		break;
	case 0x3:
		iris_dom_set(2);
		break;
	default:
		break;
	}
	// bit: 0x: nothing, 10: disable dual, 11: enable dual
	switch (osd_meta) {
	case 0x2:
		iris_blending_enable(false);
		break;
	case 0x3:
		iris_blending_enable(true);
		break;
	default:
		break;
	}

	switch (pwil_meta) {
	case 0x2:
		iris_pwil_dport_disable(false, framecount_meta);
		break;
	case 0x3:
		iris_pwil_dport_disable(true, framecount_meta);
		break;
	default:
		break;
	}

	switch (dpp_meta) {
	case 0x2:
		iris_pwil_dpp_en(false);
		break;
	case 0x3:
		iris_pwil_dpp_en(true);
		break;
	default:
		break;
	}

	switch (dpp_only_meta) {
	case 0x2:
		iris_dpp_en(false);
		break;
	case 0x3:
		iris_dpp_en(true);
		break;
	default:
		break;
	}

	if (panel_lock && pcfg->lightup_ops.release_panel_lock)
		pcfg->lightup_ops.release_panel_lock();
}

int iris_get_abyp_mode(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGD("%s(%d), abyp mode: %d",
			__func__, __LINE__,
			pcfg->abyp_ctrl.abypass_mode);

	return pcfg->abyp_ctrl.abypass_mode;
}

/*== ESD ==*/
int iris_esd_ctrl_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	return pcfg->lp_ctrl.esd_ctrl;
}

static int _iris_esd_check(void)
{
	int rc = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	if (pcfg->pw_chip_func_ops.iris_esd_check_)
		rc = pcfg->pw_chip_func_ops.iris_esd_check_();

	return rc;
}
int iris_revise_return_value(int rc)
{
	int val = rc;
	int mode;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	mode = pcfg->abyp_ctrl.abypass_mode;

	if ((mode == PASS_THROUGH_MODE) && ((iris_esd_ctrl_get() & 0x4) == 0)) {
		val = 1;
		IRIS_LOGI("Disable esd recovery according to esd control\n");
	}
	return val;
}

int iris_status_get(void)
{
	int rc = 0;
	int mode;
	struct iris_cfg *pcfg;
	ktime_t lp_ktime0;

	pcfg = iris_get_cfg();

	// check abyp mode
	mode = pcfg->abyp_ctrl.abypass_mode;

	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
		IRIS_LOGI("esd %s, mode: %d", __func__, mode);
		lp_ktime0 = ktime_get();
	}

	if ((mode != PASS_THROUGH_MODE) && (mode != ANALOG_BYPASS_MODE)) {
		rc = 1;
		goto exit;
	}

	if ((iris_esd_ctrl_get() & 0x1) && (mode == PASS_THROUGH_MODE)) {
		// iris esd check in pt mode
		rc = _iris_esd_check();
		if (rc <= 0)
			goto exit;
	}

	if (iris_esd_ctrl_get() & 0x2)
		rc = 2;
	else
		rc = 1;
#if defined(IRIS_HDK_DEV)
	if ((mode == ANALOG_BYPASS_MODE) && (iris_platform_get() == IRIS_FPGA)) {
		IRIS_LOGD("Ignore esd check in FPGA ABYP mode!");
		rc = 1;
	}
#endif

exit:
	if (rc <= 0) {
		if ((iris_esd_ctrl_get() & 0x4) == 0) {
			if (iris_esd_ctrl_get() & 0x2)
				rc = 2;
			else
				rc = 1;
		} else {
			atomic_set(&pcfg->iris_esd_flag, 1);
			IRIS_LOGI("%s(), %d: set iris_esd_flag = 1", __func__, __LINE__);
		}
	} else {
		if ((iris_esd_ctrl_get() & 0x8) ||
			((iris_esd_ctrl_get() & 0x3) && IRIS_IF_LOGD())) {
			IRIS_LOGI("esd %s done rc: %d, spend time %d us", __func__,
				rc, (u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
			}
	}
	return rc;
}

static uint32_t _iris_get_regs_dump_len(void)
{
	u32 len = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	if (pcfg->iris_chip_type == CHIP_IRIS5)
		len = ARRAY_SIZE(_regs_dump_i5);
	else if (pcfg->iris_chip_type == CHIP_IRIS7)
		len = ARRAY_SIZE(_regs_dump_i7);
	else if (pcfg->iris_chip_type == CHIP_IRIS7P)
		len = ARRAY_SIZE(_regs_dump_i7p);
	else if (pcfg->iris_chip_type == CHIP_IRIS8)
		len = ARRAY_SIZE(_regs_dump_i8);

	return len;
}

int _iris_get_regs_dump_addr(uint32_t idx)
{
	int addr = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	if (pcfg->iris_chip_type == CHIP_IRIS5)
		addr = _regs_dump_i5[idx];
	else if (pcfg->iris_chip_type == CHIP_IRIS7)
		addr = _regs_dump_i7[idx];
	else if (pcfg->iris_chip_type == CHIP_IRIS7P)
		addr = _regs_dump_i7p[idx];
	else if (pcfg->iris_chip_type == CHIP_IRIS8)
		addr = _regs_dump_i8[idx];

	return addr;
}

void iris_dump_status(void)
{
	u32 value, i;
	struct iris_cfg *pcfg;
	u32 len = _iris_get_regs_dump_len();
	uint8_t read_path = PATH_I2C;
	int rc = -1;

	pcfg = iris_get_cfg();

	if (iris_esd_ctrl_get() & 0x20)
		return;
	iris_dump_ap_kickoff_fps();
	IRIS_LOGI("Iris_dump_status:");
	IRIS_LOGI("ESD - ctrl:%d, iris_cnt:%d, panel_cnt:%d",
		pcfg->lp_ctrl.esd_ctrl, pcfg->lp_ctrl.esd_cnt_iris, pcfg->lp_ctrl.esd_cnt_panel);
	IRIS_LOGI("Power - dpg:%d, bsram:%d, frc:%d, hdr:%d",
		pcfg->lp_ctrl.dynamic_power, _iris_bsram_power, _iris_frc_power, _iris_hdr_power);
	IRIS_LOGI("ABYP - mode:%d, lp:%d, gpio:%d, fail:%d",
		pcfg->abyp_ctrl.abypass_mode, pcfg->lp_ctrl.abyp_lp, iris_check_abyp_ready(), pcfg->abyp_ctrl.abyp_failed);
	IRIS_LOGI("ULPS - enabled:%d, lp:%d",
			pcfg->lp_ctrl.force_exit_ulps_during_switching, pcfg->lp_ctrl.ulps_lp);
	IRIS_LOGI("Status - frc:%d, dual:%d, memc_mode:%d",
		pcfg->frc_enabled, pcfg->dual_enabled, pcfg->memc_info.memc_mode);

	IRIS_LOGI("Iris_dump_regs, len: %d", len);
	for (i = 0; i < len; i++) {
		if (read_path == PATH_I2C) {
			if (pcfg->iris_i2c_read) //use i2c to read
				rc = pcfg->iris_i2c_read(_iris_get_regs_dump_addr(i), &value);
			if (rc != 0) {
				IRIS_LOGE("I2C read failed ! [%02d] %08x : %08x", i, _iris_get_regs_dump_addr(i), value);
				break;
			}
		} else
			value = iris_ocp_read(_iris_get_regs_dump_addr(i), IRIS_CMD_SET_STATE_HS);
		IRIS_LOGI("[%02d] %08x : %08x", i, _iris_get_regs_dump_addr(i), value);
	}
}

#if 0
bool iris_check_reg_read(struct dsi_panel *panel)
{
	int i, j = 0;
	int len = 0, *lenp;
	int group = 0, count = 0;
	struct drm_panel_esd_config *config;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (!panel)
		return false;

	if (!(iris_esd_ctrl_get() & 0x8) && !(IRIS_IF_LOGD()))
		return false;

	config = &(panel->esd_config);

	lenp = config->status_valid_params ?: config->status_cmds_rlen;
	count = config->status_cmd.count;

	for (i = 0; i < count; i++)
		len += lenp[i];

	for (j = 0; j < config->groups; ++j) {
		for (i = 0; i < len; ++i) {
			IRIS_LOGI("panel esd [%d] - [%d] : 0x%x", j, i, config->return_buf[i]);

			if (config->return_buf[i] != config->status_value[group + i]) {
				pcfg->lp_ctrl.esd_cnt_panel++;
				IRIS_LOGI("mismatch: 0x%x != 0x%x. Cnt:%d", config->return_buf[i],
					config->status_value[group + i], pcfg->lp_ctrl.esd_cnt_panel);
				break;
			}
		}

		if (i == len)
			return true;
		group += len;
	}

	return false;
}
#endif

/*== Low Power Misc ==*/

static void _iris_tx_buf_to_vc_set(bool enable, bool chain)
{
	struct iris_update_regval regval;
	int len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS5)
		return;

	/* MIPI_TX */
	regval.ip = IRIS_IP_TX;
	regval.opt_id = pcfg->id_tx_bypass_ctrl;
	regval.mask = 0x00000008;
	regval.value = (enable ? 0x00000008 : 0x0);

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(PATH_DSI, true);
	pcfg->vc_ctrl.vc_enable = enable;

	IRIS_LOGI("%s vc %d, chain: %d", __func__, enable, chain);
}

void iris_tx_buf_to_vc_set(bool enable)
{
	_iris_tx_buf_to_vc_set(enable, false);
}

static void _iris_flfp_set(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->lp_ctrl.te_swap) {
		/* SYS */
		regval.ip = IRIS_IP_SYS;
		regval.opt_id = pcfg->id_sys_te_swap;
		regval.mask = pcfg->te_swap_mask_value;
		regval.value = (enable ? regval.mask : 0x0);

		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);
	} else {
		/* SYS */
		regval.ip = IRIS_IP_SYS;
		regval.opt_id = pcfg->id_sys_te_bypass;
		regval.mask = 0x02000000;  //EXT_TE_SEL
		regval.value = (enable ? 0x02000000 : 0x0);

		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);
	}

	if (pcfg->rx_mode == IRIS_CMD_MODE && pcfg->tx_mode == IRIS_CMD_MODE) {
		/* DTG change for CMD mode only
		* TE_DLY need to 0, DTG TE2 using pad_te
		* enable:  TE using mipi_rx
		* disable: TE using pad_te
		*/
		regval.ip = IRIS_IP_DTG;
		regval.opt_id = ID_DTG_TE_SEL;
		regval.mask = 0x000001C0;
		regval.value = (enable ? 0x00000140 : 0x00000040);
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);
		iris_init_update_ipopt_t(IRIS_IP_DTG, 0xF0, 0xF0, 1); /* Use force update */
	}

	/* MIPI_TX */
	regval.ip = IRIS_IP_TX;
	regval.opt_id = pcfg->id_tx_te_flow_ctrl;
	regval.mask = 0x00000003;
	regval.value = (enable ? 0x2 : 0x0); // 0x2: DTG, 0x0: GPIO

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);

	if (!chain)
		iris_update_pq_opt(PATH_DSI, true);

	pcfg->lp_ctrl.flfp_enable = enable;

	IRIS_LOGI("%s %d, chain: %d", __func__, enable, chain);
}

void iris_tx_pb_req_set(bool enable, bool chain)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;

	pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_TX;
	regval.opt_id = pcfg->id_tx_te_flow_ctrl;
	regval.mask = 0x00030000; //PB_FLOW_MODE_NORMAL
	if(pcfg->iris_chip_type == CHIP_IRIS8)
		regval.value = (enable ? 0x30000 : 0x10000);
	else
		regval.value = (enable ? 0x30000 : 0x20000);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);

	if (!chain)
		iris_update_pq_opt(PATH_DSI, true);
}

/* switch low power setting for frc/pt mode
	frc_mode: 0 -- pt mode, 1 -- frc/dual mode
*/
void iris_frc_lp_switch(bool frc_mode, bool chain)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (frc_mode) {
		if (iris_dynamic_power_get()) {
			iris_dynamic_power_set(false, false);
			usleep_range(1000, 1010);
			if (pcfg->iris_chip_type == CHIP_IRIS7) {
				iris_pmu_power_set(pcfg->pq_pwr, false, 0);
				iris_pmu_power_set(pcfg->pq_pwr, true, 0);
				usleep_range(1000, 1010);
			}
			_dpg_temp_disable = true;
		}
		if (pcfg->lp_ctrl.flfp_enable) {
			_iris_flfp_set(false, true);
			_flfp_temp_disable = true;
		}
		if (pcfg->lp_ctrl.ulps_lp) {
			iris_ulps_enable(false, true);
			_ulps_temp_disable = true;
		}
		/* disable virtual channel before memc */
		_iris_tx_buf_to_vc_set(pcfg->vc_ctrl.vc_arr[VC_FRC], chain);
	} else {
		iris_tx_pb_req_set(true, true);
		if (_dpg_temp_disable) {
			if (!iris_dynamic_power_get())
				iris_dynamic_power_set(true, true);
			_dpg_temp_disable = false;
		}
		if (_flfp_temp_disable) {
			if (!pcfg->lp_ctrl.flfp_enable)
				_iris_flfp_set(true, true);
			_flfp_temp_disable = false;
		}
		if (_ulps_temp_disable) {
			if (!pcfg->lp_ctrl.ulps_lp)
				iris_ulps_enable(true, true);
			_ulps_temp_disable = false;
		}
		/* enable vritual chanel after exit frc */
		_iris_tx_buf_to_vc_set(pcfg->vc_ctrl.vc_arr[VC_PT], chain);
	}
	IRIS_LOGD("%s %d chain: %d.", __func__, frc_mode, chain);
}

bool iris_qsync_update_need(void)
{
	return _need_update_iris_for_qsync_in_pt;
}

void iris_qsync_mode_update(u32 qsync_mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->qsync_mode = qsync_mode;
}

void iris_qsync_set(bool enable)
{
	/* Was removed by IOC-83*/
	/*
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;

	if (!iris_is_chip_supported())
		return;

	pcfg = iris_get_cfg();
	if (pcfg->iris_chip_type == CHIP_IRIS5) {
		if (pcfg->pw_chip_func_ops.iris_qsync_set_i5_)
			pcfg->pw_chip_func_ops.iris_qsync_set_i5_(enable);
		return;
	}
	IRIS_LOGD("%s, mode: %d. enable: %d", __func__, pcfg->abyp_ctrl.abypass_mode, enable);

	if (pcfg->abyp_ctrl.abypass_mode == PASS_THROUGH_MODE) {
		IRIS_LOGI("%s, Qsync %d.", __func__, enable);
		regval.ip = IRIS_IP_DTG;
		regval.opt_id = 0xf2; //ivsa filter ctrl
		regval.mask = 0x80000000;
		regval.value = (enable ? 0x0 : 0x80000000);
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);

		regval.ip = IRIS_IP_DTG;
		regval.opt_id = 0xf0; //dtg update
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);

		//pwil te_reset
		//Fixme: WA for two continous frames in one TE
		regval.ip = IRIS_IP_PWIL;
		regval.opt_id = pcfg->id_piad_blend_info;
		regval.mask = 0x00000100;
		regval.value = (enable ? 0x0 : 0x100);
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);

		iris_dma_trig((1<<12), 0);

		iris_update_pq_opt(PATH_DSI, true);
		_need_update_iris_for_qsync_in_pt = false;
	} else {
		_need_update_iris_for_qsync_in_pt = true;
		IRIS_LOGI("Need update iris for Qsync, mode: %d", pcfg->qsync_mode);
	}
	*/
}

void iris_lp_setting_off(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->abyp_ctrl.pending_mode = MAX_MODE;
	pcfg->abyp_ctrl.abyp_failed = false;
	pcfg->abyp_ctrl.preloaded = false;
	_iris_bsram_power = false;
	_iris_frc_power = false;
	iris_pmu_power_set(pcfg->bsram_pwr, 0, 1);
	iris_pmu_power_set(pcfg->frc_pwr, 0, 1);
	if (pcfg->iris_chip_type == CHIP_IRIS7) {
		iris_pmu_power_set(MIPI2_PWR, 0, 1);
		iris_pmu_power_set(DSCU_PWR, 0, 1);
	} else if (pcfg->iris_chip_type == CHIP_IRIS8) {
		iris_pmu_power_set(MIPI2_PWR, 0, 1);
	}

	iris_dbp_switch(false, true);

	iris_frc_lp_switch(false, true);
}

static int _iris_mipi_queue(int value)
{
	int rc = 1;
	struct iris_cfg *pcfg = iris_get_cfg();
	char mipi_queue[2] = {0x0, 0x0};
	struct iris_cmd_desc cmds = {
		{
			.channel = 0,
			.type = MIPI_DSI_EXECUTE_QUEUE,
			.flags = 0,
			.tx_len = sizeof(mipi_queue),
			.tx_buf = mipi_queue,
		}, 1, 0};
	struct iris_cmd_set cmdset = {
		.state = IRIS_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &cmds,
	};

	mipi_queue[0] = value;

	IRIS_LOGI("%s, value: 0x%02x 0x%02x", __func__, mipi_queue[0], mipi_queue[1]);

	if (pcfg->lightup_ops.transfer)
		rc = pcfg->lightup_ops.transfer(cmdset.cmds, cmdset.count,
						cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);

	return rc;
}

static ssize_t _iris_abyp_write(uint32_t val)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	//if (!mutex_trylock(&pcfg_ven->panel->panel_lock))
		//return -EFAULT;
	if (pcfg->lightup_ops.acquire_panel_lock)
		pcfg->lightup_ops.acquire_panel_lock();

	if (val == 0) {
		mutex_lock(&pcfg->abyp_ctrl.abypass_mutex);
		iris_lp_abyp_exit();
		mutex_unlock(&pcfg->abyp_ctrl.abypass_mutex);
	} else if (val == 1) {
		mutex_lock(&pcfg->abyp_ctrl.abypass_mutex);
		iris_lp_abyp_enter();
		mutex_unlock(&pcfg->abyp_ctrl.abypass_mutex);
	} else if (val >= 11 && val <= 19) {
		IRIS_LOGI("%s one wired %d", __func__, (int)(val - 11 + IRIS_POWER_UP_SYS));
		iris_send_one_wired_cmd((int)(val - 11 + IRIS_POWER_UP_SYS));
	} else if (val == 20) {
		_iris_send_grcp_abyp(false);
		IRIS_LOGI("grcp abyp->pt");
	} else if (val == 21) {
		_iris_send_grcp_abyp(true);
		IRIS_LOGI("grcp pt->abyp");
	} else if (val == 23) {
		if (pcfg->iris_core_lightup)
			pcfg->iris_core_lightup();
		IRIS_LOGI("lightup Iris abyp_panel_cmds");
	} else if (val == 24) {
		iris_abyp_switch_proc(PASS_THROUGH_MODE);
	} else if (val == 25) {
		iris_abyp_switch_proc(ANALOG_BYPASS_MODE);
	} else if (val == 26) {
		if (pcfg->iris_chip_type == CHIP_IRIS7)
			iris_set_two_wire0_enable();
	}
	if (pcfg->lightup_ops.release_panel_lock)
		pcfg->lightup_ops.release_panel_lock();

	return 0;
}

static ssize_t _iris_lp_write(uint32_t val)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	//if (!mutex_trylock(&pcfg_ven->panel->panel_lock))
	//	return -EFAULT;
	if (pcfg->lightup_ops.acquire_panel_lock)
		pcfg->lightup_ops.acquire_panel_lock();

	if (val == 0) {
		mutex_lock(&pcfg->abyp_ctrl.abypass_mutex);
		iris_lp_abyp_exit();
		mutex_unlock(&pcfg->abyp_ctrl.abypass_mutex);
	} else if (val == 1) {
		mutex_lock(&pcfg->abyp_ctrl.abypass_mutex);
		iris_lp_abyp_enter();
		mutex_unlock(&pcfg->abyp_ctrl.abypass_mutex);
	} else if (val >= 11 && val <= 19) {
		IRIS_LOGI("%s one wired %d", __func__, (int)(val - 11 + IRIS_POWER_UP_SYS));
		iris_send_one_wired_cmd((int)(val - 11 + IRIS_POWER_UP_SYS));
	} else if (val == 20) {
		_iris_send_grcp_abyp(false);
		IRIS_LOGI("grcp abyp->pt");
	} else if (val == 21) {
		_iris_send_grcp_abyp(true);
		IRIS_LOGI("grcp pt->abyp");
	} else if (val == 23) {
		if (pcfg->iris_core_lightup)
			pcfg->iris_core_lightup();
	}

	if (val == 0) {
		iris_dynamic_power_set(false, false);
		iris_ulps_enable(false, false);
		IRIS_LOGI("disable dynamic & ulps low power.");
	} else if (val == 1) {
		iris_dynamic_power_set(true, false);
		iris_ulps_enable(true, false);
		IRIS_LOGI("enable dynamic & ulps low power.");
	} else if (val == 2) {
		IRIS_LOGI("dynamic power: %d", iris_dynamic_power_get());
		IRIS_LOGI("abyp_lp: %u", pcfg->lp_ctrl.abyp_lp);
		IRIS_LOGI("ulps enable: %d", iris_ulps_enable_get());
	} else if (val == 3) {
		pcfg->lp_ctrl.abyp_lp = 1;
		IRIS_LOGI("set abyp_lp: %d", pcfg->lp_ctrl.abyp_lp);
	} else if (val == 4) {
		pcfg->lp_ctrl.abyp_lp = 2;
		IRIS_LOGI("set abyp_lp: %d", pcfg->lp_ctrl.abyp_lp);
	} else if (val == 11) {
		iris_pmu_mipi2_set(true, false);
	} else if (val == 12) {
		iris_pmu_mipi2_set(false, false);
	} else if (val == 13) {
		iris_pmu_bsram_set(true, false);
	} else if (val == 14) {
		iris_pmu_bsram_set(false, false);
	} else if (val == 15) {
		iris_pmu_frc_set(true, false);
	} else if (val == 16) {
		iris_pmu_frc_set(false, false);
	} else if (val == 17) {
		iris_pmu_dscu_set(true);
	} else if (val == 18) {
		iris_pmu_dscu_set(false);
	} else if (val == 19) {
		_iris_flfp_set(true, false);
	} else if (val == 20) {
		_iris_flfp_set(false, false);
	} else if (val == 21) {
		iris_dma_gen_ctrl(_debug_lp_opt & 0xf, (_debug_lp_opt >> 4) & 0xf, 0);
		iris_update_pq_opt(PATH_DSI, true);
	} else if (val == 22) {
		iris_dma_trig(_debug_lp_opt, 0);
		iris_update_pq_opt(PATH_DSI, true);
	} else if (val == 23) {
		_iris_mipi_queue(1);
	} else if (val == 24) {
		_iris_mipi_queue(2);
	} else if (val == 255) {
		IRIS_LOGI("lp debug usages:");
		IRIS_LOGI("0  -- disable dynamic & ulps low power.");
		IRIS_LOGI("1  -- enable dynamic & ulps low power.");
		IRIS_LOGI("2  -- show low power flag.");
		IRIS_LOGI("3  -- enable abyp.");
		IRIS_LOGI("4  -- disable abyp.");
		IRIS_LOGI("11 -- enable mipi2 power.");
		IRIS_LOGI("12 -- disable mipi2 power.");
		IRIS_LOGI("13 -- enable bsram power.");
		IRIS_LOGI("14 -- disable bram power.");
		IRIS_LOGI("15 -- enable frc power.");
		IRIS_LOGI("16 -- disable frc power.");
		IRIS_LOGI("17 -- enable dsc unit power.");
		IRIS_LOGI("18 -- disable dsc unit power.");
		IRIS_LOGI("19 -- enable flfp.");
		IRIS_LOGI("20 -- disable flfp.");
		IRIS_LOGI("21 -- dma gen ctrl.");
		IRIS_LOGI("22 -- dma trig ctrl.");
		IRIS_LOGI("255 -- show debug usages.");
	}
	if (pcfg->lightup_ops.release_panel_lock)
		pcfg->lightup_ops.release_panel_lock();
	return 0;
}

static ssize_t _iris_esd_write(uint32_t val)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	//if (!mutex_trylock(&pcfg_ven->panel->panel_lock))
	//	return -EFAULT;
	if (pcfg->lightup_ops.acquire_panel_lock)
		pcfg->lightup_ops.acquire_panel_lock();

	if (val <= 128) {
		pcfg->lp_ctrl.esd_ctrl = val;
	} else if (val == 144) {
		pcfg->lp_ctrl.esd_ctrl |= 0x10;
	} else if (val == 200) {
		IRIS_LOGI("clear ESD count!");
		pcfg->lp_ctrl.esd_cnt_iris = 0;
		pcfg->lp_ctrl.esd_cnt_panel = 0;
	}
	if (pcfg->lightup_ops.release_panel_lock)
		pcfg->lightup_ops.release_panel_lock();
	return 0;
}
/*== Low Power debug related ==*/

ssize_t _iris_abyp_sysfs_write(const char *buf, ssize_t count)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint(buf, 0, &val))
		IRIS_LOGW("iris kstrtouint fail, line:%d", __LINE__);

	_iris_abyp_write(val);

	return count;
}
ssize_t _iris_abyp_sysfs_read(char *buf, ssize_t count)
{
	int len = 0;

	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	len = scnprintf(buf, count,
					"abyp status gpio: %d\n", iris_check_abyp_ready());
	len += scnprintf(buf + len, count,
					 "abyp mode: %d\n", pcfg->abyp_ctrl.abypass_mode);
	len += scnprintf(buf + len, count,
					 "abyp lp: %d\n", pcfg->lp_ctrl.abyp_lp);
	return len;
}
ssize_t _iris_lp_sysfs_write(const char *buf, ssize_t count)
{
	uint32_t val;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint(buf, 0, &val))
		IRIS_LOGW("iris kstrtouint fail, line:%d", __LINE__);

	_iris_lp_write(val);

	return count;
}
ssize_t _iris_lp_sysfs_read(char *buf, ssize_t count)
{
	int len = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	IRIS_LOGI("dynamic power: %d", iris_dynamic_power_get());
	IRIS_LOGI("abyp lp: %d", pcfg->lp_ctrl.abyp_lp);
	IRIS_LOGI("ulps enable: %d", iris_ulps_enable_get());

	len = scnprintf(buf, count,
					"dpg: %d\n", iris_dynamic_power_get());
	len += scnprintf(buf + len, count,
					"ulps enable: %d\n", iris_ulps_enable_get());
	len += scnprintf(buf + len, count,
					"abyp lp: %d\n", pcfg->lp_ctrl.abyp_lp);
	return len;
}
ssize_t _iris_esd_sysfs_write(const char *buf, ssize_t count)
{
	uint32_t val;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint(buf, 0, &val))
		IRIS_LOGW("iris kstrtouint fail, line:%d", __LINE__);

	_iris_esd_write(val);
	return count;
}
ssize_t _iris_esd_sysfs_read(char *buf)
{
	int len;

	len = _iris_print_esd_mesg_to_buffer(buf);

	return len;
}

static ssize_t _iris_abyp_dbg_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	char buf[64] = {0};

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, buff, count))
		return -EFAULT;

	if (kstrtoul(buf, 0, &val))
		return -EFAULT;

	_iris_abyp_write(val);

	return count;
}

static ssize_t _iris_abyp_dbg_read(struct file *file, char __user *buff,
								  size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];
	int buf_len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	buf_len = sizeof(bp);

	if (*ppos)
		return 0;

	tot = scnprintf(bp, buf_len,
					"abyp status gpio: %d\n", iris_check_abyp_ready());
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp mode: %d\n", pcfg->abyp_ctrl.abypass_mode);
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp lp: %d\n", pcfg->lp_ctrl.abyp_lp);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static ssize_t _iris_lp_dbg_write(struct file *file,
								 const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	char buf[64] = {0};

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, buff, count))
		return -EFAULT;

	if (kstrtoul(buf, 0, &val))
		return -EFAULT;

	_iris_lp_write(val);

	return count;
}

static ssize_t _iris_lp_dbg_read(struct file *file, char __user *buff,
								 size_t count, loff_t *ppos)
{
	int tot = 0;
	char *bp = NULL;
	int buf_len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	buf_len = DUMP_BUFFER_LENGTH;

	bp = kmalloc(DUMP_BUFFER_LENGTH, GFP_KERNEL);
	if (bp == NULL) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (*ppos)
		return 0;

	IRIS_LOGI("dynamic power: %d", iris_dynamic_power_get());
	IRIS_LOGI("abyp lp: %d", pcfg->lp_ctrl.abyp_lp);
	IRIS_LOGI("ulps enable: %d", iris_ulps_enable_get());

	tot = scnprintf(bp, buf_len,
					"dpg: %d\n", iris_dynamic_power_get());
	tot += scnprintf(bp + tot, buf_len - tot,
					"ulps enable: %d\n", iris_ulps_enable_get());
	tot += scnprintf(bp + tot, buf_len - tot,
					"abyp lp: %d\n", pcfg->lp_ctrl.abyp_lp);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static ssize_t _iris_esd_dbg_write(struct file *file,
								 const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	char buf[64] = {0};

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, buff, count))
		return -EFAULT;

	if (kstrtoul(buf, 0, &val))
		return -EFAULT;

	_iris_esd_write(val);
	return count;
}

static int _iris_print_esd_mesg_to_buffer(char *p_buff)
{
	int tot = 0, rc = 0;
	u32 value, i, len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	tot = scnprintf(p_buff, DUMP_BUFFER_LENGTH,
		"ESD - ctrl:%d, iris_cnt:%d, panel_cnt:%d\n",
		pcfg->lp_ctrl.esd_ctrl, pcfg->lp_ctrl.esd_cnt_iris, pcfg->lp_ctrl.esd_cnt_panel);
	tot += scnprintf(p_buff + tot, DUMP_BUFFER_LENGTH - tot,
		"Power - dpg:%d, bsram:%d, frc:%d, hdr:%d\n",
		pcfg->lp_ctrl.dynamic_power, _iris_bsram_power, _iris_frc_power, _iris_hdr_power);
	tot += scnprintf(p_buff + tot, DUMP_BUFFER_LENGTH - tot,
		"ABYP - mode:%d, lp:%d, gpio:%d, fail:%d\n",
		pcfg->abyp_ctrl.abypass_mode, pcfg->lp_ctrl.abyp_lp, iris_check_abyp_ready(), pcfg->abyp_ctrl.abyp_failed);
	tot += scnprintf(p_buff + tot, DUMP_BUFFER_LENGTH - tot,
		"ULPS - enabled:%d, lp:%d\n",
			pcfg->lp_ctrl.force_exit_ulps_during_switching, pcfg->lp_ctrl.ulps_lp);
	tot += scnprintf(p_buff + tot, DUMP_BUFFER_LENGTH - tot,
		"Status - frc:%d, dual:%d, memc_mode:%d\n",
		pcfg->frc_enabled, pcfg->dual_enabled, pcfg->memc_info.memc_mode);


	len = _iris_get_regs_dump_len();
	tot += scnprintf(p_buff + tot, DUMP_BUFFER_LENGTH - tot,
				"Iris_dump_regs, len: %d\n", len);
	for (i = 0; i < len; i++) {
		rc = pcfg->iris_i2c_read(_iris_get_regs_dump_addr(i), &value);
		if (rc != 0) {
			tot += scnprintf(p_buff + tot, DUMP_BUFFER_LENGTH - tot,
					"I2C read failed ! [%02d] %08x : %08x\n", i, _iris_get_regs_dump_addr(i), value);
			break;
		}
		tot += scnprintf(p_buff + tot, DUMP_BUFFER_LENGTH - tot,
						"[%02d] %08x : %08x\n", i, _iris_get_regs_dump_addr(i), value);
	}
	IRIS_LOGI("data len :%d", tot);


	return tot;

}
static ssize_t _iris_esd_dbg_read(struct file *file, char __user *buff,
								 size_t count, loff_t *ppos)
{
	int rc = 0;
	char *p_buff = NULL;

	p_buff = kmalloc(DUMP_BUFFER_LENGTH, GFP_KERNEL);
	if (p_buff == NULL) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (*ppos) {
		rc = 0;
		goto error;
	}

	rc = _iris_print_esd_mesg_to_buffer(p_buff);

	if (copy_to_user(buff, p_buff, rc)) {
		rc = -EFAULT;
		goto error;
	}
	*ppos += rc;

error:
	kfree(p_buff);
	p_buff = NULL;

	return rc;
}

static ssize_t lp_opt_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	uint32_t val = _debug_lp_opt;

	return scnprintf(buf, SZ_32, "%u\n", val);
}
static ssize_t lp_opt_store(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint(buf, 0, &val))
		IRIS_LOGW("iris kstrtouint fail, line:%d", __LINE__);

	_debug_lp_opt = val;

	return count;
}
static ssize_t abyp_opt_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	uint32_t val = _debug_on_opt;

	return scnprintf(buf, SZ_32, "%u\n", val);
}
static ssize_t abyp_opt_store(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint(buf, 0, &val))
		IRIS_LOGW("iris kstrtouint fail, line:%d", __LINE__);

	_debug_on_opt = val;

	return count;
}
static ssize_t trace_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	uint32_t val = debug_trace_opt;

	return scnprintf(buf, SZ_32, "%u\n", val);
}
static ssize_t trace_store(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint(buf, 0, &val))
		IRIS_LOGW("iris kstrtouint fail, line:%d", __LINE__);

	debug_trace_opt = val;

	return count;
}
static ssize_t vc_enable_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return scnprintf(buf, SZ_32, "%u\n", (u8)pcfg->vc_ctrl.vc_enable);
}
static ssize_t vc_enable_store(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	uint32_t val = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint(buf, 0, &val))
		IRIS_LOGW("iris kstrtouint fail, line:%d", __LINE__);

	pcfg->vc_ctrl.vc_enable = (bool)val;

	return count;
}
static ssize_t abyp_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	return _iris_abyp_sysfs_read(buf, SZ_32);
}
static ssize_t abyp_store(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return _iris_abyp_sysfs_write(buf, count);
}
static ssize_t lp_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	return _iris_lp_sysfs_read(buf, SZ_32);
}
static ssize_t lp_store(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return _iris_lp_sysfs_write(buf, count);
}
static ssize_t esd_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	return _iris_esd_sysfs_read(buf);
}
static ssize_t esd_store(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return _iris_esd_sysfs_write(buf, count);
}
static ssize_t chip_version_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	int n = 0,  i = 0, len = 8 * IRIS_FEATURES_MAX + 1;
	bool use_more_feature = false;
	struct iris_chip_caps chip_capabilities;

	chip_capabilities = iris_get_chip_caps();
	n += scnprintf(buf + n, len - n, "0x");
	for (i = IRIS_FEATURES_MAX - 1; i >= 0; i--) {
		if (chip_capabilities.features[i] != 0) {
			use_more_feature = true;
		}
		if (use_more_feature) {
			n += scnprintf(buf + n, len - n, "%08x", chip_capabilities.features[i]);
		}
	}
	n += scnprintf(buf + n, len - n, "%08x", chip_capabilities.caps);
	IRIS_LOGI("write to sysfs attr: chip_version = %s", buf);
	return n;
}
static ssize_t chip_version_store(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	IRIS_LOGW("Chip version is read only.");
	return -EFAULT;
}

#define IRIS_ATTR(_name, _mode, _show, _store) \
struct kobj_attribute iris_attr_##_name = __ATTR(_name, _mode, _show, _store)
static IRIS_ATTR(lp_opt, 0644, lp_opt_show, lp_opt_store);
static IRIS_ATTR(abyp_opt, 0644, abyp_opt_show, abyp_opt_store);
static IRIS_ATTR(trace, 0644, trace_show, trace_store);
static IRIS_ATTR(vc_enable, 0644, vc_enable_show, vc_enable_store);
static IRIS_ATTR(abyp, 0644, abyp_show, abyp_store);
static IRIS_ATTR(lp, 0644, lp_show, lp_store);
static IRIS_ATTR(esd, 0644, esd_show, esd_store);
static IRIS_ATTR(chip_version, 0644, chip_version_show, chip_version_store);

static struct attribute *iris_dev_attrs[] = {
	&iris_attr_lp_opt.attr,
	&iris_attr_abyp_opt.attr,
	&iris_attr_trace.attr,
	&iris_attr_vc_enable.attr,
	&iris_attr_abyp.attr,
	&iris_attr_lp.attr,
	&iris_attr_esd.attr,
	&iris_attr_chip_version.attr,
	NULL
};
static const struct attribute_group iris_attr_group = {
	.attrs = iris_dev_attrs,
};
void iris_sysfs_status_deinit(void)
{
	kobject_put(iris_get_cfg()->iris_kobj);
}

int pw_iris_dbgfs_lp_init(void *display)
{
	struct iris_cfg *pcfg;
	int retval;

	static const struct file_operations iris_abyp_dbg_fops = {
		.open = simple_open,
		.write = _iris_abyp_dbg_write,
		.read = _iris_abyp_dbg_read,
	};

	static const struct file_operations iris_lp_dbg_fops = {
		.open = simple_open,
		.write = _iris_lp_dbg_write,
		.read = _iris_lp_dbg_read,
	};

	static const struct file_operations iris_esd_dbg_fops = {
		.open = simple_open,
		.write = _iris_esd_dbg_write,
		.read = _iris_esd_dbg_read,
	};

	pcfg = iris_get_cfg();

	pcfg->iris_kobj = kobject_create_and_add(IRIS_SYSFS_TOP_DIR, kernel_kobj);
	if (IS_ERR_OR_NULL(pcfg->iris_kobj)) {
		IRIS_LOGE("sysfs create iris folder error");
	} else {
		retval = sysfs_create_group(pcfg->iris_kobj, &iris_attr_group);
		if (retval) {
			kobject_put(pcfg->iris_kobj);
			IRIS_LOGW("sysfs create group iris fail");
		} else
			IRIS_LOGI("create sysfs group iris");
	}

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("lp_opt", 0644, pcfg->dbg_root,
			(u32 *)&_debug_lp_opt);

	debugfs_create_u32("abyp_opt", 0644, pcfg->dbg_root,
			(u32 *)&_debug_on_opt);

	debugfs_create_u8("vc_enable", 0644, pcfg->dbg_root,
					  (u8 *)&(pcfg->vc_ctrl.vc_enable));
	debugfs_create_u8("rx_mode", 0644, pcfg->dbg_root,
					  (u8 *)&(pcfg->rx_mode));
	debugfs_create_u8("tx_mode", 0644, pcfg->dbg_root,
					  (u8 *)&(pcfg->tx_mode));

	debugfs_create_u32("trace", 0644, pcfg->dbg_root,
			(u32 *)&debug_trace_opt);

	debugfs_create_u32("debug_frc_switch", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->debug_frc_switch);

	if (debugfs_create_file("abyp", 0644, pcfg->dbg_root, display,
				&iris_abyp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("lp", 0644, pcfg->dbg_root, display,
				&iris_lp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("esd", 0644, pcfg->dbg_root, display,
				&iris_esd_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

u32 iris_get_pq_disable_val(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->iris_pq_disable;
}

void iris_set_fps_trace_opt(int trace_opt)
{
	debug_trace_opt = trace_opt;
}

void iris_bulksram_power_domain_proc_i7(void)
{
}

void iris_disable_temp_sensor(void)
{
}


