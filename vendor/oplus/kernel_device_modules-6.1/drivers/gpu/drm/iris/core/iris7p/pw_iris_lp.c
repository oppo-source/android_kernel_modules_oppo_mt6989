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
#include "pw_iris_pq.h"
#include "pw_iris_gpio.h"
#include "pw_iris_timing_switch.h"
#include "pw_iris_log.h"
#include "pw_iris_i3c.h"
#include "pw_iris_dts_fw.h"
#include <linux/kobject.h>
#include <linux/delay.h>
#include "pw_iris_reg_i7p.h"



void iris_global_var_init_i7p(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	pcfg->iris_dtg_addr = IRIS_DTG_ADDR;
	pcfg->dtg_ctrl = DTG_CTRL;
	pcfg->dtg_update = DTG_UPDATE;
	pcfg->ovs_dly = DTG_OVS_DLY;

	pcfg->status_reg_addr = STATUS_REG_ADDR_I7P;
	pcfg->id_sys_enter_abyp = ID_SYS_ENTER_ABYP_I7P;
	pcfg->id_sys_exit_abyp = ID_SYS_EXIT_ABYP_I7P;
	pcfg->ulps_mask_value = ULPS_MASK_VALUE_I7P;
	pcfg->id_piad_blend_info = PIAD_BLEND_INFO_OPT_ID_I7P;
	pcfg->te_swap_mask_value = TE_SWAP_MASK_VALUE_I7P;
	pcfg->id_tx_te_flow_ctrl = ID_TX_TE_FLOW_CTRL_I7P;
	pcfg->id_tx_bypass_ctrl = ID_TX_BYPASS_CTRL_I7P;
	pcfg->id_sys_mpg = ID_SYS_MPG_I7P;
	pcfg->id_sys_dpg = ID_SYS_DPG_I7P;
	pcfg->id_sys_ulps = ID_SYS_ULPS_I7P;
	pcfg->id_sys_abyp_ctrl = ID_SYS_ABYP_CTRL_I7P;
	pcfg->id_sys_dma_ctrl = ID_SYS_DMA_CTRL_I7P;
	pcfg->id_sys_dma_gen_ctrl = ID_SYS_DMA_GEN_CTRL_I7P;
	pcfg->id_sys_te_swap = ID_SYS_TE_SWAP_I7P;
	pcfg->id_sys_te_bypass = ID_SYS_TE_BYPASS_I7P;
	pcfg->id_sys_pmu_ctrl = ID_SYS_PMU_CTRL_I7P;
	pcfg->pq_pwr = PQ_PWR_I7P;
	pcfg->frc_pwr = FRC_PWR_I7P;
	pcfg->bsram_pwr = BSRAM_PWR_I7P;
	pcfg->id_rx_dphy = ID_RX_DPHY_I7P;
	pcfg->iris_rd_packet_data = IRIS_RD_PACKET_DATA_I7P;
	pcfg->iris_tx_intstat_raw = IRIS_TX_INTSTAT_RAW_I7P;
	pcfg->iris_tx_intclr = IRIS_TX_INTCLR_I7P;
	pcfg->iris_mipi_tx_header_addr = IRIS_MIPI_TX_HEADER_ADDR_I7P;
	pcfg->iris_mipi_tx_payload_addr = IRIS_MIPI_TX_PAYLOAD_ADDR_I7P;
	pcfg->iris_mipi_tx_header_addr_i3 = IRIS_MIPI_TX_HEADER_ADDR_I3_I7P;
	pcfg->iris_mipi_tx_payload_addr_i3 = IRIS_MIPI_TX_PAYLOAD_ADDR_I3_I7P;
}

/* control dma channels trigger
	input: channels -- bit 0, ch0; bit 1, ch1; bit 2, ch2; bit 3, ch3
	source -- trigger source selection
	chain -- send command with chain or not
*/
static void _iris_dma_gen_ctrl(int channels, int source, bool chain, u8 opt)
{
	int mask, dma_ctrl;
	int value = 0;
	u32 *payload = NULL;
	mask = (BIT(3)|BIT(4));

	payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, opt, 2);
	if (!payload) {
		IRIS_LOGE("%s(), can not get pwil SYS_DMA_GEN_CTRL property in sys setting", __func__);
		return;
	}

	payload[0] = payload[2];

	if (channels & 0x1) {
		value |= (0x20 | source);
		dma_ctrl = payload[2] & mask;
		dma_ctrl |= value;
		payload[0] &= ~BIT(5);
		payload[2] = (payload[2] & ~0x7f) | dma_ctrl;
	}
	if (channels & 0x2) {
		value |= ((0x20 | source) << 7);
		dma_ctrl = payload[2] & (mask << 7);
		dma_ctrl |= value;
		payload[0] &= ~(BIT(5) << 7);
		payload[2] = (payload[2] & ~(0x7f << 7)) | dma_ctrl;
	}
	if (channels & 0x4) {
		value |= ((0x20 | source) << 14);
		dma_ctrl = payload[2] & (mask << 14);
		dma_ctrl |= value;
		payload[0] &= ~(BIT(5) << 14);
		payload[2] = (payload[2] & ~(0x7f << 14)) | dma_ctrl;
	}
	if (channels & 0x8) {
		value |= ((0x20 | source) << 21);
		dma_ctrl = payload[2] & (mask << 21);
		dma_ctrl |= value;
		payload[0] &= ~(BIT(5) << 21);
		payload[2] = (payload[2] & ~(0x7f << 21)) | dma_ctrl;
	}

	iris_set_ipopt_payload_data(IRIS_IP_SYS, opt, 2, payload[0]);

	IRIS_LOGD("%s: payload[0] 0x%x, payload[2] 0x%x",
				__func__, payload[0], payload[2]);

	iris_set_ipopt_payload_data(IRIS_IP_SYS, opt, 4, payload[2]);
	iris_init_update_ipopt_t(IRIS_IP_SYS, opt, opt, chain);
}

void iris_dma_gen_ctrl_i7p(int channels, int source, bool chain)
{
	struct iris_cfg *pcfg;
	int next_channels = (channels >> 4) & 0x0F;

	pcfg = iris_get_cfg();

	if (source > 7) {
		IRIS_LOGE("%s, source %d is wrong!", __func__, source);
		return;
	}

	IRIS_LOGD("%s: channels 0x%x, source %d, chain %d",
		__func__, channels, source, chain);
	if (channels & 0x0F) {
		int wait = (!chain && next_channels) ? 1 : 0;

		_iris_dma_gen_ctrl((channels & 0xF), source, wait, pcfg->id_sys_dma_gen_ctrl);
	}

	if (next_channels)
		_iris_dma_gen_ctrl(next_channels, source, chain, ID_SYS_DMA_GEN_CTRL2);
}

void iris_bulksram_power_domain_proc_i7p(void)
{
	u32 i, value;
	int rc = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	for (i = 0; i < 5; i++) {
		usleep_range(1000, 1010);
		rc = pcfg->iris_i2c_write(0xf0000068, 0x13);
		if (rc < 0) {
			IRIS_LOGE("i2c enable bulksram power failed, rc:%d", rc);
			continue;
		}
		rc = pcfg->iris_i2c_read(0xf0000068, &value);
		if ((rc < 0) || value != 0x13) {
			IRIS_LOGE("i2c read bulksram power failed, value:%d, rc:%d", value, rc);
			continue;
		}
		rc = pcfg->iris_i2c_write(0xf0000068, 0x03);
		if (rc < 0) {
			IRIS_LOGE("i2c disable bulksram power failed, rc:%d", rc);
			continue;
		}
		rc = pcfg->iris_i2c_read(0xf0000068, &value);
		if ((rc < 0) || value != 0x03) {
			IRIS_LOGE("i2c read bulksram power failed, value:%d, rc:%d", value, rc);
			continue;
		}
		break;
	}
	IRIS_LOGI("%s %d", __func__, i);
}

int iris_esd_check_i7p(void)
{
	int rc = 1;
	unsigned int run_status = 0x00;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 value = 0;
	char get_diag_result[1] = {0x0f};
	char rbuf[16] = {0};
	struct iris_cmd_desc cmds = {
		{
			.channel = 0,
			.type = MIPI_DSI_DCS_READ,
			.flags = MIPI_DSI_MSG_REQ_ACK,
			.tx_len = sizeof(get_diag_result),
			.tx_buf = get_diag_result,
			.rx_len = 2,
			.rx_buf = rbuf,
			}, 1, 0};
	struct iris_cmd_set cmdset = {
		.state = IRIS_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &cmds,
	};

	if (pcfg->ocp_read_by_i2c && pcfg->iris_i2c_read) {
		if (pcfg->iris_i2c_read(0xf1640204, &value) < 0) {
			IRIS_LOGE("%s(): iris i2c read fail", __func__);
			rc = -1;
			goto exit;
		}
		IRIS_LOGD("%s(), %d, value = 0x%08x", __func__, __LINE__, value);
		rbuf[0] = (value >> 8) & 0xff;
		rbuf[1] = (value >> 0) & 0xff;
		rc = 0;
	} else
		if (pcfg->lightup_ops.transfer)
			rc = pcfg->lightup_ops.transfer(cmdset.cmds, cmdset.count,
							cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);

	run_status = rbuf[1] & 0x7;
	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
		IRIS_LOGI("dsi read iris esd value: 0x%02x 0x%02x. run_status:0x%x. rc:%d.",
					rbuf[0], rbuf[1], run_status, rc);
	}
	if (rc) {
		IRIS_LOGI("%s dsi read iris esd err: %d", __func__, rc);
		rc = -1;
		goto exit;
	}

	if (iris_esd_ctrl_get() & 0x10) {
		run_status = 0xff;
		pcfg->lp_ctrl.esd_ctrl &= ~0x10;
		IRIS_LOGI("iris esd %s force trigger", __func__);
	}

	if (run_status != 0) {
		IRIS_LOGI("iris esd err status 0x%x. ctrl: %d", run_status, pcfg->lp_ctrl.esd_ctrl);
		iris_dump_status();
		rc = -2;
	} else {
		rc = 1;
	}

exit:
	if (rc < 0) {
		pcfg->lp_ctrl.esd_cnt_iris++;
		IRIS_LOGI("iris esd err cnt: %d. rc %d", pcfg->lp_ctrl.esd_cnt_iris, rc);
	}

	IRIS_LOGD("%s rc:%d", __func__, rc);

	return rc;
}

void iris_pwil_update_i7p(void)
{
	u32 cmd[6];
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe3, 2);
	if (!payload)
		return;
	cmd[0] = IRIS_PWIL_ADDR + EMPTY_FRAME_TIMEOUT_CNT;
	cmd[1] = payload[0];

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe5, 2);
	if (!payload)
		return;
	cmd[2] = IRIS_PWIL_ADDR + BUSY_DOMAIN_DLY;
	cmd[3] = payload[0];

	cmd[4] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
	cmd[5] = 0x00000100;

	iris_ocp_write_mult_vals(6, cmd);
}
