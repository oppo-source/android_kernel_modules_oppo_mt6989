// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/vmalloc.h>
#include <video/mipi_display.h>
#include <linux/kernel.h>
#include <linux/of.h>
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
#include "mtk_drm_helper.h"

#include "dsi_iris_api.h"
#include "dsi_iris_mtk_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "pw_iris_lp.h"
#include "pw_iris_log.h"
#include "pw_iris_pq.h"
#include "pw_iris_i3c.h"
#include "dsi_iris_cmpt.h"

//#define IRIS_TX_HV_PAYLOAD_LEN   120
//#define IRIS_TX_PAYLOAD_LEN 124
//#define IRIS_RD_PACKET_DATA  0xF13DC018
//#define IRIS_TX_INTSTAT_RAW 0xF13DFFE4
//#define IRIS_TX_READ_RESPONSE_RECEIVED 0x80000000
//#define IRIS_TX_READ_ERR_MASK 0x6FEFFEFF
//#define IRIS_TX_INTCLR 0xF13DFFF0
//#define IRIS_RD_PACKET_DATA_I3  0xF0C1C018
#define IRIS_CMDQ_MAX_PKT_SIZE 3840

//static char iris_read_cmd_rbuf[16];
//static struct iris_ocp_cmd ocp_cmd;
//static struct iris_ocp_cmd ocp_test_cmd[DSI_CMD_CNT];
//static struct iris_cmd_desc iris_test_cmd[DSI_CMD_CNT];
static int g_cmdq_num;
static atomic_t g_cmdq_cnt = ATOMIC_INIT(0);
static struct iris_mtk_dsi_op *gp_mtk_dsi_op;


#define IRIS_DSI_READ_CMD(type)                                         \
	((type == MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM) ||                    \
	 (type == MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM) ||                    \
	 (type == MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM) ||                    \
	 (type == MIPI_DSI_DCS_READ) ||                                        \
	 (type == MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE))

bool iris_is_read_cmd(struct iris_cmd_desc *pdesc)
{
	if (!pdesc)
		return false;

	return IRIS_DSI_READ_CMD(pdesc[0].msg.type);
}

static int _iris_send_rd_cmd(struct iris_cmd_set *pcmdset);
struct iris_mtk_dsi_op *iris_get_mtk_dsi_op(void)
{
	return gp_mtk_dsi_op;
}

void iris_set_mtk_dsi_op(struct iris_mtk_dsi_op *op)
{
	gp_mtk_dsi_op = op;
}

int iris_create_cmdq_handle(struct cmdq_pkt **phandle)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct drm_crtc *crtc = iris_get_vendor_cfg()->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle = NULL;
	struct cmdq_client *client = NULL;
	bool is_frame_mode = mtk_crtc_is_frame_trigger_mode(pcfg_ven->crtc);

	client = (is_frame_mode) ? mtk_crtc->gce_obj.client[CLIENT_CFG] :
			mtk_crtc->gce_obj.client[CLIENT_DSI_CFG];

	if (!(mtk_crtc->enabled)) {
		DDPINFO("%s:%d, crtc is slept\n", __func__,
				__LINE__);
		return -EINVAL;
	}

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	*phandle = cmdq_pkt_create(client);
	cmdq_handle = *phandle;

	return 0;
}
int iris_create_conti_cmdq_handle(struct cmdq_pkt **phandle, bool *is_frame_mode)
{
	struct drm_crtc *crtc = iris_get_vendor_cfg()->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle = NULL;

	if (!(mtk_crtc->enabled)) {
		DDPPR_ERR("%s:%d, crtc is slept\n", __func__,
				__LINE__);
		return -EINVAL;
	}

	*phandle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	cmdq_handle = *phandle;

	return 0;
}

int iris_wait_for_bypass_cmdq_done(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int const timeout = msecs_to_jiffies(500);

	if (pcfg->valid < PARAM_LIGHTUP)
		return 0;

	reinit_completion(&pcfg->abyp_ctrl.bypass_switch);
	if (!wait_for_completion_timeout(&pcfg->abyp_ctrl.bypass_switch, timeout)) {
		IRIS_LOGE("wait for bypass grcp cmdq timeout");
		return -EINVAL;
	}
	return 0;
}

static void iris_dsi_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;
	struct iris_cfg *pcfg = iris_get_cfg();

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	if (pcfg->abyp_ctrl.wait_cmdq &&
		(pcfg->abyp_ctrl.cmdq_num == cb_data->misc)) {
		if (pcfg->wait_cmdq_done)
			complete_all(&pcfg->abyp_ctrl.bypass_switch);
		pcfg->abyp_ctrl.wait_cmdq = false;
	}

	IRIS_LOGI("%s ----- exit async cmdq num:%x", __func__, cb_data->misc);
	kfree(cb_data);

	if (atomic_dec_and_test(&g_cmdq_cnt))
		IRIS_LOGI("%s ----- async cmdq is empty", __func__);
}

bool iris_is_cmdq_empty(void)
{
	return atomic_read(&g_cmdq_cnt) == 0;
}


int iris_destroy_cmdq_handle(struct cmdq_pkt **phandle, bool is_frame_mode)
{
	struct cmdq_pkt *cmdq_handle = *phandle;
	int ret = -EINVAL;
	struct drm_crtc *crtc = iris_get_vendor_cfg()->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (is_frame_mode) {
		priv = mtk_crtc->base.dev->dev_private;
		if (mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_IDLEMGR_ASYNC)) {
			mtk_drm_idle_async_flush(crtc, USER_IRIS_COMMAND, cmdq_handle);
		} else {
			ret = cmdq_pkt_flush(cmdq_handle);
			if (ret < 0)
				DDPPR_ERR("%s:%d, flush error:%d\n", __func__, __LINE__, ret);
			cmdq_pkt_destroy(cmdq_handle);
		}
	} else {
		struct mtk_cmdq_cb_data *cb_data;

		cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
		if (!cb_data) {
			IRIS_LOGE("%s:%d, cb data creation failed\n",
					__func__, __LINE__);
			return ret;
		}
		cb_data->crtc = crtc;
		cb_data->cmdq_handle = cmdq_handle;
		g_cmdq_num = g_cmdq_num < 0xffffffe ? g_cmdq_num : 0;
		cb_data->misc =  g_cmdq_num;
		if (pcfg->abyp_ctrl.wait_cmdq)
			pcfg->abyp_ctrl.cmdq_num = cb_data->misc;
		IRIS_LOGI("%s ++++ enter async cmdq num:%x", __func__, g_cmdq_num);
		if (cmdq_pkt_flush_threaded(cmdq_handle, iris_dsi_cmdq_cb, cb_data) < 0) {
			IRIS_LOGI("failed to flush bl_cmdq_cb\n");
			ret = -EINVAL;
		}
		g_cmdq_num++;
		atomic_inc(&g_cmdq_cnt);
		return 0;

	}

	return 0;
}

void iris_vdo_mode_send_cmd_pre(void *cmdq_handle)
{
	struct cmdq_pkt * handle = (struct cmdq_pkt * )cmdq_handle;
	struct mtk_ddp_comp *comp = iris_get_vendor_cfg()->mtk_comp;
	struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
	/* wait and clear EOF
	 * avoid other display related task break fps change task
	 * because fps change need stop & re-start vdo mode
	 */
	cmdq_pkt_wfe(handle,
				mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
	/* VDO to CMD with LP*/
	mtk_ddp_comp_io_cmd(comp, handle, DSI_STOP_VDO_MODE,
				    NULL);
}

void iris_vdo_mode_send_cmd_post(void *cmdq_handle)
{
	struct cmdq_pkt * handle = (struct cmdq_pkt * )cmdq_handle;
	struct mtk_ddp_comp *comp = iris_get_vendor_cfg()->mtk_comp;
	struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;

	cmdq_pkt_clear_event(handle,
				mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
	mtk_ddp_comp_io_cmd(comp, handle,
				    DSI_START_VDO_MODE, NULL);

	mtk_disp_mutex_trigger(mtk_crtc->mutex[0], handle);
	mtk_ddp_comp_io_cmd(comp, handle, COMP_REG_START,
				NULL);
}

void iris_vdo_mode_send_cmd_with_handle(void *handle,
		void *data, int len, u32 flag, int type)
{
	struct cmdq_pkt *cmdq_handle = handle;
	struct mtk_ddp_comp *comp = iris_get_vendor_cfg()->mtk_comp;

	if (type == 0)
		iris_vdo_mode_send_cmd(cmdq_handle, comp, data, len, flag);
	else if (type == 1)
		iris_get_mtk_dsi_op()->vdo2cmd_cb(cmdq_handle, comp, data, len, flag);
}

void iris_vdo_mode_send_cmd(struct cmdq_pkt *handle,
		struct mtk_ddp_comp *comp,
		void *data, int len, u32 flag)
{
	if (!data || len == 0)
		return;

	iris_vdo_mode_send_cmd_pre(handle);

	iris_get_mtk_dsi_op()->vdo2cmd_cb(handle, comp, data, len, flag);

	iris_vdo_mode_send_cmd_post(handle);
	DDPMSG("%s -\n", __func__);
}

int iris_vdo_mode_send_panel_cmd(struct cmdq_pkt *handle, u8 *data, int len)
{
	u8 arr[512] = {0};
	u32 num = 0;
	u8 *ptr = arr;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_is_chip_supported() || !iris_is_pt_mode(false))
		return -EINVAL;

	if (!data || len == 0)
		return -EINVAL;

	if (pcfg->pwil_mode == RFB_MODE || pcfg->pwil_mode == FRC_MODE) {
		IRIS_LOGI("%s: not send panel cmd in RFB and FRC mode", __func__);
		return 0;
	}
	num = iris_vdo_panel_cmd(ptr, data, len);
	//panel commands need to use HS
	iris_get_mtk_dsi_op()->vdo2cmd_cb(handle, iris_get_vendor_cfg()->mtk_comp, ptr, num, 1);
	return 0;
}

int iris_destroy_conti_cmdq_handle(struct cmdq_pkt **phandle, bool is_frame_mode)
{
	struct drm_crtc *crtc = iris_get_vendor_cfg()->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle = *phandle;
	struct mtk_drm_private *priv = NULL;

	if (is_frame_mode)
		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

	priv = mtk_crtc->base.dev->dev_private;
	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_IDLEMGR_ASYNC)) {
		mtk_drm_idle_async_flush(crtc, USER_IRIS_COMMAND, cmdq_handle);
	} else {
		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}

	return 0;
}


int iris_adjust_cmds_div(struct iris_cmd_set *pset)
{
	int i = 0;
	int sum = 0;
	int split_pkt_size = iris_get_cfg()->split_pkt_size;

	int div_base = IRIS_CMDQ_MAX_PKT_SIZE / (split_pkt_size + 8);

	for (i = 0; i < pset->count; i++)
		sum += pset->cmds[i].msg.tx_len;

	if (split_pkt_size > sum)
		div_base = pset->count;

	return div_base;
}

int iris_cmd_mode_send_cmdq_cmds(struct iris_cmd_set *pset, struct cmdq_pkt *cmdq_handle)
{
	int i = 0;
	int count = pset->count;
	int div_base = 0;
	int div_count = 0;
	int curr = 0;
	struct iris_cmd_desc *pdesc = NULL;
	struct cmdq_pkt *temp_handle = cmdq_handle;
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	bool is_frame_mode = mtk_crtc_is_frame_trigger_mode(pcfg_ven->crtc);


	if (!pset || pset->count == 0) {
		IRIS_LOGE("no commands need to send");
		return -EINVAL;
	}

	div_base = iris_adjust_cmds_div(pset);
	if (!cmdq_handle) {
		DDPINFO("%s:%d pset:0x%p, handle:0x%p, count:%d\n",
				__func__, __LINE__,
				pset, cmdq_handle,
				count);
		iris_send_cmdq_cmds_pre();

		div_count = count > div_base ? div_base : count;
		while(div_count) {
			if (iris_create_cmdq_handle(&temp_handle))
				return -EINVAL;

			for (i = 0; i < div_count; i++) {
				pdesc = pset->cmds + curr + i;
				iris_get_mtk_dsi_op()->transfer(pcfg_ven->mtk_comp,
					temp_handle, (void *)pdesc->msg.tx_buf, pdesc->msg.tx_len,
					pset->state);
			}
			iris_destroy_cmdq_handle(&temp_handle, is_frame_mode);

			count -= div_count;
			curr += div_count;
			div_count = count > div_base ? div_base : count;
		}
		iris_send_cmdq_cmds_post();
		DDPINFO("%s:%d count:%d, curr:%d, div:%d\n",
				__func__, __LINE__,
				pset->count,
				curr, div_count);
	} else {
		DDPINFO("%s:%d pset:0x%p, handle:0x%p, count:%d\n",
				__func__, __LINE__,
				pset, cmdq_handle,
				count);

		for (i = 0; i < count; i++) {
			pdesc = pset->cmds + i;
			iris_get_mtk_dsi_op()->transfer(pcfg_ven->mtk_comp,
					cmdq_handle, (void *)pdesc->msg.tx_buf, pdesc->msg.tx_len, pset->state);
		}
	}
	return 0;
}

int iris_vdo_mode_send_cmdq_cmds(struct iris_cmd_set *pset, struct cmdq_pkt *cmdq_handle)
{
	int i = 0;
	int count = pset->count;
	int div_base = 0;
	int div_count = 0;
	int curr = 0;
	struct iris_cmd_desc *pdesc = NULL;
	struct cmdq_pkt *temp_handle = cmdq_handle;
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	bool is_frame_mode = mtk_crtc_is_frame_trigger_mode(pcfg_ven->crtc);

	if (!pset || pset->count == 0) {
		IRIS_LOGE("no commands need to send");
		return -EINVAL;
	}

	div_base = iris_adjust_cmds_div(pset);
	div_count = count > div_base ? div_base : count;
	while (div_count) {
		if (!cmdq_handle && iris_create_cmdq_handle(&temp_handle))
			return -EINVAL;

		iris_vdo_mode_send_cmd_pre(temp_handle);

		for (i = 0; i < div_count; i++) {
			pdesc = pset->cmds + curr + i;
			iris_get_mtk_dsi_op()->vdo2cmd_cb(temp_handle,
				pcfg_ven->mtk_comp, (void *)pdesc->msg.tx_buf, pdesc->msg.tx_len,
				pset->state);
		}
		iris_vdo_mode_send_cmd_post(temp_handle);

		if (!cmdq_handle)
			iris_destroy_cmdq_handle(&temp_handle, is_frame_mode);

		count -= div_count;
		curr += div_count;
		div_count = count > div_base ? div_base : count;
	}


	IRIS_LOGD("%s:%d count:%d, curr:%d, div:%d\n",
			__func__, __LINE__,
			pset->count,
			curr, div_count);

	return 0;
}

int iris_send_cmdq_cmds(struct iris_cmd_set *pset, struct cmdq_pkt *cmdq_handle)
{
	int ret = -EINVAL;
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	bool is_frame_mode = mtk_crtc_is_frame_trigger_mode(pcfg_ven->crtc);

	if (is_frame_mode)
		ret = iris_cmd_mode_send_cmdq_cmds(pset, cmdq_handle);
	else
		ret = iris_vdo_mode_send_cmdq_cmds(pset, cmdq_handle);
	return ret;
}

static void iris_send_noncmdq_cmds(struct iris_cmd_desc *cmds, u32 cnt,
		enum iris_cmd_set_state state)
{
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	for (i = 0; i < cnt; i++) {
		mipi_dsi_generic_write(pcfg->dsi_dev, cmds[i].msg.tx_buf, cmds[i].msg.tx_len);
	}
}


static int _iris_send_wr_cmd(struct iris_cmd_set *pcmdset, struct cmdq_pkt *handle)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->valid == PARAM_PARSED)
		iris_send_noncmdq_cmds(pcmdset->cmds, pcmdset->count, pcmdset->state);
	else if (pcfg->valid > PARAM_PARSED) {
		if (pcfg->valid == PARAM_PREPARED)
			pcmdset->state = IRIS_CMD_SET_STATE_HS;

		iris_send_cmdq_cmds(pcmdset, handle);
	}

	return 0;
}

static void _iris_dsi_send_cmds(struct iris_cmd_desc *cmds, u32 cnt,
		enum iris_cmd_set_state state,
		struct cmdq_pkt *cmdq_handle)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cmd_set cmdset = {
			.state = state,
			.count = cnt,
			.cmds = cmds,
	};

	if (state == IRIS_CMD_SET_STATE_LP)
		pcfg->dsi_dev->mode_flags |= MIPI_DSI_MODE_LPM;
	else
		pcfg->dsi_dev->mode_flags &= ~(MIPI_DSI_MODE_LPM);

	if (IRIS_DSI_READ_CMD(cmds->msg.type))
		_iris_send_rd_cmd(&cmdset);
	else
		_iris_send_wr_cmd(&cmdset, cmdq_handle);
}

u8 iris_get_cmd_type(u8 cmd, u32 count)
{
	u8 dtype = 0;

	if (cmd < 0xB0) {
		if (count > 1)
			dtype = MIPI_DSI_DCS_LONG_WRITE;
		else if (count == 1)
			dtype = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		else
			dtype = MIPI_DSI_DCS_SHORT_WRITE;
	} else {
		if (count > 1)
			dtype = MIPI_DSI_GENERIC_LONG_WRITE;
		else if (count == 1)
			dtype = MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM;
		else
			dtype = MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM;
	}
	return dtype;
}


void iris_cmd_desc_para_fill(struct iris_cmd_desc *dsi_cmd)
{
	u32 count = dsi_cmd->msg.tx_len;
	u8 cmd = *((u8 *)dsi_cmd->msg.tx_buf);

	dsi_cmd->post_wait_ms = 0;
	dsi_cmd->last_command = 1;

	if (IRIS_DSI_READ_CMD(dsi_cmd->msg.type)) {
		return;
	}

	dsi_cmd->msg.type = iris_get_cmd_type(cmd, count-1);
}
#if 0
static void _iris_add_cmd_addr_val(
		struct iris_ocp_cmd *pcmd, u32 addr, u32 val)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(addr);
	*(u32 *)(pcmd->cmd + pcmd->cmd_len + 4) = cpu_to_le32(val);
	pcmd->cmd_len += 8;
}

static void _iris_add_cmd_payload(struct iris_ocp_cmd *pcmd, u32 payload)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(payload);
	pcmd->cmd_len += 4;
}

void iris_ocp_write_val(u32 address, u32 value)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0} };

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));

	_iris_add_cmd_payload(&ocp_cmd, 0xFFFFFFF0 | OCP_SINGLE_WRITE_BYTEMASK);
	_iris_add_cmd_addr_val(&ocp_cmd, address, value);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	IRIS_LOGD("%s(), addr: %#x, value: %#x", __func__, address, value);

	pcfg->lightup_ops.transfer(iris_ocp_cmd, 1, IRIS_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);
}

void iris_ocp_write_vals(u32 header, u32 address, u32 size, u32 *pvalues, enum iris_cmd_set_state state)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cmd_desc iris_ocp_cmd[] = {
		{{
			.channel = 0,
			.type = MIPI_DSI_GENERIC_LONG_WRITE,
			.flags = 0,
			.tx_len = CMD_PKT_SIZE,
			.tx_buf = ocp_cmd.cmd,
			}, 1, 0} };

	u32 max_size = CMD_PKT_SIZE / 4 - 2;
	u32 i;

	while (size > 0) {
		memset(&ocp_cmd, 0, sizeof(ocp_cmd));

		_iris_add_cmd_payload(&ocp_cmd, header);
		_iris_add_cmd_payload(&ocp_cmd, address);
		if (size < max_size) {
			for (i = 0; i < size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			size = 0;
		} else {
			for (i = 0; i < max_size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			address += max_size * 4;
			pvalues += max_size;
			size -= max_size;
		}
		iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;
		IRIS_LOGD("%s(), header: %#x, addr: %#x, len: %zu", __func__,
				header, address, iris_ocp_cmd[0].msg.tx_len);

		pcfg->lightup_ops.transfer(iris_ocp_cmd, 1, state, pcfg->vc_ctrl.to_iris_vc_id);
	}
}

/*pvalues need to be one address and one value*/
static void _iris_dsi_write_mult_vals(u32 size, u32 *pvalues)
{
	int i,j;
	int cmds_count;
	struct iris_ocp_cmd *p_ocp_cmd = NULL;
	struct iris_cmd_desc *p_iris_ocp_cmd = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 split_pkt_size = pcfg->split_pkt_size;
	/*need to remove one header length*/
	u32 max_size = ((split_pkt_size - 4) >> 2); /*(244 -4)/4*/
	u32 header = 0xFFFFFFF4;

	if (size % 2 != 0) {
		IRIS_LOGE("%s(), need to be mult pair of address and value", __func__);
		return;
	}
	cmds_count = (size + max_size -1) / max_size;
	p_ocp_cmd = kmalloc_array(cmds_count, sizeof(struct iris_ocp_cmd), GFP_KERNEL);
	if (p_ocp_cmd == NULL) {
		IRIS_LOGE("%s(), failed to alloc memory for p_ocp_cmd", __func__);
		return;
	}
	p_iris_ocp_cmd = kmalloc_array(cmds_count, sizeof(struct iris_cmd_desc), GFP_KERNEL);
	if (p_iris_ocp_cmd == NULL) {
		IRIS_LOGE("%s(), failed to alloc memory for p_iris_ocp_cmd", __func__);
		kfree(p_ocp_cmd);
		return;
	}

	for(i=0; i<cmds_count; i++) {
		memset(&p_ocp_cmd[i], 0, sizeof(struct iris_ocp_cmd));
		memset(&p_iris_ocp_cmd[i], 0, sizeof(struct iris_cmd_desc));

		_iris_add_cmd_payload(&p_ocp_cmd[i], header);
		if (size < max_size) {
			for (j = 0; j < size; j++)
				_iris_add_cmd_payload(&p_ocp_cmd[i], pvalues[j]);

			size = 0;
		} else {
			for (j = 0; j < max_size; j++)
				_iris_add_cmd_payload(&p_ocp_cmd[i], pvalues[j]);

			pvalues += max_size;
			size -= max_size;
		}
		p_iris_ocp_cmd[i].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
		p_iris_ocp_cmd[i].msg.tx_len = p_ocp_cmd[i].cmd_len;
		p_iris_ocp_cmd[i].msg.tx_buf = p_ocp_cmd[i].cmd;
		p_iris_ocp_cmd[i].last_command = 1;	// no used
		IRIS_LOGD("%s(), header: 0x%08x, len: %zu",
				__func__,
				header, p_iris_ocp_cmd[i].msg.tx_len);
	}
	pcfg->lightup_ops.transfer(p_iris_ocp_cmd,
		cmds_count, IRIS_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);
	kfree(p_ocp_cmd);
	kfree(p_iris_ocp_cmd);
}

/*pvalues need to be one address and one value*/
void iris_ocp_write_mult_vals(u32 size, u32 *pvalues)
{
	IRIS_LOGD("%s(%d), path select dsi", __func__, __LINE__);
	_iris_dsi_write_mult_vals(size, pvalues);
}

static void _iris_ocp_write_addr(u32 address, u32 mode)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0} };

	/* Send OCP command.*/
	memset(&ocp_cmd, 0, sizeof(ocp_cmd));

	_iris_add_cmd_payload(&ocp_cmd, OCP_SINGLE_READ);
	_iris_add_cmd_payload(&ocp_cmd, address);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	pcfg->lightup_ops.transfer(iris_ocp_cmd, 1, mode, pcfg->vc_ctrl.to_iris_vc_id);
}
#endif

int iris_send_cmdq_rdcmd(struct iris_cmd_set *pset)
{
	bool is_frame_mode;
	struct cmdq_pkt *cmdq_handle = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct mipi_dsi_device *dsi = pcfg->dsi_dev;
	struct iris_cmd_desc  *cmd = pset->cmds;
	struct mtk_ddp_comp *comp = pcfg_ven->mtk_comp;
	unsigned int slot_index = DISP_SLOT_IRIS_READ_BASE;

	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.tx_len = cmd->msg.tx_len,
		.tx_buf = cmd->msg.tx_buf,
		.rx_len = cmd->msg.rx_len,
		.rx_buf = cmd->msg.rx_buf,
	};

	is_frame_mode = mtk_crtc_is_frame_trigger_mode(pcfg_ven->crtc);

	if (iris_create_cmdq_handle(&cmdq_handle))
		return -EINVAL;
	//transfer rd cmd
	iris_get_mtk_dsi_op()->transfer_rdcmd(comp, cmdq_handle, &msg, slot_index);
	iris_destroy_cmdq_handle(&cmdq_handle, is_frame_mode);

	iris_get_mtk_dsi_op()->obtain_rdvalue(comp, &msg, slot_index);

	return 0;
}

int iris_send_noncmdq_rdcmd(struct iris_cmd_set *pcmdset)
{
	struct iris_cmd_desc *cmd = pcmdset->cmds;
	struct iris_cfg *pcfg = iris_get_cfg();
	u8 *tx_buf = (u8 *)cmd->msg.tx_buf;
	u32 tx_len = cmd->msg.tx_len;
	u8 *rx_buf = (u8 *)cmd->msg.rx_buf;
	u32 rx_len = cmd->msg.rx_len;

	if (cmd->msg.type == MIPI_DSI_DCS_READ)
		mipi_dsi_dcs_read(pcfg->dsi_dev, tx_buf[0], rx_buf, rx_len);
	else
		mipi_dsi_generic_read(pcfg->dsi_dev, tx_buf, tx_len, rx_buf, rx_len);
	return 0;
}

static int _iris_send_rd_cmd(struct iris_cmd_set *pcmdset)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcmdset || pcmdset->count > 1)
		return -EINVAL;

	if (pcfg->valid == PARAM_PARSED)
		iris_send_noncmdq_rdcmd(pcmdset);
	else if (pcfg->valid > PARAM_PARSED)
		iris_send_cmdq_rdcmd(pcmdset);

	return 0;
}

#if 0
static u32 _iris_ocp_read_value(u32 mode)
{
	u32 response_value = 0;
	char pi_read[1] = {0xC0};
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cmd_desc pi_read_cmd[] = {
		{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_REQ_ACK,
			sizeof(pi_read), pi_read,
			sizeof(iris_read_cmd_rbuf),
			iris_read_cmd_rbuf}, 1, 0}};


	//iris_set_msg_flags(pi_read_cmd, READ_FLAG);
	/* Read response.*/
	memset(iris_read_cmd_rbuf, 0, sizeof(iris_read_cmd_rbuf));

	if (pcfg->lightup_ops.transfer)
		pcfg->lightup_ops.transfer(pi_read_cmd, 1, mode, pcfg->vc_ctrl.to_iris_vc_id);

	IRIS_LOGD("%s(), read register: 0x%02x 0x%02x 0x%02x 0x%02x", __func__,
			iris_read_cmd_rbuf[0], iris_read_cmd_rbuf[1],
			iris_read_cmd_rbuf[2], iris_read_cmd_rbuf[3]);

	response_value = iris_read_cmd_rbuf[0] | (iris_read_cmd_rbuf[1] << 8)
				| (iris_read_cmd_rbuf[2] << 16) | (iris_read_cmd_rbuf[3] << 24);

	return response_value;
}

static u32 _iris_dsi_ocp_read(u32 address, u32 mode)
{
	u32 value = 0;

	_iris_ocp_write_addr(address, mode);

	value = _iris_ocp_read_value(mode);
	IRIS_LOGD("%s(), addr: %#x, value: %#x", __func__, address, value);

	return value;
}

u32 iris_ocp_read(u32 address, u32 mode)
{
	u32 value = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_ATRACE_BEGIN(__func__);
	if (pcfg->ocp_read_by_i2c && pcfg->iris_i2c_read) {
		IRIS_LOGD("%s(%d), path select i2c", __func__, __LINE__);
		if (pcfg->iris_i2c_read(address, &value) < 0)
			IRIS_LOGE("i2c read reg fails, reg addr=0x%x", address);
		IRIS_ATRACE_END(__func__);
		return value;
	}
	IRIS_LOGD("%s(%d), path select dsi", __func__, __LINE__);
	value = _iris_dsi_ocp_read(address, mode);
	IRIS_ATRACE_END(__func__);

	return value;
}

static void _iris_dump_packet(u8 *data, int size)
{
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4, data, size, false);
}

void iris_write_test(u32 iris_addr,
		int ocp_type, u32 pkt_size)
{
	union iris_ocp_cmd_header ocp_header;
	struct iris_cmd_desc iris_cmd = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0,
			CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0};
	u32 test_value = 0xFFFF0000;
	struct iris_cfg *pcfg = iris_get_cfg();

	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));
	memcpy(ocp_cmd.cmd, &ocp_header.header32, OCP_HEADER);
	ocp_cmd.cmd_len = OCP_HEADER;

	switch (ocp_type) {
	case OCP_SINGLE_WRITE_BYTEMASK:
	case OCP_SINGLE_WRITE_BITMASK:
		for (; ocp_cmd.cmd_len <= (pkt_size - 8); ) {
			_iris_add_cmd_addr_val(&ocp_cmd, iris_addr, test_value);
			test_value++;
		}
		break;
	case OCP_BURST_WRITE:
		test_value = 0xFFFF0000;
		_iris_add_cmd_addr_val(&ocp_cmd, iris_addr, test_value);
		if (pkt_size <= ocp_cmd.cmd_len)
			break;
		test_value++;
		for (; ocp_cmd.cmd_len <= pkt_size - 4;) {
			_iris_add_cmd_payload(&ocp_cmd, test_value);
			test_value++;
		}
		break;
	default:
		break;
	}

	IRIS_LOGI("%s(), len: %d, iris addr: %#x, test value: %#x",
			__func__,
			ocp_cmd.cmd_len, iris_addr, test_value);
	iris_cmd.msg.tx_len = ocp_cmd.cmd_len;
	pcfg->lightup_ops.transfer(&iris_cmd, 1, IRIS_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);

	if (IRIS_IF_LOGD())
		_iris_dump_packet(ocp_cmd.cmd, ocp_cmd.cmd_len);
}

void iris_write_test_muti_pkt(struct iris_ocp_dsi_tool_input *ocp_input)
{
	union iris_ocp_cmd_header ocp_header;
	u32 test_value = 0xFF000000;
	int cnt = 0;

	u32 iris_addr, ocp_type, pkt_size, total_cnt;
	struct iris_cfg *pcfg = iris_get_cfg();

	ocp_type = ocp_input->iris_ocp_type;
	test_value = ocp_input->iris_ocp_value;
	iris_addr = ocp_input->iris_ocp_addr;
	total_cnt = ocp_input->iris_ocp_cnt;
	pkt_size = ocp_input->iris_ocp_size;

	memset(iris_test_cmd, 0, sizeof(iris_test_cmd));
	memset(ocp_test_cmd, 0, sizeof(ocp_test_cmd));

	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	switch (ocp_type) {
	case OCP_SINGLE_WRITE_BYTEMASK:
	case OCP_SINGLE_WRITE_BITMASK:
		for (cnt = 0; cnt < total_cnt; cnt++) {
			memcpy(ocp_test_cmd[cnt].cmd,
					&ocp_header.header32, OCP_HEADER);
			ocp_test_cmd[cnt].cmd_len = OCP_HEADER;

			test_value = 0xFF000000 | (cnt << 16);
			while (ocp_test_cmd[cnt].cmd_len <= (pkt_size - 8)) {
				_iris_add_cmd_addr_val(&ocp_test_cmd[cnt],
						(iris_addr + cnt * 4), test_value);
				test_value++;
			}

			iris_test_cmd[cnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
			iris_test_cmd[cnt].msg.tx_len = ocp_test_cmd[cnt].cmd_len;
			iris_test_cmd[cnt].msg.tx_buf = ocp_test_cmd[cnt].cmd;
		}
		iris_test_cmd[total_cnt - 1].last_command = true;
		break;
	case OCP_BURST_WRITE:
		for (cnt = 0; cnt < total_cnt; cnt++) {
			memcpy(ocp_test_cmd[cnt].cmd,
					&ocp_header.header32, OCP_HEADER);
			ocp_test_cmd[cnt].cmd_len = OCP_HEADER;
			test_value = 0xFF000000 | (cnt << 16);

			_iris_add_cmd_addr_val(&ocp_test_cmd[cnt],
					(iris_addr + cnt * 4), test_value);
			/* if(pkt_size <= ocp_test_cmd[cnt].cmd_len)
			 * break;
			 */
			test_value++;
			while (ocp_test_cmd[cnt].cmd_len <= pkt_size - 4) {
				_iris_add_cmd_payload(&ocp_test_cmd[cnt], test_value);
				test_value++;
			}

			iris_test_cmd[cnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
			iris_test_cmd[cnt].msg.tx_len = ocp_test_cmd[cnt].cmd_len;
			iris_test_cmd[cnt].msg.tx_buf = ocp_test_cmd[cnt].cmd;
		}
		iris_test_cmd[total_cnt - 1].last_command = true;
		break;
	default:
		break;
	}

	IRIS_LOGI("%s(), total cnt: %#x iris addr: %#x test value: %#x",
		__func__, total_cnt, iris_addr, test_value);

	pcfg->lightup_ops.transfer(iris_test_cmd, total_cnt, IRIS_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);

	if (IRIS_IF_NOT_LOGV())
		return;

	for (cnt = 0; cnt < total_cnt; cnt++)
		_iris_dump_packet(ocp_test_cmd[cnt].cmd,
				ocp_test_cmd[cnt].cmd_len);
}
#endif

int iris_dsi_send_cmds(struct iris_cmd_desc *cmds,
		u32 count, enum iris_cmd_set_state state, u8 vc_id)
{
	_iris_dsi_send_cmds(cmds, count, state, NULL);

	return 0;
}
#if 0
static u32 _iris_pt_get_split_pkt_cnt(int dlen)
{
	u32 sum = 1;

	if (dlen > IRIS_TX_HV_PAYLOAD_LEN)
		sum = (dlen - IRIS_TX_HV_PAYLOAD_LEN
				+ IRIS_TX_PAYLOAD_LEN - 1) / IRIS_TX_PAYLOAD_LEN + 1;
	return sum;
}

/*
 * @Description: use to do statitics for cmds which should not less than 252
 *      if the payload is out of 252, it will change to more than one cmds
 * the first payload need to be
 *	4 (ocp_header) + 8 (tx_addr_header + tx_val_header)
 *	+ 2* payload_len (TX_payloadaddr + payload_len)<= 252
 * the sequence payloader need to be
 *	4 (ocp_header) + 2* payload_len (TX_payloadaddr + payload_len)<= 252
 *	so the first payload should be no more than 120
 *	the second and sequence need to be no more than 124
 *
 * @Param: cmdset  cmds request
 * @return: the cmds number need to split
 **/
static u32 _iris_pt_calc_cmd_cnt(struct iris_cmd_set *cmdset)
{
	u32 i = 0;
	u32 sum = 0;
	u32 dlen = 0;

	for (i = 0; i < cmdset->count; i++) {
		dlen = cmdset->cmds[i].msg.tx_len;
		sum += _iris_pt_get_split_pkt_cnt(dlen);
	}

	return sum;
}

static int _iris_pt_alloc_cmds(
		struct iris_cmd_set *cmdset,
		struct iris_cmd_desc **ptx_cmds,
		struct iris_ocp_cmd **pocp_cmds)
{
	int cmds_cnt = _iris_pt_calc_cmd_cnt(cmdset);

	IRIS_LOGD("%s(%d), cmds cnt: %d malloc len: %lu",
			__func__, __LINE__,
			cmds_cnt, cmds_cnt * sizeof(**ptx_cmds));
	*ptx_cmds = kvmalloc(cmds_cnt * sizeof(**ptx_cmds), GFP_KERNEL);
	if (!(*ptx_cmds)) {
		IRIS_LOGE("%s(), failed to malloc buf, len: %lu",
				__func__,
				cmds_cnt * sizeof(**ptx_cmds));
		return -ENOMEM;
	}

	*pocp_cmds = kvmalloc(cmds_cnt * sizeof(**pocp_cmds), GFP_KERNEL);
	if (!(*pocp_cmds)) {
		IRIS_LOGE("%s(), failed to malloc buf for pocp cmds", __func__);
		kvfree(*ptx_cmds);
		*ptx_cmds = NULL;
		return -ENOMEM;
	}
	return cmds_cnt;
}

static void _iris_pt_init_tx_cmd_hdr(
		struct iris_cmd_set *cmdset, struct iris_cmd_desc *dsi_cmd,
		union iris_mipi_tx_cmd_header *header)
{
	u8 dtype = dsi_cmd->msg.type;

	memset(header, 0x00, sizeof(*header));
	header->stHdr.dtype = dtype;
	header->stHdr.linkState = (cmdset->state == IRIS_CMD_SET_STATE_LP) ? 1 : 0;
}

static void _iris_pt_set_cmd_hdr(
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd, bool is_write)
{
	u32 dlen = 0;
	u8 *ptr = NULL;

	if (!dsi_cmd)
		return;

	dlen = dsi_cmd->msg.tx_len;

	if (is_write)
		pheader->stHdr.writeFlag = 0x01;
	else
		pheader->stHdr.writeFlag = 0x00;

	if (pheader->stHdr.longCmdFlag == 0) {
		ptr = (u8 *)dsi_cmd->msg.tx_buf;
		if (dlen == 1) {
			pheader->stHdr.len[0] = ptr[0];
		} else if (dlen == 2) {
			pheader->stHdr.len[0] = ptr[0];
			pheader->stHdr.len[1] = ptr[1];
		}
	} else {
		pheader->stHdr.len[0] = dlen & 0xff;
		pheader->stHdr.len[1] = (dlen >> 8) & 0xff;
	}
}

static void _iris_pt_set_wrcmd_hdr(
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	_iris_pt_set_cmd_hdr(pheader, dsi_cmd, true);
}

static void _iris_pt_set_rdcmd_hdr(
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	_iris_pt_set_cmd_hdr(pheader, dsi_cmd, false);
}

static void _iris_pt_init_ocp_cmd(struct iris_ocp_cmd *pocp_cmd)
{
	union iris_ocp_cmd_header ocp_header;

	if (!pocp_cmd) {
		IRIS_LOGE("%s(), invalid pocp cmd!", __func__);
		return;
	}

	memset(pocp_cmd, 0x00, sizeof(*pocp_cmd));
	ocp_header.header32 = 0xfffffff0 | OCP_SINGLE_WRITE_BYTEMASK;
	memcpy(pocp_cmd->cmd, &ocp_header.header32, OCP_HEADER);
	pocp_cmd->cmd_len = OCP_HEADER;
}

static void _iris_add_tx_cmds(
		struct iris_cmd_desc *ptx_cmd,
		struct iris_ocp_cmd *pocp_cmd, u8 wait)
{
	struct iris_cmd_desc desc_init_val = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0,
			CMD_PKT_SIZE, NULL, 0, NULL}, 1, 0};

	memcpy(ptx_cmd, &desc_init_val, sizeof(struct iris_cmd_desc));
	ptx_cmd->msg.tx_buf = pocp_cmd->cmd;
	ptx_cmd->msg.tx_len = pocp_cmd->cmd_len;
	ptx_cmd->post_wait_ms = wait;
}

static u32 _iris_pt_short_write(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	//struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = IRIS_MIPI_TX_HEADER_ADDR;

	pheader->stHdr.longCmdFlag = 0x00;

	_iris_pt_set_wrcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: 0x%4x",
			__func__, __LINE__,
			pheader->hdr32);
	_iris_add_cmd_addr_val(pocp_cmd, address, pheader->hdr32);

	return sum;
}

static u32 _iris_pt_short_read(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	//struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = IRIS_MIPI_TX_HEADER_ADDR ;

	pheader->stHdr.longCmdFlag = 0x00;
	_iris_pt_set_rdcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: 0x%4x",
			__func__, __LINE__,
			pheader->hdr32);
	_iris_add_cmd_addr_val(pocp_cmd, address, pheader->hdr32);

	return sum;
}

static u32 _iris_pt_get_split_pkt_len(u16 dlen, int sum, int k)
{
	u16 split_len = 0;

	if (k == 0)
		split_len = dlen <  IRIS_TX_HV_PAYLOAD_LEN
			? dlen : IRIS_TX_HV_PAYLOAD_LEN;
	else if (k == sum - 1)
		split_len = dlen - IRIS_TX_HV_PAYLOAD_LEN
			- (k - 1) * IRIS_TX_PAYLOAD_LEN;
	else
		split_len = IRIS_TX_PAYLOAD_LEN;

	return split_len;
}

static void _iris_pt_add_split_pkt_payload(
		struct iris_ocp_cmd *pocp_cmd, u8 *ptr, u16 split_len)
{
	u32 i = 0;
	union iris_mipi_tx_cmd_payload payload;
	//struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = IRIS_MIPI_TX_PAYLOAD_ADDR;

	memset(&payload, 0x00, sizeof(payload));
	for (i = 0; i < split_len; i += 4, ptr += 4) {
		if (i + 4 > split_len) {
			payload.pld32 = 0;
			memcpy(payload.p, ptr, split_len - i);
		} else
			payload.pld32 = *(u32 *)ptr;

		IRIS_LOGD("%s(), payload: %#x", __func__, payload.pld32);
		_iris_add_cmd_addr_val(pocp_cmd, address,
				payload.pld32);
	}
}

static u32 _iris_pt_long_write(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	u8 *ptr = NULL;
	u32 i = 0;
	u32 sum = 0;
	u16 dlen = 0;
	u32 split_len = 0;
	//struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = IRIS_MIPI_TX_HEADER_ADDR;

	dlen = dsi_cmd->msg.tx_len;

	pheader->stHdr.longCmdFlag = 0x1;
	_iris_pt_set_wrcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: %#x",
			__func__, __LINE__,
			pheader->hdr32);
	_iris_add_cmd_addr_val(pocp_cmd, address,
			pheader->hdr32);

	ptr = (u8 *)dsi_cmd->msg.tx_buf;
	sum = _iris_pt_get_split_pkt_cnt(dlen);

	while (i < sum) {
		ptr += split_len;
		split_len = _iris_pt_get_split_pkt_len(dlen, sum, i);
		_iris_pt_add_split_pkt_payload(pocp_cmd + i, ptr, split_len);

		i++;
		if (i < sum)
			_iris_pt_init_ocp_cmd(pocp_cmd + i);
	}
	return sum;
}

static u32 _iris_pt_add_cmd(
		struct iris_cmd_desc *ptx_cmd, struct iris_ocp_cmd *pocp_cmd,
		struct iris_cmd_desc *dsi_cmd, struct iris_cmd_set *cmdset)
{
	u32 i = 0;
	u16 dtype = 0;
	u32 sum = 0;
	u8 wait = 0;
	union iris_mipi_tx_cmd_header header;

	_iris_pt_init_tx_cmd_hdr(cmdset, dsi_cmd, &header);

	dtype = dsi_cmd->msg.type;
	switch (dtype) {
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		sum = _iris_pt_short_read(pocp_cmd, &header, dsi_cmd);
		break;
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		sum = _iris_pt_short_write(pocp_cmd, &header, dsi_cmd);
		break;
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
		sum = _iris_pt_long_write(pocp_cmd, &header, dsi_cmd);
		break;
	default:
		IRIS_LOGE("%s(), invalid type: %#x",
				__func__,
				dsi_cmd->msg.type);
		break;
	}

	for (i = 0; i < sum; i++) {
		wait = (i == sum - 1) ? dsi_cmd->post_wait_ms : 0;
		_iris_add_tx_cmds(ptx_cmd + i, pocp_cmd + i, wait);
	}
	return sum;
}

static int _iris_pt_send_cmds(struct iris_cmd_desc *ptx_cmds, u32 cmds_cnt)
{
	struct iris_cmd_set panel_cmds;
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;
	u8 vc_id = 0;

	memset(&panel_cmds, 0x00, sizeof(panel_cmds));

	panel_cmds.cmds = ptx_cmds;
	panel_cmds.count = cmds_cnt;
	panel_cmds.state = IRIS_CMD_SET_STATE_HS;

	if (pcfg->vc_ctrl.vc_enable) {
		vc_id = (panel_cmds.state == IRIS_CMD_SET_STATE_LP) ?
			pcfg->vc_ctrl.to_panel_lp_vc_id :
			pcfg->vc_ctrl.to_panel_hs_vc_id;
	}
	rc = pcfg->lightup_ops.transfer(panel_cmds.cmds,
			panel_cmds.count, panel_cmds.state, vc_id);

	if (iris_get_cont_splash_type() == IRIS_CONT_SPLASH_LK)
		iris_print_desc_cmds(panel_cmds.cmds, panel_cmds.count, panel_cmds.state);
	return rc;
}

static int __iris_pt_write_panel_cmd(struct iris_cmd_set *cmdset,
		struct iris_ocp_cmd **ret_pocp_cmds,
		struct iris_cmd_desc **ret_ptx_cmds)
{
	u32 i = 0;
	u32 j = 0;
	int cmds_cnt = 0;
	u32 offset = 0;
	int rc = 0;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct iris_cmd_desc *ptx_cmds = NULL;
	struct iris_cmd_desc *dsi_cmds = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!cmdset) {
		IRIS_LOGE("%s(), invalid cmdset!", __func__);
		return -EINVAL;
	}

	if (cmdset->count == 0) {
		IRIS_LOGD("%s(), invalid cmdset count!", __func__);
		return -EINVAL;
	}

	cmds_cnt = _iris_pt_alloc_cmds(cmdset, &ptx_cmds, &pocp_cmds);
	if (cmds_cnt < 0) {
		IRIS_LOGE("%s(), invalid cmds count: %d", __func__, cmds_cnt);
		return -EINVAL;
	}

	for (i = 0; i < cmdset->count; i++) {
		/*initial val*/
		dsi_cmds = cmdset->cmds + i;

		if (pcfg->platform_ops.fill_desc_para)
			pcfg->platform_ops.fill_desc_para(dsi_cmds);

		_iris_pt_init_ocp_cmd(pocp_cmds + j);
		offset = _iris_pt_add_cmd(
				ptx_cmds + j, pocp_cmds + j, dsi_cmds, cmdset);
		j += offset;
	}

	if (j != (u32)cmds_cnt) {
		IRIS_LOGE("%s(), invalid cmd count: %d, j: %d",
				__func__,
				cmds_cnt, j);
		rc = -ENOMEM;
	} else
		rc = cmds_cnt;

	*ret_pocp_cmds = pocp_cmds;
	*ret_ptx_cmds = ptx_cmds;

	return rc;
}


int iris_conver_one_panel_cmd(u8 *dest, u8 *src, int len)
{
	int ret = 0;
	struct iris_cmd_set cmdset;
	struct iris_cmd_desc desc;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct iris_cmd_desc *ptx_cmds = NULL;

	if (!src || !dest) {
		IRIS_LOGE("src or dest is error");
		return -EINVAL;
	}

	//abypass mode
	if (!iris_is_pt_mode()) {
		memcpy(dest, src, len);
		return len;
	}

	cmdset.cmds = &desc;
	cmdset.count = 1;
	desc.msg.tx_buf = src;
	desc.msg.tx_len = len;

	ret = __iris_pt_write_panel_cmd(&cmdset, &pocp_cmds, &ptx_cmds);
	if (ret == -EINVAL)
		return ret;
	else if (ret == 1) {
		memcpy(dest, ptx_cmds->msg.tx_buf, ptx_cmds->msg.tx_len);
		ret = ptx_cmds->msg.tx_len;
	} else
		ret = -EINVAL;

	kvfree(pocp_cmds);
	kvfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;

	return ret;
}
EXPORT_SYMBOL(iris_conver_one_panel_cmd);
static int  _iris_pt_write_panel_cmd(struct iris_cmd_set *cmdset)
{
	int ret = 0;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct iris_cmd_desc *ptx_cmds = NULL;

	if (!cmdset || !cmdset->count) {
		IRIS_LOGE("cmdset is error!");
		return ret;
	}

	ret = __iris_pt_write_panel_cmd(cmdset, &pocp_cmds, &ptx_cmds);
	if (ret == -EINVAL)
		return ret;
	else if (ret > 0) {
		ret = _iris_pt_send_cmds(ptx_cmds, (u32)ret);
	}

	kvfree(pocp_cmds);
	kvfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;
	return ret;
}
#endif

int iris_send_ddic_cmd(struct cmdq_pkt *handle,
		struct mtk_drm_crtc *crtc,
		struct mtk_ddic_dsi_msg *cmd_msg)
{
	int i = 0;
	struct iris_cmd_desc desc;
	struct iris_cmd_set cmdset;
	struct mipi_dsi_msg msg;

	if (!handle || !crtc || !cmd_msg) {
		IRIS_LOGE("%s invalid param!", __func__);
		return -EINVAL;
	}
	/* Check cmd_msg param */
	if (cmd_msg->tx_cmd_num == 0 ||
		cmd_msg->tx_cmd_num > MAX_TX_CMD_NUM) {
		IRIS_LOGE("%s: type is %s, tx_cmd_num is %d\n",
			__func__, cmd_msg->type, (int)cmd_msg->tx_cmd_num);
		return -EINVAL;
	}

	for (i = 0; i < cmd_msg->tx_cmd_num; i++) {
		if (cmd_msg->tx_buf[i] == 0 || cmd_msg->tx_len[i] == 0) {
			IRIS_LOGE("%s: tx_buf[%d] is %s, tx_len[%d] is %d\n",
				__func__, i, (char *)cmd_msg->tx_buf[i], i,
				(int)cmd_msg->tx_len[i]);
			return -EINVAL;
		}
	}


	msg.channel = cmd_msg->channel;
	if (cmd_msg->flags & MIPI_DSI_MSG_USE_LPM)
		cmdset.state = IRIS_CMD_SET_STATE_LP;

	for (i = 0; i < cmd_msg->tx_cmd_num; i++) {
		msg.type = cmd_msg->type[i];
		msg.tx_len = cmd_msg->tx_len[i];
		msg.tx_buf = cmd_msg->tx_buf[i];
		desc.msg = msg;
		desc.last_command = 1;
		cmdset.cmds = &desc;
		cmdset.count = 1;

		_iris_pt_write_panel_cmd(&cmdset);
	}
	return 0;
}

#if 0
static void _iris_pt_switch_cmd(
		struct iris_cmd_set *cmdset,
		struct iris_cmd_desc *dsi_cmd)
{
	if (!cmdset || !dsi_cmd) {
		IRIS_LOGE("%s(), invalid input param", __func__);
		return;
	}

	cmdset->cmds = dsi_cmd;
	cmdset->count = 1;
}

static int _iris_pt_write_max_pkt_size(
		struct iris_cmd_set *cmdset)
{
	u32 rlen = 0;
	int rc = 0;
	struct iris_cmd_set local_cmdset;
	static char max_pktsize[2] = {0x00, 0x00}; /* LSB tx first, 10 bytes */
	static struct iris_cmd_desc pkt_size_cmd = {
		{0, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, MIPI_DSI_MSG_REQ_ACK,
			sizeof(max_pktsize), max_pktsize, 0, NULL}, 1, 0};

	rlen = cmdset->cmds[0].msg.rx_len;
	if (rlen > 128) {
		IRIS_LOGE("%s(), invalid len: %d", __func__, rlen);
		return -EINVAL;
	}

	max_pktsize[0] = (rlen & 0xFF);
	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	_iris_pt_switch_cmd(&local_cmdset, &pkt_size_cmd);
	rc = _iris_pt_write_panel_cmd(&local_cmdset);

	return rc;
}

static int _iris_pt_send_panel_rdcmd(
		struct iris_cmd_set *cmdset)
{
	struct iris_cmd_set local_cmdset;
	struct iris_cmd_desc *dsi_cmd = cmdset->cmds;
	int rc = 0;

	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	_iris_pt_switch_cmd(&local_cmdset, dsi_cmd);

	/*passthrough write to panel*/
	rc = _iris_pt_write_panel_cmd(&local_cmdset);
	return rc;
}

static int _iris_pt_remove_respond_hdr(char *ptr, int *offset)
{
	int rc = 0;
	char cmd;

	if (!ptr)
		return -EINVAL;

	cmd = ptr[0];
	IRIS_LOGV("%s(), cmd: 0x%02x", __func__, cmd);
	switch (cmd) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		IRIS_LOGD("%s(), rx ACK_ERR_REPORT", __func__);
		rc = -EINVAL;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		*offset = 1;
		rc = 1;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		*offset = 1;
		rc = 2;
		break;
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		*offset = 4;
		rc = ptr[1];
		break;
	default:
		rc = 0;
	}

	return rc;
}

static int _iris_pt_read(struct iris_cmd_set *cmdset, uint8_t path)
{
	u32 i = 0;
	u32 rlen = 0;
	u32 intstat = 0;
	int retry_cnt, frame_interval;
	u32 offset = 0;
	int rc = 0;
	union iris_mipi_tx_cmd_payload val;
	u8 *rbuf = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = 0;

	switch (pcfg->chip_ver) {
	case IRIS7_CHIP_VERSION:
	case IRIS5_CHIP_VERSION:
		address = IRIS_RD_PACKET_DATA;
		break;
	case IRIS3_CHIP_VERSION:
		address = IRIS_RD_PACKET_DATA_I3;
		break;
	default:
		IRIS_LOGE("chip version not supported!");
		break;
	}

	rbuf = (u8 *)cmdset->cmds[0].msg.rx_buf;
	rlen = cmdset->cmds[0].msg.rx_len;

	if (!rbuf || rlen <= 0) {
		IRIS_LOGE("%s(), rbuf: %p, rlen: %d", __func__, rbuf, rlen);
		return -EINVAL;
	}

	if (pcfg->timing.refresh_rate != 0)
		frame_interval = 1000/pcfg->timing.refresh_rate + 1;
	else
		frame_interval = 1000/60 + 1;

	retry_cnt = frame_interval * 2;
	IRIS_LOGD("%s, frame_interval is %d ms, retry_cnt %d", __func__, frame_interval, retry_cnt);

	/*read iris for data*/
	for (i = 0; i < retry_cnt; i++) {
		usleep_range(1000, 1001);
		intstat = iris_ocp_read(IRIS_TX_INTSTAT_RAW, cmdset->state);

		if (intstat & IRIS_TX_READ_RESPONSE_RECEIVED)
			break;
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
			IRIS_LOGI("%s retry: %d", __func__, (i + 1));
	}

	if (intstat & IRIS_TX_READ_ERR_MASK) {
		rc = -1;
		IRIS_LOGE("%s(), Tx read error 0x%x, rc: %d",
				  __func__, intstat, rc);
		return rc;
	}
	val.pld32 = iris_ocp_read(address, cmdset->state);

	rlen = _iris_pt_remove_respond_hdr(val.p, &offset);
	IRIS_LOGV("%s(), read len: %d", __func__, rlen);

	if (rlen <= 0) {
		rc = -1;
		IRIS_LOGE("%s(), failed to remove respond header, val: 0x%x, rlen: %d, rc: %d",
				  __func__, val.pld32, rlen, rc);
		return rc;
	}

	if (rlen <= 2) {
		if (offset < 4) {
			for (i = 0; i < rlen; i++)
				rbuf[i] = val.p[offset + i];
		} else {
			val.pld32 = iris_ocp_read(address, cmdset->state);
			for (i = 0; i < rlen; i++)
				rbuf[i] = val.p[i];
		}
	} else {
		int j = 0;
		int len = 0;
		int num = (rlen + 3) / 4;

		for (i = 0; i < num; i++) {
			len = (i == num - 1) ? rlen - 4 * i : 4;
			val.pld32 = iris_ocp_read(address, IRIS_CMD_SET_STATE_HS);
			for (j = 0; j < len; j++)
				rbuf[i * 4 + j] = val.p[j];
		}
	}

	return rc;
}

int iris_pt_read_panel_cmd(
						   struct iris_cmd_set *cmdset)
{
	u8 vc_id = 0;
	int rc = 0;
	struct iris_cfg *pcfg = NULL;

	IRIS_LOGD("%s(), enter", __func__);

	if (!cmdset || cmdset->count != 1) {
		IRIS_LOGE("%s(), invalid input, cmdset: %p", __func__, cmdset);
		return -EINVAL;
	}

	pcfg = iris_get_cfg();


	if (!pcfg->vc_ctrl.vc_enable) {
		/*step1  write max packet size*/
		rc = _iris_pt_write_max_pkt_size(cmdset);

		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			return rc;
		}
		iris_ocp_write_val(IRIS_TX_INTCLR, 0xFFFFFFFF);
		/*step2 write read cmd to panel*/
		rc = _iris_pt_send_panel_rdcmd(cmdset);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			goto exit;
		}

		/*step3 delay one frame*/
		//msleep(1000);

		/*step3 read panel data*/
		rc = _iris_pt_read(cmdset, pcfg->read_path);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			goto exit;
		}
	} else {
		vc_id = (cmdset->state == IRIS_CMD_SET_STATE_LP) ?
			pcfg->vc_ctrl.to_panel_lp_vc_id :
			pcfg->vc_ctrl.to_panel_hs_vc_id;
		rc = pcfg->lightup_ops.transfer(cmdset->cmds,
								cmdset->count, IRIS_CMD_SET_STATE_HS, vc_id);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			goto exit;
		}
	}
exit:
	return rc;
}

int iris_pt_send_panel_cmd(
		struct iris_cmd_set *cmdset)
{
	u8 vc_id = 0;
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	if (!cmdset) {
		IRIS_LOGE("%s(), invalid cmdset: %p",
			__func__, cmdset);
		return -EINVAL;
	}

	if (cmdset->count == 1 && cmdset->cmds[0].msg.type == MIPI_DSI_DCS_READ) {
		ktime_t lp_ktime0;
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			lp_ktime0 = ktime_get();
		}
		rc = iris_pt_read_panel_cmd(cmdset);
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			IRIS_LOGI("%s rc: %d, spend time %d us", __func__,
				rc,(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
		}
		return rc;
	}

	if (!pcfg->vc_ctrl.vc_enable) {
		IRIS_LOGI("using ocp type to panel");
		_iris_pt_write_panel_cmd(cmdset);
	} else {
		vc_id = (cmdset->state == IRIS_CMD_SET_STATE_LP) ?
			pcfg->vc_ctrl.to_panel_lp_vc_id :
			pcfg->vc_ctrl.to_panel_hs_vc_id;
		IRIS_LOGI("using aux channel %d", vc_id);
		pcfg->lightup_ops.transfer(cmdset->cmds,
			cmdset->count, IRIS_CMD_SET_STATE_HS, vc_id);
	}

	return rc;
}

int iris_abyp_send_panel_cmd(struct iris_cmd_set *cmdset)
{
	u8 vc_id = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!cmdset ) {
		IRIS_LOGE("%s(), invalid cmdset: %p",
			__func__, cmdset);
		return -EINVAL;
	}

	pcfg->lightup_ops.transfer(cmdset->cmds,
			cmdset->count, cmdset->state, vc_id);
	return 0;
}





void iris_set_pwil_mode(u8 mode, bool osd_enable, int state, bool commit)
{
	char pwil_mode[2] = {0x00, 0x00};
	u32 meta;
	bool dsc_en = false;

	struct iris_cfg *pcfg = iris_get_cfg();

	if (mode == PT_MODE) {
		pwil_mode[0] = 0x0;
		pwil_mode[1] = 0x81;
	} else if (mode == RFB_MODE) {
		pwil_mode[0] = 0xc;
		pwil_mode[1] = 0x81;
	} else if (mode == FRC_MODE) {
		pwil_mode[0] = 0x4;
		pwil_mode[1] = 0x82;
	}
	if (osd_enable)
		pwil_mode[0] |= 0x80;

	if (pcfg->iris_memc_ops.iris_memc_get_main_panel_dsc_en_info)
		pcfg->iris_memc_ops.iris_memc_get_main_panel_dsc_en_info(&dsc_en);

	if (pcfg && dsc_en)
		pwil_mode[0] |= 0x10;

	IRIS_LOGI("%s(), set pwil mode: %x, %x", __func__, pwil_mode[0], pwil_mode[1]);

	meta = pwil_mode[0] | (pwil_mode[1] << 8);
	if (commit)
		pcfg->lightup_ops.send_pwil_cmd(NULL, IRIS_RX0_ADDR + RX_VDO_META, meta);
	iris_rx_meta_dma_list_send(meta, commit);
}

int iris_platform_get(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->platform_type;
}


void iris_dtg_eco(bool enable, bool chain)
{
	u32 *payload = NULL;

	IRIS_LOGI("%s: %d", __func__, enable);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x90, 2);
	if (enable)
		payload[0] |= 0x800;
	else
		payload[0] &= ~0x800;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x90, 0x90, 1);
	if (chain) {
		iris_dma_trig(DMA_CH12, 0);
		iris_update_pq_opt(PATH_DSI, true);
	}
}

void iris_dsi_rx_mode_switch(uint8_t rx_mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;
	u32 ovs_dly_rfb;
	u32 *payload = NULL;

	IRIS_LOGI("%s: %d", __func__, rx_mode);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xf0, 3);
	if (rx_mode == IRIS_CMD_MODE)
		payload[0] |= 0x20001;
	else
		payload[0] &= ~0x20001;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xf0, 0xf0, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x70, 3);
	if (rx_mode == IRIS_CMD_MODE)
		payload[0] &= ~0x4000;
	else
		payload[0] |= 0x4000;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x70, 0x70, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x90, 2);
	if (rx_mode == IRIS_CMD_MODE)
		payload[0] &= ~0x800;
	else
		payload[0] |= 0x800;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x90, 0x90, 1);
	iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe6, 0xe6, 1);

	regval.ip = IRIS_IP_DTG;
	regval.opt_id = ID_DTG_TE_SEL;
	regval.mask = 0x0000001C;
	regval.value = ((rx_mode == IRIS_CMD_MODE) ? 0x00000014 : 0x00000000);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);

	if (rx_mode == IRIS_CMD_MODE)
		iris_init_update_ipopt_t(IRIS_IP_DTG, 0xf3, 0xf3, 0x01);
	else {
		payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0xf5, 2);
		ovs_dly_rfb = payload[0];
		payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0xf8, 2);
		payload[3] = ovs_dly_rfb;
		iris_init_update_ipopt_t(IRIS_IP_DTG, 0xf8, 0xf8, 0x01);
	}

	regval.ip = IRIS_IP_DTG;
	regval.opt_id = 0xF0;
	regval.mask = 0x0000000F;
	regval.value = 0x2;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DTG, 0xF0, 0xF0, 0);
	iris_update_pq_opt(PATH_DSI, true);

	pcfg->rx_mode = rx_mode;
}

void iris_dtg_update_reset(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;

	if ((pcfg->rx_mode == IRIS_VIDEO_MODE) && (pcfg->tx_mode == IRIS_VIDEO_MODE)) {
		regval.ip = IRIS_IP_DTG;
		regval.opt_id = 0xF0;
		regval.mask = 0x0000000F;
		regval.value = 0xF;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(IRIS_IP_DTG, 0xF0, 0xF0, 0);
	}
}

void iris_sw_te_enable(void)
{
	u32 *payload = NULL;
	u32 dtg_ctrl;
	u32 cmd[8];

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, ID_DTG_TE_SEL, 2);
	dtg_ctrl = payload[0];
	cmd[0] = IRIS_DTG_ADDR + DTG_CTRL;
	cmd[1] = dtg_ctrl & ~0x800;
	cmd[2] = IRIS_DTG_ADDR + DTG_UPDATE;
	cmd[3] = 0x1;
	cmd[4] = IRIS_DTG_ADDR + DTG_CTRL;
	cmd[5] = dtg_ctrl | 0x800 | 0x14;
	cmd[6] = IRIS_DTG_ADDR + DTG_UPDATE;
	cmd[7] = 0x1;
	iris_ocp_write_mult_vals(8, cmd);
}

void iris_ovs_dly_change(bool enable)
{
	u32 *payload = NULL;
	u32 ovs_dly_pt, ovs_dly_rfb;
	u32 cmd[4];

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0x00, 2);
	ovs_dly_pt = payload[15];
	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0xf5, 2);
	ovs_dly_rfb = payload[0];

	cmd[0] = IRIS_DTG_ADDR + OVS_DLY;
	if (enable)
		cmd[1] = ovs_dly_rfb;
	else
		cmd[1] =  ovs_dly_pt;
	cmd[2] = IRIS_DTG_ADDR + DTG_UPDATE;
	cmd[3] = 0x2;
	iris_ocp_write_mult_vals(4, cmd);
}


void iris_set_valid(int step)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->valid == PARAM_EMPTY)
		return;
	pcfg->valid = step;
}
EXPORT_SYMBOL(iris_set_valid);
#endif
