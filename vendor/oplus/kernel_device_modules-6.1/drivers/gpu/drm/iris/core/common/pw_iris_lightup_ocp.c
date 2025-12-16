/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
//#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/vmalloc.h>
#include <video/mipi_display.h>

#include "pw_iris_def.h"
#include "pw_iris_lightup_ocp.h"
#include "pw_iris_lightup.h"
#include "pw_iris_api.h"
#include "pw_iris_log.h"
#include "pw_iris_i3c.h"
#include "pw_iris_lp.h"
#include "pw_iris_api.h"

#define IRIS_TX_HV_PAYLOAD_LEN   120
#define IRIS_TX_PAYLOAD_LEN 124

#define IRIS_TX_READ_RESPONSE_RECEIVED 0x80000000
#define IRIS_TX_READ_ERR_MASK 0x6FEFFEFF
#define IRIS_TX_READ_ERR_MASK_IRIS8 0x4FEFFEFF
#define IRIS_RD_PACKET_DATA_I3  0xF0C1C018

static char iris_read_cmd_rbuf[16];
static struct iris_ocp_cmd ocp_cmd;
static struct iris_ocp_cmd ocp_test_cmd[DSI_CMD_CNT];
static struct iris_cmd_desc iris_test_cmd[DSI_CMD_CNT];

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
		{{
			.channel = 0,
			.type = MIPI_DSI_GENERIC_LONG_WRITE,
			.flags = 0,
			.tx_len = CMD_PKT_SIZE,
			.tx_buf = ocp_cmd.cmd,
			}, 1, 0} };
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
void iris_dsi_write_mult_vals(u32 size, u32 *pvalues)
{
	int i, j;
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
	max_size -= max_size % 2;
	cmds_count = (size + max_size - 1) / max_size;
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

	for (i = 0; i < cmds_count; i++) {
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

static void _iris_i2c_write_mult_vals(u32 size, u32 *pvalues)
{
	int ret = 0;
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (size % 2 != 0) {
		IRIS_LOGE("%s(), need to be mult pair of address and value", __func__);
		return;
	}

	while (i < size) {
		ret |= pcfg->iris_i2c_write(pvalues[i], pvalues[i+1]);
		i += 2;
	}

	if (ret)
		IRIS_LOGE("%s(%d), i2c send fail, return: %d",
				__func__, __LINE__, ret);
}

/*pvalues need to be one address and one value*/
void iris_ocp_write_mult_vals(u32 size, u32 *pvalues)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int path = pcfg->iris_i2c_preload ? PATH_I2C : PATH_DSI;

	if (pcfg->force_i2c_type)
		path = PATH_I2C;

	if (pcfg->iris_chip_type == CHIP_IRIS5 &&
		pcfg->pw_chip_func_ops.iris_ocp_write_mult_vals_i5_)
		pcfg->pw_chip_func_ops.iris_ocp_write_mult_vals_i5_(size, pvalues);
	else if (path == PATH_I2C) {
		_iris_i2c_write_mult_vals(size, pvalues);
		IRIS_LOGD("%s(%d), path select i2c", __func__, __LINE__);
	} else {
		iris_dsi_write_mult_vals(size, pvalues);
		IRIS_LOGD("%s(%d), path select dsi", __func__, __LINE__);
	}
}

static void _iris_ocp_write_addr(u32 address, u32 mode)
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
	/* Send OCP command.*/
	memset(&ocp_cmd, 0, sizeof(ocp_cmd));

	_iris_add_cmd_payload(&ocp_cmd, OCP_SINGLE_READ);
	_iris_add_cmd_payload(&ocp_cmd, address);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	pcfg->lightup_ops.transfer(iris_ocp_cmd, 1, mode, pcfg->vc_ctrl.to_iris_vc_id);
}

static u32 _iris_ocp_read_value(u32 mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	char pi_read[1] = {0x00};
	struct iris_cmd_desc pi_read_cmd[] = {
		{{
			.channel = 0,
			.type = MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM,
			.flags = MIPI_DSI_MSG_REQ_ACK,
			.tx_len = sizeof(pi_read),
			.tx_buf = pi_read,
		 }, 1, 0} };
	u32 response_value;

	if (pcfg->iris_set_msg_flags)
		pcfg->iris_set_msg_flags(pi_read_cmd, READ_FLAG);
	/* Read response.*/
	memset(iris_read_cmd_rbuf, 0, sizeof(iris_read_cmd_rbuf));
	pi_read_cmd[0].msg.rx_len = 4;
	pi_read_cmd[0].msg.rx_buf = iris_read_cmd_rbuf;
	pcfg->lightup_ops.transfer(pi_read_cmd, 1, mode, pcfg->vc_ctrl.to_iris_vc_id);

	IRIS_LOGD("%s(), read register: 0x%02x 0x%02x 0x%02x 0x%02x", __func__,
			iris_read_cmd_rbuf[0], iris_read_cmd_rbuf[1],
			iris_read_cmd_rbuf[2], iris_read_cmd_rbuf[3]);

	response_value = iris_read_cmd_rbuf[0] | (iris_read_cmd_rbuf[1] << 8)
				| (iris_read_cmd_rbuf[2] << 16) | (iris_read_cmd_rbuf[3] << 24);

	return response_value;
}

u32 _iris_dsi_ocp_read(u32 address, u32 mode)
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

void _iris_dump_packet(u8 *data, int size)
{
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4, data, size, false);
}

void iris_write_test(u32 iris_addr,
		int ocp_type, u32 pkt_size)
{
	union iris_ocp_cmd_header ocp_header;
	struct iris_cmd_desc iris_cmd = {
		{
			.channel = 0,
			.type = MIPI_DSI_GENERIC_LONG_WRITE,
			.flags = 0,
			.tx_len = CMD_PKT_SIZE,
			.tx_buf = ocp_cmd.cmd,
		 }, 1, 0};
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
		struct iris_ocp_cmd *pocp_cmd, u8 wait, u16 flags)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cmd_desc desc_init_val = {
		{
			.channel = 0,
			.type = MIPI_DSI_GENERIC_LONG_WRITE,
			.flags = 0,
			.tx_len = CMD_PKT_SIZE,
			.tx_buf = NULL,
		}, 0, 0};

	memcpy(ptx_cmd, &desc_init_val, sizeof(struct iris_cmd_desc));
	ptx_cmd->msg.tx_buf = pocp_cmd->cmd;
	ptx_cmd->msg.tx_len = pocp_cmd->cmd_len;
	ptx_cmd->msg.flags = flags;
	if (pcfg->iris_is_last_cmd)
		ptx_cmd->last_command = pcfg->iris_is_last_cmd(&ptx_cmd->msg);
	ptx_cmd->post_wait_ms = wait;
}

static u32 _iris_pt_short_write(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS7_CHIP_VERSION ?
		pcfg->iris_mipi_tx_header_addr : pcfg->iris_mipi_tx_header_addr_i3;

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
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS7_CHIP_VERSION ?
		pcfg->iris_mipi_tx_header_addr : pcfg->iris_mipi_tx_header_addr_i3;

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
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS7_CHIP_VERSION ?
		pcfg->iris_mipi_tx_payload_addr : pcfg->iris_mipi_tx_payload_addr_i3;

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
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS7_CHIP_VERSION ?
		pcfg->iris_mipi_tx_header_addr : pcfg->iris_mipi_tx_header_addr_i3;

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
	u16 flags = 0;
	union iris_mipi_tx_cmd_header header;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	_iris_pt_init_tx_cmd_hdr(cmdset, dsi_cmd, &header);

	dtype = pcfg->iris_switch_cmd_type(dsi_cmd->msg.type);
	if ((pcfg->iris_chip_type == CHIP_IRIS7P)||(pcfg->iris_chip_type == CHIP_IRIS8))
		dtype &= 0x3f;//add for FPGA->I7 scaler up use case
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
		if ((i != sum - 1) && pcfg->iris_set_msg_flags)
			pcfg->iris_set_msg_flags(dsi_cmd, BATCH_FLAG);
		flags = dsi_cmd->msg.flags;
		_iris_add_tx_cmds(ptx_cmd + i, pocp_cmd + i, wait, flags);
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
	if (iris_get_abyp_mode() == ANALOG_BYPASS_MODE) {
		memcpy(dest, src, len);
		return len;
	}

	cmdset.cmds = &desc;
	cmdset.count = 1;
	cmdset.state = IRIS_CMD_SET_STATE_HS;
	desc.msg.tx_buf = src;
	desc.msg.tx_len = len;

	ret = __iris_pt_write_panel_cmd(&cmdset, &pocp_cmds, &ptx_cmds);
	if (ret == -EINVAL)
		return ret;
	else if (ret == 1) {
		_iris_pt_send_cmds(ptx_cmds, (u32)ret);
	} else
		ret = -EINVAL;

	kvfree(pocp_cmds);
	kvfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;

	return ret;
}
EXPORT_SYMBOL(iris_conver_one_panel_cmd);

int iris_vdo_panel_cmd(u8 *dest, u8 *src, int len)
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
	if (iris_get_abyp_mode() == ANALOG_BYPASS_MODE) {
		memcpy(dest, src, len);
		return len;
	}

	cmdset.cmds = &desc;
	cmdset.count = 1;
	cmdset.state = IRIS_CMD_SET_STATE_HS;
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

int  _iris_pt_write_panel_cmd(struct iris_cmd_set *cmdset)
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
	else if (ret > 0)
		ret = _iris_pt_send_cmds(ptx_cmds, (u32)ret);

	kvfree(pocp_cmds);
	kvfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;
	return ret;
}

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

static int _iris_pt_write_max_pkt_size(struct iris_cmd_set *cmdset)
{
	u32 rlen = 0;
	int rc = 0;
	struct iris_cmd_set local_cmdset;
	static char max_pktsize[2] = {0x00, 0x00}; /* LSB tx first, 10 bytes */
	static struct iris_cmd_desc pkt_size_cmd = {
		{
			.channel = 0,
			.type = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
			.flags =  MIPI_DSI_MSG_REQ_ACK,
			.tx_len = sizeof(max_pktsize),
			.tx_buf =  max_pktsize,
		}, 1, 0};

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

static int _iris_pt_send_panel_rdcmd(struct iris_cmd_set *cmdset)
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
	u32 refresh_rate = 0;
	u32 iris_tx_read_err_mask = IRIS_TX_READ_ERR_MASK;
	struct iris_mode_info timing_info;

	switch (pcfg->chip_ver) {
	case IRIS7_CHIP_VERSION:
	case IRIS5_CHIP_VERSION:
		address = pcfg->iris_rd_packet_data;
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
	if (pcfg->lightup_ops.obtain_cur_timing_info(&timing_info) == 0)
		refresh_rate = timing_info.refresh_rate;

	if (refresh_rate != 0)
		frame_interval = 1000/refresh_rate + 1;
	else
		frame_interval = 1000/60 + 1;

	retry_cnt = frame_interval * 2;
	IRIS_LOGD("%s, frame_interval is %d ms, retry_cnt %d", __func__, frame_interval, retry_cnt);

	/*read iris for data*/
	for (i = 0; i < retry_cnt; i++) {
		usleep_range(1000, 1001);
		intstat = iris_ocp_read(pcfg->iris_tx_intstat_raw, cmdset->state);

		if (intstat & IRIS_TX_READ_RESPONSE_RECEIVED)
			break;
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
			IRIS_LOGI("%s retry: %d", __func__, (i + 1));
	}

	if (pcfg->iris_chip_type == CHIP_IRIS8)
		iris_tx_read_err_mask = IRIS_TX_READ_ERR_MASK_IRIS8;

	if (intstat & iris_tx_read_err_mask) {
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

int iris_pt_read_panel_cmd(struct iris_cmd_set *cmdset)
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
		iris_ocp_write_val(pcfg->iris_tx_intclr, 0xFFFFFFFF);
		/*step2 write read cmd to panel*/
		rc = _iris_pt_send_panel_rdcmd(cmdset);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			goto exit;
		}
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

int iris_pt_send_panel_cmd(struct iris_cmd_set *cmdset)
{
	u8 vc_id = 0;
	int rc = 0;
	struct iris_cmd_set local_cmdset;
	struct iris_cmd_desc *local_cmd = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!cmdset) {
		IRIS_LOGE("%s(), cmdset: %p", __func__, cmdset);
		return -EINVAL;
	}

	memset(&local_cmdset, 0x00, sizeof(local_cmdset));
	local_cmd = kvzalloc(cmdset->count * sizeof(struct iris_cmd_desc), GFP_KERNEL);
	if (!local_cmd) {
		IRIS_LOGE("%s(), failed to alloc desc cmd", __func__);
		return -ENOMEM;
	}
	memcpy(local_cmd, cmdset->cmds, cmdset->count * sizeof(struct iris_cmd_desc));
	memcpy(&local_cmdset, cmdset, sizeof(local_cmdset));
	local_cmdset.cmds = local_cmd;

	{
		int32_t i = 0;
		struct iris_cmd_desc *cmds = local_cmdset.cmds;
		enum iris_cmd_set_state state = local_cmdset.state;

		for (i = 0; i < local_cmdset.count; i++) {
			if (state == IRIS_CMD_SET_STATE_LP)
				cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

			if (cmds->last_command)
				cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

			cmds++;
		}
	}

	if (local_cmdset.count == 1 && pcfg->iris_is_read_cmd(&local_cmdset.cmds[0])) {
		ktime_t lp_ktime0;

		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
			lp_ktime0 = ktime_get();

		rc = iris_pt_read_panel_cmd(&local_cmdset);
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			IRIS_LOGI("%s rc: %d, spend time %d us", __func__,
				rc, (u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
		}
		kvfree(local_cmd);
		return rc;
	}

	if (!pcfg->vc_ctrl.vc_enable) {
		IRIS_LOGD("using ocp type to panel");
		_iris_pt_write_panel_cmd(&local_cmdset);
	} else {
		vc_id = (local_cmdset.state == IRIS_CMD_SET_STATE_LP) ?
			pcfg->vc_ctrl.to_panel_lp_vc_id :
			pcfg->vc_ctrl.to_panel_hs_vc_id;
		IRIS_LOGD("using aux channel %d", vc_id);
		pcfg->lightup_ops.transfer(local_cmdset.cmds,
			local_cmdset.count, IRIS_CMD_SET_STATE_HS, vc_id);
	}

	kvfree(local_cmd);
	return rc;
}

int iris_abyp_send_panel_cmd(struct iris_cmd_set *cmdset)
{
	u8 vc_id = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!cmdset) {
		IRIS_LOGE("%s(), cmdset: %p", __func__, cmdset);
		return -EINVAL;
	}

	pcfg->lightup_ops.transfer(cmdset->cmds,
			cmdset->count, cmdset->state, vc_id);
	return 0;
}

int iris_platform_get(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->platform_type;
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

//TODO need to remove
#define ID_DTG_TE_SEL 0xF1
void iris_sw_te_enable(void)
{
	u32 *payload = NULL;
	u32 dtg_ctrl;
	u32 cmd[8];
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->abyp_ctrl.abypass_mode != PASS_THROUGH_MODE)
		return;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, ID_DTG_TE_SEL, 2);
	dtg_ctrl = payload[0];
	cmd[0] = pcfg->iris_dtg_addr + pcfg->dtg_ctrl;
	cmd[1] = dtg_ctrl & ~0x800;
	cmd[2] = pcfg->iris_dtg_addr + pcfg->dtg_update;
	cmd[3] = 0x1;
	cmd[4] = pcfg->iris_dtg_addr + pcfg->dtg_ctrl;
	cmd[5] = dtg_ctrl | 0x800 | 0x14;
	cmd[6] = pcfg->iris_dtg_addr + pcfg->dtg_update;
	cmd[7] = 0x1;
	iris_ocp_write_mult_vals(8, cmd);
}

void iris_ovs_dly_change(bool enable)
{
	u32 *payload = NULL;
	u32 ovs_dly_pt, ovs_dly_rfb;
	u32 cmd[4];
	struct iris_cfg *pcfg = iris_get_cfg();

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0x00, 2);
	ovs_dly_pt = payload[15];
	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0xf5, 2);
	ovs_dly_rfb = payload[0];

	cmd[0] = pcfg->iris_dtg_addr + pcfg->ovs_dly;
	if (enable)
		cmd[1] = ovs_dly_rfb;
	else
		cmd[1] =  ovs_dly_pt;
	cmd[2] = pcfg->iris_dtg_addr + pcfg->dtg_update;
	cmd[3] = 0x2;
	iris_ocp_write_mult_vals(4, cmd);
}

void iris_panel_cmd_passthrough(unsigned int cmd, unsigned char count, unsigned char *para_list,
	u8 *buffer, u8 buffer_size, unsigned char rd_cmd)
{
	u8 buf[512] = {0,};
	u32 sum = 1;
	struct iris_cmd_set cmdset;
	struct iris_cmd_desc desc[1];

	memset(&cmdset, 0x00, sizeof(cmdset));
	cmdset.cmds = desc;

	buf[0] = cmd;
	if (!rd_cmd) {
		memcpy(buf+1, para_list, count);
		sum = count+1;
	} else {
		cmdset.cmds[0].msg.rx_buf = buffer;
		cmdset.cmds[0].msg.rx_len = buffer_size;
	}

	cmdset.cmds[0].msg.tx_buf = buf;
	cmdset.cmds[0].msg.tx_len = sum;
	cmdset.count = 1;
	cmdset.state = IRIS_CMD_SET_STATE_LP;

	if (iris_is_pt_mode(false))
		iris_pt_send_panel_cmd(&cmdset);
	else
		iris_abyp_send_panel_cmd(&cmdset);
}
EXPORT_SYMBOL(iris_panel_cmd_passthrough);

void iris_set_valid(int step)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->valid == PARAM_EMPTY)
		return;
	pcfg->valid = step;
}
EXPORT_SYMBOL(iris_set_valid);

int iris_get_valid(void)
{
       struct iris_cfg *pcfg = iris_get_cfg();

       if (pcfg)
               return pcfg->valid;

       return PARAM_NONE;
}
EXPORT_SYMBOL(iris_get_valid);

