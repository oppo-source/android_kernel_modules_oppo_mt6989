// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include "pw_iris_lightup.h"
#include "pw_iris_log.h"

void iris_mult_addr_pad_i7p(uint8_t **p, uint32_t *poff, uint32_t left_len)
{

	switch (left_len) {
	case 4:
		iris_set_ocp_type(*p, 0xFFFFFFFF);
		*p += 4;
		*poff += 4;
		break;
	case 8:
		iris_set_ocp_type(*p, 0xFFFFFFFF);
		iris_set_ocp_base_addr(*p, 0xFFFFFFFF);
		*p += 8;
		*poff += 8;
		break;
	case 12:
		iris_set_ocp_type(*p, 0xFFFFFFFF);
		iris_set_ocp_base_addr(*p, 0xFFFFFFFF);
		iris_set_ocp_first_val(*p, 0xFFFFFFFF);
		*p += 12;
		*poff += 12;
		break;
	case 0:
		break;
	default:
		IRIS_LOGE("%s()%d, left len not aligh to 4.", __func__, __LINE__);
		break;
	}
}

int iris_convert_dsi_to_i2c_i7p(uint8_t *payload)
{
	uint8_t slot;
	uint32_t header, address;
	uint32_t *pval = (uint32_t *)payload;

	header = cpu_to_le32(pval[0]);
	address = cpu_to_le32(pval[1]);
	IRIS_LOGD("%s,%d: header = 0x%08x, addr = 0x%08x", __func__, __LINE__, pval[0], pval[1]);

	if ((header & 0xf) == 0xc) {  //direct bus
		slot = (header >> 24) & 0xf;
		switch (slot) {
		case 0:
		case 1:
			header = 0x00000000;
			break;
		case 2:
			header = 0x00000000;
			if (address >= 0x6800 && address <= 0x6E5C)
				address += 0xF1680000;
			else if (address >= 0x6E60 && address <= 0x7300)
				address += 0xF1680000;
			break;
		case 3:
			header = 0x00000000;
			if ((address >= 0x6380) && (address <= 0x67FC)) {
				address += 0xF1680000;
			} else if ((address >= 0x7800) && (address <= 0x7BFC)) {
				address += 0xF1680000;
			} else if ((address >= 0x7C00) && (address <= 0x8000)) {
				address += 0xF1680000;
			} else if ((address >= 0x8000) && (address <= 0x1BEEC)) {
				address += 0xF1680000;
			} else if (address >= 0x1BEF0 && address < 0x2FDE0) {
				address += 0xF16AC110;
			} else {
				IRIS_LOGE("%s(): invalid addr in slot 3\n", __func__);
				return -EINVAL;
			}
			break;
		default:
			IRIS_LOGE("%s(): invalid direct bus slot num %d\n", __func__, slot);
			return -EINVAL;
		}
		pval[0] = header;
		pval[1] = address;
	}
	IRIS_LOGD("%s,%d: header = 0x%08x, addr = 0x%08x", __func__, __LINE__, pval[0], pval[1]);
	return header == 0x00000000 ? I2C_MULT_CONT_ADDR_VAL : I2C_MULT_ADDR_VAL_PAIR;
 }