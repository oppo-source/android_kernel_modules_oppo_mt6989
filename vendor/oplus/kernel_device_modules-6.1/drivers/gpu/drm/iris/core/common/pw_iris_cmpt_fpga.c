// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2019-2023, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2023.
 */

#include "pw_iris_api.h"

void iris_fpga_adjust_read_cnt(u32 read_offset,
							   u32 rx_byte,
							   u32 read_cnt,
							   int *pcnt)
{
	/*
	 *  Warn: pls do not port to customer, this change just used for
	 *        hx8379a fwvga vinvout panel, since this panel always return
	 *        long response data in esd read: 9c 01 00 13 1c 6a d5
	 */
	if (iris_need_short_read_workaround()) {
		if (read_offset == 0)
			if ((rx_byte == 4) && ((read_cnt == 7) || (read_cnt == 11)))
				*pcnt += (read_cnt - rx_byte) / 4 + (read_cnt - rx_byte) % 4 ? 1 : 0;
	}
}

void iris_fpga_adjust_read_buf(u32 repeated_bytes,
							   u32 read_offset,
							   u32 rx_byte,
							   u32 read_cnt,
							   u8 *rd_buf)
{
	int i = 0;
	/*
	 *  Warn: pls do not port to customer, this change just used for
	 *        hx8379a fwvga vinvout panel, since this panel always return
	 *        long response data in esd read: 9c 01 00 13 1c 6a d5
	 */
	if (iris_need_short_read_workaround()) {
		if (!repeated_bytes) {
			if (read_offset == 0)
				if ((rx_byte == 4) && ((read_cnt == 7) || (read_cnt == 11))) {
					for (i = 0; i < read_cnt; i++)
						rd_buf[i] = rd_buf[i+1];
				}
		}
	}
}
