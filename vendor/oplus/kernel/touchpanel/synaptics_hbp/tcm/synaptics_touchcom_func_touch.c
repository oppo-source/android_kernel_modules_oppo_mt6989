// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TCM touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file synaptics_tcm2_func_touch.c
 *
 * This file implements the touch report handling functions.
 * The declarations are available in synaptics_touchcom_func_touch.h.
 */

#include "synaptics_touchcom_func_touch.h"
#include "../syna_tcm2.h"
#include "../touchpanel_healthinfo/touchpanel_healthinfo.h"

/**
 * syna_tcm_get_touch_data()
 *
 * Get data entity from the received report according to bit offset and bit
 * length defined in the touch report configuration.
 *
 * @param
 *    [ in] report:      touch report generated by TouchComm device
 *    [ in] report_size: size of given report
 *    [ in] offset:      bit offset in the report
 *    [ in] bits:        number of bits representing the data
 *    [out] data:        data parsed
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_touch_data(const unsigned char *report,
		unsigned int report_size, unsigned int offset,
		unsigned int bits, unsigned int *data)
{
	unsigned char mask;
	unsigned char byte_data;
	unsigned int output_data;
	unsigned int bit_offset;
	unsigned int byte_offset;
	unsigned int data_bits;
	unsigned int available_bits;
	unsigned int remaining_bits;

	if (bits == 0 || bits > 32) {
		LOGE("Invalid number of bits %d\n", bits);
		return _EINVAL;
	}

	if (!report) {
		LOGE("Invalid report data\n");
		return _EINVAL;
	}

	if (offset + bits > report_size * 8) {
		*data = 0;
		return 0;
	}

	output_data = 0;
	remaining_bits = bits;

	bit_offset = offset % 8;
	byte_offset = offset / 8;

	while (remaining_bits) {
		byte_data = report[byte_offset];
		byte_data >>= bit_offset;

		available_bits = 8 - bit_offset;
		data_bits = MIN(available_bits, remaining_bits);
		mask = 0xff >> (8 - data_bits);

		byte_data &= mask;

		output_data |= byte_data << (bits - remaining_bits);

		bit_offset = 0;
		byte_offset += 1;
		remaining_bits -= data_bits;
	}

	*data = output_data;

	return 0;
}

/**
 * syna_tcm_get_gesture_data()
 *
 * The contents of the gesture data entity depend on which gesture
 * is detected. The default size of data is defined in 16-64 bits natively.
 *
 * @param
 *    [ in] report:       touch report generated by TouchComm device
 *    [ in] report_size:  size of given report
 *    [ in] offset:       bit offset in the report
 *    [ in] bits:         total bits representing the gesture data
 *    [out] gesture_data: gesture data parsed
 *    [ in] gesture_id:   gesture id retrieved
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_get_gesture_data(const unsigned char *report,
		unsigned int report_size, unsigned int offset,
		unsigned int bits, struct tcm_gesture_data_blob *gesture_data,
		unsigned int gesture_id)
{
	int retval;
	unsigned int idx;
	unsigned int data;
	unsigned int size;
	unsigned int data_end;

	if (!report) {
		LOGE("Invalid report data\n");
		return _EINVAL;
	}

	if (offset + bits > report_size * 8)
		return 0;

	data_end = offset + bits;

	size = (sizeof(gesture_data->data) / sizeof(unsigned char));

	idx = 0;
	while ((offset < data_end) && (idx < size)) {
		retval = syna_tcm_get_touch_data(report, report_size,
				offset, 16, &data);
		if (retval < 0) {
			LOGE("Fail to get object index\n");
			return retval;
		}
		gesture_data->data[idx++] = (unsigned char)(data & 0xff);
		gesture_data->data[idx++] = (unsigned char)((data >> 8) & 0xff);
		offset += 16;
	}

	switch (gesture_id) {
	case GESTURE_ID_NONE:
		break;
	case GESTURE_ID_DOUBLE_TAP:
	case GESTURE_ID_ACTIVE_TAP_AND_HOLD:
	case GESTURE_ID_ACTIVE_SINGLE_TAP:
		LOGD("Tap info: (%d, %d)\n",
			syna_pal_le2_to_uint(gesture_data->tap_x),
			syna_pal_le2_to_uint(gesture_data->tap_y));
		break;
	case GESTURE_ID_SWIPE:
		LOGD("Swipe info: direction:%x (%d, %d)\n",
			syna_pal_le2_to_uint(gesture_data->swipe_direction),
			syna_pal_le2_to_uint(gesture_data->tap_x),
			syna_pal_le2_to_uint(gesture_data->tap_y));
		break;
	default:
		LOGW("Unknown gesture_id:%d\n", gesture_id);
		break;
	}

	return 0;
}

/**
 * syna_get_report_data - Retrieve data from touch report
 *
 * @tcm_info: handle of tcm module
 * @offset: start bit of retrieved data
 * @bits: total bits of retrieved data
 * @data: pointer of data, at most 4 byte
 * Retrieve data from the touch report based on the bit offset and bit length
 * information from the touch report configuration.
 */
static int syna_tcm_get_report_data(const unsigned char *touch_report,
				unsigned int report_size, unsigned int offset,
				unsigned int bits, unsigned int *data)
{
	unsigned char mask = 0;
	unsigned char byte_data = 0;
	unsigned int output_data = 0;
	unsigned int bit_offset = offset % 8;
	unsigned int byte_offset = offset / 8;
	unsigned int data_bits = 0;
	unsigned int available_bits = 0;
	unsigned int remaining_bits = bits;
	int retval = 0;

	if (bits == 0 || bits > 32) {
		LOGD("larger than 32 bits:%d\n", bits);
		retval = secure_memcpy((unsigned char *)data, bits / 8, &touch_report[byte_offset],
				   bits / 8, bits / 8);

		if (retval < 0) {
			LOGE("Failed to copy report data\n");
			return retval;
		}

		return 0;
	}

	if (offset + bits > report_size * 8) {
		LOGD("offset and bits beyond total read length\n");
		*data = 0;
		return 0;
	}

	while (remaining_bits) {
		byte_data = touch_report[byte_offset];
		byte_data >>= bit_offset;

		available_bits = 8 - bit_offset;
		data_bits = MIN(available_bits, remaining_bits);
		mask = 0xff >> (8 - data_bits);

		byte_data &= mask;

		output_data |= byte_data << (bits - remaining_bits);

		bit_offset = 0;
		byte_offset += 1;
		remaining_bits -= data_bits;
	}

	*data = output_data;

	return 0;
}

/**
 * syna_tcm_get_custome_grip_info()
 *
 * The contents of the grip info entity
 * is detected. The default size of data is defined in 16-64 bits natively.
 *
 * @param
 *    [ in] report:       touch report generated by TouchComm device
 *    [ in] report_size:  size of given report
 *    [ in] offset:       bit offset in the report
 *    [ in] bits:         total bits representing the gesture data
 *    [out] grip_info:    grip info data parsed
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_get_custome_grip_info(const unsigned char *report,
		unsigned int report_size, unsigned int offset,
		unsigned int bits, struct tcm_grip_info_blob *grip_info)
{
	int retval;
	unsigned int idx;
	unsigned int data;
	unsigned int size;
	unsigned int data_end;

	if (!report) {
		LOGE("Invalid report data\n");
		return _EINVAL;
	}

	if (offset + bits > report_size * 8)
		return 0;

	data_end = offset + bits;

	size = (sizeof(grip_info->data) / sizeof(unsigned char));

	idx = 0;
	while ((offset < data_end) && (idx < size)) {
		retval = syna_tcm_get_touch_data(report, report_size,
				offset, 16, &data);
		if (retval < 0) {
			LOGE("Fail to get object index\n");
			return retval;
		}
		grip_info->data[idx++] = (unsigned char)(data & 0xff);
		grip_info->data[idx++] = (unsigned char)((data >> 8) & 0xff);
		offset += 16;
	}

	/* here only print 4 bytes for debug */
	LOGD("Get Grip Info[%d bits]:0x%08x\n", bits,*((unsigned int*)grip_info->data));

	return 0;
}


/**
 * syna_tcm_parse_touch_report()
 *
 * Traverse through touch report configuration and parse the contents of
 * report packet to get the exactly touched data entity from touch reports.
 *
 * At the end of function, the touched data will be parsed and stored at the
 * associated position in structure touch_data_blob.
 *
 * @param
 *    [ in] tcm_dev:     the device handle
 *    [ in] report:      touch report generated by TouchComm device
 *    [ in] report_size: size of given report
 *    [out] touch_data:  touch data generated
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_parse_touch_report(struct tcm_dev *tcm_dev,
		unsigned char *report, unsigned int report_size,
		struct tcm_touch_data_blob *touch_data)
{
	int retval;
	bool active_only;
	bool num_of_active_objects;
	unsigned char code;
	unsigned int size;
	unsigned int idx;
	unsigned int obj;
	unsigned int next;
	unsigned int data;
	unsigned int bits;
	unsigned int offset;
	unsigned int objects;
	unsigned int active_objects;
	unsigned int config_size;
	unsigned char *config_data;
	struct tcm_objects_data_blob *object_data;
	static unsigned int end_of_foreach;

	struct syna_tcm *tcm = container_of(touch_data, struct syna_tcm, tp_data);

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!report) {
		LOGE("Invalid report data\n");
		return _EINVAL;
	}

	if (report_size <= 0) {
		LOGE("Invalid report data length\n");
		return _EINVAL;
	}

	if (!touch_data) {
		LOGE("Invalid touch data structure\n");
		return _EINVAL;
	}

	if (tcm_dev->max_objects == 0) {
		LOGE("Invalid max_objects supported\n");
		return _EINVAL;
	}

	object_data = touch_data->object_data;

	if (!object_data) {
		LOGE("Invalid object_data\n");
		return _EINVAL;
	}

	config_data = tcm_dev->touch_config.buf;
	config_size = tcm_dev->touch_config.data_length;

	if ((!config_data) || (config_size == 0)) {
		LOGE("Invalid config_data\n");
		return _EINVAL;
	}

	size = sizeof(touch_data->object_data);
	syna_pal_mem_set(touch_data->object_data, 0x00, size);

	num_of_active_objects = false;

	idx = 0;
	offset = 0;
	objects = 0;
	active_objects = 0;
	active_only = false;
	obj = 0;
	next = 0;

	while (idx < config_size) {
		code = config_data[idx++];
		switch (code) {
		case TOUCH_REPORT_END:
			goto exit;
		case TOUCH_REPORT_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			active_only = true;
			break;
		case TOUCH_REPORT_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TOUCH_REPORT_FOREACH_END:
			end_of_foreach = idx;
			if (active_only) {
				if (num_of_active_objects) {
					objects++;
					obj++;
					if (objects < active_objects)
						idx = next;
				} else if (offset < report_size * 8) {
					obj++;
					idx = next;
				}
			} else {
				obj++;
				if (obj < tcm_dev->max_objects)
					idx = next;
			}
			break;
		case TOUCH_REPORT_PAD_TO_NEXT_BYTE:
			offset = syna_pal_ceil_div(offset, 8) * 8;
			break;
		case TOUCH_REPORT_TIMESTAMP:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get time-stamp\n");
				return retval;
			}
			touch_data->timestamp = data;
			offset += bits;
			break;
		case TOUCH_REPORT_OBJECT_N_INDEX:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get object index\n");
				return retval;
			}
			obj = data;
			touch_data->obji = data;
			offset += bits;
			break;
		case TOUCH_REPORT_OBJECT_N_CLASSIFICATION:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get object classification\n");
				return retval;
			}
			object_data[obj].status = (unsigned char)data;
			offset += bits;
			break;
		case TOUCH_REPORT_OBJECT_N_X_POSITION:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get object x position\n");
				return retval;
			}
			object_data[obj].x_pos = data;
			offset += bits;
			break;
		case TOUCH_REPORT_OBJECT_N_Y_POSITION:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get object y position\n");
				return retval;
			}
			object_data[obj].y_pos = data;
			offset += bits;
			break;
		case TOUCH_REPORT_OBJECT_N_Z:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get object z\n");
				return retval;
			}
			object_data[obj].z = data;
			offset += bits;
			break;
		case TOUCH_REPORT_OBJECT_N_X_WIDTH:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get object x width\n");
				return retval;
			}
			object_data[obj].x_width = data;
			offset += bits;
			break;
		case TOUCH_REPORT_OBJECT_N_Y_WIDTH:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get object y width\n");
				return retval;
			}
			object_data[obj].y_width = data;
			offset += bits;
			break;
		case TOUCH_REPORT_OBJECT_N_TX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get object tx position\n");
				return retval;
			}
			object_data[obj].tx_pos = data;
			offset += bits;
			break;
		case TOUCH_REPORT_OBJECT_N_RX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get object rx position\n");
				return retval;
			}
			object_data[obj].rx_pos = data;
			offset += bits;
			break;
		case TOUCH_REPORT_NUM_OF_ACTIVE_OBJECTS:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get number of active objects\n");
				return retval;
			}
			active_objects = data;
			num_of_active_objects = true;
			touch_data->num_of_active_objects = data;
			offset += bits;
			if (touch_data->num_of_active_objects == 0) {
				if (end_of_foreach == 0)
					return 0;

				idx = end_of_foreach;
			}
			break;
		case TOUCH_REPORT_0D_BUTTONS_STATE:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get 0D buttons state\n");
				return retval;
			}
			touch_data->buttons_state = data;
			offset += bits;
			break;
		case TOUCH_REPORT_GESTURE_ID:
			if (tcm_dev->cb_custom_gesture) {
				retval = tcm_dev->cb_custom_gesture(
					TOUCH_REPORT_GESTURE_ID, config_data,
					&idx, report, &offset, report_size,
					tcm_dev->cbdata_gesture);
			} else {
				bits = config_data[idx++];
				retval = syna_tcm_get_touch_data(report,
					report_size, offset, bits, &data);
				touch_data->gesture_id = data;
				offset += bits;
				if (touch_data->gesture_id) {
					LOGI("Gesture ID=%d, pwr_state = %d, sub_pwr_state = %d\n",
						touch_data->gesture_id, tcm->pwr_state, tcm->sub_pwr_state);
					if ((tcm->pwr_state == PWR_ON ||tcm->sub_pwr_state <= SUB_PWR_RESUME_DONE) &&
						(touch_data->gesture_id != TOUCH_HOLD_DOWN &&
						touch_data->gesture_id != TOUCH_HOLD_UP)) {
						if (tcm->health_monitor_support) {
							tp_healthinfo_report(&tcm->monitor_data, HEALTH_REPORT, "report_gesture_event_in_resume_cnt");
						}
						LOGE("unexpected gesture id report in resume state\n");
					}
				}
				if (touch_data->gesture_id == 3) {
					LOGE("debug syna data: \n");
					if (report_size > 0) {
						LOGE("report buf:%*ph\n", report_size, report);
					}
				}
			}
			if (retval < 0) {
				LOGE("Fail to get gesture id\n");
				return retval;
			}
			break;
		case TOUCH_REPORT_GESTURE_DATA:
		/*case TOUCH_REPORT_CUSTOM_GESTURE_INFO:*/
			if (tcm_dev->cb_custom_gesture) {
				retval = tcm_dev->cb_custom_gesture(
					TOUCH_REPORT_GESTURE_DATA, config_data,
					&idx, report, &offset, report_size,
					tcm_dev->cbdata_gesture);
			} else {
				bits = config_data[idx++];
				retval = syna_tcm_get_gesture_data(report,
						report_size,
						offset, bits,
						&touch_data->gesture_data,
						touch_data->gesture_id);
				offset += bits;
			}
			if (retval < 0) {
				LOGE("Fail to get gesture data\n");
				return retval;
			}
			break;
		case TOUCH_REPORT_CUSTOM_GESTURE_INFO:
			bits = config_data[idx++];
			retval = syna_tcm_get_report_data(report, report_size, offset, bits,
						      (unsigned int *)(&touch_data->extra_gesture_info[0]));

			if (retval < 0) {
				LOGE("Failed to get gesture double tap\n");
				return retval;
			}

			offset += bits;
			break;

		case TOUCH_REPORT_CUSTOM_GESTURE_COORDINATE:
			bits = config_data[idx++];
			retval = syna_tcm_get_report_data(report, report_size, offset, bits,
						      (unsigned int *)(&touch_data->data_point[0]));

			if (retval < 0) {
				LOGE("Failed to get gesture double tap\n");
				return retval;
			}

			offset += bits;
			break;
		case TOUCH_REPORT_FRAME_RATE:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get frame rate\n");
				return retval;
			}
			touch_data->frame_rate = data;
			offset += bits;
			break;
		case TOUCH_REPORT_FORCE_MEASUREMENT:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get force measurement data\n");
				return retval;
			}
			touch_data->force_data = data;
			offset += bits;
			break;
		case TOUCH_REPORT_FINGERPRINT_AREA_MEET:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get data for fingerprint area\n");
				return retval;
			}
			touch_data->fingerprint_area_meet = data;
			offset += bits;
			break;
		case TOUCH_REPORT_POWER_IM:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get power IM\n");
				return retval;
			}
			touch_data->power_im = data;
			offset += bits;
			break;
		case TOUCH_REPORT_CID_IM:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get CID IM\n");
				return retval;
			}
			touch_data->cid_im = data;
			offset += bits;
			break;
		case TOUCH_REPORT_RAIL_IM:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get rail IM\n");
				return retval;
			}
			touch_data->rail_im = data;
			offset += bits;
			break;
		case TOUCH_REPORT_CID_VARIANCE_IM:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get CID variance IM\n");
				return retval;
			}
			touch_data->cid_variance_im = data;
			offset += bits;
			break;
		case TOUCH_REPORT_NSM_FREQUENCY_INDEX:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get NSM frequency\n");
				return retval;
			}
			touch_data->nsm_frequency = data;
			offset += bits;
			break;
		case TOUCH_REPORT_NSM_STATE:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get NSM state\n");
				return retval;
			}
			touch_data->nsm_state = data;
			offset += bits;
			break;
		case TOUCH_REPORT_CPU_CYCLES_USED_SINCE_LAST_FRAME:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get cpu cycles info\n");
				return retval;
			}
			touch_data->num_of_cpu_cycles = data;
			offset += bits;
			break;
		case TOUCH_REPORT_FACE_DETECT:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to detect face\n");
				return retval;
			}
			touch_data->fd_data = data;
			offset += bits;
			break;
		case TOUCH_REPORT_SENSING_MODE:
			bits = config_data[idx++];
			retval = syna_tcm_get_touch_data(report, report_size,
					offset, bits, &data);
			if (retval < 0) {
				LOGE("Fail to get sensing mode\n");
				return retval;
			}
			touch_data->sensing_mode = data;
			offset += bits;
			break;
		case TOUCH_REPORT_CUSTOM_GRIP_INFO:
			bits = config_data[idx++];
			retval = syna_tcm_get_custome_grip_info(report,
					report_size,
					offset, bits,
					&object_data[obj].grip_info);
			offset += bits;

			if (retval < 0) {
				LOGE("Fail to get custom gesture info\n");
				return retval;
			}
			break;
		default:
			/* use custom parsing method, if registered */
			if (tcm_dev->cb_custom_touch_entity) {
				retval = tcm_dev->cb_custom_touch_entity(
					code, config_data, &idx, report,
					&offset, report_size,
					tcm_dev->cbdata_touch_entity);
				if (retval >= 0)
					continue;
			}

			LOGW("Unknown touch config code:0x%02x (length:%d)\n",
				code, config_data[idx]);
			bits = config_data[idx++];
			offset += bits;
			break;
		}
	}

exit:
	return 0;
}

/**
 * syna_tcm_set_touch_report_config()
 *
 * Setup the format and content of touch report if needed.
 *
 * TouchComm allows to set how touch reports are formatted and what items get
 * reported each time a touch report is generated.
 *
 * @param
 *    [ in] tcm_dev:     the device handle
 *    [ in] config:      the customized report configuration
 *    [ in] config_size: size of given config
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_touch_report_config(struct tcm_dev *tcm_dev,
		unsigned char *config, unsigned int config_size)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int size;
	struct tcm_application_info *app_info;
	unsigned char *data;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if ((!config) || (config_size == 0)) {
		LOGE("Invalid given config data\n");
		return _EINVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Not in application fw mode, mode: %d\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}

	app_info = &tcm_dev->app_info;
	size = syna_pal_le2_to_uint(app_info->max_touch_report_config_size);

	if (config_size > size) {
		LOGE("Invalid config size: %d (max: %d)\n", config_size, size);
		return _EINVAL;
	}

	data = syna_pal_mem_alloc(size, sizeof(unsigned char));
	if (!data) {
		LOGE("Fail to allocate memory for touch config setting\n");
		return _ENOMEM;
	}

	retval = syna_pal_mem_cpy(data,
			size,
			config,
			config_size,
			config_size);
	if (retval < 0) {
		LOGE("Fail to copy custom touch config\n");
		goto exit;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SET_TOUCH_REPORT_CONFIG,
			data,
			size,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to write command CMD_SET_TOUCH_REPORT_CONFIG\n");
		goto exit;
	}

	LOGI("Set touch config done\n");

exit:
	if (data)
		syna_pal_mem_free((void *)data);

	return retval;
}

/**
 * syna_tcm_preserve_touch_report_config()
 *
 * Retrieve and preserve the current touch report configuration.
 *
 * The retrieved configuration is stored in touch_config buffer defined
 * in structure syna_tcm_dev for later using of touch position parsing.
 *
 * The touch_config buffer will be allocated internally and its size will
 * be updated accordingly.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_preserve_touch_report_config(struct tcm_dev *tcm_dev)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int size = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Not in application fw mode, mode: %d\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_TOUCH_REPORT_CONFIG,
			NULL,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to write command CMD_GET_TOUCH_REPORT_CONFIG\n");
		goto exit;
	}

	syna_tcm_buf_lock(&tcm_dev->resp_buf);

	size = tcm_dev->resp_buf.data_length;
	retval = syna_tcm_buf_alloc(&tcm_dev->touch_config,
			size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal touch_config\n");
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}

	syna_tcm_buf_lock(&tcm_dev->touch_config);

	retval = syna_pal_mem_cpy(tcm_dev->touch_config.buf,
			tcm_dev->touch_config.buf_size,
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			size);
	if (retval < 0) {
		LOGE("Fail to clone touch config\n");
		syna_tcm_buf_unlock(&tcm_dev->touch_config);
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}

	tcm_dev->touch_config.data_length = size;

	syna_tcm_buf_unlock(&tcm_dev->touch_config);
	syna_tcm_buf_unlock(&tcm_dev->resp_buf);

exit:
	return retval;
}

/**
 * syna_tcm_set_custom_touch_entity_callback()
 *
 * Set up callback function to handle custom touch data.
 *
 * Once finding the "new" custom entity in touch report, the core library will
 * invoke the custom parsing method to handle this "new" code entity.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] p_cb:     the callback function pointer
 *    [ in] p_cbdata: pointer to caller data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_custom_touch_entity_callback(struct tcm_dev *tcm_dev,
		tcm_custom_touch_entity_callback_t p_cb, void *p_cbdata)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	tcm_dev->cb_custom_touch_entity = p_cb;
	tcm_dev->cbdata_touch_entity = p_cbdata;

	LOGI("enabled\n");

	return 0;
}

/**
 * syna_tcm_set_custom_gesture_callback()
 *
 * Set up callback function to handle the gesture data defined as the
 * following code entities
 *        - TOUCH_REPORT_GESTURE_ID
 *        - TOUCH_REPORT_GESTURE_DATA
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] p_cb:     the callback function pointer
 *    [ in] p_cbdata: pointer to caller data passed to callback function
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_custom_gesture_callback(struct tcm_dev *tcm_dev,
		tcm_custom_gesture_callback_t p_cb, void *p_cbdata)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	tcm_dev->cb_custom_gesture = p_cb;
	tcm_dev->cbdata_gesture = p_cbdata;

	LOGI("enabled\n");

	return 0;
}

