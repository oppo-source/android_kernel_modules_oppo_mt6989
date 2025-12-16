// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/version.h>
#include "pw_iris_api.h"
#include "pw_iris_i2c.h"
#include "pw_iris_lightup.h"
#include "pw_iris_log.h"

#define IRIS_PURE_COMPATIBLE_NAME  "pixelworks,iris-i2c"
#define IRIS_PURE_I2C_DRIVER_NAME  "pixelworks-i2c"

/*iris i2c handle*/
struct i2c_client  *iris_pure_i2c_handle;

static void __iris_pure_i2c_buf_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	memset(pcfg->iris_pure_i2c_cfg.buf, 0x00, IRIS_I2C_BUF_LEN);
	pcfg->iris_pure_i2c_cfg.buf_index = 0;
}

static void __iris_pure_i2c_buf_reset(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->iris_pure_i2c_cfg.buf_index = 0;
}


static uint8_t *__iris_pure_i2c_alloc_buf(uint32_t byte_len)
{
	uint8_t *pbuf = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGV("%s: buf = %p, index = %d, byte_len = %d", __func__,
		pcfg->iris_pure_i2c_cfg.buf, pcfg->iris_pure_i2c_cfg.buf_index, byte_len);

	if ((pcfg->iris_pure_i2c_cfg.buf_index + byte_len) > IRIS_I2C_BUF_LEN) {
		IRIS_LOGE("%s(): can not alloc i2c buf\n", __func__);
		return NULL;
	}

	pbuf = pcfg->iris_pure_i2c_cfg.buf;
	pbuf += pcfg->iris_pure_i2c_cfg.buf_index;
	pcfg->iris_pure_i2c_cfg.buf_index += byte_len;

	IRIS_LOGV("%s: buf = %p, index = %d", __func__, pbuf, pcfg->iris_pure_i2c_cfg.buf_index);

	return pbuf;
}

int iris_pure_i2c_single_read(uint32_t addr, uint32_t *val)
{
	int ret = -1;
	uint8_t *w_data_list = NULL;
	uint8_t *r_data_list = NULL;
	struct i2c_msg msgs[2];
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_pure_i2c_handle || !val) {
		IRIS_LOGE("%s, %d: the parameter is not right\n", __func__, __LINE__);
		return -EINVAL;
	}

	memset(msgs, 0, 2 * sizeof(msgs[0]));

	w_data_list = kmalloc(5, GFP_KERNEL);
	if (!w_data_list) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	r_data_list = kmalloc(4, GFP_KERNEL);
	if (!r_data_list) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		kfree(w_data_list);
		return -ENOMEM;
	}

	w_data_list[0] = 0xcc;
	w_data_list[1] = (addr >> 0) & 0xff;
	w_data_list[2] = (addr >> 8) & 0xff;
	w_data_list[3] = (addr >> 16) & 0xff;
	w_data_list[4] = (addr >> 24) & 0xff;

	r_data_list[0] = 0x00;
	r_data_list[1] = 0x00;
	r_data_list[2] = 0x00;
	r_data_list[3] = 0x00;

	msgs[0].addr = (iris_pure_i2c_handle->addr & 0xff);
	msgs[0].flags = 0;
	msgs[0].buf = w_data_list;
	msgs[0].len = 5;

	msgs[1].addr = (iris_pure_i2c_handle->addr & 0xff);
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = r_data_list;
	msgs[1].len = 4;

	mutex_lock(&pcfg->i2c_read_mutex);
	ret = i2c_transfer(iris_pure_i2c_handle->adapter, &msgs[0], 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s, %d: i2c_transfer failed, write cmd, addr = 0x%08x, ret = %d\n",
			__func__, __LINE__, addr, ret);
	}

	if (ret == 0) {
		udelay(20);
		ret = i2c_transfer(iris_pure_i2c_handle->adapter, &msgs[1], 1);
		if (ret == 1) {
			ret = 0;
		} else {
			ret = ret < 0 ? ret : -EIO;
			IRIS_LOGE("%s, %d: i2c_transfer failed, read cmd, addr = 0x%08x, ret = %d\n",
				__func__, __LINE__, addr, ret);
		}
	}
	mutex_unlock(&pcfg->i2c_read_mutex);

	if (ret == 0) {
		*val = (r_data_list[0] << 24) |
			(r_data_list[1] << 16) |
			(r_data_list[2] << 8) |
			(r_data_list[3] << 0);
	}

	kfree(w_data_list);
	kfree(r_data_list);
	w_data_list = NULL;
	r_data_list = NULL;

	return ret;

}

int iris_pure_i2c_single_write(uint32_t addr, uint32_t val)
{

	int ret = 0;
	struct i2c_msg msg;
	uint8_t *data_list = NULL;

	if (!iris_pure_i2c_handle) {
		IRIS_LOGE("%s, %d: the parameter is not right\n", __func__, __LINE__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

	data_list = kmalloc(9, GFP_KERNEL);
	if (!data_list) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	data_list[0] = 0xcc;
	data_list[1] = (addr >> 0) & 0xff;
	data_list[2] = (addr >> 8) & 0xff;
	data_list[3] = (addr >> 16) & 0xff;
	data_list[4] = (addr >> 24) & 0xff;
	data_list[5] = (val >> 24) & 0xff;
	data_list[6] = (val >> 16) & 0xff;
	data_list[7] = (val >> 8) & 0xff;
	data_list[8] = (val >> 0) & 0xff;

	msg.addr = (iris_pure_i2c_handle->addr & 0xff);
	msg.flags = 0;
	msg.buf = data_list;
	msg.len = 9;

	ret = i2c_transfer(iris_pure_i2c_handle->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s, %d: i2c_transfer failed, write cmd, addr = 0x%08x, ret = %d\n",
			__func__, __LINE__, addr, ret);
	}

	kfree(data_list);
	data_list = NULL;
	return ret;
}

int iris_pure_i2c_mult_single_write(struct iris_i2c_msg *dsi_msg)
{
	int ret = 0;
	int i = 0;
	int pos = 0;
	struct i2c_msg *msgs;
	uint32_t addr, val;
	const int reg_len = 9;
	uint8_t *data = NULL;
	uint8_t *data_list = NULL;
	uint32_t *pval = (uint32_t *)dsi_msg->buf;
	/*f4 need to one value one address */
	uint32_t sum = dsi_msg->len >> 3;

	if (!iris_pure_i2c_handle) {
		IRIS_LOGE("%s, %d: the parameter is not right\n", __func__, __LINE__);
		return -EINVAL;
	}

	data_list = kmalloc(9 * sum, GFP_KERNEL);
	if (!data_list) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	msgs = kmalloc_array(sum, sizeof(struct i2c_msg), GFP_KERNEL);
	if (!msgs)
		goto FREE_BUFFER;
	memset(msgs, 0x00, sum * sizeof(struct i2c_msg));

	for (i = 0; i < sum; i++) {
		pos = reg_len *  i;
		addr = pval[2 * i];
		val = pval[2 * i + 1];

		data_list[pos] = 0xcc;
		data_list[pos + 1] = (addr >> 0) & 0xff;
		data_list[pos + 2] = (addr >> 8) & 0xff;
		data_list[pos + 3] = (addr >> 16) & 0xff;
		data_list[pos + 4] = (addr >> 24) & 0xff;
		data_list[pos + 5] = (val >> 24) & 0xff;
		data_list[pos + 6] = (val >> 16) & 0xff;
		data_list[pos + 7] = (val >> 8) & 0xff;
		data_list[pos + 8] = (val >> 0) & 0xff;

		data = &data_list[pos];

		msgs[i].addr = (iris_pure_i2c_handle->addr & 0xff);
		msgs[i].flags = 0;
		msgs[i].buf = data;
		msgs[i].len = reg_len;
		IRIS_LOGD("%s addr:%x data:%x", __func__, addr, val);
	}

	ret = i2c_transfer(iris_pure_i2c_handle->adapter, msgs, sum);
	if (ret == sum) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s, %d: i2c_transfer failed, write cmd, addr = 0x%08x, ret = %d\n",
			__func__, __LINE__, addr, ret);
	}

	kfree(msgs);
	msgs = NULL;
FREE_BUFFER:
	kfree(data_list);
	data_list = NULL;
	return ret;
}

int iris_pure_i2c_burst_write(uint32_t addr, uint32_t *val, uint16_t reg_num)
{

	int i;
	int ret = -1;
	u32 msg_len = 0;
	struct i2c_msg msg;
	uint8_t *data_list = NULL;

	if (!val || reg_num < 1 || !iris_pure_i2c_handle) {
		IRIS_LOGE("%s, %d: the parameter is not right\n", __func__, __LINE__);
		return -EINVAL;
	}

	memset(&msg, 0x00, sizeof(msg));

	msg_len = 5 + reg_num * 4;

	data_list = kmalloc(msg_len, GFP_KERNEL);
	if (data_list == NULL) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	data_list[0] = 0xfc;
	data_list[1] = (addr >> 0) & 0xff;
	data_list[2] = (addr >> 8) & 0xff;
	data_list[3] = (addr >> 16) & 0xff;
	data_list[4] = (addr >> 24) & 0xff;

	for (i = 0; i < reg_num; i++) {
		data_list[i*4 + 5] = (val[i] >> 24) & 0xff;
		data_list[i*4 + 6] = (val[i] >> 16) & 0xff;
		data_list[i*4 + 7] = (val[i] >> 8) & 0xff;
		data_list[i*4 + 8] = (val[i] >> 0) & 0xff;
	}

	msg.addr = (iris_pure_i2c_handle->addr & 0xff);
	msg.flags = 0;
	msg.buf = data_list;
	msg.len = msg_len;

	ret = i2c_transfer(iris_pure_i2c_handle->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s, %d: i2c_transfer failed, write cmd, addr = 0x%08x, ret = %d\n",
			__func__, __LINE__, addr, ret);
	}

	kfree(data_list);
	data_list = NULL;
	return ret;

}

int iris_pure_i2c_multi_write(struct iris_i2c_msg *dsi_msg, uint32_t msg_num)
{
	int ret = -1;
	uint32_t i = 0;
	uint32_t byte_count = 0;
	uint32_t total_len = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct i2c_msg *i2c_msg = NULL;

	if ((dsi_msg == NULL) || (msg_num == 0)) {
		IRIS_LOGE("%s, %d: pbuf is NULL or num = 0\n", __func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < msg_num; i++) {
		if (dsi_msg[i].len > I2C_MSG_MAX_LEN) {
			IRIS_LOGE("%s: msg len exceed max i2c xfer len\n", __func__);
			return -EINVAL;
		}

		total_len += dsi_msg[i].len;
		if (total_len > IRIS_I2C_BUF_LEN) {
			IRIS_LOGE("%s: total len exceed max i2c buf len\n", __func__);
			return -EINVAL;
		}
	}

	i2c_msg = vmalloc(sizeof(struct i2c_msg) * msg_num);
	if (!i2c_msg) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	__iris_pure_i2c_buf_reset();
	for (i = 0; i < msg_num; i++) {
		uint32_t *payload = NULL;
		ret = -EINVAL;
		byte_count = dsi_msg[i].len;
		i2c_msg[i].buf = __iris_pure_i2c_alloc_buf(byte_count);
		if (!i2c_msg[i].buf) {
			IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
			ret = -ENOMEM;
			goto error;
		}
		memcpy(i2c_msg[i].buf, dsi_msg[i].buf, byte_count);

		if (!pcfg->pw_chip_func_ops.iris_convert_dsi_to_i2c)
			goto error;

		ret = pcfg->pw_chip_func_ops.iris_convert_dsi_to_i2c(i2c_msg[i].buf);
		payload = (uint32_t *)i2c_msg[i].buf;
		if (ret == I2C_MULT_CONT_ADDR_VAL)
			ret = iris_pure_i2c_burst_write(payload[1], payload + 2, (byte_count - 8) >> 2);
		else if (ret == I2C_MULT_ADDR_VAL_PAIR) {
			struct iris_i2c_msg msg = {
				.buf = dsi_msg[i].buf + 4,
				.len = byte_count - 4,
			};
			ret = iris_pure_i2c_mult_single_write(&msg);
		}
	}
error:
	__iris_pure_i2c_buf_reset();
	vfree(i2c_msg);
	i2c_msg = NULL;
	return ret;
}

static int iris_pure_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;

	iris_pure_i2c_handle = client;
	pcfg->iris_pure_i2c_cfg.buf = kmalloc(IRIS_I2C_BUF_LEN, GFP_KERNEL);
	if (!pcfg->iris_pure_i2c_cfg.buf) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}

	__iris_pure_i2c_buf_init();
	IRIS_LOGI("%s,%d: %p %p\n", __func__, __LINE__, iris_pure_i2c_handle,
			pcfg->iris_pure_i2c_cfg.buf);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void iris_pure_i2c_remove(struct i2c_client *client)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_pure_i2c_handle = NULL;
	kfree(pcfg->iris_pure_i2c_cfg.buf);
	pcfg->iris_pure_i2c_cfg.buf = NULL;
	pcfg->iris_pure_i2c_cfg.buf_index = 0;
}
#else
static int iris_pure_i2c_remove(struct i2c_client *client)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_pure_i2c_handle = NULL;
	kfree(pcfg->iris_pure_i2c_cfg.buf);
	pcfg->iris_pure_i2c_cfg.buf = NULL;
	pcfg->iris_pure_i2c_cfg.buf_index = 0;
	return 0;
}
#endif

static const struct i2c_device_id iris_pure_i2c_id_table[] = {
	{IRIS_PURE_I2C_DRIVER_NAME, 0},
	{},
};


static const struct of_device_id iris_pure_match_table[] = {
	{.compatible = IRIS_PURE_COMPATIBLE_NAME,},
	{ },
};

static struct i2c_driver plx_pure_i2c_driver = {
	.driver = {
		.name = IRIS_PURE_I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = iris_pure_match_table,
	},
	.probe = iris_pure_i2c_probe,
	.remove =  iris_pure_i2c_remove,
	.id_table = iris_pure_i2c_id_table,
};


int iris_pure_i2c_bus_init(void)
{
	int ret;

	IRIS_LOGD("%s()\n", __func__);
	iris_pure_i2c_handle = NULL;
	ret = i2c_add_driver(&plx_pure_i2c_driver);
	if (ret != 0)
		IRIS_LOGE("iris pure i2c add driver fail: %d\n", ret);
	return 0;
}

void iris_pure_i2c_bus_exit(void)
{

	i2c_del_driver(&plx_pure_i2c_driver);
	iris_pure_i2c_remove(iris_pure_i2c_handle);
}
