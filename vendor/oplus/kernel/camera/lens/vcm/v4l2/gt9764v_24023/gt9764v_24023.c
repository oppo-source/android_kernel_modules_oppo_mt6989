// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define GT9764V_NAME				"gt9764v_24023"
#define GT9764V_MAX_FOCUS_POS			1023
#define GT9764V_ORIGIN_FOCUS_POS			512
#define GT9764V_MOVE_STEPS			  	50
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define GT9764V_FOCUS_STEPS			1
#define GT9764V_CONTROL_REG			0x02
#define GT9764V_SET_POSITION_ADDR		0x03
#define GT9764V_STATUS_REG				0x05

#define GT9764V_CONTROL_POWER_DOWN		BIT(0)
#define GT9764V_AAC_MODE_EN			BIT(1)

#define GT9764V_CMD_DELAY			0xff
#define GT9764V_CTRL_DELAY_US			5000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */

#define GT9764V_MOVE_DELAY_US      1000
#define GT9764V_INIT_DELAY_US      1000
#define GT9764V_SET_VOLTAGE        2800000

/* gt9764v device structure */
struct gt9764v_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
};

static inline struct gt9764v_device *to_gt9764v_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct gt9764v_device, ctrls);
}

static inline struct gt9764v_device *sd_to_gt9764v_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct gt9764v_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char delay;
};

static struct regval_list gt9764v_init_regs[] = {
    {0x02, 0x00, 2},
    {0x02, 0x02, 2},
    {0x06, 0x40, 2},
    {0x07, 0x6F, 2},
    {0x0B, 0x20, 2},
};

static int gt9764v_write_smbus(struct gt9764v_device *gt9764v, unsigned char reg,
			      unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gt9764v->sd);
	int ret = 0;

	if (reg == GT9764V_CMD_DELAY  && value == GT9764V_CMD_DELAY)
		usleep_range(GT9764V_CTRL_DELAY_US,
			     GT9764V_CTRL_DELAY_US + 100);
	else
		ret = i2c_smbus_write_byte_data(client, reg, value);
	return ret;
}

static int gt9764v_write_array(struct gt9764v_device *gt9764v,
			      struct regval_list *vals, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		pr_info("Init write [%d, %d]", vals[i].reg_num, vals[i].value);
		ret = gt9764v_write_smbus(gt9764v, vals[i].reg_num,
					 vals[i].value);
		if (ret < 0)
			return ret;

		usleep_range(vals[i].delay * GT9764V_INIT_DELAY_US,
						vals[i].delay * GT9764V_INIT_DELAY_US + 1000);
	}
	return 0;
}

static int gt9764v_set_position(struct gt9764v_device *gt9764v, u16 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&gt9764v->sd);
    int loop_time = 0, status = 0;
    pr_info("gt9764v set position is %d", val);
    /*wait for I2C bus idle*/
    while (loop_time < 20) {
        status = i2c_smbus_read_byte_data(client, GT9764V_STATUS_REG);
        status = status & 0x01;//get reg 05 status
        pr_debug("gt9764v 0x05 status:%x", status);
        if(status == 0){
            break;
        }
        loop_time++;
        usleep_range(GT9764V_MOVE_DELAY_US, GT9764V_MOVE_DELAY_US + 100);
    }
    if (loop_time >= 20)
    {
        pr_debug("waiting 0x05 flag timeout!");
    }
	return i2c_smbus_write_word_data(client, GT9764V_SET_POSITION_ADDR,
					 swab16(val << 1));
}

static int gt9764v_release(struct gt9764v_device *gt9764v)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&gt9764v->sd);

	diff_dac = GT9764V_ORIGIN_FOCUS_POS - gt9764v->focus->val;

	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		GT9764V_MOVE_STEPS;

	val = gt9764v->focus->val;

	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (GT9764V_MOVE_STEPS*(-1)) : GT9764V_MOVE_STEPS);
		ret = gt9764v_set_position(gt9764v, val);
		if (ret) {
			pr_info("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(GT9764V_MOVE_DELAY_US,
			     GT9764V_MOVE_DELAY_US + 1000);
	}

	// last step to origin
	ret = gt9764v_set_position(gt9764v, GT9764V_ORIGIN_FOCUS_POS);
	if (ret) {
		pr_info("%s I2C failure: %d",
				__func__, ret);
		return ret;
	}

    ret = i2c_smbus_write_byte_data(client, GT9764V_CONTROL_REG,
                    GT9764V_CONTROL_POWER_DOWN);
    if (ret)
        return ret;

    pr_info("%s -\n", __func__);
    return 0;
}

static int gt9764v_init(struct gt9764v_device *gt9764v)
{
    int ret, val;
    pr_info("%s +\n", __func__);

    ret = gt9764v_write_array(gt9764v, gt9764v_init_regs,
                    ARRAY_SIZE(gt9764v_init_regs));
	if (ret)
		return ret;

	val = GT9764V_ORIGIN_FOCUS_POS;
    ret = gt9764v_set_position(gt9764v, val);
	if (ret)
		return ret;

	pr_info("%s -\n", __func__);
    return 0;
}

/* Power handling */
static int gt9764v_power_off(struct gt9764v_device *gt9764v)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = gt9764v_release(gt9764v);
	if (ret)
		pr_info("gt9764v release failed!\n");

	ret = regulator_disable(gt9764v->vin);
	if (ret)
		return ret;

	ret = regulator_disable(gt9764v->vdd);
	if (ret)
		return ret;

	if (gt9764v->vcamaf_pinctrl && gt9764v->vcamaf_off)
		ret = pinctrl_select_state(gt9764v->vcamaf_pinctrl,
					gt9764v->vcamaf_off);

	return ret;
}

static int gt9764v_power_on(struct gt9764v_device *gt9764v)
{
	int ret;

	pr_info("%s\n", __func__);

	regulator_set_voltage(gt9764v->vin, GT9764V_SET_VOLTAGE, GT9764V_SET_VOLTAGE);
	ret = regulator_enable(gt9764v->vin);
	if (ret < 0)
		return ret;

	if (gt9764v->vcamaf_pinctrl && gt9764v->vcamaf_on)
		ret = pinctrl_select_state(gt9764v->vcamaf_pinctrl,
					gt9764v->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(GT9764V_CTRL_DELAY_US, GT9764V_CTRL_DELAY_US + 100);

	ret = gt9764v_init(gt9764v);
	if (ret < 0)
		goto fail;

	return 0;

fail:
	regulator_disable(gt9764v->vin);
	regulator_disable(gt9764v->vdd);
	if (gt9764v->vcamaf_pinctrl && gt9764v->vcamaf_off) {
		pinctrl_select_state(gt9764v->vcamaf_pinctrl,
				gt9764v->vcamaf_off);
	}

	return ret;
}

static int gt9764v_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct gt9764v_device *gt9764v = to_gt9764v_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		ret = gt9764v_set_position(gt9764v, ctrl->val);
		if (ret) {
			pr_info("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops gt9764v_vcm_ctrl_ops = {
	.s_ctrl = gt9764v_set_ctrl,
};

static int gt9764v_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct gt9764v_device *gt9764v = sd_to_gt9764v_vcm(sd);

	pr_info("%s\n", __func__);

	ret = gt9764v_power_on(gt9764v);
	if (ret < 0) {
		pr_info("%s power on fail, ret = %d",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int gt9764v_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gt9764v_device *gt9764v = sd_to_gt9764v_vcm(sd);

	pr_info("%s\n", __func__);

	gt9764v_power_off(gt9764v);

	return 0;
}

static const struct v4l2_subdev_internal_ops gt9764v_int_ops = {
	.open = gt9764v_open,
	.close = gt9764v_close,
};

static const struct v4l2_subdev_ops gt9764v_ops = { };

static void gt9764v_subdev_cleanup(struct gt9764v_device *gt9764v)
{
	v4l2_async_unregister_subdev(&gt9764v->sd);
	v4l2_ctrl_handler_free(&gt9764v->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&gt9764v->sd.entity);
#endif
}

static int gt9764v_init_controls(struct gt9764v_device *gt9764v)
{
	struct v4l2_ctrl_handler *hdl = &gt9764v->ctrls;
	const struct v4l2_ctrl_ops *ops = &gt9764v_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	gt9764v->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, GT9764V_MAX_FOCUS_POS, GT9764V_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	gt9764v->sd.ctrl_handler = hdl;

	return 0;
}

static int gt9764v_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gt9764v_device *gt9764v;
	int ret;

	pr_info("%s\n", __func__);

	gt9764v = devm_kzalloc(dev, sizeof(*gt9764v), GFP_KERNEL);
	if (!gt9764v)
		return -ENOMEM;

	gt9764v->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(gt9764v->vin)) {
		ret = PTR_ERR(gt9764v->vin);
		if (ret != -EPROBE_DEFER)
			pr_info("cannot get vin regulator\n");
		return ret;
	}

	gt9764v->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(gt9764v->vdd)) {
		ret = PTR_ERR(gt9764v->vdd);
		if (ret != -EPROBE_DEFER)
			pr_info("cannot get vdd regulator\n");
		return ret;
	}

	gt9764v->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(gt9764v->vcamaf_pinctrl)) {
		ret = PTR_ERR(gt9764v->vcamaf_pinctrl);
		gt9764v->vcamaf_pinctrl = NULL;
		pr_info("cannot get pinctrl\n");
	} else {
		gt9764v->vcamaf_on = pinctrl_lookup_state(
			gt9764v->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(gt9764v->vcamaf_on)) {
			ret = PTR_ERR(gt9764v->vcamaf_on);
			gt9764v->vcamaf_on = NULL;
			pr_info("cannot get vcamaf_on pinctrl\n");
		}

		gt9764v->vcamaf_off = pinctrl_lookup_state(
			gt9764v->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(gt9764v->vcamaf_off)) {
			ret = PTR_ERR(gt9764v->vcamaf_off);
			gt9764v->vcamaf_off = NULL;
			pr_info("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&gt9764v->sd, client, &gt9764v_ops);
	gt9764v->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gt9764v->sd.internal_ops = &gt9764v_int_ops;

	ret = gt9764v_init_controls(gt9764v);
	if (ret)
		goto err_cleanup;

#if defined(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&gt9764v->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	gt9764v->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&gt9764v->sd);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	gt9764v_subdev_cleanup(gt9764v);
	return ret;
}

static void gt9764v_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gt9764v_device *gt9764v = sd_to_gt9764v_vcm(sd);

	pr_info("%s\n", __func__);

	gt9764v_subdev_cleanup(gt9764v);

	// return 0;
}

static const struct i2c_device_id gt9764v_id_table[] = {
	{ GT9764V_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, gt9764v_id_table);

static const struct of_device_id gt9764v_of_table[] = {
	{ .compatible = "oplus,gt9764v_24023" },
	{ },
};
MODULE_DEVICE_TABLE(of, gt9764v_of_table);

static struct i2c_driver gt9764v_i2c_driver = {
	.driver = {
		.name = GT9764V_NAME,
		.of_match_table = gt9764v_of_table,
	},
	.probe_new  = gt9764v_probe,
	.remove = gt9764v_remove,
	.id_table = gt9764v_id_table,
};

module_i2c_driver(gt9764v_i2c_driver);

MODULE_AUTHOR("Dongchun Zhu <dongchun.zhu@mediatek.com>");
MODULE_DESCRIPTION("GT9764V VCM driver");
MODULE_LICENSE("GPL v2");
