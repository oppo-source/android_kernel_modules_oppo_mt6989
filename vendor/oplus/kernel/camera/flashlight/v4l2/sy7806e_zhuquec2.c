// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021 Oplus

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/pm_runtime.h>

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
#include "flashlight-core.h"
#include <linux/power_supply.h>
#include <soc/oplus/system/oplus_project.h>
#endif

#define SY7806E_ZHUQUEC2_NAME	"sy7806e_zhuquec2"
#define SY7806E_2_ZHUQUEC2_NAME	"sy7806e_2_zhuquec2"

/* registers definitions */
#define REG_ENABLE		0x01
#define REG_LED0_FLASH_BR	0x03
#define REG_LED1_FLASH_BR	0x04
#define REG_LED0_TORCH_BR	0x05
#define REG_LED1_TORCH_BR	0x06
#define REG_SOFT_RST		0x07
#define REG_FLASH_TOUT		0x08
#define REG_FLAG1		0x0A
#define REG_FLAG2		0x0B
#define REG_DEVICE_ID   0x0C
#define PCB_T0   0x10

/* fault mask */
#define FAULT_TIMEOUT	(1<<0)
#define FAULT_THERMAL_SHUTDOWN	(1<<2)
#define FAULT_LED0_SHORT_CIRCUIT	(1<<5)
#define FAULT_LED1_SHORT_CIRCUIT	(1<<4)

/*  FLASH Brightness
 *	min 11.72 mA, step 11.72 mA, max 1500 mA
 */
#define SY7806E_ZHUQUEC2_FLASH_BRT_MIN 11720
#define SY7806E_ZHUQUEC2_FLASH_BRT_STEP 11720
#define SY7806E_ZHUQUEC2_FLASH_BRT_MAX 1500000
#define SY7806E_ZHUQUEC2_FLASH_BRT_uA_TO_REG(a)	\
	((a) < SY7806E_ZHUQUEC2_FLASH_BRT_MIN ? 0 :	\
	 (((a) - SY7806E_ZHUQUEC2_FLASH_BRT_MIN) / SY7806E_ZHUQUEC2_FLASH_BRT_STEP))
#define SY7806E_ZHUQUEC2_FLASH_BRT_REG_TO_uA(a)		\
	((a) * SY7806E_ZHUQUEC2_FLASH_BRT_STEP + SY7806E_ZHUQUEC2_FLASH_BRT_MIN)

/*  FLASH TIMEOUT DURATION
 *	min 40ms, step 40ms,200ms, max 1600ms
 */
#define SY7806E_ZHUQUEC2_FLASH_TOUT_MIN 40
#define SY7806E_ZHUQUEC2_FLASH_TOUT_STEP 200
#define SY7806E_ZHUQUEC2_FLASH_TOUT_MAX 1600

/*  TORCH BRT
 *	min 1.8 mA, step 2.8 mA, max 357 mA
 */
#define SY7806E_ZHUQUEC2_TORCH_BRT_MIN 1800
#define SY7806E_ZHUQUEC2_TORCH_BRT_STEP 2800
#define SY7806E_ZHUQUEC2_TORCH_BRT_MAX 357000
#define SY7806E_ZHUQUEC2_TORCH_BRT_uA_TO_REG(a)	\
	((a) < SY7806E_ZHUQUEC2_TORCH_BRT_MIN ? 0 :	\
	 (((a) - SY7806E_ZHUQUEC2_TORCH_BRT_MIN) / SY7806E_ZHUQUEC2_TORCH_BRT_STEP))
#define SY7806E_ZHUQUEC2_TORCH_BRT_REG_TO_uA(a)		\
	((a) * SY7806E_ZHUQUEC2_TORCH_BRT_STEP + SY7806E_ZHUQUEC2_TORCH_BRT_MIN)

enum sy7806e_zhuquec2_led_id {
	SY7806E_ZHUQUEC2_LED0 = 0,
	SY7806E_ZHUQUEC2_LED1,
	SY7806E_ZHUQUEC2_LED_MAX
};

enum v4l2_flash_led_nums {
	SY7806E_ZHUQUEC2_CONTROL_LED0 = 2,
	SY7806E_ZHUQUEC2_CONTROL_LED1,
	SY7806E_ZHUQUEC2_CONTROL_LED2,
};

/* struct sy7806e_zhuquec2_platform_data
 *
 * @max_flash_timeout: flash timeout
 * @max_flash_brt: flash mode led brightness
 * @max_torch_brt: torch mode led brightness
 */
struct sy7806e_zhuquec2_platform_data {
	u32 max_flash_timeout;
	u32 max_flash_brt[SY7806E_ZHUQUEC2_LED_MAX];
	u32 max_torch_brt[SY7806E_ZHUQUEC2_LED_MAX];
};

enum led_enable {
	MODE_SHDN = 0x0,
	MODE_TORCH = 0x08,
	MODE_FLASH = 0x0C,
};

/**
 * struct sy7806e_zhuquec2_flash
 *
 * @dev: pointer to &struct device
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: V4L2 LED mode
 * @ctrls_led: V4L2 controls
 * @subdev_led: V4L2 subdev
 */
struct sy7806e_zhuquec2_flash {
	struct device *dev;
	struct sy7806e_zhuquec2_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	enum v4l2_flash_led_mode led_mode;
	struct v4l2_ctrl_handler ctrls_led[SY7806E_ZHUQUEC2_LED_MAX];
	struct v4l2_subdev subdev_led[SY7806E_ZHUQUEC2_LED_MAX];
	struct device_node *dnode[SY7806E_ZHUQUEC2_LED_MAX];
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
	struct flashlight_device_id flash_dev_id[SY7806E_ZHUQUEC2_LED_MAX];
#endif
};

struct sy7806e_zhuquec2_current_val {
	unsigned int flash_val;
	unsigned int flash_val2;
	unsigned int torch_val;
	unsigned int torch_val2;
};
static struct sy7806e_zhuquec2_current_val sy7806e_flash_current_val = {0x17, 0x17, 0x17, 0x17};
static struct sy7806e_zhuquec2_current_val sy7806e_2_flash_current_val = {0x17, 0x17, 0x17, 0x17};

/* define usage count */
static int use_count;
static unsigned int flash_device_id;
static int Pcb_Version = 0; //hw board id
static int led_index = 0b101; // led2 and led0 is floodlight,led1 is spotlight
static bool is_register = false;
static int flash_led_mode = 0b111; //LedIndex,0b111-LED2LED1LED0 three led light up

static struct sy7806e_zhuquec2_flash *sy7806e_zhuquec2_flash_data = NULL;
static struct sy7806e_zhuquec2_flash *sy7806e_2_zhuquec2_flash_data = NULL;

#define to_sy7806e_zhuquec2_flash(_ctrl, _no)	\
	container_of(_ctrl->handler, struct sy7806e_zhuquec2_flash, ctrls_led[_no])

/* define pinctrl */
struct pinctrl *sy7806e_zhuquec2_hwen_pinctrl;
struct pinctrl_state *sy7806e_zhuquec2_hwen_high;
struct pinctrl_state *sy7806e_zhuquec2_hwen_low;
#define SY7806E_ZHUQUEC2_PINCTRL_PIN_HWEN 0
#define SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_LOW 0
#define SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_HIGH 1
#define SY7806E_ZHUQUEC2_PINCTRL_STATE_HWEN_HIGH "hwen-high"
#define SY7806E_ZHUQUEC2_PINCTRL_STATE_HWEN_LOW  "hwen-low"
/******************************************************************************
 * Pinctrl configuration 
 *****************************************************************************/
static int sy7806e_zhuquec2_pinctrl_init(struct device *dev)
{
	int ret = 0;
	if (!dev || !dev->of_node)
		return -ENODEV;
	/* get pinctrl */
	sy7806e_zhuquec2_hwen_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(sy7806e_zhuquec2_hwen_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(sy7806e_zhuquec2_hwen_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	sy7806e_zhuquec2_hwen_high = pinctrl_lookup_state(
			sy7806e_zhuquec2_hwen_pinctrl,
			SY7806E_ZHUQUEC2_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(sy7806e_zhuquec2_hwen_high)) {
		pr_info("Failed to init (%s)\n",
			SY7806E_ZHUQUEC2_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(sy7806e_zhuquec2_hwen_high);
	}
	sy7806e_zhuquec2_hwen_low = pinctrl_lookup_state(
			sy7806e_zhuquec2_hwen_pinctrl,
			SY7806E_ZHUQUEC2_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(sy7806e_zhuquec2_hwen_low)) {
		pr_info("Failed to init (%s)\n", SY7806E_ZHUQUEC2_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(sy7806e_zhuquec2_hwen_low);
	}

	return ret;
}

static int sy7806e_zhuquec2_pinctrl_set(int pin, int state)
{
	int ret = 0;
	if (IS_ERR(sy7806e_zhuquec2_hwen_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case SY7806E_ZHUQUEC2_PINCTRL_PIN_HWEN:
		if (state == SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(sy7806e_zhuquec2_hwen_low))
			pinctrl_select_state(sy7806e_zhuquec2_hwen_pinctrl,
					sy7806e_zhuquec2_hwen_low);
		else if (state == SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(sy7806e_zhuquec2_hwen_high))
			pinctrl_select_state(sy7806e_zhuquec2_hwen_pinctrl,
					sy7806e_zhuquec2_hwen_high);
		else
			pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("pin(%d) state(%d)\n", pin, state);

	return ret;
}

/* enable mode control */
static int sy7806e_zhuquec2_mode_ctrl(struct sy7806e_zhuquec2_flash *flash)
{
	int rval = -EINVAL;

	pr_info("%s mode:%d", __func__, flash->led_mode);
	switch (flash->led_mode) {
	case V4L2_FLASH_LED_MODE_NONE:
		rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
					  REG_ENABLE, 0x0C, MODE_SHDN);
		rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
					  REG_ENABLE, 0x0C, MODE_SHDN);
		break;
	case V4L2_FLASH_LED_MODE_TORCH:
		rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
					  REG_ENABLE, 0x0C, MODE_TORCH);
		rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
					  REG_ENABLE, 0x0C, MODE_TORCH);
		break;
	case V4L2_FLASH_LED_MODE_FLASH:
		rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
					  REG_ENABLE, 0x0C, MODE_FLASH);
		rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
					  REG_ENABLE, 0x0C, MODE_FLASH);
		break;
	default:
		break;
	}
	return rval;
}

/* led1/2 enable/disable */
static int sy7806e_zhuquec2_enable_ctrl(struct sy7806e_zhuquec2_flash *flash,
			      enum sy7806e_zhuquec2_led_id led_no, bool on)
{
	int rval;

	pr_info("%s: enable:%d flash_led_mode:%d", __func__, on, flash_led_mode);
	if (on) {
		if (led_index == 0b011) {
			if (flash_led_mode == 0) {
				rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
						REG_ENABLE, 0x03, 0x03);
				rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
						REG_ENABLE, 0x03, 0x00);
			} else {
				rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
						REG_ENABLE, 0x01, flash_led_mode & 0x01);
				rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
						REG_ENABLE, 0x02, (flash_led_mode & 0x04) >> 1);
				rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
						REG_ENABLE, 0x01, (flash_led_mode & 0x02) >> 1);
			}
		} else {
			if (flash_led_mode == 0) {
				rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
							REG_ENABLE, 0x03, 0x01);
				rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
							REG_ENABLE, 0x03, 0x01);
			} else {
				rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
						REG_ENABLE, 0x03, flash_led_mode & 0x03);
				rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
						REG_ENABLE, 0x01, (flash_led_mode & 0x04) >> 2);
			}
		}
	} else {
		rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				REG_ENABLE, 0x03, 0x00);
		rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				REG_ENABLE, 0x03, 0x00);
	}
	pr_debug("%s: return val:%d", __func__,  rval);
	return rval;
}

static int sy7806e_zhuquec2_select_led(struct sy7806e_zhuquec2_flash *flash,
										int led_num)
{
	int rval;
	pr_info("engineer cam select led->%d", led_num);
	if (led_num == SY7806E_ZHUQUEC2_CONTROL_LED0) {
		rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap, REG_ENABLE, 0x03, 0x01);
		rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap, REG_ENABLE, 0x03, 0x00);
	} else if (led_num == SY7806E_ZHUQUEC2_CONTROL_LED1) {
		rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap, REG_ENABLE, 0x03, 0x02);
		rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap, REG_ENABLE, 0x03, 0x00);
	} else if (led_num == SY7806E_ZHUQUEC2_CONTROL_LED2) {
		rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap, REG_ENABLE, 0x03, 0x01);
		rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap, REG_ENABLE, 0x03, 0x00);
	} else {
		rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap, REG_ENABLE, 0x03, 0x00);
		rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap, REG_ENABLE, 0x03, 0x00);
	}
	 return rval;
}

/* torch1/2 brightness control */
static int sy7806e_zhuquec2_torch_brt_ctrl(struct sy7806e_zhuquec2_flash *flash,
				 enum sy7806e_zhuquec2_led_id led_no, unsigned int brt)
{
	int rval;
	u8 br_bits;

	pr_info("%s %d brt:%u", __func__, led_no, brt);
	if (brt < SY7806E_ZHUQUEC2_TORCH_BRT_MIN)
		return sy7806e_zhuquec2_enable_ctrl(flash, led_no, false);

	if (brt >= SY7806E_ZHUQUEC2_TORCH_BRT_MAX) {
		brt = SY7806E_ZHUQUEC2_TORCH_BRT_MAX;
		pr_info("%s limit to torch max %d", __func__, SY7806E_ZHUQUEC2_TORCH_BRT_MAX);
	}

	br_bits = SY7806E_ZHUQUEC2_TORCH_BRT_uA_TO_REG(brt);

	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				REG_LED0_TORCH_BR, 0xFF, br_bits);
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				REG_LED1_TORCH_BR, 0xFF, br_bits);
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				REG_LED0_TORCH_BR, 0xFF, br_bits);
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				REG_LED1_TORCH_BR, 0xFF, br_bits);

	pr_info("%s: return val:%d  br_bits = 0x%x", __func__,  rval, br_bits);
	return rval;
}

/* flash1/2 brightness control */
static int sy7806e_zhuquec2_flash_brt_ctrl(struct sy7806e_zhuquec2_flash *flash,
				 enum sy7806e_zhuquec2_led_id led_no, unsigned int brt)
{
	int rval;
	u8 br_bits;

	pr_info("%s %d brt:%u", __func__, led_no, brt);
	if (brt < SY7806E_ZHUQUEC2_FLASH_BRT_MIN)
		return sy7806e_zhuquec2_enable_ctrl(flash, led_no, false);

	if (brt >= SY7806E_ZHUQUEC2_FLASH_BRT_MAX) {
		brt = SY7806E_ZHUQUEC2_FLASH_BRT_MAX;
		pr_info("%s limit to flash max %d", __func__, SY7806E_ZHUQUEC2_FLASH_BRT_MAX);
	}

	br_bits = SY7806E_ZHUQUEC2_FLASH_BRT_uA_TO_REG(brt);


	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				  REG_LED0_FLASH_BR, 0xFF, br_bits);
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				  REG_LED1_FLASH_BR, 0xFF, br_bits);
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				  REG_LED0_FLASH_BR, 0xFF, br_bits);
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				  REG_LED1_FLASH_BR, 0xFF, br_bits);

	pr_info("%s: return val:%d  br_bits = 0x%x", __func__,  rval, br_bits);
	return rval;
}

/* flash1/2 timeout control */
static int sy7806e_zhuquec2_flash_tout_ctrl(struct sy7806e_zhuquec2_flash *flash,
				unsigned int tout)
{
	int rval;
	u8 tout_bits = 0x00;

	if (tout > 1600) {
		tout = 1600;
		pr_info("%s: limit to max tout = %d", __func__, tout);
	}

	if (tout <= 40) {
		tout_bits = 0x00;
	} else if (tout <= 400) {
		tout_bits = (tout - 40) / 40;
	} else if (tout <= 1600) {
		tout_bits = (tout - 400) / 200 + 0x09;
	}

	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				  REG_FLASH_TOUT, 0x0F, tout_bits);
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				  REG_FLASH_TOUT, 0x0F, tout_bits);

	pr_info("%s: tout = %d , tout_bits = 0x%x, ", __func__, tout, tout_bits);

	return rval;
}

/* v4l2 controls  */
static int sy7806e_zhuquec2_get_ctrl(struct v4l2_ctrl *ctrl, enum sy7806e_zhuquec2_led_id led_no)
{
	struct sy7806e_zhuquec2_flash *flash = to_sy7806e_zhuquec2_flash(ctrl, led_no);
	int rval = -EINVAL;

	mutex_lock(&flash->lock);

	if (ctrl->id == V4L2_CID_FLASH_FAULT) {
		s32 fault = 0;
		unsigned int reg_val = 0;
		unsigned int reg_val1 = 0;
		unsigned int reg_val2 = 0;

		rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_FLAG1, &reg_val1);
		if (rval < 0)
			goto out;
		rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_FLAG1, &reg_val2);
		if (rval < 0)
			goto out;
		reg_val = reg_val1 | reg_val2;
		if (reg_val & FAULT_LED0_SHORT_CIRCUIT)
			fault |= V4L2_FLASH_FAULT_SHORT_CIRCUIT;
		if (reg_val & FAULT_LED1_SHORT_CIRCUIT)
			fault |= V4L2_FLASH_FAULT_SHORT_CIRCUIT;
		if (reg_val & FAULT_THERMAL_SHUTDOWN)
			fault |= V4L2_FLASH_FAULT_OVER_TEMPERATURE;
		if (reg_val & FAULT_TIMEOUT)
			fault |= V4L2_FLASH_FAULT_TIMEOUT;
		ctrl->cur.val = fault;
	}

out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int sy7806e_zhuquec2_set_ctrl(struct v4l2_ctrl *ctrl, enum sy7806e_zhuquec2_led_id led_no)
{
	struct sy7806e_zhuquec2_flash *flash = to_sy7806e_zhuquec2_flash(ctrl, led_no);
	int rval = -EINVAL;

	unsigned int boost_reg_val;
	rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_FLAG2, &boost_reg_val);
	rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_FLAG2, &boost_reg_val);
	pr_info("%s: led_no:%d ctrID:0x%x, ctrlVal:0x%x, boost_reg_val:0x%x", __func__, led_no, ctrl->id, ctrl->val, boost_reg_val);
	mutex_lock(&flash->lock);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		flash->led_mode = ctrl->val;
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH)
			rval = sy7806e_zhuquec2_mode_ctrl(flash);
		else
			rval = 0;
		if (flash->led_mode == V4L2_FLASH_LED_MODE_NONE)
			sy7806e_zhuquec2_enable_ctrl(flash, led_no, false);
		else if (flash->led_mode == V4L2_FLASH_LED_MODE_TORCH)
			rval = sy7806e_zhuquec2_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_SOURCE:
		rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
					  REG_ENABLE, 0x20, (ctrl->val) << 5);
		if (rval < 0)
			goto err_out;
		rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
					  REG_ENABLE, 0x20, (ctrl->val) << 5);
		if (rval < 0)
			goto err_out;
		break;

	case V4L2_CID_FLASH_STROBE:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		flash->led_mode = V4L2_FLASH_LED_MODE_FLASH;
		rval = sy7806e_zhuquec2_mode_ctrl(flash);
		rval = sy7806e_zhuquec2_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		sy7806e_zhuquec2_enable_ctrl(flash, led_no, false);
		flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
		rval = sy7806e_zhuquec2_mode_ctrl(flash);
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		rval = sy7806e_zhuquec2_flash_tout_ctrl(flash, ctrl->val);
		break;

	case V4L2_CID_FLASH_INTENSITY:
		rval = sy7806e_zhuquec2_flash_brt_ctrl(flash, led_no, ctrl->val);
		break;

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		rval = sy7806e_zhuquec2_torch_brt_ctrl(flash, led_no, ctrl->val);
		break;
	}

err_out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int sy7806e_zhuquec2_led1_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return sy7806e_zhuquec2_get_ctrl(ctrl, SY7806E_ZHUQUEC2_LED1);
}

static int sy7806e_zhuquec2_led1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return sy7806e_zhuquec2_set_ctrl(ctrl, SY7806E_ZHUQUEC2_LED1);
}

static int sy7806e_zhuquec2_led0_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return sy7806e_zhuquec2_get_ctrl(ctrl, SY7806E_ZHUQUEC2_LED0);
}

static int sy7806e_zhuquec2_led0_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return sy7806e_zhuquec2_set_ctrl(ctrl, SY7806E_ZHUQUEC2_LED0);
}

static const struct v4l2_ctrl_ops sy7806e_zhuquec2_led_ctrl_ops[SY7806E_ZHUQUEC2_LED_MAX] = {
	[SY7806E_ZHUQUEC2_LED0] = {
			.g_volatile_ctrl = sy7806e_zhuquec2_led0_get_ctrl,
			.s_ctrl = sy7806e_zhuquec2_led0_set_ctrl,
			},
	[SY7806E_ZHUQUEC2_LED1] = {
			.g_volatile_ctrl = sy7806e_zhuquec2_led1_get_ctrl,
			.s_ctrl = sy7806e_zhuquec2_led1_set_ctrl,
			}
};

static int sy7806e_zhuquec2_init_controls(struct sy7806e_zhuquec2_flash *flash,
				enum sy7806e_zhuquec2_led_id led_no)
{
	struct v4l2_ctrl *fault;
	u32 max_flash_brt = flash->pdata->max_flash_brt[led_no];
	u32 max_torch_brt = flash->pdata->max_torch_brt[led_no];
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led[led_no];
	const struct v4l2_ctrl_ops *ops = &sy7806e_zhuquec2_led_ctrl_ops[led_no];

	v4l2_ctrl_handler_init(hdl, 8);

	/* flash mode */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_LED_MODE,
			       V4L2_FLASH_LED_MODE_TORCH, ~0x7,
			       V4L2_FLASH_LED_MODE_NONE);
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;

	/* flash source */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_STROBE_SOURCE,
			       0x1, ~0x3, V4L2_FLASH_STROBE_SOURCE_SOFTWARE);

	/* flash strobe */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE, 0, 0, 0, 0);

	/* flash strobe stop */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE_STOP, 0, 0, 0, 0);

	/* flash strobe timeout */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TIMEOUT,
			  SY7806E_ZHUQUEC2_FLASH_TOUT_MIN,
			  flash->pdata->max_flash_timeout,
			  SY7806E_ZHUQUEC2_FLASH_TOUT_STEP,
			  flash->pdata->max_flash_timeout);

	/* flash brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  SY7806E_ZHUQUEC2_FLASH_BRT_MIN, max_flash_brt,
			  SY7806E_ZHUQUEC2_FLASH_BRT_STEP, max_flash_brt);

	/* torch brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  SY7806E_ZHUQUEC2_TORCH_BRT_MIN, max_torch_brt,
			  SY7806E_ZHUQUEC2_TORCH_BRT_STEP, max_torch_brt);

	/* fault */
	fault = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_FAULT, 0,
				  V4L2_FLASH_FAULT_OVER_VOLTAGE
				  | V4L2_FLASH_FAULT_OVER_TEMPERATURE
				  | V4L2_FLASH_FAULT_SHORT_CIRCUIT
				  | V4L2_FLASH_FAULT_TIMEOUT, 0, 0);
	if (fault != NULL)
		fault->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (hdl->error)
		return hdl->error;

	flash->subdev_led[led_no].ctrl_handler = hdl;
	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops sy7806e_zhuquec2_ops = {
	.core = NULL,
};

static const struct regmap_config sy7806e_zhuquec2_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xF,
};

static void sy7806e_zhuquec2_v4l2_i2c_subdev_init(struct v4l2_subdev *sd,
		struct i2c_client *client,
		const struct v4l2_subdev_ops *ops)
{
	v4l2_subdev_init(sd, ops);
	sd->flags |= V4L2_SUBDEV_FL_IS_I2C;
	/* the owner is the same as the i2c_client's driver owner */
	sd->owner = client->dev.driver->owner;
	sd->dev = &client->dev;
	/* i2c_client and v4l2_subdev point to one another */
	v4l2_set_subdevdata(sd, client);
	i2c_set_clientdata(client, sd);
	/* initialize name */
	snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		client->dev.driver->name, i2c_adapter_id(client->adapter),
		client->addr);
}

static int sy7806e_zhuquec2_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int rval = -EINVAL;
	unsigned int flash1_led_state = 0;
	unsigned int flash2_led_state = 0;
	pr_info("%s: set old flash current before open and en high", __func__);
	sy7806e_zhuquec2_pinctrl_set(SY7806E_ZHUQUEC2_PINCTRL_PIN_HWEN, SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_HIGH);
	msleep(2);
	rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_ENABLE, &flash1_led_state);
	rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_ENABLE, &flash2_led_state);
	if (((flash1_led_state & 0x03) != 0) || ((flash2_led_state & 0x03) != 0)) {
		pr_info("%s: flash led is enabled and does not need to be opened", __func__);
		return 0;
	}
	flash_device_id = 0;
	rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_DEVICE_ID, &flash_device_id);
	pr_info("%s: flash1_device_id :%x", __func__, flash_device_id);
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,REG_LED0_FLASH_BR, 0xFF, sy7806e_flash_current_val.flash_val);//flash current
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,REG_LED1_FLASH_BR, 0xFF, sy7806e_flash_current_val.flash_val2);//flash current
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,REG_LED0_TORCH_BR, 0xFF, sy7806e_flash_current_val.torch_val);//torch current
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,REG_LED1_TORCH_BR, 0xFF, sy7806e_flash_current_val.torch_val2);//torch current

	flash_device_id = 0;
	rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_DEVICE_ID, &flash_device_id);
	pr_info("%s: flash2_device_id :%x", __func__, flash_device_id);
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,REG_LED0_FLASH_BR, 0xFF, sy7806e_2_flash_current_val.flash_val);//flash current
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,REG_LED1_FLASH_BR, 0xFF, sy7806e_2_flash_current_val.flash_val2);//flash current
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,REG_LED0_TORCH_BR, 0xFF, sy7806e_2_flash_current_val.torch_val);//torch current
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,REG_LED1_TORCH_BR, 0xFF, sy7806e_2_flash_current_val.torch_val2);//torch current

	rval = pm_runtime_get_sync(sd->dev);
	if (rval < 0) {
		pm_runtime_put_noidle(sd->dev);
		return rval;
	}

	return 0;
}

static int sy7806e_zhuquec2_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int rval = -EINVAL;
	unsigned int flash1_led_state = 0;
	unsigned int flash2_led_state = 0;
	pr_info("%s\n", __func__);
	rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_ENABLE, &flash1_led_state);
	rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_ENABLE, &flash2_led_state);
	if (((flash1_led_state & 0x03) != 0) || ((flash2_led_state & 0x03) != 0)) {
		pr_info("%s: flash led is enabled and does not need to be closed", __func__);
		return 0;
	}
	rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_LED0_FLASH_BR, &sy7806e_flash_current_val.flash_val);
	rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_LED1_FLASH_BR, &sy7806e_flash_current_val.flash_val2);
	rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_LED0_TORCH_BR, &sy7806e_flash_current_val.torch_val);
	rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_LED1_TORCH_BR, &sy7806e_flash_current_val.torch_val2);
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,REG_SOFT_RST, 0x80, 0x80);

	rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_LED0_FLASH_BR, &sy7806e_2_flash_current_val.flash_val);
	rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_LED1_FLASH_BR, &sy7806e_2_flash_current_val.flash_val2);
	rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_LED0_TORCH_BR, &sy7806e_2_flash_current_val.torch_val);
	rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_LED1_TORCH_BR, &sy7806e_2_flash_current_val.torch_val2);
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,REG_SOFT_RST, 0x80, 0x80);
	pr_info("%s: set soft and hardware reset before close", __func__);
	sy7806e_zhuquec2_pinctrl_set(SY7806E_ZHUQUEC2_PINCTRL_PIN_HWEN, SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_LOW);
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops sy7806e_zhuquec2_int_ops = {
	.open = sy7806e_zhuquec2_open,
	.close = sy7806e_zhuquec2_close,
};

static int sy7806e_zhuquec2_subdev_init(struct sy7806e_zhuquec2_flash *flash,
			      enum sy7806e_zhuquec2_led_id led_no, char *led_name)
{
	struct i2c_client *client = to_i2c_client(flash->dev);
	struct device_node *np = flash->dev->of_node, *child;
	const char *fled_name = "flash";
	int rval;

	// pr_info("%s %d", __func__, led_no);

	sy7806e_zhuquec2_v4l2_i2c_subdev_init(&flash->subdev_led[led_no],
				client, &sy7806e_zhuquec2_ops);
	flash->subdev_led[led_no].flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->subdev_led[led_no].internal_ops = &sy7806e_zhuquec2_int_ops;
	strscpy(flash->subdev_led[led_no].name, led_name,
		sizeof(flash->subdev_led[led_no].name));

	for (child = of_get_child_by_name(np, fled_name); child;
			child = of_find_node_by_name(child, fled_name)) {
		int rv;
		u32 reg = 0;

		rv = of_property_read_u32(child, "reg", &reg);
		if (rv)
			continue;

		if (reg == led_no) {
			flash->dnode[led_no] = child;
			flash->subdev_led[led_no].fwnode =
				of_fwnode_handle(flash->dnode[led_no]);
		}
	}

	rval = sy7806e_zhuquec2_init_controls(flash, led_no);
	if (rval)
		goto err_out;
	rval = media_entity_pads_init(&flash->subdev_led[led_no].entity, 0, NULL);
	if (rval < 0)
		goto err_out;
	flash->subdev_led[led_no].entity.function = MEDIA_ENT_F_FLASH;

	rval = v4l2_async_register_subdev(&flash->subdev_led[led_no]);
	if (rval < 0)
		goto err_out;

	return rval;

err_out:
	v4l2_ctrl_handler_free(&flash->ctrls_led[led_no]);
	return rval;
}

/* flashlight init */
static int sy7806e_zhuquec2_init(struct sy7806e_zhuquec2_flash *flash)
{
	int rval = 0;
	unsigned int reg_val;

	sy7806e_zhuquec2_pinctrl_set(SY7806E_ZHUQUEC2_PINCTRL_PIN_HWEN, SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_HIGH);

	/* set timeout */
	rval = sy7806e_zhuquec2_flash_tout_ctrl(flash, 400);
	if (rval < 0)
		return rval;
	/* output disable */
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
	rval = sy7806e_zhuquec2_mode_ctrl(flash);
	if (rval < 0)
		return rval;

	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				  REG_LED0_TORCH_BR, 0xFF, 0x00);
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				  REG_LED1_TORCH_BR, 0xFF, 0x00);
	if (rval < 0)
		return rval;
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				  REG_LED0_TORCH_BR, 0xFF, 0x00);
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				  REG_LED1_TORCH_BR, 0xFF, 0x00);
	if (rval < 0)
		return rval;
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				  REG_LED0_FLASH_BR, 0xFF, 0x00);
	rval = regmap_update_bits(sy7806e_zhuquec2_flash_data->regmap,
				  REG_LED1_FLASH_BR, 0xFF, 0x00);
	if (rval < 0)
		return rval;
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				  REG_LED0_FLASH_BR, 0xFF, 0x00);
	rval = regmap_update_bits(sy7806e_2_zhuquec2_flash_data->regmap,
				  REG_LED1_FLASH_BR, 0xFF, 0x00);
	if (rval < 0)
		return rval;
	/* reset faults */
	rval = regmap_read(sy7806e_zhuquec2_flash_data->regmap, REG_FLAG1, &reg_val);
	rval = regmap_read(sy7806e_2_zhuquec2_flash_data->regmap, REG_FLAG1, &reg_val);
	return rval;
}

/* flashlight uninit */
static int sy7806e_zhuquec2_uninit(struct sy7806e_zhuquec2_flash *flash)
{
	sy7806e_zhuquec2_pinctrl_set(SY7806E_ZHUQUEC2_PINCTRL_PIN_HWEN, SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_LOW);
	return 0;
}

static int sy7806e_zhuquec2_flash_open(void)
{
	return 0;
}

static int sy7806e_zhuquec2_flash_release(void)
{
	return 0;
}

static int sy7806e_zhuquec2_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	int ret;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if ((int)fl_arg->arg) {
			sy7806e_zhuquec2_torch_brt_ctrl(sy7806e_zhuquec2_flash_data, channel, 25000);
			sy7806e_zhuquec2_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
			sy7806e_zhuquec2_mode_ctrl(sy7806e_zhuquec2_flash_data);
		} else {
			sy7806e_zhuquec2_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
			sy7806e_zhuquec2_mode_ctrl(sy7806e_zhuquec2_flash_data);
			sy7806e_zhuquec2_enable_ctrl(sy7806e_zhuquec2_flash_data, channel, false);
		}
		break;

	case OPLUS_FLASH_IOC_SELECT_LED_NUM:
		if (fl_arg->arg == SY7806E_ZHUQUEC2_CONTROL_LED0 || fl_arg->arg == SY7806E_ZHUQUEC2_CONTROL_LED1
		    || fl_arg->arg == SY7806E_ZHUQUEC2_CONTROL_LED2) {
			sy7806e_zhuquec2_flash_data->led_mode = V4L2_FLASH_LED_MODE_FLASH;
			 ret = sy7806e_zhuquec2_select_led(sy7806e_zhuquec2_flash_data, fl_arg->arg);
			if (ret < 0) {
				pr_err("engineer cam set led[%d] fail", fl_arg->arg);
				return ret;
			}
		} else {
			sy7806e_zhuquec2_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
			sy7806e_zhuquec2_mode_ctrl(sy7806e_zhuquec2_flash_data);
			sy7806e_zhuquec2_enable_ctrl(sy7806e_zhuquec2_flash_data, channel, false);
		}
		break;
	case OPLUS_FLASH_IOC_SELECT_LED_MODE:
		if (fl_arg->arg >= 0) {
			flash_led_mode = fl_arg->arg;
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int sy7806e_zhuquec2_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	//mutex_lock(&sy7806e_zhuquec2_mutex);
	if (set) {
		if (!use_count)
			ret = sy7806e_zhuquec2_init(sy7806e_zhuquec2_flash_data);
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = sy7806e_zhuquec2_uninit(sy7806e_zhuquec2_flash_data);
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	//mutex_unlock(&sy7806e_zhuquec2_mutex);

	return 0;
}

static ssize_t sy7806e_zhuquec2_strobe_store(struct flashlight_arg arg)
{
	sy7806e_zhuquec2_set_driver(1);
	//sy7806e_zhuquec2_set_level(arg.channel, arg.level);
	//sy7806e_zhuquec2_timeout_ms[arg.channel] = 0;
	//sy7806e_zhuquec2_enable(arg.channel);
	sy7806e_zhuquec2_torch_brt_ctrl(sy7806e_zhuquec2_flash_data, arg.channel,
				arg.level * 25000);
	sy7806e_zhuquec2_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
	sy7806e_zhuquec2_mode_ctrl(sy7806e_zhuquec2_flash_data);
	sy7806e_zhuquec2_enable_ctrl(sy7806e_zhuquec2_flash_data, arg.channel, true);
	msleep(arg.dur);
	//sy7806e_zhuquec2_disable(arg.channel);
	sy7806e_zhuquec2_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
	sy7806e_zhuquec2_mode_ctrl(sy7806e_zhuquec2_flash_data);
	sy7806e_zhuquec2_enable_ctrl(sy7806e_zhuquec2_flash_data, arg.channel, false);
	sy7806e_zhuquec2_set_driver(0);
	return 0;
}

static struct flashlight_operations sy7806e_zhuquec2_flash_ops = {
	sy7806e_zhuquec2_flash_open,
	sy7806e_zhuquec2_flash_release,
	sy7806e_zhuquec2_ioctl,
	sy7806e_zhuquec2_strobe_store,
	sy7806e_zhuquec2_set_driver
};

static int sy7806e_zhuquec2_parse_dt(struct sy7806e_zhuquec2_flash *flash)
{
	struct device_node *np, *cnp;
	struct device *dev = flash->dev;
	u32 decouple = 0;
	int i = 0;
	int of_led_index_cnt = 0;

	if (!dev || !dev->of_node)
		return -ENODEV;

	np = dev->of_node;
	of_led_index_cnt = of_property_read_u32(np, "led-index", &led_index);

	if (!of_led_index_cnt ){
		Pcb_Version = get_PCB_Version();
		led_index = (Pcb_Version == PCB_T0) ? 0b011: 0b101;// c2&T0
	}else {
		led_index = 0b101;
	}

	pr_info("%s: of_led_index_cnt:%x led_index:%x,Pcb_Version:%d", __func__, of_led_index_cnt, led_index, Pcb_Version);
	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type",
					&flash->flash_dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"ct", &flash->flash_dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"part", &flash->flash_dev_id[i].part))
			goto err_node_put;
		snprintf(flash->flash_dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				flash->subdev_led[i].name);
		flash->flash_dev_id[i].channel = i;
		flash->flash_dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				flash->flash_dev_id[i].type,
				flash->flash_dev_id[i].ct,
				flash->flash_dev_id[i].part,
				flash->flash_dev_id[i].name,
				flash->flash_dev_id[i].channel,
				flash->flash_dev_id[i].decouple);
		if (flashlight_dev_register_by_device_id(&flash->flash_dev_id[i],
			&sy7806e_zhuquec2_flash_ops))
			return -EFAULT;
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static const struct i2c_device_id sy7806e_zhuquec2_id_table[] = {
	{SY7806E_ZHUQUEC2_NAME, 0},
	{SY7806E_2_ZHUQUEC2_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sy7806e_zhuquec2_id_table);

static const struct of_device_id sy7806e_zhuquec2_of_table[] = {
	{ .compatible = "oplus,sy7806e_zhuquec2" },
	{ .compatible = "oplus,sy7806e_2_zhuquec2" },
	{ },
};
MODULE_DEVICE_TABLE(of, sy7806e_zhuquec2_of_table);

static int sy7806e_zhuquec2_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct sy7806e_zhuquec2_flash *flash;
	const struct of_device_id *match;
	struct sy7806e_zhuquec2_platform_data *pdata = dev_get_platdata(&client->dev);
	int rval;

	pr_info("%s:%d", __func__, __LINE__);

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;
	if (is_register == false) {
		sy7806e_zhuquec2_pinctrl_init(&client->dev);
		sy7806e_zhuquec2_pinctrl_set(SY7806E_ZHUQUEC2_PINCTRL_PIN_HWEN, SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_HIGH);
	}

	flash->regmap = devm_regmap_init_i2c(client, &sy7806e_zhuquec2_regmap);
	if (IS_ERR(flash->regmap)) {
		rval = PTR_ERR(flash->regmap);
		return rval;
	}
	flash_device_id = 0;
	rval = regmap_read(flash->regmap, REG_DEVICE_ID, &flash_device_id);
	pr_info("%s: flash_device_id:%x client->addr:%x rval:%d", __func__, flash_device_id, client->addr, rval);

	/* if there is no platform data, use chip default value */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		pdata->max_flash_timeout = SY7806E_ZHUQUEC2_FLASH_TOUT_MAX;
		/* led 1 */
		pdata->max_flash_brt[SY7806E_ZHUQUEC2_LED0] = SY7806E_ZHUQUEC2_FLASH_BRT_MAX;
		pdata->max_torch_brt[SY7806E_ZHUQUEC2_LED0] = SY7806E_ZHUQUEC2_TORCH_BRT_MAX;
		/* led 2 */
		pdata->max_flash_brt[SY7806E_ZHUQUEC2_LED1] = SY7806E_ZHUQUEC2_FLASH_BRT_MAX;
		pdata->max_torch_brt[SY7806E_ZHUQUEC2_LED1] = SY7806E_ZHUQUEC2_TORCH_BRT_MAX;
	}
	flash->pdata = pdata;
	flash->dev = &client->dev;
	mutex_init(&flash->lock);
	if (!flash->dev->of_node) {
		pr_info("invalid flash->dev->of_node\n");
		return -ENOMEM;
	}
	match = of_match_node(sy7806e_zhuquec2_of_table, flash->dev->of_node);
	if (!match) {
		pr_info("invalid compatible string\n");
		return -ENOMEM;
	}
	pr_info("%s: match->compatible:%s", __func__, match->compatible);
	if (of_compat_cmp(sy7806e_zhuquec2_of_table[0].compatible, match->compatible, strlen(match->compatible)) == 0) {
		sy7806e_zhuquec2_flash_data = flash;
	} else if (of_compat_cmp(sy7806e_zhuquec2_of_table[1].compatible,match->compatible, strlen(match->compatible)) == 0){
		sy7806e_2_zhuquec2_flash_data = flash;
	}

	if (sy7806e_zhuquec2_flash_data && sy7806e_2_zhuquec2_flash_data) {
		if (rval < 0)
			return rval;
		rval = sy7806e_zhuquec2_subdev_init(flash, SY7806E_ZHUQUEC2_LED0, "sy7806e_zhuquec2-led0");
		if (rval < 0)
			return rval;
	}

	pm_runtime_enable(flash->dev);
	if (sy7806e_zhuquec2_flash_data && sy7806e_2_zhuquec2_flash_data) {
		rval = sy7806e_zhuquec2_parse_dt(flash);
	}
	i2c_set_clientdata(client, flash);

	rval = regmap_update_bits(flash->regmap,
				REG_LED0_TORCH_BR, 0xFF, 0x17);//torch current 65.66mA
	rval = regmap_update_bits(flash->regmap,
				REG_LED1_TORCH_BR, 0xFF, 0x17);//torch current 65.66mA

	if (is_register == true) {
		sy7806e_zhuquec2_pinctrl_set(SY7806E_ZHUQUEC2_PINCTRL_PIN_HWEN, SY7806E_ZHUQUEC2_PINCTRL_PINSTATE_LOW);
	}
	is_register = true;
	pr_info("%s:%d", __func__, __LINE__);
	return 0;
}

static void sy7806e_zhuquec2_remove(struct i2c_client *client)
{
	struct sy7806e_zhuquec2_flash *flash = i2c_get_clientdata(client);
	unsigned int i;
	if (flash) {
		for (i = SY7806E_ZHUQUEC2_LED0; i < SY7806E_ZHUQUEC2_LED_MAX; i++) {
			v4l2_device_unregister_subdev(&flash->subdev_led[i]);
			v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
			media_entity_cleanup(&flash->subdev_led[i].entity);
		}
	}

	pm_runtime_disable(&client->dev);

	pm_runtime_set_suspended(&client->dev);
}

static int __maybe_unused sy7806e_zhuquec2_suspend(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct sy7806e_zhuquec2_flash *flash = i2c_get_clientdata(client);

	pr_info("%s %d", __func__, __LINE__);

	return 0;//sy7806e_zhuquec2_uninit(flash);
}

static int __maybe_unused sy7806e_zhuquec2_resume(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct sy7806e_zhuquec2_flash *flash = i2c_get_clientdata(client);

	pr_info("%s %d", __func__, __LINE__);

	return 0;//sy7806e_zhuquec2_init(flash);
}

static const struct dev_pm_ops sy7806e_zhuquec2_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(sy7806e_zhuquec2_suspend, sy7806e_zhuquec2_resume, NULL)
};

static struct i2c_driver sy7806e_zhuquec2_i2c_driver = {
	.driver = {
		   .name = SY7806E_ZHUQUEC2_NAME,
		   .pm = &sy7806e_zhuquec2_pm_ops,
		   .of_match_table = sy7806e_zhuquec2_of_table,
		   },
	.probe = sy7806e_zhuquec2_probe,
	.remove = sy7806e_zhuquec2_remove,
	.id_table = sy7806e_zhuquec2_id_table,
};

static struct i2c_driver sy7806e_2_zhuquec2_i2c_driver = {
	.driver = {
		   .name = SY7806E_2_ZHUQUEC2_NAME,
		   .pm = &sy7806e_zhuquec2_pm_ops,
		   .of_match_table = sy7806e_zhuquec2_of_table,
		   },
	.probe = sy7806e_zhuquec2_probe,
	.remove = sy7806e_zhuquec2_remove,
	.id_table = sy7806e_zhuquec2_id_table,
};

static int __init i2c_driver_init(void)
{
	int ret;

	ret = i2c_add_driver(&sy7806e_zhuquec2_i2c_driver);
	if (ret) {
		pr_info("cannot register sy7806e_zhuquec2_i2c_driver\n");
		i2c_del_driver(&sy7806e_zhuquec2_i2c_driver);
		return ret;
	}

	ret = i2c_add_driver(&sy7806e_2_zhuquec2_i2c_driver);
	if (ret) {
		pr_info("cannot register sy7806e_2_zhuquec2_i2c_driver\n");
		i2c_del_driver(&sy7806e_2_zhuquec2_i2c_driver);
		return ret;
	}

	pr_info("sy7806e_zhuquec2 LED flash v4l2 driver register success\n");
	return 0;
}

static void __exit i2c_driver_exit(void)
{
	i2c_del_driver(&sy7806e_2_zhuquec2_i2c_driver);
	i2c_del_driver(&sy7806e_zhuquec2_i2c_driver);
	pr_info("sy7806e_zhuquec2 LED flash v4l2 driver exit\n");
}

module_init(i2c_driver_init);
module_exit(i2c_driver_exit);

MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("sy7806e_zhuquec2 LED flash v4l2 driver");
MODULE_LICENSE("GPL");
