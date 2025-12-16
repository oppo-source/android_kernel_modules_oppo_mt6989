// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 Oplus Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "hf_manager.h"
#include "hf_sensor_io.h"
#include "../adaptor-i2c.h"
#include "../ois_def.h"
#include <linux/timekeeping.h>
#include <linux/spinlock_types.h>

extern int adaptor_i2c_rd_p8(struct i2c_client *i2c_client, u16 addr, u16 reg,
                             u8 *p_vals, u32 n_vals);

extern int adaptor_i2c_wr_u8(struct i2c_client *i2c_client, u16 addr, u16 reg,
                             u8 val);
extern int adaptor_i2c_wr_u32(struct i2c_client *i2c_client, u16 addr, u16 reg,
                             u32 val);
extern int adaptor_i2c_rd_u32(struct i2c_client *i2c_client, u16 addr, u16 reg,
                             u32 *val);
extern int adaptor_i2c_wr_u16(struct i2c_client *i2c_client, u16 addr, u16 reg,
               u16 val);
extern int adaptor_i2c_rd_u16(struct i2c_client *i2c_client, u16 addr, u16 reg,
               u16 *val);
extern int get_support_sensor_list(struct sensor_info *list);
extern int Rohm_bu24721_fw_download(void);
extern int do_ois_cali(unsigned short *gyro_offset_x,
                       unsigned short *gyro_offset_y);
extern void Update_Gyro_offset_gain_cal_from_flash(void);
extern void Gyro_gain_set(unsigned short X_gain, unsigned short Y_gain);
extern void WriteGyroGainToFlash(void);

#define DRIVER_NAME "bu24721"
#define GYRO_VENDOR_BOSCH (0x1)
#define GYRO_VENDOR_IVEN (0x2)

#define LOG_INF(format, args...) \
    pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define LOG_ERR(format, args...) \
    pr_err(DRIVER_NAME " [%s] " format, __func__, ##args)

#define OIS_LOG_IF(cond, ...)  do { if ( (cond) ) { LOG_INF(__VA_ARGS__); } }while(0)

#define BU24721_NAME "bu24721"

#define BU24721_I2C_SLAVE_ADDR 0x7C

// #define Pro_ID_0 0x1A890423
// #define Pro_ID_0 0x1A890603
// #define Pro_ID_0 0x1A890623
#define Pro_ID_0 0x1A890893

#define BU24721_CTRL_DELAY_US 5000

#define POWER_OFF_CNT 5

// #define RT_SAMPLE
// #define NEW_SAMPLE
#define HYBRID_SAMPLE

#define MAX_TIME_INTERVAL (24*1000*1000)
#define MIN_TIME_INTERVAL (9*1000*1000)
//*** OIS mode ***//
#define Servo_on 0x01
#define OIS_on 0x02
#define OIS_standby 0x00
#define Servo_off 0x04
#define OIS_cal 0x03
#define OIS_still 0x63
#define OIS_View 0x79
#define OIS_movie 0x61
#define OIS_exp 0x03
#define OIS_zero 0x6B
#define OIS_zero_tri 0xEB

//*** SPI MODE ***//
#define SPI_Master 0x00
#define SPI_Monitor 0x04

#define endian(var) (((var << 8) & 0xff00) | ((var >> 8) & 0xff))

#define GROUPS 1

#define REG_MONITOR

#define REG_MONITOR_WAIT (30 * 1000)

#define REG_MONITOR_WAIT_THREAD (1000 * 1000)

// #define OIS_HALL_INTERVAL (17777 * 112)
#define OIS_HALL_INTERVAL (1972300)
// #define DELAY_POWER_DOWN

static struct sensor_info support_sensors[] = {
    {
        .sensor_type = SENSOR_TYPE_OIS1,
        .gain = 10000,
        .name = {'t', 'e', 'l', 'e', 'o', 'i', 's'},
        .vendor = {'o', 'p', 'l', 'u', 's'},
    },
};

static int g_gyro_offset[] = {0xdc, 0x18};
static unsigned short g_gyro_gain_x = 0;
static unsigned short g_gyro_gain_y = 0;
static int g_still_en = 0;
static int g_init = 0;
static int g_center = 0;
static int g_search = 0;
static int dbg = 0;
static unsigned char g_gyro_type = 0x1;
static unsigned char g_pantilt = 0x0;
static unsigned char g_anglelimit = 0x0;
static struct hf_client *gyro_client = NULL;
// static struct sensor_info sen_if_list[SENSOR_TYPE_SENSOR_MAX];
static int reg_monitor_init_cnt = 0;
static int first_into = 1;
static int buffer_dump = 0;
static spinlock_t ois_spinlock;
#ifdef DELAY_POWER_DOWN
static int delay_power_off = 0;
#endif

#ifdef NEW_SAMPLE
static int64_t ts_static = 0;
#endif
#ifdef HYBRID_SAMPLE
static int64_t ts_static = 0;
#endif

/* bu24721 device structure */
struct bu24721_device {
    struct v4l2_ctrl_handler ctrls;
    struct v4l2_subdev sd;
    struct v4l2_ctrl *focus;
    struct regulator *vin;
    struct regulator *vdd;
    struct pinctrl *vcamaf_pinctrl;
    struct pinctrl_state *vcamaf_on;
    struct pinctrl_state *vcamaf_off;
    struct hf_device hf_dev;
};

union bu24721_hall_data {
    struct eis_buffer {
        char padding;
        char n_value : 4;
        char ois_enable : 1;
        unsigned short hall[2 * 12 * GROUPS];
        short delay_cnt;
        char EOL;
    } d;
    char data[54];
};

#define OIS_DATA_NUMBER 32
struct OisInfo {
    int32_t is_ois_supported;
    int32_t data_mode; /* ON/OFF */
    int32_t samples;
    int32_t x_shifts[OIS_DATA_NUMBER];
    int32_t y_shifts[OIS_DATA_NUMBER];
    int64_t timestamps[OIS_DATA_NUMBER];
};

struct mtk_ois_pos_info {
    struct OisInfo *p_ois_info;
};

#define OIS_HALL_DATA_SIZE 52
#include "bu24721_if.h"
unsigned char poll_status(struct i2c_client *client)
{
    int i;
    unsigned char data = 0;
    int ret, retry = 3;
    for (i = 0; i < 50; i++) {
        do {
            ret = adaptor_i2c_rd_p8(client, client->addr, 0xF024, &data, 1);
            if (ret >= 0) {
                break;
            } else {
                LOG_ERR("i2c error, ret[%d] retry[%d]", ret, retry);
            }
            // ETIMEDOUT
            if (ret == -110) {
                data = 0;
                LOG_ERR("check F024 status timeout, break");
                return data;
            }
            retry--;
        } while (retry > 0);

        if (ret < 0) {
            LOG_ERR("check F024 status failed, i2c error, retry[%d]", retry);
            break;
        }

        if ((data & 0x01) == 1) {
            break;
        }

        usleep_range(800, 1200);
    } /*for*/

    if (data != 0x1) {
        OIS_LOG_IF(dbg, "check F024 status [%d]", data);
    }

    return data;
}

/* Control commnad */
#define VIDIOC_MTK_S_OIS_MODE _IOW('V', BASE_VIDIOC_PRIVATE + 2, int32_t)

#define VIDIOC_MTK_G_OIS_POS_INFO \
    _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct mtk_ois_pos_info)

static inline struct bu24721_device *to_bu24721_ois(struct v4l2_ctrl *ctrl)
{
    return container_of(ctrl->handler, struct bu24721_device, ctrls);
}

static inline struct bu24721_device *
sd_to_bu24721_ois(struct v4l2_subdev *subdev)
{
    return container_of(subdev, struct bu24721_device, sd);
}

static int bu24721_release(struct bu24721_device *bu24721)
{
    return 0;
}

int switch2flash(struct i2c_client *client)
{
    return 0;
}

static struct i2c_ops_info inf = {"bu24721", 0, 0};

#define SHOW(buf, len, fmt, ...) { \
    len += snprintf(buf + len, PAGE_SIZE - len, fmt, ##__VA_ARGS__); \
}

static ssize_t bu24721_i2c_ops_show(struct device *dev,
               struct device_attribute *attr, char *buf)
{
    int len = 0;

    SHOW(buf, len, "%s i2c read 0x%08x = 0x%08x\n",
            inf.name,
            inf.RegAddr,
            inf.RegData);
    return len;
}


static ssize_t bu24721_i2c_ops_store(struct device *dev,
               struct device_attribute *attr,
               const char *buf, size_t count)
{
    char delim[] = " ";
    char *token = NULL;
    char *sbuf = kzalloc(sizeof(char) * (count + 1), GFP_KERNEL);
    char *s = sbuf;
    int ret;
    unsigned int num_para = 0;
    char *arg[DBG_ARG_IDX_MAX_NUM];
    u32 val;
    u32 reg;
    unsigned short tmp_val = 0;
    // struct adaptor_ctx *ctx = to_ctx(dev_get_drvdata(dev));
    struct i2c_client *client = i2c_verify_client(dev);
    if (!client) {
        LOG_ERR("client is null!");
    }

    inf.RegAddr = 0;
    inf.RegData = 0;

    if (!sbuf)
        goto ERR_DEBUG_OPS_STORE;

    memcpy(sbuf, buf, count);

    token = strsep(&s, delim);
    while (token != NULL && num_para < DBG_ARG_IDX_MAX_NUM) {
        if (strlen(token)) {
            arg[num_para] = token;
            num_para++;
        }

        token = strsep(&s, delim);
    }

    if (num_para > DBG_ARG_IDX_MAX_NUM) {
        LOG_ERR("Wrong command parameter number %u\n", num_para);
        goto ERR_DEBUG_OPS_STORE;
    }
    ret = kstrtouint(arg[DBG_ARG_IDX_I2C_ADDR], 0, &reg);
    if (ret)
        goto ERR_DEBUG_OPS_STORE;
    inf.RegAddr = reg;

    if (num_para == DBG_ARG_IDX_MAX_NUM) {
        ret = kstrtouint(arg[DBG_ARG_IDX_I2C_DATA], 0, &val);
        if (ret)
            goto ERR_DEBUG_OPS_STORE;
        inf.RegData = val;

        ret = adaptor_i2c_wr_u16(client, client->addr, inf.RegAddr, (unsigned short)inf.RegData);
        LOG_ERR("%s i2c write 0x%08x = 0x%08x ret = %d\n",
            __func__,
            inf.RegAddr, inf.RegData, ret);
    }

    ret = adaptor_i2c_rd_u16(client, client->addr, inf.RegAddr, &tmp_val);
    inf.RegData = (unsigned int)tmp_val;
    LOG_ERR("%s i2c read 0x%08x = 0x%08x  ret = %d\n",
        __func__,
        inf.RegAddr, inf.RegData, ret);


ERR_DEBUG_OPS_STORE:

    kfree(sbuf);
    LOG_ERR("exit %s\n", __func__);

    return count;
}


static DEVICE_ATTR_RW(bu24721_i2c_ops);

static ssize_t bu24721_i2c_ops32_show(struct device *dev,
               struct device_attribute *attr, char *buf)
{
    int len = 0;

    SHOW(buf, len, "%s i2c read 0x%08x = 0x%08x\n",
            inf.name,
            inf.RegAddr,
            inf.RegData);
    return len;
}


static ssize_t bu24721_i2c_ops32_store(struct device *dev,
               struct device_attribute *attr,
               const char *buf, size_t count)
{
    char delim[] = " ";
    char *token = NULL;
    char *sbuf = kzalloc(sizeof(char) * (count + 1), GFP_KERNEL);
    char *s = sbuf;
    int ret;
    unsigned int num_para = 0;
    char *arg[DBG_ARG_IDX_MAX_NUM];
    u32 val;
    u32 reg;
    // struct adaptor_ctx *ctx = to_ctx(dev_get_drvdata(dev));
    struct i2c_client *client = i2c_verify_client(dev);
    if (!client) {
        LOG_ERR("client is null!");
    }

    inf.RegAddr = 0;
    inf.RegData = 0;

    if (!sbuf)
        goto ERR_DEBUG_OPS_STORE;

    memcpy(sbuf, buf, count);

    token = strsep(&s, delim);
    while (token != NULL && num_para < DBG_ARG_IDX_MAX_NUM) {
        if (strlen(token)) {
            arg[num_para] = token;
            num_para++;
        }

        token = strsep(&s, delim);
    }

    if (num_para > DBG_ARG_IDX_MAX_NUM) {
        LOG_ERR("Wrong command parameter number %u\n", num_para);
        goto ERR_DEBUG_OPS_STORE;
    }
    ret = kstrtouint(arg[DBG_ARG_IDX_I2C_ADDR], 0, &reg);
    if (ret)
        goto ERR_DEBUG_OPS_STORE;
    inf.RegAddr = reg;

    if (num_para == DBG_ARG_IDX_MAX_NUM) {
        ret = kstrtouint(arg[DBG_ARG_IDX_I2C_DATA], 0, &val);
        if (ret)
            goto ERR_DEBUG_OPS_STORE;
        inf.RegData = val;

        ret = adaptor_i2c_wr_u32(client, client->addr, inf.RegAddr, inf.RegData);
        LOG_ERR("%s i2c write 0x%08x = 0x%08x ret = %d\n",
            __func__,
            inf.RegAddr, inf.RegData, ret);
    }

    ret = adaptor_i2c_rd_u32(client, client->addr, inf.RegAddr, &(inf.RegData));
        LOG_ERR("%s i2c read 0x%08x = 0x%08x  ret = %d\n",
            __func__,
            inf.RegAddr, inf.RegData, ret);


ERR_DEBUG_OPS_STORE:

    kfree(sbuf);
    LOG_ERR("exit %s\n", __func__);

    return count;
}
static DEVICE_ATTR_RW(bu24721_i2c_ops32);

static ssize_t bu24721_dbg_show(struct device *dev, struct device_attribute *attr, char *buf) {
    return scnprintf(buf, PAGE_SIZE, "%d\n", dbg);
}

static ssize_t bu24721_dbg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    unsigned long data;
    int ret;
    ret = kstrtoul(buf, 10, &data);
    if (ret) {
        LOG_ERR("kstrtoul failed %d", ret);
        return count;
    }
    dbg = data & 0x1;
    buffer_dump = data & 0x2;
    return count;
}

static DEVICE_ATTR_RW(bu24721_dbg);

/* Power handling */
static int bu24721_power_off(struct bu24721_device *bu24721)
{
    int ret;

    LOG_ERR("%s E\n", __func__);

    g_still_en = 0;
    g_init = 0;
    g_center = 0;

    ret = bu24721_release(bu24721);
    if (ret)
        LOG_ERR("bu24721 release failed!\n");

    ret = regulator_disable(bu24721->vin);
    if (ret)
        return ret;

    ret = regulator_disable(bu24721->vdd);
    if (ret)
        return ret;

    if (bu24721->vcamaf_pinctrl && bu24721->vcamaf_off)
        ret = pinctrl_select_state(bu24721->vcamaf_pinctrl,
                                   bu24721->vcamaf_off);

    g_pantilt = 0;
    g_anglelimit = 0;
    LOG_ERR("%s X\n", __func__);
    return ret;
}

static int bu24721_power_on(struct bu24721_device *bu24721)
{
    int ret = 0;
    int ret1;
    struct i2c_client *client = v4l2_get_subdevdata(&bu24721->sd);
    unsigned char poll_ret;

    LOG_ERR("%s E\n", __func__);

    ret = regulator_enable(bu24721->vin);
    ret1 = regulator_set_voltage(bu24721->vin, 1780000, 1820000);
    if (ret < 0 || ret1 < 0) {
        LOG_ERR("%s bu24721->vin set failed ret[%d] ret1[%d]", __func__,
               ret, ret1);
        return -1;
    }

    usleep_range(BU24721_CTRL_DELAY_US, BU24721_CTRL_DELAY_US + 100);

    ret = regulator_enable(bu24721->vdd);
    ret1 = regulator_set_voltage(bu24721->vdd, 2780000, 2820000);
    if (ret < 0 || ret1 < 0) {
        LOG_ERR("%s bu24721->vdd set failed ret[%d] ret1[%d]", __func__,
               ret, ret1);
        return -1;
    }

    if (bu24721->vcamaf_pinctrl && bu24721->vcamaf_on)
        ret = pinctrl_select_state(bu24721->vcamaf_pinctrl,
                                   bu24721->vcamaf_on);

    if (ret < 0)
        return ret;

    /*
     * TODO(b/139784289): Confirm hardware requirements and adjust/remove
     * the delay.
     */
    usleep_range(BU24721_CTRL_DELAY_US, BU24721_CTRL_DELAY_US + 100);

    //ret = bu24721_init(bu24721);
    // standby / wakeup into idle
    client->addr = BU24721_I2C_SLAVE_ADDR >> 1;
    poll_ret = poll_status(client);
    if ((poll_ret & 0x01) != 1) {
        LOG_ERR("ois device not ready. poll_ret[%u] return", poll_ret);
        ret = -1;
    }
    ret = adaptor_i2c_wr_u8(client, client->addr, 0xF050, 0x00);
    poll_ret = poll_status(client);
    if ((poll_ret & 0x01) != 1) {
        LOG_ERR("wakeup poll faied.");
        if (ret < 0) {
            LOG_ERR("wakeup failed ret[%d] poll_ret[%u], return", ret, poll_ret);
            goto fail;
        }
    }
    LOG_ERR("%s X\n", __func__);
    return 0;

fail:
    regulator_disable(bu24721->vin);
    regulator_disable(bu24721->vdd);
    if (bu24721->vcamaf_pinctrl && bu24721->vcamaf_off) {
        pinctrl_select_state(bu24721->vcamaf_pinctrl,
                             bu24721->vcamaf_off);
    }
    LOG_ERR("%s fail process X\n", __func__);
    return ret;
}

static int bu24721_set_ctrl(struct v4l2_ctrl *ctrl)
{
    /* struct bu24721_device *bu24721 = to_bu24721_ois(ctrl); */

    return 0;
}

static const struct v4l2_ctrl_ops bu24721_ois_ctrl_ops = {
    .s_ctrl = bu24721_set_ctrl,
};

static int bu24721_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    int ret;

    OIS_LOG_IF(dbg, "%s\n", __func__);

    ret = pm_runtime_get_sync(sd->dev);
    if (ret < 0) {
        pm_runtime_put_noidle(sd->dev);
        return ret;
    }

    return 0;
}

static int bu24721_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    OIS_LOG_IF(dbg, "%s\n", __func__);

    pm_runtime_put(sd->dev);

    return 0;
}

static long bu24721_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
                                   void *arg)
{
    int ret = 0;

    switch (cmd) {
    case VIDIOC_MTK_S_OIS_MODE: {
        int *ois_mode = arg;

        if (*ois_mode)
            OIS_LOG_IF(dbg, "VIDIOC_MTK_S_OIS_MODE Enable\n");
        else
            OIS_LOG_IF(dbg, "VIDIOC_MTK_S_OIS_MODE Disable\n");
    } break;

    case VIDIOC_MTK_G_OIS_POS_INFO: {
        struct mtk_ois_pos_info *info = arg;
        struct OisInfo pos_info;
        int i = 0;

        memset(&pos_info, 0, sizeof(struct OisInfo));

        /* To Do */
        pos_info.data_mode = 1;

        pos_info.samples = OIS_DATA_NUMBER;
        pos_info.is_ois_supported = 1;
        for (i = 0; i < OIS_DATA_NUMBER; i++) {
            pos_info.x_shifts[i] = 0xab + i;
            pos_info.y_shifts[i] = 0xab + i;
            pos_info.timestamps[i] = 123 + i;
        }

        if (copy_to_user((void *)info->p_ois_info, &pos_info,
                         sizeof(pos_info)))
            ret = -EFAULT;
    } break;

    default:
        ret = -ENOIOCTLCMD;
        break;
    }

    return ret;
}

#ifdef REG_MONITOR

static struct task_struct *reg_monitor_task;

static int reg_monitor_kthread(void *arg) {
    struct i2c_client *client = (struct i2c_client *)arg;
    char gyro_raw[4] = {0, 0, 0, 0};
    char gyro_target_raw[4] = {0, 0, 0, 0};
    char hall_raw[4] = {0, 0, 0, 0};
    unsigned short gain_x, gain_y;
    struct sensor_info gyro_info = {0};
    int ret = 0;
#ifdef DELAY_POWER_DOWN
    int delay_cnt = POWER_OFF_CNT;
#endif
    struct v4l2_subdev *sd;
    struct bu24721_device *bu24721;
    sd = i2c_get_clientdata(client);
    bu24721 = sd_to_bu24721_ois(sd);
    adaptor_i2c_rd_u16(client, client->addr, 0xF07A, &gain_x);
    adaptor_i2c_rd_u16(client, client->addr, 0xF07C, &gain_y);
    LOG_ERR("gain raw code [0x%x, 0x%x]", gain_x, gain_y);
    while(!kthread_should_stop()) {
        if (dbg) {
            // print gyro data form monitor
            adaptor_i2c_wr_u8(client, client->addr, 0xf088, 0);
            poll_status(client);
            adaptor_i2c_rd_p8(client, client->addr, 0xF08A, &gyro_raw[0], 2);
            poll_status(client);
            adaptor_i2c_wr_u8(client, client->addr, 0xf088, 1);
            poll_status(client);
            adaptor_i2c_rd_p8(client, client->addr, 0xF08A, &gyro_raw[2], 2);
            OIS_LOG_IF(dbg, "gyro raw code [0x%x, 0x%x]", (u16)gyro_raw[0] << 8 | gyro_raw[1],
                    (u16)gyro_raw[2] << 8 | gyro_raw[3]);
            if (g_still_en) {
                adaptor_i2c_wr_u8(client, client->addr, 0xf15e, 0);
                poll_status(client);
                adaptor_i2c_rd_p8(client, client->addr, 0xF15c, &gyro_target_raw[0], 2);
                poll_status(client);
                adaptor_i2c_wr_u8(client, client->addr, 0xf15e, 1);
                poll_status(client);
                adaptor_i2c_rd_p8(client, client->addr, 0xF15c, &gyro_target_raw[2], 2);
                OIS_LOG_IF(dbg, "target raw code [0x%x 0x%x]",
                        (u16)gyro_target_raw[0] << 8 | gyro_target_raw[1],
                        (u16)gyro_target_raw[2] << 8 | gyro_target_raw[3]);

                adaptor_i2c_wr_u8(client, client->addr, 0xf060, 0);
                poll_status(client);
                adaptor_i2c_rd_p8(client, client->addr, 0xF062, &hall_raw[0], 2);
                poll_status(client);
                adaptor_i2c_wr_u8(client, client->addr, 0xf060, 1);
                poll_status(client);
                adaptor_i2c_rd_p8(client, client->addr, 0xF062, &hall_raw[2], 2);
                OIS_LOG_IF(dbg, "hall raw code [0x%x 0x%x]",
                        (u16)hall_raw[0] << 8 | hall_raw[1],
                        (u16)hall_raw[2] << 8 | hall_raw[3]);
            }
        }
        usleep_range(REG_MONITOR_WAIT, REG_MONITOR_WAIT + 1000);

        if (reg_monitor_init_cnt == 0 && g_search == 1) {
            if (first_into) {
                if (gyro_client) {
                    ret = hf_client_get_sensor_info(gyro_client, SENSOR_TYPE_GYROSCOPE, &gyro_info);
                    // ret = get_support_sensor_list(sen_if_list);
                    if(ret < 0) {
                        LOG_ERR("get GYRO sensor info failed");
                    } else {
                        // gyro_info = sen_if_list[SENSOR_TYPE_GYROSCOPE];
                        LOG_ERR("sensor_info sensor type[%d], name[%s], vendor[%s]", gyro_info.sensor_type, gyro_info.name, gyro_info.vendor);
                        if (!strcmp(gyro_info.vendor, "bosch")) {
                            g_gyro_type = GYRO_VENDOR_BOSCH;
                            LOG_ERR("bosch gyro match!");
                        } else if (!strcmp(gyro_info.vendor, "iven_sense")) {
                            g_gyro_type = GYRO_VENDOR_IVEN;
                            LOG_ERR("iven sense gyro match!");
                        } else {
                            LOG_ERR("no gyro type match!");
                            g_gyro_type = GYRO_VENDOR_BOSCH;
                        }
                        first_into = 0;
                        // if (bu24721_init(bu24721)) {
                        //     LOG_ERR("thread init failed return");
                        //     return -1;
                        // }
                    }
                } else {
                    LOG_ERR("NOMEM, create gyro hf_client failed!");
                }
            }
        }
#ifdef DELAY_POWER_DOWN
        if (delay_power_off) {
            if (delay_cnt <= 0) {
                bu24721_power_off(bu24721);
                delay_power_off = 0;
                break;
            }
            usleep_range(REG_MONITOR_WAIT_THREAD, REG_MONITOR_WAIT_THREAD + 1000);
            delay_cnt--;
            LOG_ERR("power off %d...", delay_cnt);
        } else {
            delay_cnt = POWER_OFF_CNT;
        }
#endif
    } //while
    if (reg_monitor_init_cnt == 0) {
        reg_monitor_init_cnt++;
    }
    LOG_ERR("%d reg_monitor_kthread return...", reg_monitor_init_cnt);
    return 0;
}

static int reg_monitor_thread_init(struct i2c_client *client)
{
    int err;
    LOG_ERR("Kernel thread initalizing...\n");
    reg_monitor_task = kthread_create(reg_monitor_kthread, client, "reg_monitor_kthread");
    if (IS_ERR(reg_monitor_task)) {
        LOG_ERR("Unable to start kernel thread./n");
        err = PTR_ERR(reg_monitor_task);
        reg_monitor_task = NULL;
        return err;
    }
    wake_up_process(reg_monitor_task);
    return 0;
}
static void __maybe_unused reg_monitor_thread_exit(void)
{
    if(reg_monitor_task){
        LOG_ERR("Cancel this kernel thread.\n");
        kthread_stop(reg_monitor_task);
        LOG_ERR("Canceled.\n");
    }
}

#endif /*REG_MONITOR*/

static int bu24721_batch(struct hf_device *hfdev, int sensor_type,
                         int64_t delay, int64_t latency)
{
    pr_debug("%s id:%d delay:%lld latency:%lld\n", __func__, sensor_type,
             delay, latency);
    return 0;
}

// static int bu24721_event_report(struct hf_device *hfdev, unsigned int *word,
//                                 int len)
// {
//     struct hf_manager *manager;
//     struct hf_manager_event event;
//     int i;
//     manager = bu24721->hf_dev.manager;
//     memset(&event, 0, sizeof(struct hf_manager_event));
//     event.sensor_type = SENSOR_TYPE_OIS1;
//     event.accurancy = SENSOR_ACCURANCY_HIGH;
//     event.action = DATA_ACTION;
//     event.timestamp = ktime_get_boottime_ns();
//     for (i = 0; i < len; i++) {
//         event.word[i] = word[i];
//     }
//     manager->report(manager, &event);
//     manager->complete(manager);
//     return 0;
// }

static short convert(unsigned short value)
{
    short l_value = 0;
    l_value = (short)(value + 65536L);
    return l_value;
}

static int stroke_convert(int code)
{
    int pixel_shift = 0;
    pixel_shift = code * 1739;
    return pixel_shift;
}

static int bu24721_enable(struct hf_device *hfdev, int sensor_type, int en)
{
    struct i2c_client *client;
    struct v4l2_subdev *sd;
    struct bu24721_device *bu24721;
    int ret = 0;

    if (0 == hfdev) {
        LOG_ERR("%s hfdev is null!! return error", __func__);
        return -1;
    }
    OIS_LOG_IF(dbg, "%s bu24721 hfdev[%p]", __func__, hfdev);
    client = hf_device_get_private_data(hfdev);
    sd = i2c_get_clientdata(client);
    bu24721 = sd_to_bu24721_ois(sd);

    LOG_ERR("%s sensor_type[%d] en[%d]", __func__, sensor_type, en);

    OIS_LOG_IF(dbg, "%s all_data size[%zd]", __func__, sizeof(union bu24721_hall_data));

#ifdef DELAY_POWER_DOWN
    if (en) {
        if ((delay_power_off == 1) && (g_init == 1)) {
            delay_power_off = 0;
            LOG_ERR("SKIP power on, already on.");
            return 0;
        }
        ret = bu24721_power_on(bu24721);
        if (ret < 0) {
            return -1;
        }
#ifdef REG_MONITOR
        reg_monitor_thread_init(client);
#endif
        delay_power_off = 0;
    } else {
        if (g_init) {
            delay_power_off = 1;
        } else {
            LOG_ERR("no inited yet, skip power off");
        }
    }
#else /*DELAY_POWER_DOWN*/
    if (en) {
        ret = bu24721_power_on(bu24721);
        if (ret < 0) {
            return -1;
        }
#ifdef REG_MONITOR
        if (reg_monitor_init_cnt == 0) {
            g_search = 1;
            reg_monitor_thread_init(client);
        }
#endif
    } else {
#ifdef REG_MONITOR
    if (g_init || (reg_monitor_init_cnt == 0)) {
        g_search = 0;
        reg_monitor_thread_exit();
    }
#endif
        bu24721_power_off(bu24721);
    }
#endif /*DELAY_POWER_DOWN*/

    return 0;
}


static int bu24721_init(struct bu24721_device *bu24721 , unsigned char pantilt, unsigned char anglelimit)
{
    struct i2c_client *client = v4l2_get_subdevdata(&bu24721->sd);
    unsigned char poll_ret;
    int ret = 0;

    union {
        int checksum;
        unsigned char checksum_byte[4];
    } Prog_ID;
    int id;
    LOG_ERR("bu24721_init E");
    client->addr = BU24721_I2C_SLAVE_ADDR >> 1;
    OIS_LOG_IF(dbg, "%s i2c_client %p addr 0x%x", __func__, client, client->addr);
    poll_ret = poll_status(client);
    if ((poll_ret & 0x01) != 1) {
        LOG_ERR("ois device not ready. poll_ret[%u] return", poll_ret);
        return -1;
    }

    adaptor_i2c_rd_p8(client, client->addr, 0xF01C, Prog_ID.checksum_byte,
                      4);
    id = (int)Prog_ID.checksum_byte[0] << 24 |
         Prog_ID.checksum_byte[1] << 16 | Prog_ID.checksum_byte[2] << 8 |
         Prog_ID.checksum_byte[3];
    LOG_ERR("%s current firmware ID raw[0x%x] endian byte[0x%x]", __func__,
           Prog_ID.checksum,
           (int)Prog_ID.checksum_byte[0] << 24 |
               Prog_ID.checksum_byte[1] << 16 |
               Prog_ID.checksum_byte[2] << 8 |
               Prog_ID.checksum_byte[3]);


    if (id != Pro_ID_0) {
        if (id == 0) {
            LOG_ERR("no firmware!!!");
        } else {
            LOG_ERR("firmware need update! current firmware ID[0x%x] Pro_ID_0[0x%x]",
                   id, Pro_ID_0);
        }
    }

    if (g_center == 0) {
        // lens center on / serv on
        adaptor_i2c_wr_u8(client, client->addr, 0xF020, 0x1);
        // usleep_range(20000, 20000 + 1000);
        poll_ret = poll_status(client);
        if (poll_ret != 1)
            LOG_ERR("servo_on failed");
        g_center = 1;
    }

    ret = adaptor_i2c_wr_u32(client, client->addr, 0xF080, 0x506048C3);
    if (ret < 0) {
        LOG_ERR("coil flux failed, ret[%d] poll_ret[%u], return", ret, poll_ret);
    }
    ret = adaptor_i2c_wr_u32(client, client->addr, 0xF084, 0x01800180);
    if (ret < 0) {
        LOG_ERR("coil flux failed, ret[%d] poll_ret[%u], return", ret, poll_ret);
    }
    poll_ret = poll_status(client);
    if (poll_ret != 1)
        LOG_ERR("coil flux failed");

    Update_Gyro_offset_gain_cal_from_flash();

    // lens linear crosstalk compensation on
    adaptor_i2c_wr_u8(client, client->addr, 0xF17C, 0x1);
    poll_ret = poll_status(client);
    if (poll_ret != 1)
        LOG_ERR("servo_linaer correction failed");

    // select gyro default:0x0 for qcom icm42631
    // 0x1 for MTK bmi260
    // 0x2 for mtk icm42631
    ret = adaptor_i2c_wr_u8(client, client->addr, 0xF12B, g_gyro_type);
    poll_ret = poll_status(client);
    if (poll_ret != 1) {
        if (ret < 0) {
            LOG_ERR("select gyro sensor failed ret[%d] poll_ret[%u], return", ret, poll_ret);
            return -1;
        }
    }

    ret = adaptor_i2c_wr_u8(client, client->addr, 0xF02A, SPI_Monitor);
    poll_ret = poll_status(client);
    if (poll_ret != 1) {
        if (ret < 0) {
            LOG_ERR("spi monitor failed, ret[%d] poll_ret[%u], return", ret, poll_ret);
            return -1;
        }
    }

    // gyro read on
    ret = adaptor_i2c_wr_u8(client, client->addr, 0xF023, 0x0);
    poll_ret = poll_status(client);
    if (poll_ret != 1) {
        if (ret < 0) {
            LOG_ERR("gyro_on failed, ret[%d] poll_ret[%u], return", ret, poll_ret);
            return -1;
        }
    }

    // select ois still mode
    adaptor_i2c_wr_u8(client, client->addr, 0xF021, 0xEB);
    poll_ret = poll_status(client);
    if (poll_ret != 1)
        LOG_ERR("still mode on failed");

    // [7:4] pan and tilt Angular velocity limit  [3:0] pan and tilt angle limit
    // A means 1.1 degree ;  9 means 1.0 degree.
    if(pantilt == 0)
        pantilt = 0x79;
    adaptor_i2c_wr_u8(client, client->addr, 0xF18E, pantilt);
    poll_ret = poll_status(client);
    if (poll_ret != 1) {
        LOG_ERR("ois_pantilt failed");
    } else {
        g_pantilt = pantilt;
        LOG_ERR("init set pantilt 0x%x", pantilt);
    }

    if(anglelimit == 0)
        anglelimit = 0xF0;
    // [3：0]    [7:4] angle limit: F means 1.6 degree
    adaptor_i2c_wr_u8(client, client->addr, 0xF025, anglelimit);
    poll_ret = poll_status(client);
    if (poll_ret != 1) {
        LOG_ERR("angle limit failed");
    } else {
        g_anglelimit = anglelimit;
        LOG_ERR("init set angle limit 0x%x", anglelimit);
    }
#ifdef REG_MONITOR
    if (!g_init) {
        reg_monitor_thread_init(client);
    }
#endif
    g_init = 1;
    g_still_en = 0;
    LOG_ERR("bu24721_init X");
    return 0;
}

int bu24721_config_cali(struct hf_device *hfdev, int sensor_type, void *data,
                        uint8_t length)
{
    struct i2c_client *client;
    unsigned short offset_x, offset_y;
    unsigned char poll_ret;
    int ret = 0;
    struct bu24721_device *bu24721;
    struct v4l2_subdev *sd;
    mois_config_data *config;
    client = hf_device_get_private_data(hfdev);
    if (!client) {
        LOG_ERR("i2c client is null !!! return");
        return -1;
    }
    OIS_LOG_IF(dbg, "%s sensor_type[%d] length[%d] mois_config_data size[%zd]",
            __func__, sensor_type, length, sizeof(mois_config_data));
    if (data) {
        config = (mois_config_data *)data;
        if((g_pantilt != config->pantilt || g_anglelimit != config->anglelimit) && config->pantilt !=0) {
                g_pantilt = config->pantilt;
                g_anglelimit = config->anglelimit;
            }
        OIS_LOG_IF(dbg, "received pantilt 0x%x limit 0x%x", config->pantilt, config->anglelimit);
        OIS_LOG_IF(dbg, "config->mode is [%d]", config->mode);
    } else {
        LOG_ERR("%s config data is null !!! return", __func__);
        return -1;
    }
    if (!g_init && (config->mode != AK_WorkingMode) && (config->mode != AK_Centering)) {
        LOG_ERR("ois is not init !!! return");
        return -1;
    }
    LOG_ERR("%s config->mode:%d", __func__, config->mode);
    switch (config->mode) {
    // add for center on before init
    case AK_Centering: {
        if(g_center == 0) {
            // lens center on / serv on
            adaptor_i2c_wr_u8(client, client->addr, 0xF020, 0x1);
            // usleep_range(20000, 20000 + 1000);
            poll_ret = poll_status(client);
            if (poll_ret != 1)
                LOG_ERR("servo_on failed");
            LOG_ERR("center on before init success");
            g_center = 1;
        }
    } break;
    case AK_WorkingMode: {
        if (g_init) {
            LOG_ERR("%s already in working mode !!! return", __func__);
            return 0;
        }
        sd = i2c_get_clientdata(client);
        bu24721 = sd_to_bu24721_ois(sd);
        ret = bu24721_init(bu24721, g_pantilt, g_anglelimit);
        if (ret < 0) {
            LOG_ERR("bu24721_init failed");
            return -1;
        }
    } break;
    case AK_StandbyMode: {
        sd = i2c_get_clientdata(client);
        bu24721 = sd_to_bu24721_ois(sd);
        adaptor_i2c_wr_u8(client, client->addr, 0xF054, 0x00);
        poll_ret = poll_status(client);
        if ((poll_ret & 0x01) != 1) {
            LOG_ERR("set standby failed, poll_ret[%u] ", poll_ret);
            return -1;
        }
    } break;
    case AK_EnableMOIS:
    // fall through
    case AK_Still: {
        if(g_pantilt == 0)
            g_pantilt = 0x79;
        adaptor_i2c_wr_u8(client, client->addr, 0xF18E, g_pantilt);
        poll_ret = poll_status(client);
        if (poll_ret != 1) {
            LOG_ERR("ois_pantilt failed");
        } else {
            LOG_ERR("inter still mode set pantilt 0x%x", g_pantilt);
        }
        if(g_anglelimit == 0)
            g_anglelimit = 0xF0;
        // [3：0]    [7:4] angle limit: F means 1.6 degree
        adaptor_i2c_wr_u8(client, client->addr, 0xF025, g_anglelimit);
        poll_ret = poll_status(client);
        if (poll_ret != 1) {
            LOG_ERR("angle limit failed");
        } else {
            LOG_ERR("inter still mode set angle limit 0x%x", g_anglelimit);
        }
        LOG_ERR("into still mode");
        // ois on -- slected mode active
        ret = adaptor_i2c_wr_u8(client, client->addr, 0xF020, 0x02);
        if (ret < 0) {
            LOG_ERR("ois_on failed, ret[%d] return", ret);
            return -1;
        }
        poll_ret = poll_status(client);
        if (poll_ret != 1) {
            LOG_ERR("ois_on failed, poll_ret[%u] return", poll_ret);
            return -1;
        } else {
            LOG_ERR("ois on");
            g_still_en = 1;
        }
    } break;
    case AK_DisableMOIS:
    // fall through
    case AK_CenteringOn: {
        LOG_ERR("into center on mode");
        ret = adaptor_i2c_wr_u8(client, client->addr, 0xF020, 0x01);
        if (ret < 0) {
            LOG_ERR("center_on failed, ret[%d] return", ret);
            return -1;
        }
        poll_ret = poll_status(client);
        if (poll_ret != 1) {
            LOG_ERR("center_on failed, poll_ret[%u] return", poll_ret);
            return -1;
        } else {
            LOG_ERR("center on success");
            g_still_en = 0;
        }
    } break;
    case AK_Movie: {
        LOG_ERR("into movie mode");
        adaptor_i2c_wr_u8(client, client->addr, 0xF021, 0x61);
        poll_ret = poll_status(client);
        if (poll_ret != 1)
            LOG_ERR("movie mode failed");
        // ois on -- slected mode active
        adaptor_i2c_wr_u8(client, client->addr, 0xF020, 0x02);
        poll_ret = poll_status(client);
        if (poll_ret != 1)
            LOG_ERR("ois_on failed");
    } break;
    case AK_Pantilt: {
        // lens center on / serv on
        adaptor_i2c_wr_u8(client, client->addr, 0xF020, 0x1);
        //usleep_range(20000, 20000 + 1000);
        poll_ret = poll_status(client);
        if (poll_ret != 1){
            LOG_ERR("center_on failed, poll_ret[%u] return", poll_ret);
            return -1;
        }
        LOG_ERR("into AK_Pantilt mode");
        ret = adaptor_i2c_wr_u8(client, client->addr, 0xF021, 0x03);
        if (ret < 0) {
            LOG_ERR("AK_Pantilt failed, ret[%d] return", ret);
            return -1;
        }
        poll_ret = poll_status(client);
        if (poll_ret != 1){
            LOG_ERR("AK_Pantilt failed, poll_ret[%u] return", poll_ret);
            return -1;
        }
        // ois on -- slected mode active
        ret = adaptor_i2c_wr_u8(client, client->addr, 0xF020, 0x02);
        if (ret < 0) {
            LOG_ERR("AK_Pantilt ois on failed, ret[%d] return", ret);
            return -1;
        }
        if (poll_ret != 1){
            LOG_ERR("AK_Pantilt ois on failed, poll_ret[%u] return", poll_ret);
            return -1;
        }
    } break;
    case AK_TestMode: {
        if ((66 < g_gyro_offset[0]) || (g_gyro_offset[0] < -66)
            || (66 < g_gyro_offset[1]) || (g_gyro_offset[1] < -66)) {
            LOG_ERR("do gyro offset...");
            do_ois_cali(&offset_x, &offset_y);
            g_gyro_offset[0] = convert(offset_x);
            g_gyro_offset[1] = convert(offset_y);
            Update_Gyro_offset_gain_cal_from_flash();
            LOG_ERR("%s sensor_type[%d] cali[%d,%d] g_gyro_offset[%d,%d]", __func__,
                    sensor_type, offset_x, offset_y, g_gyro_offset[0], g_gyro_offset[1]);
        } else {
            LOG_ERR("%s skip offset cali, sensor_type[%d] cali[%d,%d]",
                    __func__, sensor_type, g_gyro_offset[0],
                    g_gyro_offset[1]);
        }
    } break;
    case MOIS_Gyro_Gain_Cal: {
        LOG_ERR("traverse gyro gain[%d,%d]", config->mois_gain_x, config->mois_gain_y);
        if (!g_still_en) {
            // lens center on / serv on
            adaptor_i2c_wr_u8(client, client->addr, 0xF020, 0x1);
            //usleep_range(20000, 20000 + 1000);
            poll_ret = poll_status(client);
            if (poll_ret != 1){
                LOG_ERR("center_on failed, poll_ret[%u] return", poll_ret);
                return -1;
            }
            LOG_ERR("into gyro gain cal AK_Pantilt mode");
            ret = adaptor_i2c_wr_u8(client, client->addr, 0xF021, 0x03);
            if (ret < 0) {
                LOG_ERR("gyro gain cal AK_Pantilt failed, ret[%d] return", ret);
                return -1;
            }
            poll_ret = poll_status(client);
            if (poll_ret != 1){
                LOG_ERR("gyro gain cal AK_Pantilt failed, poll_ret[%u] return", poll_ret);
                return -1;
            }
            // ois on -- slected mode active
            ret = adaptor_i2c_wr_u8(client, client->addr, 0xF020, 0x02);
            if (ret < 0) {
                LOG_ERR("gyro gain cal AK_Pantilt ois on failed, ret[%d] return", ret);
                return -1;
            }
            if (poll_ret != 1){
                LOG_ERR("gyro gain cal AK_Pantilt ois on failed, poll_ret[%u] return", poll_ret);
                return -1;
            }
            g_still_en = 1;
        }
        g_gyro_gain_x = (unsigned short)config->mois_gain_x;
        g_gyro_gain_y = (unsigned short)config->mois_gain_y;
        OIS_LOG_IF(dbg, "gyro_gain[%u,%u]", g_gyro_gain_x, g_gyro_gain_y);
        Gyro_gain_set(g_gyro_gain_x, g_gyro_gain_y);
    } break;
    default:
        OIS_LOG_IF(dbg, "into default mode just break");
        break;
    }

    return 0;
}



#ifdef RT_SAMPLE
static int bu24721_sample(struct hf_device *hfdev)
{
    struct i2c_client *client;
    struct v4l2_subdev *sd;
    struct bu24721_device *bu24721;
    struct hf_manager *manager;
    struct hf_manager_event event;
    union bu24721_hall_data all_data;
    int i = 0;
    int j = 0;
    int i2c_ret;
    int64_t ts_before, ts_after, ts_mid, ts_diff;
    int data_cnt = 0, delay_cnt = 0;
    // char f0e2[2] = { 0x55, 0x56 };
    // char f0e4[2] = { 0x55, 0x56 };

    // OIS_LOG_IF(dbg, "%s bu24721 hfdev[%p]", __func__, hfdev);
    if (hfdev) {
        client = hf_device_get_private_data(hfdev);
    } else {
        LOG_ERR("NULL hfdev");
        goto err;
    }
    if (client) {
        sd = i2c_get_clientdata(client);
    } else {
        LOG_ERR("NULL client");
        goto err;
    }
    if (sd) {
        bu24721 = sd_to_bu24721_ois(sd);
    } else {
        LOG_ERR("NULL sd");
        goto err;
    }
    if (bu24721) {
        manager = bu24721->hf_dev.manager;
    } else {
        LOG_ERR("NULL bu24721");
        goto err;
    }
    if (!manager) {
        LOG_ERR("NULL manager");
        goto err;
    }

    if (poll_status(client) != 1) {
        LOG_ERR("%s sample poll failed. device not ready, return");
        goto err;
    }



    // read f0e2 f0e4 to check spi monitor gyro
    // adaptor_i2c_rd_p8(client, client->addr, 0xF0e2, f0e2, 2);
    // adaptor_i2c_rd_p8(client, client->addr, 0xF0e4, f0e4, 2);
    // OIS_LOG_IF(dbg, "%s f0e2[%d] f0e4[%d]", __func__, (u16)f0e2[0] << 8 | f0e2[1],
    //     (u16)f0e4[0] << 8 | f0e4[1]);
    // spin_lock(&ois_spinlock);
    memset(&all_data, 0, sizeof(all_data));
    memset(&event, 0, sizeof(struct hf_manager_event));
    ts_before = ktime_get_boottime_ns();
    i2c_ret = adaptor_i2c_rd_p8(client, client->addr, 0xF200,
                                &all_data.data[1], 48 * GROUPS + 4);
    ts_after = ktime_get_boottime_ns();
    // spin_unlock(&ois_spinlock);
    if (i2c_ret < 0) {
        LOG_ERR("read F200 i2c error");
        goto err;
    }
    OIS_LOG_IF(buffer_dump, "f200 head 0x%x", all_data.data[1]);
    OIS_LOG_IF(buffer_dump, "ts_before[%lld] ts_after[%lld] interval_us[%lld]", ts_before, ts_after, (ts_after - ts_before) / 1000);
    if (all_data.d.n_value > 12) {
        data_cnt = 12;
    } else {
        data_cnt = all_data.d.n_value;
    }
    if (data_cnt < 12) {
        delay_cnt = endian(all_data.d.hall[data_cnt * 2]);
    } else {
        delay_cnt = endian(all_data.d.delay_cnt);
    }
    ts_diff = ts_after - ts_before;
    ts_mid = ts_before + ts_diff / 3;
    event.timestamp = ts_mid;
    // print raw eis buffer code
    // for(i = 1;i < 54;i++) {
    //     OIS_LOG_IF(dbg, "raw code [%d] 0x%x", i, all_data.data[i]);
    // }


    // OIS_LOG_IF(dbg, "%s value is [%d] ois en[%d] delay[%d]", __func__,
    //     all_data.d.n_value, all_data.d.ois_enable,
    //     endian(all_data.d.delay_cnt));

    // OIS_LOG_IF(dbg, "%s  char[0]  value is [%d] ois en[%d]", __func__, all_data.data[0] & 0xf, (all_data.data[0] & 0x10) >> 4);
    // OIS_LOG_IF(dbg, "%s  char[1]  value is [%d] ois en[%d]", __func__, all_data.data[1] & 0xf, (all_data.data[1] & 0x10) >> 4);
    event.timestamp -= (17800 * delay_cnt);
    event.timestamp -= (data_cnt * 2000000);

    event.sensor_type = SENSOR_TYPE_OIS1;
    event.accurancy = SENSOR_ACCURANCY_HIGH;
    event.action = DATA_ACTION;
    // [0] [1] hall
    // [2] [3] pixel shift
    // [4] [5] gyro
    for (i = 0; i < data_cnt; i++) {
        event.timestamp += 2000000;
        for (j = 0; j < GROUPS; j++) {
            event.word[2 * j] = (int)convert(endian(all_data.d.hall[2 * i * GROUPS + 2 * j]));
            event.word[2 * j + 1] = (int)convert(endian(all_data.d.hall[2 * i * GROUPS + 2 * j + 1]));
            event.word[2 * j + 2] = stroke_convert(event.word[2 * j]);
            event.word[2 * j + 3] = stroke_convert(event.word[2 * j + 1]) * -1;
            event.word[2 * j] = event.word[2 * j] * 10000;
            event.word[2 * j + 1] = event.word[2 * j + 1] * 10000;
            OIS_LOG_IF(buffer_dump, "eisbuffer event.word[%d] index[%d] endian[%u] convert[%d] pixel shift[%d]",2 * j,2 * i * GROUPS + 2 * j, endian(all_data.d.hall[2 * i * GROUPS + 2 * j]), event.word[2 * j], event.word[2 * j + 2]);
            OIS_LOG_IF(buffer_dump, "eisbuffer event.word[%d] index[%d] endian[%u] convert[%d] pixel shift[%d]",2 * j + 1,2 * i * GROUPS + 2 * j + 1, endian(all_data.d.hall[2 * i * GROUPS + 2 * j + 1]), event.word[2 * j + 1], event.word[2 * j + 3]);
        }
        OIS_LOG_IF(buffer_dump, "ts[%lld]", event.timestamp);
        manager->report(manager, &event);
    }

    manager->complete(manager);

    return 0;

err:
    return -1;
}

#endif /*RT_SAMPLE*/

/*NEW_SAMPLE*/
#ifdef NEW_SAMPLE

static int64_t get_timestamp(int delay_cnt, int data_cnt, int64_t ts, int64_t ts_diff) {
    int64_t diff;
    int delay_time;
    int min_threshold_time, max_threshold_time;

    diff = ts - ts_static;
    delay_time = delay_cnt * 17777;
    min_threshold_time = OIS_HALL_INTERVAL * (data_cnt - 3) + delay_time;
    max_threshold_time = OIS_HALL_INTERVAL * (data_cnt + 2) + delay_time;
    if (ts_diff > 2000000) {
        OIS_LOG_IF(buffer_dump, "i2c delay too large, spend[%lld] data_cnt[%d] diff_us[%lld]", ts_diff, data_cnt, diff/1000);
    }
    OIS_LOG_IF(buffer_dump, "diff_us[%lld] ts_ms[%lld] cnt[%d] delay_time_us[%d]", diff/1000, ts/1000000, data_cnt, delay_time/1000);
    if (diff > max_threshold_time || diff < min_threshold_time) {
        if (ts_diff < 2000000) {
            spin_lock(&ois_spinlock);
            ts_static = ts - delay_time - data_cnt * OIS_HALL_INTERVAL;
            spin_unlock(&ois_spinlock);
            LOG_ERR("reset timestamp[%lld]  diff_us[%lld] delay_cnt[%d] data_cnt[%d] threshold_time_us[%d, %d]", ts_static, diff/1000, delay_cnt, data_cnt, min_threshold_time/1000, max_threshold_time/1000);
        } else {
            LOG_ERR("ts_diff too large, wait next time");
        }
    }
    return ts_static;
}

static int bu24721_sample(struct hf_device *hfdev)
{
    struct i2c_client *client;
    struct v4l2_subdev *sd;
    struct bu24721_device *bu24721;
    struct hf_manager *manager;
    struct hf_manager_event event;
    union bu24721_hall_data all_data;
    int i = 0, j = 0;
    int i2c_ret;
    int64_t ts_before, ts_after, ts_mid, ts_diff;
    int data_cnt = 0;
    int delay_cnt = 0;

    if (hfdev) {
        client = hf_device_get_private_data(hfdev);
    } else {
        LOG_ERR("NULL hfdev");
        goto err;
    }
    if (client) {
        sd = i2c_get_clientdata(client);
    } else {
        LOG_ERR("NULL client");
        goto err;
    }
    if (sd) {
        bu24721 = sd_to_bu24721_ois(sd);
    } else {
        LOG_ERR("NULL sd");
        goto err;
    }
    if (bu24721) {
        manager = bu24721->hf_dev.manager;
    } else {
        LOG_ERR("NULL bu24721");
        goto err;
    }
    if (!manager) {
        LOG_ERR("NULL manager");
        goto err;
    }

    if (poll_status(client) != 1) {
        LOG_ERR("%s sample poll failed. device not ready, return");
        goto err;
    }

    memset(&all_data, 0, sizeof(all_data));
    memset(&event, 0, sizeof(struct hf_manager_event));
    ts_before = ktime_get_boottime_ns();
    i2c_ret = adaptor_i2c_rd_p8(client, client->addr, 0xF200,
                                &all_data.data[1], 48 * GROUPS + 4);
    ts_after = ktime_get_boottime_ns();
    ts_diff = ts_after - ts_before;
    ts_mid = ts_before + ts_diff / 3;
    OIS_LOG_IF(buffer_dump, "ts_before[%lld] ts_after[%lld] interval_us[%lld]", ts_before, ts_after, ts_diff / 1000);

    if (i2c_ret < 0) {
        LOG_ERR("read F200 i2c error");
        goto err;
    }

    if (all_data.d.n_value > 12) {
        data_cnt = 12;
    } else {
        data_cnt = all_data.d.n_value;
    }
    if (data_cnt < 12) {
        delay_cnt = endian(all_data.d.hall[data_cnt * 2]);
    } else {
        delay_cnt = endian(all_data.d.delay_cnt);
    }
    ts_mid = get_timestamp(delay_cnt, data_cnt, ts_mid, ts_diff);

    event.sensor_type = SENSOR_TYPE_OIS1;
    event.accurancy = SENSOR_ACCURANCY_HIGH;
    event.action = DATA_ACTION;
    // [0] [1] hall
    // [2] [3] pixel shift
    // [4] [5] gyro
    for (i = 0; i < data_cnt; i++) {
        ts_mid += OIS_HALL_INTERVAL;
        event.timestamp = ts_mid;
        OIS_LOG_IF(buffer_dump, "ts[%lld]", event.timestamp);
        for (j = 0; j < GROUPS; j++) {
            event.word[2 * j] = (int)convert(endian(all_data.d.hall[2 * i * GROUPS + 2 * j]));
            event.word[2 * j + 1] = (int)convert(endian(all_data.d.hall[2 * i * GROUPS + 2 * j + 1]));
            event.word[2 * j + 2] = stroke_convert(event.word[2 * j]);
            event.word[2 * j + 3] = stroke_convert(event.word[2 * j + 1]) * -1;
            event.word[2 * j] = event.word[2 * j] * 10000;
            event.word[2 * j + 1] = event.word[2 * j + 1] * 10000;
            OIS_LOG_IF(buffer_dump, "eisbuffer event.word[%d] index[%d] endian[%u] convert[%d] pixel shift[%d]",2 * j,2 * i * GROUPS + 2 * j, endian(all_data.d.hall[2 * i * GROUPS + 2 * j]), event.word[2 * j], event.word[2 * j + 2]);
            OIS_LOG_IF(buffer_dump, "eisbuffer event.word[%d] index[%d] endian[%u] convert[%d] pixel shift[%d]",2 * j + 1,2 * i * GROUPS + 2 * j + 1, endian(all_data.d.hall[2 * i * GROUPS + 2 * j + 1]), event.word[2 * j + 1], event.word[2 * j + 3]);
        }
        manager->report(manager, &event);
    }
    spin_lock(&ois_spinlock);
    ts_static = ts_mid;
    spin_unlock(&ois_spinlock);

    manager->complete(manager);

    return 0;

err:
    return -1;
}

#endif /*NEW_SAMPLE*/

#ifdef HYBRID_SAMPLE
static int64_t get_timestamp(int delay_cnt, int data_cnt, int64_t ts, int64_t ts_diff) {
    int64_t diff;
    int delay_time;
    int min_threshold_time, max_threshold_time;

    diff = ts - ts_static;
    delay_time = delay_cnt * 17777;
    min_threshold_time = OIS_HALL_INTERVAL * (data_cnt - 3) + delay_time;
    max_threshold_time = OIS_HALL_INTERVAL * (data_cnt + 2) + delay_time;
    if (ts_diff > 2000000) {
        OIS_LOG_IF(buffer_dump, "i2c delay too large, spend[%lld] data_cnt[%d] diff_us[%lld]", ts_diff, data_cnt, diff/1000);
    }
    OIS_LOG_IF(buffer_dump, "diff_us[%lld] ts_ms[%lld] cnt[%d] delay_time_us[%d]", diff/1000, ts/1000000, data_cnt, delay_time/1000);

    if (ts_diff < 800000 || diff > max_threshold_time * 10) {
        ts_static = ts - delay_time - data_cnt * OIS_HALL_INTERVAL;
        OIS_LOG_IF(buffer_dump, "reset timestamp[%lld]  diff_us[%lld] delay_cnt[%d] data_cnt[%d] threshold_time_us[%d, %d]", ts_static, diff/1000, delay_cnt, data_cnt, min_threshold_time/1000, max_threshold_time/1000);
    // } else {
    //     LOG_ERR("ts_diff[%lld] too large, wait next time or diff[%lld] too large", ts_diff, diff);
    }

    return ts_static;
}

static int bu24721_sample(struct hf_device *hfdev)
{
    struct i2c_client *client;
    struct v4l2_subdev *sd;
    struct bu24721_device *bu24721;
    struct hf_manager *manager;
    struct hf_manager_event event;
    union bu24721_hall_data all_data;
    int i = 0, j = 0;
    int i2c_ret;
    int64_t ts_before, ts_after, ts_mid, ts_diff;
    int data_cnt = 0;
    int delay_cnt = 0;

    if (hfdev) {
        client = hf_device_get_private_data(hfdev);
    } else {
        LOG_ERR("NULL hfdev");
        goto err;
    }
    if (client) {
        sd = i2c_get_clientdata(client);
    } else {
        LOG_ERR("NULL client");
        goto err;
    }
    if (sd) {
        bu24721 = sd_to_bu24721_ois(sd);
    } else {
        LOG_ERR("NULL sd");
        goto err;
    }
    if (bu24721) {
        manager = bu24721->hf_dev.manager;
    } else {
        LOG_ERR("NULL bu24721");
        goto err;
    }
    if (!manager) {
        LOG_ERR("NULL manager");
        goto err;
    }

    if (poll_status(client) != 1) {
        LOG_ERR("sample poll failed. device not ready, return");
        goto err;
    }

    memset(&all_data, 0, sizeof(all_data));
    memset(&event, 0, sizeof(struct hf_manager_event));
    ts_before = ktime_get_boottime_ns();
    i2c_ret = adaptor_i2c_rd_p8(client, client->addr, 0xF200,
                                &all_data.data[1], 48 * GROUPS + 4);
    ts_after = ktime_get_boottime_ns();
    ts_diff = ts_after - ts_before;
    ts_mid = ts_before + ts_diff / 3;
    OIS_LOG_IF(buffer_dump, "ts_before[%lld] ts_after[%lld] interval_us[%lld]", ts_before, ts_after, ts_diff / 1000);

    if (i2c_ret < 0) {
        LOG_ERR("read F200 i2c error");
        goto err;
    }

    if (all_data.d.n_value > 12) {
        data_cnt = 12;
    } else {
        data_cnt = all_data.d.n_value;
    }
    if (data_cnt < 12) {
        delay_cnt = endian(all_data.d.hall[data_cnt * 2]);
    } else {
        delay_cnt = endian(all_data.d.delay_cnt);
    }
    ts_mid = get_timestamp(delay_cnt, data_cnt, ts_mid, ts_diff);

    event.sensor_type = SENSOR_TYPE_OIS1;
    event.accurancy = SENSOR_ACCURANCY_HIGH;
    event.action = DATA_ACTION;
    // [0] [1] hall
    // [2] [3] pixel shift
    // [4] [5] gyro
    for (i = 0; i < data_cnt; i++) {
        ts_mid += OIS_HALL_INTERVAL;
        event.timestamp = ts_mid;
        OIS_LOG_IF(buffer_dump, "ts[%lld]", event.timestamp);
        for (j = 0; j < GROUPS; j++) {
            event.word[2 * j] = (int)convert(endian(all_data.d.hall[2 * i * GROUPS + 2 * j]));
            event.word[2 * j + 1] = (int)convert(endian(all_data.d.hall[2 * i * GROUPS + 2 * j + 1]));
            event.word[2 * j + 2] = stroke_convert(event.word[2 * j]);
            event.word[2 * j + 3] = stroke_convert(event.word[2 * j + 1]) * -1;
            event.word[2 * j] = event.word[2 * j] * 10000;
            event.word[2 * j + 1] = event.word[2 * j + 1] * 10000;
            OIS_LOG_IF(buffer_dump, "eisbuffer event.word[%d] index[%d] endian[%u] convert[%d] pixel shift[%d]",2 * j,2 * i * GROUPS + 2 * j, endian(all_data.d.hall[2 * i * GROUPS + 2 * j]), event.word[2 * j], event.word[2 * j + 2]);
            OIS_LOG_IF(buffer_dump, "eisbuffer event.word[%d] index[%d] endian[%u] convert[%d] pixel shift[%d]",2 * j + 1,2 * i * GROUPS + 2 * j + 1, endian(all_data.d.hall[2 * i * GROUPS + 2 * j + 1]), event.word[2 * j + 1], event.word[2 * j + 3]);
        }
        manager->report(manager, &event);
    }
    spin_lock(&ois_spinlock);
    ts_static = ts_mid;
    spin_unlock(&ois_spinlock);

    manager->complete(manager);

    return 0;

err:
    return -1;
}

#endif /*HYBRID_SAMPLE*/


static int bu24721_custom_cmd(struct hf_device *hfdev, int sensor_type,
                              struct custom_cmd *cust_cmd)
{
    int i = 0;
    OIS_LOG_IF(dbg, "%s cammand[%x], type[%d] rxlen[%d] txlen[%d]", __func__,
            cust_cmd->command, sensor_type, cust_cmd->rx_len,
            cust_cmd->tx_len);
    for (i = 0; i < cust_cmd->tx_len; i++) {
        // printk(KERN_CONT "%s cust_cmd %x ", __func__, cust_cmd->data[i]);
    }
    if (cust_cmd->command == 0xdc) {
        cust_cmd->rx_len = 2;
        // memcpy(cust_cmd->data, testdata, 2 * sizeof(int));
        cust_cmd->data[0] = g_gyro_offset[0] * 114;
        cust_cmd->data[1] = g_gyro_offset[1] * 114;
    }
    // save gain to flash
    if (cust_cmd->command == 0x83) {
        LOG_ERR("save gain[%d,%d] len[%d]", cust_cmd->data[0], cust_cmd->data[1], cust_cmd->tx_len);
        g_gyro_gain_x = (unsigned short)cust_cmd->data[0];
        g_gyro_gain_y = (unsigned short)cust_cmd->data[1];
        Gyro_gain_set(g_gyro_gain_x, g_gyro_gain_y);
        WriteGyroGainToFlash();
    }
    return 0;
}

static const struct v4l2_subdev_internal_ops bu24721_int_ops = {
    .open = bu24721_open,
    .close = bu24721_close,
};

static struct v4l2_subdev_core_ops bu24721_ops_core = {
    .ioctl = bu24721_ops_core_ioctl,
};

static const struct v4l2_subdev_ops bu24721_ops = {
    .core = &bu24721_ops_core,
};

static void bu24721_subdev_cleanup(struct bu24721_device *bu24721)
{
    v4l2_async_unregister_subdev(&bu24721->sd);
    v4l2_ctrl_handler_free(&bu24721->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&bu24721->sd.entity);
#endif
}

static int bu24721_init_controls(struct bu24721_device *bu24721)
{
    struct v4l2_ctrl_handler *hdl = &bu24721->ctrls;
    /* const struct v4l2_ctrl_ops *ops = &bu24721_ois_ctrl_ops; */

    v4l2_ctrl_handler_init(hdl, 1);

    if (hdl->error) {
        OIS_LOG_IF(dbg, "%s init err", __func__);
        return hdl->error;
    }

    bu24721->sd.ctrl_handler = hdl;

    return 0;
}
#ifdef test_cmd
static ssize_t test_app1_cmd(char *buf, int sensor_type, int action,
                             unsigned int request)
{
    ssize_t ret = 0;
    struct hf_client *client = NULL;
    struct hf_manager_cmd cmd;
    struct hf_manager_event data[1];

    client = hf_client_create();
    if (!client) {
        OIS_LOG_IF(dbg, "hf_client_create fail\n");
        return -ENOMEM;
    }
    ret = hf_client_find_sensor(client, sensor_type);
    if (ret < 0) {
        OIS_LOG_IF(dbg, "hf_client_find_sensor %u fail\n", sensor_type);
        goto out;
    }
    hf_client_request_sensor_cali(client, sensor_type, request, true);
    memset(&cmd, 0, sizeof(cmd));
    cmd.sensor_type = sensor_type;
    cmd.action = action;
    ret = hf_client_control_sensor(client, &cmd);
    if (ret < 0) {
        OIS_LOG_IF(dbg, "hf_client_control_sensor %u %u fail\n", sensor_type,
                action);
        goto out;
    }
    ret = hf_client_poll_sensor_timeout(client, data, ARRAY_SIZE(data),
                                        msecs_to_jiffies(3000));
    hf_client_request_sensor_cali(client, sensor_type, request, false);
    if (ret >= 0)
        ret = sprintf(buf, "[%d,%d,%d,%lld,%d,%d,%d]\n",
                      data[0].sensor_type, data[0].action,
                      data[0].accurancy, data[0].timestamp,
                      data[0].word[0], data[0].word[1],
                      data[0].word[2]);

    // disable sensor
    cmd.action = 0;
    ret = hf_client_control_sensor(client, &cmd);
    if (ret < 0) {
        OIS_LOG_IF(dbg, "hf_client_control_sensor %u %u fail\n", sensor_type,
                action);
        goto out;
    }
out:
    hf_client_destroy(client);
    return ret;
}

static ssize_t test_app1_cust(char *buf, int sensor_type, int action)
{
    ssize_t ret = 0;
    struct hf_client *client = NULL;
    struct custom_cmd cmd;

    client = hf_client_create();
    if (!client) {
        OIS_LOG_IF(dbg, "hf_client_create fail\n");
        return -ENOMEM;
    }
    ret = hf_client_find_sensor(client, sensor_type);
    if (ret < 0) {
        OIS_LOG_IF(dbg, "hf_client_find_sensor %u fail\n", sensor_type);
        goto out;
    }
    memset(&cmd, 0, sizeof(cmd));
    cmd.command = action;
    cmd.tx_len = 0;
    cmd.rx_len = 48;
    ret = hf_client_custom_cmd(client, sensor_type, &cmd);
    if (ret >= 0)
        ret = sprintf(buf, "[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]\n",
                      cmd.data[0], cmd.data[1], cmd.data[2],
                      cmd.data[3], cmd.data[4], cmd.data[5],
                      cmd.data[6], cmd.data[7], cmd.data[8],
                      cmd.data[9], cmd.data[10], cmd.data[11]);
out:
    hf_client_destroy(client);
    return ret;
}

#endif

struct test_app_t {
    struct task_struct *task;
    struct hf_client *client;
    int sensor_type;
    int sensor_type_gyro;
    int val1;
    int val2;
};


// #define test_thread 1



#ifdef test_thread
static struct test_app_t test_app = {.sensor_type = SENSOR_TYPE_OIS1,
                                     .sensor_type_gyro = SENSOR_TYPE_GYROSCOPE,
                                     .client = 0};

static int test_app_kthread(void *arg)
{
    struct hf_client *client = NULL;
    struct hf_manager_event data[4];
    int size = 0, i = 0;
    int ret;
    struct hf_manager_cmd cmd;
    union batch {
        int64_t time;
        int8_t data[8];
    };
    union batch *p_batch;
    int loop_cnt = 0;
    int sensor_type = test_app.sensor_type;

    if (!test_app.client) {
        client = hf_client_create();
        if (!client) {
            LOG_ERR("hf_client_create fail\n");
            goto ex;
        }
        test_app.client = client;
    }

    ret = hf_client_find_sensor(client, test_app.sensor_type);
    if (ret < 0) {
        OIS_LOG_IF(dbg, "hf_client_find_sensor %u fail\n",
                test_app.sensor_type);
        goto ex;
    }

    ret = hf_client_find_sensor(client, test_app.sensor_type_gyro);
    if (ret < 0) {
        OIS_LOG_IF(dbg, "hf_client_find_sensor %u fail\n",
                test_app.sensor_type_gyro);
        // goto ex;
    }

    OIS_LOG_IF(dbg, "size of union batch [%d]  cam.data[%d]\n", sizeof(union batch),
            sizeof(cmd.data));

    memset(&cmd, 0, sizeof(cmd));
    cmd.sensor_type = test_app.sensor_type;
    cmd.action = 1;
    p_batch = (union batch *)(cmd.data);
    p_batch->time = 500 * 1000000;
    p_batch++;
    p_batch->time = 500 * 1000000;
    ret = hf_client_control_sensor(client, &cmd);
    if (ret < 0) {
        OIS_LOG_IF(dbg, "hf_client_control_sensor %u action[%u] fail\n",
                sensor_type, cmd.action);
        goto ex;
    }

    // usleep_range(10 * 1000 * 1000, 12 * 1000 * 1000);

#if 0
    // enable gyro
    memset(&cmd, 0, sizeof(cmd));
    cmd.sensor_type = test_app.sensor_type_gyro;
    cmd.action = 1;
    p_batch = (union batch *)(cmd.data);
    p_batch->time = 2 * 1000000;
    p_batch++;
    p_batch->time = 2 * 1000000;
    ret = hf_client_control_sensor(client, &cmd);
    if (ret < 0) {
        OIS_LOG_IF(dbg, "hf_client_control_sensor %u action[%u] fail\n",
                test_app.sensor_type_gyro, cmd.action);
        // goto ex;
    }

    cmd.sensor_type = SENSOR_TYPE_OIS1;
    cmd.action = 1;
    p_batch = (union batch *)(cmd.data);
    p_batch->time = 10 * 1000000;
    p_batch++;
    p_batch->time = 10 * 1000000;
    ret = hf_client_control_sensor(client, &cmd);
    if (ret < 0) {
        OIS_LOG_IF(dbg, "hf_client_control_sensor %u action[%u] fail\n",
                SENSOR_TYPE_OIS1, cmd.action);
        // goto ex;
    } else {
        OIS_LOG_IF(dbg, "hf_client_control_sensor %u action[%u] seccuss\n",
                SENSOR_TYPE_OIS1, cmd.action);
    }
#endif

    while (1) {
        memset(data, 0, sizeof(data));
        size = hf_client_poll_sensor(client, data, ARRAY_SIZE(data));
        // size = hf_client_poll_sensor_timeout(client, data, ARRAY_SIZE(data), msecs_to_jiffies(3000));
        if (size < 0)
            continue;
        for (i = 0; i < size; ++i) {
            if (data[i].sensor_type == SENSOR_TYPE_OIS1) {
                OIS_LOG_IF(dbg, "sois [%dth] [%d,%d,%lld,gyro x 0x%x,gyro y 0x%x,tg x 0x%x,tg y 0x%x,hall x 0x%x,hall y 0x%x]\n",
                        i, data[i].sensor_type, data[i].action,
                        data[i].timestamp, data[i].word[0],
                        data[i].word[1], data[i].word[2],
                        data[i].word[3], data[i].word[4],
                        data[i].word[5]);
            }
            if (data[i].sensor_type == SENSOR_TYPE_OIS1) {
                OIS_LOG_IF(dbg, "bu24721 [%dth] [%d,%d,%lld] g[%d,%d] t[%d,%d] h[%d,%d]\n",
                        i, data[i].sensor_type, data[i].action,
                        data[i].timestamp, data[i].word[0],
                        data[i].word[1], data[i].word[2],
                        data[i].word[3], data[i].word[4],
                        data[i].word[5]);
                loop_cnt++;
            }
        }
        if (loop_cnt > 300)
            goto ex;
    }
ex:
    // memset(&cmd, 0, sizeof(cmd));
    // cmd.sensor_type = test_app.sensor_type;
    // //disable
    // cmd.action = 0;
    // p_batch = (union batch *)(cmd.data);
    // p_batch->time = 24 * 1000000;
    // p_batch++;
    // p_batch->time = 24 * 1000000;
    // ret = hf_client_control_sensor(client, &cmd);
    // if (ret < 0) {
    //     OIS_LOG_IF(dbg, "hf_client_control_sensor %u action[%u] fail\n",
    //         sensor_type, cmd.action);
    // }
    do_exit(1);
    return 0;
}
#endif
struct bu24721_device *bu24721;

static int bu24721_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    int ret;
    int err = 0;
#ifdef test_cmd
    char buf[100] = {0};
#endif

    LOG_ERR("%s\n", __func__);

    bu24721 = devm_kzalloc(dev, sizeof(*bu24721), GFP_KERNEL);
    if (!bu24721)
        return -ENOMEM;

    bu24721->vin = devm_regulator_get(dev, "iovdd");
    if (IS_ERR(bu24721->vin)) {
        ret = PTR_ERR(bu24721->vin);
        if (ret != -EPROBE_DEFER)
            LOG_ERR("cannot get vin regulator\n");
        return ret;
    }

    bu24721->vdd = devm_regulator_get(dev, "avdd");
    if (IS_ERR(bu24721->vdd)) {
        ret = PTR_ERR(bu24721->vdd);
        if (ret != -EPROBE_DEFER)
            LOG_ERR("cannot get vdd regulator\n");
        return ret;
    }

    bu24721->vcamaf_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR(bu24721->vcamaf_pinctrl)) {
        ret = PTR_ERR(bu24721->vcamaf_pinctrl);
        bu24721->vcamaf_pinctrl = NULL;
        OIS_LOG_IF(dbg, "cannot get pinctrl\n");
    } else {
        bu24721->vcamaf_on = pinctrl_lookup_state(
            bu24721->vcamaf_pinctrl, "vcamaf_on");

        if (IS_ERR(bu24721->vcamaf_on)) {
            ret = PTR_ERR(bu24721->vcamaf_on);
            bu24721->vcamaf_on = NULL;
            LOG_ERR("cannot get vcamaf_on pinctrl\n");
        }

        bu24721->vcamaf_off = pinctrl_lookup_state(
            bu24721->vcamaf_pinctrl, "vcamaf_off");

        if (IS_ERR(bu24721->vcamaf_off)) {
            ret = PTR_ERR(bu24721->vcamaf_off);
            bu24721->vcamaf_off = NULL;
            LOG_ERR("cannot get vcamaf_off pinctrl\n");
        }
    }

    v4l2_i2c_subdev_init(&bu24721->sd, client, &bu24721_ops);
    bu24721->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    bu24721->sd.internal_ops = &bu24721_int_ops;

    ret = bu24721_init_controls(bu24721);
    if (ret)
        goto err_init;

    bu24721->hf_dev.dev_name = "bu24721_ois";
    bu24721->hf_dev.device_poll = HF_DEVICE_IO_POLLING;
    bu24721->hf_dev.device_bus = HF_DEVICE_IO_SYNC;
    bu24721->hf_dev.support_list = support_sensors;
    bu24721->hf_dev.support_size = ARRAY_SIZE(support_sensors);
    bu24721->hf_dev.enable = bu24721_enable;
    bu24721->hf_dev.batch = bu24721_batch;
    bu24721->hf_dev.sample = bu24721_sample;
    bu24721->hf_dev.custom_cmd = bu24721_custom_cmd;
    bu24721->hf_dev.config_cali = bu24721_config_cali;
    hf_device_set_private_data(&bu24721->hf_dev, client);
    err = hf_device_register_manager_create(&bu24721->hf_dev);
    if (err < 0) {
        LOG_ERR( "%s hf_manager_create fail\n", __func__);
        err = -1;
        goto err_cleanup;
    }
    spin_lock_init(&ois_spinlock);
    ret = device_create_file(dev, &dev_attr_bu24721_dbg);
    if (ret)
        LOG_ERR("failed to create sysfs bu24721_dbg\n");
    ret = device_create_file(dev, &dev_attr_bu24721_i2c_ops);
    if (ret)
        LOG_ERR("failed to create sysfs bu24721_i2c_ops\n");
    ret = device_create_file(dev, &dev_attr_bu24721_i2c_ops32);
    if (ret)
        LOG_ERR("failed to create sysfs bu24721_i2c_ops32\n");

#if defined(CONFIG_MEDIA_CONTROLLER)
    ret = media_entity_pads_init(&bu24721->sd.entity, 0, NULL);
    if (ret < 0)
        goto err_cleanup;

    bu24721->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

    ret = v4l2_async_register_subdev(&bu24721->sd);
    if (ret < 0)
        goto err_cleanup;

    gyro_client = hf_client_create();

    ret = bu24721_power_on(bu24721);
    if (ret < 0) {
        LOG_ERR("bu24721 init failed caused I2C failed.");
        goto err_cleanup;
    }
    ret = Rohm_bu24721_fw_download();
    if (ret < 0) {
        LOG_ERR("updata FW failed!");
    }
    bu24721_power_off(bu24721);

    pm_runtime_enable(dev);

#ifdef test_cmd

    memset(buf, 0, sizeof(buf));
    test_app1_cmd(buf, SENSOR_TYPE_OIS1, 1,
                  HF_MANAGER_REQUEST_TEST_DATA);
    OIS_LOG_IF(dbg, "bu24721  %s", buf);

    memset(buf, 0, sizeof(buf));
    test_app1_cust(buf, SENSOR_TYPE_OIS1, 0);
    OIS_LOG_IF(dbg, "bu24721 cust %s", buf);
#endif
#ifdef test_thread
    test_app.task = kthread_run(test_app_kthread, &test_app, "test_app");
    if (IS_ERR(test_app.task))
        LOG_ERR("kthread_run create fail\n");
#endif

    return 0;

err_cleanup:
    hf_device_unregister_manager_destroy(&bu24721->hf_dev);
err_init:
    bu24721_subdev_cleanup(bu24721);
    OIS_LOG_IF(dbg, "bu24721 err_cleanup");
    return ret;
}

static void bu24721_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct bu24721_device *bu24721 = sd_to_bu24721_ois(sd);
    struct device *dev = &client->dev;
    LOG_ERR("%s\n", __func__);

    bu24721_subdev_cleanup(bu24721);
    pm_runtime_disable(&client->dev);
    device_remove_file(dev, &dev_attr_bu24721_dbg);
    device_remove_file(dev, &dev_attr_bu24721_i2c_ops);
    device_remove_file(dev, &dev_attr_bu24721_i2c_ops32);

    if (gyro_client) {
        hf_client_destroy(gyro_client);
    }

    if (!pm_runtime_status_suspended(&client->dev))
        bu24721_power_off(bu24721);
    pm_runtime_set_suspended(&client->dev);

    return ;
}

static int __maybe_unused bu24721_ois_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct bu24721_device *bu24721 = sd_to_bu24721_ois(sd);

    return bu24721_power_off(bu24721);
}

static int __maybe_unused bu24721_ois_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct bu24721_device *bu24721 = sd_to_bu24721_ois(sd);

    return bu24721_power_on(bu24721);
}

static const struct i2c_device_id bu24721_id_table[] = {
    {BU24721_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, bu24721_id_table);

static const struct of_device_id bu24721_of_table[] = {
    {.compatible = "mediatek,bu24721"},
    {},
};
MODULE_DEVICE_TABLE(of, bu24721_of_table);

static const struct dev_pm_ops bu24721_pm_ops = {SET_SYSTEM_SLEEP_PM_OPS(
    pm_runtime_force_suspend,
    pm_runtime_force_resume) SET_RUNTIME_PM_OPS(bu24721_ois_suspend,
                                                bu24721_ois_resume, NULL)};

static struct i2c_driver bu24721_i2c_driver = {
    .driver = {
        .name = BU24721_NAME,
        // .pm = &bu24721_pm_ops,
        .of_match_table = bu24721_of_table,
    },
    .probe_new = bu24721_probe,
    .remove = bu24721_remove,
    .id_table = bu24721_id_table,
};

module_i2c_driver(bu24721_i2c_driver);

MODULE_AUTHOR("Dont have");
MODULE_DESCRIPTION("BU24721 OIS driver");
MODULE_LICENSE("GPL v2");
