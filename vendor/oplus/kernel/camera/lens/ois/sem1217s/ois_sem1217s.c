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
#include "sem1217s_fw.h"
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
extern int sem1217s_fw_download(void);

#define DRIVER_NAME "sem1217s"

#define LOG_INF(format, args...) \
    pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define LOG_ERR(format, args...) \
    pr_err(DRIVER_NAME " [%s] " format, __func__, ##args)

#define OIS_LOG_IF(cond, ...)  do { if ( (cond) ) { LOG_INF(__VA_ARGS__); } }while(0)

#define SEM1217S_NAME "sem1217s"

#define SEM1217S_I2C_SLAVE_ADDR 0XC2

#define Pro_ID_0 0x45230200

#define SEM1217S_CTRL_DELAY_US 10000

#define POWER_OFF_CNT 5

//*** SPI MODE ***//
#define SPI_Master 0x00
#define SPI_Monitor 0x04

#define endian(var) (((var << 8) & 0xff00) | ((var >> 8) & 0xff))

#define GROUPS 1

#define REG_MONITOR

#define REG_MONITOR_WAIT (30 * 1000)

#define SEM1217S_EIS_DATA_SIZE (44)
#define OIS_HALL_DATA_SET_COUNT 11
#define OIS_HALL_DATA_SET_SIZE 4

#define INPUT_MIN (4000)
#define INPUT_MAX (28000)
#define OUTPUT_MIN (-2048)
#define OUTPUT_MAX (2048)
#define I2C_TIME (492 * 1000)
#define OIS_HALL_INTERVAL (1972300)
#define X_GAIN_GOLDEN 0xb4c8563f
#define Y_GAIN_GOLDEN 0x2731483f

static struct sensor_info support_sensors[] = {
    {
        .sensor_type = SENSOR_TYPE_OIS1,
        .gain = 1,
        .name = {'t', 'e', 'l', 'e', 'o', 'i', 's'},
        .vendor = {'o', 'p', 'l', 'u', 's'},
    },
};

static int g_gyro_offset[] = {0xdc, 0x18};
static unsigned short g_gyro_gain_x = 0;
static unsigned short g_gyro_gain_y = 0;
static int g_still_en = 0;
static int dbg = 0;
static int g_gyro_cal = 0;
static struct hf_client *gyro_client = NULL;
static int buffer_dump = 0;
static int64_t g_last_time = 0;
static int64_t ts_static = 0;
static uint32_t g_gain_x = 0;
static uint32_t g_gain_y = 0;
static bool sleep_mode_state = false;
static spinlock_t ois_spinlock;

/* sem1217s device structure */
struct sem1217s_device {
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
#include "sem1217s_if.h"

/* Control commnad */
#define VIDIOC_MTK_S_OIS_MODE _IOW('V', BASE_VIDIOC_PRIVATE + 2, int32_t)

#define VIDIOC_MTK_G_OIS_POS_INFO \
    _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct mtk_ois_pos_info)

static inline struct sem1217s_device *to_sem1217s_ois(struct v4l2_ctrl *ctrl)
{
    return container_of(ctrl->handler, struct sem1217s_device, ctrls);
}

static inline struct sem1217s_device *
sd_to_sem1217s_ois(struct v4l2_subdev *subdev)
{
    return container_of(subdev, struct sem1217s_device, sd);
}

static int sem1217s_release(struct sem1217s_device *sem1217s)
{
    return 0;
}

int switch2flash(struct i2c_client *client)
{
    return 0;
}

static struct i2c_ops_info inf = {"sem1217s", 0, 0};

#define SHOW(buf, len, fmt, ...) { \
    len += snprintf(buf + len, PAGE_SIZE - len, fmt, ##__VA_ARGS__); \
}

static ssize_t sem1217s_i2c_ops_show(struct device *dev,
               struct device_attribute *attr, char *buf)
{
    int len = 0;

    SHOW(buf, len, "%s i2c read 0x%08x = 0x%08x\n",
            inf.name,
            inf.RegAddr,
            inf.RegData);
    return len;
}


static ssize_t sem1217s_i2c_ops_store(struct device *dev,
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


static DEVICE_ATTR_RW(sem1217s_i2c_ops);

static ssize_t sem1217s_i2c_ops32_show(struct device *dev,
               struct device_attribute *attr, char *buf)
{
    int len = 0;

    SHOW(buf, len, "%s i2c read 0x%08x = 0x%08x\n",
            inf.name,
            inf.RegAddr,
            inf.RegData);
    return len;
}


static ssize_t sem1217s_i2c_ops32_store(struct device *dev,
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
static DEVICE_ATTR_RW(sem1217s_i2c_ops32);

static ssize_t sem1217s_dbg_show(struct device *dev, struct device_attribute *attr, char *buf) {
    return scnprintf(buf, PAGE_SIZE, "%d\n", dbg);
}

static ssize_t sem1217s_dbg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
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

static DEVICE_ATTR_RW(sem1217s_dbg);

static int sem1217s_set_ctrl(struct v4l2_ctrl *ctrl)
{
    /* struct sem1217s_device *sem1217s = to_sem1217s_ois(ctrl); */

    return 0;
}

static const struct v4l2_ctrl_ops sem1217s_ois_ctrl_ops = {
    .s_ctrl = sem1217s_set_ctrl,
};

static int sem1217s_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
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

static int sem1217s_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    OIS_LOG_IF(dbg, "%s\n", __func__);

    pm_runtime_put(sd->dev);

    return 0;
}

static long sem1217s_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
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
    char gyro_computed_raw[4] = {0, 0, 0, 0};
    char hall_raw[4] = {0, 0, 0, 0};
    struct v4l2_subdev *sd;
    struct sem1217s_device *sem1217s;
    sd = i2c_get_clientdata(client);
    sem1217s = sd_to_sem1217s_ois(sd);

    while(!kthread_should_stop()) {
        if (dbg) {
            // adaptor_i2c_rd_u16(client, client->addr, 0x0601, &ois_error);
            // OIS_LOG_IF(dbg, "ois error [0x%x]", ois_error);
            // print gyro data form monitor
            adaptor_i2c_rd_p8(client, client->addr, SEM1217S_REG_GYRO_X, &gyro_raw[0], 2);

            adaptor_i2c_rd_p8(client, client->addr, SEM1217S_REG_GYRO_Y, &gyro_raw[2], 2);
            OIS_LOG_IF(dbg, "gyro raw code [0x%x, 0x%x]", (u16)gyro_raw[1] << 8 | gyro_raw[0],
                    (u16)gyro_raw[3] << 8 | gyro_raw[2]);
            if (g_still_en) {

                adaptor_i2c_rd_p8(client, client->addr, SEM1217S_REG_GYRO_COMPUTED_X, &gyro_computed_raw[0], 2);

                adaptor_i2c_rd_p8(client, client->addr, SEM1217S_REG_GYRO_COMPUTED_Y, &gyro_computed_raw[2], 2);
                OIS_LOG_IF(dbg, "gyro computed code [0x%x 0x%x]",
                        (u16)gyro_computed_raw[1] << 8 | gyro_computed_raw[0],
                        (u16)gyro_computed_raw[3] << 8 | gyro_computed_raw[2]);

                adaptor_i2c_rd_p8(client, client->addr, SEM1217S_REG_SX_OUT, &hall_raw[0], 2);

                adaptor_i2c_rd_p8(client, client->addr, SEM1217S_REG_SY_OUT, &hall_raw[2], 2);
                OIS_LOG_IF(dbg, "reg s out code [0x%x 0x%x]",
                        (u16)hall_raw[1] << 8 | hall_raw[0],
                        (u16)hall_raw[3] << 8 | hall_raw[2]);
            }
        }
        usleep_range(REG_MONITOR_WAIT, REG_MONITOR_WAIT + 1000);

    } //while
    LOG_ERR("reg_monitor_kthread return...");
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

void big_to_little_endian(uint32_t *g_gain)
{
    unsigned char table[4] = {0};
    table[0] = (*g_gain >> 24) & 0xff;
    table[1] = (*g_gain >> 16) & 0xff;
    table[2] = (*g_gain >> 8) & 0xff;
    table[3] = *g_gain & 0xff;
    *g_gain = table[3] << 24 | table[2] << 16 | table[1] << 8 | table[0];
}

static int sem1217s_init(struct sem1217s_device *sem1217s)
{
    struct i2c_client *client = v4l2_get_subdevdata(&sem1217s->sd);
    int ret = 0;
    unsigned char ois_ret;
    unsigned char data = 0;

    union {
        int checksum;
        unsigned char checksum_byte[4];
    } Prog_ID;
    int id;
    LOG_ERR("sem1217s_init E");
    client->addr = SEM1217S_I2C_SLAVE_ADDR >> 1;
    OIS_LOG_IF(dbg, "%s i2c_client %p addr 0x%x", __func__, client, client->addr);

    adaptor_i2c_rd_p8(client, client->addr, SEM1217S_REG_APP_VER, Prog_ID.checksum_byte,
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
    ret = adaptor_i2c_rd_p8(client, client->addr, 0x0603, &data, 1);

    if (data != 1) {
        ret = adaptor_i2c_wr_u8(client, client->addr, 0x0603, 0x01);
        LOG_ERR("gyro mode 0x%x need to set !",data);
    }

    // reads the value of OIS_STS register to confirm
    // whether the status of SEM1217S is READY Status or not.
    ret = adaptor_i2c_rd_p8(client, client->addr, SEM1217S_REG_OIS_STS, &ois_ret, 1);

    if (ret < 0) {
        LOG_ERR("read ois device not ready failed ret[%d]", ret);
    }
    if ((ois_ret & 0x01) != 1) {
        LOG_ERR("ois device not ready. need set ois_ret[%u]", ois_ret);
        ret = adaptor_i2c_wr_u8(client, client->addr, 0x0000, 0x00);
    }

    // set Gyro Mount Type
    ret = adaptor_i2c_wr_u8(client, client->addr, 0x0627, 0x01);
    if (ret < 0) {
        LOG_ERR("enable OIS failed ret[%d]", ret);
    }

    // enable OIS
    ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_MODE, 0x00);
    if (ret < 0) {
        LOG_ERR("enable OIS failed ret[%d]", ret);
    }

    // enable OIS control
    ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_CTRL, 0x1);
    if (ret < 0) {
        LOG_ERR("enable OIS control failed ret[%d]", ret);
    }

    // enable statmon
    ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_STATMON_CTRL, 0x01);
    if (ret < 0) {
        LOG_ERR("enable statmon failed ret[%d]", ret);
    }

    ret = adaptor_i2c_rd_p8(client, client->addr, 0x0603, &data, 1);

    LOG_ERR("now gyro mode 0x%x",data);

    //read gyro gain
    ret = adaptor_i2c_rd_u32(client, client->addr, SEM1217S_REG_GX_GAIN, &g_gain_x);
    if (ret < 0) {
        LOG_ERR("read gyro gainX failed ret[%d]", ret);
        g_gain_x = X_GAIN_GOLDEN;
    }
    ret =  adaptor_i2c_rd_u32(client, client->addr, SEM1217S_REG_GY_GAIN, &g_gain_y);
    if (ret < 0) {
        LOG_ERR("read gyro gainY failed ret[%d]", ret);
        g_gain_x = Y_GAIN_GOLDEN;
    }
    big_to_little_endian(&g_gain_x);
    big_to_little_endian(&g_gain_y);
    LOG_ERR("gain raw code [0x%x, 0x%x]", g_gain_x, g_gain_y);
    g_still_en = 0;
    LOG_ERR("sem1217s_init X");
    return 0;
}

static int sem1217s_power_on(struct sem1217s_device *sem1217s)
{
    int ret = 0;
    int ret1;

    LOG_ERR("%s E\n", __func__);

    ret = regulator_enable(sem1217s->vin);
    ret1 = regulator_set_voltage(sem1217s->vin, 1780000, 1820000);
    if (ret < 0 || ret1 < 0) {
        LOG_ERR("%s sem1217s->vin set failed ret[%d] ret1[%d]", __func__,
               ret, ret1);
        return -1;
    }

    usleep_range(5000, 5000 + 100);

    ret = regulator_enable(sem1217s->vdd);
    ret1 = regulator_set_voltage(sem1217s->vdd, 2780000, 2820000);
    if (ret < 0 || ret1 < 0) {
        LOG_ERR("%s sem1217s->vdd set failed ret[%d] ret1[%d]", __func__,
               ret, ret1);
        return -1;
    }

    if (sem1217s->vcamaf_pinctrl && sem1217s->vcamaf_on)
        ret = pinctrl_select_state(sem1217s->vcamaf_pinctrl,
                                   sem1217s->vcamaf_on);

    if (ret < 0)
        return ret;

    /*
     * TODO(b/139784289): Confirm hardware requirements and adjust/remove
     * the delay.
     */
    usleep_range(SEM1217S_CTRL_DELAY_US, SEM1217S_CTRL_DELAY_US + 100);

    ret = sem1217s_init(sem1217s);
    if (ret < 0) {
        LOG_ERR("sem1217s_init failed ret[%d] return", ret);
        // goto fail;
    }

    LOG_ERR("%s X\n", __func__);
    return 0;

// fail:
//     regulator_disable(sem1217s->vin);
//     regulator_disable(sem1217s->vdd);
//     if (sem1217s->vcamaf_pinctrl && sem1217s->vcamaf_off) {
//         pinctrl_select_state(sem1217s->vcamaf_pinctrl,
//                              sem1217s->vcamaf_off);
//     }
//     LOG_ERR("%s fail process X\n", __func__);
//     return ret;
}

/* Power handling */
static int sem1217s_power_off(struct sem1217s_device *sem1217s)
{
    int ret;

    LOG_ERR("%s E\n", __func__);
    g_gyro_cal = 0;

    ret = sem1217s_release(sem1217s);
    if (ret)
        LOG_ERR("sem1217s release failed!\n");

    ret = regulator_disable(sem1217s->vin);
    if (ret)
        return ret;

    ret = regulator_disable(sem1217s->vdd);
    if (ret)
        return ret;

    if (sem1217s->vcamaf_pinctrl && sem1217s->vcamaf_off)
        ret = pinctrl_select_state(sem1217s->vcamaf_pinctrl,
                                   sem1217s->vcamaf_off);

    LOG_ERR("%s X\n", __func__);
    return ret;
}

#endif /*REG_MONITOR*/

static int sem1217s_batch(struct hf_device *hfdev, int sensor_type,
                         int64_t delay, int64_t latency)
{
    pr_debug("%s id:%d delay:%lld latency:%lld\n", __func__, sensor_type,
             delay, latency);
    return 0;
}

static short convert(unsigned short value)
{
    short l_value = 0;
    l_value = (short)(value + 65536L);
    return l_value;
}

int map_value(int value) {

    int mapped_value = 0;
    int64_t format_value = 0;
    if (value == 0) {
        LOG_ERR("map_value input error %d!\n", value);
        return mapped_value;
    }
    format_value = (value - 16000) << 12;
    format_value = format_value *125LL /3LL;
    // Map the ratio to the output range -2048*1000000~2048*1000000
    mapped_value = (int)(format_value);
    OIS_LOG_IF(dbg, "%s value =%d ,format_value = %lld ,mapped_value =%d \n", __func__, value, format_value, mapped_value);

    return mapped_value;
}

int sem1217s_hall_convert(int value) {

    int mapped_value = 0;
    if (value > 2048) {
        mapped_value = value - 65536;
    } else {
        mapped_value = value;
    }
    OIS_LOG_IF(dbg, "%s value = %d, mapped_value =%d \n", __func__, value, mapped_value);

    return mapped_value;
}

static int sem1217s_enable(struct hf_device *hfdev, int sensor_type, int en)
{
    struct i2c_client *client;
    struct v4l2_subdev *sd;
    struct sem1217s_device *sem1217s;
    int ret = 0;

    if (0 == hfdev) {
        LOG_ERR("%s hfdev is null!! return error", __func__);
        return -1;
    }
    OIS_LOG_IF(dbg, "%s sem1217s hfdev[%p]", __func__, hfdev);
    client = hf_device_get_private_data(hfdev);
    sd = i2c_get_clientdata(client);
    sem1217s = sd_to_sem1217s_ois(sd);

    LOG_ERR("%s sensor_type[%d] en[%d]", __func__, sensor_type, en);
    if (en) {
        ret = sem1217s_power_on(sem1217s);
        if (ret < 0) {
            return -1;
        }
        g_last_time = 0;
#ifdef REG_MONITOR
        reg_monitor_thread_init(client);
#endif
    } else {
#ifdef REG_MONITOR
        reg_monitor_thread_exit();
#endif
        g_last_time = 0;
        sem1217s_power_off(sem1217s);
    }

    return 0;
}

int sem1217s_config_cali(struct hf_device *hfdev, int sensor_type, void *data,
                        uint8_t length)
{
    struct i2c_client *client;
    int ret = 0;
    unsigned short offset_x, offset_y;
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
        OIS_LOG_IF(dbg, "config->mode is [%d]", config->mode);
    } else {
        LOG_ERR("%s config data is null !!! return", __func__);
        return -1;
    }
    LOG_ERR("%s config->mode:%d", __func__, config->mode);
    switch (config->mode) {
    // add for center on before init
    case AK_Centering: {
        // add for center on before init
        LOG_ERR("into AK_Centering mode");
        ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_MODE, 0x03);
        if (ret < 0) {
            LOG_ERR("AK_Centering failed, ret[%d] return", ret);
            return -1;
        }
        LOG_ERR("AK_Centering on success");
    } break;
    case AK_WorkingMode: {
        LOG_ERR("into AK_Working mode");
        LOG_ERR("this mode to do");
    } break;
    case AK_StandbyMode: {
        LOG_ERR("into AK_Standby mode");
        LOG_ERR("this mode to do");
    } break;
    case AK_EnableMOIS: {
        if (sleep_mode_state) {
            ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_CTRL, 0x01);
            if (ret < 0) {
                LOG_ERR("start ois failed, ret[%d] return", ret);
                return -1;
            }
            sleep_mode_state = false;
            LOG_INF("exit ois standby success");
        }
    }
    fallthrough;
    case AK_Still: {
        LOG_ERR("into ois on mode");
        ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_MODE, 0x00);
        if (ret < 0) {
            LOG_ERR("ois on failed, ret[%d] return", ret);
            return -1;
        }
        g_still_en = 1;
        LOG_ERR("ois on success");
    } break;
    case AK_DisableMOIS:
    // fall through
    case AK_CenteringOn: {
        LOG_ERR("into center on mode");
        ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_MODE, 0x03);
        if (ret < 0) {
            LOG_ERR("center_on failed, ret[%d] return", ret);
            return -1;
        }
        LOG_ERR("center on success");
    } break;
    case AK_SleepMode: {
        if (!sleep_mode_state) {
            ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_CTRL, 0x00);
            if (ret < 0) {
                LOG_ERR("stop ois failed, ret[%d] return", ret);
                return -1;
            }
            sleep_mode_state = true;
            LOG_INF("enter ois standby success");
        }
    } break;
    case AK_ManualMovieLens: {
        LOG_ERR("into fixed on mode");
        ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_MODE, 0x0B);
        if (ret < 0) {
            LOG_ERR("fixed failed, ret[%d] return", ret);
            return -1;
        }
        LOG_ERR("fixed on success");
    } break;
    case AK_Pantilt: {
        LOG_ERR("into pantilt mode");
        ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_MODE, 0x00);
        if (ret < 0) {
            LOG_ERR("into pantilt failed, ret[%d] return", ret);
            return -1;
        }
        g_still_en = 1;
        LOG_ERR("into pantilt success");
    } break;
    case AK_TestMode: {
        if (1) {
            g_gyro_cal = 1;
            LOG_ERR("do gyro offset...");
            do_ois_cali(&offset_x, &offset_y);
            g_gyro_offset[0] = convert(offset_x);
            g_gyro_offset[1] = convert(offset_y);
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
            ret = adaptor_i2c_wr_u8(client, client->addr, SEM1217S_REG_OIS_MODE, 0x00);
            if (ret < 0) {
                LOG_ERR("Gyro Gain ois on failed, ret[%d] return", ret);
                return -1;
            }
            LOG_ERR("Gyro Gain ois on success");
            g_still_en = 1;
        }
        g_gyro_gain_x = (unsigned short)config->mois_gain_x;
        g_gyro_gain_y = (unsigned short)config->mois_gain_y;
        OIS_LOG_IF(dbg, "gyro_gain[%u,%u]", g_gyro_gain_x, g_gyro_gain_y);
        SEM1217S_Gyro_gain_set(g_gyro_gain_x, g_gyro_gain_y);
        LOG_ERR("into MOIS_Gyro_Gain_Cal");
    } break;
    default:
        OIS_LOG_IF(dbg, "into default mode just break");
        break;
    }

    return 0;
}

static int64_t get_timestamp(int32_t delay_time, int32_t data_cnt, int64_t ts_diff, int64_t ts_before) {
    int64_t ts;
    if (ts_diff > 2000000) {
        OIS_LOG_IF(buffer_dump, "i2c delay too large, spend ts_diff(us)[%lld] ", ts_diff/1000);
    }
    OIS_LOG_IF(buffer_dump, "ts_diff(us)[%lld] data_cnt[%d] delay_time_us[%d], ts_static[%lld]", ts_diff/1000, data_cnt, delay_time/1000, ts_static);

    if (ts_static == 0) {
        ts = ts_before + (ts_diff - I2C_TIME);
        ts_static = ts - delay_time - data_cnt * OIS_HALL_INTERVAL;
    } else if (ts_diff < 800000 && data_cnt > 5) {
        ts = ts_before + ts_diff / 3;
        ts_static = ts - delay_time - data_cnt * OIS_HALL_INTERVAL;
        OIS_LOG_IF(buffer_dump, "reset timestamp[%lld] i2c_delay_us[%lld] delay_time_us[%d] data_cnt[%d]", ts_static, ts_diff/1000, delay_time/1000, data_cnt);
    }

    return ts_static;
}
static int sem1217s_sample(struct hf_device *hfdev)
{
    struct i2c_client *client;
    struct v4l2_subdev *sd;
    struct sem1217s_device *sem1217s;
    struct hf_manager *manager;
    struct hf_manager_event event;
    uint8_t  read_buff[44];
    uint8_t  *temp_buff = NULL;
    int i = 0;
    int j = 0;
    int i2c_ret;
    int32_t fifo_count = 0;
    int32_t delayCount = 0;
    int32_t delayTime = 0;
    int64_t ts_before,ts_after,ts_diff,ts_mid;
    temp_buff = kzalloc(SEM1217S_EIS_DATA_SIZE, 0);

    if(!temp_buff) {
        return -1;
    }

    if (sleep_mode_state) {
        goto err;
    }

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
        sem1217s = sd_to_sem1217s_ois(sd);
    } else {
        LOG_ERR("NULL sd");
        goto err;
    }
    if (sem1217s) {
        manager = sem1217s->hf_dev.manager;
    } else {
        LOG_ERR("NULL sem1217s");
        goto err;
    }
    if (!manager) {
        LOG_ERR("NULL manager");
        goto err;
    }

    memset(temp_buff, 0, SEM1217S_EIS_DATA_SIZE);
    memset(&event, 0, sizeof(struct hf_manager_event));
    ts_before = ktime_get_boottime_ns();
    i2c_ret = adaptor_i2c_rd_p8(client, client->addr, SEM1217S_HALL_DATA_START,
                                (void *)temp_buff, SEM1217S_EIS_DATA_SIZE);
    ts_after = ktime_get_boottime_ns();
    if (i2c_ret < 0) {
        LOG_ERR("read EIS i2c error");
        goto err;
    }
    ts_diff = ts_after - ts_before;
    fifo_count = temp_buff[0] & 0xF;
    delayCount = ((uint32_t)(temp_buff[3] << 8) | temp_buff[2]);

    if (fifo_count == 0) {
        LOG_ERR("read fifo_count error");
        goto err;
    }
    if (fifo_count > 10) {
        LOG_ERR("read fifo_count num error %d", fifo_count);
        fifo_count = 10;
    }
    OIS_LOG_IF(buffer_dump, "EIS delaycount %x,%x => %d", temp_buff[2],temp_buff[3], delayCount);

    delayTime = delayCount * 100 / 3; //nano sec
    ts_mid = get_timestamp(delayTime, fifo_count, ts_diff, ts_before);
    // 0        1..3            4.5.6.7       40.41.42.43
    // fifo   reservred        HALL XY0        HALL XY9
    // big-little ending/new-old data order convert
    for (i = fifo_count - 1; i >= 0; i--, j++) {
        read_buff[i * 4 + 1] = temp_buff[4 + j * 4 + 0]; // HALL X
        read_buff[i * 4 + 0] = temp_buff[4 + j * 4 + 1]; // HALL X
        read_buff[i * 4 + 3] = temp_buff[4 + j * 4 + 2]; // HALL Y
        read_buff[i * 4 + 2] = temp_buff[4 + j * 4 + 3]; // HALL Y
    }

    // [0] [1] gyro
    // [2] [3] target
    // [4] [5] hall
    event.sensor_type = SENSOR_TYPE_OIS1;
    event.accurancy = SENSOR_ACCURANCY_HIGH;
    event.action = DATA_ACTION;

    for (i = 0; i < fifo_count; i++) {
        ts_mid += OIS_HALL_INTERVAL;
        event.timestamp = ts_mid;
        for (j = 0; j < GROUPS; j++) {
            event.word[2 * j] = g_gain_x & 0xFFFF;
            event.word[2 * j + 1] = (g_gain_x >> 16) & 0xFFFF;
            event.word[2 * j + 2] = g_gain_y & 0xFFFF;
            event.word[2 * j + 3] = (g_gain_y >> 16) & 0xFFFF;
            event.word[2 * j + 4] = sem1217s_hall_convert(((read_buff[i * 4] << 8) + read_buff[i * 4 + 1]));
            event.word[2 * j + 5] = sem1217s_hall_convert(((read_buff[i * 4 + 2] << 8) + read_buff[i * 4 + 3]));
            OIS_LOG_IF(buffer_dump, "eisbuffer X event.word[%d] index[%d] gyro[%d] target[%d] hall[%d]",2 * j,2 * i * GROUPS + 2 * j, event.word[2 * j], event.word[2 * j + 2], event.word[2 * j + 5]);
            OIS_LOG_IF(buffer_dump, "eisbuffer Y event.word[%d] index[%d] gyro[%u] target[%d] hall[%d]",2 * j + 1,2 * i * GROUPS + 2 * j + 1, event.word[2 * j + 1], event.word[2 * j + 3], event.word[2 * j + 5]);
        }
        OIS_LOG_IF(buffer_dump, "ts[%lld]", event.timestamp);
        manager->report(manager, &event);
    }
    spin_lock(&ois_spinlock);
    ts_static = ts_mid;
    spin_unlock(&ois_spinlock);
    manager->complete(manager);
    kfree(temp_buff);
    return 0;

err:
    g_last_time = 0;
    kfree(temp_buff);
    return -1;
}


static int sem1217s_custom_cmd(struct hf_device *hfdev, int sensor_type,
                              struct custom_cmd *cust_cmd)
{
    int i = 0;
    unsigned short x_hall = 0;
    unsigned short y_hall = 0;
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
        SEM1217S_Gyro_gain_set(g_gyro_gain_x, g_gyro_gain_y);
        SEM1217S_WriteGyroGainToFlash();
    }
    // ois manual control
    if (cust_cmd->command == 0x10) {
        LOG_ERR("save hall[%d,%d] len[%d]", cust_cmd->data[0], cust_cmd->data[1], cust_cmd->tx_len);
        x_hall = (unsigned short)((cust_cmd->data[0] + OUTPUT_MAX) * (INPUT_MAX - INPUT_MIN) / (OUTPUT_MAX - OUTPUT_MIN) + INPUT_MIN);
        y_hall = (unsigned short)((cust_cmd->data[1] + OUTPUT_MAX) * (INPUT_MAX - INPUT_MIN) / (OUTPUT_MAX - OUTPUT_MIN) + INPUT_MIN);
        SEM1217S_Hall_set(x_hall, y_hall);
    }
    return 0;
}

static const struct v4l2_subdev_internal_ops sem1217s_int_ops = {
    .open = sem1217s_open,
    .close = sem1217s_close,
};

static struct v4l2_subdev_core_ops sem1217s_ops_core = {
    .ioctl = sem1217s_ops_core_ioctl,
};

static const struct v4l2_subdev_ops sem1217s_ops = {
    .core = &sem1217s_ops_core,
};

static void sem1217s_subdev_cleanup(struct sem1217s_device *sem1217s)
{
    v4l2_async_unregister_subdev(&sem1217s->sd);
    v4l2_ctrl_handler_free(&sem1217s->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&sem1217s->sd.entity);
#endif
}

static int sem1217s_init_controls(struct sem1217s_device *sem1217s)
{
    struct v4l2_ctrl_handler *hdl = &sem1217s->ctrls;
    /* const struct v4l2_ctrl_ops *ops = &sem1217s_ois_ctrl_ops; */

    v4l2_ctrl_handler_init(hdl, 1);

    if (hdl->error) {
        OIS_LOG_IF(dbg, "%s init err", __func__);
        return hdl->error;
    }

    sem1217s->sd.ctrl_handler = hdl;

    return 0;
}

struct sem1217s_device *sem1217s;

static int sem1217s_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    int ret;
    int err = 0;
#ifdef test_cmd
    char buf[100] = {0};
#endif

    LOG_ERR("%s\n", __func__);

    sem1217s = devm_kzalloc(dev, sizeof(*sem1217s), GFP_KERNEL);
    if (!sem1217s)
        return -ENOMEM;

    sem1217s->vin = devm_regulator_get(dev, "iovdd");
    if (IS_ERR(sem1217s->vin)) {
        ret = PTR_ERR(sem1217s->vin);
        if (ret != -EPROBE_DEFER)
            LOG_ERR("cannot get vin regulator\n");
        return ret;
    }

    sem1217s->vdd = devm_regulator_get(dev, "avdd");
    if (IS_ERR(sem1217s->vdd)) {
        ret = PTR_ERR(sem1217s->vdd);
        if (ret != -EPROBE_DEFER)
            LOG_ERR("cannot get vdd regulator\n");
        return ret;
    }

    sem1217s->vcamaf_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR(sem1217s->vcamaf_pinctrl)) {
        ret = PTR_ERR(sem1217s->vcamaf_pinctrl);
        sem1217s->vcamaf_pinctrl = NULL;
        OIS_LOG_IF(dbg, "cannot get pinctrl\n");
    } else {
        sem1217s->vcamaf_on = pinctrl_lookup_state(
            sem1217s->vcamaf_pinctrl, "vcamaf_on");

        if (IS_ERR(sem1217s->vcamaf_on)) {
            ret = PTR_ERR(sem1217s->vcamaf_on);
            sem1217s->vcamaf_on = NULL;
            LOG_ERR("cannot get vcamaf_on pinctrl\n");
        }

        sem1217s->vcamaf_off = pinctrl_lookup_state(
            sem1217s->vcamaf_pinctrl, "vcamaf_off");

        if (IS_ERR(sem1217s->vcamaf_off)) {
            ret = PTR_ERR(sem1217s->vcamaf_off);
            sem1217s->vcamaf_off = NULL;
            LOG_ERR("cannot get vcamaf_off pinctrl\n");
        }
    }

    v4l2_i2c_subdev_init(&sem1217s->sd, client, &sem1217s_ops);
    sem1217s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    sem1217s->sd.internal_ops = &sem1217s_int_ops;

    ret = sem1217s_init_controls(sem1217s);
    if (ret)
        goto err_cleanup;

    sem1217s->hf_dev.dev_name = "sem1217s_ois";
    sem1217s->hf_dev.device_poll = HF_DEVICE_IO_POLLING;
    sem1217s->hf_dev.device_bus = HF_DEVICE_IO_SYNC;
    sem1217s->hf_dev.support_list = support_sensors;
    sem1217s->hf_dev.support_size = ARRAY_SIZE(support_sensors);
    sem1217s->hf_dev.enable = sem1217s_enable;
    sem1217s->hf_dev.batch = sem1217s_batch;
    sem1217s->hf_dev.sample = sem1217s_sample;
    sem1217s->hf_dev.custom_cmd = sem1217s_custom_cmd;
    sem1217s->hf_dev.config_cali = sem1217s_config_cali;
    hf_device_set_private_data(&sem1217s->hf_dev, client);
    err = hf_device_register_manager_create(&sem1217s->hf_dev);
    if (err < 0) {
        LOG_ERR( "%s hf_manager_create fail\n", __func__);
        err = -1;
        goto err_cleanup;
    }
    spin_lock_init(&ois_spinlock);
    ret = device_create_file(dev, &dev_attr_sem1217s_dbg);
    if (ret)
        LOG_ERR("failed to create sysfs sem1217s_dbg\n");
    ret = device_create_file(dev, &dev_attr_sem1217s_i2c_ops);
    if (ret)
        LOG_ERR("failed to create sysfs sem1217s_i2c_ops\n");
    ret = device_create_file(dev, &dev_attr_sem1217s_i2c_ops32);
    if (ret)
        LOG_ERR("failed to create sysfs sem1217s_i2c_ops32\n");

#if defined(CONFIG_MEDIA_CONTROLLER)
    ret = media_entity_pads_init(&sem1217s->sd.entity, 0, NULL);
    if (ret < 0)
        goto err_cleanup;

    sem1217s->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

    ret = v4l2_async_register_subdev(&sem1217s->sd);
    if (ret < 0)
        goto err_cleanup;

    gyro_client = hf_client_create();

    ret = sem1217s_power_on(sem1217s);
    if (ret < 0) {
        LOG_ERR("sem1217s init failed caused I2C failed.");
        goto err_cleanup;
    }
    ret = sem1217s_fw_download();
    if (ret < 0) {
        LOG_ERR("updata FW failed!");
    }
    sem1217s_power_off(sem1217s);

    pm_runtime_enable(dev);

    return 0;

err_cleanup:
    hf_device_unregister_manager_destroy(&sem1217s->hf_dev);
    sem1217s_subdev_cleanup(sem1217s);
    OIS_LOG_IF(dbg, "sem1217s err_cleanup");
    return ret;
}

static void sem1217s_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct sem1217s_device *sem1217s = sd_to_sem1217s_ois(sd);
    struct device *dev = &client->dev;
    LOG_ERR("%s\n", __func__);

    sem1217s_subdev_cleanup(sem1217s);
    pm_runtime_disable(&client->dev);
    device_remove_file(dev, &dev_attr_sem1217s_dbg);
    device_remove_file(dev, &dev_attr_sem1217s_i2c_ops);
    device_remove_file(dev, &dev_attr_sem1217s_i2c_ops32);

    if (gyro_client) {
        hf_client_destroy(gyro_client);
    }

    if (!pm_runtime_status_suspended(&client->dev))
        sem1217s_power_off(sem1217s);
    pm_runtime_set_suspended(&client->dev);

    return ;
}

static int __maybe_unused sem1217s_ois_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct sem1217s_device *sem1217s = sd_to_sem1217s_ois(sd);

    return sem1217s_power_off(sem1217s);
}

static int __maybe_unused sem1217s_ois_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct sem1217s_device *sem1217s = sd_to_sem1217s_ois(sd);

    return sem1217s_power_on(sem1217s);
}

static const struct i2c_device_id sem1217s_id_table[] = {
    {SEM1217S_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, sem1217s_id_table);

static const struct of_device_id sem1217s_of_table[] = {
    {.compatible = "mediatek,sem1217s"},
    {},
};
MODULE_DEVICE_TABLE(of, sem1217s_of_table);

static const struct dev_pm_ops sem1217s_pm_ops = {SET_SYSTEM_SLEEP_PM_OPS(
    pm_runtime_force_suspend,
    pm_runtime_force_resume) SET_RUNTIME_PM_OPS(sem1217s_ois_suspend,
                                                sem1217s_ois_resume, NULL)};

static struct i2c_driver sem1217s_i2c_driver = {
    .driver = {
        .name = SEM1217S_NAME,
        // .pm = &sem1217s_pm_ops,
        .of_match_table = sem1217s_of_table,
    },
    .probe_new = sem1217s_probe,
    .remove = sem1217s_remove,
    .id_table = sem1217s_id_table,
};

module_i2c_driver(sem1217s_i2c_driver);

MODULE_AUTHOR("Rick");
MODULE_DESCRIPTION("SEM1217S OIS driver");
MODULE_LICENSE("GPL v2");
