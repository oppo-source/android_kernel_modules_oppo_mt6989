// SPDX-License-Identifier: GPL-2.0+
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <string.h>

static struct i2c_client *wl2866d_i2c_client = NULL;
#define SGM38121_CHIPID 0x80

enum {
    OUT_DVDD1,
    OUT_DVDD2,
    OUT_AVDD1,
    OUT_AVDD2,
    VOL_ENABLE,
    VOL_DISABLE,
    DISCHARGE_ENABLE,
    DISCHARGE_DISABLE,
};

struct wl2866d_map {
    unsigned char reg;
    unsigned char value;
};

static const struct wl2866d_map wl2866d_on_config[] = {
    {0x03, 0x55},
    {0x04, 0x55},
    {0x05, 0x80},
    {0x06, 0x80},
    {0x0E, 0x0F},
    {0x0E, 0x00},
    {0x02, 0x8F},
    {0x02, 0x00},
};

struct wl2866d_reg_map {
    char* supply_name;
    unsigned char val;
};

static const struct wl2866d_reg_map wl2866d_reg_config[] = {
    {"dvdd1", 0b0001},  //main  dvdd
    {"avdd1", 0b1000},  //main  avdd
    {"dvdd2", 0b0010},  //front dvdd
    {"avdd2", 0b0100},  //front avdd
};

struct wl2866d_platform_data {
    struct device *dev;
    struct regmap *regmap;
};

enum wl2866d_regulator_ids {
    WL2866D_LDO1,
    WL2866D_LDO2,
    WL2866D_LDO3,
    WL2866D_LDO4,
};

typedef enum {
    WL2866D_CHIP_ID_REG = 0x00,
    WL2866D_NA,
    WL2866D_DISCHARGE_REG_ENABLE,
    WL2866D_LDO1VOUT = 0x03,
    WL2866D_LDO2VOUT,
    WL2866D_LDO3VOUT,
    WL2866D_LDO4VOUT,
    WL2866D_ENABLE = 0x0e,
    WL2866D_REG_MAX = 0x0f,
} wl2866d_registers_t;

#define WL2866D_MAX_REG (WL2866D_REG_MAX)

static int wl2866d_i2c_read(struct i2c_client *i2c, unsigned char reg_addr, unsigned char *reg_val)
{
    int ret = -1;
    ret = i2c_smbus_read_byte_data(i2c, reg_addr);
    if (ret < 0) {
        dev_err(&i2c->dev, "i2c read failed, ret = %d\n", ret);
    } else {
        *reg_val = ret;
    }
    return ret;
}

static int wl2866d_i2c_write(struct i2c_client *i2c, unsigned char reg_addr, unsigned char reg_val)
{
    int ret = -1;
    ret = i2c_smbus_write_byte_data(i2c, reg_addr, reg_val);
    if (ret < 0) {
        dev_err(&i2c->dev, "i2c write failed, ret = %d", ret);
    }
    return ret;
}

static int wl2866d_init_voltage(struct i2c_client *i2c) {
    int ret = 0;
    int i;

    if (!i2c) {
        pr_err("wl2866d i2c client is NULL, probe failed!\n");
        return -1;
    }
    wl2866d_i2c_client = i2c;
    for (i = 0; i < ARRAY_SIZE(wl2866d_on_config); i++) {
        ret = wl2866d_i2c_write(wl2866d_i2c_client, wl2866d_on_config[i].reg, wl2866d_on_config[i].value);
        if (ret < 0) {
            dev_err(&i2c->dev, "init voltage failed!\n");
        }
    }
    return ret;
}

static int wl2866d_enable_reg(struct regulator_dev *rdev, unsigned char flag, bool enable)
{
    int ret = 0;
    unsigned char reg_val = 0;
    dev_info(&wl2866d_i2c_client->dev, "supply_name is %s\n", rdev->supply_name);

    ret = wl2866d_i2c_read(wl2866d_i2c_client, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
    if (ret < 0) {
        dev_err(&wl2866d_i2c_client->dev, "read enable failed!\n");
        return ret;
    }

    if (enable) {
        reg_val |= flag;
    } else {
        flag = (~flag & 0xF);
        reg_val &= flag;
    }

    ret = wl2866d_i2c_write(wl2866d_i2c_client, wl2866d_on_config[VOL_ENABLE].reg, reg_val);
    if (ret < 0) {
        dev_err(&wl2866d_i2c_client->dev, "write enable failed!\n");
        return ret;
    }

    dev_info(&wl2866d_i2c_client->dev, "write enable success!\n");
    return ret;
}

static int wl2866d_regulator_enable_regmap(struct regulator_dev *rdev)
{
    int ret = -1;
    int i;

    if (wl2866d_i2c_client && rdev->supply_name) {
        for (i = 0; i < ARRAY_SIZE(wl2866d_reg_config); i++) {
            if (0 == strcmp(wl2866d_reg_config[i].supply_name, rdev->supply_name)) {
                ret = wl2866d_enable_reg(rdev, wl2866d_reg_config[i].val, true);
                break;
            }
        }
        if (ret < 0) {
            dev_err(&wl2866d_i2c_client->dev, "regulator enable failed!\n");
        }
    }
    return ret;
}

static int wl2866d_regulator_disable_regmap(struct regulator_dev *rdev)
{
    int ret = -1;
    int i;

    if (wl2866d_i2c_client && rdev->supply_name) {
        for (i = 0; i < ARRAY_SIZE(wl2866d_reg_config); i++) {
            if (0 == strcmp(wl2866d_reg_config[i].supply_name, rdev->supply_name)) {
                ret = wl2866d_enable_reg(rdev, wl2866d_reg_config[i].val, false);
                break;
            }
        }
        if (ret < 0) {
            dev_err(&wl2866d_i2c_client->dev, "regulator disable failed!\n");
        }
    }
    return ret;
}

static const struct regulator_ops wl2866d_ops = {
    .enable = wl2866d_regulator_enable_regmap,
    .disable = wl2866d_regulator_disable_regmap,
    .is_enabled = regulator_is_enabled_regmap,
    .list_voltage = regulator_list_voltage_linear_range,
    .map_voltage = regulator_map_voltage_linear_range,
    .set_voltage_sel = regulator_set_voltage_sel_regmap,
    .get_voltage_sel = regulator_get_voltage_sel_regmap,
};
#define WL2866D_DLDO(_num, _supply, _default)                                                   \
    [WL2866D_LDO ## _num] = {                                                                   \
        .name             =       "ONLDO"#_num,                                                 \
        .of_match         =       of_match_ptr("ONLDO"#_num),                                   \
        .regulators_node  =       of_match_ptr("regulators"),                                   \
        .type             =       REGULATOR_VOLTAGE,                                            \
        .owner            =       THIS_MODULE,                                                  \
        .linear_ranges    =       (struct linear_range[]) {                                     \
              REGULATOR_LINEAR_RANGE(_default, 0, 0xff, 6000),                                  \
        },                                                                                      \
        .n_linear_ranges  =       1,                                                            \
        .vsel_reg         =       WL2866D_LDO ## _num ## VOUT,                                  \
        .vsel_mask        =       0xff,                                                         \
        .enable_reg       =       WL2866D_DISCHARGE_REG_ENABLE,                                 \
        .enable_mask      =       BIT(_num - 1),                                                \
        .enable_time      =       150,                                                          \
        .supply_name      =       _supply,                                                      \
        .ops              =       &wl2866d_ops,                                                 \
    }

#define WL2866D_ALDO(_num, _supply, _default)                                                   \
    [WL2866D_LDO ## _num] = {                                                                   \
        .name             =       "ONLDO"#_num,                                                 \
        .of_match         =       of_match_ptr("ONLDO"#_num),                                   \
        .regulators_node  =       of_match_ptr("regulators"),                                   \
        .type             =       REGULATOR_VOLTAGE,                                            \
        .owner            =       THIS_MODULE,                                                  \
        .linear_ranges    =       (struct linear_range[]) {                                     \
              REGULATOR_LINEAR_RANGE(_default, 0, 0xff, 12500),                                 \
        },                                                                                      \
        .n_linear_ranges  =       1,                                                            \
        .vsel_reg         =       WL2866D_LDO ## _num ## VOUT,                                  \
        .vsel_mask        =       0xff,                                                         \
        .enable_reg       =       WL2866D_DISCHARGE_REG_ENABLE,                                 \
        .enable_mask      =       BIT(_num - 1),                                                \
        .enable_time      =       150,                                                          \
        .supply_name      =       _supply,                                                      \
        .ops              =       &wl2866d_ops,                                                 \
    }

static struct regulator_desc wl2866d_regulators[] = {
    WL2866D_DLDO(1, "dvdd1", 600000),    // main  dvdd
    WL2866D_DLDO(2, "dvdd2", 600000),    // front dvdd
    WL2866D_ALDO(3, "avdd1", 1200000),   // front avdd
    WL2866D_ALDO(4, "avdd2", 1200000),   // main  avdd
};
typedef struct {
    int wl2866d_slave_id;
    int chip_id_reg;
    int chip_id;
    struct regulator_desc *reg_desc;
    int reg_desc_size;
} wl2866d_dev_info_t;

/*SlaveId is 7bit address, 0xFF is a invalid address.*/
static wl2866d_dev_info_t dev_info[] = {
    /*wl2866d*/
    {.wl2866d_slave_id = 0x28,
     .chip_id_reg = WL2866D_CHIP_ID_REG,
     .chip_id = 0x00,
     .reg_desc = &wl2866d_regulators[0],
     .reg_desc_size = ARRAY_SIZE(wl2866d_regulators),
    },
    /*Ivalid*/
    {0xff},
};

static const struct regmap_config wl2866d_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = WL2866D_MAX_REG,
};

static int wl2866d_i2c_probe(struct i2c_client *i2c,
                 const struct i2c_device_id *id)
{
    struct wl2866d_platform_data *pdata;
    struct regulator_config config = { };
    struct regulator_dev *rdev;
    int i, ret, index=0;
    unsigned int data;
    int access_time = 3;
    struct regulator_desc *wl2866d_regulators = NULL;

    if (i2c->dev.of_node) {
        pdata = devm_kzalloc(&i2c->dev,
                sizeof(struct wl2866d_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&i2c->dev, "Failed to allocate memory\n");
            return -ENOMEM;
        }
    } else {
        pdata = i2c->dev.platform_data;
    }
    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
        dev_err(&i2c->dev, "fail : i2c functionality check...\n");
        return -EOPNOTSUPP;
    }

    if (pdata == NULL) {
        dev_err(&i2c->dev, "fail : no platform data.\n");
        return -ENODATA;
    }

    /*process the wl2866d IC*/
    pdata->regmap = devm_regmap_init_i2c(i2c, &wl2866d_regmap);
    if (IS_ERR(pdata->regmap)) {
        ret = PTR_ERR(pdata->regmap);
        dev_err(&i2c->dev, "Failed to create regmap: %d\n", ret);
        return ret;
    }

    while(dev_info[index].wl2866d_slave_id != 0xFF){
        dev_info(&i2c->dev, "get product id of regulator(slaveId:0x%x)\n", dev_info[index].wl2866d_slave_id);
        i2c->addr = dev_info[index].wl2866d_slave_id;
        ret = regmap_read(pdata->regmap, dev_info[index].chip_id_reg, &data);
        while(ret<0 && --access_time) {
            mdelay(2);
            ret = regmap_read(pdata->regmap, dev_info[index].chip_id_reg, &data);
        }
        if (ret < 0) {
            dev_err(&i2c->dev, "Failed to read CHIP_ID: %d\n", ret);
            index += 1;
            continue;
        }
        if (data == dev_info[index].chip_id || data == SGM38121_CHIPID) {
            break;
        }
        index += 1;
    }
    if(0xFF == dev_info[index].wl2866d_slave_id){
        dev_err(&i2c->dev, "No valid regulator IC.\n");
        return -ENODEV;
    }
    dev_info(&i2c->dev, "find the regulator ic, slaveId:0x%x.\n", dev_info[index].wl2866d_slave_id);

    config.dev = &i2c->dev;
    config.regmap = pdata->regmap;
    config.init_data = NULL;
    config.ena_gpiod = NULL;
    wl2866d_regulators = dev_info[index].reg_desc;
    for (i = 0; i < dev_info[index].reg_desc_size; i++) {
        rdev = devm_regulator_register(&i2c->dev,
                           &wl2866d_regulators[i],
                           &config);
        if (IS_ERR(rdev)) {
            ret = PTR_ERR(rdev);
            dev_err(&i2c->dev, "Failed to register %s: %d\n",
                wl2866d_regulators[i].name, ret);
            return ret;
        }
        dev_info(&i2c->dev, "register regulator ldo %s ok\n", wl2866d_regulators[i].name);
    }
    ret = wl2866d_init_voltage(i2c);
    dev_info(&i2c->dev, "regulator probe end\n");
    return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id wl2866d_dt_ids[] = {
    { .compatible = "wl2866d-pmic", },
    {}
};
MODULE_DEVICE_TABLE(of, wl2866d_dt_ids);
#endif

static const struct i2c_device_id wl2866d_i2c_id[] = {
    { "wl2866d-pmic", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, wl2866d_i2c_id);

static struct i2c_driver wl2866d_regulator_driver = {
    .driver = {
        .name = "wl2866d-pmic",
        .owner = THIS_MODULE
        //.of_match_table    = of_match_ptr(wl2866d_dt_ids),
    },
    .probe = wl2866d_i2c_probe,
    .id_table = wl2866d_i2c_id,
};

module_i2c_driver(wl2866d_regulator_driver);

MODULE_DESCRIPTION("WL2866D PMIC voltage regulator driver");
MODULE_AUTHOR("XXX");
MODULE_LICENSE("GPL");
