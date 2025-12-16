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

static struct i2c_client *aw37004_i2c_client = NULL;

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

struct aw37004_map {
    unsigned char reg;
    unsigned char value;
};

static const struct aw37004_map aw37004_on_config[] = {
    {0x03, 0x55},
    {0x04, 0x55},
    {0x05, 0x80},
    {0x06, 0x80},
    {0x0E, 0x0F},
    {0x0E, 0x00},
    {0x02, 0x8F},
    {0x02, 0x00},
};

struct aw37004_reg_map {
    char* supply_name;
    unsigned char val;
};

static const struct aw37004_reg_map aw37004_reg_config[] = {
    {"dvdd1", 0b0001},
    {"dvdd2", 0b0010},
    {"avdd1", 0b0100},
    {"avdd2", 0b1000},
};

struct aw37004_platform_data {
    struct device *dev;
    struct regmap *regmap;
};

enum aw37004_regulator_ids {
    AW37004_LDO1,
    AW37004_LDO2,
    AW37004_LDO3,
    AW37004_LDO4,
};

enum sgm38121_regulator_ids {
    sgm38121_LDO1,
    sgm38121_LDO2,
    sgm38121_LDO3,
    sgm38121_LDO4,
};

typedef enum {
    AW37004_CHIP_ID_REG = 0x00,
    AW37004_NA,
    AW37004_DISCHARGE_REG_ENABLE,
    AW37004_LDO1VOUT = 0x03,
    AW37004_LDO2VOUT,
    AW37004_LDO3VOUT,
    AW37004_LDO4VOUT,
    AW37004_ENABLE = 0x0e,
    AW37004_REG_MAX = 0x0f,
} aw37004_registers_t;

#define AW37004_MAX_REG (AW37004_REG_MAX)

typedef enum {
    sgm38121_CHIP_ID_REG = 0x00,
    sgm38121_NA,
    sgm38121_DISCHARGE_REG_ENABLE,
    sgm38121_LDO1VOUT = 0x03,
    sgm38121_LDO2VOUT,
    sgm38121_LDO3VOUT,
    sgm38121_LDO4VOUT,
    sgm38121_ENABLE = 0x0e,
    sgm38121_REG_MAX = 0x0f,
} sgm38121_registers_t;

#define sgm38121_MAX_REG (sgm38121_REG_MAX)

static int aw37004_i2c_read(struct i2c_client *i2c, unsigned char reg_addr, unsigned char *reg_val)
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

static int aw37004_i2c_write(struct i2c_client *i2c, unsigned char reg_addr, unsigned char reg_val)
{
    int ret = -1;
    ret = i2c_smbus_write_byte_data(i2c, reg_addr, reg_val);
    if (ret < 0) {
        dev_err(&i2c->dev, "i2c write failed, ret = %d", ret);
    }
    return ret;
}

static int aw37004_init_voltage(struct i2c_client *i2c) {
    int ret = 0;
    int i;

    if (!i2c) {
        pr_err("aw37004 i2c client is NULL, probe failed!\n");
        return -1;
    }
    aw37004_i2c_client = i2c;
    for (i = 0; i < ARRAY_SIZE(aw37004_on_config); i++) {
        ret = aw37004_i2c_write(aw37004_i2c_client, aw37004_on_config[i].reg, aw37004_on_config[i].value);
        if (ret < 0) {
            dev_err(&i2c->dev, "init voltage failed!, i = %d\n", i);
        }
    }
    return ret;
}

static int aw37004_enable_reg(struct regulator_dev *rdev, unsigned char flag, bool enable)
{
    int ret = 0;
    unsigned char reg_val = 0;
    dev_info(&aw37004_i2c_client->dev, "supply_name is %s\n", rdev->supply_name);

    ret = aw37004_i2c_read(aw37004_i2c_client, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
    if (ret < 0) {
        dev_err(&aw37004_i2c_client->dev, "read enable failed!\n");
        return ret;
    }

    if (enable) {
        reg_val |= flag;
    } else {
        flag = (~flag & 0xF);
        reg_val &= flag;
    }

    ret = aw37004_i2c_write(aw37004_i2c_client, aw37004_on_config[VOL_ENABLE].reg, reg_val);
    if (ret < 0) {
        dev_err(&aw37004_i2c_client->dev, "write enable failed!\n");
        return ret;
    }

    dev_info(&aw37004_i2c_client->dev, "write enable success!\n");
    dev_info(&aw37004_i2c_client->dev, "reg(0x02) = 0x%x, reg(0x03) = 0x%x, reg(0x04) = 0x%x, reg(0x05) = 0x%x, reg(0x06) = 0x%x, reg(0x0E) = 0x%x\n", \
                i2c_smbus_read_byte_data(aw37004_i2c_client, 0x02), i2c_smbus_read_byte_data(aw37004_i2c_client, 0x03), i2c_smbus_read_byte_data(aw37004_i2c_client, 0x04), \
                i2c_smbus_read_byte_data(aw37004_i2c_client, 0x05), i2c_smbus_read_byte_data(aw37004_i2c_client, 0x06), i2c_smbus_read_byte_data(aw37004_i2c_client, 0x0E));
    return ret;
}

static int aw37004_regulator_enable_regmap(struct regulator_dev *rdev)
{
    int ret = -1;
    int i;

    if (aw37004_i2c_client && rdev->supply_name) {
        for (i = 0; i < ARRAY_SIZE(aw37004_reg_config); i++) {
            if (0 == strcmp(aw37004_reg_config[i].supply_name, rdev->supply_name)) {
                ret = aw37004_enable_reg(rdev, aw37004_reg_config[i].val, true);
                break;
            }
        }
        if (ret < 0) {
            dev_err(&aw37004_i2c_client->dev, "regulator enable failed!\n");
        }
    }
    return ret;
}

static int aw37004_regulator_disable_regmap(struct regulator_dev *rdev)
{
    int ret = -1;
    int i;

    if (aw37004_i2c_client && rdev->supply_name) {
        for (i = 0; i < ARRAY_SIZE(aw37004_reg_config); i++) {
            if (0 == strcmp(aw37004_reg_config[i].supply_name, rdev->supply_name)) {
                ret = aw37004_enable_reg(rdev, aw37004_reg_config[i].val, false);
                break;
            }
        }
        if (ret < 0) {
            dev_err(&aw37004_i2c_client->dev, "regulator disable failed!\n");
        }
    }
    return ret;
}

static const struct regulator_ops aw37004_ops = {
    .enable = aw37004_regulator_enable_regmap,
    .disable = aw37004_regulator_disable_regmap,
    .is_enabled = regulator_is_enabled_regmap,
    .list_voltage = regulator_list_voltage_linear_range,
    .map_voltage = regulator_map_voltage_linear_range,
    .set_voltage_sel = regulator_set_voltage_sel_regmap,
    .get_voltage_sel = regulator_get_voltage_sel_regmap,
};
#define AW37004_DLDO(_num, _supply, _default)                                                   \
    [AW37004_LDO ## _num] = {                                                                   \
        .name             =       "ONLDO"#_num,                                                 \
        .of_match         =       of_match_ptr("ONLDO"#_num),                                   \
        .regulators_node  =       of_match_ptr("regulators"),                                   \
        .type             =       REGULATOR_VOLTAGE,                                            \
        .owner            =       THIS_MODULE,                                                  \
        .linear_ranges    =       (struct linear_range[]) {                                     \
              REGULATOR_LINEAR_RANGE(_default, 0x00, 0xF0, 6000),                               \
        },                                                                                      \
        .n_linear_ranges  =       1,                                                            \
        .vsel_reg         =       AW37004_LDO ## _num ## VOUT,                                  \
        .vsel_mask        =       0xff,                                                         \
        .enable_reg       =       AW37004_DISCHARGE_REG_ENABLE,                                 \
        .enable_mask      =       BIT(_num - 1),                                                \
        .enable_time      =       150,                                                          \
        .supply_name      =       _supply,                                                      \
        .ops              =       &aw37004_ops,                                                 \
    }

#define AW37004_ALDO(_num, _supply, _default)                                                   \
    [AW37004_LDO ## _num] = {                                                                   \
        .name             =       "ONLDO"#_num,                                                 \
        .of_match         =       of_match_ptr("ONLDO"#_num),                                   \
        .regulators_node  =       of_match_ptr("regulators"),                                   \
        .type             =       REGULATOR_VOLTAGE,                                            \
        .owner            =       THIS_MODULE,                                                  \
        .linear_ranges    =       (struct linear_range[]) {                                     \
              REGULATOR_LINEAR_RANGE(_default, 0x00, 0xB0, 12500),                              \
        },                                                                                      \
        .n_linear_ranges  =       1,                                                            \
        .vsel_reg         =       AW37004_LDO ## _num ## VOUT,                                  \
        .vsel_mask        =       0xff,                                                         \
        .enable_reg       =       AW37004_DISCHARGE_REG_ENABLE,                                 \
        .enable_mask      =       BIT(_num - 1),                                                \
        .enable_time      =       150,                                                          \
        .supply_name      =       _supply,                                                      \
        .ops              =       &aw37004_ops,                                                 \
    }

#define sgm38121_DLDO(_num, _supply, _default)                                                  \
    [sgm38121_LDO ## _num] = {                                                                  \
        .name             =       "ONLDO"#_num,                                                 \
        .of_match         =       of_match_ptr("ONLDO"#_num),                                   \
        .regulators_node  =       of_match_ptr("regulators"),                                   \
        .type             =       REGULATOR_VOLTAGE,                                            \
        .owner            =       THIS_MODULE,                                                  \
        .linear_ranges    =       (struct linear_range[]) {                                     \
              REGULATOR_LINEAR_RANGE(_default, 0x00, 0xA0, 8000),                               \
        },                                                                                      \
        .n_linear_ranges  =       1,                                                            \
        .vsel_reg         =       sgm38121_LDO ## _num ## VOUT,                                 \
        .vsel_mask        =       0xff,                                                         \
        .enable_reg       =       AW37004_DISCHARGE_REG_ENABLE,                                 \
        .enable_mask      =       BIT(_num - 1),                                                \
        .enable_time      =       150,                                                          \
        .supply_name      =       _supply,                                                      \
        .ops              =       &aw37004_ops,                                                 \
    }

#define sgm38121_ALDO(_num, _supply, _default)                                                  \
    [sgm38121_LDO ## _num] = {                                                                  \
        .name             =       "ONLDO"#_num,                                                 \
        .of_match         =       of_match_ptr("ONLDO"#_num),                                   \
        .regulators_node  =       of_match_ptr("regulators"),                                   \
        .type             =       REGULATOR_VOLTAGE,                                            \
        .owner            =       THIS_MODULE,                                                  \
        .linear_ranges    =       (struct linear_range[]) {                                     \
              REGULATOR_LINEAR_RANGE(_default, 0x00, 0xFC, 8000),                               \
        },                                                                                      \
        .n_linear_ranges  =       1,                                                            \
        .vsel_reg         =       sgm38121_LDO ## _num ## VOUT,                                 \
        .vsel_mask        =       0xff,                                                         \
        .enable_reg       =       AW37004_DISCHARGE_REG_ENABLE,                                 \
        .enable_mask      =       BIT(_num - 1),                                                \
        .enable_time      =       150,                                                          \
        .supply_name      =       _supply,                                                      \
        .ops              =       &aw37004_ops,                                                 \
    }

static struct regulator_desc aw37004_regulators[] = {
    /* supply-name Reference Specifications */
    AW37004_DLDO(1, "dvdd1", 600000),
    AW37004_DLDO(2, "dvdd2", 600000),
    AW37004_ALDO(3, "avdd1", 1200000),
    AW37004_ALDO(4, "avdd2", 1200000),
};
static struct regulator_desc sgm38121_regulators[] = {
    /* supply-name Reference Specifications */
    sgm38121_DLDO(1, "dvdd1", 504000),
    sgm38121_DLDO(2, "dvdd2", 504000),
    sgm38121_ALDO(3, "avdd1", 1384000),
    sgm38121_ALDO(4, "avdd2", 1384000),
};

typedef struct {
    int aw37004_slave_id;
    int chip_id_reg;
    int chip_id;
    struct regulator_desc *reg_desc;
    int reg_desc_size;
} aw37004_dev_info_t;

/*SlaveId is 7bit address, 0xFF is a invalid address.*/
static aw37004_dev_info_t dev_info[] = {
    /*aw37004*/
    {.aw37004_slave_id = 0x28,
     .chip_id_reg = AW37004_CHIP_ID_REG,
     .chip_id = 0x00,
     .reg_desc = &aw37004_regulators[0],
     .reg_desc_size = ARRAY_SIZE(aw37004_regulators),
    },
    /*sgm38121*/
    {.aw37004_slave_id = 0x28,
     .chip_id_reg = AW37004_CHIP_ID_REG,
     .chip_id = 0x80,
     .reg_desc = &sgm38121_regulators[0],
     .reg_desc_size = ARRAY_SIZE(sgm38121_regulators),
    },
    /*Ivalid*/
    {0xff},
};

static const struct regmap_config aw37004_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = AW37004_MAX_REG,
};

static int aw37004_i2c_probe(struct i2c_client *i2c,
                 const struct i2c_device_id *id)
{
    struct aw37004_platform_data *pdata;
    struct regulator_config config = { };
    struct regulator_dev *rdev;
    int i, ret, index=0;
    unsigned int data;
    int access_time = 3;
    struct regulator_desc *aw37004_regulators = NULL;

    if (i2c->dev.of_node) {
        pdata = devm_kzalloc(&i2c->dev,
                sizeof(struct aw37004_platform_data), GFP_KERNEL);
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

    /*process the aw37004 IC*/
    pdata->regmap = devm_regmap_init_i2c(i2c, &aw37004_regmap);
    if (IS_ERR(pdata->regmap)) {
        ret = PTR_ERR(pdata->regmap);
        dev_err(&i2c->dev, "Failed to create regmap: %d\n", ret);
        return ret;
    }

    while(dev_info[index].aw37004_slave_id != 0xFF){
        dev_info(&i2c->dev, "get product id of regulator(slaveId:0x%x)\n", dev_info[index].aw37004_slave_id);
        i2c->addr = dev_info[index].aw37004_slave_id;
        ret = regmap_read(pdata->regmap, dev_info[index].chip_id_reg, &data);
        access_time = 3;
        while(ret<0 && --access_time) {
            mdelay(2);
            ret = regmap_read(pdata->regmap, dev_info[index].chip_id_reg, &data);
        }
        if (ret < 0) {
            dev_err(&i2c->dev, "Failed to read CHIP_ID: %d\n", ret);
            index++;
            continue;
        }
        if (data == dev_info[index].chip_id) {
            break;
        }
        index++;
    }
    if(0xFF == dev_info[index].aw37004_slave_id){
        dev_err(&i2c->dev, "No valid regulator IC.\n");
        return -ENODEV;
    }
    dev_info(&i2c->dev, "find the regulator ic, slaveId:0x%x, chip_id:0x%x.\n", dev_info[index].aw37004_slave_id, data);

    config.dev = &i2c->dev;
    config.regmap = pdata->regmap;
    config.init_data = NULL;
    config.ena_gpiod = NULL;
    aw37004_regulators = dev_info[index].reg_desc;
    for (i = 0; i < dev_info[index].reg_desc_size; i++) {
        rdev = devm_regulator_register(&i2c->dev,
                           &aw37004_regulators[i],
                           &config);
        if (IS_ERR(rdev)) {
            ret = PTR_ERR(rdev);
            dev_err(&i2c->dev, "Failed to register %s: %d\n",
                aw37004_regulators[i].name, ret);
            return ret;
        }
        dev_info(&i2c->dev, "register regulator ldo %s ok\n", aw37004_regulators[i].name);
    }
    ret = aw37004_init_voltage(i2c);
    dev_info(&i2c->dev, "regulator probe end\n");
    return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id aw37004_dt_ids[] = {
    { .compatible = "aw37004-pmic", },
    {}
};
MODULE_DEVICE_TABLE(of, aw37004_dt_ids);
#endif

static const struct i2c_device_id aw37004_i2c_id[] = {
    { "aw37004-pmic", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, aw37004_i2c_id);

static struct i2c_driver aw37004_regulator_driver = {
    .driver = {
        .name = "aw37004-pmic",
        .owner = THIS_MODULE
        //.of_match_table    = of_match_ptr(aw37004_dt_ids),
    },
    .probe = aw37004_i2c_probe,
    .id_table = aw37004_i2c_id,
};

module_i2c_driver(aw37004_regulator_driver);

MODULE_DESCRIPTION("AW37004 PMIC voltage regulator driver");
MODULE_AUTHOR("XXX");
MODULE_LICENSE("GPL");
