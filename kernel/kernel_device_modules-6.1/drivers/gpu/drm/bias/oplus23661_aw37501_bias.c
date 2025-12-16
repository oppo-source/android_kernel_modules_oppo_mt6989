#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include "oplus23661_aw37501_bias.h"

static struct i2c_client *_lcm_i2c_client;

int lcm_i2c_write_bytes(u8 reg , u8 val)
{
    int ret = 0;

    if (_lcm_i2c_client == NULL) {
        pr_err("%s _lcm_i2c_client is null", __func__);
        return 0;
    }

    ret = i2c_smbus_write_byte_data(_lcm_i2c_client, reg, val);
    if (ret < 0)
        pr_err("%s _lcm_i2c_client write data fail", __func__);

    return ret;
}

EXPORT_SYMBOL(lcm_i2c_write_bytes);

int lcm_i2c_read_bytes(u8 reg , u8 *data)
{
    int ret;

    if (_lcm_i2c_client == NULL) {
        pr_err("%s _lcm_i2c_client is null", __func__);
        return 0;
    }

    ret = i2c_smbus_read_byte_data(_lcm_i2c_client, reg);
	if(ret < 0)
	{
        pr_err("%s _lcm_i2c_client read data fail", __func__);
		return ret;
	}

	*data = (u8)ret;
	return 0;
}
EXPORT_SYMBOL(lcm_i2c_read_bytes);

static int _lcm_i2c_probe(struct i2c_client *client,
              const struct i2c_device_id *id)
{
    u8 bias_id;
    _lcm_i2c_client = client;
    lcm_i2c_read_bytes(0x04, &bias_id);
    pr_err("%s: bias name=%s, bias addr=0x%x, bias id=0x%x\n",
            __func__, client->name, client->addr, bias_id);
    return 0;
}

static void _lcm_i2c_remove(struct i2c_client *client)
{
    _lcm_i2c_client = NULL;
    i2c_unregister_device(client);
}

static const struct of_device_id _lcm_i2c_of_match[] = {
    {
        .compatible = "awinic,aw37501",
    },
    {
    }
};

static const struct i2c_device_id _lcm_i2c_id[] = { { LCM_I2C_ID_NAME, 0 },
                            {} };

static struct i2c_driver _lcm_i2c_driver = {
    .id_table = _lcm_i2c_id,
    .probe = _lcm_i2c_probe,
    .remove = _lcm_i2c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = LCM_I2C_ID_NAME,
        .of_match_table = _lcm_i2c_of_match,
    },
};

static int __init _lcm_i2c_init(void)
{
    i2c_add_driver(&_lcm_i2c_driver);
    return 0;
}

static void __exit _lcm_i2c_exit(void)
{
    i2c_del_driver(&_lcm_i2c_driver);
}

module_init(_lcm_i2c_init);
module_exit(_lcm_i2c_exit);

MODULE_AUTHOR("qiubaorong");
MODULE_DESCRIPTION("LCD Bias Driver");
MODULE_LICENSE("GPL v2");
