#ifndef __BU24721_IF_H__
#define __BU24721_IF_H__

#define BU24721_FLASH_SLV_ADDR 0x38 //0x50-->0x   //0x38
#define BU24721_SLV_ADDR 0x3E

extern struct bu24721_device *bu24721;
extern int adaptor_i2c_rd_u8(struct i2c_client *i2c_client, u16 addr, u16 reg,
			     u8 *val);

extern int adaptor_i2c_rd_u16(struct i2c_client *i2c_client, u16 addr, u16 reg,
			      u16 *val);

extern int adaptor_i2c_rd_p8(struct i2c_client *i2c_client, u16 addr, u16 reg,
			     u8 *p_vals, u32 n_vals);

extern int adaptor_i2c_wr_u8(struct i2c_client *i2c_client, u16 addr, u16 reg,
			     u8 val);

extern int adaptor_i2c_wr_u16(struct i2c_client *i2c_client, u16 addr, u16 reg,
			      u16 val);

extern int adaptor_i2c_wr_p8(struct i2c_client *i2c_client, u16 addr, u16 reg,
			     u8 *p_vals, u32 n_vals);

extern int adaptor_i2c_wr_p16(struct i2c_client *i2c_client, u16 addr, u16 reg,
			      u16 *p_vals, u32 n_vals);

extern int adaptor_i2c_wr_seq_p8(struct i2c_client *i2c_client, u16 addr,
				 u16 reg, u8 *p_vals, u32 n_vals);

extern int adaptor_i2c_wr_regs_u8(struct i2c_client *i2c_client, u16 addr,
				  u16 *list, u32 len);

extern int adaptor_i2c_wr_regs_u16(struct i2c_client *i2c_client, u16 addr,
				   u16 *list, u32 len);

uint8_t I2C_OIS_8bit__read(uint32_t addr)
{
	uint8_t data;
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	ret = adaptor_i2c_rd_u8(i2c_client, BU24721_SLV_ADDR, addr, &data);
	if (ret < 0) {
		pr_err("%s failed addr[%x]", __func__, addr);
	}
	return data;
}

uint16_t I2C_OIS_16bit__read(uint32_t addr)
{
	uint16_t data;
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	ret = adaptor_i2c_rd_u16(i2c_client, BU24721_SLV_ADDR, addr, &data);
	if (ret < 0) {
		pr_err("%s failed addr[%x]", __func__, addr);
	}
	return data;
}

uint32_t I2C_OIS_32bit__read(uint32_t addr)
{
	char data[4] = { 0 };
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	ret = adaptor_i2c_rd_p8(i2c_client, BU24721_SLV_ADDR, addr, data, 4);
	if (ret < 0) {
		pr_err("%s failed addr[%x]", __func__, addr);
	}
	return (uint32_t)data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
}

int I2C_OIS_8bit_write(uint32_t addr, uint8_t data)
{
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	// pr_info("i2c_client %p", i2c_client);
	ret = adaptor_i2c_wr_u8(i2c_client, BU24721_SLV_ADDR, addr, data);
	if (ret < 0) {
		pr_err("%s failed addr[%x] data[%x]", __func__, addr, data);
	}
	return ret;
}

int I2C_OIS_16bit_write(uint32_t addr, uint16_t data)
{
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	ret = adaptor_i2c_wr_u16(i2c_client, BU24721_SLV_ADDR, addr, data);
	if (ret < 0) {
		pr_err("%s failed addr[%x] data[%x]", __func__, addr, data);
	}
	return ret;
}

void I2C_OIS_block_write(void *register_data, int size)
{
	int ret;
	uint16_t reg_addr;
	uint8_t *data = register_data;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	reg_addr = (uint16_t)data[0] << 8 | data[1];
	ret = adaptor_i2c_wr_p8(i2c_client, BU24721_SLV_ADDR, reg_addr,
				&data[2], size - 2);
	if (ret < 0) {
		pr_err("%s failed addr[%x]", __func__, reg_addr);
	}
}

//FW----------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------FW---------------------------------------------------------------

uint8_t I2C_FW_8bit__read(uint32_t addr)
{
	uint8_t data;
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	ret = adaptor_i2c_rd_u8(i2c_client, BU24721_FLASH_SLV_ADDR, addr,
				&data);
	if (ret < 0) {
		pr_err("%s failed addr[%x]", __func__, addr);
	}
	return data;
}

uint16_t I2C_FM_16bit__read(uint32_t addr)
{
	uint16_t data;
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	ret = adaptor_i2c_rd_u16(i2c_client, BU24721_FLASH_SLV_ADDR, addr, &data);
	if (ret < 0) {
		pr_err("%s failed addr[%x]", __func__, addr);
	}
	return data;
}

uint32_t I2C_FM_32bit__read(uint32_t addr)
{
	char data[4] = { 0 };
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	ret = adaptor_i2c_rd_p8(i2c_client, BU24721_FLASH_SLV_ADDR, addr, data,
				4);
	if (ret < 0) {
		pr_err("%s failed addr[%x]", __func__, addr);
	}
	return (uint32_t)data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
}

int I2C_FM_8bit_write(uint32_t addr, uint8_t data)
{
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	// pr_info("FM i2c_client %p", i2c_client);
	ret = adaptor_i2c_wr_u8(i2c_client, BU24721_FLASH_SLV_ADDR, addr, data);
	if (ret < 0) {
		pr_err("%s failed addr[%x] data[%x]", __func__, addr, data);
	}
	return ret;
}

int I2C_FM_16bit_write(uint32_t addr, uint16_t data)
{
	int ret;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	ret = adaptor_i2c_wr_u16(i2c_client, BU24721_FLASH_SLV_ADDR, addr,
				 data);
	if (ret < 0) {
		pr_err("%s failed addr[%x] data[%x]", __func__, addr, data);
	}
	return ret;
}

void I2C_FM_block_write(void *register_data, int size)
{
	int ret;
	uint16_t reg_addr;
	uint8_t *data = register_data;
	struct i2c_client *i2c_client;
	i2c_client = bu24721->hf_dev.private_data;
	reg_addr = (uint16_t)data[0] << 8 | data[1];
	ret = adaptor_i2c_wr_p8(i2c_client, BU24721_FLASH_SLV_ADDR, reg_addr,
				&data[2], size - 2);
	if (ret < 0) {
		pr_err("%s failed addr[%x]", __func__, reg_addr);
	}
}

void Wait(int us)
{
	msleep((us + 999) / 1000);
}

#endif