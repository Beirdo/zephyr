/*
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zilog_zdu0110xxx_eeprom

/**
 * @file
 * @brief EEPROM driver for Zilog ZDU0110xxx UARTs
 *
 * This driver supports the on-chip EEPROM found on Zilog ZDU0110xxx UARTs.
 */

#include <drivers/eeprom.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_EEPROM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(eeprom_zdu0110xxx);

#include <drivers/uart/zdu0110xxx.h>


struct eeprom_zdu0110xxx_drv_config {
	size_t size;
	char *parent_dev_name;
};

struct eeprom_zdu0110xxx_data {
	const struct device *parent;
};

static int eeprom_zdu0110xxx_read(const struct device *dev,
				off_t offset, void *data_buf, size_t len)
{
	const struct eeprom_zdu0110xxx_drv_config *config = dev->config;
	struct eeprom_zdu0110xxx_data *data = dev->data;
	int ret;
	uint8_t cmd[3];
	size_t cmd_len = 0;

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}

	cmd[cmd_len++] = ZDU_CMD_EEPROM_SET_CURR_LOC;
	cmd[cmd_len++] = (uint8_t)offset;
	cmd[cmd_len++] = ZDU_CMD_EEPROM_READ;
	
	ret = zdu0110xxx_send_command(data->parent, cmd, cmd_len, data_buf, len);

	if (ret != 0) {
		LOG_ERR("failed to read EEPROM (offset=%08x len=%d err=%d)",
			(unsigned int) offset, len, ret);
		return -EINVAL;
	}

	return 0;
}

static int eeprom_zdu0110xxx_write(const struct device *dev,
				 off_t offset, const void *data_buf, size_t len)
{
	const struct eeprom_zdu0110xxx_drv_config *config = dev->config;
	struct eeprom_zdu0110xxx_data *data = dev->data;

	int ret;
	uint8_t cmd[64];
	size_t cmd_len = 0;
	int index = 0;
	int copy_len;

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to write past device boundary");
		return -EINVAL;
	}
	
	cmd[cmd_len++] = ZDU_CMD_EEPROM_SET_CURR_LOC;
	cmd[cmd_len++] = (uint8_t)offset;
	
	while (index < len) {
		if (index != 0) {
			cmd_len = 0;
		}
	
		copy_len = len - index;
		copy_len = copy_len > 64 - cmd_len ? 64 - cmd_len : copy_len;
		if (copy_len <= 0) {
			break;
		}
		
		cmd[cmd_len++] = ZDU_CMD_EEPROM_WRITE;

		memcpy(&cmd[cmd_len], &((const uint8_t *)data_buf)[index], copy_len);
		cmd_len += copy_len;
		index += copy_len;

		ret = zdu0110xxx_send_command(data->parent, cmd, cmd_len, NULL, 0);
		if (ret != 0) {
			LOG_ERR("failed to write EEPROM (offset=%08x len=%d err=%d)",
				(unsigned int) offset, len, ret);
			return -EINVAL;
		}
	}

	return ret;
}

static size_t eeprom_zdu0110xxx_size(const struct device *dev)
{
	const struct eeprom_zdu0110xxx_drv_config *config = dev->config;

	return config->size;
}

static int eeprom_zdu0110xxx_init(const struct device *dev)
{
	const struct eeprom_zdu0110xxx_drv_config *config = dev->config;
	struct eeprom_zdu0110xxx_data *data = dev->data;

	data->parent = device_get_binding(config->parent_dev_name);
	if (!data->parent) {
		LOG_ERR("parent ZDU0110xxx device '%s' not found",
			config->parent_dev_name);
		return -EINVAL;
	}

	return 0;
}

static const struct eeprom_driver_api eeprom_zdu0110xxx_api = {
	.read = eeprom_zdu0110xxx_read,
	.write = eeprom_zdu0110xxx_write,
	.size = eeprom_zdu0110xxx_size,
};

#define EEPROM_ZDU0110XXX_DEVICE(id)										\
	static const struct eeprom_zdu0110xxx_drv_config						\
			eeprom_zdu0110xxx_##id##_cfg = {								\
		.size = 512,														\
		.parent_dev_name = DT_INST_BUS_LABEL(id),							\
	};																		\
																			\
	static struct eeprom_zdu0110xxx_data eeprom_zdu0110xxx_##id##_data;		\
																			\
	DEVICE_AND_API_INIT(eeprom_zdu0110xxx_##id,								\
			    DT_INST_LABEL(id),											\
			    &eeprom_zdu0110xxx_init,									\
			    &eeprom_zdu0110xxx_##id##_data,								\
			    &eeprom_zdu0110xxx_##id##_cfg,								\
			    POST_KERNEL,												\
			    CONFIG_EEPROM_ZDU0110XXX_INIT_PRIORITY,						\
			    &eeprom_zdu0110xxx_api);

DT_INST_FOREACH_STATUS_OKAY(EEPROM_ZDU0110XXX_DEVICE)
