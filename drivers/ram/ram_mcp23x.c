/*
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for Microchip 23Axxx/23Kxxx SPI SRAMs.
 */

#include <drivers/ram.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_RAM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(ram_mcp23x);

/* 23x instruction set */
#define RAM_MCP23X_READ			0x03	/* Read data                    */
#define RAM_MCP23X_WRITE		0x02	/* Write data                   */
#define RAM_MCP23X_RDSR 		0x05	/* Read STATUS register         */
#define RAM_MCP23X_WRSR			0x01	/* Write STATUS register        */

/* 23x status register bits */
#define RAM_MCP23X_MODE_MASK	0xC0	/* Operating mode of SRAM       */
#define RAM_MCP23X_HOLD			0x01	/* Enable HOLD pin feature		*/
#define RAM_MCP23X_STATUS_MASK	0xC1

/* 23x operating modes */
#define RAM_MCP23X_MODE_BYTE	0x00	/* Byte mode (default)			*/
#define RAM_MCP23X_MODE_PAGE	0x80	/* Page mode					*/
#define RAM_MCP23X_MODE_SEQ		0x40	/* Sequential mode				*/
#define RAM_MCP23X_MODE_RES		0xC0	/* Reserved - do not use		*/

struct ram_mcp23x_config {
	const char *bus_dev_name;
	uint16_t bus_addr;
	uint32_t max_freq;
	const char *spi_cs_dev_name;
	gpio_pin_t spi_cs_pin;
	gpio_dt_flags_t spi_cs_dt_flags;
	gpio_pin_t wp_gpio_pin;
	gpio_dt_flags_t wp_gpio_flags;
	const char *wp_gpio_name;
	size_t size;
	uint8_t addr_width;
	bool readonly;
	uint16_t timeout;
	ram_api_read read_fn;
	ram_api_write write_fn;
};

struct ram_mcp23x_data {
	const struct device *bus_dev;
	struct spi_config spi_cfg;
	struct spi_cs_control spi_cs;
	const struct device *wp_gpio_dev;
	struct k_mutex lock;
};

static inline int ram_mcp23x_write_protect(const struct device *dev)
{
	const struct ram_mcp23x_config *config = dev->config;
	struct ram_mcp23x_data *data = dev->data;

	if (!data->wp_gpio_dev) {
		return 0;
	}

	return gpio_pin_set(data->wp_gpio_dev, config->wp_gpio_pin, 1);
}

static inline int ram_mcp23x_write_enable(const struct device *dev)
{
	const struct ram_mcp23x_config *config = dev->config;
	struct ram_mcp23x_data *data = dev->data;

	if (!data->wp_gpio_dev) {
		return 0;
	}

	return gpio_pin_set(data->wp_gpio_dev, config->wp_gpio_pin, 0);
}

static int ram_mcp23x_read(const struct device *dev, off_t offset, void *buf,
			    size_t len)
{
	const struct ram_mcp23x_config *config = dev->config;
	struct ram_mcp23x_data *data = dev->data;
	uint8_t *pbuf = buf;
	int ret;

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	while (len) {
		ret = _ram_mcp23x_spi_read(dev, offset, pbuf, len);
		if (ret < 0) {
			LOG_ERR("failed to read RAM (err %d)", ret);
			k_mutex_unlock(&data->lock);
			return ret;
		}

		pbuf += ret;
		offset += ret;
		len -= ret;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}


static int ram_mcp23x_write(const struct device *dev, off_t offset,
			     const void *buf,
			     size_t len)
{
	const struct ram_mcp23x_config *config = dev->config;
	struct ram_mcp23x_data *data = dev->data;
	const uint8_t *pbuf = buf;
	int ret;

	if (config->readonly) {
		LOG_WRN("attempt to write to read-only device");
		return -EACCES;
	}

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to write past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = ram_mcp23x_write_enable(dev);
	if (ret) {
		LOG_ERR("failed to write-enable RAM (err %d)", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	while (len) {
		ret = _ram_mcp23x_spi_write(dev, offset, pbuf, len);
		if (ret < 0) {
			LOG_ERR("failed to write to RAM (err %d)", ret);
			ram_mcp23x_write_protect(dev);
			k_mutex_unlock(&data->lock);
			return ret;
		}

		pbuf += ret;
		offset += ret;
		len -= ret;
	}

	ret = ram_mcp23x_write_protect(dev);
	if (ret) {
		LOG_ERR("failed to write-protect RAM (err %d)", ret);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static size_t ram_mcp23x_size(const struct device *dev)
{
	const struct ram_mcp23x_config *config = dev->config;

	return config->size;
}

static int ram_mcp23x_rdsr(const struct device *dev, uint8_t *status)
{
	struct ram_mcp23x_data *data = dev->data;
	uint8_t rdsr[2] = { RAM_MCP23X_RDSR, 0 };
	uint8_t sr[2];
	int err;
	const struct spi_buf tx_buf = {
		.buf = rdsr,
		.len = sizeof(rdsr),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf = {
		.buf = sr,
		.len = sizeof(sr),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	err = spi_transceive(data->bus_dev, &data->spi_cfg, &tx, &rx);
	if (!err) {
		*status = sr[1];
	}

	return err;
}

static int ram_mcp23x_wrsr(const struct device *dev, uint8_t status)
{
	struct ram_mcp23x_data *data = dev->data;
	uint8_t wrsr[2] = { RAM_MCP23X_WRSR, status & RAM_MCP23X_STATUS_MASK };
	int err;
	const struct spi_buf tx_buf = {
		.buf = wrsr,
		.len = sizeof(wrsr),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	err = spi_transceive(data->bus_dev, &data->spi_cfg, &tx, NULL);
	return err;
}


static int _ram_mcp23x_spi_read(const struct device *dev, off_t offset, void *buf,
			    			size_t len)
{
	const struct ram_mcp23x_config *config = dev->config;
	struct ram_mcp23x_data *data = dev->data;
	size_t cmd_len = 1 + config->addr_width / 8;
	uint8_t cmd[4] = { RAM_MCP23X_READ, 0, 0, 0 };
	uint8_t *paddr;
	int err;
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = cmd_len,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_bufs[2] = {
		{
			.buf = NULL,
			.len = cmd_len,
		},
		{
			.buf = buf,
			.len = len,
		},
	};
	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs),
	};

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}

	paddr = &cmd[1];
	switch (config->addr_width) {
	case 16:
		*paddr++ = offset >> 8;
		__fallthrough;
	case 8:
		*paddr++ = offset;
		break;
	default:
		__ASSERT(0, "invalid address width");
	}

	err = spi_transceive(data->bus_dev, &data->spi_cfg, &tx, &rx);
	if (err < 0) {
		return err;
	}

	return len;
}

static int _ram_mcp23x_spi_write(const struct device *dev, off_t offset,
			     const void *buf, size_t len)
{
	const struct ram_mcp23x_config *config = dev->config;
	struct ram_mcp23x_data *data = dev->data;
	uint8_t cmd[4] = { RAM_MCP23X_WRITE, 0, 0, 0 };
	size_t cmd_len = 1 + config->addr_width / 8;
	uint8_t *paddr;
	int err;
	const struct spi_buf tx_bufs[2] = {
		{
			.buf = cmd,
			.len = cmd_len,
		},
		{
			.buf = (void *)buf,
			.len = len,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};

	paddr = &cmd[1];
	switch (config->addr_width) {
	case 16:
		*paddr++ = offset >> 8;
		__fallthrough;
	case 8:
		*paddr++ = offset;
		break;
	default:
		__ASSERT(0, "invalid address width");
	}

	err = spi_transceive(data->bus_dev, &data->spi_cfg, &tx, NULL);
	if (err) {
		return err;
	}

	return count;
}

static int ram_mcp23x_init(const struct device *dev)
{
	const struct ram_mcp23x_config *config = dev->config;
	struct ram_mcp23x_data *data = dev->data;
	int err;

	k_mutex_init(&data->lock);

	data->bus_dev = device_get_binding(config->bus_dev_name);
	if (!data->bus_dev) {
		LOG_ERR("could not get parent bus device");
		return -EINVAL;
	}

	data->spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
		SPI_WORD_SET(8);
	data->spi_cfg.frequency = config->max_freq;
	data->spi_cfg.slave = config->bus_addr;

	if (config->spi_cs_dev_name) {
		data->spi_cs.gpio_dev =
			device_get_binding(config->spi_cs_dev_name);
		if (!data->spi_cs.gpio_dev) {
			LOG_ERR("could not get SPI CS GPIO device");
			return -EINVAL;
		}

		data->spi_cs.gpio_pin = config->spi_cs_pin;
		data->spi_cs.gpio_dt_flags = config->spi_cs_dt_flags;
		data->spi_cfg.cs = &data->spi_cs;
	}

	if (config->wp_gpio_name) {
		data->wp_gpio_dev = device_get_binding(config->wp_gpio_name);
		if (!data->wp_gpio_dev) {
			LOG_ERR("could not get WP GPIO device");
			return -EINVAL;
		}

		err = gpio_pin_configure(data->wp_gpio_dev, config->wp_gpio_pin,
					 GPIO_OUTPUT_ACTIVE | config->wp_gpio_flags);
		if (err) {
			LOG_ERR("failed to configure WP GPIO pin (err %d)",
				err);
			return err;
		}
	}

	/* This driver is written primarily to run in sequential mode, disable HOLD pin */
	return ram_mcp23x_wrsr(dev, RAM_MCP23X_MODE_SEQ);
}

static const struct ram_driver_api ram_mcp23x_api = {
	.read = ram_mcp23x_read,
	.write = ram_mcp23x_write,
	.size = ram_mcp23x_size,
};

#define ASSERT_ADDR_W_VALID(w)			\
	BUILD_ASSERT(w == 16U, "Unsupported address width")

#define INST_DT_MCP23X(inst, t) DT_INST(inst, microchip_23x##t)

#define RAM_MCP23X_DEVICE(n, t) \
	ASSERT_ADDR_W_VALID(DT_PROP(INST_DT_MCP23X(n, t), address_width)); \
	static const struct ram_mcp23x_config ram_mcp23x##t##_config_##n = { \
		.bus_dev_name = DT_BUS_LABEL(INST_DT_MCP23X(n, t)), \
		.bus_addr = DT_REG_ADDR(INST_DT_MCP23X(n, t)), \
		.max_freq = UTIL_AND( \
			DT_NODE_HAS_PROP(INST_DT_MCP23X(n, t), \
					 spi_max_frequency), \
			DT_PROP(INST_DT_MCP23X(n, t), spi_max_frequency)), \
		.spi_cs_dev_name = UTIL_AND( \
			DT_SPI_DEV_HAS_CS_GPIOS(INST_DT_MCP23X(n, t)), \
			DT_SPI_DEV_CS_GPIOS_LABEL(INST_DT_MCP23X(n, t))),	\
		.spi_cs_pin = UTIL_AND( \
			DT_SPI_DEV_HAS_CS_GPIOS(INST_DT_MCP23X(n, t)), \
			DT_SPI_DEV_CS_GPIOS_PIN(INST_DT_MCP23X(n, t))), \
		.spi_cs_dt_flags = UTIL_AND( \
			DT_SPI_DEV_HAS_CS_GPIOS(INST_DT_MCP23X(n, t)), \
			DT_SPI_DEV_CS_GPIOS_FLAGS(INST_DT_MCP23X(n, t))), \
		.wp_gpio_pin = UTIL_AND( \
			DT_NODE_HAS_PROP(INST_DT_MCP23X(n, t), wp_gpios), \
			DT_GPIO_PIN(INST_DT_MCP23X(n, t), wp_gpios)), \
		.wp_gpio_flags = UTIL_AND( \
			DT_NODE_HAS_PROP(INST_DT_MCP23X(n, t), wp_gpios), \
			DT_GPIO_FLAGS(INST_DT_MCP23X(n, t), wp_gpios)), \
		.wp_gpio_name = UTIL_AND( \
			DT_NODE_HAS_PROP(INST_DT_MCP23X(n, t), wp_gpios), \
			DT_GPIO_LABEL(INST_DT_MCP23X(n, t), wp_gpios)), \
		.size = DT_PROP(INST_DT_MCP23X(n, t), size), \
		.addr_width = DT_PROP(INST_DT_MCP23X(n, t), address_width), \
		.readonly = DT_PROP(INST_DT_MCP23X(n, t), read_only), \
		.timeout = DT_PROP(INST_DT_MCP23X(n, t), timeout), \
	}; \
	static struct ram_mcp23x_data ram_mcp23x##t##_data_##n; \
	DEVICE_AND_API_INIT(ram_mcp23x##t##_##n, \
			    DT_LABEL(INST_DT_MCP23X(n, t)), \
			    &ram_mcp23x_init, &ram_mcp23x##t##_data_##n, \
			    &ram_mcp23x##t##_config_##n, POST_KERNEL, \
			    CONFIG_RAM_MCP23x_INIT_PRIORITY, \
			    &ram_mcp23x_api)

#define RAM_MCP23x256_DEVICE(n) RAM_MCP23X_DEVICE(n, 256)
#define RAM_MCP23x640_DEVICE(n) RAM_MCP23X_DEVICE(n, 640)

#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_MCP23X_FOREACH(t, inst_expr) \
	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(microchip_23x##t),	\
		     CALL_WITH_ARG, inst_expr)

INST_DT_MCP23X_FOREACH(256, RAM_MCP23x256_DEVICE);
INST_DT_MCP23X_FOREACH(640, RAM_MCP23x640_DEVICE);

