/*
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * Heavily based on drivers/eeprom.h which is:
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for Off-SOC RAM drivers
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RAM_H_
#define ZEPHYR_INCLUDE_DRIVERS_RAM_H_

/**
 * @brief RAM Interface
 * @defgroup ram_interface RAM Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/types.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*ram_api_read)(const struct device *dev, off_t offset,
			       void *data,
			       size_t len);
typedef int (*ram_api_write)(const struct device *dev, off_t offset,
				const void *data, size_t len);
typedef size_t (*ram_api_size)(const struct device *dev);

__subsystem struct ram_driver_api {
	ram_api_read read;
	ram_api_write write;
	ram_api_size size;
};

/**
 *  @brief Read data from off-SOC RAM
 *
 *  @param dev RAM device
 *  @param offset Address offset to read from.
 *  @param data Buffer to store read data.
 *  @param len Number of bytes to read.
 *
 *  @return 0 on success, negative errno code on failure.
 */
__syscall int ram_read(const struct device *dev, off_t offset, void *data,
			  size_t len);

static inline int z_impl_ram_read(const struct device *dev, off_t offset,
				     void *data, size_t len)
{
	const struct ram_driver_api *api =
		(const struct ram_driver_api *)dev->api;

	return api->read(dev, offset, data, len);
}

/**
 *  @brief Write data to off-SOC RAM
 *
 *  @param dev RAM device
 *  @param offset Address offset to write data to.
 *  @param data Buffer with data to write.
 *  @param len Number of bytes to write.
 *
 *  @return 0 on success, negative errno code on failure.
 */
__syscall int ram_write(const struct device *dev, off_t offset,
			   const void *data,
			   size_t len);

static inline int z_impl_ram_write(const struct device *dev, off_t offset,
				      const void *data, size_t len)
{
	const struct ram_driver_api *api =
		(const struct ram_driver_api *)dev->api;

	return api->write(dev, offset, data, len);
}

/**
 *  @brief Get the size of the off-SOC RAM in bytes
 *
 *  @param dev RAM device.
 *
 *  @return RAM size in bytes.
 */
__syscall size_t ram_get_size(const struct device *dev);

static inline size_t z_impl_ram_get_size(const struct device *dev)
{
	const struct ram_driver_api *api =
		(const struct ram_driver_api *)dev->api;

	return api->size(dev);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/ram.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_RAM_H_ */
