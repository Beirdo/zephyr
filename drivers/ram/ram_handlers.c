/*
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <syscall_handler.h>
#include <drivers/ram.h>

static inline int z_vrfy_ram_read(const struct device *dev, off_t offset,
				     void *data, size_t len)
{
	Z_OOPS(Z_SYSCALL_DRIVER_RAM(dev, read));
	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(data, len));
	return z_impl_ram_read((const struct device *)dev, offset,
				  (void *)data,
				  len);
}
#include <syscalls/ram_read_mrsh.c>

static inline int z_vrfy_ram_write(const struct device *dev, off_t offset,
				      const void *data, size_t len)
{
	Z_OOPS(Z_SYSCALL_DRIVER_RAM(dev, write));
	Z_OOPS(Z_SYSCALL_MEMORY_READ(data, len));
	return z_impl_ram_write((const struct device *)dev, offset,
				   (const void *)data, len);
}
#include <syscalls/ram_write_mrsh.c>

static inline size_t z_vrfy_ram_get_size(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_RAM(dev, size));
	return z_impl_ram_get_size((const struct device *)dev);
}
#include <syscalls/ram_get_size_mrsh.c>
