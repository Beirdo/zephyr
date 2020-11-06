/*
 * Copyright (c) 2020 Geanix ApS
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Header file for the MCP23017 driver.
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_MCP23017_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_MCP23017_H_

#include <kernel.h>

#include <drivers/gpio.h>
#include <drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register definitions */
#define REG_IODIR_PORTA                 0x00
#define REG_IODIR_PORTB                 0x01
#define REG_IPOL_PORTA                  0x02
#define REG_IPOL_PORTB                  0x03
#define REG_GPINTEN_PORTA               0x04
#define REG_GPINTEN_PORTB               0x05
#define REG_DEFVAL_PORTA                0x06
#define REG_DEFVAL_PORTB                0x07
#define REG_INTCON_PORTA                0x08
#define REG_INTCON_PORTB                0x09
#define REG_IOCON_PORTA					0x0A
#define REG_IOCON_PORTB					0x0B
#define REG_GPPU_PORTA                  0x0C
#define REG_GPPU_PORTB                  0x0D
#define REG_INTF_PORTA                  0x0E
#define REG_INTF_PORTB                  0x0F
#define REG_INTCAP_PORTA                0x10
#define REG_INTCAP_PORTB                0x11
#define REG_GPIO_PORTA                  0x12
#define REG_GPIO_PORTB                  0x13
#define REG_OLAT_PORTA                  0x14
#define REG_OLAT_PORTB                  0x15

#define MCP23017_ADDR                   0x40

/** Configuration data */
struct gpio_mcp23017_drv_config {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_config common;

	/** The master I2C device's name */
	const char * const i2c_master_dev_name;

	/** The slave address of the chip */
	uint16_t i2c_slave_addr;

#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
	/* Interrupt pin definition */
	const char *int_gpio_port;

	gpio_pin_t int_gpio_pin;

	gpio_flags_t int_gpio_flags;
#endif
};

/** Runtime driver data */
struct gpio_mcp23017_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_config data;

	/** Master I2C device */
	const struct device *i2c_master;

	struct k_sem lock;

	struct {
		uint16_t iodir;
		uint16_t ipol;
		uint16_t gpinten;
		uint16_t defval;
		uint16_t intcon;
		uint16_t iocon;
		uint16_t gppu;
		uint16_t intf;
		uint16_t intcap;
		uint16_t gpio;
		uint16_t olat;
	} reg_cache;
	
#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
	/* Self-reference to the driver instance */
	const struct device *instance;

	/* port ISR callback routine address */
	sys_slist_t callbacks;

	/* interrupt triggering pin masks */
	struct {
		uint16_t edge_rising;
		uint16_t edge_falling;
		uint16_t edge_both;
		uint16_t level_high;
		uint16_t level_low;
	} interrupts;

	struct gpio_callback gpio_callback;

	struct k_work interrupt_worker;

	bool interrupt_active;
#endif
};

#ifdef __cplusplus
}
#endif

#endif  /* ZEPHYR_DRIVERS_GPIO_GPIO_MCP23S17_H_ */
