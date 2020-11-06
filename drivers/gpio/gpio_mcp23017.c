/*
 * Copyright (c) 2020 Geanix ApS
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mcp23017

/**
 * @file Driver for MCP23017 I2C-based GPIO driver.
 */

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <sys/byteorder.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

#include "gpio_utils.h"
#include "gpio_mcp23017.h"

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(gpio_mcp23017);

/**
 * @brief Read both port 0 and port 1 registers of certain register function.
 *
 * Given the register in reg, read the pair of port 0 and port 1.
 *
 * @param dev Device struct of the MCP23017.
 * @param reg Register to read (the PORT0 of the pair of registers).
 * @param cache Pointer to the cache to be updated after successful read.
 * @param buf Buffer to read data into.
 *
 * @return 0 if successful, failed otherwise.
 */
static int read_port_regs(const struct device *dev, uint8_t reg,
			  uint16_t *cache, uint16_t *buf)
{
	const struct gpio_mcp23017_drv_config * const config = dev->config;
	struct gpio_mcp23017_drv_data * const data =
		(struct gpio_mcp23017_drv_data * const)dev->data;
	const struct device *i2c_master = data->i2c_master;
	uint16_t i2c_addr = config->i2c_slave_addr;
	uint16_t port_data, value;
	int ret;

	ret = i2c_burst_read(i2c_master, i2c_addr, reg,
			     (uint8_t *)&port_data, sizeof(port_data));
	if (ret != 0) {
		LOG_ERR("MCP23017[0x%X]: error reading register 0x%X (%d)",
			i2c_addr, reg, ret);
		return ret;
	}

	value = sys_le16_to_cpu(port_data);
	*cache = value;
	*buf = value;

	LOG_DBG("MCP23017[0x%X]: Read: REG[0x%X] = 0x%X, REG[0x%X] = 0x%X",
		i2c_addr, reg, (*buf & 0xFF), (reg + 1), (*buf >> 8));

	return 0;
}


/**
 * @brief Write both port 0 and port 1 registers of certain register function.
 *
 * Given the register in reg, write the pair of port 0 and port 1.
 *
 * @param dev Device struct of the MCP23017.
 * @param reg Register to write into (the PORT0 of the pair of registers).
 * @param cache Pointer to the cache to be updated after successful write.
 * @param value New value to set.
 *
 * @return 0 if successful, failed otherwise.
 */
static int write_port_regs(const struct device *dev, uint8_t reg,
			   uint16_t *cache, uint16_t value)
{
	const struct gpio_mcp23017_drv_config * const config = dev->config;
	struct gpio_mcp23017_drv_data * const data =
		(struct gpio_mcp23017_drv_data * const)dev->data;
	const struct device *i2c_master = data->i2c_master;
	uint16_t i2c_addr = config->i2c_slave_addr;
	uint16_t port_data;
	int ret;

	LOG_DBG("MCP23017[0x%X]: Write: REG[0x%X] = 0x%X, REG[0x%X] = "
		"0x%X", i2c_addr, reg, (value & 0xFF),
		(reg + 1), (value >> 8));

	port_data = sys_cpu_to_le16(value);

	ret = i2c_burst_write(i2c_master, i2c_addr, reg,
			      (uint8_t *)&port_data, sizeof(port_data));
	if (ret == 0) {
		*cache = value;
	} else {
		LOG_ERR("MCP23017[0x%X]: error writing to register 0x%X "
			"(%d)", i2c_addr, reg, ret);
	}

	return ret;
}

/**
 * @brief Setup the pin direction (input or output)
 *
 * @param dev Device struct of the MCP23017
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int setup_pin_dir(const struct device *dev, uint32_t pin, int flags)
{
	struct gpio_mcp23017_drv_data *const data =
		(struct gpio_mcp23017_drv_data *const)dev->data;
	uint16_t *dir = &data->reg_cache.iodir;
	uint16_t *output = &data->reg_cache.gpio;
	int ret;

	if ((flags & GPIO_OUTPUT) != 0U) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			*output |= BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			*output &= ~BIT(pin);
		}
		*dir &= ~BIT(pin);
	} else {
		*dir |= BIT(pin);
	}

	ret = write_port_regs(dev, REG_GPIO_PORTA, &data->reg_cache.gpio, *output);
	if (ret != 0) {
		return ret;
	}

	return write_port_regs(dev, REG_IODIR_PORTA, &data->reg_cache.iodir, *dir);
}

/**
 * @brief Setup the pin pull up/pull down status
 *
 * @param dev Device struct of the MCP23017
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int setup_pin_pullupdown(const struct device *dev, uint32_t pin,
				int flags)
{
	struct gpio_mcp23017_drv_data *const data =
		(struct gpio_mcp23017_drv_data *const)dev->data;
	uint16_t port;

	/* Setup pin pull up or pull down */
	port = data->reg_cache.gppu;

	/* pull down == 0, pull up == 1 */
	if ((flags & GPIO_PULL_DOWN) != 0U) {
		return -ENOTSUP;
	}

	WRITE_BIT(port, pin, (flags & GPIO_PULL_UP) != 0U);

	return write_port_regs(dev, REG_GPPU_PORTA, &data->reg_cache.gppu, port);
}


/**
 * @brief Configure pin or port
 *
 * @param dev Device struct of the MCP23017
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_mcp23017_config(const struct device *dev,
			       gpio_pin_t pin, gpio_flags_t flags)
{
	int ret;
	struct gpio_mcp23017_drv_data * const data =
		(struct gpio_mcp23017_drv_data * const)dev->data;

#if (CONFIG_GPIO_LOG_LEVEL >= LOG_LEVEL_DEBUG)
	const struct gpio_mcp23017_drv_config * const config = dev->config;
	uint16_t i2c_addr = config->i2c_slave_addr;
#endif

	/* Does not support disconnected pin */
	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == GPIO_DISCONNECTED) {
		return -ENOTSUP;
	}

	/* Open-drain support is per port, not per pin.
	 * So can't really support the API as-is.
	 */
	if ((flags & GPIO_SINGLE_ENDED) != 0U) {
		return -ENOTSUP;
	}

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	ret = setup_pin_dir(dev, pin, flags);
	if (ret) {
		LOG_ERR("MCP23017[0x%X]: error setting pin direction (%d)",
			i2c_addr, ret);
		goto done;
	}

	ret = setup_pin_pullupdown(dev, pin, flags);
	if (ret) {
		LOG_ERR("MCP23017[0x%X]: error setting pin pull up/down "
			"(%d)", i2c_addr, ret);
		goto done;
	}

done:
	k_sem_give(&data->lock);
	return ret;
}

static int gpio_mcp23017_port_get_raw(const struct device *dev,
				     uint32_t *value)
{
	struct gpio_mcp23017_drv_data * const data =
		(struct gpio_mcp23017_drv_data * const)dev->data;
	uint16_t buf;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	ret = read_port_regs(dev, REG_GPIO_PORTA,
			      &data->reg_cache.gpio, &buf);
	if (ret == 0) {
		*value = buf;
	}

	k_sem_give(&data->lock);
	return ret;
}

static int gpio_mcp23017_port_set_masked_raw(const struct device *dev,
					      uint32_t mask, uint32_t value)
{
	struct gpio_mcp23017_drv_data * const data =
		(struct gpio_mcp23017_drv_data * const)dev->data;
	uint16_t reg_out;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	reg_out = data->reg_cache.gpio;
	reg_out = (reg_out & ~mask) | (mask & value);

	ret = write_port_regs(dev, REG_GPIO_PORTA,
			       &data->reg_cache.gpio, reg_out);

	k_sem_give(&data->lock);

	return ret;
}

static int gpio_mcp23017_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	return gpio_mcp23017_port_set_masked_raw(dev, mask, mask);
}

static int gpio_mcp23017_port_clear_bits_raw(const struct device *dev,
					uint32_t mask)
{
	return gpio_mcp23017_port_set_masked_raw(dev, mask, 0);
}

static int gpio_mcp23017_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	struct gpio_mcp23017_drv_data *const data =
		(struct gpio_mcp23017_drv_data *const)dev->data;
	uint16_t reg_out;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	reg_out = data->reg_cache.gpio;
	reg_out ^= mask;

	ret = write_port_regs(dev, REG_GPIO_PORTA,
			       &data->reg_cache.gpio, reg_out);

	k_sem_give(&data->lock);

	return ret;
}


#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
static void gpio_mcp23017_interrupt_worker(struct k_work *work)
{
	struct gpio_mcp23017_drv_data * const data = CONTAINER_OF(
		work, struct gpio_mcp23017_drv_data, interrupt_worker);
	uint16_t input_new, input_cache, changed_pins, trig_edge;
	uint16_t trig_level = 0;
	uint16_t triggered_int = 0;
	uint16_t enabled_int = 0;
	int ret;

	k_sem_take(&data->lock, K_FOREVER);

    const struct device *dev = data->instance;
	input_cache = data->reg_cache.gpio;

	ret = read_port_regs(dev, REG_GPIO_PORTA,
			      &data->reg_cache.gpio, &input_new);

	if (ret == 0) {
		ret = read_port_regs(dev, REG_GPINTEN_PORTA,
					&data->reg_cache.gpinten, &enabled_int);
	}

	if (ret == 0) {
		/* Note: MCP23017 Interrupt status is cleared by reading inputs */

		changed_pins = (input_cache ^ input_new);

		trig_edge = (changed_pins & input_new &
			     data->interrupts.edge_rising);
		trig_edge |= (changed_pins & input_cache &
			      data->interrupts.edge_falling);
		trig_edge |= (changed_pins & data->interrupts.edge_both);
		trig_edge &= enabled_int;

		trig_level = (input_new & data->interrupts.level_high);
		trig_level |= (~input_new & data->interrupts.level_low);
		trig_level &= enabled_int;

		triggered_int = trig_edge | trig_level;
	}

	k_sem_give(&data->lock);

	if (triggered_int != 0) {
		gpio_fire_callbacks(&data->callbacks, dev, triggered_int);
	}

	/* Emulate level triggering */
	if (trig_level != 0) {
		/* Reschedule worker */
		k_work_submit(&data->interrupt_worker);
	}
}

static void gpio_mcp23017_interrupt_callback(const struct device *dev,
					    struct gpio_callback *cb,
					    gpio_port_pins_t pins)
{
	struct gpio_mcp23017_drv_data * const data =
		CONTAINER_OF(cb, struct gpio_mcp23017_drv_data, gpio_callback);

	ARG_UNUSED(pins);

	/* Cannot read MCP23017 registers from ISR context, queue worker */
	k_work_submit(&data->interrupt_worker);
}
#endif /* CONFIG_GPIO_MCP23017_INTERRUPT */

static int gpio_mcp23017_pin_interrupt_configure(const struct device *dev,
						  gpio_pin_t pin,
						  enum gpio_int_mode mode,
						  enum gpio_int_trig trig)
{
	int ret = 0;

	if (!IS_ENABLED(CONFIG_GPIO_MCP23017_INTERRUPT)
	    && (mode != GPIO_INT_MODE_DISABLED)) {
		return -ENOTSUP;
	}

#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
	const struct gpio_mcp23017_drv_config * const config = dev->config;
	struct gpio_mcp23017_drv_data * const data =
		(struct gpio_mcp23017_drv_data * const)dev->data;
	const struct device *int_gpio_dev;
	uint16_t reg;
	bool enabled, edge, level, active;

	/* Check for an invalid pin number */
	if (BIT(pin) > config->common.port_pin_mask) {
		return -EINVAL;
	}

	/* Check configured pin direction */
	if ((mode != GPIO_INT_MODE_DISABLED) &&
	    (BIT(pin) & data->reg_cache.iodir) != BIT(pin)) {
		LOG_ERR("MCP23017[0x%X]: output pin cannot trigger interrupt",
			config->i2c_slave_addr);
		return -ENOTSUP;
	}

	k_sem_take(&data->lock, K_FOREVER);

	/* Update interrupt masks */
	enabled = ((mode & GPIO_INT_MODE_DISABLED) == 0U);
	edge = (mode == GPIO_INT_MODE_EDGE);
	level = (mode == GPIO_INT_MODE_LEVEL);
	
	WRITE_BIT(data->interrupts.edge_rising, pin, (enabled &&
		edge && ((trig & GPIO_INT_TRIG_HIGH) == GPIO_INT_TRIG_HIGH)));
	WRITE_BIT(data->interrupts.edge_falling, pin, (enabled &&
		edge && ((trig & GPIO_INT_TRIG_LOW) == GPIO_INT_TRIG_LOW)));
	WRITE_BIT(data->interrupts.edge_both, pin, (enabled &&
		edge && ((trig & GPIO_INT_TRIG_BOTH) == GPIO_INT_TRIG_BOTH)));
		
	WRITE_BIT(data->interrupts.level_high, pin, (enabled &&
		level && ((trig & GPIO_INT_TRIG_HIGH) == GPIO_INT_TRIG_HIGH)));
	WRITE_BIT(data->interrupts.level_low, pin, (enabled &&
		level && ((trig & GPIO_INT_TRIG_LOW) == GPIO_INT_TRIG_LOW)));

	active = ((data->interrupts.edge_rising |
		   data->interrupts.edge_falling |
		   data->interrupts.edge_both |
		   data->interrupts.level_high |
		   data->interrupts.level_low) != 0);

	/* Enable / disable interrupt as needed */
	if (active != data->interrupt_active) {
		if (active) {
			/* Read current status to reset any
			 * active signal on INT line
			 */
			ret = read_port_regs(dev, REG_GPIO_PORTA,
				      &data->reg_cache.gpio, &reg);
		}

		if (ret != 0) {
			LOG_ERR("MCP23017[0x%X]: failed to clear potential interrupts "
			    "(%d), disabling", config->i2c_slave_addr, ret);
			active = 0;
		}

		if (active) {
			/* Disable the interrupt on the receiving pin just in case */
			int_gpio_dev = device_get_binding(config->int_gpio_port);
			ret = gpio_pin_interrupt_configure(int_gpio_dev,
				config->int_gpio_pin, GPIO_INT_MODE_DISABLED);
			if (ret != 0) {
				LOG_ERR("MCP23017[0x%X]: failed to disable interrupt "
					"on pin %d (%d)", config->i2c_slave_addr,
					config->int_gpio_pin, ret);
				active = 0;
			}
		}
		
		ret = write_port_regs(dev, REG_GPINTEN_PORTA,
			       &data->reg_cache.gpinten, active);

		if (ret != 0) {
			LOG_ERR("MCP23017[0x%X]: failed to configure interrupts "
			    "to %04X (%d), disabling", config->i2c_slave_addr,
				active, ret);
			active = 0;
		}


		if (active) {
			uint16_t iocon_val;
			
			/* Set IOCON to mirror the interrupts (i.e. both interrupt pins are
			 * for both ports) 
			 */
			iocon_val = data->reg_cache.iocon;
			iocon_val |= 0x4040;
			
			ret = write_port_regs(dev, REG_IOCON_PORTA, 
					&data->reg_cache.iocon, iocon_val);
					
			if (ret != 0) {
				LOG_ERR("MCP23017[0x%X]: failed to configure interrupt "
				    "mirroring (%d), disabling", config->i2c_slave_addr, ret);
				active = 0;
			}
		}
	
		if (active) {
			int_gpio_dev = device_get_binding(config->int_gpio_port);
			ret = gpio_pin_interrupt_configure(int_gpio_dev,
				config->int_gpio_pin, GPIO_INT_EDGE_TO_ACTIVE);
				
			if (ret != 0) {
				LOG_ERR("MCP23017[0x%X]: failed to configure interrupt "
					"on pin %d (%d)", config->i2c_slave_addr,
					config->int_gpio_pin, ret);
				active = 0;
			}
		}

		data->interrupt_active = active;
	}

	k_sem_give(&data->lock);
#endif /* CONFIG_GPIO_MCP23017_INTERRUPT */
	return ret;
}

#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
static int gpio_mcp23017_manage_callback(const struct device *dev,
					struct gpio_callback *callback,
					bool set)
{
	struct gpio_mcp23017_drv_data * const data =
		(struct gpio_mcp23017_drv_data * const)dev->data;

	k_sem_take(&data->lock, K_FOREVER);

	gpio_manage_callback(&data->callbacks, callback, set);

	k_sem_give(&data->lock);
	return 0;
}
#endif


static const struct gpio_driver_api gpio_mcp23017_drv_api_funcs = {
	.pin_configure = gpio_mcp23017_config,
	.port_get_raw = gpio_mcp23017_port_get_raw,
	.port_set_masked_raw = gpio_mcp23017_port_set_masked_raw,
	.port_set_bits_raw = gpio_mcp23017_port_set_bits_raw,
	.port_clear_bits_raw = gpio_mcp23017_port_clear_bits_raw,
	.port_toggle_bits = gpio_mcp23017_port_toggle_bits,
	.pin_interrupt_configure = gpio_mcp23017_pin_interrupt_configure,
#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
	.manage_callback = gpio_mcp23017_manage_callback,
#endif
};

/**
 * @brief Initialization function of MCP23017
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int gpio_mcp23017_init(const struct device *dev)
{
	const struct gpio_mcp23017_drv_config * const config = dev->config;
	struct gpio_mcp23017_drv_data * const data =
		(struct gpio_mcp23017_drv_data * const)dev->data;
	const struct device *i2c_master;
#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
	const struct device *int_gpio_dev;
	int ret;
#endif

	/* Find out the device struct of the I2C master */
	i2c_master = device_get_binding((char *)config->i2c_master_dev_name);
	if (!i2c_master) {
		return -EINVAL;
	}
	data->i2c_master = i2c_master;

	k_sem_init(&data->lock, 1, 1);

#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
	/* Store self-reference for interrupt handling */
	data->instance = dev;

	/* Prepare interrupt worker */
	k_work_init(&data->interrupt_worker, gpio_mcp23017_interrupt_worker);

	/* Configure GPIO interrupt pin */
	int_gpio_dev = device_get_binding(config->int_gpio_port);
	if (int_gpio_dev == NULL) {
		LOG_ERR("MCP23017[0x%X]: error getting interrupt GPIO"
			" device (%s)", config->i2c_slave_addr,
			config->int_gpio_port);
		return -ENODEV;
	}

	ret = gpio_pin_configure(int_gpio_dev, config->int_gpio_pin,
				(config->int_gpio_flags | GPIO_INPUT));
	if (ret != 0) {
		LOG_ERR("MCP23017[0x%X]: failed to configure interrupt"
			" pin %d (%d)", config->i2c_slave_addr,
			config->int_gpio_pin, ret);
		return ret;
	}

	/* Prepare GPIO callback for interrupt pin */
	gpio_init_callback(&data->gpio_callback,
			   gpio_mcp23017_interrupt_callback,
			   BIT(config->int_gpio_pin));
	gpio_add_callback(int_gpio_dev, &data->gpio_callback);
#endif

	return 0;
}


#define GPIO_MCP23017_DEVICE_INSTANCE(inst)									\
static const struct gpio_mcp23017_drv_config gpio_mcp23017_##inst##_cfg = {	\
	.common = {																\
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),				\
	},																		\
	.i2c_master_dev_name = DT_INST_BUS_LABEL(inst),							\
	.i2c_slave_addr = DT_INST_REG_ADDR(inst),								\
	IF_ENABLED(CONFIG_GPIO_MCP23017_INTERRUPT, (							\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, interrupt_gpios), (				\
	.int_gpio_port = DT_INST_GPIO_LABEL(inst, interrupt_gpios),				\
	.int_gpio_pin = DT_INST_GPIO_PIN(inst, interrupt_gpios),				\
	.int_gpio_flags = DT_INST_GPIO_FLAGS(inst, interrupt_gpios),			\
	))))																	\
};																			\
																			\
static struct gpio_mcp23017_drv_data gpio_mcp23017_##inst##_drvdata = {		\
		/* Default for registers according to datasheet */					\
		.reg_cache.iodir = 0xFFFF,											\
		.reg_cache.ipol = 0x0,												\
		.reg_cache.gpinten = 0x0,											\
		.reg_cache.defval = 0x0,											\
		.reg_cache.intcon = 0x0,											\
		.reg_cache.iocon = 0x0,												\
		.reg_cache.gppu = 0x0,												\
		.reg_cache.intf = 0x0,												\
		.reg_cache.intcap = 0x0,											\
		.reg_cache.gpio = 0x0,												\
		.reg_cache.olat = 0x0,												\
	IF_ENABLED(CONFIG_GPIO_MCP23017_INTERRUPT, (							\
  		.interrupt_active = false,											\
	))																		\
};																			\
																			\
DEVICE_AND_API_INIT(gpio_mcp23017_##inst,									\
	DT_INST_LABEL(inst),													\
	gpio_mcp23017_init,														\
	&gpio_mcp23017_##inst##_drvdata,										\
	&gpio_mcp23017_##inst##_cfg,											\
	POST_KERNEL,															\
	CONFIG_GPIO_MCP23017_INIT_PRIORITY,										\
	&gpio_mcp23017_drv_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(GPIO_MCP23017_DEVICE_INSTANCE)
