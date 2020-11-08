/*
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_pcf8574

/**
 * @file Driver for PCF8574 I2C-based GPIO driver.
 */

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

#include "gpio_utils.h"

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_pcf8574);


/** Configuration data */
struct gpio_pcf8574_drv_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;

	/** The master I2C device's name */
	const char * const i2c_master_dev_name;

	/** The slave address of the chip */
	uint16_t i2c_slave_addr;

#ifdef CONFIG_GPIO_PCF8574_INTERRUPT
	/* Interrupt pin definition */
	const char *int_gpio_port;

	gpio_pin_t int_gpio_pin;

	gpio_flags_t int_gpio_flags;
#endif
};

/** Runtime driver data */
struct gpio_pcf8574_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	/** Master I2C device */
	const struct device *i2c_master;

	struct {
		uint8_t input;
		uint8_t output;
		uint8_t dir;
	} reg_cache;

	struct k_sem lock;

#ifdef CONFIG_GPIO_PCF8574_INTERRUPT
	/* Self-reference to the driver instance */
	const struct device *instance;

	/* port ISR callback routine address */
	sys_slist_t callbacks;

	/* interrupt triggering pin masks */
	struct {
		uint8_t edge_rising;
		uint8_t edge_falling;
		uint8_t edge_both;
		uint8_t level_high;
		uint8_t level_low;
		uint8_t enabled;
	} interrupts;

	struct gpio_callback gpio_callback;

	struct k_work interrupt_worker;

	bool interrupt_active;
#endif
};


static int update_gpio(const struct device *dev, uint8_t *read_value)
{
	struct gpio_pcf8574_drv_data * data = dev->data;
	const struct gpio_pcf8574_drv_config * const config = dev->config;
	uint8_t reg_dir = data->reg_cache.dir;
	uint8_t reg_out = data->reg_cache.output;
	uint8_t output_value;
	uint8_t input_value;
	int ret;

	/* 
	 * To set a GPIO to an input, one needs to write out a high signal.  All
	 * GPIOs are pseudo-bidirectional, with an open-drain setup, and a built-in
	 * weak pullup
	 */
	 
	/* dir 0 -> output, dir 1 -> input */
	output_value = (reg_out & ~reg_dir) | reg_dir;
	
	if (output_value != reg_out) {
		ret = i2c_write(data->i2c_master, &output_value, 1, config->i2c_slave_addr);
		if (ret != 0) {
			return ret;
		}
		
		data->reg_cache.output = output_value;
	}
	
	if (read_value) {
		ret = i2c_read(data->i2c_master, &input_value, 1, config->i2c_slave_addr);
		if (ret != 0) {
			return ret;
		}
		
		input_value &= reg_dir;
		data->reg_cache.input = input_value;
		*read_value = input_value;
	}
	
	return ret;
}


/**
 * @brief Setup the pin direction (input or output)
 *
 * @param dev Device struct of the PCF8574
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int setup_pin_dir(const struct device *dev, uint32_t pin, int flags)
{
	struct gpio_pcf8574_drv_data * const data =
		(struct gpio_pcf8574_drv_data * const)dev->data;
	uint8_t reg_dir = data->reg_cache.dir;
	uint8_t reg_out = data->reg_cache.output;
	int ret;

	/* For each pin, 0 == output, 1 == input */
	if ((flags & GPIO_OUTPUT) != 0U) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			reg_out |= BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			reg_out &= ~BIT(pin);
		}
		reg_dir &= ~BIT(pin);
	} else {
		reg_dir |= BIT(pin);
	}

	data->reg_cache.dir = reg_dir;
	data->reg_cache.output = reg_out;

	ret = update_gpio(dev, NULL);

	return ret;
}

/**
 * @brief Configure pin or port
 *
 * @param dev Device struct of the PCA95XX
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_pcf8574_config(const struct device *dev,
			       gpio_pin_t pin, gpio_flags_t flags)
{
	int ret;
	struct gpio_pcf8574_drv_data * data = dev->data;

#if (CONFIG_GPIO_LOG_LEVEL >= LOG_LEVEL_DEBUG)
	const struct gpio_pcf8574_drv_config * const config = dev->config;
	uint16_t i2c_addr = config->i2c_slave_addr;
#endif

	/* Does not support disconnected pin */
	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == GPIO_DISCONNECTED) {
		return -ENOTSUP;
	}

	/* Open-drain support is always on on all pins.*/
	if ((flags & GPIO_SINGLE_ENDED) == 0U) {
		return -ENOTSUP;
	}

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	ret = setup_pin_dir(dev, pin, flags);
	if (ret) {
		LOG_ERR("PCF8574[0x%X]: error setting pin direction (%d)",
			i2c_addr, ret);
		goto done;
	}

done:
	k_sem_give(&data->lock);
	return ret;
}

static int gpio_pcf8574_port_get_raw(const struct device *dev,
				     uint32_t *value)
{
	struct gpio_pcf8574_drv_data * data = dev->data;
	uint8_t buf;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	ret = update_gpio(dev, &buf);
	if (ret != 0) {
		goto done;
	}

	*value = buf;

done:
	k_sem_give(&data->lock);
	return ret;
}

static int gpio_pcf8574_port_set_masked_raw(const struct device *dev,
					      uint32_t mask, uint32_t value)
{
	struct gpio_pcf8574_drv_data * const data =
		(struct gpio_pcf8574_drv_data * const)dev->data;
	uint8_t reg_out;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	reg_out = data->reg_cache.output;
	reg_out = (reg_out & ~mask) | (mask & value);

	data->reg_cache.output = reg_out;
	ret = update_gpio(dev, NULL);

	k_sem_give(&data->lock);

	return ret;
}

static int gpio_pcf8574_port_set_bits_raw(const struct device *dev,
					  uint32_t mask)
{
	return gpio_pcf8574_port_set_masked_raw(dev, mask, mask);
}

static int gpio_pcf8574_port_clear_bits_raw(const struct device *dev,
					    uint32_t mask)
{
	return gpio_pcf8574_port_set_masked_raw(dev, mask, 0);
}

static int gpio_pcf8574_port_toggle_bits(const struct device *dev,
					 uint32_t mask)
{
	struct gpio_pcf8574_drv_data * const data =
		(struct gpio_pcf8574_drv_data * const)dev->data;
	uint16_t reg_out;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	reg_out = data->reg_cache.output;
	reg_out ^= mask;
	data->reg_cache.output = reg_out;

	ret = update_gpio(dev, NULL);

	k_sem_give(&data->lock);

	return ret;
}

#ifdef CONFIG_GPIO_PCF8574_INTERRUPT
static void gpio_pcf8574_interrupt_worker(struct k_work *work)
{
	struct gpio_pcf8574_drv_data * const data = CONTAINER_OF(
		work, struct gpio_pcf8574_drv_data, interrupt_worker);
	uint8_t input_new, input_cache, changed_pins, trig_edge;
	uint8_t trig_level = 0;
	uint8_t triggered_int = 0;
	int ret;

	k_sem_take(&data->lock, K_FOREVER);

	input_cache = data->reg_cache.input;

	ret = update_gpio(data->instance, &input_new);
	if (ret == 0) {
		/* Note: PCF Interrupt status is cleared by reading inputs */

		changed_pins = (input_cache ^ input_new);

		trig_edge = (changed_pins & input_new &
			     data->interrupts.edge_rising);
		trig_edge |= (changed_pins & input_cache &
			      data->interrupts.edge_falling);
		trig_edge |= (changed_pins & data->interrupts.edge_both);
		trig_edge &= data->interrupts.enabled;
		
		trig_level = (input_new & data->interrupts.level_high);
		trig_level |= (~input_new & data->interrupts.level_low);
		trig_level &= data->interrupts.enabled;

		triggered_int = (trig_edge | trig_level);
	}

	k_sem_give(&data->lock);

	if (triggered_int != 0) {
		gpio_fire_callbacks(&data->callbacks, data->instance, triggered_int);
	}

	/* Emulate level triggering */
	if (trig_level != 0) {
		/* Reschedule worker */
		k_work_submit(&data->interrupt_worker);
	}
}

static void gpio_pcf8574_interrupt_callback(const struct device *dev,
					    struct gpio_callback *cb,
					    gpio_port_pins_t pins)
{
	struct gpio_pcf8574_drv_data * const data =
		CONTAINER_OF(cb, struct gpio_pcf8574_drv_data, gpio_callback);

	ARG_UNUSED(pins);

	/* Cannot read PCF8574 registers from ISR context, queue worker */
	k_work_submit(&data->interrupt_worker);
}
#endif /* CONFIG_GPIO_PCF8574_INTERRUPT */

static int gpio_pcf8574_pin_interrupt_configure(const struct device *dev,
						  gpio_pin_t pin,
						  enum gpio_int_mode mode,
						  enum gpio_int_trig trig)
{
	int ret = 0;

	if (!IS_ENABLED(CONFIG_GPIO_PCF8574_INTERRUPT)
	    && (mode != GPIO_INT_MODE_DISABLED)) {
		return -ENOTSUP;
	}

#ifdef CONFIG_GPIO_PCF8574_INTERRUPT
	const struct gpio_pcf8574_drv_config * const config = dev->config;
	struct gpio_pcf8574_drv_data * data = dev->data;
	const struct device *int_gpio_dev;
	uint8_t reg;
	bool enabled, edge, level, active;

	/* Check for an invalid pin number */
	if (BIT(pin) > config->common.port_pin_mask) {
		return -EINVAL;
	}

	/* Check configured pin direction */
	if ((mode != GPIO_INT_MODE_DISABLED) &&
	    (BIT(pin) & data->reg_cache.dir) != BIT(pin)) {
		LOG_ERR("PCF8574[0x%X]: output pin cannot trigger interrupt",
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

	WRITE_BIT(data->interrupts.enabled, pin, enabled);

	active = ((data->interrupts.edge_rising ||
		   data->interrupts.edge_falling ||
		   data->interrupts.edge_both ||
		   data->interrupts.level_high ||
		   data->interrupts.level_low) > 0);

	/* Enable / disable interrupt as needed */
	if (active != data->interrupt_active) {
		int_gpio_dev = device_get_binding(config->int_gpio_port);
		ret = gpio_pin_interrupt_configure(int_gpio_dev,
			config->int_gpio_pin, (active ?
				GPIO_INT_EDGE_TO_ACTIVE :
				GPIO_INT_MODE_DISABLED));
	
		if (ret != 0) {
			LOG_ERR("PCF8574[0x%X]: failed to configure interrupt "
				"on pin %d (%d)", config->i2c_slave_addr,
				config->int_gpio_pin, ret);
			goto done;
		}
		data->interrupt_active = active;

		if (active) {
			/* Read current status to reset any
			 * active signal on INT line
			 */
			update_gpio(dev, &reg);
		}
	}

done:
	k_sem_give(&data->lock);
#endif /* CONFIG_GPIO_PCF8574_INTERRUPT */
	return ret;
}

#ifdef CONFIG_GPIO_PCF8574_INTERRUPT
static int gpio_pcf8574_manage_callback(const struct device *dev,
					struct gpio_callback *callback,
					bool set)
{
	struct gpio_pcf8574_drv_data * const data =
		(struct gpio_pcf8574_drv_data * const)dev->data;

	k_sem_take(&data->lock, K_FOREVER);

	gpio_manage_callback(&data->callbacks, callback, set);

	k_sem_give(&data->lock);
	return 0;
}
#endif

static const struct gpio_driver_api gpio_pcf8574_drv_api_funcs = {
	.pin_configure = gpio_pcf8574_config,
	.port_get_raw = gpio_pcf8574_port_get_raw,
	.port_set_masked_raw = gpio_pcf8574_port_set_masked_raw,
	.port_set_bits_raw = gpio_pcf8574_port_set_bits_raw,
	.port_clear_bits_raw = gpio_pcf8574_port_clear_bits_raw,
	.port_toggle_bits = gpio_pcf8574_port_toggle_bits,
	.pin_interrupt_configure = gpio_pcf8574_pin_interrupt_configure,
#ifdef CONFIG_GPIO_PCF8574_INTERRUPT
	.manage_callback = gpio_pcf8574_manage_callback,
#endif
};

/**
 * @brief Initialization function of PCF8574
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int gpio_pcf8574_init(const struct device *dev)
{
	const struct gpio_pcf8574_drv_config * const config = dev->config;
	struct gpio_pcf8574_drv_data * const data =
		(struct gpio_pcf8574_drv_data * const)dev->data;
	const struct device *i2c_master;

	/* Find out the device struct of the I2C master */
	i2c_master = device_get_binding((char *)config->i2c_master_dev_name);
	if (!i2c_master) {
		return -EINVAL;
	}
	data->i2c_master = i2c_master;

	k_sem_init(&data->lock, 1, 1);

#ifdef CONFIG_GPIO_PCF8574_INTERRUPT
	const struct device *int_gpio_dev;
	int ret;

	/* Store self-reference for interrupt handling */
	data->instance = dev;

	/* Prepare interrupt worker */
	k_work_init(&data->interrupt_worker, gpio_pcf8574_interrupt_worker);

	/* Configure GPIO interrupt pin */
	int_gpio_dev = device_get_binding(config->int_gpio_port);
	if (int_gpio_dev == NULL) {
		LOG_ERR("PCF8574[0x%X]: error getting interrupt GPIO"
			" device (%s)", config->i2c_slave_addr,
			config->int_gpio_port);
		return -ENODEV;
	}

	ret = gpio_pin_configure(int_gpio_dev, config->int_gpio_pin,
				(config->int_gpio_flags | GPIO_INPUT));
	if (ret != 0) {
		LOG_ERR("PCF8574[0x%X]: failed to configure interrupt"
			" pin %d (%d)", config->i2c_slave_addr,
			config->int_gpio_pin, ret);
		return ret;
	}

	/* Prepare GPIO callback for interrupt pin */
	gpio_init_callback(&data->gpio_callback,
			   gpio_pcf8574_interrupt_callback,
			   BIT(config->int_gpio_pin));
	gpio_add_callback(int_gpio_dev, &data->gpio_callback);
#endif

	return 0;
}

#define GPIO_PCF8574_DEVICE_INSTANCE(inst)									\
static const struct gpio_pcf8574_drv_config gpio_pcf8574_##inst##_cfg = {	\
	.common = {																\
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),				\
	},																		\
	.i2c_master_dev_name = DT_INST_BUS_LABEL(inst),							\
	.i2c_slave_addr = DT_INST_REG_ADDR(inst),								\
	IF_ENABLED(CONFIG_GPIO_PCF8574_INTERRUPT, (								\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, interrupt_gpios), (				\
	.int_gpio_port = DT_INST_GPIO_LABEL(inst, interrupt_gpios),				\
	.int_gpio_pin = DT_INST_GPIO_PIN(inst, interrupt_gpios),				\
	.int_gpio_flags = DT_INST_GPIO_FLAGS(inst, interrupt_gpios),			\
	))))																	\
};																			\
																			\
static struct gpio_pcf8574_drv_data gpio_pcf8574_##inst##_drvdata = {		\
	.reg_cache.input = 0xFF,												\
	.reg_cache.output = 0xFF,												\
	.reg_cache.dir = 0xFF,													\
	IF_ENABLED(CONFIG_GPIO_PCF8574_INTERRUPT, (								\
	.interrupt_active = false,												\
	))																		\
};																			\
																			\
DEVICE_AND_API_INIT(gpio_pcf8574_##inst,									\
	DT_INST_LABEL(inst),													\
	gpio_pcf8574_init,														\
	&gpio_pcf8574_##inst##_drvdata,											\
	&gpio_pcf8574_##inst##_cfg,												\
	POST_KERNEL,															\
	CONFIG_GPIO_PCF8574_INIT_PRIORITY,										\
	&gpio_pcf8574_drv_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(GPIO_PCF8574_DEVICE_INSTANCE)
