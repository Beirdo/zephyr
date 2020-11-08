/*
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief GPIO driver for the ZDU0110xxx family
 */

#include <drivers/gpio.h>
#include <zephyr.h>
#include <string.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_zdu0110xxx);

#include <drivers/uart/zdu0110xxx.h>

#include "gpio_utils.h"

struct gpio_zdu0110xxx_drv_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	char *parent_dev_name;
	uint8_t io_count;
	char *dev_type;
	
#ifdef CONFIG_GPIO_ZDU0110XXX_INTERRUPT
	/* Interrupt pin definition */
	const char *int_gpio_port;

	gpio_pin_t int_gpio_pin;

	gpio_flags_t int_gpio_flags;
#endif
};

struct gpio_zdu0110xxx_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	const struct device *parent;
	bool is_qux;

	struct k_sem lock;

	struct {
		uint16_t output;
		uint16_t input;
		uint16_t iodir;
	} reg_cache;

#ifdef CONFIG_GPIO_ZDU0110XXX_INTERRUPT
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

static size_t config_command_append(uint8_t *cmd, size_t cmd_len, uint16_t value, 
		bool is_qux)
{
	if (is_qux) {
		cmd[cmd_len++] = (value >> 8);
	}
	cmd[cmd_len++] = (value & 0xFF);

	return cmd_len;
}

static int gpio_zdu0110xxx_config(const struct device *dev,
				gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_zdu0110xxx_drv_config *config = dev->config;
	struct gpio_zdu0110xxx_drv_data *data = dev->data;
	uint16_t pin_mask;
	uint8_t *cmd;
	size_t cmd_len = 0;
	bool is_qux = data->is_qux;
	int err = 0;
	int ret;

	if (pin >= config->io_count) {
		return -EINVAL;
	}
	
	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	const struct device *parent = data->parent;

	ret = zdu0110xxx_get_buffers(parent, &cmd, NULL);
	if (ret != 0) {
		return ret;
	}
	
	k_sem_take(&data->lock, K_FOREVER);

	pin_mask = 1 << pin;

	cmd[cmd_len++] = ZDU_CMD_GPIO_SET_CONFIG;
	cmd[cmd_len++] = ZDU_SUBCMD_GPIO_OPEN_DRAIN;
	cmd_len = config_command_append(cmd, cmd_len, pin_mask, is_qux);

	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		/* ZDU0110xxx GPIOs support open-drain, but not open-source */
		if ((flags & GPIO_OPEN_DRAIN) == 0) {
			err = -ENOTSUP;
			goto done;
		}
		
		cmd_len = config_command_append(cmd, cmd_len, pin_mask, is_qux);
	} else {
		cmd_len = config_command_append(cmd, cmd_len, 0x0000, is_qux);
	}


	cmd[cmd_len++] = ZDU_CMD_GPIO_SET_CONFIG;
	cmd[cmd_len++] = ZDU_SUBCMD_GPIO_PULLUPS;
	cmd_len = config_command_append(cmd, cmd_len, pin_mask, is_qux);

	if ((flags & GPIO_PULL_DOWN) != 0) {
		/* ZDU0110xxx GPIOs support weak pull-ups, but not pull-downs */
		err = -ENOTSUP;
		goto done;
	} else if ((flags & GPIO_PULL_DOWN) != 0) {
		cmd_len = config_command_append(cmd, cmd_len, pin_mask, is_qux);
	} else {
		cmd_len = config_command_append(cmd, cmd_len, 0x0000, is_qux);
	}

	if (pin > 1) {
		if ((flags & GPIO_INT_ENABLE) != 0) {
			/* ZDU0110xxx only support interrupts on GPIO0 and GPIO1 */
			err = -ENOTSUP;
			goto done;
		}
	}
	
	cmd[cmd_len++] = ZDU_CMD_GPIO_SET_CONFIG;
	cmd[cmd_len++] = ZDU_SUBCMD_GPIO_DIRECTION;
	cmd_len = config_command_append(cmd, cmd_len, pin_mask, is_qux);
		
	switch (flags & GPIO_DIR_MASK) {
	case GPIO_INPUT:
		cmd_len = config_command_append(cmd, cmd_len, 0x0000, is_qux);
		data->reg_cache.iodir &= ~pin_mask;
		break;
	case GPIO_OUTPUT:
		cmd_len = config_command_append(cmd, cmd_len, pin_mask, is_qux);
		data->reg_cache.iodir |= pin_mask;
		
		cmd[cmd_len++] = ZDU_CMD_GPIO_SET_OUTPUT;
		cmd_len = config_command_append(cmd, cmd_len, pin_mask, is_qux);

		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			cmd_len = config_command_append(cmd, cmd_len, pin_mask, is_qux);
			data->reg_cache.output |= pin_mask;
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			cmd_len = config_command_append(cmd, cmd_len, 0x0000, is_qux);
			data->reg_cache.output &= ~pin_mask;
		}
		break;
	default:
		err = -ENOTSUP;
		goto done;
	}

	err = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	ret = zdu0110xxx_release_buffers(parent);

done:
	k_sem_give(&data->lock);

	if (ret != 0) {
		return ret;
	}

	return err;
}

static int gpio_zdu0110xxx_port_get_raw(const struct device *dev,
				      gpio_port_value_t *value)
{
	struct gpio_zdu0110xxx_drv_data *data = dev->data;
	uint8_t cmd[1] = {ZDU_CMD_GPIO_GET_INPUT};
	uint8_t buf[2];
	size_t buf_len = data->is_qux ? 2 : 1;
	int index;
	uint16_t outval = 0;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	ret = zdu0110xxx_send_command(data->parent, cmd, 1, buf, buf_len);
	if (ret == 0) {
		for (index = 0; index < buf_len; index++) {
			outval <<= 8;
			outval += buf[index];
		}

		data->reg_cache.input = outval;
		*value = (gpio_port_value_t)outval;
	}

	k_sem_give(&data->lock);
	
	return ret;
}

static int gpio_zdu0110xxx_port_set_masked_raw(const struct device *dev,
					     gpio_port_pins_t mask,
					     gpio_port_value_t value)
{
	struct gpio_zdu0110xxx_drv_data *data = dev->data;
	bool is_qux = data->is_qux;
	uint8_t *cmd;
	size_t cmd_len = 0;
	uint16_t masked_value;
	int ret;
	int err = 0;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	const struct device *parent = data->parent;
	
	ret = zdu0110xxx_get_buffers(parent, &cmd, NULL);
	if (ret != 0) {
		return ret;
	}

	k_sem_take(&data->lock, K_FOREVER);

	masked_value = value & mask;

	cmd[cmd_len++] = ZDU_CMD_GPIO_SET_OUTPUT;
	cmd_len = config_command_append(cmd, cmd_len, mask, is_qux);
	cmd_len = config_command_append(cmd, cmd_len, value, is_qux);

	data->reg_cache.output &= ~mask;
	data->reg_cache.output |= masked_value;

	err = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);
	
	ret = zdu0110xxx_release_buffers(parent);

	k_sem_give(&data->lock);
	
	if (ret != 0) {
		return ret;
	}
	
	return err;
}

static int gpio_zdu0110xxx_port_set_bits_raw(const struct device *dev,
					   gpio_port_pins_t pins)
{
	return gpio_zdu0110xxx_port_set_masked_raw(dev, pins, pins);
}

static int gpio_zdu0110xxx_port_clear_bits_raw(const struct device *dev,
					     gpio_port_pins_t pins)
{
	return gpio_zdu0110xxx_port_set_masked_raw(dev, pins, 0x0000);
}

static int gpio_zdu0110xxx_port_toggle_bits(const struct device *dev,
					  gpio_port_pins_t pins)
{
	struct gpio_zdu0110xxx_drv_data *data = dev->data;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	uint16_t value = data->reg_cache.output;

	k_sem_give(&data->lock);

	return gpio_zdu0110xxx_port_set_masked_raw(dev, pins, ~value);
}



#ifdef CONFIG_GPIO_ZDU0110XXX_INTERRUPT
static void gpio_zdu0110xxx_interrupt_worker(struct k_work *work)
{
	struct gpio_zdu0110xxx_drv_data * const data = CONTAINER_OF(
		work, struct gpio_zdu0110xxx_drv_data, interrupt_worker);
	uint16_t input_new, input_cache, changed_pins, trig_edge;
	uint16_t trig_level = 0;
	uint16_t triggered_int = 0;
	uint16_t enabled_int = 0;
	int ret;
	uint8_t *cmd;
	size_t cmd_len;
	uint8_t *rsp;
	size_t rsp_len;
	size_t index;

	const struct device *parent = data->parent;
	ret = zdu0110xxx_get_buffers(parent, &cmd, &rsp);
	if (ret != 0) {
		return;
	}

	k_sem_take(&data->lock, K_FOREVER);

    const struct device *dev = data->instance;
    bool is_qux = data->is_qux;
	input_cache = data->reg_cache.input & 0x0003;

	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_GPIO_GET_INPUT;
	rsp_len = is_qux ? 2 : 1;
	
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, rsp, rsp_len);

	if (ret == 0) {
		input_new = 0;
		for (index = 0; index < rsp_len; index++) {
			input_new <<= 8;
			input_new += rsp[index];
		}
		data->reg_cache.input = input_new;
		input_new &= 0x0003;

		cmd_len = 0;
		cmd[cmd_len++] = ZDU_CMD_GPIO_GET_CONFIG;
		cmd[cmd_len++] = ZDU_SUBCMD_GPIO_INTERRUPT;
		ret = zdu0110xxx_send_command(parent, cmd, cmd_len, rsp, 1);
	}

	if (ret == 0) {
		enabled_int = rsp[0];
		
		cmd_len = 0;
		cmd[cmd_len++] = ZDU_CMD_GPIO_GET_INT_STATUS;
		ret = zdu0110xxx_send_command(parent, cmd, cmd_len, rsp, 1);
	}
	
	if (ret == 0) {
		changed_pins = rsp[0];

		/* Note: MCP23017 Interrupt status is cleared by reading status */

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

	ret = zdu0110xxx_release_buffers(parent);
	ARG_UNUSED(ret);
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


static void gpio_zdu0110xxx_interrupt_callback(const struct device *dev,
					    struct gpio_callback *cb,
					    gpio_port_pins_t pins)
{
	struct gpio_zdu0110xxx_drv_data * const data =
		CONTAINER_OF(cb, struct gpio_zdu0110xxx_drv_data, gpio_callback);

	ARG_UNUSED(pins);

	/* Cannot read ZDU0110xxx registers from ISR context, queue worker */
	k_work_submit(&data->interrupt_worker);
}
#endif /* CONFIG_GPIO_ZDU0110XXX_INTERRUPT */



static int gpio_zdu0110xxx_pin_interrupt_configure(const struct device *dev,
						 gpio_pin_t pin,
						 enum gpio_int_mode mode,
						 enum gpio_int_trig trig)
{
	int ret = 0;

	if (pin > 1) {
		/* ZDU0110xxx only supports interrupts on GPIO0, GPIO1 */
		return -ENOTSUP;
	}
	
	if (!IS_ENABLED(CONFIG_GPIO_ZDU0110XXX_INTERRUPT)
	    && (mode != GPIO_INT_MODE_DISABLED)) {
		return -ENOTSUP;
	}

#ifdef CONFIG_GPIO_ZDU0110XXX_INTERRUPT
	const struct gpio_zdu0110xxx_drv_config * const config = dev->config;
	struct gpio_zdu0110xxx_drv_data * data = dev->data;
	const struct device *int_gpio_dev;
	const struct device *parent = data->parent;
	uint8_t i2c_slave_addr = zdu0110xxx_get_slave_addr(parent);
	bool enabled, edge, level, active;
	uint8_t *cmd;
	size_t cmd_len;
	uint8_t *rsp;
	int err = 0;

	/* Check for an invalid pin number */
	if (BIT(pin) > config->common.port_pin_mask) {
		return -EINVAL;
	}

	/* Check configured pin direction */
	if ((mode != GPIO_INT_MODE_DISABLED) &&
	    (BIT(pin) & data->reg_cache.iodir) != 0) {
		LOG_ERR("ZDU0110xxx-GPIO[0x%X]: output pin cannot trigger interrupt",
			i2c_slave_addr);
		return -ENOTSUP;
	}

	ret = zdu0110xxx_get_buffers(parent, &cmd, &rsp);
	if (ret != 0) {
		return ret;
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
			cmd_len = 0;
			cmd[cmd_len++] = ZDU_CMD_GPIO_GET_INT_STATUS;
			err = zdu0110xxx_send_command(parent, cmd, cmd_len, rsp, 1);
		}

		if (err != 0) {
			LOG_ERR("ZDU0110xxx-GPIO[0x%X]: failed to clear potential interrupts "
			    "(%d), disabling", i2c_slave_addr, err);
			active = 0;
		}

		if (active) {
			/* Disable the interrupt on the receiving pin just in case */
			int_gpio_dev = device_get_binding(config->int_gpio_port);
			err = gpio_pin_interrupt_configure(int_gpio_dev,
				config->int_gpio_pin, GPIO_INT_MODE_DISABLED);
			if (err != 0) {
				LOG_ERR("ZDU0110xxx-GPIO[0x%X]: failed to disable interrupt "
					"on pin %d (%d)", i2c_slave_addr, config->int_gpio_pin, err);
				active = 0;
			}
		}
		
		cmd_len = 0;
		cmd[cmd_len++] = ZDU_CMD_GPIO_SET_CONFIG;
		cmd[cmd_len++] = ZDU_SUBCMD_GPIO_INTERRUPT;
		cmd_len = config_command_append(cmd, cmd_len, 0x03, false);
		cmd_len = config_command_append(cmd, cmd_len, active, false);
		
		err = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);
		if (err != 0) {
			LOG_ERR("ZDU0110xxx-GPIO[0x%X]: failed to configure interrupts "
			    "to %02X (%d), disabling", i2c_slave_addr, active, err);
			active = 0;
		}

		if (active) {
			int_gpio_dev = device_get_binding(config->int_gpio_port);
			err = gpio_pin_interrupt_configure(int_gpio_dev,
				config->int_gpio_pin, GPIO_INT_EDGE_TO_ACTIVE);
				
			if (err != 0) {
				LOG_ERR("ZDU0110xxx-GPIO[0x%X]: failed to configure interrupt "
					"on pin %d (%d)", i2c_slave_addr,
					config->int_gpio_pin, err);
				active = 0;
			}
		}

		data->interrupt_active = active;
	}

	ret = zdu0110xxx_release_buffers(parent);

	k_sem_give(&data->lock);
#endif /* CONFIG_GPIO_ZDU0110XXX_INTERRUPT */

	if (ret != 0) {
		return ret;
	}

	return err;
}


#ifdef CONFIG_GPIO_ZDU0110XXX_INTERRUPT
static int gpio_zdu0110xxx_manage_callback(const struct device *dev,
					struct gpio_callback *callback,
					bool set)
{
	struct gpio_zdu0110xxx_drv_data * const data =
		(struct gpio_zdu0110xxx_drv_data * const)dev->data;

	k_sem_take(&data->lock, K_FOREVER);

	gpio_manage_callback(&data->callbacks, callback, set);

	k_sem_give(&data->lock);
	return 0;
}
#endif


static int gpio_zdu0110xxx_init(const struct device *dev)
{
	const struct gpio_zdu0110xxx_drv_config *config = dev->config;
	struct gpio_zdu0110xxx_drv_data *data = dev->data;
	
#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
	const struct device *int_gpio_dev;
	int ret;
#endif

	const struct device *parent = device_get_binding(config->parent_dev_name);
	if (!parent) {
		LOG_ERR("ZDU0110xxx-GPIO: parent ZDU0110xxx device '%s' not found",
			config->parent_dev_name);
		return -EINVAL;
	}
	
	data->parent = parent;

	uint8_t i2c_slave_addr = zdu0110xxx_get_slave_addr(parent);

	data->is_qux = (strcmp(config->dev_type, "qux") == 0);

	k_sem_init(&data->lock, 1, 1);

#ifdef CONFIG_GPIO_MCP23017_INTERRUPT
	/* Store self-reference for interrupt handling */
	data->instance = dev;

	/* Prepare interrupt worker */
	k_work_init(&data->interrupt_worker,
		    gpio_zdu0110xxx_interrupt_worker);

	/* Configure GPIO interrupt pin */
	int_gpio_dev = device_get_binding(config->int_gpio_port);
	if (int_gpio_dev == NULL) {
		LOG_ERR("ZDU0110xxx-GPIO[0x%X]: error getting GPIO interrupt GPIO"
			" device (%s)", i2c_slave_addr,
			config->int_gpio_port);
		return -ENODEV;
	}

	ret = gpio_pin_configure(int_gpio_dev, config->int_gpio_pin,
				(config->int_gpio_flags | GPIO_INPUT));
	if (ret != 0) {
		LOG_ERR("ZDU0110xxx-GPIO[0x%X]: failed to configure GPIO interrupt"
			" pin %d (%d)", i2c_slave_addr,
			config->int_gpio_pin, ret);
		return ret;
	}

	/* Prepare GPIO callback for interrupt pin */
	gpio_init_callback(&data->gpio_callback,
			   gpio_zdu0110xxx_interrupt_callback,
			   BIT(config->int_gpio_pin));
	gpio_add_callback(int_gpio_dev, &data->gpio_callback);
#endif

	return 0;
}


static const struct gpio_driver_api gpio_zdu0110xxx_api = {
	.pin_configure = gpio_zdu0110xxx_config,
	.port_set_masked_raw = gpio_zdu0110xxx_port_set_masked_raw,
	.port_set_bits_raw = gpio_zdu0110xxx_port_set_bits_raw,
	.port_clear_bits_raw = gpio_zdu0110xxx_port_clear_bits_raw,
	.port_toggle_bits = gpio_zdu0110xxx_port_toggle_bits,
	.pin_interrupt_configure = gpio_zdu0110xxx_pin_interrupt_configure,
	.port_get_raw = gpio_zdu0110xxx_port_get_raw,
#ifdef CONFIG_GPIO_ZDU0110XXX_INTERRUPT
	.manage_callback = gpio_zdu0110xxx_manage_callback,
#endif
};

BUILD_ASSERT(CONFIG_GPIO_ZDU0110XXX_INIT_PRIORITY >
	     CONFIG_UART_ZDU0110XXX_INIT_PRIORITY,
	     "ZDU0110xxx GPIO driver must be initialized after ZDU0110xxx UART "
	     "driver");
	     
#define DT_INST_ZDU0110XXX(inst, t) DT_INST(inst, zilog_zdu0110##t##_gpio)
#define GET_INT_GPIO(inst, t)	DT_INST_ZDU0110XXX(inst, t), interrupt_gpios

#define GPIO_ZDU0110XXX_DEVICE(t, id, iocount)								\
	static const struct gpio_zdu0110xxx_drv_config							\
			gpio_zdu0110##t##_##id##_cfg = {								\
		.common = {                                             			\
			.port_pin_mask =                                				\
				 GPIO_PORT_PIN_MASK_FROM_DT_NODE(DT_INST_ZDU0110XXX(id, t))	\
		},                                                      			\
		.parent_dev_name = DT_BUS_LABEL(DT_INST_ZDU0110XXX(id, t)),			\
		.io_count = iocount,												\
		.dev_type = "##t##",												\
	IF_ENABLED(CONFIG_GPIO_ZDU0110XXX_INTERRUPT, (							\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(DT_INST_ZDU0110XXX(id, t), interrupt_gpios),	\
	(	 																	\
		.int_gpio_port = DT_GPIO_LABEL(DT_INST_ZDU0110XXX(id, t), interrupt_gpios),			\
		.int_gpio_pin = DT_GPIO_PIN(DT_INST_ZDU0110XXX(id, t), interrupt_gpios),				\
		.int_gpio_flags = DT_GPIO_FLAGS(DT_INST_ZDU0110XXX(id, t), interrupt_gpios),			\
	))))																	\
	};																		\
																			\
	static struct gpio_zdu0110xxx_drv_data									\
			gpio_zdu0110##t##_##id##_data = {								\
		.reg_cache = {														\
			.output = 0x0000,												\
			.input = 0xFFFF,												\
			.iodir = 0x0000,												\
		},																	\
	IF_ENABLED(CONFIG_GPIO_ZDU0110XXX_INTERRUPT, (							\
  		.interrupt_active = false,											\
	))																		\
	};																		\
																			\
	DEVICE_AND_API_INIT(gpio_zdu0110##t##_##id,								\
			    DT_LABEL(DT_INST_ZDU0110XXX(id, t)),						\
			    &gpio_zdu0110xxx_init,										\
			    &gpio_zdu0110##t##_##id##_data,								\
			    &gpio_zdu0110##t##_##id##_cfg,								\
			    POST_KERNEL,												\
			    CONFIG_GPIO_ZDU0110XXX_INIT_PRIORITY,						\
			    &gpio_zdu0110xxx_api);


#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_ZDU0110XXX_FOREACH(t, inst_expr)				\
	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(zilog_zdu0110##t##_gpio),	\
		     CALL_WITH_ARG, inst_expr)


/*
 * ZDU0110RFX - GPIO0 - GPIO2
 */
#define GPIO_ZDU0110RFX_DEVICE(id)	GPIO_ZDU0110XXX_DEVICE(rfx, id, 3)

/*
 * ZDU0110RHX - GPIO0 - GPIO6
 */
#define GPIO_ZDU0110RHX_DEVICE(id)	GPIO_ZDU0110XXX_DEVICE(rhx, id, 7)

/*
 * ZDU0110RJX - GPIO0 - GPIO7
 */
#define GPIO_ZDU0110RJX_DEVICE(id)	GPIO_ZDU0110XXX_DEVICE(rjx, id, 8)

/*
 * ZDU0110QUX - GPIO0 - GPIO11
 */
#define GPIO_ZDU0110QUX_DEVICE(id)	GPIO_ZDU0110XXX_DEVICE(qux, id, 12)

INST_DT_ZDU0110XXX_FOREACH(rfx, GPIO_ZDU0110RFX_DEVICE);
INST_DT_ZDU0110XXX_FOREACH(rhx, GPIO_ZDU0110RHX_DEVICE);
INST_DT_ZDU0110XXX_FOREACH(rjx, GPIO_ZDU0110RJX_DEVICE);
INST_DT_ZDU0110XXX_FOREACH(qux, GPIO_ZDU0110QUX_DEVICE);
