/*
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* See https://www.zilog.com/docs/PS0389.pdf for datasheet */

#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

#include <zephyr.h>

#define LOG_LEVEL CONFIG_UART_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(uart_zdu0110xxx);

#include <drivers/uart/zdu0110xxx.h>

struct uart_zdu0110xxx_drv_config {
	struct uart_device_config common;
	
	/** The master I2C device's name */
	const char * const i2c_master_dev_name;

	/** The slave address of the chip */
	uint16_t i2c_slave_addr;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* Interrupt pin definition */
	const char *int_gpio_port;

	gpio_pin_t int_gpio_pin;

	gpio_flags_t int_gpio_flags;
#endif
};

struct uart_zdu0110xxx_parent_data {
	/* This needs to be first */
	bool is_port;

	/* Reference to the ports (children) */
	const struct device *children[2];
	uint8_t child_count;

	/** Master I2C device */
	const struct device *i2c_master;

	struct k_sem lock;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* Self-reference to the driver instance */
	const struct device *instance;

	struct gpio_callback gpio_callback;

	struct k_work interrupt_worker;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};


struct uart_zdu0110xxx_port_data {
	/* This needs to be first */
	bool is_port;

	/* Reference to the parent */
	const char * parent_dev_name;
	const struct device *parent;

	uint32_t baudrate;
	uint8_t uart_number;
	uint8_t uart_enable;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uint8_t irq_status;
	uint8_t err_status;
	uint8_t int_enable;

	uart_irq_callback_user_data_t cb; /**< Callback function pointer */
	void *cb_data;  /**< Callback function arg */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};


uint8_t zdu0110xxx_get_slave_addr(const struct device *dev)
{
	const struct uart_zdu0110xxx_drv_config * const config = dev->config;
	return config->i2c_slave_addr;
}


/**
 * @brief Send command to the ZDU0110xxx, and get result.
 *
 * @param dev	        ZDU0110xxx device structure.
 * @param cmd_buf       Data buffer for command data (including command)
 * @param cmd_buf_len   Length of data in cmd_data (not validated yet)
 * @param rsp_buf       Data buffer for command response (NULL if none required)
 * @param rsp_buf_len   Length of data to read into rsp_buf
 *
 * @return	    0 on success.
 */
int zdu0110xxx_send_command(const struct device *dev,
                uint8_t *cmd_buf, size_t cmd_buf_len,
                uint8_t *rsp_buf, size_t rsp_buf_len) 
{
	const struct uart_zdu0110xxx_drv_config * const config = dev->config;
	struct uart_zdu0110xxx_parent_data * data = dev->data;
	const struct device *i2c_master = data->i2c_master;
	uint16_t i2c_addr = config->i2c_slave_addr;
	int ret;

	if (rsp_buf) {
		ret = i2c_write_read(i2c_master, i2c_addr, cmd_buf, cmd_buf_len, 
				rsp_buf, rsp_buf_len);
	} else {
		ret = i2c_write(i2c_master, cmd_buf, cmd_buf_len, i2c_addr);
	}

	if (ret != 0) {
		LOG_ERR("ZDU0110xxx[0x%X]: error sending command 0x%X (%d)",
			i2c_addr, cmd_buf[0], ret);
	}

	return ret;                	
}


static int uart_zdu0110xxx_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return -EINVAL;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[16];
	size_t cmd_len;
	uint8_t rsp[1];
	uint8_t uart_number = data->uart_number;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -1;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_GET_STATUS(uart_number);
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, rsp, 1);
	if (ret != 0) {
		ret = -EFAULT;
		goto done;
	}
	
	if ((rsp[0] & 0x20) == 0) {
		/* RX Ready bit not set */
		ret = -1;
		goto done;
	}
	
	/* Read data into *c */
	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_RX_DATA(uart_number);
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, c, 1);

done:
	k_sem_give(&parent_data->lock);

	return ret;
}

static void uart_zdu0110xxx_poll_out(const struct device *dev,
				     unsigned char c)
{
	const struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[16];
	size_t cmd_len;
	uint8_t uart_number = data->uart_number;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	/* Write data from c */
	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_TX_DATA(uart_number);
	cmd[cmd_len++] = c;
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	k_sem_give(&parent_data->lock);
	
	ARG_UNUSED(ret);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_zdu0110xxx_fifo_fill(const struct device *dev,
						const uint8_t *tx_data, int size)
{
	const struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return -EINVAL;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[65];
	size_t cmd_len;
	int num_tx = 0;
	int index;
	uint8_t rsp[2];
	uint8_t tx_fifo_level;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	/* Write data from tx_data into the TX FIFO until data is all sent,
	 * or FIFO is full.  Return number of bytes successfully sent
	 */

	/* Pull how much is in the TX FIFO */
	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_GET_FIFO_LEVELS(data->uart_number);
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, rsp, 2);
	
	if (ret != 0) {
		goto done;
	}
	
	tx_fifo_level = rsp[1];
	
	num_tx = size > 64 - tx_fifo_level ? 64 - tx_fifo_level : size;
	
	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_TX_DATA(data->uart_number);
	
	for (index = 0; index < num_tx; index++) {
		cmd[cmd_len++] = tx_data[index]; 
	}
	
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	if (ret != 0) {
		num_tx = 0;
	}	

done:
	k_sem_give(&parent_data->lock);

	return num_tx;
}

static int uart_zdu0110xxx_fifo_read(const struct device *dev,
							uint8_t *rx_data,
							const int size)
{
	const struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return -EINVAL;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[16];
	size_t cmd_len;
	int num_rx = 0;
	uint8_t rsp[2];
	uint8_t rx_fifo_level;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	/* Read data from the RX FIFO into rx_data until buffer is full,
	 * or FIFO is empty.  Return number of bytes successfully received
	 */

	/* Pull how much is in the RX FIFO */
	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_GET_FIFO_LEVELS(data->uart_number);
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, rsp, 2);
	
	if (ret != 0) {
		goto done;
	}
	
	rx_fifo_level = rsp[0];
	
	num_rx = size > rx_fifo_level ? rx_fifo_level : size;
	
	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_RX_DATA(data->uart_number);
	
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, rx_data, num_rx);

	if (ret != 0) {
		num_rx = 0;
	}	

done:
	k_sem_give(&parent_data->lock);

	return num_rx;
}

static void uart_zdu0110xxx_irq_tx_enable(const struct device *dev)
{
	struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[4];
	size_t cmd_len;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_INT_ENABLE(data->uart_number);
	data->int_enable |= 0xC0;	/* All TX interrupts */
	
	cmd[cmd_len++] = data->int_enable;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_ENABLE(data->uart_number);
	data->uart_enable |= 0x02;	/* TXE */
	cmd[cmd_len++] = data->uart_enable;
	
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	k_sem_give(&parent_data->lock);

	ARG_UNUSED(ret);
}

static void uart_zdu0110xxx_irq_tx_disable(const struct device *dev)
{
	struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[4];
	size_t cmd_len;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_INT_ENABLE(data->uart_number);
	data->int_enable &= 0x1F;	/* All TX interrupts disabled */
	
	cmd[cmd_len++] = data->int_enable;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_ENABLE(data->uart_number);
	data->uart_enable &= 0x01;	/* TXE off */
	cmd[cmd_len++] = data->uart_enable;
	
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	k_sem_give(&parent_data->lock);

	ARG_UNUSED(ret);
}

static int uart_zdu0110xxx_irq_tx_ready(const struct device *dev)
{
	const struct uart_zdu0110xxx_port_data *data = dev->data;

	/* TX Full is off. */
	return ((data->irq_status & 0x20) == 0x00);
}

static void uart_zdu0110xxx_irq_rx_enable(const struct device *dev)
{
	struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[4];
	size_t cmd_len;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_INT_ENABLE(data->uart_number);
	data->int_enable |= 0x0C;	/* All RX interrupts */
	
	cmd[cmd_len++] = data->int_enable;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_ENABLE(data->uart_number);
	data->uart_enable |= 0x01;	/* RXE */
	cmd[cmd_len++] = data->uart_enable;
	
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	k_sem_give(&parent_data->lock);

	ARG_UNUSED(ret);
}

static void uart_zdu0110xxx_irq_rx_disable(const struct device *dev)
{
	struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[4];
	size_t cmd_len;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_INT_ENABLE(data->uart_number);
	data->int_enable &= 0xC1;	/* All RX interrupts disabled */
	
	cmd[cmd_len++] = data->int_enable;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_ENABLE(data->uart_number);
	data->uart_enable &= 0x02;	/* RXE disabled */
	cmd[cmd_len++] = data->uart_enable;
	
	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	k_sem_give(&parent_data->lock);

	ARG_UNUSED(ret);
}

static int uart_zdu0110xxx_irq_tx_complete(const struct device *dev)
{
	const struct uart_zdu0110xxx_port_data *data = dev->data;

	/* TX Empty is on */
	return ((data->irq_status & 0x80) == 0x80);
}

static int uart_zdu0110xxx_irq_rx_ready(const struct device *dev)
{
	const struct uart_zdu0110xxx_port_data *data = dev->data;

	/* RX Data is on */
	return ((data->irq_status & 0x08) == 0x08);
}

static void uart_zdu0110xxx_irq_err_enable(const struct device *dev)
{
	struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[4];
	size_t cmd_len;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_INT_ENABLE(data->uart_number);
	data->int_enable |= 0x01;	/* ERR interrupt */
	cmd[cmd_len++] = data->int_enable;

	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	k_sem_give(&parent_data->lock);

	ARG_UNUSED(ret);
}

static void uart_zdu0110xxx_irq_err_disable(const struct device *dev)
{
	struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint8_t cmd[4];
	size_t cmd_len;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_INT_ENABLE(data->uart_number);
	data->int_enable &= 0xCC;	/* ERR interrupt disabled */
	cmd[cmd_len++] = data->int_enable;

	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	k_sem_give(&parent_data->lock);

	ARG_UNUSED(ret);
}

static int uart_zdu0110xxx_irq_is_pending(const struct device *dev)
{
	const struct uart_zdu0110xxx_port_data *data = dev->data;

	/* Any IRQ pending */
	return ((data->irq_status & data->int_enable) != 0x00);
}

static int uart_zdu0110xxx_irq_update(const struct device *dev)
{
    struct uart_zdu0110xxx_port_data *data = dev->data;
	const struct device *parent = data->parent;
	if (!parent) {
		return -EINVAL;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

    uint8_t uart_number = data->uart_number;
    uint8_t cmd[16];
    size_t cmd_len;
    int ret;
    
	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_GET_INT_STATUS(uart_number);
	ret = zdu0110xxx_send_command(dev, cmd, cmd_len, &data->irq_status, 1);
	if (ret != 0) {
		goto done;
	}
	
	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_GET_STATUS(uart_number);
	ret = zdu0110xxx_send_command(dev, cmd, cmd_len, &data->err_status, 1);

done:
	k_sem_give(&parent_data->lock);

	return 1;
}

static void uart_zdu0110xxx_irq_callback_set(const struct device *dev,
					     uart_irq_callback_user_data_t cb,
					     void *cb_data)
{
	struct uart_zdu0110xxx_port_data *data = dev->data;

	data->cb = cb;
	data->cb_data = cb_data;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 *
 * @return N/A
 */
#if 0
static void uart_zdu0110xxx_isr(const struct device *dev)
{
	struct uart_zdu0110xxx_parent_data *data = dev->data;

	/* Cannot read ZDU0110xxx registers from ISR context, queue worker */
	k_work_submit(&data->interrupt_worker);
}
#endif


static void uart_zdu0110xxx_interrupt_callback(const struct device *dev,
					    struct gpio_callback *cb,
					    gpio_port_pins_t pins)
{
	struct uart_zdu0110xxx_parent_data * const data =
		CONTAINER_OF(cb, struct uart_zdu0110xxx_parent_data, gpio_callback);

	ARG_UNUSED(pins);

	/* Cannot read ZDU0110xxx registers from ISR context, queue worker */
	k_work_submit(&data->interrupt_worker);
}

static void uart_zdu0110xxx_interrupt_worker(struct k_work *work)
{
	struct uart_zdu0110xxx_parent_data * const data = CONTAINER_OF(
		work, struct uart_zdu0110xxx_parent_data, interrupt_worker);
	int index;

	k_sem_take(&data->lock, K_FOREVER);

    /* This is triggered by either UART.  We need to check both */
    for (index = 0; index < data->child_count; index++) {
		const struct device *child = data->children[index];
		struct uart_zdu0110xxx_port_data *port_data = child->data;
			
		if (port_data->cb) {
			port_data->cb(child, port_data->cb_data);
		}
    }
    
	k_sem_give(&data->lock);
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


static int uart_zdu0110xxx_port_init(const struct device *dev);

static int uart_zdu0110xxx_parent_init(const struct device *dev)
{
	const struct uart_zdu0110xxx_drv_config *config = dev->config;
	struct uart_zdu0110xxx_parent_data *data = dev->data;
	const struct device *int_gpio_dev;

	if (data->is_port) {
		return uart_zdu0110xxx_port_init(dev);
	}

	int ret = 0;

	k_sem_init(&data->lock, 1, 1);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uint8_t cmd[16];
	size_t cmd_len;

	/* Configure UART parent */
	cmd_len = 0;

	/* Store self-reference for interrupt handling */
	data->instance = dev;

	/* Prepare interrupt worker */
	k_work_init(&data->interrupt_worker, uart_zdu0110xxx_interrupt_worker);

	/* Configure GPIO interrupt pin */
	int_gpio_dev = device_get_binding(config->int_gpio_port);
	if (int_gpio_dev == NULL) {
		LOG_ERR("ZDU0110xxx[0x%X]: error getting UART interrupt GPIO"
			" device (%s)", config->i2c_slave_addr,
			config->int_gpio_port);
		return -ENODEV;
	}

	ret = gpio_pin_configure(int_gpio_dev, config->int_gpio_pin,
				(config->int_gpio_flags | GPIO_INPUT));
	if (ret != 0) {
		LOG_ERR("ZDU0110xxx[0x%X]: failed to configure UART interrupt"
			" pin %d (%d)", config->i2c_slave_addr,
			config->int_gpio_pin, ret);
		return ret;
	}

	/* Prepare GPIO callback for interrupt pin */
	gpio_init_callback(&data->gpio_callback,
			   uart_zdu0110xxx_interrupt_callback,
			   BIT(config->int_gpio_pin));
	gpio_add_callback(int_gpio_dev, &data->gpio_callback);

	ret = zdu0110xxx_send_command(dev, cmd, cmd_len, NULL, 0);
#endif

	return ret;
}


static int zdu0110xxx_set_baud_rate(const struct device *dev, uint32_t baudrate)
{
	struct uart_zdu0110xxx_drv_config *config = 
			(struct uart_zdu0110xxx_drv_config *)dev->config;
	struct uart_zdu0110xxx_port_data *data = dev->data;
			
	const struct device *parent = data->parent;
	if (!parent) {
		return -EINVAL;
	}
	
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	uint16_t baud_counter;
	uint32_t actual_baud;
	int8_t baud_percent_variance;
	uint8_t uart_number = data->uart_number;
	uint8_t cmd[16];
	size_t cmd_len;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&parent_data->lock, K_FOREVER);

	/* Baud rate settings calculated from 8MHz */
	baud_counter = (uint16_t)(((baudrate / 10) + 5) / 10);
	baud_counter = (uint16_t)(10000 / (((100000 / baud_counter) + 5) / 10));
	actual_baud = baud_counter * 100;
	
	baud_percent_variance = (int8_t)(((actual_baud - baudrate) * 100) / baudrate);
	
	if ((baud_percent_variance > 3) || (baud_percent_variance < -3)) {
		LOG_ERR("ZDU0110[0x%X]: requested baud rate (%d) not attainable,"
				"closest match is %d, which is %d%% off",
			config->i2c_slave_addr, baudrate, actual_baud, baud_percent_variance);
		return -EINVAL;
	}

	data->baudrate = actual_baud;

	/* Configure UART */
	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_BAUD(uart_number);
	cmd[cmd_len++] = (baud_counter >> 8);
	cmd[cmd_len++] = (baud_counter & 0xFF);

	ret = zdu0110xxx_send_command(parent, cmd, cmd_len, NULL, 0);

	k_sem_give(&parent_data->lock);

	return ret;
}

static int uart_zdu0110xxx_port_init(const struct device *dev)
{
	struct uart_zdu0110xxx_port_data *data = dev->data;
			
	if (!data->is_port) {
		return uart_zdu0110xxx_parent_init(dev);
	}

	data->parent = device_get_binding(data->parent_dev_name);
	if (!data->parent) {
		LOG_ERR("parent ZDU0110xxx device '%s' not found",
			data->parent_dev_name);
		return -EINVAL;
	}

	const struct device *parent = data->parent;
	const struct uart_zdu0110xxx_drv_config *config = parent->config;
	struct uart_zdu0110xxx_parent_data *parent_data = parent->data;

	((struct device *)dev)->config = config;

	uint8_t uart_number = data->uart_number;
	
	parent_data->children[uart_number] = dev;
	
	uint8_t config2;
	uint8_t cmd[16];
	size_t cmd_len;
	int ret;

	ret = zdu0110xxx_set_baud_rate(dev, data->baudrate);

	if (ret != 0) {
		return ret;
	}

	/* Configure UART */
	cmd_len = 0;
	cmd[cmd_len++] = ZDU_CMD_UART_SET_CONFIG(uart_number);
	cmd[cmd_len++] = 0x86;	/* N81 - also default */
	config2 = 0x00;
#ifdef CONFIG_UART_ZDU0110_FLOW_CONTROL_SOFTWARE
	config2 |= 0x04;
#endif
#ifdef CONFIG_UART_ZDU0110_FLOW_CONTROL_HARDWARE
	config2 |= 0x02;
#endif
	cmd[cmd_len++] = config2;	/* RX/TX disabled, no loopback, flow control */
	
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	data->int_enable = 0;
	data->uart_enable = 0;
#else
	cmd[cmd_len++] = ZDU_CMD_UART_SET_ENABLE(uart_number);
	cmd{cmd_len++] = 0x03;	/* RXE, TXE */
#endif

	return zdu0110xxx_send_command(dev, cmd, cmd_len, NULL, 0);
}

int uart_zdu0110xxx_line_ctrl_set(const struct device *dev, uint32_t ctrl,
			     uint32_t val)
{
	if (ctrl == UART_LINE_CTRL_BAUD_RATE) {	
		return zdu0110xxx_set_baud_rate(dev, val);
	}
	
	return -ENOTSUP;
}

int uart_zdu0110xxx_line_ctrl_get(const struct device *dev, uint32_t ctrl,
			     uint32_t *val)
{
	const struct uart_zdu0110xxx_drv_config *config = dev->config;
	const struct uart_zdu0110xxx_port_data *data = dev->data;
	uint8_t uart_number = data->uart_number;

	if (!val) {
		return 0;
	}
	
	if (ctrl == UART_LINE_CTRL_BAUD_RATE) {	
		*val = data->baudrate;
		return 0;
	}
	
	return -ENOTSUP;
}



static const struct uart_driver_api uart_zdu0110xxx_driver_api = {
	.poll_in          = uart_zdu0110xxx_poll_in,
	.poll_out         = uart_zdu0110xxx_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill	      = uart_zdu0110xxx_fifo_fill,
	.fifo_read	      = uart_zdu0110xxx_fifo_read,
	.irq_tx_enable	  = uart_zdu0110xxx_irq_tx_enable,
	.irq_tx_disable	  = uart_zdu0110xxx_irq_tx_disable,
	.irq_tx_ready	  = uart_zdu0110xxx_irq_tx_ready,
	.irq_rx_enable	  = uart_zdu0110xxx_irq_rx_enable,
	.irq_rx_disable	  = uart_zdu0110xxx_irq_rx_disable,
	.irq_tx_complete  = uart_zdu0110xxx_irq_tx_complete,
	.irq_rx_ready	  = uart_zdu0110xxx_irq_rx_ready,
	.irq_err_enable	  = uart_zdu0110xxx_irq_err_enable,
	.irq_err_disable  = uart_zdu0110xxx_irq_err_disable,
	.irq_is_pending	  = uart_zdu0110xxx_irq_is_pending,
	.irq_update		  = uart_zdu0110xxx_irq_update,
	.irq_callback_set = uart_zdu0110xxx_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_LINE_CTRL
	.line_ctrl_set    = uart_zdu0110xxx_line_ctrl_set,
	.line_ctrl_get    = uart_zdu0110xxx_line_ctrl_get,
#endif

};



#define DT_INST_MASTER(inst, t) DT_INST(inst, zilog_zdu0110##t##)
#define DT_INST_PORT(inst, t) DT_INST(inst, zilog_zdu0110##t##_port)

#define GET_CURRENT_SPEED_COMMA(id)	DT_PROP(id, current_speed),
#define NULL_COMMA(id)		NULL,
#define GET_INT_GPIO(id, t)	(DT_INST_MASTER(id, t), interrupt_gpios)


#define UART_ZDU0110XXX_PORT_DEVICE(t, id)									\
	static struct uart_zdu0110xxx_port_data									\
			uart_zdu0110##t##_port_##id##_data = {							\
		.is_port = true,													\
		.parent_dev_name = DT_BUS_LABEL(DT_INST_PORT(id, t)),				\
		.uart_number = DT_PROP(DT_INST_PORT(id, t), port_number),			\
		.baudrate = DT_PROP(DT_INST_PORT(id, t), current_speed),			\
		.uart_enable = 0x00,												\
	IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, (								\
	    .irq_status = 0x00, 												\
	    .err_status = 0x00, 												\
	    .int_enable = 0x00, 												\
	))																		\
	};																		\
																			\
	DEVICE_AND_API_INIT(zilog_zdu0110##t##_port_##id,						\
			    DT_PROP(DT_INST_PORT(id, t), label),						\
			    &uart_zdu0110xxx_port_init,									\
			    &uart_zdu0110##t##_port_##id##_data,						\
			    NULL,														\
			    POST_KERNEL,												\
			    CONFIG_GPIO_ZDU0110XXX_INIT_PRIORITY + 1,					\
			    &uart_zdu0110xxx_driver_api);


#define UART_ZDU0110XXX_PARENT_DEVICE(t, id, uartcount)						\
	static const struct uart_zdu0110xxx_drv_config							\
			uart_zdu0110##t##_##id##_cfg = {								\
		.parent_dev_name = DT_INST_BUS_LABEL(DT_INST_MASTER(id, t)),		\
		.uart_count = uartcount,											\
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, (							\
			.cb = NULL,														\
			IF_ENABLED(DT_INST_NODE_HAS_PROP(GET_INT_GPIO(id, t)), (		\
			.int_gpio_port = DT_INST_GPIO_LABEL(GET_INT_GPIO(id, t)),		\
			.int_gpio_pin = DT_INST_GPIO_PIN(GET_INT_GPIO(id, t)),			\
			.int_gpio_flags = DT_INST_GPIO_FLAGS(GET_INT_GPIO(id, t)),		\
		)))),																\
	};																		\
																			\
	static struct uart_zdu0110xxx_parent_data								\
			uart_zdu0110##t##_##id##_data = {								\
		.is_port = false,													\
		.child_count = uartcount,											\
		.children = { DT_FOREACH_CHILD(DT_INST_MASTER, NULL_COMMA) },		\
	};																		\
																			\
	DEVICE_INIT(uart_zdu0110##t##_##id,										\
			    DT_PROP(DT_INST_MASTER(id, t), label),						\
			    &uart_zdu0110xxx_parent_init,								\
			    &uart_zdu0110##t##_##id##_data,								\
			    &uart_zdu0110##t##_##id##_cfg,								\
			    POST_KERNEL,												\
			    CONFIG_GPIO_ZDU0110XXX_INIT_PRIORITY);


#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_ZDU0110XXX_FOREACH(t, inst_expr)				\
	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(zilog_zdu0110##t),	\
		     CALL_WITH_ARG, inst_expr)

#define INST_DT_ZDU0110XXX_PORT_FOREACH(t, inst_expr)				\
	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(zilog_zdu0110##t##_port),	\
		     CALL_WITH_ARG, inst_expr)


/*
 * ZDU0110RFX - one UART
 */
#define UART_ZDU0110RFX_PARENT_INSTANCE(id)	\
	UART_ZDU0110XXX_PARENT_DEVICE(rfx, id, 1)
#define UART_ZDU0110RFX_PORT_INSTANCE(id) UART_ZDU0110XXX_PORT_DEVICE(rfx, id)



/*
 * ZDU0110RHX - one UART
 */
#define UART_ZDU0110RHX_PARENT_INSTANCE(id)	\
	UART_ZDU0110XXX_PARENT_DEVICE(rhx, id, 1)
#define UART_ZDU0110RHX_PORT_INSTANCE(id) UART_ZDU0110XXX_PORT_DEVICE(rhx, id)


/*
 * ZDU0110RJX - two UARTs
 */
#define UART_ZDU0110RJX_PARENT_INSTANCE(id)	\
	UART_ZDU0110XXX_PARENT_DEVICE(rjx, id, 1)
#define UART_ZDU0110RJX_PORT_INSTANCE(id) UART_ZDU0110XXX_PORT_DEVICE(rjx, id)


/*
 * ZDU0110QUX - two UARTs
 */
#define UART_ZDU0110QUX_PARENT_INSTANCE(id)	\
	UART_ZDU0110XXX_PARENT_DEVICE(qux, id, 1)
#define UART_ZDU0110QUX_PORT_INSTANCE(id) UART_ZDU0110XXX_PORT_DEVICE(qux, id)


INST_DT_ZDU0110XXX_FOREACH(rfx, UART_ZDU0110RFX_PARENT_INSTANCE);
INST_DT_ZDU0110XXX_PORT_FOREACH(rfx, UART_ZDU0110RFX_PORT_INSTANCE);

INST_DT_ZDU0110XXX_FOREACH(rhx, UART_ZDU0110RHX_PARENT_INSTANCE);
INST_DT_ZDU0110XXX_PORT_FOREACH(rhx, UART_ZDU0110RHX_PORT_INSTANCE);

INST_DT_ZDU0110XXX_FOREACH(rjx, UART_ZDU0110RJX_PARENT_INSTANCE);
INST_DT_ZDU0110XXX_PORT_FOREACH(rjx, UART_ZDU0110RJX_PORT_INSTANCE);

INST_DT_ZDU0110XXX_FOREACH(qux, UART_ZDU0110QUX_PARENT_INSTANCE);
INST_DT_ZDU0110XXX_PORT_FOREACH(qux, UART_ZDU0110QUX_PORT_INSTANCE);

