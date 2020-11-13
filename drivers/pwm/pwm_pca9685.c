/*
 * Copyright (c) 2015 Intel Corporation.
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Driver for PCA9685 I2C-based PWM driver.
 */

#include <errno.h>

#include <kernel.h>

#include <drivers/i2c.h>
#include <drivers/pwm.h>

#include "pwm_pca9685.h"

#define DT_DRV_COMPAT nxp_pca9685

#define REG_MODE1		0x00
#define REG_MODE2		0x01

#define REG_LED_ON_L(n)		((4 * n) + 0x06)
#define	REG_LED_ON_H(n)		((4 * n) + 0x07)
#define REG_LED_OFF_L(n)	((4 * n) + 0x08)
#define REG_LED_OFF_H(n)	((4 * n) + 0x09)

#define REG_ALL_LED_ON_L	0xFA
#define REG_ALL_LED_ON_H	0xFB
#define REG_ALL_LED_OFF_L	0xFC
#define REG_ALL_LED_OFF_H	0xFD
#define REG_PRE_SCALE		0xFE

/* Maximum PWM outputs */
#define MAX_PWM_OUT		16

/* Special case to turn all on/off */
#define ALL_PWM_OUTPUT	0xFF

/* How many ticks per one period */
#define PWM_ONE_PERIOD_TICKS	4096

/**
 * @brief Check to see if a I2C master is identified for communication.
 *
 * @param dev Device struct.
 * @return 1 if I2C master is identified, 0 if not.
 */
static inline int has_i2c_master(const struct device *dev)
{
	struct pwm_pca9685_drv_data * const drv_data =
		(struct pwm_pca9685_drv_data * const)dev->data;
	const struct device *i2c_master = drv_data->i2c_master;

	if (i2c_master) {
		return 1;
	} else {
		return 0;
	}
}

/*
 * period_count is always taken as 4095. To control the on period send
 * value to pulse_count
 */
static int pwm_pca9685_pin_set_cycles(const struct device *dev, uint32_t pwm,
				      uint32_t period_count, uint32_t pulse_count,
				      pwm_flags_t flags)
{
	const struct pwm_pca9685_config * const config =
		dev->config;
	struct pwm_pca9685_drv_data * const drv_data =
		(struct pwm_pca9685_drv_data * const)dev->data;
	const struct device *i2c_master = drv_data->i2c_master;
	uint16_t i2c_addr = config->i2c_slave_addr;
	uint8_t buf[5];
	uint16_t on_time;
	uint16_t off_time;
	bool inverted;
	size_t buf_len;

	if (!has_i2c_master(dev)) {
		return -EINVAL;
	}

	if (pwm > MAX_PWM_OUT && pwm != ALL_PWM_OUTPUT) {
		return -EINVAL;
	}

	inverted = ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED);

	buf_len = 0;
	if (pwm == ALL_PWM_OUTPUT) {
		buf[buf_len++] = REG_ALL_LED_ON_L;
	} else {
		buf[buf_len++] = REG_LED_ON_L(pwm);
	}

	/* If either pulse_count > max ticks, treat PWM as 100%.
	 * If pulse_count value == 0, treat it as 0%.
	 * Otherwise, populate registers accordingly.
	 */
	if (pulse_count >= PWM_ONE_PERIOD_TICKS) {
		on_time = 0x1000;
		off_time = 0x0000;
	} else if (pulse_count == 0U) {
		on_time = 0x0000;
		off_time = 0x1000;
	} else {
		if ((flags & PWM_FLAG_START_DELAY) != 0) {
			on_time = period_count & 0x0FFF;
		} else {
			on_time = 0x0000;
		}
		off_time = pulse_count;
	}
	
	if (inverted) {
		uint16_t temp = on_time;
		on_time = off_time;
		off_time = temp;
	}
	
	buf[buf_len++] = (on_time >> 8) & 0xFF;
	buf[buf_len++] = on_time & 0xFF;
	buf[buf_len++] = (off_time >> 8) & 0xFF;
	buf[buf_len++] = off_time & 0xFF;

	return i2c_write(i2c_master, buf, buf_len, i2c_addr);
}


static int pwm_pca9685_get_cycles_per_sec(const struct device *dev,
					uint32_t pwm,
					uint64_t *cycles)
{
	struct pwm_pca9685_drv_data * const drv_data =
		(struct pwm_pca9685_drv_data * const)dev->data;

	if (!cycles) {
		return -EINVAL;
	}

	/* This device has a global clock */
	ARG_UNUSED(pwm);
	
	*cycles = drv_data->cycles_per_sec;
	return 0;
}


static const struct pwm_driver_api pwm_pca9685_drv_api_funcs = {
	.pin_set =  pwm_pca9685_pin_set_cycles,
	.get_cycles_per_sec = pwm_pca9685_get_cycles_per_sec,
};

/**
 * @brief Initialization function of PCA9685
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
int pwm_pca9685_init(const struct device *dev)
{
	const struct pwm_pca9685_config * const config = dev->config;
	struct pwm_pca9685_drv_data * const drv_data =
		(struct pwm_pca9685_drv_data * const)dev->data;
	const struct device *i2c_master;
	uint8_t buf[6];
	size_t buf_len;
	int ret;

	/* Find out the device struct of the I2C master */
	i2c_master = device_get_binding((char *)config->i2c_master_dev_name);
	if (!i2c_master) {
		return -EINVAL;
	}
	
	if ((config->prescaler < 3) || (config->prescaler > 0xFF)) {
		return -EINVAL;
	}

	drv_data->cycles_per_sec = (25000000LL / (config->prescaler + 1));
	drv_data->i2c_master = i2c_master;

	/* MODE1 register */
	buf_len = 0;
	buf[buf_len++] = REG_MODE1;
	buf[buf_len++] = (1 << 5); /* register addr auto increment */

	ret = i2c_write(i2c_master, buf, 2, config->i2c_slave_addr);
	if (ret != 0) {
		return -EPERM;
	}
	
	/* Turn off all LEDs by default, and set the Prescale from the 
	 * requested clock frequency 
	 */
	buf_len = 0;
	buf[buf_len++] = REG_ALL_LED_ON_L;
	buf[buf_len++] = 0x00;
	buf[buf_len++] = 0x00;
	buf[buf_len++] = 0x10;
	buf[buf_len++] = 0x00;
	buf[buf_len++] = config->prescaler & 0xFF;
	ret = i2c_write(i2c_master, buf, 2, config->i2c_slave_addr);
	if (ret != 0) {
		return -EPERM;
	}

	return 0;
}


#define PWM_PCA9685_DEVICE_INSTANCE(id)								\
static const struct pwm_pca9685_config pwm_pca9685_##id##_cfg = {	\
	.i2c_master_dev_name = DT_INST_BUS_LABEL(id),					\
	.i2c_slave_addr = DT_INST_REG_ADDR(id),							\
	.prescaler = DT_INST_PROP(id, prescaler),						\
};																	\
																	\
static struct pwm_pca9685_drv_data pwm_pca9685_##id##_drvdata;		\
																	\
/* This has to init after I2C master */								\
DEVICE_AND_API_INIT(pwm_pca9685_##id,								\
	DT_INST_LABEL(id),												\
	pwm_pca9685_init,												\
	&pwm_pca9685_##id##_drvdata,									\
	&pwm_pca9685_##id##_cfg,										\
	POST_KERNEL,													\
	CONFIG_PWM_PCA9685_INIT_PRIORITY,								\
	&pwm_pca9685_drv_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(PWM_PCA9685_DEVICE_INSTANCE)
