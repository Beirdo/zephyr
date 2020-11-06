/*
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ADC driver for the MCP342x I2C ADCs.
 */

#include <drivers/adc.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr.h>

LOG_MODULE_REGISTER(adc_mcp342x, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define MCP342X_RESOLUTION 12U

struct mcp342x_drv_config {
	uint8_t channels;
	
	/** The master I2C device's name */
	const char * const i2c_master_dev_name;

	/** The slave address of the chip */
	uint16_t i2c_slave_addr;
};

struct mcp342x_drv_data {
	struct adc_context ctx;

	const struct adc_channel_cfg channel_cfg[4];
	
	/* bitmap of configured channels */
	uint8_t channels;

	/* sample buffer */
	uint16_t *buffer;
	uint16_t *repeat_buffer;

	/* Pointer back to the device */
	const struct device *dev;
	
	/** Master I2C device */
	const struct device *i2c_master;

	struct k_sem lock;
	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(stack,
			CONFIG_ADC_MCP342X_ACQUISITION_THREAD_STACK_SIZE);
};

static int mcp342x_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct mcp342x_drv_config *config = dev->config;
	struct mcp342x_drv_data *data = dev->data;

	switch(channel_cfg->gain){
		case ADC_GAIN_1:
		case ADC_GAIN_2:
		case ADC_GAIN_4:
		case ADC_GAIN_8:
			/* These all exist */
		default:
			LOG_ERR("unsupported channel gain '%d'", channel_cfg->gain);
			return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("unsupported channel reference '%d'",
			channel_cfg->reference);
		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("unsupported acquisition_time '%d'",
			channel_cfg->acquisition_time);
		return -ENOTSUP;
	}

	if (channel_cfg->channel_id >= config->channels) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	memcpy((void *)&data->channel_cfg[channel_cfg->channel_id], (const void *)channel_cfg, sizeof(channel_cfg));

	return 0;
}

static int mcp342x_validate_buffer_size(const struct device *dev,
					const struct adc_sequence *sequence)
{
	const struct mcp342x_drv_config *config = dev->config;
	uint8_t channels = 0;
	size_t needed;
	uint32_t mask;

	for (mask = BIT(config->channels - 1); mask != 0; mask >>= 1) {
		if (mask & sequence->channels) {
			channels++;
		}
	}

	needed = channels * sizeof(uint16_t);
	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int mcp342x_start_read(const struct device *dev,
			      const struct adc_sequence *sequence)
{
	const struct mcp342x_drv_config *config = dev->config;
	struct mcp342x_drv_data *data = dev->data;
	int err;

	switch (sequence->resolution) {
		case 12:
		case 14:
		case 16:
			/* These are supported */
			break;
		default:
			LOG_ERR("unsupported resolution %d", sequence->resolution);
			return -ENOTSUP;
	}	

	if (find_msb_set(sequence->channels) > config->channels) {
		LOG_ERR("unsupported channels in mask: 0x%08x",
			sequence->channels);
		return -ENOTSUP;
	}

	err = mcp342x_validate_buffer_size(dev, sequence);
	if (err) {
		LOG_ERR("buffer size too small");
		return err;
	}

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int mcp342x_read_async(const struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct mcp342x_drv_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = mcp342x_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int mcp342x_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	return mcp342x_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct mcp342x_drv_data *data = CONTAINER_OF(ctx, struct mcp342x_drv_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	k_sem_give(&data->lock);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct mcp342x_drv_data *data = CONTAINER_OF(ctx, struct mcp342x_drv_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int mcp342x_read_channel(const struct device *dev, 
				struct adc_context *ctx, uint8_t channel, uint16_t *result)
{
	const struct mcp342x_drv_config *config = dev->config;
	struct mcp342x_drv_data *data = dev->data;
	
	uint8_t tx_config;
	uint8_t rx_buffer[3];
	/* need to get this from the sequence! */
	uint8_t resolution = ctx->sequence.resolution;	
	uint32_t acq_time;
	int err;

	/* 
	 * 240 SPS (4167uS) for 12 bit, 
	 * 60 SPS (16667uS) for 14 bit,
	 * 15 SPS (66667uS) for 16 bit
	 */
	
	switch (resolution) {
		case 12:
			acq_time = 4167;
			tx_config = 0x00;
			break;
		case 14:
			acq_time = 16667;
			tx_config = 0x04;
			break;
		case 16:
			acq_time = 66667;
			tx_config = 0x08;
			break;
		default:
			return -ENOTSUP;
	}

	switch (data->channel_cfg[channel].gain) {
		case ADC_GAIN_1:
			tx_config |= 0x00;
			break;
		case ADC_GAIN_2:
			tx_config |= 0x01;
			break;
		case ADC_GAIN_4:
			tx_config |= 0x02;
			break;
		case ADC_GAIN_8:
			tx_config |= 0x03;
			break;
		default:
			return -ENOTSUP;
	}

	/* Setup for one-shot reading on the requested channel */
	tx_config |= 0x90 | (channel << 5);

	err = i2c_write(data->i2c_master, &tx_config, 1, config->i2c_slave_addr);
	if (err) {
		return err;
	}

	/* Delay for the sample time before reading */
	k_usleep(acq_time);

	err = i2c_read(data->i2c_master, rx_buffer, 3, config->i2c_slave_addr);
	if (err) {
		return err;
	}

	*result = sys_get_be16(rx_buffer);
	*result &= BIT_MASK(resolution);

	if ((rx_buffer[2] & 0x80) != 0) {
		return -EBUSY;
	}

	return 0;
}

static void mcp342x_acquisition_thread(struct mcp342x_drv_data *data)
{
	uint16_t result = 0;
	uint8_t channel;
	int err;

	while (true) {
		k_sem_take(&data->lock, K_FOREVER);

		while (data->channels) {
			channel = find_lsb_set(data->channels) - 1;

			LOG_DBG("reading channel %d", channel);

			err = mcp342x_read_channel(data->dev, &data->ctx, channel, &result);
			if (err) {
				LOG_ERR("failed to read channel %d (err %d)",
					channel, err);
				adc_context_complete(&data->ctx, err);
				break;
			}

			LOG_DBG("read channel %d, result = %d", channel,
				result);

			*data->buffer++ = result;
			WRITE_BIT(data->channels, channel, 0);
		}

		adc_context_on_sampling_done(&data->ctx, data->dev);
	}
}

static int mcp342x_init(const struct device *dev)
{
	const struct mcp342x_drv_config *config = dev->config;
	struct mcp342x_drv_data *data = dev->data;
	const struct device *i2c_master;

	data->dev = dev;

	/* Find out the device struct of the I2C master */
	i2c_master = device_get_binding((char *)config->i2c_master_dev_name);
	if (!i2c_master) {
		return -EINVAL;
	}
	data->i2c_master = i2c_master;

	k_sem_init(&data->lock, 0, 1);
	
	k_thread_create(&data->thread, data->stack,
			CONFIG_ADC_MCP342X_ACQUISITION_THREAD_STACK_SIZE,
			(k_thread_entry_t)mcp342x_acquisition_thread,
			data, NULL, NULL,
			CONFIG_ADC_MCP342X_ACQUISITION_THREAD_PRIO,
			0, K_NO_WAIT);

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api mcp342x_adc_api = {
	.channel_setup = mcp342x_channel_setup,
	.read = mcp342x_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = mcp342x_read_async,
#endif
	.ref_internal = 2048,
};

#define INST_DT_MCP342X(inst, t) DT_INST(inst, microchip_mcp##t)

#define MCP342X_DEVICE(t, n, ch)											\
	static struct mcp342x_drv_data mcp##t##_data_##n = {					\
		ADC_CONTEXT_INIT_TIMER(mcp##t##_data_##n, ctx), 					\
		ADC_CONTEXT_INIT_LOCK(mcp##t##_data_##n, ctx),						\
		ADC_CONTEXT_INIT_SYNC(mcp##t##_data_##n, ctx),						\
	};																		\
	static const struct mcp342x_drv_config mcp##t##_config_##n = {			\
		.channels = ch, 													\
		.i2c_master_dev_name = DT_BUS_LABEL(INST_DT_MCP342X(n, t)),	\
		.i2c_slave_addr = DT_REG_ADDR_BY_IDX(INST_DT_MCP342X(n, t), 0),		\
	};																		\
	DEVICE_AND_API_INIT(mcp##t##_##n,										\
			    DT_LABEL(INST_DT_MCP342X(n, t)),							\
			    &mcp342x_init,												\
			    &mcp##t##_data_##n, 										\
			    &mcp##t##_config_##n,										\
			    POST_KERNEL,												\
			    CONFIG_ADC_MCP320X_INIT_PRIORITY,							\
			    &mcp342x_adc_api);

/*
 * MCP3425: 1 channel
 */
#define MCP3425_DEVICE(n) MCP342X_DEVICE(3425, n, 1)

/*
 * MCP3426: 2 channels
 */
#define MCP3426_DEVICE(n) MCP342X_DEVICE(3426, n, 2)

/*
 * MCP3427: 2 channels
 */
#define MCP3427_DEVICE(n) MCP342X_DEVICE(3427, n, 2)

/*
 * MCP3428: 4 channels
 */
#define MCP3428_DEVICE(n) MCP342X_DEVICE(3428, n, 4)


#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_MCP342X_FOREACH(t, inst_expr)				\
	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(microchip_mcp##t),	\
		     CALL_WITH_ARG, inst_expr)

INST_DT_MCP342X_FOREACH(3425, MCP3425_DEVICE)
INST_DT_MCP342X_FOREACH(3426, MCP3426_DEVICE)
INST_DT_MCP342X_FOREACH(3427, MCP3427_DEVICE)
INST_DT_MCP342X_FOREACH(3428, MCP3428_DEVICE)