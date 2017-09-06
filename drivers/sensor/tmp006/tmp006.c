/*
 * Copyright (c) 2017 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <i2c.h>
#include <sensor.h>

#define SYS_LOG_DOMAIN "TMP006"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SENSOR_LEVEL
#include <logging/sys_log.h>

/* TMP006 Result Register */
#define TMP006_TMP_REGISTER  (0x0001)

/* scale in micro degrees Celsius */
#define TMP006_TEMP_SCALE	        31250

#define DEV_DATA(dev) \
	((struct tmp006_data *const)(dev)->driver_data)

struct tmp006_data {
	struct device *i2c;
	s16_t sample;
};

static struct tmp006_data tmp006_driver_data;

static int tmp006_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct tmp006_data *drv_data = DEV_DATA(dev);
	u8_t reg = TMP006_TMP_REGISTER;
	u8_t rx_buf[2];
	struct i2c_msg msgs[2] = {
		{
			.buf = &reg,
			.len = 1,
			.flags = I2C_MSG_WRITE | I2C_MSG_RESTART,
		},
		{
			.buf = rx_buf,
			.len = 2,
			.flags = I2C_MSG_READ | I2C_MSG_STOP,
		},
	};

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_TEMP);

	if (i2c_transfer(drv_data->i2c, msgs, 2, CONFIG_TMP006_I2C_ADDR) < 0) {
		SYS_LOG_DBG("I/O Error on device: %s",
			    CONFIG_TMP006_I2C_MASTER_DEV_NAME);
		return -EIO;
	}

	drv_data->sample = (rx_buf[0] << 6) | (rx_buf[1] >> 2);

	return 0;
}

static int tmp006_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct tmp006_data *drv_data = DEV_DATA(dev);
	s32_t uval;

	if (chan != SENSOR_CHAN_TEMP) {
		return -ENOTSUP;
	}

	uval = drv_data->sample * TMP006_TEMP_SCALE;

	/* Convert to the format required by the sensor API: */
	val->val1 = uval / 1000000;
	val->val2 = uval % 1000000;

	return 0;
}

static const struct sensor_driver_api tmp006_driver_api = {
	.sample_fetch = tmp006_sample_fetch,
	.channel_get = tmp006_channel_get,
};

static int tmp006_init(struct device *dev)
{
	struct tmp006_data *drv_data = DEV_DATA(dev);

	drv_data->i2c = device_get_binding(CONFIG_TMP006_I2C_MASTER_DEV_NAME);
	if (drv_data->i2c == NULL) {
		SYS_LOG_DBG("Could not open device: %s",
			    CONFIG_TMP006_I2C_MASTER_DEV_NAME);
		return -EINVAL;
	}

	dev->driver_api = &tmp006_driver_api;

	return 0;
}

DEVICE_AND_API_INIT(tmp006, CONFIG_TMP006_NAME, tmp006_init,
		    &tmp006_driver_data, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &tmp006_driver_api);
