/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The logic here is adapted from SimpleLink SDK's I2CCC32XX.c module. */

#include <kernel.h>
#include <errno.h>
#include <i2c.h>
#include <soc.h>
#include "i2c-priv.h"

/* Driverlib includes */
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/i2c.h>

#define I2C_MASTER_CMD_BURST_RECEIVE_START_NACK	 I2C_MASTER_CMD_BURST_SEND_START
#define I2C_MASTER_CMD_BURST_RECEIVE_STOP \
	I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP
#define I2C_MASTER_CMD_BURST_RECEIVE_CONT_NACK	 I2C_MASTER_CMD_BURST_SEND_CONT

#define DEV_CFG(dev) \
	((const struct i2c_cc32xx_config *const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2c_cc32xx_data *const)(dev)->driver_data)
#define DEV_BASE(dev) \
	((DEV_CFG(dev))->base)


enum i2c_cc32xx_state {
	/* I2C is idle, and not performing a transaction */
	I2C_CC32XX_IDLE_MODE = 0,
	/* I2C is currently performing a write operation */
	I2C_CC32XX_WRITE_MODE,
	/* I2C is currently performing a read operation */
	I2C_CC32XX_READ_MODE,
	/* I2C error has occurred */
	I2C_CC32XX_ERROR = 0xFF
};

struct i2c_cc32xx_config {
	u32_t base;
	u32_t bitrate;
};

struct i2c_cc32xx_data {
	struct k_sem mutex;
	struct k_sem transfer_complete;

	volatile enum i2c_cc32xx_state state;

	u8_t  *write_buf_idx;  /* Internal inc. writeBuf index */
	size_t write_cnt;      /* Internal dec. writeCounter */
	u8_t  *read_buf_idx;   /* Internal inc. readBuf index */
	size_t read_cnt;       /* Internal dec. readCounter */
	u16_t  slave_addr;     /* Cache slave address for ISR use */
};

static void configure_i2c_irq(void);

static int i2c_cc32xx_configure(struct device *dev, u32_t dev_config_raw)
{
	u32_t base = DEV_BASE(dev);
	union dev_config dev_config = (union dev_config)dev_config_raw;
	u32_t bitrate_id;

	if (!dev_config.bits.is_master_device) {
		return -EINVAL;
	}

	if (dev_config.bits.use_10_bit_addr) {
		return -EINVAL;
	}

	switch (dev_config.bits.speed) {
	case I2C_SPEED_STANDARD:
		bitrate_id = 0;
		break;
	case I2C_SPEED_FAST:
		bitrate_id = 1;
		break;
	default:
		return -EINVAL;
	}

	MAP_I2CMasterInitExpClk(base, CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
				bitrate_id);

	return 0;
}

static void _i2c_cc32xx_prime_transfer(struct device *dev, struct i2c_msg *msg,
				      u16_t addr)
{
	struct i2c_cc32xx_data *data = DEV_DATA(dev);
	u32_t base = DEV_BASE(dev);

	/* Initialize internal counters and buf pointers: */
	if (msg->flags & I2C_MSG_READ) {
		data->read_buf_idx = msg->buf;
		data->read_cnt = msg->len;
	} else {
		data->write_buf_idx = msg->buf;
		data->write_cnt = msg->len;
	}
	data->slave_addr = addr;

	/* Start transfer in Transmit mode */
	if (data->write_cnt) {
		/* Specify the I2C slave address */
		MAP_I2CMasterSlaveAddrSet(base, addr, false);

		/* Update the I2C state */
		data->state = I2C_CC32XX_WRITE_MODE;

		/* Write data contents into data register */
		MAP_I2CMasterDataPut(base, *((data->write_buf_idx)++));

		/* Start the I2C transfer in master transmit mode */
		MAP_I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_START);

	} else {
		/* Start transfer in Receive mode */
		/* Specify the I2C slave address */
		MAP_I2CMasterSlaveAddrSet(base, addr, true);

		/* Update the I2C mode */
		data->state = I2C_CC32XX_READ_MODE;

		if (data->read_cnt < 2) {
			/* Start the I2C transfer in master receive mode */
			MAP_I2CMasterControl(base,
				       I2C_MASTER_CMD_BURST_RECEIVE_START_NACK);
		} else {
			/* Start the I2C transfer in burst receive mode */
			MAP_I2CMasterControl(base,
					    I2C_MASTER_CMD_BURST_RECEIVE_START);
		}
	}
}

static int i2c_cc32xx_transfer(struct device *dev, struct i2c_msg *msgs,
			       u8_t num_msgs, u16_t addr)
{
	struct i2c_cc32xx_data *data = DEV_DATA(dev);
	int retval = 0;

	/* Iterate over all the messages */
	for (int i = 0; i < num_msgs; i++) {
		/* Acquire the driver mutex */
		k_sem_take(&data->mutex, K_FOREVER);

		/* Protect prime_transfer from irq: */
		irq_disable(TI_CC32XX_I2C_40020000_IRQ_0);
		_i2c_cc32xx_prime_transfer(dev, msgs, addr);
		irq_enable(TI_CC32XX_I2C_40020000_IRQ_0);

		/* Wait for the transfer to complete */
		k_sem_take(&data->transfer_complete, K_FOREVER);

		/* Release the mutex */
		k_sem_give(&data->mutex);

		/* Return an error if the transfer didn't complete */
		if (data->state == I2C_CC32XX_ERROR) {
			retval = -EIO;
			break;
		}

		/* Move to the next message */
		msgs++;
	}

	return retval;
}

static void _i2c_cc32xx_isr_handle_write(u32_t base,
					 struct i2c_cc32xx_data *data)
{
	/* Decrement write Counter */
	data->write_cnt--;

	/* Check if more data needs to be sent */
	if (data->write_cnt) {

		/* Write data contents into data register */
		MAP_I2CMasterDataPut(base, *(data->write_buf_idx));
		data->write_buf_idx++;

		if ((data->write_cnt < 2) && !(data->read_cnt)) {
			/* Everything has been sent, nothing to receive */
			/* Next state: Idle mode */
			data->state = I2C_CC32XX_IDLE_MODE;

			/* Send last byte with STOP bit */
			MAP_I2CMasterControl(base,
					     I2C_MASTER_CMD_BURST_SEND_FINISH);
		} else {
			/*
			 * Either there is more date to be transmitted or some
			 * data needs to be received next
			 */
			MAP_I2CMasterControl(base,
					     I2C_MASTER_CMD_BURST_SEND_CONT);

		}
	} else {
		/* At this point, we know that we need to receive data */
		/*
		 * We need to check after we are done transmitting data, if
		 * we need to receive any data.
		 * In a corner case when we have only one byte transmitted
		 * and no data to receive, the I2C will automatically send
		 * the STOP bit. In other words, here we only need to check
		 * if data needs to be received. If so, how much.
		 */
		if (data->read_cnt) {
			/* Next state: Receive mode */
			data->state = I2C_CC32XX_READ_MODE;

			/* Switch into Receive mode */
			MAP_I2CMasterSlaveAddrSet(base, data->slave_addr, true);

			if (data->read_cnt > 1) {
				/* Send a repeated START */
				MAP_I2CMasterControl(base,
					  I2C_MASTER_CMD_BURST_RECEIVE_START);
			} else {
				/*
				 * Send a repeated START with a NACK since it's
				 * the last byte to be received.
				 * I2C_MASTER_CMD_BURST_RECEIVE_START_NACK is
				 * is locally defined because there is no macro
				 * to receive data and send a NACK after sending
				 * a start bit (0x00000003)
				 */
				MAP_I2CMasterControl(base,
				      I2C_MASTER_CMD_BURST_RECEIVE_START_NACK);
			}
		} else {
			/* Done with all transmissions */
			data->state = I2C_CC32XX_IDLE_MODE;
			/*
			 * No more data needs to be received, so follow up with
			 * a STOP bit.
			 * Again, there is no equivalent macro (0x00000004) so
			 * I2C_MASTER_CMD_BURST_RECEIVE_STOP is used.
			 */
			MAP_I2CMasterControl(base,
					     I2C_MASTER_CMD_BURST_RECEIVE_STOP);
		}
	}
}

static void _i2c_cc32xx_isr_handle_read(u32_t base,
					struct i2c_cc32xx_data *data)
{

	/* Save the received data */
	*(data->read_buf_idx) = MAP_I2CMasterDataGet(base);
	data->read_buf_idx++;

	/* Check if any data needs to be received */
	data->read_cnt--;
	if (data->read_cnt) {
		if (data->read_cnt > 1) {
			/* More data to be received */
			MAP_I2CMasterControl(base,
					     I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		} else {
			/*
			 * Send NACK because it's the last byte to be received
			 * There is no NACK macro equivalent (0x00000001) so
			 * I2C_MASTER_CMD_BURST_RECEIVE_CONT_NACK is used
			 */
			MAP_I2CMasterControl(base,
				       I2C_MASTER_CMD_BURST_RECEIVE_CONT_NACK);
		}
	} else {
		/* Next state: Idle mode */
		data->state = I2C_CC32XX_IDLE_MODE;

		/*
		 * No more data needs to be received, so follow up with a
		 * STOP bit
		 * Again, there is no equivalent macro (0x00000004) so
		 * I2C_MASTER_CMD_BURST_RECEIVE_STOP is used
		 */
		MAP_I2CMasterControl(base,
				     I2C_MASTER_CMD_BURST_RECEIVE_STOP);
	}
}

static void i2c_cc32xx_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	u32_t base = DEV_BASE(dev);
	struct i2c_cc32xx_data *data = DEV_DATA(dev);
	u32_t err_status;

	/* Get the interrupt status of the I2C controller */
	err_status = MAP_I2CMasterErr(base);

	/* Clear interrupt source to avoid additional interrupts */
	MAP_I2CMasterIntClear(base);

	/* Check for I2C Errors */
	if ((err_status == I2C_MASTER_ERR_NONE) ||
	    (data->state == I2C_CC32XX_ERROR)) {
		/* No errors, now check what we need to do next */
		switch (data->state) {
		/*
		 * ERROR case is OK because if an Error is detected, a STOP bit
		 * is sent; which in turn will call another interrupt. This
		 * interrupt call will then post the transfer_complete semaphore
		 * to unblock the transfer function.
		 */
		case I2C_CC32XX_ERROR:
		case I2C_CC32XX_IDLE_MODE:
			/* Indicate transfer complete */
			k_sem_give(&data->transfer_complete);
			break;

		case I2C_CC32XX_WRITE_MODE:
			_i2c_cc32xx_isr_handle_write(base, data);
			break;

		case I2C_CC32XX_READ_MODE:
			_i2c_cc32xx_isr_handle_read(base, data);
			break;

		default:
			data->state = I2C_CC32XX_ERROR;
			break;
		}
	} else {
		/* Some sort of error happened */
		data->state = I2C_CC32XX_ERROR;

		if (err_status & (I2C_MASTER_ERR_ARB_LOST |
				  I2C_MASTER_ERR_ADDR_ACK)) {
			k_sem_give(&data->transfer_complete);
		} else {
			/* Send a STOP bit to end I2C communications */
			/*
			 * I2C_MASTER_CMD_BURST_SEND_ERROR_STOP -and-
			 * I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP
			 * have the same values
			 */
			MAP_I2CMasterControl(base,
					  I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
			k_sem_give(&data->transfer_complete);
		}
	}
}

static int i2c_cc32xx_init(struct device *dev)
{
	u32_t base = DEV_BASE(dev);
	const struct i2c_cc32xx_config *config = DEV_CFG(dev);
	struct i2c_cc32xx_data *data = DEV_DATA(dev);
	u32_t bitrate_cfg;
	int error;
	u32_t regval;

	k_sem_init(&data->mutex, 1, UINT_MAX);
	k_sem_init(&data->transfer_complete, 0, UINT_MAX);

	/* In case of app restart: disable I2C module, clear NVIC interrupt */
	/* Note: this was done *during* pinmux setup in SimpleLink SDK. */
	MAP_I2CMasterDisable(base);
	/* Clear INT_I2CA0 */
	MAP_IntPendClear((unsigned long)(TI_CC32XX_I2C_40020000_IRQ_0 + 16));

	configure_i2c_irq();

	data->state = I2C_CC32XX_IDLE_MODE;

	/* Take I2C hardware semaphore. */
	regval = HWREG(0x400F7000);
	regval = (regval & ~0x3) | 0x1;
	HWREG(0x400F7000) = regval;

	/* Set to default configuration: */
	bitrate_cfg = _i2c_map_dt_bitrate(config->bitrate);
	error = i2c_cc32xx_configure(dev, I2C_MODE_MASTER | bitrate_cfg);
	if (error) {
		return error;
	}

	/* Clear any pending interrupts */
	MAP_I2CMasterIntClear(base);

	/* Enable the I2C Master for operation */
	MAP_I2CMasterEnable(base);

	/* Unmask I2C interrupts */
	MAP_I2CMasterIntEnable(base);

	return 0;
}

static const struct i2c_driver_api i2c_cc32xx_driver_api = {
	.configure = i2c_cc32xx_configure,
	.transfer = i2c_cc32xx_transfer,
};


static const struct i2c_cc32xx_config i2c_cc32xx_config = {
	.base = TI_CC32XX_I2C_40020000_BASE_ADDRESS,
	.bitrate = TI_CC32XX_I2C_40020000_CLOCK_FREQUENCY,
};

static struct i2c_cc32xx_data i2c_cc32xx_data;

DEVICE_AND_API_INIT(i2c_cc32xx, TI_CC32XX_I2C_40020000_LABEL, &i2c_cc32xx_init,
		    &i2c_cc32xx_data, &i2c_cc32xx_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &i2c_cc32xx_driver_api);

static void configure_i2c_irq(void)
{
	IRQ_CONNECT(TI_CC32XX_I2C_40020000_IRQ_0,
		    TI_CC32XX_I2C_40020000_IRQ_0_PRIORITY,
		    i2c_cc32xx_isr, DEVICE_GET(i2c_cc32xx), 0);

	irq_enable(TI_CC32XX_I2C_40020000_IRQ_0);
}
