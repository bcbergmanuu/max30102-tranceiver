/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
//#define DT_DRV_COMPAT maxim_max30101

#include <zephyr.h>
#include <stdio.h>
#include <logging/log.h>
#include "max30102.h"

#define register_count 9
#define led_channels 1

const struct device *i2c;


max30102_config max30102cfg = {
// 	.i2c_label = DT_INST_PROP(0, label),
 	.i2c_addr = 0x57// DT_INST_REG_ADDR(0),
};



LOG_MODULE_REGISTER(MAX30102, 0);

int GetSamplesAvailable()
{
	uint8_t fifo_wr_ptr;
	uint8_t fifo_rd_ptr;
	int samples_avalable = 0;

	int wr = i2c_reg_read_byte(i2c, max30102cfg.i2c_addr, 0x04, &fifo_wr_ptr);
	int rd = i2c_reg_read_byte(i2c, max30102cfg.i2c_addr, 0x06, &fifo_rd_ptr);
	if(wr || rd) {
		LOG_ERR("max30102 read or write pointer error");
		return 0;
	}

	if (fifo_wr_ptr != fifo_rd_ptr)
	{
		samples_avalable = fifo_wr_ptr - fifo_rd_ptr;
		if (samples_avalable < 0) {
			samples_avalable += 32; // wrap condition>		
		}
	}	
	return samples_avalable;
}

float temperature(void)
{
	uint8_t temperature;
	uint8_t fraction;
	i2c_reg_read_byte(i2c, max30102cfg.i2c_addr, 0x1F, &temperature);
	i2c_reg_read_byte(i2c, max30102cfg.i2c_addr, 0x20, &fraction);
	if (temperature > 0x7F)
	{
		temperature = -0xFF + (temperature - 1);
	}
	// enable temperature
	i2c_reg_write_byte(i2c, max30102cfg.i2c_addr, 0x21, 1);
	return (float)temperature + fraction * 0.0625;
}


int fetch_max30102(uint32_t * samplebuffer, int samplesToTake)
{
	if (samplesToTake == 0) return 0;	
	
	uint32_t fifo_data;
	int num_bytes = 3 * led_channels * samplesToTake; //2 channels, 3 bytes each
	uint8_t buffer[num_bytes];

	/* Read all the active channels for all samples */		 
	if (i2c_burst_read(i2c, max30102cfg.i2c_addr, 0x07, buffer, num_bytes))
	{
		LOG_ERR("could not read sample");
		return 1;
	}
	
	for (uint32_t y = 0; y < (samplesToTake * led_channels); y++)
	{				
		/* Each channel sample is 18-bits */
		fifo_data = (buffer[y*3] << 16) | (buffer[y*3 + 1] << 8) |
					(buffer[y*3 + 2]);
		fifo_data &= MAX30102_FIFO_DATA_MASK;

		samplebuffer[y] = fifo_data;
	}

	return 0;
}

int max30102data(ppg_item_t * ppg_items, int samplesToTake) {
	int buffersize = samplesToTake * led_channels;
	uint32_t buffer[buffersize]; //two led colors
	if(fetch_max30102(buffer, samplesToTake)){
		return 1;
	}
	unsigned int pic = 0; //ppg item count
	for (size_t i = 0; i < buffersize; i+=led_channels)
	{				
		ppg_items[pic].ir = buffer[i];
#if(led_channels > 1)
		ppg_items[pic].red = buffer[i+1];	
#else
		ppg_items[pic].red = 0;	
#endif
		pic++;
	}
	return 0;
}

uint8_t max30102_interrupt_state() {
	uint8_t status;
	i2c_reg_read_byte(i2c, max30102cfg.i2c_addr, 0x00, &status);
	return status;
}

int safewriteconfig(uint8_t reg, uint8_t value) {
	if (i2c_reg_write_byte(i2c, max30102cfg.i2c_addr, reg,value))
	{
		LOG_ERR("couldn't write to 0x%02x", reg);
		return -EIO;
	}
	return 0;
}

int max30102_init()
{
	uint8_t part_id;
	uint8_t mode_cfg;

	/* Get the I2C device */	
	i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	__ASSERT(device_is_ready(i2c), "max30102 device not ready");
	
	

	/* Check the part id to make sure this is MAX30102 */
	if (i2c_reg_read_byte(i2c, max30102cfg.i2c_addr,
						  MAX30102_REG_PART_ID, &part_id))
	{
		LOG_ERR("Could not get Part ID");
		return -EIO;
	}
	if (part_id != MAX30102_PART_ID)
	{
		LOG_ERR("Got Part ID 0x%02x, expected 0x%02x",
				part_id, MAX30102_PART_ID);
		return -EIO;
	}
	else
	{
		LOG_INF("Got Part ID 0x%02x as expected", part_id);
	}

	/* Reset the sensor */
	if (i2c_reg_write_byte(i2c, max30102cfg.i2c_addr,
						   MAX30102_REG_MODE_CFG,
						   MAX30102_MODE_CFG_RESET_MASK))
	{
		return -EIO;
	}

	/* Wait for reset to be cleared */
	do
	{
		if (i2c_reg_read_byte(i2c, max30102cfg.i2c_addr,
							  MAX30102_REG_MODE_CFG, &mode_cfg))
		{
			LOG_ERR("Could read mode cfg after reset");
			return -EIO;
		}
	} while (mode_cfg & MAX30102_MODE_CFG_RESET_MASK);
	
	uint8_t regsettings[register_count][2] = {
			//Interrupt Enable
			{0x02, 0b10000000},
			//DIE_TEMP_RDY_EN = false
			{0x03, 0b00000000},
			//FIFO config - average, yes rollover, fifo almost full @ 4 left	
			{0x08, 0b11111111},
			// mode configuration
			{0x09, 0b00000010},
			//Sample Rate Control
			{0x0A, 0b01110111},
			//LED pulse amplitude 
			{0x0C, 0x7f},  	//red
			{0x0D, 0x7f},	//ir
			//multi-LED mode control 
			{0x11, 0b00000010},
			{0x12, 0b00000000}
		};		
	
	for (size_t i = 0; i < register_count; i++)
	{		
		if(safewriteconfig(regsettings[i][0], regsettings[i][1])) {
			return -EIO;
		}
	}

	return 0;
}