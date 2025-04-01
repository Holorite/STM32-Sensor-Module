/*
 * ms5.c
 *
 *  Created on: Mar 11, 2025
 *      Author: julian
 */

#include "stc31/ms5.h"
#include "stc31/sensirion_common.h"
#include "stc31/sensirion_i2c.h"
#include "stc31/sensirion_i2c_hal.h"

#include <stm32u0xx_hal.h>

static uint8_t communication_buffer[20] = {0};

static uint8_t _i2c_address = MS5_ADDR;

int16_t ms5_measure_data(struct ms5_data_t* data) {
	int16_t local_error = NO_ERROR;
	uint8_t* buffer_ptr = communication_buffer;
	uint16_t local_offset = 0;
	local_offset =
		sensirion_i2c_add_command8_to_buffer(buffer_ptr, local_offset, 0x48);
	local_error =
		sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
	if (local_error != NO_ERROR) {
		return local_error;
	}

	sensirion_i2c_hal_sleep_usec(10 * 1000);

	local_offset = 0;
	local_offset = sensirion_i2c_add_command8_to_buffer(buffer_ptr, local_offset, 0x00);
	local_error =  sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
	if (local_error != NO_ERROR) {
			return local_error;
	}

	sensirion_i2c_hal_sleep_usec(1000);

	local_error = sensirion_i2c_hal_read(_i2c_address, buffer_ptr, 3);
	//if (local_error == 1) local_error = NO_ERROR; // crc error expected due to sensiron function
	if (local_error != NO_ERROR) {
		return local_error;
	}
	data->pressure = ((uint32_t) buffer_ptr[0] << 16) | ((uint32_t) buffer_ptr[1] << 8) | (uint32_t) buffer_ptr[2];
	return local_error;
}
