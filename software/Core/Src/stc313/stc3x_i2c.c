/*
 * THIS FILE IS AUTOMATICALLY GENERATED
 *
 * Generator:     sensirion-driver-generator 0.38.1
 * Product:       stc3x
 * Model-Version: 1.0.0
 */
/*
 * Copyright (c) 2024, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "stc31/stc3x_i2c.h"
#include "stc31/sensirion_common.h"
#include "stc31/sensirion_i2c.h"
#include "stc31/sensirion_i2c_hal.h"

#define sensirion_hal_sleep_us sensirion_i2c_hal_sleep_usec

static uint8_t communication_buffer[47] = {0};

static uint8_t _i2c_address;

static uint32_t _measurement_delay = STC31_MEASUREMENT_DELAY_LOW;

void stc3x_init(uint8_t i2c_address) {
    _i2c_address = i2c_address;
}

int16_t stc3x_get_sensor_state(uint8_t* state, uint16_t state_size) {
    int16_t local_error = 0;
    local_error = stc3x_prepare_read_state();
    if (local_error != NO_ERROR) {
        return local_error;
    }
    local_error = stc3x_read_sensor_state(state, state_size);
    return local_error;
}

int16_t stc3x_set_sensor_state(const uint8_t* sensor_state,
                               uint16_t sensor_state_size) {
    int16_t local_error = 0;
    local_error = stc3x_write_sensor_state(sensor_state, sensor_state_size);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    local_error = stc3x_apply_state();
    return local_error;
}

int16_t stc3x_get_product_id(uint32_t* product_id, uint64_t* serial_number) {
    uint32_t product_number = 0;
    uint32_t serial_number_high = 0;
    uint32_t serial_number_low = 0;
    int16_t local_error = 0;
    local_error = stc3x_prepare_product_identifier();
    if (local_error != NO_ERROR) {
        return local_error;
    }
    local_error = stc3x_read_product_identifier(
        &product_number, &serial_number_high, &serial_number_low);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *product_id = product_number;
    *serial_number = (uint64_t)serial_number_high;
    *serial_number *= 4294967296;
    *serial_number += serial_number_low;
    return local_error;
}

int16_t stc3x_set_binary_gas(uint16_t binary_gas) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3615);
    local_offset = sensirion_i2c_add_uint16_t_to_buffer(
        buffer_ptr, local_offset, binary_gas);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }

    if (binary_gas < 0x10) {
        _measurement_delay = STC31_MEASUREMENT_DELAY_LOW;
    } else {
        _measurement_delay = STC31_MEASUREMENT_DELAY_HIGH;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_set_relative_humidity_raw(uint16_t relative_humidity_ticks) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3624);
    local_offset = sensirion_i2c_add_uint16_t_to_buffer(
        buffer_ptr, local_offset, relative_humidity_ticks);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_set_temperature_raw(uint16_t temperature_ticks) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x361e);
    local_offset = sensirion_i2c_add_uint16_t_to_buffer(
        buffer_ptr, local_offset, temperature_ticks);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_set_pressure(uint16_t absolue_pressure) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x362f);
    local_offset = sensirion_i2c_add_uint16_t_to_buffer(
        buffer_ptr, local_offset, absolue_pressure);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_measure_gas_concentration_raw(uint16_t* gas_ticks,
                                            uint16_t* temperature_ticks) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3639);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(_measurement_delay * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 4);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *gas_ticks = sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
    *temperature_ticks = sensirion_common_bytes_to_uint16_t(&buffer_ptr[2]);
    return local_error;
}

int16_t stc3x_forced_recalibration(uint16_t reference_concentration) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3661);
    local_offset = sensirion_i2c_add_uint16_t_to_buffer(
        buffer_ptr, local_offset, reference_concentration);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(66 * 1000);
    return local_error;
}

int16_t stc3x_enable_automatic_self_calibration() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3fef);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_disable_automatic_self_calibration() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3f6e);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_prepare_read_state() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3752);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_read_sensor_state(uint8_t* state, uint16_t state_size) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0xe133);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 30);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_common_copy_bytes(&buffer_ptr[0], (uint8_t*)state, state_size);
    return local_error;
}

int16_t stc3x_write_sensor_state(const uint8_t* state, uint16_t state_size) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0xe133);
    sensirion_i2c_add_bytes_to_buffer(buffer_ptr, local_offset, state,
                                      state_size);

    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    return local_error;
}

int16_t stc3x_apply_state() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3650);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_self_test(stc3x_test_result_t* self_test_output) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x365b);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(22 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 2);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    (*self_test_output).value =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
    return local_error;
}

int16_t stc3x_prepare_product_identifier() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x367c);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    return local_error;
}

int16_t stc3x_read_product_identifier(uint32_t* product_number,
                                      uint32_t* serial_number_high,
                                      uint32_t* serial_number_low) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0xe102);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(10 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 12);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *product_number = sensirion_common_bytes_to_uint32_t(&buffer_ptr[0]);
    *serial_number_high = sensirion_common_bytes_to_uint32_t(&buffer_ptr[4]);
    *serial_number_low = sensirion_common_bytes_to_uint32_t(&buffer_ptr[8]);
    return local_error;
}

int16_t stc3x_enter_sleep_mode() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3677);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_exit_sleep_mode() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command8_to_buffer(buffer_ptr, local_offset, 0x0);
    sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    sensirion_i2c_hal_sleep_usec(12 * 1000);
    return local_error;
}

int16_t stc3x_enable_weak_filter() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3fc8);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_disable_weak_filter() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3f49);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_enable_strong_filter() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3fd5);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}

int16_t stc3x_disable_strong_filter() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x3f54);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1 * 1000);
    return local_error;
}
