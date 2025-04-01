#ifndef MS5_H
#define MS5_H

#include <stdbool.h>
#include "stc31/sensirion_config.h"

#define MS5_ADDR 0x77

struct ms5_data_t {
	uint32_t pressure;
	uint32_t temp;
};

int16_t ms5_measure_data(struct ms5_data_t* data);

#endif
