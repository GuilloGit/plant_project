#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#include <stdint.h>

void soil_sensor_init(void);
uint16_t soil_sensor_read(void);

#endif
