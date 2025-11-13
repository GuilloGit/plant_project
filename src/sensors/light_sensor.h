#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H
#include <stdint.h>
void light_sensor_init(void);
uint16_t light_sensor_read(void);

#endif
