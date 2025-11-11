#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <stdint.h>

struct color_data {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
};

void color_sensor_init(void);
int color_sensor_read(struct color_data *data);

#endif
