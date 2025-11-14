#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <stdint.h>

/* ============================================================
 *  Raw data from TCS34725 sensor
 *  ------------------------------------------------------------
 *  clear  → "white"/total light component
 *  red    → red component
 *  green  → green component
 *  blue   → blue component
 * ============================================================ */
struct color_data {
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

/* ============================================================
 *  Detected dominant colors
 * ============================================================ */
enum dominant_color {
    COLOR_UNKNOWN = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
};

/* Initializes the color sensor (I2C + TCS34725 configuration). */
void color_sensor_init(void);

/* 
 * Reads clear, red, green and blue values from TCS34725.
 * 
 * Return:
 *   0  → OK, 'data' contains valid values.
 *  -1  → Error (I2C, sensor disconnected, etc.).
 */
int color_sensor_read(struct color_data *data);

/*
 * Returns the estimated dominant color based on measured values.
 * If light is very low or there's no clear dominant color,
 * returns COLOR_UNKNOWN.
 */
enum dominant_color color_sensor_get_dominant(const struct color_data *data);

#endif /* COLOR_SENSOR_H */
