#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <stdint.h>

/* ============================================================
 *  Temperature and humidity data
 * ============================================================ */
struct temp_hum_data {
    float temperature_c;  /* Temperature in degrees Celsius */
    float humidity_rh;    /* Relative humidity in percentage */
};

/*
 * Initializes the Si7021 sensor on the I2C bus configured
 * in the device tree (i2c2) and verifies it's ready.
 */
void temp_sensor_init(void);

/*
 * Reads temperature and humidity from Si7021 sensor.
 *
 * Return:
 *   0  -> OK, 'data' contains valid values.
 *  -1  -> Error (I2C, sensor disconnected/off, timeout, etc.).
 */
int temp_sensor_read(struct temp_hum_data *data);

#endif /* TEMP_SENSOR_H */
