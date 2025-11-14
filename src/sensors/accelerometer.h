#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>

/* ============================================================
 *  Acceleration data in m/s²
 * ============================================================ */
struct accel_data {
    float x_ms2;
    float y_ms2;
    float z_ms2;
};

/*
 * Initializes the MMA8451 accelerometer on the I2C bus configured
 * in the device tree (i2c2) and sets it to measure in ±2g range.
 */
void accelerometer_init(void);

/*
 * Reads acceleration on the three axes X, Y, Z and returns it in m/s².
 *
 * Return:
 *   0  -> OK, 'data' contains valid values.
 *  -1  -> Error (I2C, sensor disconnected/off, etc.).
 */
int accelerometer_read(struct accel_data *data);

#endif /* ACCELEROMETER_H */
