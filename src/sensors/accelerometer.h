#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>

/* ============================================================
 *  Datos de aceleración en m/s²
 * ============================================================ */
struct accel_data {
    float x_ms2;
    float y_ms2;
    float z_ms2;
};

/*
 * Inicializa el acelerómetro MMA8451 en el bus I2C configurado
 * en el device tree (i2c1) y lo deja midiendo en rango ±2g.
 */
void accelerometer_init(void);

/*
 * Lee la aceleración en los tres ejes X, Y, Z y la devuelve en m/s².
 *
 * Retorno:
 *   0  -> OK, 'data' contiene valores válidos.
 *  -1  -> Error (I2C, sensor desconectado/apagado, etc.).
 */
int accelerometer_read(struct accel_data *data);

#endif /* ACCELEROMETER_H */
