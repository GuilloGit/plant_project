#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <stdint.h>

/* ============================================================
 *  Datos de temperatura y humedad
 * ============================================================ */
struct temp_hum_data {
    float temperature_c;  /* Temperatura en grados Celsius */
    float humidity_rh;    /* Humedad relativa en porcentaje */
};

/*
 * Inicializa el sensor Si7021 en el bus I2C configurado
 * en el device tree (i2c2) y verifica que esté listo.
 */
void temp_sensor_init(void);

/*
 * Lee la temperatura y humedad del sensor Si7021.
 *
 * Retorno:
 *   0  -> OK, 'data' contiene valores válidos.
 *  -1  -> Error (I2C, sensor desconectado/apagado, timeout, etc.).
 */
int temp_sensor_read(struct temp_hum_data *data);

#endif /* TEMP_SENSOR_H */
