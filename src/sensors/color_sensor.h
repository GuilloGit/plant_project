#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <stdint.h>

/* ============================================================
 *  Datos crudos del sensor TCS34725
 *  ------------------------------------------------------------
 *  clear  → componente de luz "blanca"/total
 *  red    → componente roja
 *  green  → componente verde
 *  blue   → componente azul
 * ============================================================ */
struct color_data {
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

/* ============================================================
 *  Colores dominantes detectados
 * ============================================================ */
enum dominant_color {
    COLOR_UNKNOWN = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
};

/* Inicializa el sensor de color (I2C + configuración del TCS34725). */
void color_sensor_init(void);

/* 
 * Lee los valores clear, red, green y blue del TCS34725.
 * 
 * Retorno:
 *   0  → OK, 'data' contiene valores válidos.
 *  -1  → Error (I2C, sensor desconectado, etc.).
 */
int color_sensor_read(struct color_data *data);

/*
 * Devuelve el color dominante estimado a partir de los valores
 * medidos. Si la luz es muy baja o no hay un dominante claro,
 * devuelve COLOR_UNKNOWN.
 */
enum dominant_color color_sensor_get_dominant(const struct color_data *data);

#endif /* COLOR_SENSOR_H */
