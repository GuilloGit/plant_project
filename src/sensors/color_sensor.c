
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include "color_sensor.h"
#include <stdlib.h>

/* ============================================================
 *  Configuración del TCS34725 e I2C
 * ============================================================ */

/* Dirección I2C del TCS34725 (7 bits) */
#define TCS34725_I2C_ADDR            0x29

/* Bit de comando que se OR-ea con la dirección de registro */
#define TCS34725_CMD_BIT             0x80

/* Registros principales del TCS34725 */
#define TCS34725_REG_ENABLE          0x00
#define TCS34725_REG_ATIME           0x01
#define TCS34725_REG_CONTROL         0x0F
#define TCS34725_REG_CDATAL          0x14  /* Clear data low byte (luego +1, +2, etc.) */

/* Bits del registro ENABLE */
#define TCS34725_ENABLE_PON          0x01  /* Power ON */
#define TCS34725_ENABLE_AEN          0x02  /* ADC Enable */

/* Configuración recomendada (el primero en el datasheet) */
/* ATIME = 0xF6 → tiempo de integración (~24 ms aprox.) */
#define TCS34725_ATIME_24MS          0xF6

/* CONTROL = 0x01 → ganancia 4x (ejemplo típico) */
#define TCS34725_GAIN_4X             0x01

/* Umbral mínimo para considerar que hay luz suficiente
 * antes de decidir un color dominante.
 */
#define TCS34725_CLEAR_MIN_THRESHOLD 50

/* Nodo I2C en el device tree */
#define COLOR_I2C_NODE               DT_NODELABEL(i2c2)
#define COLOR_I2C_DEV_NODE          DT_NODELABEL(tcs34725)

/* ============================================================
 *  Variables estáticas
 * ============================================================ */

static const struct device *color_i2c_dev;
static bool color_last_error = false;


/* ============================================================
 *  Funciones internas de acceso al TCS34725
 * ============================================================ */

/*
 * Escribe un registro de 8 bits en el TCS34725.
 * r: dirección del registro
 * v: valor a escribir
 */
static int tcs34725_write_reg(uint8_t reg_addr, uint8_t value)
{
    uint8_t buf[2] = {
        (uint8_t)(TCS34725_CMD_BIT | reg_addr),
        value
    };

    return i2c_write(color_i2c_dev, buf, sizeof(buf), TCS34725_I2C_ADDR);
}

/*
 * Lee un registro de 16 bits (low + high) del TCS34725.
 * r: dirección del byte low (ej. CDATAL)
 * out: puntero donde se guarda el resultado
 */
static int tcs34725_read_u16(uint8_t reg_addr, uint16_t *out)
{
    uint8_t reg = (uint8_t)(TCS34725_CMD_BIT | reg_addr);
    uint8_t buf[2];

    int ret = i2c_write_read(color_i2c_dev,
                             TCS34725_I2C_ADDR,
                             &reg, 1,
                             buf, sizeof(buf));
    if (ret != 0) {
        return ret;
    }

    /* El TCS34725 entrega los datos en formato little-endian:
     * buf[0] = LSB, buf[1] = MSB
     */
    *out = ((uint16_t)buf[1] << 8) | buf[0];
    return 0;
}

/*
 * Configura el TCS34725: power-on, enable ADC, tiempo de integración y ganancia.
 */
static int tcs34725_configure(void)
{
    int ret;

    /* Encender el oscilador interno */
    ret = tcs34725_write_reg(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON);
    if (ret != 0) {
        return ret;
    }

    /* Espera mínima para que el oscilador se estabilice (datasheet ~2.4ms) */
    k_msleep(3);

    /* Activar el ADC de color (PON + AEN) */
    ret = tcs34725_write_reg(TCS34725_REG_ENABLE,
                             TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    if (ret != 0) {
        return ret;
    }

    /* Configurar tiempo de integración */
    ret = tcs34725_write_reg(TCS34725_REG_ATIME, TCS34725_ATIME_24MS);
    if (ret != 0) {
        return ret;
    }

    /* Configurar ganancia */
    ret = tcs34725_write_reg(TCS34725_REG_CONTROL, TCS34725_GAIN_4X);
    if (ret != 0) {
        return ret;
    }

    return 0;
}


/* ============================================================
 *  color_sensor_init()
 *  ------------------------------------------------------------
 *  Inicializa el dispositivo I2C y configura el TCS34725 para
 *  comenzar a medir colores.
 * ============================================================ */
void color_sensor_init(void)
{
    color_i2c_dev = DEVICE_DT_GET(COLOR_I2C_NODE);
   // color_i2c_dev = DEVICE_DT_GET(COLOR_I2C_DEV_NODE);

    printk("Color: I2C device pointer = %p\n", color_i2c_dev);

    if (!device_is_ready(color_i2c_dev)) {
        printk("Color: ERROR (I2C no disponible)\n");
        color_last_error = true;
        return;
    }

    printk("Color: I2C device ready, configuring TCS34725...\n");

    if (tcs34725_configure() != 0) {
        printk("Color: ERROR (configuración TCS34725)\n");
        color_last_error = true;
        return;
    }

    printk("Color: TCS34725 configured successfully\n");
    color_last_error = false;
}


/* ============================================================
 *  color_sensor_read()
 *  ------------------------------------------------------------
 *  Lee los cuatro canales del TCS34725: clear, red, green, blue.
 *
 *  Manejo de errores:
 *   - Si cualquier lectura I2C falla → se asume que el módulo
 *     puede estar desconectado o apagado.
 *   - Si todos los canales son 0 (C=R=G=B=0), se considera un
 *     estado sospechoso (posible fallo o sensor sin alimentación).
 *
 *  Retorno:
 *    0  -> OK, estructura 'd' rellena.
 *   -1  -> Algún error (I2C o valores inconsistentes).
 * ============================================================ */
int color_sensor_read(struct color_data *d)
{
    if (!color_i2c_dev || !device_is_ready(color_i2c_dev)) {
        if (!color_last_error) {
            printk("Color: ERROR (I2C no inicializado)\n");
        }
        color_last_error = true;
        return -1;
    }

    uint16_t clear, red, green, blue;

    /* Cualquier fallo de I2C_write_read se interpreta como posible
     * desconexión/apagado del módulo.
     */
    if (tcs34725_read_u16(TCS34725_REG_CDATAL,     &clear) != 0 ||
        tcs34725_read_u16(TCS34725_REG_CDATAL + 2, &red)   != 0 ||
        tcs34725_read_u16(TCS34725_REG_CDATAL + 4, &green) != 0 ||
        tcs34725_read_u16(TCS34725_REG_CDATAL + 6, &blue)  != 0)
    {
        if (!color_last_error) {
            printk("Color: ERROR (lectura I2C)\n");
        }
        color_last_error = true;
        return -1;
    }

    /* Heurística: si todo es 0, puede indicar:
     *  - Módulo sin alimentación
     *  - Reset reciente
     *  - Error interno
     * En una planta real rara vez tendrás C=R=G=B=0 exactamente.
     */
    if (clear == 0 && red == 0 && green == 0 && blue == 0) {
        if (!color_last_error) {
            printk("Color: ERROR (valores todos a 0)\n");
        }
        color_last_error = true;
        return -1;
    }

    color_last_error = false;

    d->clear = clear;
    d->red   = red;
    d->green = green;
    d->blue  = blue;

    return 0;
}


/* ============================================================
 *  color_sensor_get_dominant()
 *  ------------------------------------------------------------
 *  Devuelve un color dominante simple (rojo, verde o azul) en
 *  función de los valores medidos. Si la componente "clear"
 *  es muy baja, se considera que no hay luz suficiente.
 *
 *  Nota: esto es un criterio muy simple, suficiente para [SR5]
 *  si solo necesitas etiquetar la hoja como "más roja", "más
 *  verde", etc.
 * ============================================================ */
enum dominant_color color_sensor_get_dominant(const struct color_data *d)
{
    /* Si la componente clara es muy baja, no nos fiamos del color */
    if (d->clear < TCS34725_CLEAR_MIN_THRESHOLD) {
        return COLOR_UNKNOWN;
    }

    if (d->red > d->green && d->red > d->blue) {
        return COLOR_RED;
    }
    if (d->green > d->red && d->green > d->blue) {
        return COLOR_GREEN;
    }
    if (d->blue > d->red && d->blue > d->green) {
        return COLOR_BLUE;
    }

    return COLOR_UNKNOWN;
}




// //88888888888888888888888888888888888888888888888888888888888888888888888888

// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/i2c.h>
// #include <zephyr/sys/printk.h>
// #include <stdlib.h>
// #include "color_sensor.h"

// #define ADDR 0x29
// #define CMD  0x80

// #define ENABLE 0x00
// #define ATIME  0x01
// #define CONTROL 0x0F
// #define CDATAL 0x14

// #define PON 0x01
// #define AEN 0x02

// #define I2C_NODE DT_NODELABEL(i2c1)

// static const struct device *i2c_dev;
// static bool last_error = false;
// static bool needs_reinit = false;

// static int write_reg(uint8_t r, uint8_t v)
// {
//     uint8_t buf[2] = { CMD | r, v };
//     return i2c_write(i2c_dev, buf, 2, ADDR);
// }

// static int read16(uint8_t r, uint16_t *out)
// {
//     uint8_t reg = CMD | r;
//     uint8_t buf[2];

//     if (i2c_write_read(i2c_dev, ADDR, &reg, 1, buf, 2) != 0)
//         return -1;

//     *out = buf[1] << 8 | buf[0];
//     return 0;
// }

// static void reinit_if_needed(void)
// {
//     if (!needs_reinit) return;

//     write_reg(ENABLE, PON);
//     k_msleep(3);
//     write_reg(ENABLE, PON | AEN);
//     write_reg(ATIME, 0xF6);
//     write_reg(CONTROL, 0x01);

//     needs_reinit = false;
// }

// void color_sensor_init(void)
// {
//     i2c_dev = DEVICE_DT_GET(I2C_NODE);

//     if (!device_is_ready(i2c_dev)) {
//         printk("Color: ERROR\n");
//         last_error = true;
//         return;
//     }

//     write_reg(ENABLE, PON);
//     k_msleep(3);
//     write_reg(ENABLE, PON | AEN);
//     write_reg(ATIME, 0xF6);
//     write_reg(CONTROL, 0x01);

//     last_error = false;
// }

// int color_sensor_read(struct color_data *d)
// {
//     reinit_if_needed();

//     uint16_t c, r, g, b;

//     if (read16(CDATAL, &c) < 0 ||
//         read16(CDATAL+2, &r) < 0 ||
//         read16(CDATAL+4, &g) < 0 ||
//         read16(CDATAL+6, &b) < 0)
//     {
//         if (!last_error) printk("Color: ERROR\n");
//         last_error = true;
//         needs_reinit = true;
//         return -1;
//     }

//     /* Si està desconnectat → 0 */
//     if (c == 0 && r == 0 && g == 0 && b == 0) {
//         if (!last_error) printk("Color: ERROR\n");
//         last_error = true;
//         needs_reinit = true;
//         return -1;
//     }

//     last_error = false;

//     d->clear = c;
//     d->red = r;
//     d->green = g;
//     d->blue = b;

//     return 0;
// }

// enum dominant_color color_sensor_get_dominant(const struct color_data *d)
// {
//     if (d->clear < 50) return COLOR_UNKNOWN;

//     if (d->red > d->green && d->red > d->blue) return COLOR_RED;
//     if (d->green > d->red && d->green > d->blue) return COLOR_GREEN;
//     if (d->blue > d->red && d->blue > d->green) return COLOR_BLUE;

//     return COLOR_UNKNOWN;
// }
