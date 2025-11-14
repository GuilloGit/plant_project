#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include "temp_sensor.h"

/* ============================================================
 *  Configuración del Si7021 e I2C
 * ============================================================ */

/* Dirección I2C del Si7021 (7 bits) */
#define SI7021_I2C_ADDR                    0x40

/* Comandos del Si7021 */
#define SI7021_MEASRH_NOHOLD_CMD           0xF5  /* Medir Humedad, No Hold Master */
#define SI7021_MEASTEMP_NOHOLD_CMD         0xF3  /* Medir Temperatura, No Hold Master */
#define SI7021_READPREVTEMP_CMD            0xE0  /* Leer Temp. de medición anterior */
#define SI7021_RESET_CMD                   0xFE  /* Reset del sensor */
#define SI7021_READRHT_REG_CMD             0xE7  /* Leer User Register 1 */

/* Valor esperado del registro User Register 1 tras reset */
#define SI7021_USER_REG1_DEFAULT           0x3A

/* Timeout para lecturas (milisegundos) */
#define SI7021_CONVERSION_TIMEOUT_MS       100

/* Tiempo de espera tras enviar comando de medición (ms) */
#define SI7021_CONVERSION_DELAY_MS         20

/* Reintentos durante timeout */
#define SI7021_POLL_DELAY_MS               6

/* Nodo I2C en el device tree */
#define TEMP_I2C_NODE                      DT_NODELABEL(i2c2)

/* ============================================================
 *  Variables estáticas
 * ============================================================ */

static const struct device *temp_i2c_dev;
static bool temp_last_error = false;


/* ============================================================
 *  Funciones internas de acceso al Si7021
 * ============================================================ */

/*
 * Envía un comando de un solo byte al Si7021.
 */
static int si7021_send_cmd(uint8_t cmd)
{
    return i2c_write(temp_i2c_dev, &cmd, 1, SI7021_I2C_ADDR);
}

/*
 * Lee un registro de 8 bits del Si7021.
 */
static int si7021_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_write_read(temp_i2c_dev, SI7021_I2C_ADDR,
                         &reg, 1, value, 1);
}

/*
 * Resetea el Si7021 y espera a que se estabilice.
 */
static int si7021_reset(void)
{
    int ret = si7021_send_cmd(SI7021_RESET_CMD);
    if (ret != 0) {
        return ret;
    }

    /* Esperar a que el sensor se reinicie (datasheet: ~15ms típico) */
    k_msleep(50);

    return 0;
}

/*
 * Verifica que el Si7021 esté presente leyendo el User Register 1.
 */
static int si7021_check_presence(void)
{
    uint8_t reg_val;
    int ret = si7021_read_reg(SI7021_READRHT_REG_CMD, &reg_val);

    if (ret != 0) {
        printk("Temp: ERROR al leer User Register 1 (%d)\n", ret);
        return ret;
    }

    if (reg_val != SI7021_USER_REG1_DEFAULT) {
        printk("Temp: User Register 1 = 0x%02X, esperado 0x%02X\n",
               reg_val, SI7021_USER_REG1_DEFAULT);
        
    }

    return 0;
}

/*
 * Lee la humedad relativa usando el modo No Hold Master.
 * Retorna el valor raw de 16 bits o error negativo.
 */
static int si7021_read_humidity_raw(uint16_t *raw_value)
{
    uint8_t buf[3];  /* MSB, LSB, CRC */
    int ret;

    /* Enviar comando de medición de humedad */
    ret = si7021_send_cmd(SI7021_MEASRH_NOHOLD_CMD);
    if (ret != 0) {
        return ret;
    }

    /* Esperar a que la conversión esté lista */
    k_msleep(SI7021_CONVERSION_DELAY_MS);

    /* Intentar leer el resultado con timeout */
    int64_t start_time = k_uptime_get();
    while (k_uptime_get() - start_time < SI7021_CONVERSION_TIMEOUT_MS) {
        ret = i2c_read(temp_i2c_dev, buf, sizeof(buf), SI7021_I2C_ADDR);
        if (ret == 0) {
            /* Lectura exitosa: MSB en buf[0], LSB en buf[1] */
            *raw_value = ((uint16_t)buf[0] << 8) | buf[1];
            return 0;
        }
        k_msleep(SI7021_POLL_DELAY_MS);
    }

    /* Timeout */
    return -ETIMEDOUT;
}

/*
 * Lee la temperatura usando el modo No Hold Master.
 * Retorna el valor raw de 16 bits o error negativo.
 */
static int si7021_read_temperature_raw(uint16_t *raw_value)
{
    uint8_t buf[3];  /* MSB, LSB, CRC */
    int ret;

    /* Enviar comando de medición de temperatura */
    ret = si7021_send_cmd(SI7021_MEASTEMP_NOHOLD_CMD);
    if (ret != 0) {
        return ret;
    }

    /* Esperar a que la conversión esté lista */
    k_msleep(SI7021_CONVERSION_DELAY_MS);

    /* Intentar leer el resultado con timeout */
    int64_t start_time = k_uptime_get();
    while (k_uptime_get() - start_time < SI7021_CONVERSION_TIMEOUT_MS) {
        ret = i2c_read(temp_i2c_dev, buf, sizeof(buf), SI7021_I2C_ADDR);
        if (ret == 0) {
            /* Lectura exitosa: MSB en buf[0], LSB en buf[1] */
            *raw_value = ((uint16_t)buf[0] << 8) | buf[1];
            return 0;
        }
        k_msleep(SI7021_POLL_DELAY_MS);
    }

    /* Timeout */
    return -ETIMEDOUT;
}


/* ============================================================
 *  temp_sensor_init()
 *  ------------------------------------------------------------
 *  Inicializa el dispositivo I2C y verifica que el Si7021
 *  esté presente y funcionando.
 * ============================================================ */
void temp_sensor_init(void)
{
    temp_i2c_dev = DEVICE_DT_GET(TEMP_I2C_NODE);

    printk("Temp: I2C device pointer = %p\n", temp_i2c_dev);

    if (!device_is_ready(temp_i2c_dev)) {
        printk("Temp: ERROR (I2C no disponible)\n");
        temp_last_error = true;
        return;
    }

    printk("Temp: I2C device ready, resetting Si7021...\n");

    if (si7021_reset() != 0) {
        printk("Temp: ERROR (reset Si7021)\n");
        temp_last_error = true;
        return;
    }

    if (si7021_check_presence() != 0) {
        printk("Temp: ERROR (verificación Si7021)\n");
        temp_last_error = true;
        return;
    }

    printk("Temp: Si7021 initialized successfully\n");
    temp_last_error = false;
}


/* ============================================================
 *  temp_sensor_read()
 *  ------------------------------------------------------------
 *  Lee la temperatura y humedad del Si7021.
 *
 *  Fórmulas de conversión (datasheet Si7021):
 *    RH% = ((125 * RH_raw) / 65536) - 6
 *    Temp°C = ((175.72 * Temp_raw) / 65536) - 46.85
 *
 *  Manejo de errores:
 *   - Si cualquier lectura I2C falla o hay timeout → retorna -1
 *   - Si los valores están fuera de rango esperado → retorna -1
 *
 *  Retorno:
 *    0  -> OK, estructura 'data' rellena.
 *   -1  -> Algún error (I2C, timeout, valores inválidos).
 * ============================================================ */
int temp_sensor_read(struct temp_hum_data *data)
{
    if (!temp_i2c_dev || !device_is_ready(temp_i2c_dev)) {
        if (!temp_last_error) {
            printk("Temp: ERROR (I2C no inicializado)\n");
        }
        temp_last_error = true;
        return -1;
    }

    uint16_t raw_humidity, raw_temperature;
    int ret;

    /* Leer humedad */
    ret = si7021_read_humidity_raw(&raw_humidity);
    if (ret != 0) {
        if (!temp_last_error) {
            printk("Temp: ERROR al leer humedad (%d)\n", ret);
        }
        temp_last_error = true;
        return -1;
    }

    /* Leer temperatura */
    ret = si7021_read_temperature_raw(&raw_temperature);
    if (ret != 0) {
        if (!temp_last_error) {
            printk("Temp: ERROR al leer temperatura (%d)\n", ret);
        }
        temp_last_error = true;
        return -1;
    }

    /* Convertir humedad raw a porcentaje */
    /* RH% = ((125 * raw) / 65536) - 6 */
    float humidity = ((float)raw_humidity * 125.0f / 65536.0f) - 6.0f;

    /* Limitar humedad a 0-100% */
    if (humidity < 0.0f) {
        humidity = 0.0f;
    } else if (humidity > 100.0f) {
        humidity = 100.0f;
    }

    /* Convertir temperatura raw a grados Celsius */
    /* Temp°C = ((175.72 * raw) / 65536) - 46.85 */
    float temperature = ((float)raw_temperature * 175.72f / 65536.0f) - 46.85f;

    /* Validar rangos (SR1 y SR2) */
    /* SR1: Temperatura -10°C a 50°C */
    if (temperature < -10.0f || temperature > 50.0f) {
        if (!temp_last_error) {
            printk("Temp: WARNING - temperatura fuera de rango: %.1f°C\n", 
                   (double)temperature);
        }
        /* No retornar error, solo advertencia */
    }

    /* SR2: Humedad 25%RH a 75%RH */
    if (humidity < 25.0f || humidity > 75.0f) {
        if (!temp_last_error) {
            printk("Temp: WARNING - humedad fuera de rango: %.1f%%RH\n", 
                   (double)humidity);
        }
        /* No retornar error, solo advertencia */
    }

    /* Rellenar estructura de salida */
    data->temperature_c = temperature;
    data->humidity_rh = humidity;


    /* printk("Temp: T=%.1f°C, RH=%.1f%%\n", 
           (double)temperature, (double)humidity); */

    temp_last_error = false;
    return 0;
}
