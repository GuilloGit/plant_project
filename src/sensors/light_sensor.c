#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#include "light_sensor.h"

/* ============================================================
 *  Configuración del ADC y constantes
 * ============================================================ */

#define LIGHT_ADC_NODE          DT_NODELABEL(adc1)
#define LIGHT_ADC_CHANNEL       4
#define ADC_RESOLUTION          12

#define ADC_MAX_CODE            ((1 << ADC_RESOLUTION) - 1)

/* Margen para detectar valores pegados a 0 o a Vref */
#define ADC_RAIL_GUARD          5

/* Escala del 0 al 1000 (0–100%) con resolucion de 0.1%*/
#define LIGHT_SCALE_X1000    1000

/* Calibración: valores típicos en oscuridad y mucha luz */
#define LIGHT_LEVEL_DARK_CODE   4000   // poca luz → código alto
#define LIGHT_LEVEL_BRIGHT_CODE  170   // mucha luz → código bajo

static const struct device *adc_dev;
/* use unsigned 16-bit buffer for ADC samples (driver writes into 16-bit cells)
 */
static uint16_t adc_raw_sample;
static bool error_already_reported = false;

static struct adc_channel_cfg light_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = LIGHT_ADC_CHANNEL,
};


/* ============================================================
 *  light_sensor_init()
 *  Inicializa el ADC y el canal del sensor de luz.
 * ============================================================ */
void light_sensor_init(void)
{
    adc_dev = DEVICE_DT_GET(LIGHT_ADC_NODE);

    if (!device_is_ready(adc_dev)) {
        printk("Light: ERROR (ADC no disponible)\n");
        error_already_reported = true;
        return;
    }

    if (adc_channel_setup(adc_dev, &light_cfg) != 0) {
        printk("Light: ERROR (config canal)\n");
        error_already_reported = true;
        return;
    }
}


/* ============================================================
 *  light_sensor_read()
 *  Lee el ADC una vez, comprueba que no esté saturado y convierte
 *  el valor a una escala 0–1000 (0–100% de luz).
 *
 *  Retorno:
 *    >= 0 : nivel de luz en permille (0–1000)
 *    -1   : error en lectura o valor inválido (corto / desconexión)
 * ============================================================ */
int light_sensor_read(void)
{
    if (!adc_dev || !device_is_ready(adc_dev)) {
        if (!error_already_reported) {
            printk("Light: ERROR (ADC no inicializado)\n");
        }
        error_already_reported = true;
        return -1;
    }

    struct adc_sequence seq = {
        .channels    = BIT(LIGHT_ADC_CHANNEL),
        .buffer      = &adc_raw_sample,
        .buffer_size = sizeof(adc_raw_sample),
        .resolution  = ADC_RESOLUTION,
    };

    /* lectura del ADC */
    if (adc_read(adc_dev, &seq) != 0) {
        if (!error_already_reported) {
            printk("Light: ERROR (lectura ADC)\n");
        }
        error_already_reported = true;
        return -1;
    }

    uint16_t sample = adc_raw_sample;

    /* Rail check: posible cortocircuito o sensor desconectado */
    if (sample <= ADC_RAIL_GUARD || sample >= (ADC_MAX_CODE - ADC_RAIL_GUARD)) {
        if (!error_already_reported) {
            printk("Light: ERROR (valor en riel)\n");
        }
        error_already_reported = true;
        return -1;
    }

    /* Mapear [DARK_CODE..BRIGHT_CODE] → [0..1000] */
    int32_t denominator = LIGHT_LEVEL_DARK_CODE - LIGHT_LEVEL_BRIGHT_CODE; // > 0 por diseño
    int32_t numerator   = (LIGHT_LEVEL_DARK_CODE - sample) * LIGHT_SCALE_X1000;
    int32_t light_level_percent = numerator / denominator;

    /* Limitar al rango 0–1000 por si se sale un poco */
    if (light_level_percent < 0) {
        light_level_percent = 0;
    } else if (light_level_percent > LIGHT_SCALE_X1000) {
        light_level_percent = LIGHT_SCALE_X1000;
    }

    error_already_reported = false;
    return light_level_percent;
}
