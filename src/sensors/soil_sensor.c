#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#include "soil_sensor.h"

/* ============================================================
 *  Configuración del ADC y constantes de calibración
 * ============================================================ */

#define SOIL_ADC_NODE            DT_NODELABEL(adc1)
#define SOIL_ADC_CHANNEL         3
#define SOIL_ADC_RESOLUTION      12

/* Valor máximo posible del ADC para 12 bits */
#define SOIL_ADC_MAX_CODE        ((1 << SOIL_ADC_RESOLUTION) - 1)

/* Margen para detectar valores pegados a 0 o a Vref (posible fallo) */
#define SOIL_ADC_RAIL_GUARD      5

/* Escala 0–1000 → 0.0% a 100.0% de humedad */
#define SCALEX1000   1000

/* Rango empírico medido en el sensor de humedad */
#define SOIL_DRY_CALIB            27    // Suelo seco ≈ 0% humedad
#define SOIL_WET_CALIB           3290   // Suelo muy húmedo ≈ 100% humedad

static const struct device *soil_adc_dev;
static uint16_t soil_adc_raw_sample;
static bool soil_error_already_reported = false;

static struct adc_channel_cfg soil_channel_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = SOIL_ADC_CHANNEL,
};


/* ============================================================
 *  soil_sensor_init()
 *  ------------------------------------------------------------
 *  Inicializa el ADC y configura el canal usado para el sensor
 *  de humedad de suelo.
 * ============================================================ */
void soil_sensor_init(void)
{
    soil_adc_dev = DEVICE_DT_GET(SOIL_ADC_NODE);

    if (!device_is_ready(soil_adc_dev)) {
        printk("Soil: ERROR (ADC no disponible)\n");
        soil_error_already_reported = true;
        return;
    }

    if (adc_channel_setup(soil_adc_dev, &soil_channel_cfg) != 0) {
        printk("Soil: ERROR (config canal ADC)\n");
        soil_error_already_reported = true;
        return;
    }

    soil_error_already_reported = false;
    printk("Soil: ADC listo\n");
}


/* ============================================================
 *  soil_sensor_read()
 *  ------------------------------------------------------------
 *  Lee el ADC una vez, comprueba que el valor no esté saturado
 *  (cerca de 0 o del máximo) y convierte el valor leído al rango
 *  0–1000, donde 0 = suelo seco y 1000 = suelo muy húmedo.
 *
 *  Retorno:
 *    >= 0 : humedad en porcentaje con 0.1% de resolución (0–1000 → 0–100.0%)
 *    -1   : error en lectura o valor inválido (corto / desconexión)
 * ============================================================ */
int soil_sensor_read(void)
{
    /* Evitar leer si el ADC no se inicializó correctamente */
    if (!soil_adc_dev || !device_is_ready(soil_adc_dev)) {
        if (!soil_error_already_reported) {
            printk("Soil: ERROR (ADC no inicializado)\n");
        }
        soil_error_already_reported = true;
        return -1;
    }

    struct adc_sequence seq = {
        .channels    = BIT(SOIL_ADC_CHANNEL),
        .buffer      = &soil_adc_raw_sample,
        .buffer_size = sizeof(soil_adc_raw_sample),
        .resolution  = SOIL_ADC_RESOLUTION,
    };

    /* Lectura única del ADC (Zephyr entrega el valor right-aligned) */
    if (adc_read(soil_adc_dev, &seq) != 0) {
        if (!soil_error_already_reported) {
            printk("Soil: ERROR (lectura ADC)\n");
        }
        soil_error_already_reported = true;
        return -1;
    }

    uint16_t raw = soil_adc_raw_sample;  // 0..4095 para 12 bits

    /* Comprobar si el valor está pegado a los rieles → posible fallo */
    if (raw <= SOIL_ADC_RAIL_GUARD || raw >= (SOIL_ADC_MAX_CODE - SOIL_ADC_RAIL_GUARD)) {
        if (!soil_error_already_reported) {
            printk("Soil: ERROR (valor ADC en riel)\n");
        }
        soil_error_already_reported = true;
        return -1;
    }

    /* Mapear [SOIL_DRY_CALIB..SOIL_WET_CALIB] → [0..1000] (0–100% humedad) */
    int32_t denominator = (int32_t)SOIL_WET_CALIB - (int32_t)SOIL_DRY_CALIB; // > 0 según tu calibración
    int32_t numerator   = ((int32_t)raw - (int32_t)SOIL_DRY_CALIB) * SCALEX1000;
    int32_t moisture_percent = numerator / denominator;

    /* Limitar al rango 0–1000 por seguridad */
    if (moisture_percent < 0) {
        moisture_percent = 0;
    } else if (moisture_percent > SCALEX1000) {
        moisture_percent = SCALEX1000;
    }

    soil_error_already_reported = false;
    return (int)moisture_percent;
}
