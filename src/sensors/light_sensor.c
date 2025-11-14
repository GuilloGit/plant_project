#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#include "light_sensor.h"

/* ============================================================
 *  ADC configuration and constants
 * ============================================================ */

#define LIGHT_ADC_NODE          DT_NODELABEL(adc1)
#define LIGHT_ADC_CHANNEL       4
#define ADC_RESOLUTION          12

#define ADC_MAX_CODE            ((1 << ADC_RESOLUTION) - 1)

/* Margin to detect values stuck at 0 or at Vref */
#define ADC_RAIL_GUARD          5

/* Scale from 0 to 1000 (0–100%) with 0.1% resolution */
#define LIGHT_SCALE_X1000    1000

/* Calibration: typical codes in darkness and bright light */
#define LIGHT_LEVEL_DARK_CODE   4000   // low light → high code
#define LIGHT_LEVEL_BRIGHT_CODE  170   // bright light → low code

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
 *  Initializes the ADC and the light sensor channel.
 * ============================================================ */
void light_sensor_init(void)
{
    adc_dev = DEVICE_DT_GET(LIGHT_ADC_NODE);

    if (!device_is_ready(adc_dev)) {
        printk("Light: ERROR (ADC not available)\n");
        error_already_reported = true;
        return;
    }

    if (adc_channel_setup(adc_dev, &light_cfg) != 0) {
        printk("Light: ERROR (channel config)\n");
        error_already_reported = true;
        return;
    }
}


/* ============================================================
 *  light_sensor_read()
 *  Reads the ADC once, checks it's not saturated, then converts
 *  the value to a 0–1000 scale (0–100% light).
 *
 *  Return:
 *    >= 0 : light level in permille (0–1000)
 *    -1   : read error or invalid value (short / disconnect)
 * ============================================================ */
int light_sensor_read(void)
{
    if (!adc_dev || !device_is_ready(adc_dev)) {
        if (!error_already_reported) {
            printk("Light: ERROR (ADC not initialized)\n");
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

    /* ADC reading */
    if (adc_read(adc_dev, &seq) != 0) {
        if (!error_already_reported) {
            printk("Light: ERROR (ADC read)\n");
        }
        error_already_reported = true;
        return -1;
    }

    uint16_t sample = adc_raw_sample;

    /* Rail check: possible short circuit or disconnected sensor */
    if (sample <= ADC_RAIL_GUARD || sample >= (ADC_MAX_CODE - ADC_RAIL_GUARD)) {
        if (!error_already_reported) {
            printk("Light: ERROR (rail value)\n");
        }
        error_already_reported = true;
        return -1;
    }

    /* Map [DARK_CODE..BRIGHT_CODE] → [0..1000] */
    int32_t denominator = LIGHT_LEVEL_DARK_CODE - LIGHT_LEVEL_BRIGHT_CODE; // > 0 por diseño
    int32_t numerator   = (LIGHT_LEVEL_DARK_CODE - sample) * LIGHT_SCALE_X1000;
    int32_t light_level_percent = numerator / denominator;

    /* Clamp to range 0–1000 in case of slight overshoot */
    if (light_level_percent < 0) {
        light_level_percent = 0;
    } else if (light_level_percent > LIGHT_SCALE_X1000) {
        light_level_percent = LIGHT_SCALE_X1000;
    }

    error_already_reported = false;
    return light_level_percent;
}
