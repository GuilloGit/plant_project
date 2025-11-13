#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include "light_sensor.h"

/* -------- Macros de configuracion de DT y  Canal -------- */

#define LIGHT_ADC_NODE      DT_NODELABEL(adc1) // Periferico ADC1 
#define ADC_IN      DEVICE_DT_GET(DT_ALIAS(adc_in))

#define LIGHT_ADC_CHANNEL   4    // PB2 = ADC1_IN4 Canal 4
#define ADC_RESOLUTION      12 // Resolucion de 12 bits 

static const struct device *adc_in; // Puntero al struct del ADC "device"
static uint16_t light_sample; // Buffer para almacenar la muestra del ADC

static struct adc_channel_cfg light_channel_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = LIGHT_ADC_CHANNEL,
};

void light_sensor_init(void)
{
    adc_in = ADC_IN;
    if (!device_is_ready(adc_in)) {
        printk("Light ADC device not ready\n");
        return;
    }

    if (adc_channel_setup(adc_in, &light_channel_cfg) != 0) {
        printk("Light ADC channel setup failed\n");
        return;
    }

    printk("Light sensor ADC ready (channel %d)\n", LIGHT_ADC_CHANNEL);
}

uint16_t light_sensor_read(void)
{
    struct adc_sequence seq = {
        .channels = BIT(LIGHT_ADC_CHANNEL),
        .buffer = &light_sample,
        .buffer_size = sizeof(light_sample),
        .resolution = ADC_RESOLUTION,
    };

    if (adc_read(adc_in, &seq) != 0) {
        printk("Light ADC read failed\n");
        return -1;
    }
    uint16_t raw_12bit = light_sample >> 4; // Ajustar a 12 bits
    return raw_12bit;
}
