#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include "light_sensor.h"

/* -------- CONFIG -------- */
#define LIGHT_ADC_NODE      DT_NODELABEL(adc1)
#define LIGHT_ADC_CHANNEL   4    // PB2 = ADC1_IN4
#define ADC_RESOLUTION      12

static const struct device *adc_dev;
static int16_t light_sample;

static struct adc_channel_cfg light_channel_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = LIGHT_ADC_CHANNEL,
};

void light_sensor_init(void)
{
    adc_dev = DEVICE_DT_GET(LIGHT_ADC_NODE);
    if (!device_is_ready(adc_dev)) {
        printk("Light ADC device not ready\n");
        return;
    }

    if (adc_channel_setup(adc_dev, &light_channel_cfg) != 0) {
        printk("Light ADC channel setup failed\n");
        return;
    }

    printk("Light sensor ADC ready (channel %d)\n", LIGHT_ADC_CHANNEL);
}

int light_sensor_read(void)
{
    struct adc_sequence seq = {
        .channels = BIT(LIGHT_ADC_CHANNEL),
        .buffer = &light_sample,
        .buffer_size = sizeof(light_sample),
        .resolution = ADC_RESOLUTION,
    };

    if (adc_read(adc_dev, &seq) != 0) {
        printk("Light ADC read failed\n");
        return -1;
    }

    return light_sample;
}
