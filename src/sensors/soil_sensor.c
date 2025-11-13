#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#include "soil_sensor.h"

#define ADC_RESOLUTION 12
#define ADC_CHANNEL 3
#define SOIL_ADC_NODE DT_NODELABEL(adc1)
#define ADC_IN      DEVICE_DT_GET(DT_ALIAS(adc_in))
#define SOIL_MOISTURE_NODE   DT_ALIAS(soil_moisture_sensor)

// static const struct adc_dt_spec adc_light = ADC_DT_SPEC_GET(LIGHT_NODE);

//static const struct adc_dt_spec adc_moisture = ADC_DT_SPEC_GET(SOIL_MOISTURE_NODE);
static const struct device *adc_in;
static uint16_t sample_buffer;

static struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = ADC_CHANNEL,
};


void soil_sensor_init(void)
{
    adc_in = ADC_IN;
    if (!adc_in) {
        printk("Soil ADC not found!\n");
        return;
    }

    if (adc_channel_setup(adc_in, &channel_cfg) != 0) {
        printk("Soil ADC channel setup failed!\n");
        return;
    }

    printk("Soil moisture ADC Ready\n");
}

uint16_t soil_sensor_read(void)
{
    struct adc_sequence seq = {
        .channels = BIT(ADC_CHANNEL),
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = ADC_RESOLUTION,
    };

    if (adc_read(adc_in, &seq) != 0) {
        printk("Soil ADC read failed\n");
        return 0;
    }

    /* Extract the 12-bit raw value from the 16-bit buffer.
     * The ADC driver left-aligns the value, so shift right by 4 bits
     * (16 - 12 = 4) to get the actual 12-bit raw value (0-4095).
     */
    uint16_t raw_12bit = sample_buffer >> 4;
    
    return raw_12bit;
}
