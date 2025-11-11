#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#include "soil_sensor.h"

#define ADC_RESOLUTION 12
#define ADC_CHANNEL 3
#define SOIL_ADC_NODE DT_NODELABEL(adc1)



#define ADC_IN      DEVICE_DT_GET(DT_ALIAS(adc_in))
#define LIGHT_NODE   DT_ALIAS(lightsensor)
#define SOIL_NODE   DT_ALIAS(soilsensor)

static const struct adc_dt_spec adc_light = ADC_DT_SPEC_GET(LIGHT_NODE);
static const struct adc_dt_spec adc_moisture = ADC_DT_SPEC_GET(SOIL_NODE);

static const struct device *adc_dev;
static int16_t sample_buffer;

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

 //   if (adc_channel_setup(adc_in, &channel_cfg) != 0) {
 //       printk("Soil ADC channel setup failed!\n");}
    if(adc_channel_setup_dt(&adc_moisture) != 0) {
        printk("Soil moisture ADC channel setup failed!\n");

    } else {
        printk("Soil moisture ADC Ready\n");
    }
}

int soil_sensor_read(void)
{
    struct adc_sequence seq = {
        .channels = BIT(adc_moisture.channel_id),
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = ADC_RESOLUTION,
    };

    if (adc_read(adc_moisture.dev, &seq) != 0) {
        printk("Soil ADC read failed\n");
        return -1;
    }

    return sample_buffer;
}
