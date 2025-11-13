#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

/* Project sensors */
#include "sensors/soil_sensor.h"
#include "sensors/light_sensor.h"
#include "sensors/color_sensor.h"
#include "sensors/gps.h"
#include "main.h"


volatile system_state_t current_state = NORMAL_STATE; // it has to be volatile because it's changed in an ISR
/* --- GPIO Device Bindings --- */


static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(DT_ALIAS(led_red), gpios);
static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(DT_ALIAS(led_green), gpios);
static const struct gpio_dt_spec led_b = GPIO_DT_SPEC_GET(DT_ALIAS(led_blue), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
void set_led_color(int r, int g, int b)
{
    gpio_pin_set_dt(&led_r, r); // Set Red LED
    gpio_pin_set_dt(&led_g, g); // Set Green LED
    gpio_pin_set_dt(&led_b, b); // Set Blue LED
}






void main(void)
{
    printk("\n=========================================\n");
    printk("Smart Plant Project - Main System   \n");
    printk("=========================================\n\n");

    /* ---------- INITIALIZATION ---------- */

    // printk("Initializing GPS...\n");
    // gps_init();
   


    printk("Initializing Soil Moisture Sensor...\n");
    soil_sensor_init();

     printk("Initializing Light Sensor (ADC)...\n");
     light_sensor_init();

    // printk("Initializing Color Sensor (I2C)...\n");
    // color_sensor_init();

    // printk("\nAll sensors initialized!\n\n");

    printk("Setting up LEDs and Button...\n");
    // Initialize LEDs
    int ret;
    if (!gpio_is_ready_dt(&led_r)) {
        return;
    }
    ret = gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return;
    }
    gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE); // TODO: add error checking
    gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE); // TODO: add error checking
    // Probando los leds
    set_led_color(1, 0, 0); // Red
    k_sleep(K_MSEC(500));
    set_led_color(0, 1, 0); // Green
    k_sleep(K_MSEC(500));
    set_led_color(0, 0, 1); // Blue







    /* Data structs */
    struct color_data col;


    /* ---------- MAIN LOOP ---------- */
    while (1)
    {
        /* ----- SOIL MOISTURE ----- */
        uint16_t soil_raw = soil_sensor_read();
        printk("Soil moisture RAW = %u\n", soil_raw);

        /* ----- LIGHT SENSOR ----- */
        uint16_t light_raw = light_sensor_read();
        printk("Light RAW = %u\n", light_raw);

        // /* ----- COLOR SENSOR (I2C) ----- */
        // if (color_sensor_read(&col) == 0) {
        //     printk("Color -> C:%u  R:%u  G:%u  B:%u\n",
        //            col.clear, col.red, col.green, col.blue);
        // } else {
        //     printk("Color sensor read error\n");
        // }

        // /* GPS prints appear automatically through ISR */

        printk("-----------------------------------------\n");

        k_sleep(K_SECONDS(2));
    }
}
