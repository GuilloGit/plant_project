#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>


#include "rgb_led.h"




/* --- GPIO Device Bindings --- */
static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(DT_ALIAS(led_red), gpios);
static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(DT_ALIAS(led_green), gpios);
static const struct gpio_dt_spec led_b = GPIO_DT_SPEC_GET(DT_ALIAS(led_blue), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

// Funcion para poner el color del LED RGB
void set_led_color(int r, int g, int b)
{
    gpio_pin_set_dt(&led_r, r); // Set Red LED
    gpio_pin_set_dt(&led_g, g); // Set Green LED
    gpio_pin_set_dt(&led_b, b); // Set Blue LED
}
void rgb_led_init(void)
{
    // Initialize LEDs

    gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE); // TODO: add error checking
    gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE); // TODO: add error checking
    gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE); // TODO: add error checking
    
    // Probando los leds
    set_led_color(1, 0, 0); // Red
    k_sleep(K_MSEC(500));
    set_led_color(0, 1, 0); // Green
    k_sleep(K_MSEC(500));
    set_led_color(0, 0, 1); // Blue
}


