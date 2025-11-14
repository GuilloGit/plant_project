#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

/* Project sensors */
#include "sensors/soil_sensor.h"
#include "sensors/light_sensor.h"
#include "sensors/color_sensor.h"
#include "sensors/gps.h"
#include "main.h"
#include "sensors/accelerometer.h"
#include "sensors/rgb_led.h"    


// Inicializacion de la variable de estado del sistema
volatile system_state_t current_state = NORMAL_STATE; // it has to be volatile because it's changed in an ISR

// Rutinas

// Rutina de inicio del programa
void start_routine(void){
    /* ---------- INITIALIZATION ---------- */

    // printk("Initializing GPS...\n");
    // gps_init();
    printk("Initializing Soil Moisture Sensor...\n");
    soil_sensor_init();
     printk("Initializing Light Sensor (ADC)...\n");
     light_sensor_init();
    printk("Initializing Color Sensor (I2C)...\n");
    color_sensor_init();
    printk("Setting up LEDs\n");
    rgb_led_init();
    printk("\nAll sensors initialized!\n\n");

}
void test_routine(void){
    // Rutina principal de prueba

    /* ----- SOIL MOISTURE ----- */
    uint16_t soil_raw = soil_sensor_read();
    printk("Soil moisture RAW = %u\n", soil_raw);

    /* ----- LIGHT SENSOR ----- */
    uint16_t light_raw = light_sensor_read();
    printk("Light RAW = %u\n", light_raw);

    /* ----- COLOR SENSOR (I2C) ----- */
    struct color_data col;
    if (color_sensor_read(&col) == 0) {
        printk("Color -> C:%u  R:%u  G:%u  B:%u\n",
               col.clear, col.red, col.green, col.blue);
        
        enum dominant_color dom_color = color_sensor_get_dominant(&col);
        const char *color_names[] = {"UNKNOWN", "RED", "GREEN", "BLUE"};
        printk("Dominant Color: %s\n", color_names[dom_color]);
    } else {
        printk("Color sensor read error\n"
                "Debug values: Color -> C:%u  R:%u  G:%u  B:%u\n",
               col.clear, col.red, col.green, col.blue);
    }

    // /* GPS prints appear automatically through ISR */

    /** ----- RGB LED TEST ----- */
    printk("Testing RGB LED...\n");
    set_led_color(1, 0, 0); // Red
    k_sleep(K_MSEC(500));
    set_led_color(0, 1, 0); // Green
    k_sleep(K_MSEC(500));
    set_led_color(0, 0, 1); // Blue
    k_sleep(K_MSEC(500));

    
    printk("-----------------------------------------\n");

}



void main(void)
{
    printk("\n=========================================\n");
    printk("Smart Plant Project - Main System   \n");
    printk("=========================================\n\n");
    start_routine();
    /* ---------- MAIN LOOP ---------- */
    while (1)
    {
        test_routine();

        k_sleep(K_SECONDS(2));
    }
}
