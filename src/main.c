#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* Project sensors */
#include "sensors/soil_sensor.h"
#include "sensors/light_sensor.h"
#include "sensors/color_sensor.h"
#include "sensors/gps.h"

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

    // printk("Initializing Light Sensor (ADC)...\n");
    // light_sensor_init();

    // printk("Initializing Color Sensor (I2C)...\n");
    // color_sensor_init();

    // printk("\nAll sensors initialized!\n\n");

    /* Data structs */
    struct color_data col;

    /* ---------- MAIN LOOP ---------- */
    while (1)
    {
        /* ----- SOIL MOISTURE ----- */
        uint16_t soil_raw = soil_sensor_read();
        printk("Soil moisture RAW = %u\n", soil_raw);

        // /* ----- LIGHT SENSOR ----- */
        // int light_raw = light_sensor_read();
        // if (light_raw >= 0) {
        //     printk("Light RAW = %d\n", light_raw);
        // } else {
        //     printk("Light sensor read error\n");
        // }

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
