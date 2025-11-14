#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

/* Project sensors */
#include "sensors/soil_sensor.h"
#include "sensors/light_sensor.h"
#include "sensors/color_sensor.h"
#include "sensors/temp_sensor.h"
#include "sensors/gps.h"
#include "main.h"
#include "sensors/accelerometer.h"
#include "sensors/rgb_led.h"    


// Inicializacion de la variable de estado del sistema
volatile system_state_t current_state = NORMAL_STATE; // it has to be volatile because it's changed in an ISR


// Variables globales 
    struct accel_data accel;
    int ret;
// Rutinas

// Rutina de inicio del programa
void start_routine(void){
    /* ---------- INITIALIZATION ---------- */

    printk("Initializing GPS...\n");
    gps_init();
    printk("Initializing Soil Moisture Sensor...\n");
    soil_sensor_init();
     printk("Initializing Light Sensor (ADC)...\n");
     light_sensor_init();
    printk("Initializing Color Sensor (I2C)...\n");
    color_sensor_init();
    printk("Initializing Temperature & Humidity Sensor (Si7021)...\n");
    temp_sensor_init();
    printk("Setting up LEDs\n");
    rgb_led_init();
    printk("Initializing MMA8451 Accelerometer...\n");
    accelerometer_init();
    printk("\nAll sensors initialized!\n\n");
    printk("-----------------------------------------\n");

}
void diag_routine(void){
    // Rutina principal de prueba

    /* ----- SOIL MOISTURE ----- */
    uint16_t soil_raw = soil_sensor_read();
    float soil_percent = soil_raw / 10.0f;  // Convert permille to percentage
    printk("Soil moisture = %.1f%%\n", (double)soil_percent);

    /* ----- LIGHT SENSOR ----- */
    uint16_t light_raw = light_sensor_read();
    float light_percent = (1000 - light_raw) / 10.0f;  // Invert and convert to percentage
    printk("Light = %.1f%%\n", (double)light_percent);

    /* ----- COLOR SENSOR (I2C) ----- */
    struct color_data col;
    if (color_sensor_read(&col) == 0) {
        printk("Color -> C:%u  R:%u  G:%u  B:%u\n",
               col.clear, col.red, col.green, col.blue);
        
        enum dominant_color dom_color = color_sensor_get_dominant(&col);
        const char *color_names[] = {"UNKNOWN", "RED", "GREEN", "BLUE"};
        printk("Dominant Color: %s\n", color_names[dom_color]);
        
        // Set RGB LED according to dominant color
        switch (dom_color) {
            case COLOR_RED:
                set_led_color(1, 0, 0); // Red
                break;
            case COLOR_GREEN:
                set_led_color(0, 1, 0); // Green
                break;
            case COLOR_BLUE:
                set_led_color(0, 0, 1); // Blue
                break;
            case COLOR_UNKNOWN:
            default:
                set_led_color(0, 0, 0); // Off
                break;
        }
        
    } else {
        printk("Color sensor read error\n"
                "Debug values: Color -> C:%u  R:%u  G:%u  B:%u\n",
               col.clear, col.red, col.green, col.blue);
        set_led_color(0, 0, 0); // Off on error
    }

    /* ----- TEMPERATURE & HUMIDITY SENSOR (Si7021) ----- */
    struct temp_hum_data temp_hum;
    if (temp_sensor_read(&temp_hum) == 0) {
        printk("Temperature: %.1f°C, Humidity: %.1f%%RH\n",
               (double)temp_hum.temperature_c, (double)temp_hum.humidity_rh);
    } else {
        printk("Temperature sensor read error\n");
    }

    /* ----- GPS ----- */
    struct gps_data gps;
    ret = gps_read(&gps);
    if (ret == 0) {
        printk("GPS -> Lat: %.5f, Lon: %.5f, Alt: %.1fm, Sats: %u, Time: %s UTC\n",
               (double)gps.latitude, (double)gps.longitude, (double)gps.altitude,
               gps.satellites, gps.time_utc);
    } else if (ret == -2) {
        printk("GPS: ERROR (UART disconnected or no data)\n");
    } else {
        printk("GPS: Time: %s UTC\n", gps.time_utc);
        printk("GPS: No fix (waiting for satellites...)\n");
    }


    /* ----- ACCELEROMETER TEST ----- */
    printk("Testing Accelerometer...\n");
    ret = accelerometer_read(&accel);

    if (ret == 0) {
        /* Formato: 
            * ACCELEROMETERS: X_axis: -0.23 m/s², Y_axis: 1.20 m/s², Z_axis: 9.88 m/s²
            * Cast to double to silence -Wdouble-promotion warning
            */
        printk("ACCELEROMETERS: X_axis: %.2f m/s², Y_axis: %.2f m/s², Z_axis: %.2f m/s²\n",
                (double)accel.x_ms2, (double)accel.y_ms2, (double)accel.z_ms2);
    } else {
        printk("ACCELEROMETERS: ERROR (could not read the sensor)\n");
    }
    // /** ----- RGB LED TEST ----- */
    // printk("Testing RGB LED...\n");
    // set_led_color(1, 0, 0); // Red
    // k_sleep(K_MSEC(500));
    // set_led_color(0, 1, 0); // Green
    // k_sleep(K_MSEC(500));
    // set_led_color(0, 0, 1); // Blue
    // k_sleep(K_MSEC(500));
    
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
        diag_routine();

        k_sleep(K_SECONDS(2));
    }
}
