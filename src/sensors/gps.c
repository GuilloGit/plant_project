#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdlib.h>

#include "gps.h"

#define UART1_NODE DT_NODELABEL(usart1)
#define BUF_SIZE 128

static const struct device *uart_dev;
static char nmea_line[BUF_SIZE];
static uint8_t line_pos = 0;

/* --------- Helpers ---------- */

static float nmea_to_degrees(const char *nmea, char dir)
{
    if (!nmea || strlen(nmea) < 4) return 0.0f;

    float value = atof(nmea);
    int degrees = (int)(value / 100);
    float minutes = value - (degrees * 100);

    float result = degrees + (minutes / 60.0f);
    return (dir == 'S' || dir == 'W') ? -result : result;
}

void gps_process_line(char *line)
{
    char *p = line;
    int field = 0;
    char *fields[15] = {0};

    while (*p && field < 15) {
        if (*p == ',') {
            *p = '\0';
            fields[field++] = line;
            line = p + 1;
        }
        p++;
    }

    if (fields[2] && fields[3] && fields[4] && fields[5] && fields[9]) {
        float lat = nmea_to_degrees(fields[2], fields[3][0]);
        float lon = nmea_to_degrees(fields[4], fields[5][0]);

        printk("GPS: %.6f%c, %.6f%c | Alt: %s m | Sats: %s\n",
               lat, fields[3][0],
               lon, fields[5][0],
               fields[9], fields[7]);
    }
}

/* -------- UART IRQ -------- */

static void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &c, 1) == 1) {

            if (c == '$') line_pos = 0;

            if (line_pos < BUF_SIZE - 1) {
                nmea_line[line_pos++] = c;

                if (c == '\n') {
                    nmea_line[line_pos] = '\0';

                    if (strstr(nmea_line, "$GPGGA") || strstr(nmea_line, "$GNGGA"))
                        gps_process_line(nmea_line);

                    line_pos = 0;
                }
            }
        }
    }
}

/* -------- PUBLIC API -------- */

void gps_init(void)
{
    uart_dev = DEVICE_DT_GET(UART1_NODE);
    if (!device_is_ready(uart_dev)) {
        printk("GPS UART not ready\n");
        return;
    }

    uart_irq_callback_set(uart_dev, uart_isr);
    uart_irq_rx_enable(uart_dev);

    printk("GPS UART Ready\n");
}
