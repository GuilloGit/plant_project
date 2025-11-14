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

static volatile bool gga_processed = false;   
static volatile bool got_gpgga = false;       
static bool last_error = false;

void gps_reset_flag(void)
{
    gga_processed = false;
    got_gpgga = false;
}

static float nmea_to_degrees(const char *nmea, char dir)
{
    if (!nmea || strlen(nmea) < 4)
        return 0.0f;

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
    char *fields[20] = {0};

    while (*p && field < 20) {
        if (*p == ',') {
            *p = '\0';
            fields[field++] = line;
            line = p + 1;
        }
        p++;
    }

    char *utc   = fields[1];
    char *lat_s = fields[2];
    char *lat_d = fields[3];
    char *lon_s = fields[4];
    char *lon_d = fields[5];
    char *fixq  = fields[6];
    char *sats  = fields[7];
    char *alt   = fields[9];

    if (!fixq || atoi(fixq) == 0) {
        printk("GPS: Searching satellites...\n");
        return;
    }

    float lat = nmea_to_degrees(lat_s, lat_d[0]);
    float lon = nmea_to_degrees(lon_s, lon_d[0]);

    char time_fmt[16] = "??:??:??";
    if (utc && strlen(utc) >= 6) {
        snprintf(time_fmt, sizeof(time_fmt),
                 "%c%c:%c%c:%c%c",
                 utc[0], utc[1], utc[2], utc[3], utc[4], utc[5]);
    }

    printk("GPS → Lat: %.5f  Lon: %.5f  Alt: %s m  Sats: %s  Time: %s\n",
           lat, lon, alt, sats, time_fmt);
}

static void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {

        if (uart_fifo_read(dev, &c, 1) == 1) {

            if (c == '$')
                line_pos = 0;

            if (line_pos < BUF_SIZE - 1) {
                nmea_line[line_pos++] = c;

                if (c == '\n') {
                    nmea_line[line_pos] = '\0';

                    /* Acceptem GPGGA o GNGGA però només una vegada per cicle */
                    if (!gga_processed &&
                       (strstr(nmea_line, "$GPGGA") ||
                        strstr(nmea_line, "$GNGGA")))
                    {
                        gps_process_line(nmea_line);
                        gga_processed = true;
                        got_gpgga = true;
                    }

                    line_pos = 0;
                }
            }
        }
    }
}

void gps_init(void)
{
    uart_dev = DEVICE_DT_GET(UART1_NODE);

    if (!device_is_ready(uart_dev)) {
        printk("GPS: ERROR (UART not ready)\n");
        last_error = true;
        return;
    }

    uart_irq_callback_set(uart_dev, uart_isr);
    uart_irq_rx_enable(uart_dev);

    last_error = false;
    printk("GPS UART Ready\n");
}

int gps_read(void)
{
    if (!got_gpgga) {
        if (!last_error) printk("GPS: ERROR (no data)\n");
        last_error = true;
        return -1;
    }

    last_error = false;
    return 0;
}
