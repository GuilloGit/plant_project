#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>


#define UART1_NODE DT_NODELABEL(usart1)
#define BUF_SIZE 128

/* Thread function */
void my_thread_func(void *p1, void *p2, void *p3) {
while (1) {
printk("Hello from my_thread!\n");
k_sleep(K_SECONDS(1));
}
}
/* Define the thread at compile-time */
K_THREAD_DEFINE(my_thread_id, /* Thread identifier */
                1024, /* Stack size */
                my_thread_func, /* Entry function */
                NULL, NULL, NULL, /* Parameters */
                5, /* Priority */
                0, /* Options */
                0); /* Starts immediately */

static const struct device *uart_dev;
static char nmea_line[BUF_SIZE];
static uint8_t line_pos = 0;


// GPS

// Convert NMEA (DDMM.MMMM) to decimal degrees
static float nmea_to_degrees(const char *nmea, char dir)
{
    if (!nmea || strlen(nmea) < 4) return 0.0f;
    
    float value = 0.0f;
    int degrees = 0;
    float minutes = 0.0f;
    
    // Convert the string to a number
    for (int i = 0; nmea[i]; i++) {
        if (nmea[i] >= '0' && nmea[i] <= '9') {
            value = value * 10 + (nmea[i] - '0');
        } else if (nmea[i] == '.') {
            // Read the decimal part
            float decimal = 0.0f;
            float divisor = 10.0f;
            for (int j = i + 1; nmea[j] >= '0' && nmea[j] <= '9'; j++) {
                decimal += (nmea[j] - '0') / divisor;
                divisor *= 10.0f;
            }
            value += decimal;
            break;
        }
    }
    
    // Separate degrees and minutes
    degrees = (int)(value / 100);
    minutes = value - (degrees * 100);
    
    float result = degrees + (minutes / 60.0f);
    
    // Make negative if South or West
    if (dir == 'S' || dir == 'W') {
        result = -result;
    }
    
    return result;
}
// Simple function to display key GPS information
static void print_gps_info(char *line)
{
    char *p = line;
    int field = 0;
    char *fields[15] = {0};
    
    // Split the line into comma-separated fields
    while (*p && field < 15) {
        if (*p == ',') {
            *p = '\0';
            fields[field++] = line;
            line = p + 1;
        }
        p++;
    }
    
    // Display: Time | Position in degrees | Altitude | Satellites | HDOP
    if (fields[1] && fields[2] && fields[4] && fields[9]) {
        float lat = nmea_to_degrees(fields[2], fields[3][0]);
        float lon = nmea_to_degrees(fields[4], fields[5][0]);
        
        printk("%c%c:%c%c:%c%c | %.6f° %c, %.6f° %c | Alt: %s m | Sats: %s | HDOP: %s\n",
               fields[1][0], fields[1][1], fields[1][2], 
               fields[1][3], fields[1][4], fields[1][5],
               lat >= 0 ? lat : -lat, fields[3][0],  // Latitude
               lon >= 0 ? lon : -lon, fields[5][0],  // Longitude
               fields[9],             // Altitude
               fields[7],             // Satellites
               fields[8]);            // HDOP (precision)
    }
}
static void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;
    
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &c, 1) == 1) {
            if (c == '$') {
                line_pos = 0;
            }
            
            if (line_pos < BUF_SIZE - 1) {
                nmea_line[line_pos++] = c;
                
                if (c == '\n') {
                    nmea_line[line_pos] = '\0';
                    
                    // Directly display GPGGA or GNGGA sentences
                    if (strstr(nmea_line, "$GPGGA") || strstr(nmea_line, "$GNGGA")) {
                        print_gps_info(nmea_line);
                    }
                    
                    line_pos = 0;
                }
            }
        }
    }
}
int main(void)
{
    printk("\n=== GPS Simple ===\n");
    printk("Zephyr main thread started\n");
 
    uart_dev = DEVICE_DT_GET(UART1_NODE);
    if (!device_is_ready(uart_dev)) {
        printk("UART not ready\n");
        return -1;
    }

    uart_irq_callback_set(uart_dev, uart_isr);
    uart_irq_rx_enable(uart_dev);
    
    printk("Listening on USART1...\n\n");

    while (1) {
        k_sleep(K_MSEC(1000));
        printk("Hello from main thread!\n");
    }
}
