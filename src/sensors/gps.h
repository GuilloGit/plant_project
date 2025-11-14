/* ============================================================
 *  GPS.H - Driver for GPS receiver via UART
 * ============================================================
 *  Description:
 *    Driver for GPS receiver that receives NMEA sentences through
 *    USART1. Uses UART interrupts to capture data in the
 *    background (non-blocking).
 *
 *  Operation:
 *    - GPS data is received automatically via ISR
 *    - GPGGA/GNGGA sentences are parsed
 *    - Last valid data is stored in memory
 *    - gps_read() instantly returns the last data
 *
 *  Usage:
 *    1. Call gps_init() at program start
 *    2. Call gps_read() periodically from main loop
 *    3. Check return: 0 = OK, -1 = no GPS fix
 * ============================================================ */

#ifndef GPS_H
#define GPS_H

#include <zephyr/device.h>
#include <stdbool.h>

/* ============================================================
 *  GPS data structure
 * ============================================================ */
struct gps_data {
    float latitude;       /* Latitude in decimal degrees (+N, -S) */
    float longitude;      /* Longitude in decimal degrees (+E, -W) */
    float altitude;       /* Altitude above sea level (meters) */
    uint8_t satellites;   /* Number of satellites in use */
    char time_utc[16];    /* UTC time in HH:MM:SS format */
    bool has_fix;         /* true = valid GPS fix, false = searching for satellites */
};

/* ============================================================
 *  Public functions
 * ============================================================ */

/*
 * Initializes the GPS receiver connected to USART1.
 *
 * Configures:
 *   - Device tree node USART1
 *   - UART RX interrupts
 *   - ISR callback for data reception
 *
 * Note: This function does NOT block. NMEA sentences are
 *       processed automatically in the background.
 */
void gps_init(void);

/*
 * Reads the last GPS data captured by the ISR.
 *
 * Parameters:
 *   data: Pointer to structure where GPS data will be copied
 *
 * Return:
 *    0  -> OK, 'data' contains valid coordinates (has_fix = true)
 *   -1  -> Error: no GPS fix or no data received
 *
 * Note: This function does NOT block. Returns immediately with the
 *       last available data. If GPS doesn't have fix yet,
 *       it will return -1 until it captures satellite signal.
 */
int gps_read(struct gps_data *data);

/*
 * Processes an NMEA line (internal use, called from ISR).
 *
 * Parses GPGGA/GNGGA sentences and extracts:
 *   - Coordinates (lat/lon)
 *   - Altitude
 *   - Number of satellites
 *   - UTC time
 *   - Fix status
 */
void gps_process_line(char *line);

#endif /* GPS_H */
