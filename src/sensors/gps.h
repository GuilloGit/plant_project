/* ============================================================
 *  GPS.H - Driver para receptor GPS vía UART
 * ============================================================
 *  Descripción:
 *    Driver para receptor GPS que recibe sentencias NMEA a través
 *    de USART1. Utiliza interrupciones UART para capturar datos
 *    en segundo plano (non-blocking).
 *
 *  Funcionamiento:
 *    - Los datos GPS se reciben automáticamente vía ISR
 *    - Se parsean las sentencias GPGGA/GNGGA
 *    - Los últimos datos válidos se almacenan en memoria
 *    - gps_read() devuelve instantáneamente los últimos datos
 *
 *  Uso:
 *    1. Llamar gps_init() al inicio del programa
 *    2. Llamar gps_read() periódicamente desde main loop
 *    3. Verificar retorno: 0 = OK, -1 = sin fix GPS
 * ============================================================ */

#ifndef GPS_H
#define GPS_H

#include <zephyr/device.h>
#include <stdbool.h>

/* ============================================================
 *  Estructura de datos GPS
 * ============================================================ */
struct gps_data {
    float latitude;       /* Latitud en grados decimales (+N, -S) */
    float longitude;      /* Longitud en grados decimales (+E, -W) */
    float altitude;       /* Altitud sobre el nivel del mar (metros) */
    uint8_t satellites;   /* Número de satélites en uso */
    char time_utc[16];    /* Hora UTC en formato HH:MM:SS */
    bool has_fix;         /* true = fix GPS válido, false = buscando satélites */
};

/* ============================================================
 *  Funciones públicas
 * ============================================================ */

/*
 * Inicializa el receptor GPS conectado a USART1.
 *
 * Configura:
 *   - Device tree node USART1
 *   - Interrupciones UART RX
 *   - Callback ISR para recepción de datos
 *
 * Nota: Esta función NO bloquea. Las sentencias NMEA se
 *       procesan automáticamente en segundo plano.
 */
void gps_init(void);

/*
 * Lee los últimos datos GPS capturados por la ISR.
 *
 * Parámetros:
 *   data: Puntero a estructura donde se copiarán los datos GPS
 *
 * Retorno:
 *    0  -> OK, 'data' contiene coordenadas válidas (has_fix = true)
 *   -1  -> Error: sin fix GPS o sin datos recibidos
 *
 * Nota: Esta función NO bloquea. Retorna inmediatamente con los
 *       últimos datos disponibles. Si el GPS aún no tiene fix,
 *       retornará -1 hasta que capture señal de satélites.
 */
int gps_read(struct gps_data *data);

/*
 * Procesa una línea NMEA (uso interno, llamada desde ISR).
 *
 * Parsea sentencias GPGGA/GNGGA y extrae:
 *   - Coordenadas (lat/lon)
 *   - Altitud
 *   - Número de satélites
 *   - Hora UTC
 *   - Estado del fix
 */
void gps_process_line(char *line);

#endif /* GPS_H */
