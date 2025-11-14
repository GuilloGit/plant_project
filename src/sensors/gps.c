/* ============================================================
 *  GPS.C - Implementación del driver GPS NMEA
 * ============================================================
 *  Autor: Plant Project
 *  Descripción:
 *    Driver para receptores GPS que transmiten sentencias NMEA
 *    a 9600 baudios por USART1. Implementa recepción por
 *    interrupciones (non-blocking) y parsing de GPGGA.
 *
 *  Protocolo NMEA:
 *    Las sentencias GPS siguen el formato:
 *    $GPGGA,hhmmss.ss,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,...*chk
 *
 *  Arquitectura:
 *    - ISR (uart_isr): Captura bytes UART, construye líneas NMEA
 *    - Parser: Extrae campos y convierte a grados decimales
 *    - Storage: Almacena último fix válido en variable estática
 *    - API: gps_read() copia datos sin bloquear
 *
 *  Optimización ISR:
 *    - No se usan printk() en ISR para minimizar latencia
 *    - Procesamiento mínimo (solo parsing básico)
 *    - Datos impresos desde main thread
 * ============================================================ */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdlib.h>

#include "gps.h"

/* ============================================================
 *  Configuración y constantes
 * ============================================================ */

#define UART1_NODE DT_NODELABEL(usart1)  /* Nodo USART1 en device tree */
#define BUF_SIZE 128                      /* Tamaño buffer para línea NMEA */

/* ============================================================
 *  Variables estáticas (estado del driver)
 * ============================================================ */

/* Device handle del UART */
static const struct device *uart_dev;

/* Buffer para construir línea NMEA carácter por carácter */
static char nmea_line[BUF_SIZE];
static uint8_t line_pos = 0;

/* Flags de control para evitar procesar múltiples GPGGA por ciclo */
static volatile bool gga_processed = false;   /* Ya se procesó GPGGA en este ciclo */
static volatile bool got_gpgga = false;       /* Se recibió al menos un GPGGA válido */
static bool last_error = false;               /* Flag para evitar spam de errores */
static volatile int64_t last_uart_rx_time = 0; /* Timestamp de última recepción UART */

/* Estructura con los últimos datos GPS válidos recibidos */
static struct gps_data last_gps_data = {
    .latitude = 0.0f,
    .longitude = 0.0f,
    .altitude = 0.0f,
    .satellites = 0,
    .time_utc = "--:--:--",
    .has_fix = false
};

/* ============================================================
 *  Funciones auxiliares internas
 * ============================================================ */

/*
 * Resetea flags de procesamiento (uso interno).
 *
 * Permite volver a procesar sentencias GPGGA en el siguiente
 * ciclo de lectura. Útil si se implementa polling periódico.
 */
void gps_reset_flag(void)
{
    gga_processed = false;
    got_gpgga = false;
}

/*
 * Convierte coordenadas NMEA a grados decimales.
 *
 * Formato NMEA: DDDMM.MMMM (grados + minutos)
 * Ejemplo: "3723.2475" = 37° 23.2475' = 37.387458°
 *
 * Parámetros:
 *   nmea: String con coordenada en formato NMEA
 *   dir:  Dirección ('N', 'S', 'E', 'W')
 *
 * Retorno:
 *   Coordenada en grados decimales.
 *   Valores negativos para Sur (S) y Oeste (W).
 */
static float nmea_to_degrees(const char *nmea, char dir)
{
    if (!nmea || strlen(nmea) < 4)
        return 0.0f;

    /* Parsear valor completo */
    float value = atof(nmea);

    /* Extraer grados (parte entera al dividir por 100) */
    int degrees = (int)(value / 100);

    /* Extraer minutos (resto) */
    float minutes = value - (degrees * 100);

    /* Convertir: grados + (minutos / 60) */
    float result = degrees + (minutes / 60.0f);

    /* Aplicar signo según hemisferio */
    return (dir == 'S' || dir == 'W') ? -result : result;
}

/* ============================================================
 *  gps_process_line()
 *  ------------------------------------------------------------
 *  Parsea una sentencia NMEA GPGGA y extrae los datos GPS.
 *
 *  Formato GPGGA:
 *    $GPGGA,hhmmss.ss,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,...*chk
 *    Campos:
 *      [0]: $GPGGA (identificador)
 *      [1]: UTC time (hhmmss.ss)
 *      [2]: Latitud (DDMM.MMMM)
 *      [3]: N/S (Norte/Sur)
 *      [4]: Longitud (DDDMM.MMMM)
 *      [5]: E/W (Este/Oeste)
 *      [6]: Fix quality (0=sin fix, 1=GPS, 2=DGPS)
 *      [7]: Número de satélites
 *      [8]: HDOP (precisión horizontal)
 *      [9]: Altitud (metros)
 *
 *  Esta función se llama desde la ISR cuando se recibe una
 *  línea GPGGA completa. Almacena los datos en last_gps_data.
 * ============================================================ */
void gps_process_line(char *line)
{
    char *p = line;
    int field = 0;
    char *fields[20] = {0};

    /* Separar campos por comas (parser simple) */
    while (*p && field < 20) {
        if (*p == ',') {
            *p = '\0';  /* Reemplazar coma por null terminator */
            fields[field++] = line;
            line = p + 1;
        }
        p++;
    }

    /* Extraer campos relevantes */
    char *utc   = fields[1];  /* Hora UTC */
    char *lat_s = fields[2];  /* Latitud (string) */
    char *lat_d = fields[3];  /* Dirección latitud (N/S) */
    char *lon_s = fields[4];  /* Longitud (string) */
    char *lon_d = fields[5];  /* Dirección longitud (E/W) */
    char *fixq  = fields[6];  /* Calidad del fix (0=no fix) */
    char *sats  = fields[7];  /* Número de satélites */
    char *alt   = fields[9];  /* Altitud (metros) */

    /* Formatear hora UTC (de "123456" a "12:34:56") - SIEMPRE, incluso sin fix */
    if (utc && strlen(utc) >= 6) {
        snprintf(last_gps_data.time_utc, sizeof(last_gps_data.time_utc),
                 "%c%c:%c%c:%c%c",
                 utc[0], utc[1], utc[2], utc[3], utc[4], utc[5]);
    } else {
        snprintf(last_gps_data.time_utc, sizeof(last_gps_data.time_utc), "--:--:--");
    }

    /* Verificar si hay fix GPS válido */
    if (!fixq || atoi(fixq) == 0) {
        last_gps_data.has_fix = false;
        /* Comentado para evitar bloqueo en ISR - verificar estado con gps_read() */
        // printk("GPS: Searching satellites...\n");
        return;
    }

    /* Convertir coordenadas NMEA a grados decimales */
    last_gps_data.latitude = nmea_to_degrees(lat_s, lat_d[0]);
    last_gps_data.longitude = nmea_to_degrees(lon_s, lon_d[0]);
    last_gps_data.altitude = alt ? atof(alt) : 0.0f;
    last_gps_data.satellites = sats ? (uint8_t)atoi(sats) : 0;
    last_gps_data.has_fix = true;

    /* Comentado para evitar bloqueo en contexto ISR.
     * Los datos GPS se imprimen desde el main thread usando gps_read().
     * Esto minimiza la latencia de la ISR y evita interferir con otras interrupciones. */
    // printk("GPS → Lat: %.5f  Lon: %.5f  Alt: %.1f m  Sats: %u  Time: %s\n",
    //        (double)last_gps_data.latitude, (double)last_gps_data.longitude,
    //        (double)last_gps_data.altitude, last_gps_data.satellites,
    //        last_gps_data.time_utc);
}

/* ============================================================
 *  uart_isr()
 *  ------------------------------------------------------------
 *  Interrupt Service Routine para recepción UART.
 *
 *  Funcionamiento:
 *    1. Lee bytes del FIFO UART uno por uno
 *    2. Construye línea NMEA carácter por carácter
 *    3. Detecta inicio de sentencia ('$')
 *    4. Detecta fin de línea ('\n')
 *    5. Parsea solo sentencias GPGGA/GNGGA
 *    6. Evita procesar múltiples GPGGA en un ciclo
 *
 *  Optimización:
 *    - Ejecución rápida (sin printk)
 *    - Solo procesamiento esencial
 *    - Flags para control de flujo
 * ============================================================ */
static void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    /* Procesar todos los bytes disponibles en el FIFO */
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {

        if (uart_fifo_read(dev, &c, 1) == 1) {
            /* NO actualizar timestamp aquí - solo al recibir sentencia NMEA válida */

            /* Detectar inicio de sentencia NMEA */
            if (c == '$')
                line_pos = 0;  /* Resetear buffer */

            /* Agregar carácter al buffer */
            if (line_pos < BUF_SIZE - 1) {
                nmea_line[line_pos++] = c;

                /* Detectar fin de línea */
                if (c == '\n') {
                    nmea_line[line_pos] = '\0';  /* Null terminator */

                    /* Procesar solo GPGGA/GNGGA, una vez por ciclo */
                    if (!gga_processed &&
                       (strstr(nmea_line, "$GPGGA") ||
                        strstr(nmea_line, "$GNGGA")))
                    {
                        /* Actualizar timestamp INMEDIATAMENTE al detectar GPGGA */
                        last_uart_rx_time = k_uptime_get();
                        
                        gps_process_line(nmea_line);
                        gga_processed = true;  /* Evitar reprocesar */
                        got_gpgga = true;      /* Marcar que hay datos */
                    }

                    line_pos = 0;  /* Preparar para siguiente línea */
                }
            }
        }
    }
}

/* ============================================================
 *  gps_init()
 *  ------------------------------------------------------------
 *  Inicializa el receptor GPS en USART1.
 *
 *  Configuración:
 *    1. Obtiene device handle de USART1 desde device tree
 *    2. Verifica que el UART esté listo
 *    3. Registra callback ISR para interrupciones RX
 *    4. Habilita interrupciones de recepción UART
 *
 *  Nota: Después de llamar esta función, el GPS comenzará a
 *        recibir datos automáticamente en segundo plano.
 *        No es necesario polling manual.
 * ============================================================ */
void gps_init(void)
{
    /* Obtener device handle desde device tree */
    uart_dev = DEVICE_DT_GET(UART1_NODE);

    /* Verificar que UART esté listo */
    if (!device_is_ready(uart_dev)) {
        printk("GPS: ERROR (UART not ready)\n");
        last_error = true;
        return;
    }

    printk("GPS: UART device ready at %p\n", uart_dev);

    /* Configurar ISR y habilitar interrupciones RX */
    uart_irq_callback_set(uart_dev, uart_isr);
    printk("GPS: ISR callback set\n");
    
    uart_irq_rx_enable(uart_dev);
    printk("GPS: RX interrupts enabled\n");

    /* Inicializar timestamp en 0 para detectar si nunca se reciben datos */
    last_uart_rx_time = 0;

    last_error = false;
    printk("GPS UART Ready - waiting for NMEA data...\n");
}

/* ============================================================
 *  gps_read()
 *  ------------------------------------------------------------
 *  Lee los últimos datos GPS capturados por la ISR.
 *
 *  Parámetros:
 *    data: Puntero a estructura gps_data donde se copiarán
 *          los datos GPS (lat, lon, alt, sats, time, etc.)
 *
 *  Retorno:
 *     0  -> Éxito. 'data' contiene coordenadas válidas.
 *          El GPS tiene fix y los datos son confiables.
 *    -1  -> Error. Sin fix GPS o sin datos recibidos.
 *          Posibles causas:
 *          - GPS aún buscando satélites
 *          - Cable desconectado
 *          - Receptor GPS apagado
 *
 *  Comportamiento:
 *    - NON-BLOCKING: Retorna inmediatamente
 *    - Copia los últimos datos válidos almacenados
 *    - No espera nuevos datos
 *    - Seguro para llamar desde main loop
 *
 *  Ejemplo de uso:
 *    struct gps_data gps;
 *    if (gps_read(&gps) == 0) {
 *        printk("Lat: %.5f, Lon: %.5f\n", gps.latitude, gps.longitude);
 *    } else {
 *        printk("GPS sin fix\n");
 *    }
 * ============================================================ */
int gps_read(struct gps_data *data)
{
    /* Validar puntero */
    if (!data) {
        return -1;
    }

    /* CRÍTICO: Resetear flag ANTES de cualquier verificación
     * Esto permite que la ISR procese la próxima GPGGA que llegue */
    gga_processed = false;
    
    /* Verificar si UART nunca ha recibido datos o lleva >5s sin recibirlos */
    int64_t current_time = k_uptime_get();
    bool never_received = (last_uart_rx_time == 0 && current_time > 5000);
    bool timeout = (last_uart_rx_time > 0 && (current_time - last_uart_rx_time) > 5000);
    
    if (never_received || timeout) {
        /* Resetear flags para permitir reconexión */
        got_gpgga = false;
        last_gps_data.has_fix = false;
        last_error = true;
        return -2;  /* Código especial para UART desconectado */
    }

    /* Verificar que hay datos GPS válidos con fix */
    if (!got_gpgga || !last_gps_data.has_fix) {
        /* Copiar estructura incluyendo timestamp, aunque no haya fix */
        *data = last_gps_data;
        last_error = true;
        return -1;  /* GPS conectado pero sin fix */
    }

    /* Copiar última estructura GPS válida (atómica) */
    *data = last_gps_data;

    last_error = false;
    return 0;
}
