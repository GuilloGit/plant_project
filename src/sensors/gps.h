#ifndef GPS_H
#define GPS_H

#include <zephyr/device.h>

void gps_init(void);
void gps_process_line(char *line);

#endif
