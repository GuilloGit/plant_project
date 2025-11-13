/* src/main.h */
#ifndef MAIN_H_
#define MAIN_H_

#include <zephyr/kernel.h>

/* System state definitions, shared between main.c and monitor.c */
typedef enum {
    NORMAL_STATE,
} system_state_t;

extern volatile system_state_t current_state;

#endif /* MAIN_H_ */