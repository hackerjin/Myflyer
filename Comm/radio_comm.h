#ifndef _RADIO_H
#define _RADIO_H
#include <stdint.h>
#include <stdbool.h>

void radio_init();

void radio_comm_task(void *param);
#endif
