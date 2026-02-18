#include "freertos.h"
#include "queue.h"
#ifndef GETPPM_TASK_H
#define GETPPM_TASK_H

extern QueueHandle_t QueuePPM;

struct PPM_Data{
	uint16_t ppmCh[9];
};
extern void getPPM_Task();

#endif
