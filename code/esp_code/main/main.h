#ifndef __MAIN_H
#define __MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"


#define FIRMWARE_VERSION    "1.0"

typedef enum meas_code_t { COMM_FAIL = 0, COMM_OK = 1 } meas_code_t;


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitSet(value, bit) ((value) |= (1UL << (bit)))

//* FreeRTOS Queues
extern QueueHandle_t FIFO_Meas_to_Cont;
//* FreeRTOS Tasks
extern TaskHandle_t xTaskControlHandle;
extern TaskHandle_t xTaskMeasHandle;

#endif /** __MAIN_H */