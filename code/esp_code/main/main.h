#ifndef __MAIN_H
#define __MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#define FIRMWARE_VERSION    "1.0"

typedef enum meas_code_t
{
    ATMEL_COMM_FAIL = 0,
    ATMEL_COMM_OK = 1
} meas_code_t;

//* FreeRTOS Queues
extern QueueHandle_t FIFO_Acq_to_Comm;

#define bitRead(value,bit) (((value) >> (bit)) & 0x01)
#define bitClear(value,bit) ((value) &= ~(1UL << (bit)))
#define bitSet(value,bit) ((value) |= (1UL << (bit)))

//* FreeRTOS Tasks
extern TaskHandle_t xTaskCommHandle;
extern TaskHandle_t xTaskAcqHandWle;

void task_communication(void * arg);
void task_acquire(void * arg);

#endif /** __MAIN_H */