/**
 * @file main.c
 * @author FETME (kocur.maros@gmail.com)
 * @brief 
 * @version 0.2
 * @date 2024-01-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <string.h>
#include "main.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "../components/Tasks/meas_data.h"

// Variables from main.h
QueueHandle_t FIFO_Meas_to_Cont;

TaskHandle_t xTaskMeasHandle;


extern "C" 
{ 
    #include "udp_client.h"
	void app_main(); 
	void task_meas(void * arg);
} 
void app_main()
{
    //* Initialize Components
    esp_err_t ret = nvs_flash_init();    //Initialize NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    

    

    // FIFO_Meas_to_Cont = xQueueCreate(2, sizeof(MeasData));
    init_udp();

    xTaskCreatePinnedToCore(task_meas,   /* Function to implement the task */
                            "meas data from sensosors", /* Name of the task */
                            8192,       /*Stack size in words */
                            NULL,       /* Task input parameter */
                            100,          /* Priority of the task */
                            &xTaskMeasHandle,       /* Task handle. */
                            0);  /* Core where the task should run */
  
    // xTaskCreatePinnedToCore(task_control, "Control motors and algorithm", 4096, NULL, 100, &xTaskControlHandle, 0);
}