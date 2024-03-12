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
QueueHandle_t FIFO_Acq_to_Comm;
TaskHandle_t xTaskCommHandle;
TaskHandle_t xTaskAcqHandle;

void app_main()
{
    // printf("free mem: %d\n",esp_get_free_heap_size());
    // printf("main CORE  %d\n", xPortGetCoreID());
    
    //* Initialize Components
    esp_err_t ret = nvs_flash_init();    //Initialize NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nvs_flash_init_partition("storage");    //Initialize NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase_partition("storage"));
        ret = nvs_flash_init_partition("storage"); 
    }
    ESP_ERROR_CHECK(ret);


    nvs_handle_t my_handle;
    esp_err_t err = nvs_open_from_partition("storage", "device_info",  
               NVS_READONLY, &my_handle);
    if (err == ESP_OK){
        ESP_LOGI("main.c", "NVS open OK");
        size_t required_size;
        nvs_get_str(my_handle, "type", NULL, &required_size);
        char *type = malloc(required_size*sizeof(char));
        err = nvs_get_str(my_handle, "type", type, &required_size);
        if(err != ESP_OK){
            free(type);
            type = malloc(5*sizeof(char));
            strcpy(type,"null");
        }
    }else{
        ESP_LOGE("main.c", "NVS open error");    
    }
    
    FIFO_Acq_to_Comm = xQueueCreate(2, sizeof(MeasData));
    //* Initialize taskstask_acquire, "Acquisition task for MAX11254", 4096, NULL, 10, &xTaskAcqHandle); // ADE7880
   
    xTaskCreatePinnedToCore(
                            task_acquire,   /* Function to implement the task */
                            "Acquisition task", /* Name of the task */
                            4096,       /*Stack size in words */
                            NULL,       /* Task input parameter */
                            100,          /* Priority of the task */
                            &xTaskAcqHandle,       /* Task handle. */
                            1);  /* Core where the task should run */
  
    xTaskCreatePinnedToCore(task_communication, "Communication task for MQTT", 4096, NULL, 10, &xTaskCommHandle, 0); // MQTT
}