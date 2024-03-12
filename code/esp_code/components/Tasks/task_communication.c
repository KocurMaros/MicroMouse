#include <string.h>

#include "../../main/main.h"
#include "../../build/config/sdkconfig.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "meas_data.h"
/** FreeRTOS */
static const char *TAG = "task_communication.c";


static MeasData val;
QueueHandle_t FIFO_Acq_to_Comm;

void task_communication(void *arg)
{
   
    uint32_t ulNotifiedValue;
    
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open_from_partition("storage", "device_info",  
               NVS_READONLY, &my_handle);
    
    if (err == ESP_OK)
        ESP_LOGI(TAG, "NVS open OK");
    size_t required_size;
    nvs_get_str(my_handle, "cover", NULL, &required_size);
    char* cover = malloc(required_size*sizeof(char));
    nvs_get_str(my_handle, "cover", cover, &required_size);
    nvs_get_str(my_handle, "type", NULL, &required_size);
    char* type = malloc(required_size*sizeof(char));
    err = nvs_get_str(my_handle, "type", type, &required_size);
 
    if(err != ESP_OK){
        free(cover);
        cover = malloc(5*sizeof(char));
        free(type);
        type = malloc(5*sizeof(char));
        strcpy(cover,"null");
        strcpy(type,"null");
    }
    for(;;)
    {      
       
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
        if(ulNotifiedValue == ATMEL_COMM_OK)
        {
           
        }
    }
}