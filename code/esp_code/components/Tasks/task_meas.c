
#include "nvs.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "../../main/main.h"
#include "meas_data.h"

static const char *TAG = "task_meas.c";


MeasData meas;
QueueHandle_t FIFO_Acq_to_Comm;
TaskHandle_t xTaskCommHandle;

static void IRAM_ATTR gpio_isr_handler(void* arg){

}

void task_meas(void * arg)
{

    uint8_t pin_example = 15;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = false;
    io_conf.pull_up_en = false;
    io_conf.pin_bit_mask = (1ULL << pin_example);
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin_example, gpio_isr_handler, (void*) &pin_example);

   
    uint64_t cycle_time = esp_timer_get_time();
    uint64_t act_time;
    uint64_t loop_counter = 0;
    for(;;){ 
        act_time = esp_timer_get_time();
        if ( cycle_time > act_time )  // ak pretecie act_time, vyresetuj cycle_time
            cycle_time = act_time;
        else if ((act_time - cycle_time) > 10000000 ) {          

            /*
            * Inotify to send data between tasks
            */   
            // xQueueSend( FIFO_Acq_to_Comm, &meas, 30 / portTICK_PERIOD_MS );
            // xTaskNotify(xTaskCommHandle, COMM_OK, eSetBits);
        }
        loop_counter++;
    }
}
