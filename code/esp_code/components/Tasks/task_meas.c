
#include "nvs.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "../../main/main.h"
#include "meas_data.h"

#include "tof.h"

#define XSHUT1 23
#define XSHUT2 5
#define XSHUT3 25
#define XSHUT4 12


static const char *TAG = "task_meas.c";


MeasData meas;
QueueHandle_t FIFO_Acq_to_Comm;
TaskHandle_t xTaskCommHandle;

static void IRAM_ATTR gpio_isr_handler(void* arg){

}

void task_meas(void * arg)
{
    i2c_init();
    VL53L1_Dev_t vl53l1_dev_1;
    VL53L1_Dev_t vl53l1_dev_2;
    VL53L1_Dev_t vl53l1_dev_3;
    VL53L1_Dev_t vl53l1_dev_4;
    VL53L1_Dev_t *tof_sensors[4] = {&vl53l1_dev_1,&vl53l1_dev_2,&vl53l1_dev_3,&vl53l1_dev_4};
    int xshut_pins[4] = {XSHUT1, XSHUT2, XSHUT3, XSHUT4};
    int addresses[4] = {0x2A,0x2B,0x2C,0x2D};

    int i2c_master_port = I2C_MASTER_NUM;

    for(int i = 0; i < 4; i++){
        gpio_pad_select_gpio(xshut_pins[i]);
        gpio_set_direction(xshut_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(xshut_pins[i], 0);  
        tof_sensors[i]->I2cHandle = &i2c_master_port;
        tof_sensors[i]->I2cDevAddr = 0x29;
        tof_sensors[i]->calibrated = 0;
    }   
    
    for (int i = 0; i < 4; i++){
        gpio_pad_select_gpio(xshut_pins[i]);
        gpio_set_direction(xshut_pins[i], GPIO_MODE_INPUT);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        vl53l1_init(tof_sensors[i],addresses[i]);
    }

    uint64_t cycle_time = esp_timer_get_time();
    uint64_t act_time;
    uint64_t loop_counter = 0;
    for(;;){
        act_time = esp_timer_get_time();
        if ( cycle_time > act_time )  // ak pretecie act_time, vyresetuj cycle_time
            cycle_time = act_time;
        else if ((act_time - cycle_time) > 10000 ) {
                meas.tof.tof1 = vl53l1_read(tof_sensors[0])/1000.0;
                meas.tof.tof2 = vl53l1_read(tof_sensors[1])/1000.0;
                meas.tof.tof3 = vl53l1_read(tof_sensors[2])/1000.0;
                meas.tof.tof4 = vl53l1_read(tof_sensors[3])/1000.0;

                printf("TOF1: %g [m]\n",meas.tof.tof1);
                printf("TOF2: %g [m]\n",meas.tof.tof2);
                printf("TOF3: %g [m]\n",meas.tof.tof3);
                printf("TOF4: %g [m]\n",meas.tof.tof4);
                printf("\n");
            /*
            * Inotify to send data between tasks
            */
            // xQueueSend( FIFO_Acq_to_Comm, &meas, 30 / portTICK_PERIOD_MS );
            // xTaskNotify(xTaskCommHandle, COMM_OK, eSetBits);
            cycle_time = act_time;
        }
        loop_counter++;
    }
}
