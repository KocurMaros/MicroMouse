
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
        // tof_sensors[i]->I2cHandle = &i2c_master_port;
        // tof_sensors[i]->I2cDevAddr = 0x29;
    }   
 
    tof_sensors[0]->I2cHandle = &i2c_master_port;
    tof_sensors[0]->I2cDevAddr = 0x29;

    tof_sensors[1]->I2cHandle = &i2c_master_port;
    tof_sensors[1]->I2cDevAddr = 0x29;

    tof_sensors[2]->I2cHandle = &i2c_master_port;
    tof_sensors[2]->I2cDevAddr = 0x29;

    tof_sensors[3]->I2cHandle = &i2c_master_port;
    tof_sensors[3]->I2cDevAddr = 0x29;

    gpio_pad_select_gpio(xshut_pins[0]);
    gpio_set_direction(xshut_pins[0], GPIO_MODE_INPUT);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vl53l1_init(tof_sensors[0],addresses[0]);
   
    gpio_pad_select_gpio(xshut_pins[1]);
    gpio_set_direction(xshut_pins[1], GPIO_MODE_INPUT);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vl53l1_init(tof_sensors[1],addresses[1]);

    gpio_pad_select_gpio(xshut_pins[2]);
    gpio_set_direction(xshut_pins[2], GPIO_MODE_INPUT);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vl53l1_init(tof_sensors[2],addresses[2]);

    gpio_pad_select_gpio(xshut_pins[3]);
    gpio_set_direction(xshut_pins[3], GPIO_MODE_INPUT);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vl53l1_init(tof_sensors[3],addresses[3]);


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
                vl53l1_read(tof_sensors[0]);
                vl53l1_read(tof_sensors[1]);
                vl53l1_read(tof_sensors[2]);
                vl53l1_read(tof_sensors[3]);

            /*
            * Inotify to send data between tasks
            */
            // xQueueSend( FIFO_Acq_to_Comm, &meas, 30 / portTICK_PERIOD_MS );
            // xTaskNotify(xTaskCommHandle, COMM_OK, eSetBits);
        }
        loop_counter++;
    }
}
