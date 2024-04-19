
#include "nvs.h"
#include "math.h"

#include <iostream>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "../../main/main.h"
#include "meas_data.h"


#include <bitset>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <algorithm>

extern "C" {
#include "tof.h"
#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"
}


#define XSHUT1 GPIO_NUM_23
#define XSHUT2 GPIO_NUM_5
#define XSHUT3 GPIO_NUM_25
#define XSHUT4 GPIO_NUM_12


static const char *TAG = "task_meas.c";


calibration_t cal = {
	.mag_offset = {.x = -2.382812, .y = 22.113281, .z = -106.015625},
    .mag_scale = {.x = 0.942115, .y = 1.042027, .z = 1.021565},
	 .gyro_bias_offset = {.x = 3.437389, .y = -0.237054, .z = -0.408434},

	.accel_offset = {.x = -0.006614, .y = 0.019981, .z = -0.091285},
    .accel_scale_lo = {.x = 1.003011, .y = 1.013438, .z = 0.972522},
    .accel_scale_hi = {.x = -1.003911, .y = -0.992579, .z = -1.059406},
	};

void eulerToQuaternion(double roll, double pitch, double yaw, double* quaternion) {
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);

	quaternion[0] = cr * cp * cy + sr * sp * sy; // x
	quaternion[1] = sr * cp * cy - cr * sp * sy; // y
	quaternion[2] = cr * sp * cy + sr * cp * sy; // z
	quaternion[3] = cr * cp * sy - sr * sp * cy; // w
}
/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

//   v->x = -x;
//   v->y = -z;
//   v->z = -z;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = y;
  v->y = x;
  v->z = -z;
}

MeasData meas;
QueueHandle_t FIFO_Acq_to_Comm;
TaskHandle_t xTaskCommHandle;

static void IRAM_ATTR gpio_isr_handler(void* arg){

}

extern "C" void task_meas(void * arg)
{
    i2c_init();
    VL53L1_Dev_t vl53l1_dev_1;
    VL53L1_Dev_t vl53l1_dev_2;
    VL53L1_Dev_t vl53l1_dev_3;
    VL53L1_Dev_t vl53l1_dev_4;
    VL53L1_Dev_t *tof_sensors[4] = {&vl53l1_dev_1,&vl53l1_dev_2,&vl53l1_dev_3,&vl53l1_dev_4};
    gpio_num_t xshut_pins[4] = {XSHUT1, XSHUT2, XSHUT3, XSHUT4};
    int addresses[4] = {0x2A,0x2B,0x2C,0x2D};

    int i2c_master_port = I2C_MASTER_NUM;

    for(int i = 0; i < 4; i++){
        gpio_pad_select_gpio(xshut_pins[i]);
        gpio_set_direction(xshut_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(xshut_pins[i], 0);  
        tof_sensors[i]->I2cHandle = &i2c_master_port;
        tof_sensors[i]->I2cDevAddr = 0x29;
    }   
    
    for (int i = 0; i < 4; i++){
        gpio_pad_select_gpio(xshut_pins[i]);
        gpio_set_direction(xshut_pins[i], GPIO_MODE_INPUT);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        vl53l1_init(tof_sensors[i],addresses[i]);
    }

    // uint8_t pin_example = 15;
    // gpio_config_t io_conf;
    // io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pull_down_en = false;
    // io_conf.pull_up_en = false;
    // io_conf.pin_bit_mask = (1ULL << pin_example);
    // gpio_config(&io_conf);
    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(pin_example, gpio_isr_handler, (void*) &pin_example);


    uint64_t cycle_time = esp_timer_get_time();
    uint64_t act_time;
    uint64_t loop_counter = 0;
    /***
     * MPU9250
    */
    uint64_t start_time = 0, end_time = 0;
    uint16_t frequency = 0;
    uint64_t i = 0;
    float temp = 0;

    // i2c_mpu9250_init(&cal);
    // ahrs_init(SAMPLE_FREQ_Hz, 0.9);

    for(;;){
        start_time = esp_timer_get_time();
        vector_t va, vg, vm;
        vector_t va_temp, vg_temp, vm_temp;

        // Get the Accelerometer, Gyroscope and Magnetometer values.
        // ESP_ERROR_CHECK(get_accel_gyro(&va, &vg));
        va_temp = va;
        vg_temp = vg;
        vm_temp = vm;
        // xQueueSend( FIFO_Acq_to_Comm, &meas, 10 / portTICK_RATE_MS ); 
        // xTaskNotify(xTaskCommHandle, ADE_MEASURE_OK, eSetBits); // Notify the other task
        // transform_accel_gyro(&va);
	    // transform_accel_gyro(&vg);
        end_time = esp_timer_get_time();
        frequency = 1000000.0/(end_time-start_time);
        // ahrs_init(frequency, 0.9);

        act_time = esp_timer_get_time();
        if ( cycle_time > act_time )  // ak pretecie act_time, vyresetuj cycle_time
            cycle_time = act_time;
        else if ((act_time - cycle_time) > 10000000 ) {
                meas.tof.tof1 = vl53l1_read(tof_sensors[0]);
                meas.tof.tof2 = vl53l1_read(tof_sensors[1]);
                meas.tof.tof3 = vl53l1_read(tof_sensors[2]);
                meas.tof.tof4 = vl53l1_read(tof_sensors[3]);

                printf("TOF1: %g\n",meas.tof.tof1);
                printf("TOF2: %g\n",meas.tof.tof2);
                printf("TOF3: %g\n",meas.tof.tof3);
                printf("TOF4: %g\n",meas.tof.tof4);
                printf("\n");
                // ESP_LOGI("IMU", "Temp	 %2.3f°C, Acc X	 %2.3f, Acc Y	 %2.3f, Acc Z	 %2.3f, Ang X	 %2.3f, Ang Y	 %2.3f, Ang Z	 %2.3f", temp, va_temp.x, va_temp.y, va_temp.z, vg_temp.x, vg_temp.y, vg_temp.z);
            /*
            * Inotify to send data between tasks
            */
            // xQueueSend( FIFO_Acq_to_Comm, &meas, 30 / portTICK_PERIOD_MS );
            // xTaskNotify(xTaskCommHandle, COMM_OK, eSetBits);
        }
        loop_counter++;
    }
}
