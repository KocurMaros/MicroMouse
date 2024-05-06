
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
#include <stdint.h>

#define XSHUT1 GPIO_NUM_23
#define XSHUT2 GPIO_NUM_5
#define XSHUT3 GPIO_NUM_25
#define XSHUT4 GPIO_NUM_12


#define ENCODER_1_A             GPIO_NUM_34
#define ENCODER_1_B             GPIO_NUM_35
#define ENCODER_2_A             GPIO_NUM_32
#define ENCODER_2_B             GPIO_NUM_33


static const char *TAG = "task_meas.c";


calibration_t cal = {
	.mag_offset = {.x = -2.382812, .y = 22.113281, .z = -106.015625},
    .mag_scale = {.x = 0.942115, .y = 1.042027, .z = 1.021565},
	 .gyro_bias_offset = {.x = 3.437389, .y = -0.237054, .z = -0.408434},

	.accel_offset = {.x = -0.006614, .y = 0.019981, .z = -0.091285},
    .accel_scale_lo = {.x = 1.003011, .y = 1.013438, .z = 0.972522},
    .accel_scale_hi = {.x = -1.003911, .y = -0.992579, .z = -1.059406},
	};


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

  v->x = -x;
  v->y = -z;
  v->z = -y;
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

  v->x = -y;
  v->y = z;
  v->z = -x;
}


QueueHandle_t FIFO_Acq_to_Comm;
TaskHandle_t xTaskCommHandle;

uint64_t interrupts[4] = {0,0,0,0};
void encoder_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    switch (gpio_num)
    {
    case ENCODER_1_A:
        interrupts[0]++;
        break;
    case ENCODER_1_B:
        interrupts[1]++;
        break;
    case ENCODER_2_A:
        interrupts[2]++;
        break;
    case ENCODER_2_B: 
        interrupts[3]++;
        break;
    default:
        break;
    }
}

extern "C" void task_meas(void * arg)
{
    MeasData meas;
    
    uint64_t cycle_time = esp_timer_get_time();
    uint64_t act_time;
    
    vector_t va, vg, vm;
    float roll, pitch, heading;
    
    VL53L1_Dev_t vl53l1_dev_1;
    VL53L1_Dev_t vl53l1_dev_2;
    VL53L1_Dev_t vl53l1_dev_3;
    VL53L1_Dev_t vl53l1_dev_4;
    
    /***
     * Encoder
    */
    gpio_set_direction(ENCODER_1_A, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_1_B, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_2_A, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_2_B, GPIO_MODE_INPUT);
    gpio_set_intr_type(ENCODER_1_A, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(ENCODER_1_B, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(ENCODER_2_A, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(ENCODER_2_B, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_1_A, encoder_isr_handler, (void*) ENCODER_1_A);
    gpio_isr_handler_add(ENCODER_1_B, encoder_isr_handler, (void*) ENCODER_1_B);
    gpio_isr_handler_add(ENCODER_2_A, encoder_isr_handler, (void*) ENCODER_2_A);
    gpio_isr_handler_add(ENCODER_2_B, encoder_isr_handler, (void*) ENCODER_2_B);

    /**
     * Turn of MPU9250
    */
    gpio_pad_select_gpio(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, 0);  

    /**
     * TOF init
    */
    i2c_init();
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

    /***
     * MPU9250 init
    */
    uint64_t end_time = 0;
   
    gpio_set_level(GPIO_NUM_13, 1); 
    vTaskDelay(100 / portTICK_PERIOD_MS); 
    i2c_mpu9250_init(&cal);
    ahrs_init(200, 0.8);        // 200 Hz, 0.8 beta
 
    for(;;){    

        // Get the Accelerometer, Gyroscope and Magnetometer values.
        act_time = esp_timer_get_time();
        if((act_time - end_time) > 5000){     //200Hz
            ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));
            transform_accel_gyro(&va);
            transform_accel_gyro(&vg);
            transform_mag(&vm);

            ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                    va.x, va.y, va.z,
                    vm.x, vm.y, vm.z);
            meas.orient.roll = roll;
            meas.orient.pitch = pitch;
            meas.orient.heading = heading;
            end_time = esp_timer_get_time();
        }
        meas.tof.tof1 = vl53l1_read(tof_sensors[0])/1000.0;
        meas.tof.tof2 = vl53l1_read(tof_sensors[1])/1000.0;
        meas.tof.tof3 = vl53l1_read(tof_sensors[2])/1000.0;
        meas.tof.tof4 = vl53l1_read(tof_sensors[3])/1000.0;
        act_time = esp_timer_get_time();
        if ( cycle_time > act_time )  // ak pretecie act_time, vyresetuj cycle_time
            cycle_time = act_time;
        else if ((act_time - cycle_time) > 1000000 ) {
                meas.enc.encoder1 = interrupts[0];
                meas.enc.encoder2 = interrupts[1];
                meas.enc.encoder3 = interrupts[2];
                meas.enc.encoder4 = interrupts[3];
                printf("TOF1: %g\n",meas.tof.tof1);
                printf("TOF2: %g\n",meas.tof.tof2);
                printf("TOF3: %g\n",meas.tof.tof3);
                printf("TOF4: %g\n",meas.tof.tof4);
                printf("\n");
                ahrs_get_euler_in_degrees(&heading, &pitch, &roll);

                printf("Roll: %g\n",roll);  
                printf("Pitch: %g\n",pitch);
                printf("heading: %g\n",heading);
                cycle_time = act_time;  
           /*
            * Inotify to send data between tasks
            */
            // xQueueSend( FIFO_Acq_to_Comm, &meas, 30 / portTICK_PERIOD_MS );
            // xTaskNotify(xTaskCommHandle, COMM_OK, eSetBits);
            // xQueueSend( FIFO_Acq_to_Comm, &meas, 10 / portTICK_RATE_MS ); 
            // xTaskNotify(xTaskCommHandle, ADE_MEASURE_OK, eSetBits); // Notify the other task
        }
    }
}
