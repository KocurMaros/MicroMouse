
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

#include "driver/adc.h"
#include "esp_adc_cal.h"

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


#define ENCODER_1_A GPIO_NUM_34
#define ENCODER_1_B GPIO_NUM_35
#define ENCODER_2_A GPIO_NUM_32
#define ENCODER_2_B GPIO_NUM_33

#define DEFAULT_VREF 2000 //Use adc2_vref_to_gpio() to obtain a better estimate


typedef struct 
{
	bool A_channel;
	bool B_channel;
}Encoder_channel;

static esp_adc_cal_characteristics_t *adc_chars;

static const char *TAG = "task_meas.c";




   


calibration_t cal = {
	.mag_offset = {.x = 97.166016, .y = 289.740234, .z = 140.156250},
    .mag_scale = {.x = 0.882882, .y = 1.107582, .z = 1.036830},

	.gyro_bias_offset = {.x = -0.234663, .y = -0.495243, .z = -1.010486},


	.accel_offset = {.x = 0.001634, .y = 0.024676, .z = 0.050066},
	.accel_scale_lo = {.x = -0.995133, .y = -0.989244, .z = -1.002114},
    .accel_scale_hi = {.x = 1.003912, .y = 1.013656, .z = 1.015352},

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


Encoder_channel encoderStates[2] = {{.A_channel = 0, .B_channel = 0}, 
									{.A_channel = 0, .B_channel = 0}},
				prev_encoderStates[2] = {{.A_channel = 0, .B_channel = 0}, 
										 {.A_channel = 0, .B_channel = 0}};

double interrupts[2] = { 0, 0 };


int64_t prev_time_1_A = 0;
int64_t prev_time_2_A = 0;

double periodA, periodB;
void dir_isr_handler(void *arg)
{
	uint32_t gpio_num = (uint32_t)arg;
	switch (gpio_num) {
	case ENCODER_1_A:
		if(!encoderStates[0].A_channel){
			if(prev_time_1_A == 0){
				prev_time_1_A = esp_timer_get_time();
			}else if(prev_time_1_A != 0){
				periodA = (double)(esp_timer_get_time() - prev_time_1_A)/1'000'000.0;
				interrupts[0] = 1.0/(periodA);  //imp/s
				prev_time_1_A = 0;
				encoderStates[0].A_channel = true;
			}
		}
		break;	
	case ENCODER_2_A:
		if(!encoderStates[1].A_channel){
			if(prev_time_2_A == 0){
				prev_time_2_A = esp_timer_get_time();
			}else if( prev_time_2_A != 0){
				periodB = (double)(esp_timer_get_time() - prev_time_2_A)/1'000'000.0;
				interrupts[1] = 1.0/(periodB);  //imp/s
				prev_time_2_A = 0;
				encoderStates[1].A_channel = true;
			}
		}
		break;
	}
}
/*
void dir_isr_handler(void *arg)
{
	uint32_t gpio_num = (uint32_t)arg;
	switch (gpio_num) {
	case ENCODER_1_A:
		interrupts[0] += 1.0;  //imp/s
		break;	
	case ENCODER_2_A:
		interrupts[1] += 1.0;  //imp/s
		break;
	}
}
*/




TaskHandle_t xTaskCommHandle;

static uint64_t last_time = 0;
extern "C" void task_meas(void *arg)
{
	printf("Task meas run on core: %d\n", xPortGetCoreID());
	MeasData meas;

	

	uint64_t cycle_time = esp_timer_get_time();
	uint64_t act_time, send_time = 0;

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
	gpio_set_direction(ENCODER_2_A, GPIO_MODE_INPUT);
	gpio_set_direction(ENCODER_1_B, GPIO_MODE_INPUT);
	gpio_set_direction(ENCODER_2_B, GPIO_MODE_INPUT);

	gpio_set_intr_type(ENCODER_1_A, GPIO_INTR_NEGEDGE /*GPIO_INTR_POSEDGE*/);
	gpio_set_intr_type(ENCODER_2_A, GPIO_INTR_NEGEDGE /*GPIO_INTR_POSEDGE*/);
	// gpio_set_intr_type(ENCODER_1_B, GPIO_INTR_NEGEDGE);
	// gpio_set_intr_type(ENCODER_2_B, GPIO_INTR_NEGEDGE);
	
	gpio_install_isr_service(0);
	
	gpio_isr_handler_add(ENCODER_1_A, dir_isr_handler, (void *)ENCODER_1_A);
	gpio_isr_handler_add(ENCODER_2_A, dir_isr_handler, (void *)ENCODER_2_A);
	// gpio_isr_handler_add(ENCODER_1_B, dir2_isr_handler, (void *)ENCODER_1_B);
	// gpio_isr_handler_add(ENCODER_2_B, dir2_isr_handler, (void *)ENCODER_2_B);

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
	VL53L1_Dev_t *tof_sensors[4] = { &vl53l1_dev_1, &vl53l1_dev_2, &vl53l1_dev_3, &vl53l1_dev_4 };
	gpio_num_t xshut_pins[4] = { XSHUT1, XSHUT2, XSHUT3, XSHUT4 };
	int addresses[4] = { 0x2A, 0x2B, 0x2C, 0x2D };

	int i2c_master_port = I2C_MASTER_NUM;

	for (int i = 0; i < 4; i++) {
		gpio_pad_select_gpio(xshut_pins[i]);
		gpio_set_direction(xshut_pins[i], GPIO_MODE_OUTPUT);
		gpio_set_level(xshut_pins[i], 0);
		tof_sensors[i]->I2cHandle = &i2c_master_port;
		tof_sensors[i]->I2cDevAddr = 0x29;
	}

	for (int i = 0; i < 4; i++) {
		gpio_pad_select_gpio(xshut_pins[i]);
		gpio_set_direction(xshut_pins[i], GPIO_MODE_INPUT);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		vl53l1_init(tof_sensors[i], addresses[i]);
	}

	/***
     * MPU9250 init
    */
	uint64_t end_time = 0;

	(void)gpio_set_level(GPIO_NUM_13, 1);

	//////////////  CALIBRATION ROUTINE ////////////////

	// printf("Calib starts in a sec\n");
	// vTaskDelay(1000 / portTICK_PERIOD_MS);
	// calibrate_gyro();
	// calibrate_accel();
	// printf("Steal the dejta\n");
	// vTaskDelay(5000 / portTICK_PERIOD_MS);
	// calibrate_mag();



	/////////////////////////////////////////////////////
 
	i2c_mpu9250_init(&cal);
	ahrs_init(200, 0.9); // 200 Hz, 0.8 beta

	//start after presing button

	gpio_pad_select_gpio(GPIO_NUM_0);
	(void)gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);
	(void)gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);

	while (gpio_get_level(GPIO_NUM_0) != 0) {
		// continue reading GPIO_NUM_0
	}

	meas.log.button_start = true;

	(void)adc1_config_width(ADC_WIDTH_BIT_12);
	(void)adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

	//Characterize ADC
	adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
	uint32_t ulNotifiedValue;

	double prev_inter1 = 0, prev_inter2 = 0;

	for (;;) {
		// Get the Accelerometer, Gyroscope and Magnetometer values.
		meas.log.voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_3), adc_chars) * 2.0;

		act_time = esp_timer_get_time();
		if ((act_time - end_time) > 5000) { //200Hz
			// ahrs_init(1'000'000/(act_time - end_time), 0.8); 
			ESP_ERROR_CHECK(get_accel_gyro(&va, &vg));
			transform_accel_gyro(&va);
			transform_accel_gyro(&vg);
			// transform_mag(&vm);

			ahrs_update_imu(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z), va.x, va.y, va.z);
			end_time = esp_timer_get_time();
		}

		act_time = esp_timer_get_time();
		if ((act_time - send_time) > 1'000) { //1kHz
            //printf("Motor A dir: %s, Motor B dir: %s\n", meas.enc.dir_A ? "Front" : "Reverse", meas.enc.dir_B ? "Front" : "Reverse");
			double inter1 = (int64_t)round(interrupts[0]/250.0);
			double inter2 = (int64_t)round(interrupts[1]/250.0);

			if (inter1 != 0 || abs(inter1 - prev_inter1) < 10) {
				meas.enc.encoder1 = (int64_t)round(interrupts[0]/250.0);
				
			}

			if(inter2 != 0 || abs(inter1 - prev_inter1) < 10){
				meas.enc.encoder2 = (int64_t)round(interrupts[1]/250.0);
			}

			prev_inter1 = inter1;
			prev_inter2 = inter2;
			
			// printf("Inter0: %lld, \tInter0: %lld\n",meas.enc.encoder1, meas.enc.encoder2);
			// meas.enc.encoder2 = (uint64_t)round(interrupts[1]/500.0); //(int64_t)interrupts[1];
			encoderStates[0].A_channel = false;
			encoderStates[1].A_channel = false;
			// printf("ENC1_m = %.2lf, ENC2_m = %.2lf\n states %d %d %lld %lld\n", interrupts[0], interrupts[1], encoderStates[0].A_channel, encoderStates[1].A_channel, prev_time_1_A, prev_time_2_A);

			int64_t time = esp_timer_get_time();
			meas.enc.time_diff = time - last_time;
			last_time = time;
			//printf("dt = %llu\n", meas.enc.time_diff);

			for (size_t i = 0; i < 2; i++)
				interrupts[i] = 0;

			meas.tof.tof1 = vl53l1_read(tof_sensors[0]) / 1000.0;
			meas.tof.tof2 = vl53l1_read(tof_sensors[1]) / 1000.0;
			meas.tof.tof3 = vl53l1_read(tof_sensors[2]) / 1000.0;
			meas.tof.tof4 = vl53l1_read(tof_sensors[3]) / 1000.0;

			ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
			meas.orient.roll = roll;
			meas.orient.pitch = pitch;
			meas.orient.heading = heading;

			(void)xQueueSend(FIFO_Meas_to_Cont, &meas, 50 / portTICK_RATE_MS);
			
			send_time = esp_timer_get_time();
			// printf("Queue Send: %s\n", qRet == pdTRUE ? "kebap" : "questionable kebap");
			random_flag++;
			
		}
		act_time = esp_timer_get_time();
		if (cycle_time > act_time) // ak pretecie act_time, vyresetuj cycle_time
			cycle_time = act_time;
		else if ((act_time - cycle_time) > 1000000) {
			cycle_time = act_time;
			/*
            * Inotify to send data between tasks
            */
			// xTaskNotify(xTaskMeasHandle, COMM_OK, eSetBits); // Notify the other task
		}
	}
}
