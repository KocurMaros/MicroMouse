
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
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "tof.h"
#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"
#include "udp_client.h"
#include "driver.h"
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

#define MESSAGE_BUFF_LEN 512

typedef struct 
{
	bool A_channel;
	bool B_channel;
}Encoder_channel;

static Position position;
static char message_buff[MESSAGE_BUFF_LEN];

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
	v->y = -y;
	v->z = -z;
}

#define PCNT_H_LIM_VAL      4096
#define PCNT_L_LIM_VAL      0
#define PCNT_THRESH1_VAL    2048

#define PCNT_INPUT_SIG_IO   ENCODER_1_A  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  ENCODER_1_B  // Control GPIO HIGH=count up, LOW=count down


xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */

static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    pcnt_unit_t pcnt_unit = (pcnt_unit_t)((int)arg);
    pcnt_evt_t evt;
    evt.unit = pcnt_unit;
    /* Save the PCNT event type that caused an interrupt
       to pass it to the main program */
    pcnt_get_event_status(pcnt_unit, &evt.status);
    xQueueSendFromISR(pcnt_evt_queue, &evt, NULL);
}


static void pcnt_example_init(pcnt_unit_t unit)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do on the positive / negative edge of pulse input?
        // What to do when control input is low or high?
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
    };

    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    // pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    // pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    // pcnt_event_enable(unit, PCNT_EVT_ZERO);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    // pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Install interrupt service and add isr callback handler */
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
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

    /**
     * @brief PCNT
     * 
     */
    pcnt_unit_t pcnt_unit = PCNT_UNIT_0;
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init(pcnt_unit);

    int16_t count = 0;
    pcnt_evt_t evt;
    portBASE_TYPE res;

    /**
     * @brief CONTROL
     * 
     */
	double prev_inter1 = 0, prev_inter2 = 0;
    int64_t curr_time = 0, printTime = 0;

	double motor_speed_left = 0, motor_speed_right = 0;
	memset(message_buff,'\0', MESSAGE_BUFF_LEN);

    init_motor_driver();

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

            /**
             * @brief TASK CONTROL old code
             * 
             */
            set_speed_dir(100,-100); // 20 min____150 max
            calculate_odometry(&meas.enc,&position);
            if((double)(curr_time - printTime)/1000.0 > 100.0){
				//printf("Pos X: %1.2lf,  Enc1Ticks: %lld, Enc2Ticks: %lld\n", position.x/10.0, meas.enc.encoder1, meas.enc.encoder2);
				//printf("Motor A dir %s, Motor B dir %s\n",meas.enc.dir_A ? "Forward" : "Revers", meas.enc.dir_B ? "Forward" : "Revers");
				//printf("H: %1.2f, P: %1.2f, R: %1.2f\n",meas.orient.heading, meas.orient.pitch, meas.orient.roll);
				printTime = curr_time;
			}

			send_time = esp_timer_get_time();
		}
        res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
            pcnt_get_counter_value(pcnt_unit, &count);
            ESP_LOGI(TAG, "Event PCNT unit[%d]; cnt: %d", evt.unit, count);
            if (evt.status & PCNT_EVT_THRES_1) {
                ESP_LOGI(TAG, "THRES1 EVT");
            }
            if (evt.status & PCNT_EVT_THRES_0) {
                ESP_LOGI(TAG, "THRES0 EVT");
            }
            if (evt.status & PCNT_EVT_L_LIM) {
                ESP_LOGI(TAG, "L_LIM EVT");
            }
            if (evt.status & PCNT_EVT_H_LIM) {
                ESP_LOGI(TAG, "H_LIM EVT");
            }
            if (evt.status & PCNT_EVT_ZERO) {
                ESP_LOGI(TAG, "ZERO EVT");
            }
        } else {
            pcnt_get_counter_value(pcnt_unit, &count);
            ESP_LOGI(TAG, "Current counter value :%d", count);
        }
		act_time = esp_timer_get_time();
		if (cycle_time > act_time) // ak pretecie act_time, vyresetuj cycle_time
			cycle_time = act_time;
		else if ((act_time - cycle_time) > 1000000) {
			cycle_time = act_time;
		}
	}
    deinit_pid(pid_left);
	deinit_pid(pid_right);
}
