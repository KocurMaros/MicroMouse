
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

static char message_buff[MESSAGE_BUFF_LEN];

static esp_adc_cal_characteristics_t *adc_chars;

static const char *TAG = "task_meas.c";


calibration_t cal = {
	.mag_offset = {.x = 81.474609, .y = 263.013672, .z = 151.251953},
    .mag_scale = {.x = 0.825025, .y = 1.174650, .z = 1.067693},

    .gyro_bias_offset = {.x = -0.205422, .y = -0.542939, .z = -0.990540},

     .accel_offset = {.x = 0.011020, .y = 0.026382, .z = 0.060800},
    .accel_scale_lo = {.x = 1.003300, .y = 1.013836, .z = 1.022332},
    .accel_scale_hi = {.x = -0.989345, .y = -0.988584, .z = -0.989979},

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


#define PCNT_H_LIM_VAL      16
#define PCNT_L_LIM_VAL      0
#define PCNT_THRESH1_VAL    4096

#define PCNT_INPUT_SIG_IO   ENCODER_1_A  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  0  // Control GPIO HIGH=count up, LOW=count down


xQueueHandle pcnt_evt_queue_motA;   // A queue to handle pulse counter events
xQueueHandle pcnt_evt_queue_motB;
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
int64_t left_motor_rot = 0, right_motor_rot = 0;
double interrupts[2] = { 0, 0 };

/**
 * @brief HANDLERS FOR HALF TURN OF MOTOR SO ONE EIGHTH OF A TURN
 * 
 * @param arg 
 */
static void IRAM_ATTR pcnt_example_intr_handler_motA(void *arg){
    left_motor_rot++;   
}
static void IRAM_ATTR pcnt_example_intr_handler_motB(void *arg){
    right_motor_rot++;
}

static void pcnt_example_init(pcnt_unit_t unit_motA, pcnt_unit_t unit_motB){
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config_motA = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = ENCODER_1_A,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
        .unit = unit_motA,
        .channel = PCNT_CHANNEL_0,
    };

    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config_motA);

    /* Configure and enable the input filter */
    // pcnt_set_filter_value(unit_motA, 100);
    // pcnt_filter_enable(unit_motA);

    pcnt_event_enable(unit_motA, PCNT_EVT_H_LIM);
    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit_motA);
    pcnt_counter_clear(unit_motA);

    /* Install interrupt service and add isr callback handler */
    
    pcnt_config_t pcnt_config_motB = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = ENCODER_2_A,
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
        .unit = unit_motB,
        .channel = PCNT_CHANNEL_1,
    };
    pcnt_unit_config(&pcnt_config_motB);
    
    /* Configure and enable the input filter */
    // pcnt_set_filter_value(unit_motB, 100);
    // pcnt_filter_enable(unit_motB);


    pcnt_event_enable(unit_motB, PCNT_EVT_H_LIM);
    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit_motB);
    pcnt_counter_clear(unit_motB);

    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(unit_motA, pcnt_example_intr_handler_motA, (void *)unit_motA);
    pcnt_isr_handler_add(unit_motB, pcnt_example_intr_handler_motB, (void *)unit_motB);
    
    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit_motA);
    pcnt_counter_resume(unit_motB);
}

extern "C" void task_meas(void *arg)
{
	printf("Task meas run on core: %d\n", xPortGetCoreID());
	MeasData meas;

	int64_t esp_boot_time = 0; 
    int64_t process_controller_time = 0;
    int64_t update_rot_data_time = 0;
    int64_t encoder_diff_time = 0;


	vector_t va, vg, vm;
	float roll, pitch, heading;	

	VL53L1_Dev_t vl53l1_dev_1;
	VL53L1_Dev_t vl53l1_dev_2;
	VL53L1_Dev_t vl53l1_dev_3;
	VL53L1_Dev_t vl53l1_dev_4;

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

	(void)gpio_set_level(GPIO_NUM_13, 1);

	//////////////  CALIBRATION ROUTINE ////////////////

	// printf("Calib starts in a sec\n");
	// vTaskDelay(1000 / portTICK_PERIOD_MS);
	// calibrate_gyro();
	// calibrate_accel();
	// printf("Steal the dejta\n");
	// vTaskDelay(5000 / portTICK_PERIOD_MS);

	/////////////////////////////////////////////////////
 
	i2c_mpu9250_init(&cal);
	ahrs_init(200, 0.8); // 200 Hz, 0.8 beta

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
     * @brief PCNT  intterupts from endcoders
     * 
     */
    pcnt_unit_t pcnt_unit_motA = PCNT_UNIT_0;
    pcnt_unit_t pcnt_unit_motB = PCNT_UNIT_1;

    pcnt_evt_queue_motA = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_evt_queue_motB = xQueueCreate(10, sizeof(pcnt_evt_t));

    pcnt_example_init(pcnt_unit_motA, pcnt_unit_motB);

    /**
     * @brief CONTROL
     * 
     */
    int64_t curr_time = 0, printTime = 0;
	memset(message_buff,'\0', MESSAGE_BUFF_LEN);

    float freq = 0;
    uint32_t it = 0;
	for (;;) {
        //low voltage protection
        meas.log.voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_3), adc_chars) * 2.0;
        // if(meas.log.voltage < 3400.0){
        //     while (1){
        //         meas.log.voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_3), adc_chars) * 2.0;
        //         set_speed_dir(0, 0); // 20 min____150 max
        //         printf("Voltage: %1.2f\n", meas.log.voltage);
        //     }
        // }

		// Get the Accelerometer, Gyroscope and Magnetometer values.
		esp_boot_time = esp_timer_get_time();
		if ((esp_boot_time - update_rot_data_time) > 5'000) { //200Hz
            freq = 1'000'000.0/(esp_boot_time - update_rot_data_time);
			ahrs_init(freq, 0.8); 
    		ESP_ERROR_CHECK(get_accel_gyro(&va, &vg));
			transform_accel_gyro(&va);
			transform_accel_gyro(&vg);
            // transform_mag(&vm);
			
            ahrs_update_imu(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                va.x, va.y, va.z);	
            it++;
            update_rot_data_time = esp_boot_time;
     	}

		esp_boot_time = esp_timer_get_time();
		if ((esp_boot_time - process_controller_time) > 10'000) { //30Hz update second task

			meas.tof.tof1 = vl53l1_read(tof_sensors[0]) / 1000.0;
			meas.tof.tof2 = vl53l1_read(tof_sensors[1]) / 1000.0;
			meas.tof.tof3 = vl53l1_read(tof_sensors[2]) / 1000.0;
			meas.tof.tof4 = vl53l1_read(tof_sensors[3]) / 1000.0;

			ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
			meas.orient.roll = roll;
			meas.orient.pitch = pitch;
			meas.orient.heading = heading;

            meas.enc.encoder1 = left_motor_rot *4096/256;
            meas.enc.encoder2 = right_motor_rot*4096/256;
			meas.enc.time_diff = esp_timer_get_time() - encoder_diff_time;
			encoder_diff_time = esp_timer_get_time();
            meas.log.hz_gyro = it;
            meas.log.gyro_freq = freq;

            xQueueSend(FIFO_Meas_to_Cont, &meas, 50 / portTICK_RATE_MS);
            random_flag++;
            
            it = 0;
            left_motor_rot = 0;
            right_motor_rot = 0;
			process_controller_time = esp_boot_time;
		}

		if (process_controller_time > esp_boot_time) // ak pretecie esp_boot_time, vyresetuj cycle_time
			process_controller_time = esp_boot_time;
        if(update_rot_data_time > esp_boot_time)
            update_rot_data_time = esp_boot_time;
		
	}
}
