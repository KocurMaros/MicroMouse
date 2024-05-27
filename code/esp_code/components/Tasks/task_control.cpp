#include <string.h>

#include "../../main/main.h"

#include "../../build/config/sdkconfig.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "meas_data.h"
extern "C" {
    #include "udp_client.h"
    #include "driver.h"
}
#define MESSAGE_BUFF_LEN 512

static const char *TAG = "task_control.c";

static MeasData val;
static Position position;
static char message_buff[MESSAGE_BUFF_LEN];

extern "C" void task_control(void *arg)
{
	printf("Task control run on core: %d\n", xPortGetCoreID());

	uint32_t pwm = 0;
	bool up = true;
	double motor_speed_left = 0, motor_speed_right = 0;
	memset(message_buff,'\0', MESSAGE_BUFF_LEN);

	init_motor_driver();


	uint64_t prev_random_flag = 0;
	int64_t prev_time = esp_timer_get_time(), curr_time = 0, printTime = 0;
	for (;;) {
		curr_time = esp_timer_get_time();
		/**
         * TOTO tu nechajte
        */
		if (prev_random_flag < random_flag) {
			(void)xQueueReceive(FIFO_Meas_to_Cont, &val, (100/portTICK_PERIOD_MS)); // wait longer than 10 ms 
			// printf("Queue Recieve: %s\n", qRec == pdTRUE ? "OK" : "ERROR");
			// printf("Received data from meas\n");
			// printf("Roll: %f\n", val.orient.roll);
			// printf("Pitch: %f\n", val.orient.pitch);
			// printf("Heading: %f\n", val.orient.heading);
			// printf("Voltage %f\n", val.log.voltage);

            prev_random_flag = random_flag;
        }
        if(prev_random_flag > random_flag){
            printf("Error: Random flag is smaller than previous\n");
            prev_random_flag = random_flag;
        }
       // send_message("Random message\n");
        
        /**
         * Po tadial vypisi mozte dat do prec
        */
	   	
		if (val.log.button_start && (curr_time - prev_time)/1000 > 1) { // capped at 1 kHz
			
			//printf("ENC1 = %llu, ENC2 = %llu\n", val.enc.encoder1, val.enc.encoder2); 
			//set_speed_dir(150,150); // 20 min____150 max
			//motor_update_current_speed(&val.enc, &motor_speed_left, &motor_speed_right);
			

			//calculate_odometry(&val.enc,&position);

			if((double)(curr_time - printTime)/1000.0 > 100.0){
				//printf("Pos X: %1.2lf, Pos Y: %1.2lf, Theta: %1.2lf\n", position.x, position.y, position.theta);
				printf("GYRO: %1.2f\n",val.orient.heading);
				printTime = curr_time;
			}
			
			// sprintf(message_buff,"%llu, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3lf, %1.3lf", 
			// 					esp_timer_get_time(), val.tof.tof1, val.tof.tof2, val.tof.tof3, val.tof.tof4, val.orient.heading, motor_speed_left, motor_speed_right);
			
			// send_message(message_buff);
			// memset(message_buff,'\0', MESSAGE_BUFF_LEN);
			prev_time = curr_time;
		}
		// vTaskDelay(100 / portTICK_PERIOD_MS);
		// printf("PWM: %d\n",pwm);
	}

	deinit_pid(pid_left);
	deinit_pid(pid_right);
}
