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

static const char *TAG = "task_control.c";

static MeasData val;
static Position position;

extern "C" void task_control(void *arg)
{
	printf("Task control run on core: %d\n", xPortGetCoreID());

	uint32_t pwm = 0;
	bool up = true;
	init_motor_driver();


	uint64_t prev_random_flag = 0;
	for (;;) {
		int64_t prev_time = esp_timer_get_time();
		/**
         * TOTO tu nechajte
        */
		if (prev_random_flag < random_flag) {
			xQueueReceive(FIFO_Meas_to_Cont, &val, (100 / portTICK_PERIOD_MS));
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
		if (val.log.button_start) {
			set_speed_dir(10,10);
			motor_update_current_speed(&val.enc,(double)(esp_timer_get_time() - prev_time)/1000000.0, NULL, NULL);			
		}
		// vTaskDelay(100 / portTICK_PERIOD_MS);
		// printf("PWM: %d\n",pwm);
	}

	deinit_pid(pid_left);
	deinit_pid(pid_right);
}
