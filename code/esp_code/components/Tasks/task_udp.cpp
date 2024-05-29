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
    #include "control.h"
}
#define MESSAGE_BUFF_LEN 512

static const char *TAG = "task_udp.c";

static MeasData val;
static char message_buff[MESSAGE_BUFF_LEN];
static Position position;

extern "C" void task_udp(void *arg)
{
	printf("Task control run on core: %d\n", xPortGetCoreID());
	memset(message_buff,'\0', MESSAGE_BUFF_LEN);
    
    int64_t core_time = 0;
    int64_t send_time = 0;
    init_motor_driver();

    uint32_t ulNotifiedValue;
    for (;;) {
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
		if (ulNotifiedValue == COMM_OK) {
			(void)xQueueReceive(FIFO_Meas_to_Cont, &val, (100/portTICK_PERIOD_MS)); // wait longer than 10 ms 
            
            set_speed_dir(100, 100); // 20 min____150 max
            calculate_odometry(&val.enc, &position, &val.orient);
        }
        core_time = esp_timer_get_time();
        if(core_time - send_time > 100'000){
            sprintf(message_buff,"%llu, %1.3f, %1.3f, %1.3f, %1.3f, "
                                 "%1.3f, %1.3lf, %1.3lf, %1.3f, %1.3f, %1.3lf, %1.3lf", 
								esp_timer_get_time(), val.tof.tof1, val.tof.tof2, val.tof.tof3, val.tof.tof4, 
                                val.orient.heading, get_pid_left_feedback(), get_pid_right_feedback(), val.log.voltage, val.log.gyro_freq, position.x, position.y);
			// printf("Message: %s\n", message_buff);
            // printf("It took: %d\n", val.log.hz_gyro);
            // printf("Voltage: %f\n", val.log.voltage);
            // printf("Freq gyro: %f\n", val.log.gyro_freq);
			send_message(message_buff);
			memset(message_buff,'\0', MESSAGE_BUFF_LEN);
            send_time = core_time;
        }
	}
    deinit_pid(pid_left);
	deinit_pid(pid_right);
}
