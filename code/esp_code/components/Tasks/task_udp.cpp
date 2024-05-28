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
}
#define MESSAGE_BUFF_LEN 512

static const char *TAG = "task_udp.c";

static MeasData val;
static char message_buff[MESSAGE_BUFF_LEN];

extern "C" void task_udp(void *arg)
{
	printf("Task control run on core: %d\n", xPortGetCoreID());
	memset(message_buff,'\0', MESSAGE_BUFF_LEN);
    
    
    uint64_t prev_random_flag = 0;
    for (;;) {
		/**
         * TOTO tu nechajte
        */
		if (prev_random_flag < random_flag) {
			(void)xQueueReceive(FIFO_Meas_to_Cont, &val, (100/portTICK_PERIOD_MS)); // wait longer than 10 ms 
			// printf("Queue Recieve: %s\n", qRec == pdTRUE ? "OK" : "ERROR");
			sprintf(message_buff,"%llu, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, ", 
								esp_timer_get_time(), val.tof.tof1, val.tof.tof2, val.tof.tof3, val.tof.tof4, val.orient.heading);
			
			send_message(message_buff);
			memset(message_buff,'\0', MESSAGE_BUFF_LEN);
		    prev_random_flag = random_flag;
        }
        if(prev_random_flag > random_flag){
            printf("Error: Random flag is smaller than previous\n");
            prev_random_flag = random_flag;
        }
     
	}
}
