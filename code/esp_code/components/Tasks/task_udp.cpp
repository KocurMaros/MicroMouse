#include <string.h>

#include "../../main/main.h"

#include "../../build/config/sdkconfig.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "meas_data.h"
#include <math.h>
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

int speed_left = 100, speed_right = 100;

int cmpfunc (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}
float gyro_clamp(float _gyro_actual){
    if(_gyro_actual > 2*M_PI)
        return (_gyro_actual - 2*M_PI);
    else if(_gyro_actual < 0)
        return (_gyro_actual + 2*M_PI);
    else
        return _gyro_actual;
}
float gyro_ziadane(float _gyro_actual){
    if(_gyro_actual < 0)
        return gyro_clamp(_gyro_actual - M_PI);
    else
        return gyro_clamp(_gyro_actual + M_PI);

}

extern "C" void task_udp(void *arg)
{
	printf("Task control run on core: %d\n", xPortGetCoreID());
	memset(message_buff,'\0', MESSAGE_BUFF_LEN);
    
    int64_t core_time = 0;
    int64_t send_time = 0;
    init_motor_driver();
    init_controller();
    double ret = 0;
    uint32_t ulNotifiedValue;
    
    // uint8_t tof_index = 0;
    // uint8_t tof_itterations = 3;
    // float tof1[tof_itterations] = {0};
    // float tof2[tof_itterations] = {0};
    // float tof3[tof_itterations] = {0};
    // float tof4[tof_itterations] = {0};
    // float temp_tof1[tof_itterations], temp_tof2[tof_itterations], temp_tof3[tof_itterations], temp_tof4[tof_itterations];
    float _gyro_wanted = 0;
    PID *rot_reg = init_pid(30,0,0,-150,150,NULL,NULL);
    int64_t reverse_time = 0;
    for (;;) {
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
        
		if (ulNotifiedValue == COMM_OK) {
			(void)xQueueReceive(FIFO_Meas_to_Cont, &val, (100/portTICK_PERIOD_MS)); // wait longer than 10 ms 
            // tof1[tof_index] = val.tof.tof1;
            // tof2[tof_index] = val.tof.tof2;
            // tof3[tof_index] = val.tof.tof3;
            // tof4[tof_index] = val.tof.tof4;
            // tof_index++;
            // if(tof_index >= tof_itterations)
            //     tof_index = 0;
            // memcpy(temp_tof1, tof1, sizeof(tof1));
            // memcpy(temp_tof2, tof2, sizeof(tof2));
            // memcpy(temp_tof3, tof3, sizeof(tof3));
            // memcpy(temp_tof4, tof4, sizeof(tof4));

            // qsort(temp_tof1, tof_itterations, sizeof(float), cmpfunc);
            // qsort(temp_tof2, tof_itterations, sizeof(float), cmpfunc);
            // qsort(temp_tof3, tof_itterations, sizeof(float), cmpfunc);
            // qsort(temp_tof4, tof_itterations, sizeof(float), cmpfunc);

            // val.tof.tof1 = temp_tof1[tof_itterations/2];
            // val.tof.tof2 = temp_tof2[tof_itterations/2];
            // val.tof.tof3 = temp_tof3[tof_itterations/2];
            // val.tof.tof4 = temp_tof4[tof_itterations/2];

            speed_left = 300;
            speed_right = 300;
            ret = control_braitenberg_fear(&val, &speed_left, &speed_right);
            /*
            printf("Speed left: %d, Speed right: %d\n", speed_left, speed_right);
            if(turnLeft || turnRight){
                if(_gyro_wanted == 0){
                    reverse_time = esp_timer_get_time();
                    _gyro_wanted = gyro_ziadane(val.orient.heading);
                }
                if(esp_timer_get_time() - reverse_time < 200'000){
                    set_speed_dir(-70, -70); 
                    calculate_odometry(&val.enc, &position, &val.orient);
                }else{
                    double cA =  pid_control_from_error(rot_reg, gyro_clamp(_gyro_wanted - val.orient.heading));
                    if(turnLeft){
                        speed_left = abs(cA) > 100 ? cA : 100;
                        speed_right = -(abs(cA) > 100 ? cA : 100);

                    }else{
                        speed_left = -(abs(cA) > 100 ? cA : 100);
                        speed_right = (abs(cA) > 100 ? cA : 100);
                    }
                    printf("Gyro wanted: %f, Gyro actual: %f, Control: %f\n", _gyro_wanted, val.orient.heading, cA);
                    if(abs(_gyro_wanted - val.orient.heading) < 0.1){
                        turnLeft = false;
                        turnRight = false;
                        _gyro_wanted = 0;
                    }
                }
            }
            */
            set_speed_dir(speed_left, speed_right);  
            calculate_odometry(&val.enc, &position, &val.orient);



        }
        core_time = esp_timer_get_time();
        if(core_time - send_time > 100'000){
            sprintf(message_buff,"%llu, %1.3f, %1.3f, %1.3f, %1.3f, "
                                 "%1.3f, %1.3lf, %1.3lf, %1.3f, %1.3f, %1.6lf, %1.6lf, %1.3lf, %d, %d", 
								esp_timer_get_time(), val.tof.tof1, val.tof.tof2, val.tof.tof3, val.tof.tof4, 
                                val.orient.heading, get_pid_left_feedback(), get_pid_right_feedback(), val.log.voltage, val.log.gyro_freq, position.x, position.y, ret, val.enc.encoder1, val.enc.encoder2);
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
