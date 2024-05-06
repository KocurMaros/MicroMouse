#include <string.h>

#include "../../main/main.h"

#include "../../build/config/sdkconfig.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "meas_data.h"

extern "C" {
    #include "driver.h"
}

static const char *TAG = "task_control.c";

static MeasData val;
QueueHandle_t FIFO_Acq_to_Comm;

extern "C" void task_control(void *arg)
{

    uint32_t ulNotifiedValue;

    uint32_t pwm = 0;
    bool up = true;
    init_motor_driver();
    int64_t prev_time = esp_timer_get_time();
    for(;;)
    {
        // if(!up){
        //     pwm-=100;
        //     if(pwm == 0)
        //         up = true;
        // }
        if(up){
            pwm+=10;
            if(pwm == 1100)
                up = false;
        }
        // if((esp_timer_get_time() - prev_time) > 1000000){
        //     prev_time = esp_timer_get_time();
        //     printf("Encoder 1 A: %lld\n",interrupts[0]);
        //     printf("Encoder 1 B: %lld\n",interrupts[1]);
        //     printf("Encoder 2 A: %lld\n",interrupts[2]);
        //     printf("Encoder 2 B: %lld\n",interrupts[3]);
        //     interrupts[0] = 0;
        //     interrupts[1] = 0;
        //     interrupts[2] = 0;
        //     interrupts[3] = 0;
        // }
        set_speed_dir(0,pwm,1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        printf("PWM: %d\n",pwm);
    }
}
