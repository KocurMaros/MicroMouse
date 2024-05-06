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


extern "C" void task_control(void *arg)
{
    uint32_t ulNotifiedValue;


    uint32_t pwm = 0;
    bool up = true;
    init_motor_driver();
    int64_t prev_time = esp_timer_get_time();
    for(;;)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
        xQueueReceive(FIFO_Meas_to_Cont, &val, (100/portTICK_PERIOD_MS));
        if(ulNotifiedValue == 1)
        {
            printf("Received data from meas\n");
            printf("Roll: %f\n",val.orient.roll);
            printf("Pitch: %f\n",val.orient.pitch);
            printf("Heading: %f\n",val.orient.heading);
        }
        if(up){
            pwm+=10;
            if(pwm == 1100)
                up = false;
        }
        set_speed_dir(0,pwm,1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        printf("PWM: %d\n",pwm);
    }
}
