#include <string.h>

#include "../../main/main.h"
#include "../../build/config/sdkconfig.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "meas_data.h"
#include "motor_driver.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MAX           (1023)  // 2 ** 13 -1 
#define LEDC_FREQUENCY          (8000) // Frequency in Hertz. Set frequency at 5 kHz

static const char *TAG = "task_control.c";
void pwm_change_duty_raw(uint8_t channel, uint32_t pwm);
void pwm_init(uint8_t pin, uint8_t channel);

static MeasData val;
QueueHandle_t FIFO_Acq_to_Comm;

void task_control(void *arg)
{
   
    uint32_t ulNotifiedValue;
    
    gpio_set_direction(MOTOR_A_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_A_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_2, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_A_1, 1);
    gpio_set_level(MOTOR_A_2, 0);
    gpio_set_level(MOTOR_B_1, 1);
    gpio_set_level(MOTOR_B_2, 0);

    pwm_init(MOTOR_A_PWM, MOTOR_A_PWM_CHANNEL1);
    pwm_init(MOTOR_B_PWM, MOTOR_B_PWM_CHANNEL1);
    uint32_t pwm = 0;
    bool up = true;
    for(;;)
    {       
        pwm_change_duty_raw(MOTOR_A_PWM_CHANNEL1, pwm);
        pwm_change_duty_raw(MOTOR_B_PWM_CHANNEL1, pwm);
        if(!up){
            pwm--;
            if(pwm == 0)
                up = true;
        }
        else{
            pwm++;
            if(pwm == LEDC_DUTY_MAX)
                up = false;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
        printf("PWM: %d\n",pwm);
    }
}

void pwm_init(uint8_t pin, uint8_t channel){
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = channel,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = pin,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, 0));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
    // printf("PIN %d channel %d \n",pin,channel);
}

void pwm_change_duty_raw(uint8_t channel, uint32_t pwm){
    if(pwm > LEDC_DUTY_MAX)
        pwm = LEDC_DUTY_MAX;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, pwm));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
    
}