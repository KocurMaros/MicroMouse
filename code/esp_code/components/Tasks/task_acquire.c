
#include "nvs.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#include "../../main/main.h"
#include "meas_data.h"

static const char *TAG = "task_acquire.c";


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MAX           (8191)  // 2 ** 13 -1 
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 5 kHz


MeasData meas;
QueueHandle_t FIFO_Acq_to_Comm;
TaskHandle_t xTaskCommHandle;

static void IRAM_ATTR gpio_isr_handler(void* arg){

}

void task_acquire(void * arg)
{

    uint8_t pin_example = 15;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = false;
    io_conf.pull_up_en = false;
    io_conf.pin_bit_mask = (1ULL << pin_example);
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin_example, gpio_isr_handler, (void*) &pin_example);

   
    uint64_t cycle_time = esp_timer_get_time();
    uint64_t act_time;
    uint64_t loop_counter = 0;
    for(;;){ 
        act_time = esp_timer_get_time();
        if ( cycle_time > act_time )  // ak pretecie act_time, vyresetuj cycle_time
            cycle_time = act_time;
        else if ((act_time - cycle_time) > 10000000 ) {          

            /*
            * Inotify to send data between tasks
            */   
            // xQueueSend( FIFO_Acq_to_Comm, &meas, 30 / portTICK_PERIOD_MS );
            // xTaskNotify(xTaskCommHandle, ATMEL_COMM_OK, eSetBits);
        }
        loop_counter++;
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
    channel--;
    if(pwm > 8192)
        pwm = 8191;
    // pwm_init(control_pins[channel],channel);
    pwm = 8191 - pwm;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, pwm));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
    
}