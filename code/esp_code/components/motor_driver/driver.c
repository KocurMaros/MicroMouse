#include "include/driver.h"

#include "../../build/config/sdkconfig.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "motor_pinout.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MAX (1023)			// 2 ** 13 -1
#define LEDC_FREQUENCY (8000)			// Frequency in Hertz. Set frequency at 5 kHz

PID *pid_left;
PID *pid_right;

void pwm_init(uint8_t pin, ledc_channel_t channel)
{
	ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_MODE,
									   .timer_num = LEDC_TIMER,
									   .duty_resolution = LEDC_DUTY_RES,
									   .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
									   .clk_cfg = LEDC_AUTO_CLK };
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	// Prepare and then apply the LEDC PWM channel configuration
	ledc_channel_config_t ledc_channel = { .gpio_num = pin,
										   .speed_mode = LEDC_MODE,
										   .channel = channel,
										   .intr_type = LEDC_INTR_DISABLE,
										   .timer_sel = LEDC_TIMER,
										   .duty = 0, // Set duty to 0%
										   .hpoint = 0 };
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, 0));
	// Update duty to apply the new value
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
	// printf("PIN %d channel %d \n",pin,channel);
}

void pwm_change_duty_raw(ledc_channel_t channel, uint32_t pwm)
{
	if (pwm > LEDC_DUTY_MAX)
		pwm = LEDC_DUTY_MAX;

	ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, pwm));
	// Update duty to apply the new value
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
}

void init_motor_driver()
{
	gpio_set_direction(MOTOR_A_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(MOTOR_A_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(MOTOR_B_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(MOTOR_B_2, GPIO_MODE_OUTPUT);
	pwm_init(MOTOR_A_PWM, MOTOR_A_PWM_CHANNEL);
	pwm_init(MOTOR_B_PWM, MOTOR_B_PWM_CHANNEL);

	pid_left = init_pid(1, 0, 0, 1023);
	pid_right = init_pid(1, 0, 0, 1023);
}

void move_forward()
{
	gpio_set_level(MOTOR_A_1, 1);
	gpio_set_level(MOTOR_A_2, 0);
	gpio_set_level(MOTOR_B_1, 0);
	gpio_set_level(MOTOR_B_2, 1);
}

void move_backward()
{
	gpio_set_level(MOTOR_A_1, 0);
	gpio_set_level(MOTOR_A_2, 1);
	gpio_set_level(MOTOR_B_1, 1);
	gpio_set_level(MOTOR_B_2, 0);
}

void move_left()
{
	gpio_set_level(MOTOR_A_1, 1);
	gpio_set_level(MOTOR_A_2, 0);
	gpio_set_level(MOTOR_B_1, 1);
	gpio_set_level(MOTOR_B_2, 0);
}

void move_right()
{
	gpio_set_level(MOTOR_A_1, 0);
	gpio_set_level(MOTOR_A_2, 1);
	gpio_set_level(MOTOR_B_1, 0);
	gpio_set_level(MOTOR_B_2, 1);
}

void set_speed_dir(double speed_left, double speed_right)
{
	Direction dir;
	double diff = speed_left - speed_right;

	if (diff < -0.1) {
		dir = Left;
		move_left();
	}
	else if (diff > 0.1) {
		dir = Right;
		move_right();
	}
	else if (speed_left < 0) {
		dir = Backwared;
		move_backward();
	}
	else {
		dir = Forward;
		move_forward();
	}

	double pwm_left = pid_control(pid_left, 0, true);
	double pwm_right = pid_control(pid_right, 0, true);

	pwm_change_duty_raw(MOTOR_A_PWM_CHANNEL, pwm_right);
	pwm_change_duty_raw(MOTOR_B_PWM_CHANNEL, pwm_left);
}

PID *init_pid(double kp, double ki, double kd, double limit)
{
	PID *pid = (PID*)calloc(1, sizeof(PID));

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral = 0;
	pid->last_error = 0;
	pid->goal = 0;
	pid->virginOutput = 0;
	pid->clampedOutput = 0;
	pid->limit = limit;

	return pid;
}

void deinit_pid(PID *pid)
{
	free(pid);
}

void set_goal(PID *pid, double goal)
{
	pid->goal = goal;
}

double pid_control(PID *pid, double feedback, bool limit)
{
	double error = pid->goal - feedback;

	return pid_control_from_error(pid, error, limit);
}

double pid_control_from_error(PID *pid, double error, bool limit)
{
	pid->integral += error;
	pid->integral += pid->clampedOutput - pid->virginOutput;

	double derivative = error - pid->last_error;

	pid->virginOutput = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

	if (limit) {
		if (pid->virginOutput < 0) {
			pid->clampedOutput = (pid->virginOutput < -pid->limit ? pid->limit : (pid->virginOutput > -15 ? -15. : pid->virginOutput));
		}
		else {
			pid->clampedOutput = (pid->virginOutput > pid->limit ? pid->limit : (pid->virginOutput < 15 ? 15. : pid->virginOutput));
		}
	}

	pid->last_error = (limit ? pid->clampedOutput : pid->virginOutput);

	return (limit ? pid->clampedOutput : pid->virginOutput);
}

