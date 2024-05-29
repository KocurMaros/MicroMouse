#include "include/driver.h"

#include "../../build/config/sdkconfig.h"
#include "../include/driver.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "motor_pinout.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include <math.h>

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MAX (1023)			// 2 ** 13 -1
#define LEDC_FREQUENCY (8000)			// Frequency in Hertz. Set frequency at 5 kHz

#define WHEEL_DIAMETER 24				// 24 mm
#define IMPULZS_PER_ROTATION 4096		// 4096 impulzes per rotation
#define TRACK_WIDTH 94.5				// 915 mm
#define PI 3.14159265359
#define GEAR_RATIO 4					// The motor does 4 rotations per one wheel rotation.
#define MAX_MOTOR_RPM 11000				// The max unloaded RPM of the motor

#define MAX_WHEEL_RPM MAX_MOTOR_RPM / GEAR_RATIO
#define MAX_WHEEL_SPEED ((MAX_WHEEL_RPM * PI * WHEEL_DIAMETER) / 60.0)

#define TO_MM_PER_SECOND(encoder_impulzes, time_s) ((encoder_impulzes / (time_s) /  (IMPULZS_PER_ROTATION * GEAR_RATIO) * ((PI * WHEEL_DIAMETER))))
#define TO_PWM_FROM_MM_PER_SECOND(speed_mm_s) ((speed_mm_s) / MAX_WHEEL_SPEED * 1023)
#define TO_M_FROM_MM(mms) ((mms) / 1000.0)

PID *pid_left;
PID *pid_right;
Direction dir;

void cap_pwm(int *currentPwm)
{
	*currentPwm = (*currentPwm < -1023 ? -1023 : *currentPwm > 1023 ? 1023 : *currentPwm);
}

void pwm_init(uint8_t pin, ledc_channel_t channel)
{
	ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_MODE,
									   .duty_resolution = LEDC_DUTY_RES,
									   .timer_num = LEDC_TIMER,
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

	pid_left = init_pid(10, 0.03, 0, 1023);
	pid_right = init_pid(10, 0.03, 0, 1023); //cc timo chod dopice pls dik
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

const char *direction_to_string(Direction dir)
{
	switch (dir) {
	case Forward:
		return "Forward";
	case Backward:
		return "Backward";
	case Left:
		return "Left";
	case Right:
		return "Right";
	default:
		return "Unknown";
	}
}

void set_speed_dir(int speed_left, int speed_right)
{
	if (speed_left < 0 && speed_right > 0) {
		dir = Left;
		move_left();
	}
	else if (speed_left > 0 && speed_right < 0) {
		dir = Right;
		move_right();
	}
	else if (speed_left < 0 && speed_right < 0) {
		dir = Backward;
		move_backward();
	}
	else {
		dir = Forward;
		move_forward();
	}
	//move_forward();
	
	pwm_change_duty_raw(MOTOR_A_PWM_CHANNEL, pid_control(pid_left, speed_left));
	pwm_change_duty_raw(MOTOR_B_PWM_CHANNEL, pid_control(pid_right, speed_right));
}

PID *init_pid(double kp, double ki, double kd, double limit)
{
	PID *pid = (PID*)calloc(1, sizeof(PID));

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral = 0;
	pid->last_error = 0;
	pid->feedback = 0;
	pid->reference = 0;
	pid->virginOutput = 0;
	pid->clampedOutput = 0;
	pid->limit = limit;

	return pid;
}

void deinit_pid(PID *pid)
{
	free(pid);
}

uint16_t pid_control(PID *pid, double reference)
{
	static uint64_t start_time = 0;
	
	reference = reference < 0 ? -reference : reference;
	
	reference = TO_PWM_FROM_MM_PER_SECOND(reference);
	reference = reference > pid->limit ? pid->limit : (reference < -pid->limit ? -pid->limit : reference);

	double tmp = TO_PWM_FROM_MM_PER_SECOND(pid->feedback);
	tmp = tmp> pid->limit ? pid->limit : (tmp< -pid->limit ? -pid->limit : tmp);

	double error = reference - tmp;
	//printf("Reference: %1.2lf \t Feedback: %1.2lf \t ERROR = %1.2lf\n",reference,  pid->feedback, error);
	if ((double)(esp_timer_get_time() - start_time) / 1000.0 > 100.0)
	{
		//printf("ERROR: %1.5lf, SPEED: %1.5lf\n", error, pid->feedback);
		start_time = esp_timer_get_time();
	}
	
	return pid_control_from_error(pid, error);
}

uint16_t pid_control_from_error(PID *pid, double error)
{
	pid->integral += error;

	double derivative = error - pid->last_error;

	double tmp = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
	pid->virginOutput = tmp;
	pid->clampedOutput = pid->virginOutput < 0 ? 0 : (pid->virginOutput > pid->limit ? pid->limit : pid->virginOutput);
	//printf("PID_TMP: %1.2lf \t PID OUT: %1.2lf \t PID_ACTUAL: %1.2lf \t ERROR: %1.2lf\n", tmp, pid->virginOutput, pid->clampedOutput, error);

	pid->last_error = error;

	//printf("Calculated AZ: %lf\n", pid->clampedOutput);
	return (uint16_t)round(pid->clampedOutput);
}

void motor_update_current_speed(const encoders *enc, double *left, double *right)
{
	double dt = (double)enc->time_diff / 1000000.0;
	double  left_speed = (TO_MM_PER_SECOND(enc->encoder1, dt)), 
		    right_speed = TO_MM_PER_SECOND(enc->encoder2, dt);

	left_speed = left_speed < 0 ? -left_speed : left_speed;
	right_speed = right_speed < 0 ? -right_speed : right_speed;
	//printf("LEFT_SPD = %1.2lf, RIGHT_SPD = %1.2lf , dt = %lf\n", left_speed, right_speed, dt);
    //printf("encoder1: %d, encoder2: %d\n", enc->encoder1, enc->encoder2);
	
	pid_left->feedback = left_speed;
	pid_right->feedback = right_speed;

	if(left != NULL)
		*left = left_speed;

	if(right != NULL)
		*right = right_speed;
}

static double wrap_angle(double angle)
{
	// Calculate the reminder of deviding two doubles.
	angle = fmod(angle, PI);
	if (angle < -PI) {
		angle += 2.0 * PI;
	}

	return angle;
}

void update_encoders(encoders *enc)
{
	// Update the encoder values
	switch (dir) {
	case Backward:
		enc->encoder1 = enc->encoder1 * -1;
		enc->encoder2 = enc->encoder2 * -1;
		break;
	case Left:
		enc->encoder1 = enc->encoder1;
		enc->encoder2 = enc->encoder2 * -1;
		break;
	case Right:
		enc->encoder1 = enc->encoder1 * -1;
		enc->encoder2 = enc->encoder2;
		break;
	default:
		break;
	}
}

void calculate_odometry(encoders *enc, Position *pos, const orientation *gyro)
{
	double left_speed = 0;
	double right_speed = 0;
	double delta_time_s = enc->time_diff / 1000000.;
	
	update_encoders(enc);

	motor_update_current_speed(enc, &left_speed, &right_speed);

	double left_distance = left_speed * delta_time_s;
	double right_distance = right_speed * delta_time_s;

	// Calculate the distance the robot has moved [mm]
	double distance = (left_distance + right_distance) / 2.;
	//sprintf("LEFT_DIST = %1.2lf, RIGHT_DIST = %1.2lf, DIST = %1.2lf, DIR: %s\n", left_distance, right_distance, distance, direction_to_string(dir));

	// Calculate the angle the robot has turned
	//todo: calculate angle from gyro cuz of the skid
	double angle = gyro->heading;

	// Calculate the new position of the robot
	pos->theta = wrap_angle(pos->theta + angle);
	pos->x = pos->x + distance /** cos(pos->theta)*/;
	pos->y = pos->y + distance /** sin(pos->theta)*/;
}

double get_pid_left_feedback()
{
	return pid_left->feedback;
}

double get_pid_right_feedback()
{
	return pid_right->feedback;
}