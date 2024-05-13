#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define MOTOR_A_1 GPIO_NUM_19
#define MOTOR_A_2 GPIO_NUM_18
#define MOTOR_A_PWM GPIO_NUM_17
#define MOTOR_B_1 GPIO_NUM_16
#define MOTOR_B_2 GPIO_NUM_4
#define MOTOR_B_PWM GPIO_NUM_2
#define MOTOR_A_PWM_CHANNEL (ledc_channel_t)0
#define MOTOR_B_PWM_CHANNEL (ledc_channel_t)1

#define ENCODER_1_A GPIO_NUM_34
#define ENCODER_1_B GPIO_NUM_35
#define ENCODER_2_A GPIO_NUM_32
#define ENCODER_2_B GPIO_NUM_33

#endif