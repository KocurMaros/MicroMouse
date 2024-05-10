#ifndef DRIVER_H
#define DRIVER_H

#include <stdbool.h>

typedef enum Direction {
	Forward,
	Backwared,
	Left,
	Right,
} Direction;

/**
 * @brief PID controller structure.
 *
 * WARN: Do not use the parameters directly. Use the functions provided. The only
 * exception is the @c goal parameter. This parameter can be read directly.
 */
typedef struct picontrol {
	double kp;
	double ki;
	double kd;
	double integral;
	double last_error;
	double goal;
	double virginOutput;
	double clampedOutput;
	double limit;
} PID;

extern PID *pid_left;
extern PID *pid_right;

/**
 * @brief Setup the motor driver.
 *
 * Initializes the pins and the pwm channels.
 */
void init_motor_driver();

/**
 * @brief Set the speed and direction of the motors.
 *
 * d = 2.4cm
 * 7,539822369 cm is the distance traveled per rotation.
 * Gear is 8:32.
 *
 * @param speed_left PWM value 0-1023 included.
 * @param speed_right PWM value 0-1023 included.
 */
void set_speed_dir(double speed_left, double speed_right);

/**
 * @brief Initialize a PID controller.
 *
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param limit The limit of the output.
 *
 * @return The PID controller @c PID.
 */
PID *init_pid(double kp, double ki, double kd, double limit);

/**
 * @brief Deinitialize a PID controller.
 *
 * @param pid The PID controller to deinitialize.
 */
void deinit_pid(PID *pid);

/**
 * @brief Set the goal of the PID controller.
 *
 * @param pid The PID controller.
 * @param goal The goal of the PID controller.
 */
void pid_set_goal(PID *pid, double goal);

/**
 * @brief Generate the next control signal value for the motor.
 *
 * @see pid_control_from_error
 *
 * @param pid The PID controller.
 * @param feedback The current output value.
 * @param limit If the output should be limited.
 *
 * @return The output of the PID controller.
 */
double pid_control(PID *pid, double feedback, bool limit);

/**
 * @brief Generate the next control signal value for the motor.
 *
 * @see pid_control
 *
 * @param pid The PID controller.
 * @param error The difference between the reference value and current output value.
 * @param limit If the output should be limited.
 *
 * @return The output of the PID controller.
 */
double pid_control_from_error(PID *pid, double error, bool limit);

#endif // DRIVER_H
