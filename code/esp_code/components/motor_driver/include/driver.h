#ifndef DRIVER_H
#define DRIVER_H

#include <stdbool.h>
#include "meas_data.h"

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
 * exception is the @c reference parameter. This parameter can be read directly.
 */
typedef struct picontrol {
	double kp;
	double ki;
	double kd;
	double integral;
	double last_error;

	// Reference value for the PID controller mm/s.
	double reference;

	// Speed of the wheel in mm/s.
	double feedback;
	double virginOutput;
	double clampedOutput;
	double limit;
} PID;

typedef struct position {
	double x;
	double y;
	double theta;
} Position;

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
 * @param speed_left [in] mm/s.
 * @param speed_right [in] mm/s.
 */
void set_speed_dir(int speed_left, int speed_right);

/**
 * @brief Initialize a PID controller.
 *
 * @param kp [in] Proportional gain.
 * @param ki [in] Integral gain.
 * @param kd [in] Derivative gain.
 * @param limit [in] The limit of the output.
 *
 * @return The PID controller @c PID.
 */
PID *init_pid(double kp, double ki, double kd, double limit);

/**
 * @brief Deinitialize a PID controller.
 *
 * @param pid [in] The PID controller to deinitialize.
 */
void deinit_pid(PID *pid);

/**
 * @brief Update the feedback value of the PID controller.
 *
 * @param pid [in] The PID controller.
 * @param feedback [in] The new feedback value.
 */
void pid_update_feedback(PID *pid, double feedback);

/**
 * @brief Generate the next control signal value for the motor.
 *
 * @see pid_control_from_error
 *
 * @param pid [in] The PID controller.
 * @param reference [in] The current setpoint [mm/s].
 *
 * @return The output of the PID controller [PWM].
 */
uint16_t pid_control(PID *pid, double reference);

/**
 * @brief Generate the next control signal value for the motor.
 *
 * @see pid_control
 *
 * @param pid [in] The PID controller.
 * @param error [in] The difference between the reference value and current output value [PWM].
 *
 * @return The output of the PID controller [PWM].
 */
uint16_t pid_control_from_error(PID *pid, double error);

/**
 * @brief Calculate the current speed of the whees in mm/s.
 *
 * The updated values are written to the PID controller's feedback values.
 *
 * @param enc [in] Encoder structure containting the current encoder values.
 * @param left [out] The current speed of the left wheel in mm/s.
 * @param right [out] The current speed of the right wheel in mm/s.
 */
void motor_update_current_speed(const encoders *enc, double *left, double *right);

/**
 * @brief Calculate the odometry of the robot.
 *
 * The updated values are written to the Position structure.
 *
 * @param enc [in] Encoder structure containting the current encoder values.
 * @param pos [out] The current position of the robot.
 */
void calculate_odometry(const encoders *enc, Position *pos);

#endif // DRIVER_H
