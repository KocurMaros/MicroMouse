#ifndef DRIVER_H
#define DRIVER_H

typedef enum Direction {
	Forward,
	Backwared,
	Left,
	Right,
} Direction;

/**
 * @brief Setup the motor driver.
 *
 * Initializes the pins and the pwm channels.
 */
void init_motor_driver();

/**
 * @brief Set the speed and direction of the motors.
 *
 * 7,539822369 is the distance traveled per rotation.
 * Gear is 32/8.
 *
 * @param dir Direction direction of the movement.
 * @param speed PWM value 0-1023 included.
 */
void set_speed_dir(Direction dir, int speed);

#endif // DRIVER_H
