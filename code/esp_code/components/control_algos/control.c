#include "include/control.h"

#include "../../build/config/sdkconfig.h"
// #include "../include/driver.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver.h"

#include <math.h>

/////////////////////// TOF ANGLES //////////////////////////// 
#define TOF1_ANGLE 10 //was 100 //[deg], from a horizontal line (i.e wheel to wheel)
#define TOF2_ANGLE 45
#define TOF3_ANGLE -45
#define TOF4_ANGLE -10

#define TOF_2_REFERENCE .12
#define TOF_3_REFERENCE .13

#define TOF_2_DESIRED_MIN .11 //[cm]
#define TOF_2_DESIRED_MAX .13
#define TOF_3_DESIRED_MIN .12
#define TOF_3_DESIRED_MAX .14

#define TOF_MAX .4 // The sensor distance limit[cm]
#define TURN_LEFT_SWITCH 0.33
#define TURN_LEFT_THRESH 0.23
#define FROM_LEFT_TO_STRAIGHT_THRESH 0.15


#define MAX_SPEED 150

typedef enum Wall_dir_t{
    WALL_LEFT = 0,
    WALL_RIGHT,
    NONE
}Wall_dir_t;


PID *controller;
bool turnLeft = false;

static void limit_tof_error(double *tof)
{
    if (*tof > TOF_MAX) {
        *tof = TOF_MAX;
    }
}
void pid_update_params(double P, double I, double D, uint8_t reg_num){
    switch (reg_num){
    case 1:
        controller->kp = P;
        controller->ki = I;
        controller->kd = D;
        break;
    
    default:
        break;
    }
}
double control_braitenberg_fear(const MeasData *_current_sensor_data, int *speed_left_, int *speed_right_){
    
    //2 33, 3 13, 1 & 4 25

    //2 23,3 13, 1 & 4 18

    //2 81, 3 10, 1 29, 4 13
    
    // rovno

    if (_current_sensor_data->tof.tof1 < .05 && _current_sensor_data->tof.tof4 < .05) {
        *speed_left_ = 0;
        *speed_right_ = 0;
        return 0;
    }

    
    double left_error = _current_sensor_data->tof.tof2,
           right_error = _current_sensor_data->tof.tof3;
    
    left_error = left_error > TOF_MAX ? TOF_MAX : left_error;
    right_error = right_error > TOF_MAX ? TOF_MAX : right_error;

    double err = left_error - right_error;
    double control_signal = 0;
    //double right_error = _current_sensor_data->tof.tof3 - TOF_3_REFERENCE;

    if (err >= 0)
    {
        controller->kp = 1000;
        control_signal = pid_control_from_error_d(controller, err);
    }
    else
    {
        controller->kp = 1000;//sada
        control_signal = pid_control_from_error_d(controller, err);
    }
    


    //control_sig -= left_error;
    if (control_signal > 0.1) {
        *speed_left_ -= abs(control_signal);
    }
    else if (control_signal < -0.1) {
        *speed_right_ -= abs(control_signal);
    }

    return control_signal;
}

void init_controller()
{
    controller = init_pid(1000, 0, 0, -MAX_SPEED, MAX_SPEED, NULL, NULL);
}
