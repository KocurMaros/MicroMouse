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

#define TOF_MAX 40 // The sensor distance limit[cm]
#define TURN_LEFT_SWITCH 0.33
#define TURN_LEFT_THRESH 0.23
#define FROM_LEFT_TO_STRAIGHT_THRESH 0.15


#define MAX_SPEED 100

typedef enum Wall_dir_t{
    WALL_LEFT = 0,
    WALL_RIGHT,
    NONE
}Wall_dir_t;


PID *controller;
bool turnLeft = false;

double control_braitenberg_fear(const MeasData *_current_sensor_data, int *speed_left_, int *speed_right_){
    
    //2 33, 3 13, 1 & 4 25

    //2 23,3 13, 1 & 4 18

    //2 81, 3 10, 1 29, 4 13
    
    // rovno

// START_TURN_LEFT:
//     if(turnLeft)
//     {
//         *speed_left_ = 50;
//         *speed_right_ = 100;

//         if(_current_sensor_data->tof.tof2 < FROM_LEFT_TO_STRAIGHT_THRESH)
//             turnLeft = false;

//         return;
//     }

    if(_current_sensor_data->tof.tof2 > TURN_LEFT_THRESH && _current_sensor_data->tof.tof3 < TOF_3_DESIRED_MAX)
    {
        *speed_left_ = 100;
        *speed_right_ = 100;

        return;
    }
    // else if (_current_sensor_data->tof.tof2 <= TURN_LEFT_THRESH && _current_sensor_data->tof.tof3 < TOF_3_DESIRED_MAX)
    // {
    //     turnLeft = true;
    //     goto START_TURN_LEFT;
    // }
    
    
    
    
    double left_error = _current_sensor_data->tof.tof2 - TOF_2_REFERENCE,
           right_error = _current_sensor_data->tof.tof3 - TOF_3_REFERENCE;
    
    double err = left_error - right_error;
    //double right_error = _current_sensor_data->tof.tof3 - TOF_3_REFERENCE;

    double control_signal_left = pid_control_from_error_d(controller, err);

    //control_sig -= left_error;
    if (control_signal_left > 0.1) {
        *speed_left_ -= abs(control_signal_left);
    }
    else if (control_signal_left < -0.1) {
        *speed_right_ -= abs(control_signal_left);
    }

    return control_signal_left;
}

void init_controller()
{
    controller = init_pid(1500, 1, 0, -MAX_SPEED, MAX_SPEED);
}
