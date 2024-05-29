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

#define MAX_SPEED 100

typedef enum Wall_dir_t{
    WALL_LEFT = 0,
    WALL_RIGHT,
    NONE
}Wall_dir_t;


PID *controller;

double control_braitenberg_fear(const MeasData *_current_sensor_data, int *speed_left_, int *speed_right_){
    //Wall_dir_t currrentWall = NONE;
    
    //static double control_sig;
    double left_error = _current_sensor_data->tof.tof2 - TOF_2_REFERENCE;
    //double right_error = _current_sensor_data->tof.tof3 - TOF_3_REFERENCE;

    double control_signal_left = pid_control_from_error_d(controller, left_error);

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
    controller = init_pid(40, 0.01, 0, -MAX_SPEED, MAX_SPEED);
}
