#include "include/control.h"

#include "../../build/config/sdkconfig.h"
// #include "../include/driver.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include <math.h>

/////////////////////// TOF ANGLES //////////////////////////// 
#define TOF1_ANGLE 10 //was 100 //[deg], from a horizontal line (i.e wheel to wheel)
#define TOF2_ANGLE 45
#define TOF3_ANGLE -45
#define TOF4_ANGLE -10

#define TOF_2_DESIRED_MIN 11 //[cm]
#define TOF_2_DESIRED_MAX 13
#define TOF_3_DESIRED_MIN 12
#define TOF_3_DESIRED_MAX 14

#define TOF_MAX 40 // The sensor distance limit[cm]

typedef enum Wall_dir_t{
    WALL_LEFT = 0,
    WALL_RIGHT,
    NONE
}Wall_dir_t;

void control_braitenberg_fear(const MeasData *_current_sensor_data, int *speed_left_, int *speed_right_){
    Wall_dir_t currrentWall = NONE;

    if(_current_sensor_data->tof.tof1 * 100.0 <= TOF_MAX)
        currrentWall = WALL_LEFT;
    else if(_current_sensor_data->tof.tof4 * 100 <= TOF_MAX)
    {
        currrentWall = WALL_RIGHT;
    }
    
    if (currrentWall == NONE)
    {
        printf("Cant find a wall, so I stop, Find me one!\n");
        *speed_left_ = 0;
        *speed_right_ = 0;
        return;
    }
}