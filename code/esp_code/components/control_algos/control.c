#include "include/control.h"

#include "../../build/config/sdkconfig.h"
// #include "../include/driver.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include <math.h>

#define TOF1_ANGLE 100 //[deg], from a horizontal line (i.e wheel to wheel)
#define TOF2_ANGLE 315
#define TOF3_ANGLE 45
#define TOF4_ANGLE 260



void control_brightenberg_fear(const values_tof *_current_tof_data, int *speed_left_, int *speed_right_){

}