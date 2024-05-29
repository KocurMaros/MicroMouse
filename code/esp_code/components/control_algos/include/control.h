#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include "meas_data.h"

/// @brief A fear behaviour version of the Brightenberg vehicle
/// @param _current_tof_data [in] current distance from the onboard TOF sensors [m]
/// @param speed_left_ [out] the calculated speed for the left motor [mm/s]
/// @param speed_right_ [out] the calculated speed for the right motor [mm/s]
double control_braitenberg_fear(const MeasData *_current_tof_data, int *speed_left_, int *speed_right_);

void init_controller();

#endif // CONTROL_H
