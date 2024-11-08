#ifndef MEAS_DATA_H
#define MEAS_DATA_H

#include <stdint.h>

typedef struct values_tof
{
	float tof1;
	float tof2;
	float tof3;
	float tof4;
} values_tof;
typedef struct orientation
{
	float roll;
	float pitch;
	float heading;
} orientation;

/// @brief Encoder data, where encoder 1/2 are ticks and time_diff is the time difference in us
typedef struct encoders
{
	int32_t encoder1;
	int32_t encoder2;
    int64_t time_diff;
} encoders;
typedef struct logging
{
	float voltage;
	bool button_start;
    int hz_gyro;
    float gyro_freq;
    
} logging;
typedef struct MeasData
{
	values_tof tof;
	orientation orient;
	encoders enc;
	logging log;
} MeasData;
#endif