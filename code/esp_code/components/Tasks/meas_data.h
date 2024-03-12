#ifndef MEAS_DATA_H
#define MEAS_DATA_H


typedef struct values_tof {
    float tof1;
    float tof2;
    float tof3;
    float tof4;
} values_tof;
typedef struct values_gyro {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} values_gyro;
typedef struct values_accel {
    float accel_x;
    float accel_y;
    float accel_z;
} values_accel;
typedef struct MeasData {
    values_tof tof;
    values_gyro gyro;
    values_accel accel;
} MeasData;
#endif