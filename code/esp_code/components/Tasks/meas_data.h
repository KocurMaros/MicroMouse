#ifndef MEAS_DATA_H
#define MEAS_DATA_H


typedef struct values_tof {
    float tof1;
    float tof2;
    float tof3;
    float tof4;
} values_tof;
typedef struct orientation {
    float roll;
    float pitch;
    float heading;
} orientation;
typedef struct encoders {
    uint64_t encoder1;
    uint64_t encoder2;
    uint64_t encoder3;
    uint64_t encoder4;
} encoders;
typedef struct MeasData {
    values_tof tof;
    orientation orient;
    encoders enc;
} MeasData;
#endif