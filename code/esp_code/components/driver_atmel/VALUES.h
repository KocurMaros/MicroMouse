#ifndef __VALUES_H__
#define __VALUES_H__
/**
 * @struct task_values
 * 
 */
typedef struct VALUES_Final
{
    double blank_var[6];
    uint32_t a[6];
    uint32_t aPwm_value[6];
} VALUES_Final;

typedef struct PIN_Mode
{
    uint8_t aMode[6];
    bool on_off[6];
}PIN_Mode;

typedef struct VALUES_Meas {
    VALUES_Final Final;
    PIN_Mode Mode;
} VALUES_Meas;

#endif