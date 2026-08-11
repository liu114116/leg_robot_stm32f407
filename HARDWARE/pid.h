#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"

/******************************************增量式PI*********************************************************/

typedef struct
{
    float Kp;
    float Ki;
    int min_output;
    int max_output;
    int control;
    int last_bias;
    float increment_remainder;
} PI_t;


int PI_Control(PI_t *pi,int target,int current);

void PI_Reset(PI_t *pi);

/******************************************增量式PI*********************************************************/

#endif