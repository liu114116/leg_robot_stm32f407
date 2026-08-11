#include "pid.h"

/******************************************增量式PI*********************************************************/

static int PI_Limit(int value,int min_value,int max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

int PI_Control(PI_t *pi,int target,int current)
{
    if (pi == NULL) return 0;
    int bias =target - current;
    float increment =pi->Ki *(float)(bias - pi->last_bias)+pi->Kp *(float)bias;

    increment +=pi->increment_remainder;
    int increment_int =(int)increment;
		pi->increment_remainder =increment -(float)increment_int;
    pi->control +=increment_int;
    if (pi->control >pi->max_output){
        pi->control =pi->max_output;
        pi->increment_remainder =0.0f;
    }
    if (pi->control <pi->min_output)
    {
        pi->control = pi->min_output;
        pi->increment_remainder = 0.0f;
    }
    pi->last_bias = bias;
    return pi->control;
}

void PI_Reset(PI_t *pi)
{
    if (pi == NULL) return;
    pi->last_bias = 0;
    pi->control = 0;
    pi->increment_remainder = 0.0f;
}

/******************************************增量式PI*********************************************************/
