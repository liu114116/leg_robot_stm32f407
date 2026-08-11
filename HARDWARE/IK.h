#ifndef _IK_H_
#define _IK_H_

#include <stdint.h>
#include <math.h>


#define RAD2DEG (180.0f / PI)
#define DEG2RAD (PI / 180.0f)

#define SERVO_CENTER 2048
#define SERVO_RANGE 1952.0f
#define ANGLE_MAX 171.0f
#define L 60.0f

// 逆运动学：输入末端位置 (x, y) mm，输出舵机数值
// 返回值：0=成功，-1=超出机械臂范围，-2=无有效角度解
int inverse_kinematics(float x, float y, uint16_t *a_servo, uint16_t *b_servo);

#endif
