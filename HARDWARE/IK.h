#ifndef _IK_H_
#define _IK_H_

#include <stdint.h>
#include <math.h>
#include "foc.h"


#define RAD2DEG (180.0f / PI)
#define DEG2RAD (PI / 180.0f)

#define SERVO_CENTER 2048
#define SERVO_RANGE 1952.0f
#define ANGLE_MAX 171.0f
#define L 60.0f

// ���˶�ѧ������ĩ��λ�� (x, y) mm����������ֵ
// ����ֵ��0=�ɹ���-1=������е�۷�Χ��-2=����Ч�ǶȽ�
int inverse_kinematics(float x, float y, uint16_t *a_servo, uint16_t *b_servo);

#endif
