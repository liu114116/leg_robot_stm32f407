#ifndef _COL_H_
#define _COL_H_

#include "stm32f4xx_hal.h"

#include "led.h"
#include "servo.h"
#include "hc05.h"
#include "foc.h"
#include "AS5600.h"
#include "IK.h"


/*
 * 腿部舵机初始化。
 */
void Servo_init(void);

/*
 * 原来的control()改名为leg_pos_col()。
 *
 * 函数内部逻辑保持不变。
 */
void leg_pos_col(void);

#endif