#ifndef _HC05_H_
#define _HC05_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "usart.h"

void HC05_Printf(const char *format, ...);

/*
 * 非阻塞调试输出。
 * 如果上一次数据还没发送完成，本次数据直接丢弃，
 * 不阻塞电机控制。
 */
void HC05_Printf_IT(const char *format, ...);

#endif
