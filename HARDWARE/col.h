#ifndef _COL_H_
#define _COL_H_

#include "stm32f4xx_hal.h"

#include "led.h"
#include "servo.h"
#include "hc05.h"
#include "foc.h"
#include "AS5600.h"
#include "IK.h"
#include "protocol.h"

/*
 * One-time setup:
 *   - start protocol first (USART1 interrupt RX, non-blocking)
 *   - init STS servos (non-blocking)
 *   - init 4 wheel motors (FOC, blocking align)
 *
 * Servo torque-enable is deferred to col_update() so boot never stalls.
 */
void col_init(void);

/*
 * Called every main loop iteration.
 *   - sweep servo torque-enable (non-blocking)
 *   - run wheel speed loop at fixed period
 *   - apply new chassis command to legs (IK + servo)
 *   - send state frames periodically (also drives loopback test)
 *   - blink a different LED per received data type
 */
void col_update(uint32_t now_tick);

/*
 * One-shot wiring checks (call from main() after col_init()).
 * Both print results over USART1 (HC05_Printf).
 *
 *   col_test_servo_map(): ping servos 1..8 and report which reply.
 *   col_test_motor_map(): spin motors 1..4 open-loop ~1.5s each.
 */
void col_test_servo_map(void);
void col_test_motor_map(void);

#endif
