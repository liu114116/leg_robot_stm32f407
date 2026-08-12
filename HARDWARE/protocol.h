#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "stm32f4xx_hal.h"
#include "usart.h"

#include <stdint.h>

/* ============================================================
 * Frame format (matches Linux hardware_driver)
 *
 *   [0xAA][0x55][cmd][len][payload...][crc8]
 *
 *   - len = payload byte count
 *   - all floats are IEEE754 little-endian
 *   - crc8 = poly 0x07, init 0x00, over "cmd + len + payload"
 * ============================================================ */

#define FRAME_HEAD1          0xAAU
#define FRAME_HEAD2          0x55U
#define FRAME_MAX_PAYLOAD    40U

#define CMD_CHASSIS_CMD      0x01U
#define CMD_LEG_STATE        0x02U
#define CMD_WHEEL_STATE      0x03U
#define CMD_IMU_DATA         0x04U

#define LEN_CHASSIS_CMD      40U
#define LEN_LEG_STATE        17U
#define LEN_WHEEL_STATE      13U
#define LEN_IMU_DATA         12U

/* ============================================================
 * Downlink: chassis command from Linux
 * ============================================================ */

typedef struct
{
    float foot_x[4];
    float foot_z[4];
    float left_wheel;
    float right_wheel;
} ChassisCmd_t;

/*
 * Incremented on every CRC-valid received frame.
 * Used as the LED heartbeat for the loopback test.
 */
extern volatile uint32_t g_rx_valid_count;

/*
 * Command byte of the most recent CRC-valid received frame.
 * Used to blink a different LED per data type.
 */
extern volatile uint8_t  g_rx_last_cmd;

/*
 * Set to 1 when a new ChassisCmd (0x01) is fully parsed.
 * Cleared by the consumer after applying it.
 */
extern volatile uint8_t  g_chassis_cmd_new;

extern ChassisCmd_t      g_chassis_cmd;

/* ============================================================
 * Public API
 * ============================================================ */

/*
 * Reset the RX state machine and start interrupt reception on USART1.
 */
void Protocol_Init(void);

/*
 * CRC8, poly 0x07, init 0x00, no reflection.
 * Verified: "123456789" -> 0xF4.
 */
uint8_t Protocol_Crc8(const uint8_t *data, uint16_t len);

/*
 * Feed one received byte into the RX state machine.
 * Called from HAL_UART_RxCpltCallback.
 */
void Protocol_FeedByte(uint8_t byte);

/*
 * Uplink send functions (non-blocking interrupt transmit on USART1).
 */
void Protocol_SendLegState(uint8_t leg_id, float hip_angle, float knee_angle,
                           float foot_x, float foot_z);

void Protocol_SendWheelState(uint8_t wheel_id, float velocity, float position,
                             float torque);

void Protocol_SendImuData(float roll, float pitch, float yaw);

#endif
