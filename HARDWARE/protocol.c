#include "protocol.h"

#include <string.h>

/* ============================================================
 * Globals
 * ============================================================ */

volatile uint32_t g_rx_valid_count = 0U;
volatile uint8_t  g_rx_last_cmd     = 0U;
volatile uint8_t  g_chassis_cmd_new = 0U;

ChassisCmd_t      g_chassis_cmd;

/* Non-blocking TX: static buffer + busy flag (see Transmit section). */
static uint8_t          tx_frame[FRAME_MAX_PAYLOAD + 5U];
static volatile uint8_t tx_busy = 0U;

/* ============================================================
 * RX state machine (byte by byte, no DMA)
 * ============================================================ */

typedef enum
{
    RX_STATE_HEAD1 = 0,
    RX_STATE_HEAD2,
    RX_STATE_CMD,
    RX_STATE_LEN,
    RX_STATE_PAYLOAD,
    RX_STATE_CRC
} RxState_t;

static volatile RxState_t rx_state = RX_STATE_HEAD1;
static volatile uint8_t   rx_cmd = 0U;
static volatile uint8_t   rx_len = 0U;
static volatile uint8_t   rx_idx = 0U;
static volatile uint8_t   rx_crc = 0U;

static uint8_t rx_payload[FRAME_MAX_PAYLOAD];
static uint8_t rx_byte;

/* ============================================================
 * CRC8
 * ============================================================ */

uint8_t Protocol_Crc8(const uint8_t *data, uint16_t len)
{
    uint8_t  crc = 0x00U;
    uint16_t i;
    uint8_t  bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if (crc & 0x80U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

/* ============================================================
 * Parse helpers
 * ============================================================ */

static void Protocol_ParseChassisCmd(const uint8_t *payload)
{
    uint8_t i;
    uint8_t off = 0U;

    for (i = 0U; i < 4U; i++)
    {
        memcpy(&g_chassis_cmd.foot_x[i], &payload[off], 4U);
        off += 4U;
    }

    for (i = 0U; i < 4U; i++)
    {
        memcpy(&g_chassis_cmd.foot_z[i], &payload[off], 4U);
        off += 4U;
    }

    memcpy(&g_chassis_cmd.left_wheel,  &payload[off], 4U);
    off += 4U;
    memcpy(&g_chassis_cmd.right_wheel, &payload[off], 4U);

    g_chassis_cmd_new = 1U;
}

static void Protocol_ValidateFrame(void)
{
    uint8_t  crc_buf[2U + FRAME_MAX_PAYLOAD];
    uint8_t  calc_crc;
    uint16_t i;

    crc_buf[0] = rx_cmd;
    crc_buf[1] = rx_len;
    for (i = 0U; i < rx_len; i++)
    {
        crc_buf[2U + i] = rx_payload[i];
    }

    calc_crc = Protocol_Crc8(crc_buf, (uint16_t)(2U + rx_len));

    if (calc_crc != rx_crc)
    {
        return;
    }

    g_rx_valid_count++;
    g_rx_last_cmd = rx_cmd;

    if ((rx_cmd == CMD_CHASSIS_CMD) && (rx_len == LEN_CHASSIS_CMD))
    {
        Protocol_ParseChassisCmd(rx_payload);
    }
}

/* ============================================================
 * RX byte feed
 * ============================================================ */

void Protocol_FeedByte(uint8_t byte)
{
    switch (rx_state)
    {
        case RX_STATE_HEAD1:
            if (byte == FRAME_HEAD1)
            {
                rx_state = RX_STATE_HEAD2;
            }
            break;

        case RX_STATE_HEAD2:
            if (byte == FRAME_HEAD2)
            {
                rx_state = RX_STATE_CMD;
            }
            else if (byte != FRAME_HEAD1)
            {
                rx_state = RX_STATE_HEAD1;
            }
            break;

        case RX_STATE_CMD:
            rx_cmd   = byte;
            rx_state = RX_STATE_LEN;
            break;

        case RX_STATE_LEN:
            rx_len = byte;
            rx_idx = 0U;

            if (rx_len == 0U)
            {
                rx_state = RX_STATE_CRC;
            }
            else if (rx_len <= FRAME_MAX_PAYLOAD)
            {
                rx_state = RX_STATE_PAYLOAD;
            }
            else
            {
                rx_state = RX_STATE_HEAD1;
            }
            break;

        case RX_STATE_PAYLOAD:
            rx_payload[rx_idx] = byte;
            rx_idx++;

            if (rx_idx >= rx_len)
            {
                rx_state = RX_STATE_CRC;
            }
            break;

        case RX_STATE_CRC:
            rx_crc = byte;
            Protocol_ValidateFrame();
            rx_state = RX_STATE_HEAD1;
            break;

        default:
            rx_state = RX_STATE_HEAD1;
            break;
    }
}

/* ============================================================
 * HAL UART callbacks (USART1, interrupt RX)
 * ============================================================ */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    Protocol_FeedByte(rx_byte);

    HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        tx_busy = 0U;
    }
}

/* ============================================================
 * Transmit (non-blocking, interrupt-driven)
 *
 * The frame is built into a static buffer so it stays valid while
 * HAL_UART_Transmit_IT drains it in the background. Only one frame
 * is on the wire at a time; a frame is dropped if the previous one
 * has not finished (never happens at the 10 Hz uplink rate).
 * ============================================================ */

static void Protocol_TransmitFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t  calc_crc;
    uint16_t idx = 0U;
    uint16_t i;

    if (tx_busy != 0U)
    {
        return;
    }

    tx_frame[idx++] = FRAME_HEAD1;
    tx_frame[idx++] = FRAME_HEAD2;
    tx_frame[idx++] = cmd;
    tx_frame[idx++] = len;

    for (i = 0U; i < len; i++)
    {
        tx_frame[idx++] = payload[i];
    }

    calc_crc = Protocol_Crc8(&tx_frame[2], (uint16_t)(2U + len));
    tx_frame[idx++] = calc_crc;

    tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart1, tx_frame, idx) != HAL_OK)
    {
        tx_busy = 0U;
    }
}

/* ============================================================
 * Public send functions
 * ============================================================ */

void Protocol_SendLegState(uint8_t leg_id, float hip_angle, float knee_angle,
                           float foot_x, float foot_z)
{
    uint8_t payload[LEN_LEG_STATE];

    payload[0] = leg_id;
    memcpy(&payload[1],  &hip_angle,  4U);
    memcpy(&payload[5],  &knee_angle, 4U);
    memcpy(&payload[9],  &foot_x,     4U);
    memcpy(&payload[13], &foot_z,     4U);

    Protocol_TransmitFrame(CMD_LEG_STATE, payload, LEN_LEG_STATE);
}

void Protocol_SendWheelState(uint8_t wheel_id, float velocity, float position,
                             float torque)
{
    uint8_t payload[LEN_WHEEL_STATE];

    payload[0] = wheel_id;
    memcpy(&payload[1], &velocity, 4U);
    memcpy(&payload[5], &position, 4U);
    memcpy(&payload[9], &torque,  4U);

    Protocol_TransmitFrame(CMD_WHEEL_STATE, payload, LEN_WHEEL_STATE);
}

void Protocol_SendImuData(float roll, float pitch, float yaw)
{
    uint8_t payload[LEN_IMU_DATA];

    memcpy(&payload[0], &roll,  4U);
    memcpy(&payload[4], &pitch, 4U);
    memcpy(&payload[8], &yaw,   4U);

    Protocol_TransmitFrame(CMD_IMU_DATA, payload, LEN_IMU_DATA);
}

/* ============================================================
 * Init
 * ============================================================ */

void Protocol_Init(void)
{
    rx_state = RX_STATE_HEAD1;
    rx_cmd   = 0U;
    rx_len   = 0U;
    rx_idx   = 0U;
    rx_crc   = 0U;

    g_rx_valid_count  = 0U;
    g_rx_last_cmd     = 0U;
    g_chassis_cmd_new = 0U;

    memset((void *)&g_chassis_cmd, 0, sizeof(g_chassis_cmd));

    HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1U);
}
