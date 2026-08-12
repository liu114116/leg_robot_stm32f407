#include "col.h"

/* ============================================================
 * Timing (ms)
 * ============================================================ */

#define WHEEL_LOOP_PERIOD_MS    8U
#define REPORT_PERIOD_MS        100U
#define SERVO_INIT_GAP_MS       50U

/* ============================================================
 * Leg servo layout
 *
 * Each leg has 2 servos: knee then hip.
 *   leg 0 -> knee=1, hip=2
 *   leg 1 -> knee=3, hip=4
 *   leg 2 -> knee=5, hip=6
 *   leg 3 -> knee=7, hip=8
 *
 * NOTE: verify this against your actual wiring.
 * ============================================================ */

#define LEG_COUNT               4U
#define SERVOS_PER_LEG          2U
#define TOTAL_SERVOS            (LEG_COUNT * SERVOS_PER_LEG)

/* ============================================================
 * Wheel motor layout
 *
 * The chassis command carries one left_wheel and one right_wheel
 * (rad/s). These map to 4 FOC motors as follows:
 *   left  -> motors 1, 2
 *   right -> motors 3, 4
 *
 * NOTE: verify this against your actual wiring.
 * ============================================================ */

/* ============================================================
 * LED indicators
 *
 *   led 1 (red)   = CMD_CHASSIS_CMD (0x01) received
 *   led 2 (green) = CMD_LEG_STATE   (0x02) received
 *   led 3 (blue)  = CMD_WHEEL_STATE (0x03) received
 *   led 2 + led 3 = CMD_IMU_DATA    (0x04) received
 *
 * All LEDs are active-low (state 1 = on).
 * ============================================================ */

/* ============================================================
 * Deferred servo init state
 *
 * sts_ping() / sts_enable_torque() block until a servo replies (or
 * times out). We run them one servo per tick inside col_update() so
 * boot returns immediately and the main loop keeps running.
 * ============================================================ */

static uint8_t  s_servo_init_done    = 0U;
static uint8_t  s_servo_init_id      = 1U;
static uint8_t  s_servo_init_started = 0U;
static uint32_t s_servo_init_tick    = 0U;

/* ============================================================
 * Internal helpers
 * ============================================================ */

static void ApplyChassisCmd(void)
{
    uint8_t i;

    for (i = 0U; i < LEG_COUNT; i++)
    {
        uint8_t  id[SERVOS_PER_LEG];
        int16_t  pos[SERVOS_PER_LEG];
        uint16_t spd[SERVOS_PER_LEG];
        uint8_t  acc[SERVOS_PER_LEG];

        uint16_t hip;
        uint16_t knee;
        int      ret;

        ret = inverse_kinematics(
            g_chassis_cmd.foot_x[i],
            g_chassis_cmd.foot_z[i],
            &hip,
            &knee
        );

        if (ret != 0)
        {
            continue;
        }

        id[0]  = (uint8_t)(2U * i + 1U);   /* knee */
        id[1]  = (uint8_t)(2U * i + 2U);   /* hip  */
        pos[0] = (int16_t)knee;
        pos[1] = (int16_t)hip;
        spd[0] = 2400U;
        spd[1] = 2400U;
        acc[0] = 50U;
        acc[1] = 50U;

        SyncWritePosEx(
            id,
            SERVOS_PER_LEG,
            pos,
            spd,
            acc
        );
    }
}

static void ReportState(void)
{
    /* Wheel 0 = motor 1 feedback (real values). */
    Protocol_SendWheelState(
        0U,
        FOC_GetVelocity(1U),
        Encoder_GetAngleRad_withtrack(1U),
        FOC_GetVoltageQ(1U)
    );

    /* Leg 0 = last commanded foot position (angles left 0 for now). */
    Protocol_SendLegState(
        0U,
        0.0f,
        0.0f,
        g_chassis_cmd.foot_x[0],
        g_chassis_cmd.foot_z[0]
    );

    /* IMU not wired yet, send zeros. */
    Protocol_SendImuData(0.0f, 0.0f, 0.0f);
}

static void LedToggle(uint8_t led, uint8_t *state)
{
    *state = (uint8_t)(*state ^ 1U);
    led_show(led, *state);
}

static void ServoInitTick(uint32_t now_tick)
{
    if (s_servo_init_done != 0U)
    {
        return;
    }

    if (s_servo_init_started == 0U)
    {
        s_servo_init_started = 1U;
        s_servo_init_tick    = now_tick;
        return;
    }

    if ((now_tick - s_servo_init_tick) < SERVO_INIT_GAP_MS)
    {
        return;
    }
    s_servo_init_tick = now_tick;

    if (sts_ping(s_servo_init_id) >= 0)
    {
        sts_enable_torque(s_servo_init_id);
    }

    s_servo_init_id++;

    if (s_servo_init_id > (uint8_t)TOTAL_SERVOS)
    {
        s_servo_init_done = 1U;
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

void col_init(void)
{
    /*
     * Open USART1 RX first so the protocol is live immediately.
     * Servo torque-enable is deferred to col_update (non-blocking).
     */
    Protocol_Init();

    sts_init();

    s_servo_init_done    = 0U;
    s_servo_init_id      = 1U;
    s_servo_init_started = 0U;
    s_servo_init_tick    = 0U;

    /*
     * FOC align is blocking (~1.3s per motor) and is required for
     * correct speed-loop operation, so keep it synchronous here.
     */
    FOC_Init(1U);
    FOC_Init(2U);
    FOC_Init(3U);
    FOC_Init(4U);
}

void col_update(uint32_t now_tick)
{
    static uint32_t last_foc_tick    = 0U;
    static uint32_t last_report_tick = 0U;
    static uint32_t last_rx_count    = 0U;
    static uint8_t  led_red          = 0U;
    static uint8_t  led_green        = 0U;
    static uint8_t  led_blue         = 0U;

    uint32_t elapsed;

    ServoInitTick(now_tick);

    /* ---- wheels: speed loop at fixed period ---- */
    elapsed = now_tick - last_foc_tick;
    if (elapsed >= WHEEL_LOOP_PERIOD_MS)
    {
        last_foc_tick = now_tick;

        if (g_chassis_cmd_new != 0U)
        {
            g_chassis_cmd_new = 0U;
            ApplyChassisCmd();
        }

        FOC_RunSpeed(1U, g_chassis_cmd.left_wheel,  (float)elapsed);
        FOC_RunSpeed(2U, g_chassis_cmd.left_wheel,  (float)elapsed);
        FOC_RunSpeed(3U, g_chassis_cmd.right_wheel, (float)elapsed);
        FOC_RunSpeed(4U, g_chassis_cmd.right_wheel, (float)elapsed);
    }

    /* ---- uplink state frames (also drives the loopback test) ---- */
    if ((now_tick - last_report_tick) >= REPORT_PERIOD_MS)
    {
        last_report_tick = now_tick;
        ReportState();
    }

    /* ---- one LED per received data type ---- */
    if (g_rx_valid_count != last_rx_count)
    {
        last_rx_count = g_rx_valid_count;

        switch (g_rx_last_cmd)
        {
            case CMD_CHASSIS_CMD:
                LedToggle(1U, &led_red);
                break;

            case CMD_LEG_STATE:
                LedToggle(2U, &led_green);
                break;

            case CMD_WHEEL_STATE:
                LedToggle(3U, &led_blue);
                break;

            case CMD_IMU_DATA:
                LedToggle(2U, &led_green);
                LedToggle(3U, &led_blue);
                break;

            default:
                LedToggle(1U, &led_red);
                break;
        }
    }
}

/* ============================================================
 * ID-mapping test helpers
 *
 * One-shot wiring checks. Call them from main() (after col_init()).
 * They print over USART1 via HC05_Printf so you can watch results on
 * a serial terminal.
 * ============================================================ */

void col_test_servo_map(void)
{
    uint8_t id;

    HC05_Printf("\r\n--- servo id map test ---\r\n");

    for (id = 1U; id <= (uint8_t)TOTAL_SERVOS; id++)
    {
        int ret = sts_ping(id);

        if (ret >= 0)
        {
            HC05_Printf("servo %d: OK (reply %d)\r\n", id, ret);
        }
        else
        {
            HC05_Printf("servo %d: no reply\r\n", id);
        }
    }

    HC05_Printf("--- done ---\r\n");
}

void col_test_motor_map(void)
{
    uint8_t id;

    HC05_Printf("\r\n--- motor id map test ---\r\n");
    HC05_Printf("spin each motor ~1.5s, watch the wheel\r\n");

    for (id = 1U; id <= 4U; id++)
    {
        uint32_t t0 = HAL_GetTick();

        HC05_Printf("motor %d: running...\r\n", id);

        while ((HAL_GetTick() - t0) < 1500U)
        {
            FOC_RunOpenLoop(id, 5.0f, 8.0f);
            HAL_Delay(8U);
        }

        FOC_Stop(id);
        HC05_Printf("motor %d: stopped\r\n", id);
    }

    HC05_Printf("--- done ---\r\n");
}
