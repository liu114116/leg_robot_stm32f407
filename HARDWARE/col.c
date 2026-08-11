#include "col.h"

/* ============================================================
 * 电机控制任务标志
 * ============================================================ */

volatile uint8_t g_control_pending =
    0U;

volatile uint32_t g_control_overrun =
    0U;

/* ============================================================
 * 腿部舵机初始化
 * ============================================================ */

void Servo_init(void)
{
    sts_init();

    sts_ping(1U);
    sts_ping(2U);
}

/* ============================================================
 * 腿部位置测试控制
 *
 * 原函数名：
 *     control()
 *
 * 新函数名：
 *     leg_pos_col()
 *
 * 其余逻辑保持原样。
 * ============================================================ */

void leg_pos_col(void)
{
    uint16_t a_servo;
    uint16_t b_servo;

    uint8_t ID[] =
    {
        1U,
        2U
    };

    uint8_t IDN =
        2U;

    uint16_t Speed[] =
    {
        2400U,
        2400U
    };

    uint8_t ACC[] =
    {
        50U,
        50U
    };

    int ret =
        inverse_kinematics(
            30.0f,
            -70.0f,
            &a_servo,
            &b_servo
        );

    if (ret == 0)
    {
        int16_t Position[] =
        {
            (int16_t)b_servo,
            (int16_t)a_servo
        };

        SyncWritePosEx(
            ID,
            IDN,
            Position,
            Speed,
            ACC
        );
    }

    HAL_Delay(
        500U
    );

    ret =
        inverse_kinematics(
            -10.0f,
            -70.0f,
            &a_servo,
            &b_servo
        );

    if (ret == 0)
    {
        int16_t Position[] =
        {
            (int16_t)b_servo,
            (int16_t)a_servo
        };

        SyncWritePosEx(
            ID,
            IDN,
            Position,
            Speed,
            ACC
        );
    }

    HAL_Delay(
        500U
    );

    ret =
        inverse_kinematics(
            0.0f,
            -50.0f,
            &a_servo,
            &b_servo
        );

    if (ret == 0)
    {
        int16_t Position[] =
        {
            (int16_t)b_servo,
            (int16_t)a_servo
        };

        SyncWritePosEx(
            ID,
            IDN,
            Position,
            Speed,
            ACC
        );
    }

    HAL_Delay(
        300U
    );
}



