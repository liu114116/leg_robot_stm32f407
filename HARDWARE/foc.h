#ifndef _FOC_H_
#define _FOC_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"
#include "pid.h"

#include <stdint.h>
#include <math.h>

/* ============================================================
 * 数学常量
 * ============================================================ */

#define PI                          3.14159265359f
#define _2PI                        6.28318530718f
#define _3PI_2                      4.71238898038f

/* ============================================================
 * 电源和电机参数
 * ============================================================ */

/*
 * 实际母线电压。
 */
#define VOLTAGE_POWER_SUPPLY        12.0f

/*
 * 无电流环时的最大Uq/Ud电压。
 *
 * 2208通常内阻比较低，不建议随意提高。
 */
#define VOLTAGE_LIMIT               4.0f

/*
 * 电机极对数。
 *
 * 换电机后必须确认。
 * 2208只是尺寸规格，不代表一定是7极对。
 */
#define POLE_PAIRS                  7

/*
 * 开环运行时的固定Uq电压。
 */
#define OPEN_LOOP_VOLTAGE           3.5f

/*
 * 零电角度校准电压。
 */
#define ALIGN_VOLTAGE               2.0f

/*
 * 零电角度校准时间。
 */
#define ALIGN_TIME_MS               1000U

/* ============================================================
 * 速度环参数
 * ============================================================ */

/*
 * 速度由rad/s转换为mrad/s后送入整数PI。
 *
 * 10rad/s转换后为10000mrad/s。
 */
#define SPEED_INPUT_SCALE           1000.0f

/*
 * 增量PI输出单位为mV。
 *
 * 最终会除以1000转换为V。
 */
#define SPEED_PI_MAX_OUTPUT_MV      3000
#define SPEED_PI_MIN_OUTPUT_MV     -3000

/*
 * 目标速度绝对值低于该值时，停止输出并复位PI。
 *
 * 单位：rad/s。
 */
#define SPEED_STOP_THRESHOLD        0.05f

/* ============================================================
 * FOC内部数据结构
 * ============================================================ */

typedef struct
{
    /*
     * 开环机械角度，单位rad。
     */
    float open_loop_shaft_angle;

    /*
     * 预留时间戳。
     */
    uint32_t open_loop_timestamp;

} FOC_Handle_t;

/* ============================================================
 * 调试变量
 * ============================================================ */

/*
 * 速度环目标速度，单位rad/s。
 */
extern float g_target_velocity[4];

/*
 * 速度误差，单位rad/s。
 */
extern float g_velocity_error[4];

/*
 * PI输出的Uq，单位V。
 */
extern float g_voltage_q[4];

/*
 * 零电角度，单位rad。
 */
extern float zero_electric_angle[4];

/* ============================================================
 * 对外公共接口
 * ============================================================ */

/*
 * 初始化指定电机。
 *
 * 内部完成：
 * 1. 启动PWM；
 * 2. 使能驱动器；
 * 3. 零电角度校准；
 * 4. 清除速度PI；
 * 5. 初始化开环角度。
 *
 * motor_id：
 *     1～4。
 */
void FOC_Init(
    uint8_t motor_id
);

/*
 * 开环控制。
 *
 * target_speed：
 *     目标机械角速度，单位rad/s。
 *
 * period_ms：
 *     本函数实际调用周期，单位ms。
 *
 * 例如TIM6为1ms：
 *
 * FOC_RunOpenLoop(1, 10.0f, 1.0f);
 */
void FOC_RunOpenLoop(
    uint8_t motor_id,
    float target_speed,
    float period_ms
);

/*
 * 速度闭环控制。
 *
 * 内部自动完成：
 * 1. 读取AS5600；
 * 2. 更新机械角度和速度；
 * 3. 执行增量式PI；
 * 4. 计算实时电角度；
 * 5. 输出SVPWM。
 *
 * target_speed：
 *     目标机械角速度，单位rad/s。
 *
 * period_ms：
 *     本函数实际调用周期，单位ms。
 *
 * 例如TIM6为1ms：
 *
 * FOC_RunSpeed(1, 10.0f, 1.0f);
 */
void FOC_RunSpeed(
    uint8_t motor_id,
    float target_speed,
    float period_ms
);

/*
 * 停止指定电机并清除速度PI。
 */
void FOC_Stop(
    uint8_t motor_id
);

/*
 * 设置指定电机的增量式PI参数。
 *
 * kp/ki：
 *     对应你指定公式中的Kp和Ki。
 *
 * output_min_mv/output_max_mv：
 *     输出限幅，单位mV。
 */
void FOC_SetSpeedPI(
    uint8_t motor_id,
    float kp,
    float ki,
    int output_min_mv,
    int output_max_mv
);

/*
 * 获取当前滤波速度，单位rad/s。
 */
float FOC_GetVelocity(
    uint8_t motor_id
);

/*
 * 获取当前PI输出Uq，单位V。
 */
float FOC_GetVoltageQ(
    uint8_t motor_id
);

#endif