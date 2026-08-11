#include "foc.h"
#include "AS5600.h"

/* ============================================================
 * FOC全局状态
 * ============================================================ */

static FOC_Handle_t foc_handle[4] ={{0.0f, 0U},{0.0f, 0U},{0.0f, 0U},{0.0f, 0U}};
float zero_electric_angle[4] ={0.0f,0.0f,0.0f,0.0f};
float g_target_velocity[4] ={0.0f,0.0f,0.0f,0.0f};
float g_velocity_error[4] ={0.0f,0.0f,0.0f,0.0f};
float g_voltage_q[4] ={0.0f,0.0f,0.0f,0.0f};

static PI_t speed_pi[4] =
{
	{0.0025f,0.05f,SPEED_PI_MIN_OUTPUT_MV,SPEED_PI_MAX_OUTPUT_MV,0,0},
	{0.001f,0.03f,SPEED_PI_MIN_OUTPUT_MV,SPEED_PI_MAX_OUTPUT_MV,0,0},
	{0.001f,0.03f,SPEED_PI_MIN_OUTPUT_MV,SPEED_PI_MAX_OUTPUT_MV,0,0},
	{0.001f,0.03f,SPEED_PI_MIN_OUTPUT_MV,SPEED_PI_MAX_OUTPUT_MV,0,0},
};

/* ============================================================
 * 内部通用函数
 * ============================================================ */

static float FOC_ConstrainFloat(float value,float min_value,float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static uint8_t FOC_IsValidMotor(uint8_t motor_id)
{	
	if (motor_id >= 1U &&motor_id <= 4U) return 1U;
    return 0U;
}

static TIM_HandleTypeDef *FOC_GetTimer(uint8_t motor_id)
{
    switch (motor_id)
    {
        case 1:
            return &htim2;
        case 2:
            return &htim3;
        case 3:
            return &htim4;
        case 4:
            return &htim5;
        default:
            return NULL;
    }
}

/* ============================================================
 * PWM和驱动器底层
 * ============================================================ */

static void FOC_MotorEnable(uint8_t motor_id,uint8_t state)
{
    GPIO_PinState gpio_state =(state != 0U)? GPIO_PIN_SET: GPIO_PIN_RESET;

    switch (motor_id)
    {
        case 1:
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,gpio_state);
            break;

        case 2:
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,gpio_state);
            break;

        case 3:
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,gpio_state);
            break;

        case 4:
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,gpio_state);
            break;

        default:
            break;
    }
}

static void FOC_StartPWM(uint8_t motor_id)
{
    TIM_HandleTypeDef *htim =FOC_GetTimer(motor_id);

    if (htim == NULL) return;

    HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(htim,TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);

    __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,0U);

    __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,0U);

    __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,0U);
}

static void FOC_SetPWMDuty(uint8_t motor_id,uint8_t channel,float duty_percent)
{
    TIM_HandleTypeDef *htim =FOC_GetTimer(motor_id);
    if (htim == NULL) return;

    duty_percent =FOC_ConstrainFloat(duty_percent,0.0f,100.0f);

    uint32_t arr =__HAL_TIM_GET_AUTORELOAD(htim);

    uint32_t pulse =(uint32_t)(duty_percent *(float)(arr + 1U) /100.0f+0.5f);

    if (pulse > arr) pulse = arr;

    switch (channel)
    {
        case 1:
            __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,pulse);
            break;

        case 2:
            __HAL_TIM_SET_COMPARE(
                htim,
                TIM_CHANNEL_2,
                pulse
            );
            break;

        case 3:
            __HAL_TIM_SET_COMPARE(
                htim,
                TIM_CHANNEL_3,
                pulse
            );
            break;

        default:
            break;
    }
}

static void FOC_SetThreePhasePWM(
    uint8_t motor_id,
    float Ua,
    float Ub,
    float Uc
)
{
    Ua =
        FOC_ConstrainFloat(
            Ua,
            0.0f,
            VOLTAGE_POWER_SUPPLY
        );

    Ub =
        FOC_ConstrainFloat(
            Ub,
            0.0f,
            VOLTAGE_POWER_SUPPLY
        );

    Uc =
        FOC_ConstrainFloat(
            Uc,
            0.0f,
            VOLTAGE_POWER_SUPPLY
        );

    float duty_a =
        Ua /
        VOLTAGE_POWER_SUPPLY *
        100.0f;

    float duty_b =
        Ub /
        VOLTAGE_POWER_SUPPLY *
        100.0f;

    float duty_c =
        Uc /
        VOLTAGE_POWER_SUPPLY *
        100.0f;

    FOC_SetPWMDuty(
        motor_id,
        1U,
        duty_a
    );

    FOC_SetPWMDuty(
        motor_id,
        2U,
        duty_b
    );

    FOC_SetPWMDuty(
        motor_id,
        3U,
        duty_c
    );
}

/* ============================================================
 * 角度和SVPWM
 * ============================================================ */

static float FOC_NormalizeAngle(
    float angle
)
{
    float normalized =
        fmodf(
            angle,
            _2PI
        );

    if (normalized < 0.0f)
    {
        normalized +=
            _2PI;
    }

    return normalized;
}

static float FOC_GetElectricalAngle(
    uint8_t motor_id
)
{
    if (FOC_IsValidMotor(motor_id) == 0U)
    {
        return 0.0f;
    }

    uint8_t idx =
        motor_id - 1U;

    float mechanical_angle =
        Encoder_GetAngleRad(
            motor_id
        );

    float electrical_angle =
        mechanical_angle *
        (float)POLE_PAIRS
        -
        zero_electric_angle[idx];

    return FOC_NormalizeAngle(
        electrical_angle
    );
}

/*
 * 共模注入式SVPWM。
 *
 * Uq和Ud单位：
 *     V
 *
 * electrical_angle单位：
 *     rad
 */
static void FOC_SetPhaseVoltage(
    uint8_t motor_id,
    float Uq,
    float Ud,
    float electrical_angle
)
{
    /*
     * 限制DQ电压矢量大小。
     */
    float voltage_magnitude =
        sqrtf(
            Uq * Uq +
            Ud * Ud
        );

    if (voltage_magnitude >
            VOLTAGE_LIMIT &&
        voltage_magnitude >
            0.0001f)
    {
        float scale =
            VOLTAGE_LIMIT /
            voltage_magnitude;

        Uq *= scale;
        Ud *= scale;
    }

    electrical_angle =
        FOC_NormalizeAngle(
            electrical_angle
        );

    float sin_angle =
        sinf(
            electrical_angle
        );

    float cos_angle =
        cosf(
            electrical_angle
        );

    /*
     * 逆Park变换。
     */
    float Ualpha =
        Ud * cos_angle
        -
        Uq * sin_angle;

    float Ubeta =
        Ud * sin_angle
        +
        Uq * cos_angle;

    /*
     * 逆Clarke变换。
     */
    float Ua =
        Ualpha;

    float Ub =
        -0.5f * Ualpha
        +
        0.86602540378f * Ubeta;

    float Uc =
        -0.5f * Ualpha
        -
        0.86602540378f * Ubeta;

    /*
     * SVPWM共模电压注入。
     */
    float Umax =
        Ua;

    float Umin =
        Ua;

    if (Ub > Umax)
    {
        Umax = Ub;
    }

    if (Uc > Umax)
    {
        Umax = Uc;
    }

    if (Ub < Umin)
    {
        Umin = Ub;
    }

    if (Uc < Umin)
    {
        Umin = Uc;
    }

    float common_offset =
        0.5f *
        VOLTAGE_POWER_SUPPLY
        -
        0.5f *
        (Umax + Umin);

    Ua += common_offset;
    Ub += common_offset;
    Uc += common_offset;

    FOC_SetThreePhasePWM(
        motor_id,
        Ua,
        Ub,
        Uc
    );
}

/* ============================================================
 * 零电角度校准
 * ============================================================ */

static void FOC_AlignSensor(
    uint8_t motor_id
)
{
    if (FOC_IsValidMotor(motor_id) == 0U)
    {
        return;
    }

    uint8_t idx =
        motor_id - 1U;

    /*
     * 输出固定电压矢量，将转子吸到固定位置。
     */
    FOC_SetPhaseVoltage(
        motor_id,
        ALIGN_VOLTAGE,
        0.0f,
        _3PI_2
    );

    HAL_Delay(
        ALIGN_TIME_MS
    );

    /*
     * 读取校准位置。
     */
    Encoder_UpdateOneChannel(
        motor_id,
        5.0f
    );

    zero_electric_angle[idx] =
        Encoder_GetAngleRad(
            motor_id
        )
        *
        (float)POLE_PAIRS;

    /*
     * 关闭输出。
     */
    FOC_SetPhaseVoltage(
        motor_id,
        0.0f,
        0.0f,
        _3PI_2
    );

    HAL_Delay(
        300U
    );
}

/* ============================================================
 * 公共初始化接口
 * ============================================================ */

void FOC_Init(
    uint8_t motor_id
)
{
    if (FOC_IsValidMotor(motor_id) == 0U)
    {
        return;
    }

    uint8_t idx =
        motor_id - 1U;

    foc_handle[idx].open_loop_shaft_angle =
        0.0f;

    foc_handle[idx].open_loop_timestamp =
        HAL_GetTick();

    g_target_velocity[idx] =
        0.0f;

    g_velocity_error[idx] =
        0.0f;

    g_voltage_q[idx] =
        0.0f;

    PI_Reset(
        &speed_pi[idx]
    );

    FOC_StartPWM(
        motor_id
    );

    FOC_MotorEnable(
        motor_id,
        1U
    );

    /*
     * 完成零电角度校准。
     */
    FOC_AlignSensor(
        motor_id
    );

    /*
     * 校准完成后，再读取当前位置。
     */
    Encoder_UpdateOneChannel(
        motor_id,
        1.0f
    );

    /*
     * 让开环角度从当前机械位置开始，
     * 避免开环第一次调用时产生角度突跳。
     */
    foc_handle[idx].open_loop_shaft_angle =
        Encoder_GetAngleRad(
            motor_id
        );

    PI_Reset(
        &speed_pi[idx]
    );
}

/* ============================================================
 * 开环接口
 * ============================================================ */

void FOC_RunOpenLoop(
    uint8_t motor_id,
    float target_speed,
    float period_ms
)
{
    if (FOC_IsValidMotor(motor_id) == 0U)
    {
        return;
    }

    if (period_ms <= 0.0f)
    {
        period_ms =
            1.0f;
    }

    uint8_t idx =
        motor_id - 1U;

    float period_s =
        period_ms /
        1000.0f;

    /*
     * 开环不使用速度PI。
     */
    PI_Reset(
        &speed_pi[idx]
    );

    foc_handle[idx].open_loop_shaft_angle +=
        target_speed *
        period_s;

    /*
     * 防止角度长期累加过大。
     */
    if (foc_handle[idx].open_loop_shaft_angle >
            100000.0f ||
        foc_handle[idx].open_loop_shaft_angle <
            -100000.0f)
    {
        foc_handle[idx].open_loop_shaft_angle =
            fmodf(
                foc_handle[idx].open_loop_shaft_angle,
                _2PI
            );
    }

    float electrical_angle =
        foc_handle[idx].open_loop_shaft_angle *
        (float)POLE_PAIRS
        -
        zero_electric_angle[idx];

    electrical_angle =
        FOC_NormalizeAngle(
            electrical_angle
        );

    g_target_velocity[idx] =
        target_speed;

    g_velocity_error[idx] =
        0.0f;

    g_voltage_q[idx] =
        OPEN_LOOP_VOLTAGE;

    FOC_SetPhaseVoltage(
        motor_id,
        OPEN_LOOP_VOLTAGE,
        0.0f,
        electrical_angle
    );
}

/* ============================================================
 * 速度闭环接口
 * ============================================================ */

void FOC_RunSpeed(
    uint8_t motor_id,
    float target_speed,
    float period_ms
)
{
    if (motor_id < 1U ||
        motor_id > 4U)
    {
        return;
    }

    if (period_ms <= 0.0f)
    {
        period_ms = 1.0f;
    }

    uint8_t idx =
        motor_id - 1U;

    /*
     * 编码器读取、速度更新和PI控制
     * 使用同一个实际dt。
     */
    Encoder_UpdateOneChannel(
        motor_id,
        period_ms
    );

    float current_velocity =
        Encoder_GetVel_filt(
            motor_id
        );

    if (fabsf(target_speed) <
        SPEED_STOP_THRESHOLD)
    {
        PI_Reset(
            &speed_pi[idx]
        );

        g_target_velocity[idx] =
            0.0f;

        g_velocity_error[idx] =
            -current_velocity;

        g_voltage_q[idx] =
            0.0f;

        FOC_SetPhaseVoltage(
            motor_id,
            0.0f,
            0.0f,
            FOC_GetElectricalAngle(
                motor_id
            )
        );

        return;
    }

    int target_speed_mrad_s =
        (int)(
            target_speed *
            SPEED_INPUT_SCALE
        );

    int current_speed_mrad_s =
        (int)(
            current_velocity *
            SPEED_INPUT_SCALE
        );

    int uq_millivolt =
        PI_Control(
            &speed_pi[idx],
            target_speed_mrad_s,
            current_speed_mrad_s
        );

    float Uq =
        (float)uq_millivolt /
        1000.0f;

    g_target_velocity[idx] =
        target_speed;

    g_velocity_error[idx] =
        target_speed -
        current_velocity;

    g_voltage_q[idx] =
        Uq;

    FOC_SetPhaseVoltage(
        motor_id,
        Uq,
        0.0f,
        FOC_GetElectricalAngle(
            motor_id
        )
    );
}

/* ============================================================
 * 停止接口
 * ============================================================ */

void FOC_Stop(
    uint8_t motor_id
)
{
    if (FOC_IsValidMotor(motor_id) == 0U)
    {
        return;
    }

    uint8_t idx =
        motor_id - 1U;

    PI_Reset(
        &speed_pi[idx]
    );

    g_target_velocity[idx] =
        0.0f;

    g_velocity_error[idx] =
        0.0f;

    g_voltage_q[idx] =
        0.0f;

    FOC_SetPhaseVoltage(
        motor_id,
        0.0f,
        0.0f,
        0.0f
    );
}

/* ============================================================
 * 参数和调试接口
 * ============================================================ */

void FOC_SetSpeedPI(
    uint8_t motor_id,
    float kp,
    float ki,
    int output_min_mv,
    int output_max_mv
)
{
    if (FOC_IsValidMotor(motor_id) == 0U)
    {
        return;
    }

    if (output_min_mv >
        output_max_mv)
    {
        return;
    }

    uint8_t idx =
        motor_id - 1U;

    speed_pi[idx].Kp =
        kp;

    speed_pi[idx].Ki =
        ki;

    speed_pi[idx].min_output =
        output_min_mv;

    speed_pi[idx].max_output =
        output_max_mv;

    PI_Reset(
        &speed_pi[idx]
    );
}

float FOC_GetVelocity(uint8_t motor_id)
{
    if (FOC_IsValidMotor(motor_id) == 0U) return 0.0f;
		return Encoder_GetVel_filt( motor_id);
}

float FOC_GetVoltageQ(uint8_t motor_id)
{
    if (FOC_IsValidMotor(motor_id) == 0U) return 0.0f;
    return g_voltage_q[motor_id - 1U];
}

