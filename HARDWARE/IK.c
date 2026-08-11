#include "IK.h"


#define PI 3.14159265f

// 角度归一化到 [-180°, 180°]
static float normalize_deg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// 角度 -> 舵机数值（限制在 100~4000）
static uint16_t angle_to_servo(float deg) {
    float value = SERVO_CENTER + deg * SERVO_RANGE / ANGLE_MAX;
    if (value < 100) value = 100;
    if (value > 4000) value = 4000;
    return (uint16_t)value;
}

// 检查角度是否在有效范围内
static int is_angle_valid(float a_deg, float b_deg) {
    // a 必须在 [-180°, 0°]
    if (a_deg < -180.0f || a_deg > 0.0f) return 0;
    // b 必须在 [-180°, 180°]
    if (b_deg < -180.0f || b_deg > 180.0f) return 0;
    return 1;
}

// 逆运动学：输入 (x, y)，输出舵机数值
// 返回值：0=成功，-1=超出范围，-2=无有效解
int inverse_kinematics(float x, float y, uint16_t *a_servo, uint16_t *b_servo) {
    float r = sqrtf(x*x + y*y);
    if (r > 2.0f * L || r < 0.001f) return -1;

    float phi = atan2f(y, x);
    float half = acosf(r / (2.0f * L));

    // 两个解
    float b1_rad = phi - PI + half;
    float b2_rad = phi + PI - half;
    
    float a1_rad = atan2f(y + L * sinf(b1_rad), x + L * cosf(b1_rad));
    float a2_rad = atan2f(y + L * sinf(b2_rad), x + L * cosf(b2_rad));

    // 转为角度并归一化
    float a1 = normalize_deg(a1_rad * RAD2DEG);
    float b1 = normalize_deg(b1_rad * RAD2DEG);
    float a2 = normalize_deg(a2_rad * RAD2DEG);
    float b2 = normalize_deg(b2_rad * RAD2DEG);

    // 计算膝关节角度 (必须为正)
    float knee1 = 360.0f + a1 - b1;
    float knee2 = 360.0f + a2 - b2;
    
    while (knee1 >= 360.0f) knee1 -= 360.0f;
    while (knee1 < 0.0f) knee1 += 360.0f;
    while (knee2 >= 360.0f) knee2 -= 360.0f;
    while (knee2 < 0.0f) knee2 += 360.0f;

    // 检查两个解的有效性
    int valid1 = is_angle_valid(a1, b1) && (knee1 > 0);
    int valid2 = is_angle_valid(a2, b2) && (knee2 > 0);

    float a_deg, b_deg;
    
    if (valid1 && valid2) {
        // 两个都有效，选择膝关节角度更接近 90° 的那个（更自然）
        if (fabsf(knee1 - 90.0f) < fabsf(knee2 - 90.0f)) {
            a_deg = a1; b_deg = b1;
        } else {
            a_deg = a2; b_deg = b2;
        }
    } else if (valid1) {
        a_deg = a1; b_deg = b1;
    } else if (valid2) {
        a_deg = a2; b_deg = b2;
    } else {
        return -2;  // 无有效解
    }

    // 转换为舵机数值
    *a_servo = angle_to_servo(a_deg);
    *b_servo = angle_to_servo(b_deg);

    return 0;
}



