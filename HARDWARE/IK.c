#include "IK.h"


/* PI 已在 foc.h 中定义，通过 IK.h 间接引入 */

// �Ƕȹ�һ���� [-180��, 180��]
static float normalize_deg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// �Ƕ� -> �����ֵ�������� 100~4000��
static uint16_t angle_to_servo(float deg) {
    float value = SERVO_CENTER + deg * SERVO_RANGE / ANGLE_MAX;
    if (value < 100) value = 100;
    if (value > 4000) value = 4000;
    return (uint16_t)value;
}

// ���Ƕ��Ƿ�����Ч��Χ��
static int is_angle_valid(float a_deg, float b_deg) {
    // a ������ [-180��, 0��]
    if (a_deg < -180.0f || a_deg > 0.0f) return 0;
    // b ������ [-180��, 180��]
    if (b_deg < -180.0f || b_deg > 180.0f) return 0;
    return 1;
}

// ���˶�ѧ������ (x, y)����������ֵ
// ����ֵ��0=�ɹ���-1=������Χ��-2=����Ч��
int inverse_kinematics(float x, float y, uint16_t *a_servo, uint16_t *b_servo) {
    float r = sqrtf(x*x + y*y);
    if (r > 2.0f * L || r < 0.001f) return -1;

    float phi = atan2f(y, x);
    float half = acosf(r / (2.0f * L));

    // ������
    float b1_rad = phi - PI + half;
    float b2_rad = phi + PI - half;
    
    float a1_rad = atan2f(y + L * sinf(b1_rad), x + L * cosf(b1_rad));
    float a2_rad = atan2f(y + L * sinf(b2_rad), x + L * cosf(b2_rad));

    // תΪ�ǶȲ���һ��
    float a1 = normalize_deg(a1_rad * RAD2DEG);
    float b1 = normalize_deg(b1_rad * RAD2DEG);
    float a2 = normalize_deg(a2_rad * RAD2DEG);
    float b2 = normalize_deg(b2_rad * RAD2DEG);

    // ����ϥ�ؽڽǶ� (����Ϊ��)
    float knee1 = 360.0f + a1 - b1;
    float knee2 = 360.0f + a2 - b2;
    
    while (knee1 >= 360.0f) knee1 -= 360.0f;
    while (knee1 < 0.0f) knee1 += 360.0f;
    while (knee2 >= 360.0f) knee2 -= 360.0f;
    while (knee2 < 0.0f) knee2 += 360.0f;

    // ������������Ч��
    int valid1 = is_angle_valid(a1, b1) && (knee1 > 0);
    int valid2 = is_angle_valid(a2, b2) && (knee2 > 0);

    float a_deg, b_deg;
    
    if (valid1 && valid2) {
        // ��������Ч��ѡ��ϥ�ؽڽǶȸ��ӽ� 90�� ���Ǹ�������Ȼ��
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
        return -2;  // ����Ч��
    }

    // ת��Ϊ�����ֵ
    *a_servo = angle_to_servo(a_deg);
    *b_servo = angle_to_servo(b_deg);

    return 0;
}



