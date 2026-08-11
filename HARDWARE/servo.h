#ifndef _SERVO_H_
#define _SERVO_H_
#include <stdint.h>
#include <stddef.h> 

// ========== 指令定义 ==========
#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_REG_WRITE  0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_WRITE 0x83
#define INST_RESET      0x0A

// ========== STS 舵机寄存器地址 ==========
#define STS_ID                      5
#define STS_BAUD_RATE               6
#define STS_MIN_ANGLE_LIMIT_L       9
#define STS_MIN_ANGLE_LIMIT_H       10
#define STS_MAX_ANGLE_LIMIT_L       11
#define STS_MAX_ANGLE_LIMIT_H       12
#define STS_MODE                    33
#define STS_TORQUE_ENABLE           40
#define STS_ACC                     41
#define STS_GOAL_POSITION_L         42
#define STS_GOAL_POSITION_H         43
#define STS_GOAL_TIME_L             44
#define STS_GOAL_TIME_H             45
#define STS_GOAL_SPEED_L            46
#define STS_GOAL_SPEED_H            47
#define STS_LOCK                    55
#define STS_PRESENT_POSITION_L      56
#define STS_PRESENT_POSITION_H      57
#define STS_PRESENT_SPEED_L         58
#define STS_PRESENT_SPEED_H         59
#define STS_PRESENT_VOLTAGE         62
#define STS_PRESENT_TEMPERATURE     63
#define STS_MOVING                  66

// ========== 错误码 ==========
#define STS_ERR_NO_REPLY    1
#define STS_ERR_CRC_CMP     2
#define STS_ERR_SLAVE_ID    3
#define STS_ERR_BUFF_LEN    4

//公开API
void sts_init(void); //初始化所有舵机
void setLevel(uint8_t _Level);//设置舵机返回等级（应答等级）,
int sts_ping(uint8_t ID); //ping舵机，只有ping之后才能发送指令
int sts_write_pos(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC); //STS 舵机位置控制函数（带加速度和速度）
void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]); //同步写位置指令（多个舵机同时动作）
void sts_enable_torque(uint8_t ID); //使能舵机扭矩（让舵机有力）
void sts_disable_torque(uint8_t ID); //关闭舵机扭矩（舵机无力，可自由转动）
int sts_read_temp(uint8_t ID); //读取舵机当前温度
int sts_read_pos(uint8_t ID); //读取舵机当前位置

#endif
