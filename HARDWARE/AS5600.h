#ifndef _AS5600_H_
#define _AS5600_H_

#include "stm32f4xx_hal.h"
#include "i2c.h"

#define AS5600_Address       0x36    // 7位地址
#define RAW_Angle_Hi         0x0C
#define RAW_Angle_Lo         0x0D
#define ANGLE_RESOLUTION     4096.0f
#define I2C_TIMEOUT          2

// ============ 软件I2C1: PB14(SDA), PB15(SCL) ============
#define SDA1_IN()   {GPIOB->MODER &= ~(3 << (14*2)); GPIOB->MODER |= (0 << (14*2));}
#define SDA1_OUT()  {GPIOB->MODER &= ~(3 << (14*2)); GPIOB->MODER |= (1 << (14*2));}
#define READ_SDA1   (GPIOB->IDR & (1 << 14))
#define IIC1_SCL_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define IIC1_SCL_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define IIC1_SDA_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define IIC1_SDA_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

// ============ 软件I2C2: PE14(SDA), PE15(SCL) ============
#define SDA2_IN()   {GPIOE->MODER &= ~(3 << (14*2)); GPIOE->MODER |= (0 << (14*2));}
#define SDA2_OUT()  {GPIOE->MODER &= ~(3 << (14*2)); GPIOE->MODER |= (1 << (14*2));}
#define READ_SDA2   (GPIOE->IDR & (1 << 14))
#define IIC2_SCL_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)
#define IIC2_SCL_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)
#define IIC2_SDA_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET)
#define IIC2_SDA_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET)



#define VEL_AVG_CNT  3
typedef struct
{
    uint16_t raw_count;
    float angle_raw;
    float angle_withtrack;
    int32_t rounds;
    float velocity;
    float vel_filt;
    float last_angle_withtrack;
    uint8_t track_initialized;
    uint8_t vel_initialized;
    uint8_t vel_cnt;
    float last_vel_pos;
    float vel_window_ms;

} EncoderData_t;

//// 全局编码器数据数组
//extern EncoderData_t g_encoder[4];

// 函数声明
/*******************底层********************************/
void Encoder_Init(uint8_t id);
/*******************驱动层********************************/
uint16_t AS5600_ReadHardware(I2C_HandleTypeDef *hi2c);
void delay_s(uint32_t i);
void IIC1_Start(void);
void IIC1_Stop(void);
uint8_t IIC1_Wait_Ack(void);
void IIC1_Ack(void);
void IIC1_NAck(void);
void IIC1_Send_Byte(uint8_t txd);
uint8_t IIC1_Read_Byte(uint8_t ack);
uint16_t AS5600_ReadSoftware1(void);
void IIC2_Start(void);
void IIC2_Stop(void);
uint8_t IIC2_Wait_Ack(void);
void IIC2_Ack(void);
void IIC2_NAck(void);
void IIC2_Send_Byte(uint8_t txd);
uint8_t IIC2_Read_Byte(uint8_t ack);
uint16_t AS5600_ReadSoftware2(void);

/*******************应用层********************************/

uint16_t Encoder_ReadRaw(uint8_t id);

void Encoder_UpdateOneChannel(uint8_t id,float period_ms);

void Encoder_UpdateAll(float period_ms);

uint16_t Encoder_GetRaw(uint8_t id);

float Encoder_GetAngleRad(uint8_t id);

float Encoder_GetAngleRad_withtrack(uint8_t id);

float Encoder_GetVelocity(uint8_t id);

float Encoder_GetVel_filt(uint8_t id);

int32_t Encoder_Gettrack(uint8_t id);

void Encoder_ResetTrack(uint8_t id);

#endif
