#include "AS5600.h"
#include "foc.h"

// ============ 全局变量 ============
EncoderData_t g_encoder[4] = {0};

/*******************应用层********************************/


/*
 * 根据编码器ID读取原始角度。
 *
 * 返回范围：
 *     0～4095
 */
uint16_t Encoder_ReadRaw(uint8_t id)
{
    uint16_t raw = 0U;
    switch (id)
    {
        case 1U:
        {
            raw =AS5600_ReadHardware(&hi2c1);
            return (uint16_t)((4096U - raw) &0x0FFFU);
        }
        case 2U:
        {
            raw =AS5600_ReadHardware(&hi2c3);
            return (uint16_t)(raw &0x0FFFU);
        }
        case 3U:
        {
            raw =AS5600_ReadSoftware1();
            return (uint16_t)(raw &0x0FFFU);
        }
        case 4U:
        {
            raw =AS5600_ReadSoftware2();
            return (uint16_t)(raw &0x0FFFU);
        }
        default: return 0U;
    }
}

void Encoder_UpdateOneChannel(
    uint8_t id,
    float period_ms
)
{
    if (id < 1U || id > 4U) return; 
    if (period_ms <= 0.0f) period_ms = 1.0f;
    int idx =id - 1;
    uint16_t raw =Encoder_ReadRaw(id);
    raw &=0x0FFFU;

    g_encoder[idx].raw_count =raw;
    g_encoder[idx].angle_raw =(float)raw *_2PI /ANGLE_RESOLUTION;

    if (g_encoder[idx].track_initialized == 0U)
    {
        g_encoder[idx].last_angle_withtrack =g_encoder[idx].angle_raw;
        g_encoder[idx].rounds =0;
        g_encoder[idx].track_initialized =1U;
    }
    else
    {
        float delta =g_encoder[idx].angle_raw-g_encoder[idx].last_angle_withtrack;
        if (delta < -PI)
        {
            g_encoder[idx].rounds++;
        }
        else if (delta > PI)
        {
            g_encoder[idx].rounds--;
        }
        g_encoder[idx].last_angle_withtrack =g_encoder[idx].angle_raw;
    }
    g_encoder[idx].angle_withtrack =(float)g_encoder[idx].rounds *_2PI+g_encoder[idx].angle_raw;
    if (g_encoder[idx].vel_initialized == 0U)
    {
        g_encoder[idx].last_vel_pos =g_encoder[idx].angle_withtrack;
        g_encoder[idx].vel_window_ms =0.0f;
        g_encoder[idx].vel_cnt =0U;
        g_encoder[idx].velocity =0.0f;
        g_encoder[idx].vel_filt =0.0f;
        g_encoder[idx].vel_initialized =1U;
        return;
    }
    g_encoder[idx].vel_window_ms +=period_ms;
    g_encoder[idx].vel_cnt++;
    if (g_encoder[idx].vel_cnt >= VEL_AVG_CNT)
    {
        float delta_position =g_encoder[idx].angle_withtrack-g_encoder[idx].last_vel_pos;
        float real_period_s =g_encoder[idx].vel_window_ms/1000.0f;
        if (real_period_s > 0.000001f)
        {
            g_encoder[idx].velocity =delta_position /real_period_s;
        }
				g_encoder[idx].vel_filt =g_encoder[idx].vel_filt * 0.85f+g_encoder[idx].velocity * 0.15f;
        g_encoder[idx].last_vel_pos =g_encoder[idx].angle_withtrack;
        g_encoder[idx].vel_window_ms =0.0f;
        g_encoder[idx].vel_cnt =0U;
    }
}

void Encoder_UpdateAll(float period_ms)
{
    for (uint8_t i = 1U; i <= 4U; i++)
    {
        Encoder_UpdateOneChannel(i,period_ms);
    }
}

uint16_t Encoder_GetRaw(uint8_t id)
{
    if (id < 1U || id > 4U) return 0U;
    return g_encoder[id - 1U].raw_count;
}

float Encoder_GetAngleRad(uint8_t id)
{
    if (id < 1U || id > 4U) return 0.0f;
		return g_encoder[id - 1U].angle_raw;
}

float Encoder_GetAngleRad_withtrack(uint8_t id)
{
    if (id < 1U || id > 4U)return 0.0f;
    return g_encoder[id - 1U].angle_withtrack;
}

float Encoder_GetVelocity(uint8_t id)
{
    if (id < 1U || id > 4U) return 0.0f;
		return g_encoder[id - 1U].velocity;
}

float Encoder_GetVel_filt(uint8_t id)
{
    if (id < 1U || id > 4U) return 0.0f;
    return g_encoder[id - 1U].vel_filt;
}

int32_t Encoder_Gettrack(uint8_t id)
{
    if (id < 1U || id > 4U) return 0.0f;

    return g_encoder[id - 1U].rounds;
}

void Encoder_ResetTrack(uint8_t id)
{
    if (id < 1U || id > 4U) return;

    int idx =id - 1;
    g_encoder[idx].rounds =0;
    g_encoder[idx].last_angle_withtrack =g_encoder[idx].angle_raw;
    g_encoder[idx].angle_withtrack =g_encoder[idx].angle_raw;
    g_encoder[idx].track_initialized =1U;
    g_encoder[idx].last_vel_pos =g_encoder[idx].angle_withtrack;
    g_encoder[idx].vel_window_ms =0.0f;
    g_encoder[idx].vel_cnt =0U;
    g_encoder[idx].velocity =0.0f;
    g_encoder[idx].vel_filt =0.0f;
}

/*******************应用层********************************/


/*******************底层********************************/
void Encoder_Init(uint8_t id)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if (id == 3) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }
    else if (id == 4) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    }else if(id == 0){
				__HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			
				GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
		}
}
/*******************底层********************************/


/*******************驱动层********************************/

// ============ 硬件I2C读取 ============//
uint16_t AS5600_ReadHardware(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[2] = {0};
    /*
     * I2C1和I2C3分别保存最后一次正确数据。
     * I2C偶尔出错时，避免直接返回0导致角度突变。
     */
    static uint16_t last_i2c1 = 0;
    static uint16_t last_i2c3 = 0;
    uint16_t *last_value;
    if (hi2c == &hi2c1) {
        last_value = &last_i2c1;
    } else {
        last_value = &last_i2c3;
    }
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(
        hi2c,
        AS5600_Address << 1,
        RAW_Angle_Hi,
        I2C_MEMADD_SIZE_8BIT,
        data,
        2,
        I2C_TIMEOUT
    );
    if (status == HAL_OK) {
        *last_value =
            (((uint16_t)data[0] << 8) | data[1]) & 0x0FFF;
    }
    return *last_value;
}

void delay_s(uint32_t i)
{
    while (i--) {
        for (volatile int j = 0; j < 5; j++);
    }
}
// ============ 硬件I2C读取 ============//

// ============ 软件I2C 函数 ============
// ============ 软件I2C1 函数 ============
void IIC1_Start(void)
{
    IIC1_SDA_1;
    IIC1_SCL_1;
    delay_s(20);
    IIC1_SDA_0;
    delay_s(20);
    IIC1_SCL_0;
}

void IIC1_Stop(void)
{
    IIC1_SCL_0;
    IIC1_SDA_0;
    delay_s(20);
    IIC1_SCL_1;
    IIC1_SDA_1;
    delay_s(20);
}

uint8_t IIC1_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    
    SDA1_IN();
    IIC1_SDA_1;
    IIC1_SCL_1;
    delay_s(10);
    while (READ_SDA1 != 0) {
        if (++ucErrTime > 250) {
            SDA1_OUT();
            IIC1_Stop();
            return 1;
        }
    }
    SDA1_OUT();
    IIC1_SCL_0;
    return 0;
}
void IIC1_Ack(void)
{
    IIC1_SCL_0;
    IIC1_SDA_0;
    delay_s(20);
    IIC1_SCL_1;
    delay_s(20);
    IIC1_SCL_0;
}

void IIC1_NAck(void)
{
    IIC1_SCL_0;
    IIC1_SDA_1;
    delay_s(20);
    IIC1_SCL_1;
    delay_s(20);
    IIC1_SCL_0;
}

void IIC1_Send_Byte(uint8_t txd)
{
    uint32_t i;
    
    IIC1_SCL_0;
    for (i = 0; i < 8; i++) {
        if ((txd & 0x80) != 0) IIC1_SDA_1;
        else IIC1_SDA_0;
        txd <<= 1;
        delay_s(20);
        IIC1_SCL_1;
        delay_s(20);
        IIC1_SCL_0;
        delay_s(20);
    }
}

uint8_t IIC1_Read_Byte(uint8_t ack)
{
    uint8_t i, rcv = 0;
    
    SDA1_IN();
    for (i = 0; i < 8; i++) {
        IIC1_SCL_0;
        delay_s(20);
        IIC1_SCL_1;
        rcv <<= 1;
        if (READ_SDA1 != 0) rcv++;
        delay_s(10);
    }
    SDA1_OUT();
    if (!ack) IIC1_NAck();
    else IIC1_Ack();
    return rcv;
}

uint16_t AS5600_ReadSoftware1(void)
{
    uint8_t dh, dl;
    
    IIC1_Start();
    IIC1_Send_Byte(AS5600_Address << 1);
    IIC1_Wait_Ack();
    IIC1_Send_Byte(RAW_Angle_Hi);
    IIC1_Wait_Ack();
    IIC1_Start();
    IIC1_Send_Byte((AS5600_Address << 1) + 1);
    IIC1_Wait_Ack();
    dh = IIC1_Read_Byte(1);
    dl = IIC1_Read_Byte(0);
    IIC1_Stop();
    
    return (dh << 8) + dl;
}

// ============ 软件I2C2 函数 ============
void IIC2_Start(void)
{
    IIC2_SDA_1;
    IIC2_SCL_1;
    delay_s(20);
    IIC2_SDA_0;
    delay_s(20);
    IIC2_SCL_0;
}

void IIC2_Stop(void)
{
    IIC2_SCL_0;
    IIC2_SDA_0;
    delay_s(20);
    IIC2_SCL_1;
    IIC2_SDA_1;
    delay_s(20);
}

uint8_t IIC2_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    
    SDA2_IN();
    IIC2_SDA_1;
    IIC2_SCL_1;
    delay_s(10);
    while (READ_SDA2 != 0) {
        if (++ucErrTime > 250) {
            SDA2_OUT();
            IIC2_Stop();
            return 1;
        }
    }
    SDA2_OUT();
    IIC2_SCL_0;
    return 0;
}

void IIC2_Ack(void)
{
    IIC2_SCL_0;
    IIC2_SDA_0;
    delay_s(20);
    IIC2_SCL_1;
    delay_s(20);
    IIC2_SCL_0;
}

void IIC2_NAck(void)
{
    IIC2_SCL_0;
    IIC2_SDA_1;
    delay_s(20);
    IIC2_SCL_1;
    delay_s(20);
    IIC2_SCL_0;
}

void IIC2_Send_Byte(uint8_t txd)
{
    uint32_t i;
    
    IIC2_SCL_0;
    for (i = 0; i < 8; i++) {
        if ((txd & 0x80) != 0) IIC2_SDA_1;
        else IIC2_SDA_0;
        txd <<= 1;
        delay_s(20);
        IIC2_SCL_1;
        delay_s(20);
        IIC2_SCL_0;
        delay_s(20);
    }
}

uint8_t IIC2_Read_Byte(uint8_t ack)
{
    uint8_t i, rcv = 0;
    
    SDA2_IN();
    for (i = 0; i < 8; i++) {
        IIC2_SCL_0;
        delay_s(20);
        IIC2_SCL_1;
        rcv <<= 1;
        if (READ_SDA2 != 0) rcv++;
        delay_s(10);
    }
    SDA2_OUT();
    if (!ack) IIC2_NAck();
    else IIC2_Ack();
    return rcv;
}


uint16_t AS5600_ReadSoftware2(void)
{
    uint8_t dh, dl;
    
    IIC2_Start();
    IIC2_Send_Byte(AS5600_Address << 1);
    IIC2_Wait_Ack();
    IIC2_Send_Byte(RAW_Angle_Hi);
    IIC2_Wait_Ack();
    IIC2_Start();
    IIC2_Send_Byte((AS5600_Address << 1) + 1);
    IIC2_Wait_Ack();
    dh = IIC2_Read_Byte(1);
    dl = IIC2_Read_Byte(0);
    IIC2_Stop();
    
    return (dh << 8) + dl;
}
// ============ 软件I2C 函数 ============

/*******************驱动层********************************/
