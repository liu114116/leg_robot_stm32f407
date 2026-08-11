#include "servo.h"
#include "usart.h"

// ========== 静态变量 ==========
static uint8_t Level = 1;           // 舵机返回等级
static uint8_t End = 0;             // 大小端：0=小端, 1=大端
static uint8_t u8Status = 0;        // 舵机状态
static uint8_t u8Error = 0;         // 通信状态

// ========== 底层串口函数 ==========

/**
 * @brief 通过 USART1 发送多个字节
 * @param data 数据指针
 * @param len  数据长度
 */
static void uart_send(uint8_t *data, int len)
{
    HAL_UART_Transmit(&huart5, data, len, 100);
}

/**
 * @brief 从 USART1 接收多个字节
 * @param data 接收缓冲区指针
 * @param len  期望接收的字节数
 * @return 实际接收到的字节数
 */
static int uart_read(uint8_t *data, int len)
{
    HAL_UART_Receive(&huart5, data, len, 100);
    return len;
}

/**
 * @brief 清空 USART1 接收缓冲区，丢弃所有待处理数据
 */
void rFlushSCS(void)
{
    while(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE)) {
        uint8_t dummy;
        HAL_UART_Receive(&huart5, &dummy, 1, 0);
    }
}

// ========== 硬件接口（与官方 SCS.c 兼容） ==========

/**
 * @brief 发送数据缓冲区（官方接口，使用缓冲区暂存）
 * @note  官方代码中 wFlushSCS 是空函数，实际发送在 writeSCS 中完成
 */
void wFlushSCS(void)
{
    // 官方代码中 wFlushSCS 是空函数，实际发送在 writeSCS 中完成
    // 这里留空，与官方逻辑一致
}

/**
 * @brief 发送多个字节（官方接口）
 * @param nDat 数据指针
 * @param nLen 数据长度
 * @return 发送的字节数
 */
int writeSCS(uint8_t *nDat, int nLen)
{
    uart_send(nDat, nLen);
    return nLen;
}

/**
 * @brief 发送单个字节（官方接口）
 * @param bDat 要发送的字节
 * @return 1（成功）
 */
int writeByteSCS(uint8_t bDat)
{
    uart_send(&bDat, 1);
    return 1;
}

/**
 * @brief 接收多个字节（官方接口）
 * @param nDat 接收缓冲区指针
 * @param nLen 期望接收的字节数
 * @return 实际接收的字节数
 */
int readSCS(uint8_t *nDat, int nLen)
{
    return uart_read(nDat, nLen);
}

// ========== 大小端设置 ==========

/**
 * @brief 设置数据字节序（大小端）
 * @param _End 0=小端模式（低字节在前），1=大端模式（高字节在前）
 */
void setEnd(uint8_t _End)
{
    End = _End;
}

/**
 * @brief 获取当前字节序设置
 * @return 0=小端模式，1=大端模式
 */
uint8_t getEnd(void)
{
    return End;
}

/**
 * @brief 设置舵机返回等级（应答等级）
 * @param _Level 1=开启应答，写指令后等待舵机回复
 */
void setLevel(uint8_t _Level)
{
    Level = _Level;
}

/**
 * @brief 获取最后一次指令返回的舵机状态
 * @return 舵机状态字节
 */
int getState(void)
{
    return u8Status;
}

/**
 * @brief 获取最后一次通信的错误码
 * @return 错误码（1=无回复，2=校验错误，3=ID不匹配，4=长度错误）
 */
int getLastError(void)
{
    return u8Error;
}

// ========== 数据转换 ==========

/**
 * @brief 将16位整数拆分为两个8位字节（根据当前字节序）
 * @param DataL 输出：低位字节指针
 * @param DataH 输出：高位字节指针
 * @param Data  输入：16位整数
 * @note 小端模式：DataL=低8位，DataH=高8位
 *       大端模式：DataL=高8位，DataH=低8位
 */
void Host2SCS(uint8_t *DataL, uint8_t *DataH, int Data)
{
    if(End) {
        *DataL = (Data >> 8);
        *DataH = (Data & 0xFF);
    } else {
        *DataH = (Data >> 8);
        *DataL = (Data & 0xFF);
    }
}

/**
 * @brief 将两个8位字节合并为16位整数（根据当前字节序）
 * @param DataL 低位字节
 * @param DataH 高位字节
 * @return 合并后的16位整数
 * @note 小端模式：DataL=低8位，DataH=高8位
 *       大端模式：DataL=高8位，DataH=低8位
 */
int SCS2Host(uint8_t DataL, uint8_t DataH)
{
    if(End) {
        return (DataL << 8) | DataH;
    } else {
        return (DataH << 8) | DataL;
    }
}

// ========== 写缓冲区（核心协议函数） ==========

/**
 * @brief 构建并发送舵机指令帧（核心协议函数）
 * @param ID      舵机ID（0~253，254为广播）
 * @param MemAddr 寄存器地址
 * @param nDat    数据指针
 * @param nLen    数据长度
 * @param Fun     指令类型（INST_WRITE/INST_READ/INST_PING等）
 * @note 自动计算校验和（累加取反），格式：0xFF 0xFF ID Len Cmd Addr Data... CheckSum
 */
void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
    uint8_t i;
    uint8_t msgLen = 2;
    uint8_t bBuf[6];
    uint8_t CheckSum = 0;
    
    bBuf[0] = 0xFF;
    bBuf[1] = 0xFF;
    bBuf[2] = ID;
    bBuf[4] = Fun;
    
    if(nDat) {
        msgLen += nLen + 1;
        bBuf[3] = msgLen;
        bBuf[5] = MemAddr;
        writeSCS(bBuf, 6);
    } else {
        bBuf[3] = msgLen;
        writeSCS(bBuf, 5);
    }
    
    CheckSum = ID + msgLen + Fun + MemAddr;
    if(nDat) {
        for(i = 0; i < nLen; i++) {
            CheckSum += nDat[i];
        }
        writeSCS(nDat, nLen);
    }
    
    CheckSum = ~CheckSum;
    writeSCS(&CheckSum, 1);
}

// ========== 检测帧头 ==========

/**
 * @brief 检测并同步到数据帧头（连续两个0xFF）
 * @return 1=找到帧头，0=超时未找到
 */
int checkHead(void)
{
    uint8_t bDat;
    uint8_t bBuf[2] = {0, 0};
    uint8_t Cnt = 0;
    
    while(1) {
        if(readSCS(&bDat, 1) != 1) return 0;
        bBuf[1] = bBuf[0];
        bBuf[0] = bDat;
        if(bBuf[0] == 0xFF && bBuf[1] == 0xFF) break;
        Cnt++;
        if(Cnt > 10) return 0;
    }
    return 1;
}

// ========== 指令应答 ==========

/**
 * @brief 等待并处理舵机应答帧
 * @param ID 期望的舵机ID
 * @return 1=应答正确，0=应答错误（错误码存于u8Error）
 */
int Ack(uint8_t ID)
{
    uint8_t bBuf[4];
    uint8_t calSum;
    
    u8Error = 0;
    if(ID != 0xFE && Level) {
        if(!checkHead()) {
            u8Error = STS_ERR_NO_REPLY;
            return 0;
        }
        u8Status = 0;
        if(readSCS(bBuf, 4) != 4) {
            u8Error = STS_ERR_NO_REPLY;
            return 0;
        }
        if(bBuf[0] != ID) {
            u8Error = STS_ERR_SLAVE_ID;
            return 0;
        }
        if(bBuf[1] != 2) {
            u8Error = STS_ERR_BUFF_LEN;
            return 0;
        }
        calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
        if(calSum != bBuf[3]) {
            u8Error = STS_ERR_CRC_CMP;
            return 0;
        }
        u8Status = bBuf[2];
    }
    return 1;
}


/**
 * @brief 写入1个字节到舵机寄存器
 * @param ID      舵机ID
 * @param MemAddr 寄存器地址
 * @param bDat    要写入的字节
 * @return 1=成功，0=失败
 */
int sts_write_byte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

/**
 * @brief 写入2个字节到舵机寄存器（16位数据）
 * @param ID      舵机ID
 * @param MemAddr 寄存器起始地址
 * @param wDat    要写入的16位数据
 * @return 1=成功，0=失败
 */
int sts_write_word(uint8_t ID, uint8_t MemAddr, uint16_t wDat)
{
    uint8_t buf[2];
    Host2SCS(&buf[0], &buf[1], wDat);
    rFlushSCS();
    writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

/**
 * @brief 从舵机读取多个字节
 * @param ID      舵机ID
 * @param MemAddr 寄存器起始地址
 * @param nData   接收缓冲区指针
 * @param nLen    要读取的字节数
 * @return 实际读取的字节数，0=失败
 */
int sts_read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen)
{
    int Size;
    uint8_t bBuf[4];
    uint8_t calSum;
    uint8_t i;
    
    rFlushSCS();
    writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
    wFlushSCS();
    u8Error = 0;
    
    if(!checkHead()) {
        u8Error = STS_ERR_NO_REPLY;
        return 0;
    }
    
    if(readSCS(bBuf, 3) != 3) {
        u8Error = STS_ERR_NO_REPLY;
        return 0;
    }
    
    if(bBuf[0] != ID && ID != 0xFE) {
        u8Error = STS_ERR_SLAVE_ID;
        return 0;
    }
    
    if(bBuf[1] != (nLen + 2)) {
        u8Error = STS_ERR_BUFF_LEN;
        return 0;
    }
    
    Size = readSCS(nData, nLen);
    if(Size != nLen) {
        u8Error = STS_ERR_NO_REPLY;
        return 0;
    }
    
    if(readSCS(bBuf + 3, 1) != 1) {
        u8Error = STS_ERR_NO_REPLY;
        return 0;
    }
    
    calSum = bBuf[0] + bBuf[1] + bBuf[2];
    for(i = 0; i < Size; i++) {
        calSum += nData[i];
    }
    calSum = ~calSum;
    
    if(calSum != bBuf[3]) {
        u8Error = STS_ERR_CRC_CMP;
        return 0;
    }
    
    u8Status = bBuf[2];
    return Size;
}

/**
 * @brief 从舵机读取1个字节
 * @param ID      舵机ID
 * @param MemAddr 寄存器地址
 * @return 成功返回读取的字节值，失败返回-1
 */
int sts_read_byte(uint8_t ID, uint8_t MemAddr)
{
    uint8_t bDat;
    int Size = sts_read(ID, MemAddr, &bDat, 1);
    if(Size != 1) return -1;
    return bDat;
}

/**
 * @brief 从舵机读取2个字节（16位数据）
 * @param ID      舵机ID
 * @param MemAddr 寄存器起始地址
 * @return 成功返回16位数据，失败返回-1
 */
int sts_read_word(uint8_t ID, uint8_t MemAddr)
{
    uint8_t nDat[2];
    int Size = sts_read(ID, MemAddr, nDat, 2);
    if(Size != 2) return -1;
    return SCS2Host(nDat[0], nDat[1]);
}


// ========== 同步写指令 ==========

/**
 * @brief 同步写指令核心函数
 * @param ID      舵机ID数组
 * @param IDN     舵机数量
 * @param MemAddr 寄存器起始地址
 * @param nDat    数据缓冲区（每个舵机nLen字节，连续存放）
 * @param nLen    每个舵机的数据长度
 * @note 所有舵机同时执行指令，用于多舵机同步控制
 */
void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    uint8_t mesLen = ((nLen + 1) * IDN + 4);
    uint8_t Sum = 0;
    uint8_t bBuf[7];
    uint8_t i, j;
    
    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = 0xfe;
    bBuf[3] = mesLen;
    bBuf[4] = INST_SYNC_WRITE;
    bBuf[5] = MemAddr;
    bBuf[6] = nLen;
    
    rFlushSCS();
    writeSCS(bBuf, 7);

    Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;

    for(i = 0; i < IDN; i++){
        writeSCS(&ID[i], 1);
        writeSCS(nDat + i * nLen, nLen);
        Sum += ID[i];
        for(j = 0; j < nLen; j++){
            Sum += nDat[i * nLen + j];
        }
    }
    Sum = ~Sum;
    writeSCS(&Sum, 1);
    wFlushSCS();
}

// ========== 公开 API ==========

/**
 * @brief 初始化STS舵机通信库
 * @note 设置小端模式、开启应答、清空接收缓冲区
 */
void sts_init(void)
{
    setEnd(0);      // 小端模式
    setLevel(1);    // 开启应答
    u8Status = 0;
    u8Error = 0;
    rFlushSCS();
}

/**
 * @brief PING指令：检测舵机是否在线
 * @param ID 舵机ID（1~253，254为广播）
 * @return 成功返回舵机ID，失败返回-1（错误码通过getLastError()获取）
 */
int sts_ping(uint8_t ID)
{
    uint8_t bBuf[4];
    uint8_t calSum;
    
    rFlushSCS();
    writeBuf(ID, 0, NULL, 0, INST_PING);
    wFlushSCS();
    u8Status = 0;
    
    if(!checkHead()) {
        u8Error = STS_ERR_NO_REPLY;
        return -1;
    }
    u8Error = 0;
    
    if(readSCS(bBuf, 4) != 4) {
        u8Error = STS_ERR_NO_REPLY;
        return -1;
    }
    
    if(bBuf[0] != ID && ID != 0xFE) {
        u8Error = STS_ERR_SLAVE_ID;
        return -1;
    }
    
    if(bBuf[1] != 2) {
        u8Error = STS_ERR_BUFF_LEN;
        return -1;
    }
    
    calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
    if(calSum != bBuf[3]) {
        u8Error = STS_ERR_CRC_CMP;
        return -1;
    }
    
    u8Status = bBuf[2];
    return bBuf[0];
}

/**
 * @brief  STS 舵机位置控制函数（带加速度和速度）
 * @param  ID       舵机 ID 号（1~253，254 为广播 ID）
 * @param  Position 目标位置（范围 0~4095，对应 0°~360°）
 *                  注：部分舵机量程可能为 0~1000，请根据实际舵机型号确认
 * @param  Speed    目标速度（范围 0~3400，值越大速度越快）
 *                  0 表示使用舵机内部默认速度
 * @param  ACC      加速度（范围 0~254，值越大加速越快）
 *                  0 表示使用舵机内部默认加速度
 * @retval 返回 1 表示成功，0 表示失败（可通过 getLastError() 获取错误码）
 * @note   使用前需先调用 sts_enable_torque() 使能扭矩，否则舵机无力
 * @note   位置、速度、加速度的寄存器地址定义在 STS_ACC、STS_GOAL_POSITION_L 等宏中
 */
int sts_write_pos(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
    uint8_t bBuf[7];
    
    if(Position < 0) {
        Position = -Position;
        Position |= (1 << 15);
    }
    
    bBuf[0] = ACC;
    Host2SCS(&bBuf[1], &bBuf[2], Position);
    Host2SCS(&bBuf[3], &bBuf[4], 0);
    Host2SCS(&bBuf[5], &bBuf[6], Speed);
    
    rFlushSCS();
    writeBuf(ID, STS_ACC, bBuf, 7, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

/**
 * @brief 同步写位置指令（多个舵机同时动作）
 * @param ID[]     舵机ID数组
 * @param IDN      舵机数量
 * @param Position[] 目标位置数组
 * @param Speed[]    目标速度数组
 * @param ACC[]      加速度数组
 * @note 所有舵机同时开始运动，实现同步控制
 */
void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[])
{
    uint8_t offbuf[32 * 7];
    uint8_t i;
    uint16_t V;
    
    for(i = 0; i < IDN; i++){
        if(Position[i] < 0){
            Position[i] = -Position[i];
            Position[i] |= (1 << 15);
        }

        if(Speed){
            V = Speed[i];
        } else {
            V = 0;
        }
        if(ACC){
            offbuf[i * 7] = ACC[i];
        } else {
            offbuf[i * 7] = 0;
        }
        Host2SCS(offbuf + i * 7 + 1, offbuf + i * 7 + 2, Position[i]);
        Host2SCS(offbuf + i * 7 + 3, offbuf + i * 7 + 4, 0);
        Host2SCS(offbuf + i * 7 + 5, offbuf + i * 7 + 6, V);
    }
    syncWrite(ID, IDN, STS_ACC, offbuf, 7);
}

/**
 * @brief 使能舵机扭矩（让舵机有力）
 * @param ID 舵机ID
 * @note 使能后舵机能保持位置并抵抗外力
 */
void sts_enable_torque(uint8_t ID)
{
    sts_write_byte(ID, STS_TORQUE_ENABLE, 1);
}

/**
 * @brief 关闭舵机扭矩（舵机无力，可自由转动）
 * @param ID 舵机ID
 */
void sts_disable_torque(uint8_t ID)
{
    sts_write_byte(ID, STS_TORQUE_ENABLE, 0);
}

/**
 * @brief 读取舵机当前温度
 * @param ID 舵机ID
 * @return 成功返回温度值（摄氏度），失败返回-1
 */
int sts_read_temp(uint8_t ID)
{
    return sts_read_byte(ID, STS_PRESENT_TEMPERATURE);
}

/**
 * @brief 读取舵机当前位置
 * @param ID 舵机ID
 * @return 成功返回位置值（0~4095），失败返回-1
 */
int sts_read_pos(uint8_t ID)
{
    return sts_read_word(ID, STS_PRESENT_POSITION_L);
}

