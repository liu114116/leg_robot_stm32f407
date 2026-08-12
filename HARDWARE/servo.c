#include "servo.h"
#include "usart.h"

// ========== ��̬���� ==========
static uint8_t Level = 1;           // ������صȼ�
static uint8_t End = 0;             // ��С�ˣ�0=С��, 1=���
static uint8_t u8Status = 0;        // ���״̬
static uint8_t u8Error = 0;         // ͨ��״̬

// ========== �ײ㴮�ں��� ==========

/**
 * @brief ͨ�� USART1 ���Ͷ���ֽ�
 * @param data ����ָ��
 * @param len  ���ݳ���
 */
static void uart_send(uint8_t *data, int len)
{
    HAL_UART_Transmit(&huart5, data, len, 100);
}

/**
 * @brief �� USART1 ���ն���ֽ�
 * @param data ���ջ�����ָ��
 * @param len  �������յ��ֽ���
 * @return ʵ�ʽ��յ����ֽ���
 */
static int uart_read(uint8_t *data, int len)
{
    /* Short timeout: keep absent-servo ops from stalling the main loop. */
    HAL_UART_Receive(&huart5, data, len, 5);
    return len;
}

/**
 * @brief ��� USART1 ���ջ��������������д���������
 */
void rFlushSCS(void)
{
    while(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE)) {
        uint8_t dummy;
        HAL_UART_Receive(&huart5, &dummy, 1, 0);
    }
}

// ========== Ӳ���ӿڣ���ٷ� SCS.c ���ݣ� ==========

/**
 * @brief �������ݻ��������ٷ��ӿڣ�ʹ�û������ݴ棩
 * @note  �ٷ������� wFlushSCS �ǿպ�����ʵ�ʷ����� writeSCS �����
 */
void wFlushSCS(void)
{
    // �ٷ������� wFlushSCS �ǿպ�����ʵ�ʷ����� writeSCS �����
    // �������գ���ٷ��߼�һ��
}

/**
 * @brief ���Ͷ���ֽڣ��ٷ��ӿڣ�
 * @param nDat ����ָ��
 * @param nLen ���ݳ���
 * @return ���͵��ֽ���
 */
int writeSCS(uint8_t *nDat, int nLen)
{
    uart_send(nDat, nLen);
    return nLen;
}

/**
 * @brief ���͵����ֽڣ��ٷ��ӿڣ�
 * @param bDat Ҫ���͵��ֽ�
 * @return 1���ɹ���
 */
int writeByteSCS(uint8_t bDat)
{
    uart_send(&bDat, 1);
    return 1;
}

/**
 * @brief ���ն���ֽڣ��ٷ��ӿڣ�
 * @param nDat ���ջ�����ָ��
 * @param nLen �������յ��ֽ���
 * @return ʵ�ʽ��յ��ֽ���
 */
int readSCS(uint8_t *nDat, int nLen)
{
    return uart_read(nDat, nLen);
}

// ========== ��С������ ==========

/**
 * @brief ���������ֽ��򣨴�С�ˣ�
 * @param _End 0=С��ģʽ�����ֽ���ǰ����1=���ģʽ�����ֽ���ǰ��
 */
void setEnd(uint8_t _End)
{
    End = _End;
}

/**
 * @brief ��ȡ��ǰ�ֽ�������
 * @return 0=С��ģʽ��1=���ģʽ
 */
uint8_t getEnd(void)
{
    return End;
}

/**
 * @brief ���ö�����صȼ���Ӧ��ȼ���
 * @param _Level 1=����Ӧ��дָ���ȴ�����ظ�
 */
void setLevel(uint8_t _Level)
{
    Level = _Level;
}

/**
 * @brief ��ȡ���һ��ָ��صĶ��״̬
 * @return ���״̬�ֽ�
 */
int getState(void)
{
    return u8Status;
}

/**
 * @brief ��ȡ���һ��ͨ�ŵĴ�����
 * @return �����루1=�޻ظ���2=У�����3=ID��ƥ�䣬4=���ȴ���
 */
int getLastError(void)
{
    return u8Error;
}

// ========== ����ת�� ==========

/**
 * @brief ��16λ�������Ϊ����8λ�ֽڣ����ݵ�ǰ�ֽ���
 * @param DataL �������λ�ֽ�ָ��
 * @param DataH �������λ�ֽ�ָ��
 * @param Data  ���룺16λ����
 * @note С��ģʽ��DataL=��8λ��DataH=��8λ
 *       ���ģʽ��DataL=��8λ��DataH=��8λ
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
 * @brief ������8λ�ֽںϲ�Ϊ16λ���������ݵ�ǰ�ֽ���
 * @param DataL ��λ�ֽ�
 * @param DataH ��λ�ֽ�
 * @return �ϲ����16λ����
 * @note С��ģʽ��DataL=��8λ��DataH=��8λ
 *       ���ģʽ��DataL=��8λ��DataH=��8λ
 */
int SCS2Host(uint8_t DataL, uint8_t DataH)
{
    if(End) {
        return (DataL << 8) | DataH;
    } else {
        return (DataH << 8) | DataL;
    }
}

// ========== д������������Э�麯���� ==========

/**
 * @brief ���������Ͷ��ָ��֡������Э�麯����
 * @param ID      ���ID��0~253��254Ϊ�㲥��
 * @param MemAddr �Ĵ�����ַ
 * @param nDat    ����ָ��
 * @param nLen    ���ݳ���
 * @param Fun     ָ�����ͣ�INST_WRITE/INST_READ/INST_PING�ȣ�
 * @note �Զ�����У��ͣ��ۼ�ȡ��������ʽ��0xFF 0xFF ID Len Cmd Addr Data... CheckSum
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

// ========== ���֡ͷ ==========

/**
 * @brief ��Ⲣͬ��������֡ͷ����������0xFF��
 * @return 1=�ҵ�֡ͷ��0=��ʱδ�ҵ�
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

// ========== ָ��Ӧ�� ==========

/**
 * @brief �ȴ����������Ӧ��֡
 * @param ID �����Ķ��ID
 * @return 1=Ӧ����ȷ��0=Ӧ����󣨴��������u8Error��
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
 * @brief д��1���ֽڵ�����Ĵ���
 * @param ID      ���ID
 * @param MemAddr �Ĵ�����ַ
 * @param bDat    Ҫд����ֽ�
 * @return 1=�ɹ���0=ʧ��
 */
int sts_write_byte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

/**
 * @brief д��2���ֽڵ�����Ĵ�����16λ���ݣ�
 * @param ID      ���ID
 * @param MemAddr �Ĵ�����ʼ��ַ
 * @param wDat    Ҫд���16λ����
 * @return 1=�ɹ���0=ʧ��
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
 * @brief �Ӷ����ȡ����ֽ�
 * @param ID      ���ID
 * @param MemAddr �Ĵ�����ʼ��ַ
 * @param nData   ���ջ�����ָ��
 * @param nLen    Ҫ��ȡ���ֽ���
 * @return ʵ�ʶ�ȡ���ֽ�����0=ʧ��
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
 * @brief �Ӷ����ȡ1���ֽ�
 * @param ID      ���ID
 * @param MemAddr �Ĵ�����ַ
 * @return �ɹ����ض�ȡ���ֽ�ֵ��ʧ�ܷ���-1
 */
int sts_read_byte(uint8_t ID, uint8_t MemAddr)
{
    uint8_t bDat;
    int Size = sts_read(ID, MemAddr, &bDat, 1);
    if(Size != 1) return -1;
    return bDat;
}

/**
 * @brief �Ӷ����ȡ2���ֽڣ�16λ���ݣ�
 * @param ID      ���ID
 * @param MemAddr �Ĵ�����ʼ��ַ
 * @return �ɹ�����16λ���ݣ�ʧ�ܷ���-1
 */
int sts_read_word(uint8_t ID, uint8_t MemAddr)
{
    uint8_t nDat[2];
    int Size = sts_read(ID, MemAddr, nDat, 2);
    if(Size != 2) return -1;
    return SCS2Host(nDat[0], nDat[1]);
}


// ========== ͬ��дָ�� ==========

/**
 * @brief ͬ��дָ����ĺ���
 * @param ID      ���ID����
 * @param IDN     �������
 * @param MemAddr �Ĵ�����ʼ��ַ
 * @param nDat    ���ݻ�������ÿ�����nLen�ֽڣ�������ţ�
 * @param nLen    ÿ����������ݳ���
 * @note ���ж��ͬʱִ��ָ����ڶ���ͬ������
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

// ========== ���� API ==========

/**
 * @brief ��ʼ��STS���ͨ�ſ�
 * @note ����С��ģʽ������Ӧ����ս��ջ�����
 */
void sts_init(void)
{
    setEnd(0);      // С��ģʽ
    setLevel(1);    // ����Ӧ��
    u8Status = 0;
    u8Error = 0;
    rFlushSCS();
}

/**
 * @brief PINGָ�������Ƿ�����
 * @param ID ���ID��1~253��254Ϊ�㲥��
 * @return �ɹ����ض��ID��ʧ�ܷ���-1��������ͨ��getLastError()��ȡ��
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
 * @brief  STS ���λ�ÿ��ƺ����������ٶȺ��ٶȣ�
 * @param  ID       ��� ID �ţ�1~253��254 Ϊ�㲥 ID��
 * @param  Position Ŀ��λ�ã���Χ 0~4095����Ӧ 0��~360�㣩
 *                  ע�����ֶ�����̿���Ϊ 0~1000�������ʵ�ʶ���ͺ�ȷ��
 * @param  Speed    Ŀ���ٶȣ���Χ 0~3400��ֵԽ���ٶ�Խ�죩
 *                  0 ��ʾʹ�ö���ڲ�Ĭ���ٶ�
 * @param  ACC      ���ٶȣ���Χ 0~254��ֵԽ�����Խ�죩
 *                  0 ��ʾʹ�ö���ڲ�Ĭ�ϼ��ٶ�
 * @retval ���� 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ���ͨ�� getLastError() ��ȡ�����룩
 * @note   ʹ��ǰ���ȵ��� sts_enable_torque() ʹ��Ť�أ�����������
 * @note   λ�á��ٶȡ����ٶȵļĴ�����ַ������ STS_ACC��STS_GOAL_POSITION_L �Ⱥ���
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
 * @brief ͬ��дλ��ָ�������ͬʱ������
 * @param ID[]     ���ID����
 * @param IDN      �������
 * @param Position[] Ŀ��λ������
 * @param Speed[]    Ŀ���ٶ�����
 * @param ACC[]      ���ٶ�����
 * @note ���ж��ͬʱ��ʼ�˶���ʵ��ͬ������
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
 * @brief ʹ�ܶ��Ť�أ��ö��������
 * @param ID ���ID
 * @note ʹ�ܺ����ܱ���λ�ò��ֿ�����
 */
void sts_enable_torque(uint8_t ID)
{
    sts_write_byte(ID, STS_TORQUE_ENABLE, 1);
}

/**
 * @brief �رն��Ť�أ����������������ת����
 * @param ID ���ID
 */
void sts_disable_torque(uint8_t ID)
{
    sts_write_byte(ID, STS_TORQUE_ENABLE, 0);
}

/**
 * @brief ��ȡ�����ǰ�¶�
 * @param ID ���ID
 * @return �ɹ������¶�ֵ�����϶ȣ���ʧ�ܷ���-1
 */
int sts_read_temp(uint8_t ID)
{
    return sts_read_byte(ID, STS_PRESENT_TEMPERATURE);
}

/**
 * @brief ��ȡ�����ǰλ��
 * @param ID ���ID
 * @return �ɹ�����λ��ֵ��0~4095����ʧ�ܷ���-1
 */
int sts_read_pos(uint8_t ID)
{
    return sts_read_word(ID, STS_PRESENT_POSITION_L);
}

