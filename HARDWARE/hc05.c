#include "hc05.h"

#include "stdio.h"

void HC05_Printf(const char *format, ...)
{
    char buff[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);
    
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), HAL_MAX_DELAY);
}


void HC05_Printf_IT(const char *format, ...)
{
    /*
     * 必须使用静态缓冲区。
     * HAL_UART_Transmit_IT返回后，串口仍在使用这个缓冲区。
     */
    static char buff[160];
    /*
     * 上一次还没发送完就放弃本次输出，
     * 防止阻塞速度控制。
     */
    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    {
        return;
    }
    va_list args;
    va_start(args, format);
    int len = vsnprintf(
        buff,
        sizeof(buff),
        format,
        args
    );
    va_end(args);
    if (len <= 0)
    {
        return;
    }
    if (len >= (int)sizeof(buff))
    {
        len = sizeof(buff) - 1;
    }
    HAL_UART_Transmit_IT(
        &huart1,
        (uint8_t *)buff,
        (uint16_t)len
    );
}
