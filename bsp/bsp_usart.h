#ifndef USART_H
#define USART_H

#include <stdint.h>
#include "gd32f30x.h"

#define DebugUartPort           USART0
#define USART_BAUDRATE          115200

/* DMA 缓冲大小 */
#define UART_TX_BUF_SIZE        256
#define UART_RX_BUF_SIZE        256
#define UART_DMA_RX_BUF_SIZE    128
#define UART_DMA_TX_BUF_SIZE    256

/* 命令包（兼容旧协议） */
#define UART_CMD_DATA_BUFF_SIZE  12
#define PACKET_START             0xAA5555AA

/* ---- 结构体 ---- */
typedef struct SystemData {
    volatile uint32_t SystemTimes;
} g_stSystemData, *g_pstSystemDataPtr;

extern g_pstSystemDataPtr g_pstSystemData;

/* ---- API ---- */
void USART0_Init(void);
void UartPrintf(const char *fmt, ...);

/* 非阻塞读写（通过 FIFO） */
uint32_t UartWrite(const uint8_t *data, uint32_t size);
uint32_t UartRead(uint8_t *data, uint32_t size);
uint32_t UartReadable(void);

/* 主循环轮询：启动 TX DMA */
void UartPollDmaTx(void);

/* 命令处理（兼容旧协议 0xAA5555AA） */
void UartCmdHandle(void);

#endif
