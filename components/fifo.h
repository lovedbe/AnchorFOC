#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>
#include <stdbool.h>

/* 环形缓冲区 */

typedef void (*fifo_lock_fun)(void);

typedef struct {
    uint8_t  *buf;            /* 缓冲区起始地址 */
    uint32_t  buf_size;       /* 缓冲区大小 */
    uint32_t  occupy_size;    /* 有效数据大小 */
    uint8_t  *pwrite;         /* 写指针 */
    uint8_t  *pread;          /* 读指针 */
    void (*lock)(void);       /* 锁回调 */
    void (*unlock)(void);     /* 解锁回调 */
} g_stFifo, *g_pstFifoPtr;

void    FIFO_Init(g_pstFifoPtr pfifo, uint8_t *buf, uint32_t size,
                  fifo_lock_fun lock, fifo_lock_fun unlock);
void    FIFO_Deinit(g_pstFifoPtr pfifo);
uint32_t FIFO_Write(g_pstFifoPtr pfifo, const uint8_t *data, uint32_t size);
uint32_t FIFO_Read(g_pstFifoPtr pfifo, uint8_t *data, uint32_t size);
uint32_t FIFO_GetOccupySize(g_pstFifoPtr pfifo);
uint32_t FIFO_GetFreeSize(g_pstFifoPtr pfifo);

#endif
