#include "fifo.h"
#include <string.h>

/*!
    \brief      注册 FIFO
    \param[in]  pfifo: FIFO 句柄
    \param[in]  buf: 缓冲区
    \param[in]  size: 缓冲区大小
    \param[in]  lock: 加锁函数
    \param[in]  unlock: 解锁函数
*/
void FIFO_Init(g_pstFifoPtr pfifo, uint8_t *buf, uint32_t size,
               fifo_lock_fun lock, fifo_lock_fun unlock)
{
    pfifo->buf         = buf;
    pfifo->buf_size    = size;
    pfifo->pwrite      = buf;
    pfifo->pread       = buf;
    pfifo->occupy_size = 0;
    pfifo->lock        = lock;
    pfifo->unlock      = unlock;
}

/*!
    \brief      释放 FIFO
*/
void FIFO_Deinit(g_pstFifoPtr pfifo)
{
    pfifo->buf_size    = 0;
    pfifo->occupy_size = 0;
    pfifo->buf         = NULL;
    pfifo->pwrite      = NULL;
    pfifo->pread       = NULL;
    pfifo->lock        = NULL;
    pfifo->unlock      = NULL;
}

/*!
    \brief      写入 FIFO
    \param[in]  data: 数据源
    \param[in]  size: 写入长度
    \retval     实际写入长度
*/
uint32_t FIFO_Write(g_pstFifoPtr pfifo, const uint8_t *data, uint32_t size)
{
    uint32_t free_size;

    if (size == 0 || pfifo == NULL || data == NULL)
        return 0;

    free_size = pfifo->buf_size - pfifo->occupy_size;
    if (free_size == 0)
        return 0;
    if (free_size < size)
        size = free_size;

    if (pfifo->lock) pfifo->lock();
    for (uint32_t i = 0; i < size; i++) {
        *pfifo->pwrite++ = *data++;
        if (pfifo->pwrite >= pfifo->buf + pfifo->buf_size)
            pfifo->pwrite = pfifo->buf;
        pfifo->occupy_size++;
    }
    if (pfifo->unlock) pfifo->unlock();
    return size;
}

/*!
    \brief      从 FIFO 读取
    \param[out] data: 目标缓冲
    \param[in]  size: 读取长度
    \retval     实际读取长度
*/
uint32_t FIFO_Read(g_pstFifoPtr pfifo, uint8_t *data, uint32_t size)
{
    uint32_t occupy_size;

    if (size == 0 || pfifo == NULL || data == NULL)
        return 0;

    occupy_size = pfifo->occupy_size;
    if (occupy_size == 0)
        return 0;
    if (occupy_size < size)
        size = occupy_size;

    if (pfifo->lock) pfifo->lock();
    for (uint32_t i = 0; i < size; i++) {
        *data++ = *pfifo->pread++;
        if (pfifo->pread >= pfifo->buf + pfifo->buf_size)
            pfifo->pread = pfifo->buf;
        pfifo->occupy_size--;
    }
    if (pfifo->unlock) pfifo->unlock();
    return size;
}

/*!
    \brief      获取 FIFO 有效数据大小
*/
uint32_t FIFO_GetOccupySize(g_pstFifoPtr pfifo)
{
    if (pfifo == NULL) return 0;
    return pfifo->occupy_size;
}

/*!
    \brief      获取 FIFO 空闲空间大小
*/
uint32_t FIFO_GetFreeSize(g_pstFifoPtr pfifo)
{
    if (pfifo == NULL) return 0;
    return pfifo->buf_size - pfifo->occupy_size;
}
