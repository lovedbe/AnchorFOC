#include "motor/flag_storage.h"
#include "gd32f30x_fmc.h"
#include <stdlib.h>
#include <string.h>
#include "bsp/bsp_usart.h"

/* ===== Flash 底层操作封装 ===== */

int Flash_Read(uint32_t addr, uint32_t *buf, uint32_t word_len)
{
    volatile uint32_t *p = (volatile uint32_t *)addr;
    for (uint32_t i = 0; i < word_len; i++) {
        buf[i] = p[i];
    }
    return 0;
}

int Flash_Write(uint32_t addr, uint32_t *buf, uint32_t word_len)
{
    fmc_unlock();
    for (uint32_t i = 0; i < word_len; i++) {
        if (fmc_word_program(addr + i * 4, buf[i]) != FMC_READY) {
            fmc_lock();
            return -1;
        }
    }
    fmc_lock();
    return 0;
}

int Flash_PageErase(uint32_t addr)
{
    fmc_unlock();
    fmc_state_enum st = fmc_page_erase(addr);
    fmc_lock();
    return (st == FMC_READY) ? 0 : -1;
}

/* ===== 标志读写 ===== */

int Flag_Save(void)
{
    if (!g_pstFlagData) return -1;

    /* 计算校验和 */
    uint32_t sum = 0;
    uint32_t *p = (uint32_t *)g_pstFlagData;
    for (uint32_t i = 0; i < sizeof(g_stFlagData) / 4; i++)
        sum += p[i];
    /* 用 magic 替换 checksum 字段的值再算一次（checksum 本身排除） */
    g_pstFlagData->checksum = 0;
    sum = 0;
    for (uint32_t i = 0; i < sizeof(g_stFlagData) / 4; i++)
        sum += p[i];
    g_pstFlagData->checksum = sum;

    /* 擦除页 → 写入 */
    if (Flash_PageErase(FLAG_FLASH_ADDR) != 0) return -1;
    if (Flash_Write(FLAG_FLASH_ADDR, (uint32_t *)g_pstFlagData,
                    sizeof(g_stFlagData) / 4) != 0) return -1;
    return 0;
}

int Flag_Load(void)
{
    volatile g_stFlagData *p = (volatile g_stFlagData *)FLAG_FLASH_ADDR;

    if (p->magic != FLAG_MAGIC) return -1;

    uint32_t sum = 0;
    uint32_t len = sizeof(g_stFlagData) / 4;
    /* 用临时变量算 checksum（排除 p->checksum 自身） */
    for (uint32_t i = 0; i < len; i++) {
        if ((uint32_t)(&p->checksum) == (uint32_t)(&((volatile uint32_t *)p)[i]))
            continue;
        sum += ((volatile uint32_t *)p)[i];
    }
    if (sum != p->checksum) return -1;

    if (g_pstFlagData)
        memcpy(g_pstFlagData, (void *)p, sizeof(g_stFlagData));
    return 0;
}
