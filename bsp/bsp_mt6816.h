#ifndef MT6816_H
#define MT6816_H

#include <stdint.h>
#include <stdbool.h>
#include "app/usr_config.h"

/* MT6816STD 14-bit 磁编码器 — 分辨率来自 usr_config.h */
#define MT6816_RESOLUTION    ENCODER_RESOLUTION
#define MT6816_MAX_COUNT     16383
#define MT6816_CNT_TO_DEG(x) ((float)(x) * 360.0f / MT6816_RESOLUTION)

/* 寄存器读指令: 最高位为 1 表示读, 低 7 位为地址 */
#define MT6816_CMD_READ(addr)  (0x80 | (addr))
#define MT6816_REG_ANGLE_HIGH  0x03   /* Angle<13:6> */
#define MT6816_REG_ANGLE_LOW   0x04   /* Angle<5:0> | No_Mag_Warning(bit1) | PC(bit0) */

/*
 * 引脚映射 (SPI2 remap): CS PA15, SCK PB3, MISO PB4, MOSI PB5
 * CS 软件控制
 */
#define MT6816_CS_PORT       GPIOA
#define MT6816_CS_PIN        GPIO_PIN_15
#define MT6816_CS_CLK        RCU_GPIOA

#define MT6816_SCK_PORT      GPIOB
#define MT6816_SCK_PIN       GPIO_PIN_3
#define MT6816_MISO_PORT     GPIOB
#define MT6816_MISO_PIN      GPIO_PIN_4
#define MT6816_MOSI_PORT     GPIOB
#define MT6816_MOSI_PIN      GPIO_PIN_5
#define MT6816_SPI_CLK       RCU_GPIOB

#define CS_LOW()    GPIO_BC(MT6816_CS_PORT)  = MT6816_CS_PIN
#define CS_HIGH()   GPIO_BOP(MT6816_CS_PORT) = MT6816_CS_PIN

/* 读取结果 */
typedef struct {
    uint16_t pos;            /* 14 位绝对位置 (0~16383) */
    bool     parity_ok;      /* 偶校验通过 */
    bool     no_mag;         /* 无磁告警 */
} MT6816_Data_t;

/* ---- 驱动接口 ---- */
void    MT6816_Init(void);
bool    MT6816_Read(MT6816_Data_t *d);
float   MT6816_GetAngle(void);           /* 简易: 返回角度(度), 校验失败沿用上次值 */
void    MT6816_ReadDebug(uint16_t *rx1, uint16_t *rx2);  /* 调试: 返回原始 SPI 数据 */

#endif
