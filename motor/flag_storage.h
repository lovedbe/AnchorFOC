#ifndef FLAG_STORAGE_H
#define FLAG_STORAGE_H

#include <stdint.h>

/* Flash 地址：最后 2KB 页 0x0801F800 ~ 0x0801FFFF */
#define FLAG_FLASH_ADDR     0x0801F800
#define FLAG_MAGIC          0xCA1B2017

typedef struct FlagData {
    uint32_t magic;              /* 魔数校验 */
    uint8_t  motor_calibrated;   /* 1=已校准，0=未校准 */
    uint8_t  reserved;           /* 填充 */
    uint16_t adc_zero_u0;        /* ADC0 U相零点 */
    uint16_t adc_zero_v0;        /* ADC0 V相零点 */
    uint16_t adc_zero_w1;        /* ADC1 W相零点 */
    uint16_t adc_zero_v1;        /* ADC1 V相零点 */
    float    motor_r;            /* 相电阻 (ohm) */
    float    motor_l;            /* 相电感 (H) */
    float    gain_v1;            /* ADC1 V相增益修正 (ADC0_V/ADC1_V) */
    float    gain_vw1;            /* ADC1 W相增益修正 (ADC0_V/ADC1_W) */
    float    gain_uv0;           /* ADC0 U/V通道增益比 (ADC0_V/ADC0_U) */
    uint32_t checksum;
} g_stFlagData, *g_pstFlagDataPtr;

extern g_pstFlagDataPtr g_pstFlagData;

/* Flash 底层操作 */
int  Flash_Read(uint32_t addr, uint32_t *buf, uint32_t word_len);
int  Flash_Write(uint32_t addr, uint32_t *buf, uint32_t word_len);
int  Flash_PageErase(uint32_t addr);

/* 标志读写 */
int  Flag_Save(void);
int  Flag_Load(void);

#endif
