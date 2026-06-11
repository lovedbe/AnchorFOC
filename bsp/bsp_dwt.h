#ifndef BSP_DWT_H
#define BSP_DWT_H

#include <stdint.h>

/* ===================================================================
 *  DWT 周期计数器初始化
 *  使能 Cortex-M4 DWT CYCCNT，提供 32bit 周期计数 @120MHz
 *  每周期 = 8.33ns，溢出约 35.8 秒
 * =================================================================== */
void DWT_Init(void);

/* 读取当前 CYCCNT 值 */
uint32_t DWT_GetCycles(void);

/* ===================================================================
 *  ADC 中断执行时间测量
 *  在 ISR 入口/出口调用宏，自动记录 min/max/last/avg
 * =================================================================== */
extern volatile uint32_t dwt_adc_min;
extern volatile uint32_t dwt_adc_max;
extern volatile uint32_t dwt_adc_last;
extern volatile uint32_t dwt_adc_cnt;
extern volatile uint32_t dwt_adc_accum;   /* 累加周期数，用于 CPU% 计算 */

#define DWT_ADC_START()    do { dwt_adc_start = DWT->CYCCNT; } while(0)
#define DWT_ADC_END()      do { \
    uint32_t _now = DWT->CYCCNT; \
    uint32_t _delta = _now - dwt_adc_start; \
    if (_delta > DWT_ADC_MIN_VALID) { \
        if (_delta < dwt_adc_min) dwt_adc_min = _delta; \
        if (_delta > dwt_adc_max) dwt_adc_max = _delta; \
        dwt_adc_last = _delta; \
        dwt_adc_cnt++; \
        dwt_adc_accum += _delta; \
    } \
} while(0)

/* 内部 start 变量（不要在外部直接使用） */
extern volatile uint32_t dwt_adc_start;

/* ISR 有效计数的下限（低于此值视为初次启动/测量无效，不计入统计） */
#define DWT_ADC_MIN_VALID    100   /* ~0.83µs @120MHz */

/* ===================================================================
 *  主循环空闲测量（用于 CPU 占用率计算）
 *  在 while(1) 开头 DWT_IDLE_START()，末尾 DWT_IDLE_END()
 *  CPU% = 100% - (主循环时间/总时间)
 * =================================================================== */
extern volatile uint32_t dwt_idle_start;
extern volatile uint32_t dwt_idle_accum;

#define DWT_IDLE_START()     do { dwt_idle_start = DWT->CYCCNT; } while(0)
#define DWT_IDLE_END()       do { \
    uint32_t _now = DWT->CYCCNT; \
    dwt_idle_accum += _now - dwt_idle_start; \
} while(0)

/* ===================================================================
 *  CPU 占用率计算
 *  基于 DWT 周期计数，返回 ADC ISR 占用百分比 (0~1000 = 0.0%~100.0%)
 *  调用后自动重置累加器
 * =================================================================== */
uint32_t DWT_CalcCPUUsage(void);

/* ===================================================================
 *  获取 ADC ISR 执行时间统计（微秒）
 * =================================================================== */
void DWT_GetAdcStats(float *avg_us, float *min_us, float *max_us, float *last_us);

#endif /* BSP_DWT_H */
