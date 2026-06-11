#include "bsp/bsp_dwt.h"
#include "gd32f30x.h"         /* CoreDebug, DWT */

/* ===================================================================
 *  全局变量（可被外部访问，用于串口/TFT 显示）
 * =================================================================== */
volatile uint32_t dwt_adc_start = 0;
volatile uint32_t dwt_adc_min   = 0xFFFFFFFF;
volatile uint32_t dwt_adc_max   = 0;
volatile uint32_t dwt_adc_last  = 0;
volatile uint32_t dwt_adc_cnt   = 0;
volatile uint32_t dwt_adc_accum = 0;

/* 主循环空闲测量 */
volatile uint32_t dwt_idle_start = 0;
volatile uint32_t dwt_idle_accum = 0;

/* CPU 占用率测量用（基于 ADC ISR 累加，其他中断影响很小） */
static volatile uint32_t prev_cycles = 0;
static volatile uint32_t prev_accum  = 0;

/* ===================================================================
 *  初始化 DWT 周期计数器
 * =================================================================== */
void DWT_Init(void)
{
    /* 使能 DWT 跟踪 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* 复位计数器 */
    DWT->CYCCNT = 0;
    /* 启动周期计数 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ===================================================================
 *  读取当前周期计数值
 * =================================================================== */
uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

/* ===================================================================
 *  CPU 占用率计算
 *  基于 ADC ISR 占用的周期比例（20kHz 是最大中断源）
 *  返回: 0~1000 = 0.0%~100.0%
 * =================================================================== */
uint32_t DWT_CalcCPUUsage(void)
{
    uint32_t now  = DWT->CYCCNT;
    uint32_t accum = dwt_adc_accum;
    uint32_t delta_accum = accum - prev_accum;
    uint32_t delta_total = now - prev_cycles;

    prev_cycles = now;
    prev_accum  = accum;

    if (delta_total == 0) return 0;

    uint32_t usage = (uint64_t)delta_accum * 1000 / delta_total;
    if (usage > 1000) usage = 1000;
    return usage;
}

/* ===================================================================
 *  获取 ADC ISR 执行时间统计
 *  单位: 微秒 (120MHz: 1µs = 120 cycles)
 * =================================================================== */
void DWT_GetAdcStats(float *avg_us, float *min_us, float *max_us, float *last_us)
{
    uint32_t cnt = dwt_adc_cnt;
    uint32_t sum = dwt_adc_accum;

    if (avg_us) *avg_us = (cnt > 0) ? (float)sum / cnt / 120.0f : 0.0f;
    if (min_us) *min_us = (dwt_adc_min != 0xFFFFFFFF) ? (float)dwt_adc_min / 120.0f : 0.0f;
    if (max_us) *max_us = (float)dwt_adc_max / 120.0f;
    if (last_us)*last_us = (float)dwt_adc_last / 120.0f;
}
