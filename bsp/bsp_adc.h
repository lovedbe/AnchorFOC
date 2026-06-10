/*
 * @Author: ntw48 2476672877@qq.com
 * @Date: 2026-03-14 15:36:04
 * @LastEditors: ntw48 2476672877@qq.com
 * @LastEditTime: 2026-03-20 21:38:16
 * @FilePath: \SVPWM_HALL\drivers\adc.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __ADC_H
#define __ADC_H

#include "gd32f30x.h"
#include "app/usr_config.h"

/* ================== ADC 采样通道映射（根据硬件引脚修改） ================== */
#define ADC_CHANNEL_U    ADC_CHANNEL_1  /* PA1 */
#define ADC_CHANNEL_V    ADC_CHANNEL_2  /* PA2 */
#define ADC_CHANNEL_W    ADC_CHANNEL_3  /* PA3 */

/* ================== 电流采样（值来自 usr_config.h） ================== */
#define CURRENT_AMP_GAIN        (CURRENT_AMP_R_TOP / CURRENT_AMP_R_BOT)
#define CURRENT_ADC_RES         4096.0f     /* 12-bit ADC 分辨率           */
#define CURRENT_CONV_FACTOR     (CURRENT_VDDA / (CURRENT_ADC_RES * CURRENT_SHUNT_R_OHM * CURRENT_AMP_GAIN))

/* ---------------- ADC FOC 相关函数声明 ---------------- */

/**
 * @brief  ADC初始化（注入组+TIMER0 TRGO触发+中断，适配FOC）
 */
void ADC_FOC_Init(void);

/**
 * @brief  按扇区动态配置ADC注入组通道
 * @param  sector: 扇区号（1~6）
 */
void ADC_Config_By_Sector(uint8_t sector);

/**
 * @brief  获取三相电流（注入组采样+计算）
 * @param  i_u/i_v/i_w: 输出三相电流（int16_t）
 */
void ADC_Get_Three_Phase_Current(int16_t *i_u, int16_t *i_v, int16_t *i_w);

/** @brief 编码器校准状态机数据 */
typedef struct EncCalibData
{
    uint8_t  step;
    uint32_t t_start;
    float    lock_mod;
    uint16_t enc_locked_pos;
    uint16_t enc_pp_start;
    uint16_t enc_pp_end;
    uint32_t t_print;
    int32_t  sample_idx;
    int32_t  total_samples;
    uint16_t enc_lut_start;
    int8_t   calib_dir;
    uint8_t  lut_print_line;
} g_stEncCalibData, *g_pstEncCalibDataPtr;

extern g_pstEncCalibDataPtr g_pstEncCalibData;

#endif // __ADC_H
