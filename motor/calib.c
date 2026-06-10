#include "motor/calib.h"
#include "gd32f30x_fmc.h"
#include "gd32f30x_adc.h"
#include "gd32f30x_timer.h"
#include "motor/motor.h"
#include "motor/control.h"
#include "motor/svpwm.h"
#include "motor/flag_storage.h"
#include "bsp/bsp_mt6816.h"
#include "bsp/bsp_adc.h"
#include "bsp/bsp_timer.h"
#include "bsp/bsp_usart.h"
#include "bsp/bsp_gpio.h"
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* 校准数据 Flash 地址：0x0801F000 (倒数第 4KB 区域) */
#define ENC_CALIB_FLASH_ADDR  0x0801F000

typedef struct {
    uint32_t magic;
    int16_t  encoder_offset;
    int8_t   encoder_dir;
    uint8_t  change_dir;
    uint8_t  pole_pairs;
    int16_t  offset_lut[128];
    uint32_t checksum;
} EncFlashData_t;

static int EncCalib_SaveToFlash(void)
{
    EncFlashData_t data;
    uint32_t sum = 0;

    data.magic          = 0xCA1B2016;
    data.encoder_offset = g_pstMotorData->encoder_offset;
    data.encoder_dir    = g_pstMotorData->encoder_dir;
    data.change_dir     = g_pstMotorData->change_dir;
    data.pole_pairs     = g_pstMotorData->pole_pairs;
    memcpy(data.offset_lut, g_pstMotorData->offset_lut, sizeof(data.offset_lut));

    /*  =  checksum  */
    data.checksum = 0;
    for (int i = 0; i < sizeof(EncFlashData_t) / 4; i++)
        sum += ((uint32_t *)&data)[i];
    data.checksum = sum;

    fmc_unlock();
    fmc_page_erase(ENC_CALIB_FLASH_ADDR);
    for (int i = 0; i < sizeof(EncFlashData_t) / 4; i++)
        fmc_word_program(ENC_CALIB_FLASH_ADDR + i * 4, ((uint32_t *)&data)[i]);
    fmc_lock();
    return 0;
}

int EncCalib_LoadFromFlash(void)
{
    volatile EncFlashData_t *p = (volatile EncFlashData_t *)ENC_CALIB_FLASH_ADDR;
    if (p->magic != 0xCA1B2016) return -1;

    /*  checksum  */
    uint32_t sum = 0;
    int len = sizeof(EncFlashData_t) / 4;
    for (int i = 0; i < len; i++) {
        if ((uint32_t)(&p->checksum) == (uint32_t)(&((volatile uint32_t *)p)[i]))
            continue;
        sum += ((volatile uint32_t *)p)[i];
    }
    if (sum != p->checksum) {
        /*  Bug */
        UartPrintf("ENC: legacy checksum, accept by magic\r\n");
    }

    g_pstMotorData->encoder_offset = p->encoder_offset;
    g_pstMotorData->encoder_dir    = p->encoder_dir;
    g_pstMotorData->change_dir     = p->change_dir;
    g_pstMotorData->pole_pairs     = p->pole_pairs;
    memcpy(g_pstMotorData->offset_lut, (void *)p->offset_lut, sizeof(p->offset_lut));
    g_pstMotorData->lut_valid      = 1;
    return 0;
}

int CALIB_ADC_Run(void)
{
    static uint8_t step = 0;
    static uint32_t t_target = 0;
    static uint32_t v0_raw = 0, v1_raw = 0;
    static uint32_t u0_raw = 0, w1_raw = 0;

    uint32_t now = g_pstSystemData->SystemTimes;

    switch (step) {

    /* =====  ===== */
    case 0:
        TIMER_CH0CV(TIMER0) = 1500;
        TIMER_CH1CV(TIMER0) = 1500;
        TIMER_CH2CV(TIMER0) = 1500;
        adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
        adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_W, ADC_SAMPLETIME_13POINT5);
        t_target = now + 6;
        step = 1;
        break;

    case 1:
        if (now < t_target) break;
        g_pstMotorData->zero_calib_sum0 = 0;
        g_pstMotorData->zero_calib_sum1 = 0;
        g_pstMotorData->zero_calib_count = 0;
        g_pstMotorData->zero_calib_enabled = 1;
        t_target = now + 60;
        step = 2;
        break;

    case 2:
        if (now < t_target) break;
        g_pstMotorData->zero_calib_enabled = 0;
        if (g_pstMotorData->zero_calib_count > 0) {
            g_pstMotorData->adc_zero_v0 = g_pstMotorData->zero_calib_sum0 / g_pstMotorData->zero_calib_count;
            g_pstMotorData->adc_zero_w1 = g_pstMotorData->zero_calib_sum1 / g_pstMotorData->zero_calib_count;
        }
        adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_U, ADC_SAMPLETIME_13POINT5);
        adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
        t_target = now + 6;
        step = 3;
        break;

    case 3:
        if (now < t_target) break;
        g_pstMotorData->zero_calib_sum0 = 0;
        g_pstMotorData->zero_calib_sum1 = 0;
        g_pstMotorData->zero_calib_count = 0;
        g_pstMotorData->zero_calib_enabled = 1;
        t_target = now + 60;
        step = 4;
        break;

    case 4:
        if (now < t_target) break;
        g_pstMotorData->zero_calib_enabled = 0;
        if (g_pstMotorData->zero_calib_count > 0) {
            g_pstMotorData->adc_zero_u0 = g_pstMotorData->zero_calib_sum0 / g_pstMotorData->zero_calib_count;
            g_pstMotorData->adc_zero_v1 = g_pstMotorData->zero_calib_sum1 / g_pstMotorData->zero_calib_count;
        }
        adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
        adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_W, ADC_SAMPLETIME_13POINT5);
        TIMER_CH0CV(TIMER0) = 0;
        TIMER_CH1CV(TIMER0) = 0;
        TIMER_CH2CV(TIMER0) = 0;
        UartPrintf("zero calib: U0=%d V0=%d V1=%d W1=%d\r\n",
                   g_pstMotorData->adc_zero_u0, g_pstMotorData->adc_zero_v0,
                   g_pstMotorData->adc_zero_v1, g_pstMotorData->adc_zero_w1);
        step = 5;
        break;

    /* ===== Phase ADC:  ADC0  ADC1  ===== */
    case 5:
        TIMER_CH0CV(TIMER0) = PWM_PERIOD - 1;  /* U=100% */
        TIMER_CH1CV(TIMER0) = 0;                /* V=0% */
        TIMER_CH2CV(TIMER0) = PWM_PERIOD / 2;   /* W=50% */
        t_target = now + 12;
        step = 6;
        break;

    case 6:
        if (now < t_target) break;
        adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
        adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_W, ADC_SAMPLETIME_13POINT5);
        g_pstMotorData->zero_calib_sum0 = 0;
        g_pstMotorData->zero_calib_sum1 = 0;
        g_pstMotorData->zero_calib_count = 0;
        g_pstMotorData->zero_calib_enabled = 1;
        t_target = now + 30;
        step = 7;
        break;

    case 7:
        if (now < t_target) break;
        g_pstMotorData->zero_calib_enabled = 0;
        v0_raw = (g_pstMotorData->zero_calib_count > 0)
            ? g_pstMotorData->zero_calib_sum0 / g_pstMotorData->zero_calib_count : 0;
        adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_U, ADC_SAMPLETIME_13POINT5);
        adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
        g_pstMotorData->zero_calib_sum0 = 0;
        g_pstMotorData->zero_calib_sum1 = 0;
        g_pstMotorData->zero_calib_count = 0;
        g_pstMotorData->zero_calib_enabled = 1;
        t_target = now + 30;
        step = 8;
        break;

    case 8:
        if (now < t_target) break;
        g_pstMotorData->zero_calib_enabled = 0;
        v1_raw = (g_pstMotorData->zero_calib_count > 0)
            ? g_pstMotorData->zero_calib_sum1 / g_pstMotorData->zero_calib_count : 0;
        {
            int32_t sig0 = (int32_t)v0_raw - (int32_t)g_pstMotorData->adc_zero_v0;
            int32_t sig1 = (int32_t)v1_raw - (int32_t)g_pstMotorData->adc_zero_v1;
            float adc_gain = (sig1 != 0) ? (float)sig0 / (float)sig1 : 1.0f;
            if (adc_gain < 0.90f) adc_gain = 0.90f;
            if (adc_gain > 1.10f) adc_gain = 1.10f;
            g_pstMotorData->gain_v1 = adc_gain;
        }
        UartPrintf("calib ADC: v0=%d v1=%d adc_gain=%.4f\r\n", v0_raw, v1_raw, g_pstMotorData->gain_v1);
        TIMER_CH0CV(TIMER0) = 0;
        TIMER_CH1CV(TIMER0) = 0;
        TIMER_CH2CV(TIMER0) = 0;
        t_target = now + 300;
        step = 9;
        break;

    /* ===== Phase AMP_U:  U/V  ===== */
    case 9:
        if (now < t_target) break;
        TIMER_CH0CV(TIMER0) = 0;                /* U=0% */
        TIMER_CH1CV(TIMER0) = PWM_PERIOD / 2;   /* V=50% */
        TIMER_CH2CV(TIMER0) = PWM_PERIOD - 1;  /* W=100% */
        t_target = now + 12;
        step = 10;
        break;

    case 10:
        if (now < t_target) break;
        adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_U, ADC_SAMPLETIME_13POINT5);
        adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
        g_pstMotorData->zero_calib_sum0 = 0;
        g_pstMotorData->zero_calib_sum1 = 0;
        g_pstMotorData->zero_calib_count = 0;
        g_pstMotorData->zero_calib_enabled = 1;
        t_target = now + 30;
        step = 11;
        break;

    case 11:
        if (now < t_target) break;
        g_pstMotorData->zero_calib_enabled = 0;
        u0_raw = (g_pstMotorData->zero_calib_count > 0)
            ? g_pstMotorData->zero_calib_sum0 / g_pstMotorData->zero_calib_count : 0;
        {
            int32_t sig_v = (int32_t)v0_raw - (int32_t)g_pstMotorData->adc_zero_v0;
            int32_t sig_u = (int32_t)u0_raw - (int32_t)g_pstMotorData->adc_zero_u0;
            float amp_uv = (sig_u != 0) ? (float)sig_v / (float)sig_u : 1.0f;
            if (amp_uv < 0.90f) amp_uv = 0.90f;
            if (amp_uv > 1.10f) amp_uv = 1.10f;
            g_pstMotorData->gain_uv0 = amp_uv;
        }
        UartPrintf("calib AMP_U: v0=%d u0=%d amp_uv=%.4f\r\n", v0_raw, u0_raw, g_pstMotorData->gain_uv0);
        TIMER_CH0CV(TIMER0) = 0;
        TIMER_CH1CV(TIMER0) = 0;
        TIMER_CH2CV(TIMER0) = 0;
        t_target = now + 300;
        step = 12;
        break;

    /* ===== Phase AMP_W:  W/V  ===== */
    case 12:
        if (now < t_target) break;
        TIMER_CH0CV(TIMER0) = PWM_PERIOD - 1;  /* U=100% */
        TIMER_CH1CV(TIMER0) = PWM_PERIOD / 2;   /* V=50% */
        TIMER_CH2CV(TIMER0) = 0;                /* W=0% */
        t_target = now + 12;
        step = 13;
        break;

    case 13:
        if (now < t_target) break;
        adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
        adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_W, ADC_SAMPLETIME_13POINT5);
        g_pstMotorData->zero_calib_sum0 = 0;
        g_pstMotorData->zero_calib_sum1 = 0;
        g_pstMotorData->zero_calib_count = 0;
        g_pstMotorData->zero_calib_enabled = 1;
        t_target = now + 30;
        step = 14;
        break;

    case 14:
        if (now < t_target) break;
        g_pstMotorData->zero_calib_enabled = 0;
        w1_raw = (g_pstMotorData->zero_calib_count > 0)
            ? g_pstMotorData->zero_calib_sum1 / g_pstMotorData->zero_calib_count : 0;
        {
            int32_t sig_v = (int32_t)v1_raw - (int32_t)g_pstMotorData->adc_zero_v1;
            int32_t sig_w = (int32_t)w1_raw - (int32_t)g_pstMotorData->adc_zero_w1;
            float amp_wv = (sig_w != 0) ? (float)sig_v / (float)sig_w : 1.0f;
            if (amp_wv < 0.90f) amp_wv = 0.90f;
            if (amp_wv > 1.10f) amp_wv = 1.10f;
            g_pstMotorData->gain_vw1 = g_pstMotorData->gain_v1 * amp_wv;
        }
        UartPrintf("calib AMP_W: v1=%d w1=%d amp_wv=%.4f\r\n", v1_raw, w1_raw,
                   g_pstMotorData->gain_vw1 / g_pstMotorData->gain_v1);
        TIMER_CH0CV(TIMER0) = 0;
        TIMER_CH1CV(TIMER0) = 0;
        TIMER_CH2CV(TIMER0) = 0;
        adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
        adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_W, ADC_SAMPLETIME_13POINT5);
        step = 15;
        break;

    /* =====  ===== */
    case 15:
        step = 0;
        return 1;

    default:
        step = 0;
        break;
    }
    return 0;
}

/**
 * @brief  RL  +
 * @note    FocSystemRun::FOC_CALIB  0= 1=
 */
int CALIB_RL_Run(void)
{
    static uint8_t step = 0;
    static uint32_t t_target = 0;
    static float r_voltage;
    static float r_i_sum;
    static uint32_t r_i_cnt;

    uint32_t now = g_pstSystemData->SystemTimes;

    switch (step) {

    /* ===== PI  ===== */
    case 0:
        r_voltage = 0.0f;
        r_i_sum = 0.0f;
        r_i_cnt = 0;
        t_target = now + 100;
        step = 1;
        break;

    case 1:
        if (now < t_target) break;
        UartPrintf("RL calib: measuring resistance...\r\n");
        t_target = now + 2000;          /* PI  2  */
        step = 2;
        break;

    case 2:
    {
        /* PI  target_current */
        if (now < t_target) {
            const float target_i = 0.5f;        /*  (A) */
            const float ki      = 0.005f;        /*  ( ~1kHz main loop) */
            const float max_mod = 0.5f;          /*  */

            float err = target_i - g_pstMotorData->Iu;
            r_voltage += ki * err;
            if (r_voltage > max_mod)  r_voltage = max_mod;
            if (r_voltage < -max_mod) r_voltage = -max_mod;

            /*  200ms  Iu  */
            if (now >= t_target - 200) {
                r_i_sum += g_pstMotorData->Iu;
                r_i_cnt++;
            }

            float ccr_u, ccr_v, ccr_w;
            SVPWM_Calc_Cartesian(r_voltage, 0.0f, &ccr_u, &ccr_v, &ccr_w);
            TIMER_CH0CV(TIMER0) = (uint16_t)(ccr_u * PWM_PERIOD);
            TIMER_CH1CV(TIMER0) = (uint16_t)(ccr_v * PWM_PERIOD);
            TIMER_CH2CV(TIMER0) = (uint16_t)(ccr_w * PWM_PERIOD);
            ADC_Config_By_Sector(g_pstMotorData->sector);
            break;  /*  PI */
        }
        /* 2    */
        {
            float vbus = 12.0f;
            float v_actual = r_voltage * (2.0f / 3.0f) * vbus;
            float r_phase = fabsf(v_actual / 0.5f);
            float i_avg = (r_i_cnt > 0) ? r_i_sum / r_i_cnt : 0.0f;
            g_pstMotorData->motor_r = r_phase;
            UartPrintf("R calib: R=%.4f ohm, V=%.4f, I_avg=%.4f\r\n",
                       r_phase, r_voltage, i_avg);
        }
        TIMER_CH0CV(TIMER0) = 0;
        TIMER_CH1CV(TIMER0) = 0;
        TIMER_CH2CV(TIMER0) = 0;
        step = 3;
        break;
    }

    case 3:
        t_target = now + 500;
        step = 4;
        break;

    /* ===== 50s ISR  ===== */
    /* Phase A1 */
    case 4:
        if (now < t_target) break;
        g_pstMotorData->l_alt_phase = 0;
        g_pstMotorData->l_alt_sum_pos = 0.0f;
        g_pstMotorData->l_alt_sum_neg = 0.0f;
        g_pstMotorData->l_alt_count = 0;
        g_pstMotorData->l_alt_enabled = 1;
        UartPrintf("L calib: stabilizing...\r\n");
        t_target = now + 1000;
        step = 5;
        break;

    /* Phase B1 */
    case 5:
        if (now < t_target) break;
        g_pstMotorData->l_alt_sum_pos = 0.0f;
        g_pstMotorData->l_alt_sum_neg = 0.0f;
        g_pstMotorData->l_alt_count = 0;
        UartPrintf("L calib: measuring...\r\n");
        t_target = now + 1000;
        step = 6;
        break;

    case 6:
        if (now < t_target) break;
        g_pstMotorData->l_alt_enabled = 0;
        {
            float vbus = 12.0f;
            float v_actual = 0.6f * (2.0f / 3.0f) * vbus;
            uint32_t n = g_pstMotorData->l_alt_count;
            if (n > 10) {
                float di = g_pstMotorData->l_alt_sum_pos - g_pstMotorData->l_alt_sum_neg;
                float half_n = (float)(n / 2);
                float dt = 0.00005f * half_n;
                float di_dt = (dt > 0.0001f) ? di / dt : 0.0f;
                float i_avg = (half_n > 0.0f) ? g_pstMotorData->l_alt_sum_pos / half_n : 0.0f;
                float v_drive = v_actual - g_pstMotorData->motor_r * fabsf(i_avg);
                if (v_drive < 0.1f) v_drive = 0.1f;
                float l = (fabsf(di_dt) > 0.001f) ? v_drive / fabsf(di_dt) * (2.0f / 3.0f) : 0.0f;
                g_pstMotorData->motor_l = l;
                UartPrintf("L calib: L=%.6f H (%.4f mH), di=%.4f, di/dt=%.1f, I_avg=%.4f, V_drive=%.4f\r\n",
                           l, l * 1000.0f, di, di_dt, i_avg, v_drive);
                UartPrintf("L calib: L=%.6f H (%.4f mH), cnt=%d\r\n", l, l * 1000.0f, n);
            } else {
                UartPrintf("L calib: failed (cnt=%d)\r\n", n);
            }
        }
        TIMER_CH0CV(TIMER0) = 0;
        TIMER_CH1CV(TIMER0) = 0;
        TIMER_CH2CV(TIMER0) = 0;
        step = 7;
        break;

    /* =====  ===== */
    case 7:
        step = 0;
        return 1;

    default:
        step = 0;
        break;
    }
    return 0;
}
int CALIB_ENC_Run(void)
{
    if (g_pstEncCalibData == NULL) return 0;

    /* error_sum  .bss */
    static int16_t *error_sum = NULL;

    const float VBUS = 12.0f;           /*  (V) */

    uint32_t now = g_pstSystemData->SystemTimes;  /*  (ms) */

    switch (g_pstEncCalibData->step)
    {

    /*=====================================================================
     * 2 
     *
     *    Vd  = 0
     *    A  0 
     *
     *    2s: mod(t) = g_pstEncCalibData->lock_mod  t/2    
     *   2s : mod = g_pstEncCalibData->lock_mod              
     *=====================================================================*/
    case 0:
    {
        float v_target = 0.5f * g_pstMotorData->motor_r * 1.5f;
        g_pstEncCalibData->lock_mod = v_target / ((2.0f / 3.0f) * VBUS);
        if (g_pstEncCalibData->lock_mod > 0.8f)  g_pstEncCalibData->lock_mod = 0.8f;
        if (g_pstEncCalibData->lock_mod < 0.15f) g_pstEncCalibData->lock_mod = 0.15f;

        g_pstMotorData->pole_pairs = 7;

        g_pstEncCalibData->t_start = now;
        g_pstEncCalibData->step = 1;
        break;
    }

    case 1:
    {
        float elapsed = (now - g_pstEncCalibData->t_start) / 1000.0f;
        float ramp    = (elapsed < 2.0f) ? (elapsed / 2.0f) : 1.0f;
        float mod     = g_pstEncCalibData->lock_mod * ramp;

        float a, b, c;
        SVPWM_Calc_Cartesian(mod, 0.0f, &a, &b, &c);

        TIMER_CH0CV(TIMER0) = (uint16_t)(a * PWM_PERIOD);
        TIMER_CH1CV(TIMER0) = (uint16_t)(b * PWM_PERIOD);
        TIMER_CH2CV(TIMER0) = (uint16_t)(c * PWM_PERIOD);
        ADC_Config_By_Sector(g_pstMotorData->sector);

        if (elapsed >= 2.0f) {
            g_pstEncCalibData->enc_locked_pos = g_pstMotorData->enc_raw_cache;
            g_pstEncCalibData->enc_pp_start = g_pstMotorData->enc_raw_cache;
            UartPrintf("ENC: locked=%d start=%d\r\n", g_pstEncCalibData->enc_locked_pos, g_pstEncCalibData->enc_pp_start);

            g_pstEncCalibData->t_start = now;
            g_pstEncCalibData->step = 2;
        }
        break;
    }

    /*=====================================================================
     *  2 
     *
     *   theta_elec = 360  t pp 
     *   
     *=====================================================================*/
    case 2:
    {
        float elapsed = (now - g_pstEncCalibData->t_start) / 1000.0f;

        /* 1 /s */
        float theta_elec = 360.0f * elapsed;
        float s, c;
        fast_sin_cos(theta_elec, &s, &c);

        float ua = g_pstEncCalibData->lock_mod * c;
        float ub = g_pstEncCalibData->lock_mod * s;
        Us_Limit(&ua, &ub, 0.92f);

        float a, b, ccr;
        SVPWM_Calc_Cartesian(ua, ub, &a, &b, &ccr);
        TIMER_CH0CV(TIMER0) = (uint16_t)(a * PWM_PERIOD);
        TIMER_CH1CV(TIMER0) = (uint16_t)(b * PWM_PERIOD);
        TIMER_CH2CV(TIMER0) = (uint16_t)(ccr * PWM_PERIOD);
        ADC_Config_By_Sector(g_pstMotorData->sector);

        /*  500ms  */
        if (now - g_pstEncCalibData->t_print >= 500) {
            uint16_t pos = g_pstMotorData->enc_raw_cache;
            float deg = (float)pos * 360.0f / MT6816_RESOLUTION;
            UartPrintf("ENC: t=%.1fs pos=%d (%.1f)\r\n", elapsed, pos, deg);
            g_pstEncCalibData->t_print = now;
        }

        /* 2    */
        if (elapsed >= 2.0f) {
            g_pstEncCalibData->enc_pp_end = g_pstMotorData->enc_raw_cache;
            g_pstEncCalibData->step = 3;
        }
        break;
    }

    /*=====================================================================
     *  +  LUT 
     *=====================================================================*/
    case 3:
    {
        int32_t diff = (int32_t)g_pstEncCalibData->enc_pp_end - (int32_t)g_pstEncCalibData->enc_pp_start;
        if (diff > 8191)  diff -= 16384;
        if (diff < -8191) diff += 16384;

        int dir = (diff >= 0) ? 1 : -1;
        g_pstEncCalibData->calib_dir = (int8_t)dir;
        UartPrintf("ENC: start=%d end=%d diff=%+d dir=%+d",
                   g_pstEncCalibData->enc_pp_start, g_pstEncCalibData->enc_pp_end, diff, dir);

        if (diff != 0) {
            int32_t abs_diff = (diff >= 0) ? diff : -diff;
            int pp_meas = (2 * 16384 + abs_diff / 2) / abs_diff;
            UartPrintf(" pp_meas=%d (cfg=%d) -> LUT\r\n", pp_meas, g_pstMotorData->pole_pairs);
        } else {
            UartPrintf(" pp=ERR\r\n");
            g_pstEncCalibData->step = 0;
            return 1;
        }

        /*  DC  PWM */
        {
            float a, b, c;
            SVPWM_Calc_Cartesian(g_pstEncCalibData->lock_mod, 0.0f, &a, &b, &c);
            TIMER_CH0CV(TIMER0) = (uint16_t)(a * PWM_PERIOD);
            TIMER_CH1CV(TIMER0) = (uint16_t)(b * PWM_PERIOD);
            TIMER_CH2CV(TIMER0) = (uint16_t)(c * PWM_PERIOD);
        }

        /*  LUT  */
        error_sum = (int16_t *)malloc(g_pstMotorData->pole_pairs * 128 * sizeof(int16_t));
        if (error_sum == NULL) {
            UartPrintf("ENC: LUT malloc fail\r\n");
            g_pstEncCalibData->step = 0;
            return 1;
        }
        g_pstEncCalibData->sample_idx = 0;
        g_pstEncCalibData->total_samples = (int32_t)g_pstMotorData->pole_pairs * 128;
        g_pstEncCalibData->enc_lut_start = g_pstEncCalibData->enc_pp_end;

        g_pstEncCalibData->t_start = now;
        g_pstEncCalibData->step = 4;
        break;
    }

    /*=====================================================================
     *  LUT 1 pp128 
     *
     *    360/128 = 2.8125 
     *    = mech_angle  16384 / 360
     *    =  -  14 
     *=====================================================================*/
    case 4:
    {
        float elapsed = (now - g_pstEncCalibData->t_start) / 1000.0f;
        float theta_elec = (float)g_pstEncCalibData->calib_dir * 360.0f * elapsed;

        /*  */
        float s, c;
        fast_sin_cos(theta_elec, &s, &c);
        float ua = g_pstEncCalibData->lock_mod * c;
        float ub = g_pstEncCalibData->lock_mod * s;
        Us_Limit(&ua, &ub, 0.92f);

        float a, b, ccr;
        SVPWM_Calc_Cartesian(ua, ub, &a, &b, &ccr);
        TIMER_CH0CV(TIMER0) = (uint16_t)(a * PWM_PERIOD);
        TIMER_CH1CV(TIMER0) = (uint16_t)(b * PWM_PERIOD);
        TIMER_CH2CV(TIMER0) = (uint16_t)(ccr * PWM_PERIOD);
        ADC_Config_By_Sector(g_pstMotorData->sector);

        /*  theta_elec  */
        float theta_abs = fabsf(theta_elec);
        int desired = (int)(theta_abs / (360.0f / 128.0f));
        while (g_pstEncCalibData->sample_idx <= desired && g_pstEncCalibData->sample_idx < g_pstEncCalibData->total_samples) {
            float theta_at_sample = (float)g_pstEncCalibData->sample_idx * (360.0f / 128.0f)
                                    * (float)g_pstEncCalibData->calib_dir;
            float mech_deg = theta_at_sample / (float)g_pstMotorData->pole_pairs;
            uint16_t expected = (uint16_t)(mech_deg * 16384.0f / 360.0f);

            int32_t err = (int32_t)g_pstMotorData->enc_raw_cache - (int32_t)expected;
            if (err > 8191)  err -= 16384;
            if (err < -8191) err += 16384;
            error_sum[g_pstEncCalibData->sample_idx] = (int16_t)err;
            g_pstEncCalibData->sample_idx++;
        }

        if (g_pstEncCalibData->sample_idx >= g_pstEncCalibData->total_samples) {
            UartPrintf("ENC: CW LUT done (%d samples)\r\n", g_pstEncCalibData->total_samples);

            /*  DC  */
            {
                float x, y, z;
                SVPWM_Calc_Cartesian(g_pstEncCalibData->lock_mod, 0.0f, &x, &y, &z);
                TIMER_CH0CV(TIMER0) = (uint16_t)(x * PWM_PERIOD);
                TIMER_CH1CV(TIMER0) = (uint16_t)(y * PWM_PERIOD);
                TIMER_CH2CV(TIMER0) = (uint16_t)(z * PWM_PERIOD);
            }
            g_pstEncCalibData->t_start = now;
            g_pstEncCalibData->step = 5;
        }
        break;
    }

    /*=====================================================================
     *  LUT 
     *
     *    (err_fwd + err_rev)/2
     *      + 
     *      - 
     *     
     *=====================================================================*/
    case 5:
    {
        float elapsed = (now - g_pstEncCalibData->t_start) / 1000.0f;
        float pp_f = (float)g_pstMotorData->pole_pairs;
        float theta_elec = (float)g_pstEncCalibData->calib_dir * (pp_f * 360.0f - 360.0f * elapsed);

        /*  */
        float s, c;
        fast_sin_cos(theta_elec, &s, &c);
        float ua = g_pstEncCalibData->lock_mod * c;
        float ub = g_pstEncCalibData->lock_mod * s;
        Us_Limit(&ua, &ub, 0.92f);

        float a, b, ccr;
        SVPWM_Calc_Cartesian(ua, ub, &a, &b, &ccr);
        TIMER_CH0CV(TIMER0) = (uint16_t)(a * PWM_PERIOD);
        TIMER_CH1CV(TIMER0) = (uint16_t)(b * PWM_PERIOD);
        TIMER_CH2CV(TIMER0) = (uint16_t)(ccr * PWM_PERIOD);
        ADC_Config_By_Sector(g_pstMotorData->sector);

        /*  */
        float theta_abs = fabsf(theta_elec);
        int desired = (int)(theta_abs / (360.0f / 128.0f));
        if (desired < 0) desired = 0;

        while (g_pstEncCalibData->sample_idx > 0 && (int)(g_pstEncCalibData->sample_idx - 1) >= desired) {
            g_pstEncCalibData->sample_idx--;
            float theta_at_sample = (float)g_pstEncCalibData->sample_idx * (360.0f / 128.0f)
                                    * (float)g_pstEncCalibData->calib_dir;
            float mech_deg = theta_at_sample / pp_f;
            uint16_t expected = (uint16_t)(mech_deg * 16384.0f / 360.0f);

            int32_t err = (int32_t)g_pstMotorData->enc_raw_cache - (int32_t)expected;
            if (err > 8191)  err -= 16384;
            if (err < -8191) err += 16384;
            /*  */
            error_sum[g_pstEncCalibData->sample_idx] = (int16_t)((error_sum[g_pstEncCalibData->sample_idx] + err) / 2);
        }

        if (g_pstEncCalibData->sample_idx <= 0) {
            UartPrintf("ENC: CCW LUT done -> gen LUT\r\n");
            g_pstEncCalibData->step = 6;
        }
        break;
    }

    /*=====================================================================
     *  128  LUT
     *
     *   1.  error_sum[]  encoder_offset
     *   2.  128 128  LUT
     *   3. 
     *
     *   : LUT  = enc_raw >> 714  / 128
     *           
     *=====================================================================*/
    case 6:
    {
        /*  */
        int32_t avg = 0;
        for (int i = 0; i < g_pstEncCalibData->total_samples; i++) avg += error_sum[i];
        avg /= g_pstEncCalibData->total_samples;
        g_pstMotorData->encoder_offset = (int16_t)avg;

        /*  +  128  LUT */
        int raw_offset = (int)g_pstEncCalibData->enc_lut_start * 128 / 16384;
        /*  = 1/8 = 128/8 = 16  */
        const int window = g_pstEncCalibData->total_samples / g_pstMotorData->pole_pairs / 8;

        for (int i = 0; i < 128; i++) {
            int32_t moving_avg = 0;
            for (int j = -window / 2; j < window / 2; j++) {
                int idx = i * g_pstEncCalibData->total_samples / 128 + j;
                if (idx < 0) idx += g_pstEncCalibData->total_samples;
                else if (idx >= g_pstEncCalibData->total_samples) idx -= g_pstEncCalibData->total_samples;
                moving_avg += error_sum[idx];
            }
            moving_avg /= window;

            /*  g_pstEncCalibData->enc_lut_start  LUT  */
            int lut_idx = raw_offset + i;
            if (lut_idx >= 128) lut_idx -= 128;

            g_pstMotorData->offset_lut[lut_idx] = (int16_t)(moving_avg - avg);
        }

        g_pstMotorData->lut_valid = 1;
        UartPrintf("ENC: LUT done, offset=%d\r\n", g_pstMotorData->encoder_offset);
        g_pstEncCalibData->lut_print_line = 0;
        g_pstEncCalibData->step = 7;
        break;
    }

    /* =====  LUT  ===== */
    case 7:
    {
        uint32_t now = g_pstSystemData->SystemTimes;
        if (now - g_pstEncCalibData->t_print < 15) break;  /*   15ms/ */
        g_pstEncCalibData->t_print = now;
        uint8_t i = g_pstEncCalibData->lut_print_line * 16;
        UartPrintf("LUT[%3d~%3d]: %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\r\n",
            i, i + 15,
            g_pstMotorData->offset_lut[i+0],  g_pstMotorData->offset_lut[i+1],
            g_pstMotorData->offset_lut[i+2],  g_pstMotorData->offset_lut[i+3],
            g_pstMotorData->offset_lut[i+4],  g_pstMotorData->offset_lut[i+5],
            g_pstMotorData->offset_lut[i+6],  g_pstMotorData->offset_lut[i+7],
            g_pstMotorData->offset_lut[i+8],  g_pstMotorData->offset_lut[i+9],
            g_pstMotorData->offset_lut[i+10], g_pstMotorData->offset_lut[i+11],
            g_pstMotorData->offset_lut[i+12], g_pstMotorData->offset_lut[i+13],
            g_pstMotorData->offset_lut[i+14], g_pstMotorData->offset_lut[i+15]);
        g_pstEncCalibData->lut_print_line++;
        if (g_pstEncCalibData->lut_print_line >= 8) {
            EncCalib_SaveToFlash();
            UartPrintf("ENC: saved to Flash\r\n");

            TIMER_CH0CV(TIMER0) = 0;
            TIMER_CH1CV(TIMER0) = 0;
            TIMER_CH2CV(TIMER0) = 0;

            g_pstEncCalibData->step = 0;
            free(error_sum);
            error_sum = NULL;
            return 1;
        }
        break;
    }
    }
    return 0;
}
/**
 * @brief   FocSystemRun::FOC_CALIB 
 * @note    g_pstFoc_RunState->CALIB_State 
 */
uint8_t CALIB_Run(void)
{
    /*  */
    if (g_pstFlagData && g_pstFlagData->motor_calibrated) {
        if (g_pstFoc_RunState->CALIB_State != CALIB_DONE) {
            /*  R/L */
            g_pstMotorData->motor_r = g_pstFlagData->motor_r;
            g_pstMotorData->motor_l = g_pstFlagData->motor_l;
            if (g_pstMotorData->motor_r < 0.001f) g_pstMotorData->motor_r = 2.4f;
            if (g_pstMotorData->motor_l < 1e-6f)  g_pstMotorData->motor_l = 0.00086f;
            /*  ADC  */
            g_pstMotorData->adc_zero_u0 = g_pstFlagData->adc_zero_u0;
            g_pstMotorData->adc_zero_v0 = g_pstFlagData->adc_zero_v0;
            g_pstMotorData->adc_zero_w1 = g_pstFlagData->adc_zero_w1;
            g_pstMotorData->adc_zero_v1 = g_pstFlagData->adc_zero_v1;
            /*  ADC  */
            g_pstMotorData->gain_v1 = g_pstFlagData->gain_v1;
            g_pstMotorData->gain_vw1 = g_pstFlagData->gain_vw1;
            g_pstMotorData->gain_uv0 = g_pstFlagData->gain_uv0;
            if (g_pstMotorData->gain_uv0 < 0.001f) g_pstMotorData->gain_uv0 = 1.0f;
            FOC_Update_Current_Gain(2000.0f);
            g_pstFoc_RunState->CALIB_State = CALIB_DONE;
        }
    }

    switch (g_pstFoc_RunState->CALIB_State)
    {
    case CALIB_ADC:
        if (CALIB_ADC_Run()) {
            g_pstFoc_RunState->CALIB_State = CALIB_RL;
        }
        break;

    case CALIB_RL:
        if (CALIB_RL_Run()) {
            FOC_Update_Current_Gain(2000.0f);  /*  2000 rad/s */
            g_pstFoc_RunState->CALIB_State = CALIB_ENC;
        }
        break;

    case CALIB_ENC:
        if (CALIB_ENC_Run()) {
            g_pstFoc_RunState->CALIB_State = CALIB_DONE;
        }
        break;

    case CALIB_DONE:
        UartPrintf("Calibration complete\r\n");
        g_pstFoc_RunState->CALIB_State = CALIB_IDLE;
        /*  R/L/ADC */
        if (g_pstFlagData) {
            g_pstFlagData->motor_calibrated = 1;
            g_pstFlagData->motor_r = g_pstMotorData->motor_r;
            g_pstFlagData->motor_l = g_pstMotorData->motor_l;
            g_pstFlagData->adc_zero_u0 = g_pstMotorData->adc_zero_u0;
            g_pstFlagData->adc_zero_v0 = g_pstMotorData->adc_zero_v0;
            g_pstFlagData->adc_zero_w1 = g_pstMotorData->adc_zero_w1;
            g_pstFlagData->adc_zero_v1 = g_pstMotorData->adc_zero_v1;
            g_pstFlagData->gain_v1 = g_pstMotorData->gain_v1;
            g_pstFlagData->gain_vw1 = g_pstMotorData->gain_vw1;
            g_pstFlagData->gain_uv0 = g_pstMotorData->gain_uv0;
            Flag_Save();
        }
        /* Id=0,  30 rev/s(1800RPM) */
        g_pstMotorData->Id_ref = 0.0f;
        Control_Speed_Init(g_pstMotorData, 0.14f, 3.0f, 1.0f);
        g_pstMotorData->pos_gain = 0.0f;
        g_pstMotorData->pos_setpoint = 0.0f;
        g_pstMotorData->vel_ff = 0.0f;
        g_pstMotorData->curr_ff = 0.0f;
        g_pstMotorData->last_vel_ff = 0.0f;
        g_pstMotorData->inertia = 0.000085f;
        g_pstMotorData->traj_active = 0;
        g_pstMotorData->traj_v_max = 30.0f;
        g_pstMotorData->traj_a_max = 100.0f;
        g_pstMotorData->traj_j_max = 500.0f;
        g_pstMotorData->vel_setpoint = 16.667f;
        g_pstMotorData->run_state = FOC_RUN;
        break;

    case CALIB_IDLE:
    default:
        break;
    }
    return 0;
}
