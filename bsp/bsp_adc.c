#include "gd32f30x_adc.h"
#include "gd32f30x_timer.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_misc.h"
#include "motor/motor.h"
#include "bsp/bsp_timer.h"
#include "bsp/bsp_gpio.h"
#include "motor/control.h"
#include "motor/flag_storage.h"
#include "motor/svpwm.h"
#include "motor/trajectory.h"
#include "bsp/bsp_usart.h"
#include "bsp/bsp_systick.h"
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "bsp/bsp_adc.h"
#include "bsp/bsp_mt6816.h"

/**
 * @brief  ADC初始化（ADC0+ADC1双ADC同步注入模式，适配FOC）
 * @note   完全匹配6扇区上/下桥臂时序，无时间差采样
 * @param  无
 * @retval 无
 */
void ADC_FOC_Init(void)
{
    /* 1. 使能时钟 */
    rcu_periph_clock_enable(RCU_ADC0);    // 使能ADC0时钟
    rcu_periph_clock_enable(RCU_ADC1);    // 使能ADC1时钟
    rcu_periph_clock_enable(RCU_GPIOA);   // 采样引脚GPIO时钟
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);

    /* 2. 配置ADC采样引脚 */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ,
              GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    /* 3. 复位ADC */
    adc_deinit(ADC0);
    adc_deinit(ADC1);

    /* 4. 单模式基本配置 */
    adc_resolution_config(ADC0, ADC_RESOLUTION_12B);
    adc_resolution_config(ADC1, ADC_RESOLUTION_12B);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);
    adc_special_function_config(ADC1, ADC_SCAN_MODE, DISABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);

    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_TRGO);
    adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);

    /* 5. 初始注入组通道 */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 1);
    adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_W, ADC_SAMPLETIME_13POINT5);

    /* 6. 使能ADC（黄金顺序：先使能，再校准，最后开中断） */
    adc_enable(ADC0);
    adc_enable(ADC1);

    /* 7. 硬件校准 (CLB/RSTCLB 数字通道可用) */
    adc_calibration_enable(ADC0);
    adc_calibration_enable(ADC1);

    /* 8. 切换至双ADC并行注入模式 */
    adc_mode_config(ADC_DAUL_INSERTED_PARALLEL);

    /* 9. 使能外部触发（ADC0 和 ADC1 都需要，双模式下受同一个触发源驱动） */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);

    /* 10. 最后开中断（只开ADC0中断，ADC1同步完成） */
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
    nvic_irq_enable(ADC0_1_IRQn, 1, 0);
}


/**
 * @brief  按扇区动态配置ADC注入组通道
 * @note   严格匹配6扇区上/下桥臂时序图
 * @param  sector: 扇区号（1~6）
 * @retval 无
 */
void ADC_Config_By_Sector(uint8_t sector)
{
    if(sector < 1 || sector > 6) return;
    g_pstMotorData->sector = sector;

    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 1);

    switch(sector)
    {
        case 1:
        case 6:
            adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
            adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_W, ADC_SAMPLETIME_13POINT5);
            break;
        case 2:
        case 3:
            adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_U, ADC_SAMPLETIME_13POINT5);
            adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_W, ADC_SAMPLETIME_13POINT5);
            break;
        case 4:
        case 5:
            adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_U, ADC_SAMPLETIME_13POINT5);
            adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
            break;
        default:
            adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_V, ADC_SAMPLETIME_13POINT5);
            adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_W, ADC_SAMPLETIME_13POINT5);
            break;
    }
}

/**
 * @brief  获取三相电流（注入组采样+计算）
 * @note   需先检查g_pstMotorData->adc_new_data_flag，确保数据有效
 * @param  i_u/i_v/i_w: 输出三相电流（int16_t）
 * @retval 无
 */
void ADC_Get_Three_Phase_Current(int16_t* i_u, int16_t* i_v, int16_t* i_w)
{
    // 如果没有新数据，则不更新
    if(!g_pstMotorData->adc_new_data_flag)
    {
        return;
    }

    /* 根据当前扇区解析通道，并用基尔霍夫定律计算第三相 */
    /* 所有通道归一化到同一基准 (ADC0_V 增益):
       gain_v1   = G_ADC0/G_ADC1               ADC1 对齐到 ADC0
       gain_uv0  = G_amp_V/G_amp_U             ADC0_U 对齐到 ADC0_V
       gain_vw1   = (G_ADC0×G_amp_V)/(G_ADC1×G_amp_W)  ADC1_W 对齐到 ADC0_V */
    switch(g_pstMotorData->sector)
    {
        case 1:
        case 6:
            // 采了V(ADC0)和W(ADC1)
            *i_v = (int16_t)(g_pstMotorData->adc_inject_buf[0] - g_pstMotorData->adc_zero_v0);
            {
                int32_t w_raw = (int32_t)g_pstMotorData->adc_inject_buf[1] - (int32_t)g_pstMotorData->adc_zero_w1;
                *i_w = (int16_t)((float)w_raw * g_pstMotorData->gain_vw1);
            }
            *i_u = -(*i_v + *i_w);
            break;

        case 2:
        case 3:
            // 采了U(ADC0)和W(ADC1)
            *i_u = (int16_t)(g_pstMotorData->adc_inject_buf[0] - g_pstMotorData->adc_zero_u0);
            *i_u = (int16_t)((float)(*i_u) * g_pstMotorData->gain_uv0); /* 对齐 ADC0 U/V 运放增益 */
            {
                int32_t w_raw = (int32_t)g_pstMotorData->adc_inject_buf[1] - (int32_t)g_pstMotorData->adc_zero_w1;
                *i_w = (int16_t)((float)w_raw * g_pstMotorData->gain_vw1);
            }
            *i_v = -(*i_u + *i_w);
            break;

        case 4:
        case 5:
            // 采了U(ADC0)和V(ADC1)
            *i_u = (int16_t)(g_pstMotorData->adc_inject_buf[0] - g_pstMotorData->adc_zero_u0);
            *i_u = (int16_t)((float)(*i_u) * g_pstMotorData->gain_uv0); /* 对齐 ADC0 U/V 运放增益 */
            {
                int32_t v_raw = (int32_t)g_pstMotorData->adc_inject_buf[1] - (int32_t)g_pstMotorData->adc_zero_v1;
                *i_v = (int16_t)((float)v_raw * g_pstMotorData->gain_v1);
            }
            *i_w = -(*i_u + *i_v);
            break;

        default:
            *i_u = *i_v = *i_w = 0;
            break;
    }

    g_pstMotorData->adc_new_data_flag = 0; // 清除新数据标志
}

/**
 * @brief  ADC0/1中断服务函数（处理注入组转换完成）
 * @note   采样完成后立即读取数据，无DMA延迟
 * @param  无
 * @retval 无
 */
void ADC0_1_IRQHandler(void)
{
    if(adc_interrupt_flag_get(ADC0, ADC_INT_FLAG_EOIC))
    {
        // gpio_bit_toggle(GPIOB, GPIO_PIN_9); // 调试用：每次中断翻转PB9，观察频率和稳定性
        /* 1. 读取双ADC同步转换结果 */
        g_pstMotorData->adc_inject_buf[0] = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
        g_pstMotorData->adc_inject_buf[1] = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);

        g_pstMotorData->adc_new_data_flag = 1;
        adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);

        /* 1.5 编码器缓存 + LUT 修正（与 PWM 同步更新） */
        {
            MT6816_Data_t enc;
            MT6816_Read(&enc);
            uint16_t raw = enc.pos;
            g_pstMotorData->enc_raw_cache = raw;

            if (g_pstMotorData->lut_valid) {
                /* 128 点 LUT 线性插值 */
                int idx   = raw >> 7;                           /* 高 7 位 → LUT 索引 */
                int frac  = raw & 0x7F;                         /* 低 7 位 → 插值系数 */
                int off_1 = g_pstMotorData->offset_lut[idx];
                int off_2 = g_pstMotorData->offset_lut[(idx + 1) & 0x7F];
                int off   = off_1 + ((off_2 - off_1) * frac >> 7);
                int corr  = (int)raw - off;

                /* 零点校准：减去 DC 偏移，对齐电气零位 */
                corr -= g_pstMotorData->encoder_offset;

                if (corr > 16383) corr -= 16384;
                else if (corr < 0) corr += 16384;
                g_pstMotorData->enc_corrected = (uint16_t)corr;
            } else {
                g_pstMotorData->enc_corrected = raw;
            }
        }

        /* 1.6 实时转速计算（ISR 20kHz 固定周期，不会套圈） */
        {
            static uint16_t enc_prev = 0;
            int32_t delta = (int32_t)g_pstMotorData->enc_corrected - (int32_t)enc_prev;
            if (delta > 8191) delta -= 16384;
            else if (delta < -8191) delta += 16384;
            g_pstMotorData->motor_speed_rpm = (float)delta * 20000.0f / 16384.0f * 60.0f;
            enc_prev = g_pstMotorData->enc_corrected;
        }

        /* 1.7 PLL 速度估计（移植自 DEMO ENCODER_sample） */
        {
            const float DT_PLL = 1.0f / 20000.0f;      /* 50µs */
            /* PLL 二阶系统: ω_n=188 rad/s(30Hz), ζ=0.707
             * KI = ω_n² = 35344, KP = 2ζω_n = 266 */
            /* PLL 临界阻尼 ζ=1: KP=2·ω_n=6000, KI=0.25·KP²=9e6 (同DEMO) */
            const float PLL_KP = 6000.0f;               /* 位置跟踪增益 (1/s) */
            const float PLL_KI = 9000000.0f;            /* 速度跟踪增益 (1/s²) */
            static uint8_t pll_init = 0;
            static int32_t shadow_cnt = 0;              /* 累计位置 (counts) */
            static int32_t cpr_prev = 0;                /* 上一周期 CPR 值 */
            static float pos_est = 0.0f;                /* PLL 累计位置估计 */
            static float pos_cpr = 0.0f;                /* PLL 单圈位置估计 */
            static float vel_est = 0.0f;                /* PLL 速度 (counts/s) */

            int32_t cpr_meas = (int32_t)g_pstMotorData->enc_corrected;

            if (!pll_init) {
                shadow_cnt = cpr_meas;
                cpr_prev = cpr_meas;
                pos_est = (float)shadow_cnt;
                pos_cpr = (float)cpr_meas;
                vel_est = 0.0f;
                pll_init = 1;
                g_pstMotorData->pll_vel = 0.0f;
            } else {
                /* 累计位置更新：delta = 当前 - 上一周期，处理绕回 */
                int32_t delta = cpr_meas - cpr_prev;
                if (delta > 8191)       delta -= 16384;
                else if (delta < -8191) delta += 16384;
                shadow_cnt += delta;
                cpr_prev = cpr_meas;

                /* PLL 预测 */
                pos_est += DT_PLL * vel_est;
                pos_cpr += DT_PLL * vel_est;

                /* 相位检测 */
                float err_cnt = (float)shadow_cnt - pos_est;
                float err_cpr = (float)cpr_meas - pos_cpr;
                if (err_cpr > 8192.0f)       err_cpr -= 16384.0f;
                else if (err_cpr < -8192.0f) err_cpr += 16384.0f;

                /* PLL 修正 */
                pos_est += DT_PLL * PLL_KP * err_cnt;
                pos_cpr += DT_PLL * PLL_KP * err_cpr;
                if (pos_cpr >= 16384.0f) pos_cpr -= 16384.0f;
                else if (pos_cpr < 0.0f) pos_cpr += 16384.0f;
                vel_est += DT_PLL * PLL_KI * err_cpr;

                g_pstMotorData->pll_vel = vel_est;
                /* 累积位置(rev)供位置环使用 */
                g_pstMotorData->pos_accum = (float)shadow_cnt / 16384.0f;
            }
        }

        /* 2. 零点校准模式：只累加原始值，不进入控制循环 */
        if (g_pstMotorData->zero_calib_enabled) {
            g_pstMotorData->zero_calib_sum0 += g_pstMotorData->adc_inject_buf[0];
            g_pstMotorData->zero_calib_sum1 += g_pstMotorData->adc_inject_buf[1];
            g_pstMotorData->zero_calib_count++;
            return;
        }

        /* 3. 统一读取三相电流 */
        {
            int16_t iu=0, iv=0, iw=0;
            ADC_Get_Three_Phase_Current(&iu, &iv, &iw);

            // 电流方向校正：根据硬件确认低端采样，取负对齐 Clarke 方向
            g_pstMotorData->Iu = -(float)iu * CURRENT_CONV_FACTOR;
            g_pstMotorData->Iv = -(float)iv * CURRENT_CONV_FACTOR;
            g_pstMotorData->Iw = -(float)iw * CURRENT_CONV_FACTOR;
        }

        /* 3.5 RL 校准累积：累加 Iu 用于电阻/电感计算 */
        if (g_pstMotorData->rl_accum_enabled) {
            g_pstMotorData->rl_accum_sum += g_pstMotorData->Iu;
            g_pstMotorData->rl_accum_count++;
        }

        /* 3.6 电感交替法：每周期（50µs）交替 ±Vdd，累积 Iα */
        if (g_pstMotorData->l_alt_enabled) {
            g_pstMotorData->l_alt_phase ^= 1;
            float mod = g_pstMotorData->l_alt_phase ? 0.6f : -0.6f;
            float zero = 0.0f;
            Us_Limit(&mod, &zero, 0.92f);               /* 安全钳位 */
            float ccr_u, ccr_v, ccr_w;
            SVPWM_Calc_Cartesian(mod, 0.0f, &ccr_u, &ccr_v, &ccr_w);
            TIMER_CH0CV(TIMER0) = (uint16_t)(ccr_u * PWM_PERIOD);
            TIMER_CH1CV(TIMER0) = (uint16_t)(ccr_v * PWM_PERIOD);
            TIMER_CH2CV(TIMER0) = (uint16_t)(ccr_w * PWM_PERIOD);
            ADC_Config_By_Sector(g_pstMotorData->sector);
            if (g_pstMotorData->l_alt_phase)
                g_pstMotorData->l_alt_sum_pos += g_pstMotorData->Iu;
            else
                g_pstMotorData->l_alt_sum_neg += g_pstMotorData->Iu;
            g_pstMotorData->l_alt_count++;
        }

        /* 4. 串联位置环 → 速度环 → 电流闭环（FOC_RUN 模式下 20kHz 执行） */
        const float DT_CTRL = 1.0f / 20000.0f;      /* 50µs */
        if (g_pstMotorData->run_state == FOC_RUN) {
            float vel_target, vel_fb;
            float iq_ref;

            /* 速度反馈 */
            if (g_pstMotorData->speed_fb_mode) {
                vel_fb = g_pstMotorData->motor_speed_rpm / 60.0f;
            } else {
                vel_fb = g_pstMotorData->pll_vel / 16384.0f;
            }

            /* 位置环激活？pos_gain>0则位置→速度 */
            if (g_pstMotorData->pos_gain > 0.0f) {
                if (g_pstMotorData->traj_active)
                    TRAJ_eval(g_pstMotorData, DT_CTRL);
                float vel_from_pos = Control_Position_Step(g_pstMotorData, DT_CTRL);
                vel_target = vel_from_pos + g_pstMotorData->vel_ff;
            } else {
                vel_target = g_pstMotorData->vel_setpoint;
            }

            /* 速度环 */
            if (g_pstMotorData->vel_gain > 0.0f) {
                iq_ref = Control_Speed_Step(g_pstMotorData, vel_target, vel_fb);
                iq_ref += g_pstMotorData->curr_ff;  /* 前馈叠加 */
                /* 总限幅 */
                if (iq_ref >  g_pstMotorData->current_limit) iq_ref =  g_pstMotorData->current_limit;
                if (iq_ref < -g_pstMotorData->current_limit) iq_ref = -g_pstMotorData->current_limit;
            } else {
                iq_ref = g_pstMotorData->Iq_ref;
            }

            Control_Current_Step(g_pstMotorData,
                g_pstMotorData->Id_ref, iq_ref);
        }

    }
}

