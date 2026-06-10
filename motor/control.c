/*
 * @Author: ntw48 2476672877@qq.com
 * @Date: 2026-03-26 01:50:39
 * @LastEditors: ntw48 2476672877@qq.com
 * @LastEditTime: 2026-05-05 14:51:54
 * @FilePath: \SVPWM_HALL\drivers\control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "motor/control.h"
#include "motor/calib.h"
#include "bsp/bsp_adc.h"
#include "motor/svpwm.h"
#include "bsp/bsp_timer.h"
#include "motor/motor.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_usart.h"
#include "app/sys_malloc.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

void Control_Current_Init(g_pstMotorDataPtr motor, float Ts, float kp_id, float ki_id, float kp_iq, float ki_iq, float u_limit)
{
    if(!motor) return;
    if(!motor->pi_id) motor->pi_id = (PI_Inc_t*)malloc(sizeof(PI_Inc_t));
    if(!motor->pi_iq) motor->pi_iq = (PI_Inc_t*)malloc(sizeof(PI_Inc_t));
    if(motor->pi_id) PI_Inc_Init(motor->pi_id, kp_id, ki_id, Ts, -u_limit, u_limit);
    if(motor->pi_iq) PI_Inc_Init(motor->pi_iq, kp_iq, ki_iq, Ts, -u_limit, u_limit);
}

/** @brief 根据测得的 R/L 计算 PI 增益并初始化电流环 */
void FOC_Update_Current_Gain(float bandwidth)
{
    if (!g_pstMotorData || g_pstMotorData->motor_r < 0.001f
        || g_pstMotorData->motor_l < 1e-6f) return;

    /* 使用有效电感 0.86mH（实测 10mH 为 DCR 法偏大，0.86mH 更接近真实值） */
    float L = 0.00086f;
    float kp = L * bandwidth;                           /* L × ω_c */
    float ki = g_pstMotorData->motor_r * bandwidth;    /* R × ω_c */
    float u_limit = 12.0f;

    Control_Current_Init(g_pstMotorData, 1.0f / 20000.0f, kp, ki, kp, ki, u_limit);
    UartPrintf("FOC: kp=%.4f ki=%.1f bw=%.0f\r\n", kp, ki, bandwidth);
}

/** @brief 速度环 PI 初始化 */
void Control_Speed_Init(g_pstMotorDataPtr motor, float vel_gain, float vel_ki, float current_limit)
{
    if (!motor) return;
    motor->vel_gain = vel_gain;
    motor->vel_integrator_gain = vel_ki;
    motor->current_limit = current_limit;
    motor->vel_integrator = 0.0f;
    motor->vel_setpoint = 0.0f;
    UartPrintf("SPD: kp=%.4f ki=%.4f ilim=%.2f\r\n", vel_gain, vel_ki, current_limit);
}

/** @brief 速度环 PI 计算，返回 Iq 参考电流 (A) */
float Control_Speed_Step(g_pstMotorDataPtr motor, float vel_setpoint_revs, float vel_feedback_revs)
{
    if (!motor) return 0.0f;

    const float DT = 1.0f / 20000.0f;
    float v_err = vel_setpoint_revs - vel_feedback_revs;

    /* PI: current_des = vel_gain × v_err + vel_integrator */
    float current_des = motor->vel_gain * v_err + motor->vel_integrator;

    /* 限幅 + 抗积分饱和 */
    if (current_des > motor->current_limit) {
        current_des = motor->current_limit;
    } else if (current_des < -motor->current_limit) {
        current_des = -motor->current_limit;
    } else {
        motor->vel_integrator += motor->vel_integrator_gain * DT * v_err;
    }

    return current_des;
}

/** @brief 位置环计算：返回速度参考(rev/s)，含前馈（同DEMO结构） */
float Control_Position_Step(g_pstMotorDataPtr motor, float dt)
{
    if (!motor) return 0.0f;

    float pos_fb = motor->pos_accum;

    /* === 位置P控制 === */
    float pos_err = motor->pos_setpoint - pos_fb;

    /* 速度限幅 60 rev/s = 3600 RPM（同DEMO vel_limit） */
    const float VEL_LIMIT = 60.0f;
    float vel_des = motor->pos_gain * pos_err;
    if (vel_des > VEL_LIMIT)       vel_des = VEL_LIMIT;
    else if (vel_des < -VEL_LIMIT) vel_des = -VEL_LIMIT;

    /* === 前馈：从vel_ff变化率算加速度 → 电流前馈 === */
    float accel = (motor->vel_ff - motor->last_vel_ff) / dt;   /* rev/s² */
    motor->last_vel_ff = motor->vel_ff;

    /* curr_ff = J/Kt × accel  (A) */
    motor->curr_ff = motor->inertia * accel;

    /* clamp 前馈电流到 ±0.5A（不超过限幅一半，留余量给PI） */
    if (motor->curr_ff > 0.5f)      motor->curr_ff = 0.5f;
    else if (motor->curr_ff < -0.5f) motor->curr_ff = -0.5f;

    return vel_des;  /* 速度环setpoint = 位置P输出 + vel_ff */
}

#define V_TO_MOD      (1.0f / ((2.0f / 3.0f) * 12.0f))  /* Vbus=12V */
#define SVM_LIMIT_SQ  (1.333333f)                          /* (2/√3)² */

void Control_Current_Step(g_pstMotorDataPtr motor, float Id_ref, float Iq_ref)
{
    if (!motor || !motor->pi_id || !motor->pi_iq) return;

    /* ===== 1. 编码器 → 电气角 (0~360°) ===== */
    float mech = (float)g_pstMotorData->enc_corrected * 6.1035e-5f;  /* 1/16384 */
    float elec_deg = mech * g_pstMotorData->pole_pairs * 360.0f;
    elec_deg -= (int)(elec_deg / 360.0f) * 360.0f;
    if (elec_deg < 0.0f) elec_deg += 360.0f;
    motor->electrical_angle = elec_deg;

    /* 1.5 DT 角度补偿：ARR-1→底部(0.5DT) + 新PWM完整周期(1.0DT) = 75µs @ 20kHz */
    {
        float theta_comp = motor->motor_speed_rpm * motor->pole_pairs * 0.00045f;
        motor->electrical_angle += theta_comp;
        if (motor->electrical_angle >= 360.0f) motor->electrical_angle -= 360.0f;
        else if (motor->electrical_angle < 0.0f) motor->electrical_angle += 360.0f;
    }

    /* 预计算 sin/cos，Park 和 InvPark 共用（省一次查表+插值） */
    float sin_th, cos_th;
    fast_sin_cos(motor->electrical_angle, &sin_th, &cos_th);

    /* ===== 2. Clarke: Iu/Iv/Iw → Ialpha/Ibeta ===== */
    float Ialpha, Ibeta;
    Clarke_Transform(motor->Iu, motor->Iv, &Ialpha, &Ibeta);

    /* ===== 3. Park: Ialpha/Ibeta → Id/Iq ===== */
    {
        float Id_new = Ialpha * cos_th + Ibeta * sin_th;
        float Iq_new = -Ialpha * sin_th + Ibeta * cos_th;
        /* 一阶低通滤波 α=0.5 @ 20kHz → ~1600Hz 截止 */
        motor->Id_filt += 0.5f * (Id_new - motor->Id_filt);
        motor->Iq_filt += 0.5f * (Iq_new - motor->Iq_filt);
    }

    /* ===== 4. PI 位置式 ===== */
    const float Ts = 1.0f / 20000.0f;

    float err_d = Id_ref - motor->Id_filt;
    float err_q = Iq_ref - motor->Iq_filt;
    float kp = motor->pi_id->kp;
    float ki = motor->pi_id->ki;

    motor->Ud = kp * err_d + motor->integral_d;
    motor->Uq = kp * err_q + motor->integral_q;

    // /* 高速解耦：前馈补偿 dq 轴交叉耦合电压 */
    // {
    //     float omega_elec = motor->motor_speed_rpm * motor->pole_pairs * 0.10472f;  /* ÷60×2π */
    //     motor->Ud -= omega_elec * motor->motor_l * motor->Iq_filt;
    //     motor->Uq += omega_elec * motor->motor_l * motor->Id_filt;
    // }

    /* 调制比 + 平方比较（省 sqrt） */
    float mod_d = motor->Ud * V_TO_MOD;
    float mod_q = motor->Uq * V_TO_MOD;
    float mod_len_sq = mod_d * mod_d + mod_q * mod_q;

    /* 抗积分饱和 */
    if (mod_len_sq > SVM_LIMIT_SQ) {
        motor->integral_d *= 0.99f;
        motor->integral_q *= 0.99f;
    } else {
        motor->integral_d += err_d * ki * Ts;
        motor->integral_q += err_q * ki * Ts;
    }

    /* ===== 5. InvPark: Ud/Uq → Ualpha/Ubeta ===== */
    float Ualpha = mod_d * cos_th - mod_q * sin_th;
    float Ubeta  = mod_d * sin_th + mod_q * cos_th;

    /* ===== 6. SVPWM → PWM 寄存器 ===== */
    float u_ccr, v_ccr, w_ccr;
    SVPWM_Calc_Cartesian(Ualpha, Ubeta, &u_ccr, &v_ccr, &w_ccr);

    uint32_t period = (uint32_t)PWM_PERIOD;
    if (g_pstMotorData->change_dir == ENCODER_MODE_CHANGE) {
        TIMER_CH0CV(TIMER0) = (uint32_t)(u_ccr * period);
        TIMER_CH1CV(TIMER0) = (uint32_t)(w_ccr * period);
        TIMER_CH2CV(TIMER0) = (uint32_t)(v_ccr * period);
    } else {
        TIMER_CH0CV(TIMER0) = (uint32_t)(u_ccr * period);
        TIMER_CH1CV(TIMER0) = (uint32_t)(v_ccr * period);
        TIMER_CH2CV(TIMER0) = (uint32_t)(w_ccr * period);
    }
    ADC_Config_By_Sector(g_pstMotorData->sector);
}

/**
 * @brief  α-β 静止注入测试（方法一：最快定位 SVPWM + ADC 链路）
 * @note   直接产生旋转 α-β 电压矢量（不经过 InvPark），同时回读 Id/Iq
 *         预期：Id ≈ 0, Iq ≈ 正数（与幅值成正比）
 *         若 Id 大幅偏置 → ADC 零点未校准
 *         若 Iq 为负 → Park 符号反了
 *         若 Id/Iq 大幅波动 → SVPWM 扇区或 ADC 采样时序错误
 *         在 main.c 中设 run_state = 4 启用，串口观察 Id/Iq
 */
void Control_AlphaBeta_Inject_Step(g_pstMotorDataPtr motor)
{
    if(!motor) return;

    /* 参数：2Hz 旋转，15% 电压幅值 */
    const float test_freq = 2.0f;
    const float amplitude = 0.15f;
    const float Ts = 1.0f / 20000.0f;

    /* 1. 电角度步进 */
    float delta_theta = test_freq * 360.0f * Ts;
    motor->electrical_angle += delta_theta;
    if (motor->electrical_angle >= 360.0f)
        motor->electrical_angle -= 360.0f;

    /* 2. 直接生成 α-β 电压 */
    float sin_val, cos_val;
    fast_sin_cos(motor->electrical_angle, &sin_val, &cos_val);
    float Ualpha = amplitude * cos_val;
    float Ubeta  = amplitude * sin_val;

    motor->u_alpha = Ualpha;
    motor->u_beta  = Ubeta;

    /* 3. SVPWM 输出 */
    float u_ccr, v_ccr, w_ccr;
    SVPWM_Calc_Cartesian(Ualpha, Ubeta, &u_ccr, &v_ccr, &w_ccr);

    uint32_t period = (uint32_t)PWM_PERIOD;
    TIMER_CH0CV(TIMER0) = (uint32_t)(u_ccr * period);
    TIMER_CH1CV(TIMER0) = (uint32_t)(v_ccr * period);
    TIMER_CH2CV(TIMER0) = (uint32_t)(w_ccr * period);

    ADC_Config_By_Sector(motor->sector);

    /* 4. 回读电流 → Clarke → Park（用同一个角度） */
    float Ialpha, Ibeta;
    Clarke_Transform(motor->Iu, motor->Iv, &Ialpha, &Ibeta);
    motor->I_alpha = Ialpha;
    motor->I_beta  = Ibeta;

    Park_Transform(Ialpha, Ibeta, motor->electrical_angle, &motor->Id, &motor->Iq);
}

/**
 * @brief  d-q 旋转注入测试（方法二：验证 InvPark + SVPWM 整链）
 * @note   Ud=0, Uq=amp, 经 InvPark → SVPWM，回读 Id/Iq
 *         run_state = 5 启用
 */
void Control_DQ_Inject_Step(g_pstMotorDataPtr motor)
{
    if(!motor) return;

    const float test_freq = 2.0f;
    const float amplitude = 0.15f;
    const float Ts = 1.0f / 20000.0f;

    /* 1. 电角度步进 */
    float delta_theta = test_freq * 360.0f * Ts;
    motor->electrical_angle += delta_theta;
    if (motor->electrical_angle >= 360.0f)
        motor->electrical_angle -= 360.0f;

    /* 2. d-q → InvPark → α-β */
    float Ud = 0.0f;
    float Uq = amplitude;
    float Ualpha, Ubeta;
    InvPark_Transform(Ud, Uq, motor->electrical_angle, &Ualpha, &Ubeta);

    motor->u_alpha = Ualpha;
    motor->u_beta  = Ubeta;

    /* 3. SVPWM 输出 */
    float u_ccr, v_ccr, w_ccr;
    SVPWM_Calc_Cartesian(Ualpha, Ubeta, &u_ccr, &v_ccr, &w_ccr);

    uint32_t period = (uint32_t)PWM_PERIOD;
    TIMER_CH0CV(TIMER0) = (uint32_t)(u_ccr * period);
    TIMER_CH1CV(TIMER0) = (uint32_t)(v_ccr * period);
    TIMER_CH2CV(TIMER0) = (uint32_t)(w_ccr * period);

    ADC_Config_By_Sector(motor->sector);

    /* 4. 回读电流 → Clarke → Park */
    float Ialpha, Ibeta;
    Clarke_Transform(motor->Iu, motor->Iv, &Ialpha, &Ibeta);
    motor->I_alpha = Ialpha;
    motor->I_beta  = Ibeta;

    Park_Transform(Ialpha, Ibeta, motor->electrical_angle, &motor->Id, &motor->Iq);
}

void Control_OpenLoop_Step(g_pstMotorDataPtr motor)
{
    if(!motor) return;

    /* 固定参数：PWM 载波 20kHz 中心对齐，中断周期 50μs */
    const float Ts = 1.0f / 20000.0f;

    /* ========== 步骤 1a：频率斜坡 ========== */
    if (motor->ramp_rate > 0.0f) {
        float step = motor->ramp_rate * Ts;          /* 每步频率增量 (Hz) */
        if (motor->open_loop_speed < motor->target_speed) {
            motor->open_loop_speed += step;
            if (motor->open_loop_speed > motor->target_speed)
                motor->open_loop_speed = motor->target_speed;
        } else if (motor->open_loop_speed > motor->target_speed) {
            motor->open_loop_speed -= step;
            if (motor->open_loop_speed < motor->target_speed)
                motor->open_loop_speed = motor->target_speed;
        }
    }

    /* ========== 步骤 1b：V/f 曲线 ========== */
    if (motor->v_per_hz > 0.0f) {
        float amp = motor->v_boost + motor->v_per_hz * motor->open_loop_speed;
        if (amp > 0.92f) amp = 0.92f;          /* 限幅到 SVPWM 线性区 */
        if (amp < 0.0f)  amp = 0.0f;
        motor->open_loop_amp = amp;
    }

    /* ========== 步骤 1c：电角度积分 ========== */
    {
        float delta_theta = motor->open_loop_speed * 360.0f * Ts;
        motor->electrical_angle += delta_theta;
        if (motor->electrical_angle >= 360.0f)
            motor->electrical_angle -= 360.0f;
        else if (motor->electrical_angle < 0.0f)
            motor->electrical_angle += 360.0f;
    }

    /* ========== 步骤 1d：d-q 电压给定 ========== */
    float Ud = 0.0f;                              /* 弱磁方向置零 */
    float Uq = motor->open_loop_amp;               /* 转矩方向给定幅值 */
    Us_Limit(&Ud, &Uq, 0.92f);

    /* ========== 步骤 1e：逆 Park 变换 ========== */
    float Ualpha, Ubeta;
    InvPark_Transform(Ud, Uq, motor->electrical_angle, &Ualpha, &Ubeta);

    /* ========== 步骤 1f：死区补偿 ========== */
    Dead_Time_Compensate(motor->Iu, motor->Iv, motor->Iw, &Ualpha, &Ubeta);

    motor->u_alpha = Ualpha;
    motor->u_beta  = Ubeta;

    /* ========== 步骤 1g：SVPWM 计算 ========== */
    float u_ccr, v_ccr, w_ccr;
    SVPWM_Calc_Cartesian(Ualpha, Ubeta, &u_ccr, &v_ccr, &w_ccr);

    /* ========== 步骤 1h：写入 TIMER0 比较寄存器 ========== */
    uint32_t period = (uint32_t)PWM_PERIOD;
    TIMER_CH0CV(TIMER0) = (uint32_t)(u_ccr * period);
    TIMER_CH1CV(TIMER0) = (uint32_t)(v_ccr * period);
    TIMER_CH2CV(TIMER0) = (uint32_t)(w_ccr * period);

    /* ========== 步骤 1i：配置下次 ADC 注入组通道 ========== */
    ADC_Config_By_Sector(motor->sector);

    /* ========== 步骤 1j：回算 Id/Iq（供调试观察） ========== */
    float Ialpha, Ibeta;
    Clarke_Transform(motor->Iu, motor->Iv, &Ialpha, &Ibeta);
    Park_Transform(Ialpha, Ibeta, motor->electrical_angle, &motor->Id, &motor->Iq);
}

float Motor_Get_RPM(void)
{
    return g_pstMotorData->motor_speed_rpm;
}

void FocSystemRun(void)
{
    /*主状态机*/
    switch (g_pstMotorData->run_state)
    {
    case FOC_INIT:
        Foc_State_Init();
        break;

    case FOC_CHECK:

        break;

    case FOC_CALIB:
        CALIB_Run();
        break;

    case FOC_STAND: /* 待机 */
        
    break;

    case FOC_RUN:
        {
            static uint32_t t_print = 0;
            uint32_t now = g_pstSystemData->SystemTimes;

            float rpm = Motor_Get_RPM();

            if (now - t_print >= 10) {
                float mech_deg = (float)g_pstMotorData->enc_corrected * 0.02197265625f; /* 360/16384 */
                float pll_rpm = g_pstMotorData->pll_vel * 60.0f / 16384.0f;
                UartPrintf("%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                    g_pstMotorData->Id_filt, g_pstMotorData->Iq_filt,
                    g_pstMotorData->electrical_angle, mech_deg,
                    rpm, pll_rpm,
                    g_pstMotorData->vel_setpoint * 60.0f);   /* rev/s → RPM */
                t_print = now;
            }
            break;
        }

    case FOC_STOP: 

    break;

    case FOC_HAUFT: 

    break;

    case FOC_RESET: 

    break;

    default:
        break;
    }
    LED_Blink();
}
