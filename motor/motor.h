/*
 * @Author: ntw48 2476672877@qq.com
 * @Date: 2026-03-20 21:32:38
 * @LastEditors: ntw48 2476672877@qq.com
 * @LastEditTime: 2026-03-26 01:50:49
 * @FilePath: \SVPWM_HALL\drivers\motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* 编码器方向交换标志 — 与 DEMO/demo.c 保持一致 */
#define ENCODER_DIR_NOCHANGE     0
#define ENCODER_MODE_CHANGE      1

typedef struct PI_Inc PI_Inc_t;

/**
 * @brief 电机状态结构体
 * 用于在SVPWM、ADC、FOC控制环路之间传递数据，
 * 采用类似 g_pstUartData 的指针传递方式。
 */
typedef struct MotorData {
    // SVPWM 相关状态
    uint8_t sector;           // 当前SVPWM扇区号 (1~6)
    float u_alpha;            // Alpha 轴目标电压
    float u_beta;             // Beta 轴目标电压

    // ADC 采样及校准数据
    uint16_t adc_zero_u0;      // U相零点校准值
    uint16_t adc_zero_v0;      // V相零点校准值
    uint16_t adc_zero_w1;      // W相零点校准值 (ADC1)
    uint16_t adc_zero_v1;     // V相零点校准值 (ADC1)

    uint16_t adc_inject_buf[2]; // ADC 注入组双通道采样结果缓冲
    uint8_t adc_new_data_flag;  // 新数据标志位置位，ADC中断中置1，主循环中清零

    // ADC 原始码值（零偏校正后，未转安培），用于调试
    int16_t adc_raw_u;
    int16_t adc_raw_v;
    int16_t adc_raw_w;

    // 重构的相电流 (实际物理值，单位A)
    float Iu;
    float Iv;
    float Iw;

    // FOC 运算结果
    float I_alpha;
    float I_beta;
    float Id;
    float Iq;
    float Id_filt;   /* 滤波后 Id（一阶低通 α=0.05） */
    float Iq_filt;   /* 滤波后 Iq */
    float Ud;
    float Uq;
    float integral_d;   /* 位置式 PI 积分项 (d 轴) */
    float integral_q;
    float Id_ref;
    float Iq_ref;

    // 机械与电气角度
    float electrical_angle;   // 当前电角度 (0~360度)
    float mechanical_angle;   // 机械角度
    float angle_offset;       // 转子对齐偏移，编码器角度-电角度

    // PLL 速度估计
    float pll_vel;            // PLL 速度估计 (counts/s)
    uint8_t speed_fb_mode;    // 速度反馈源: 0=PLL, 1=直接差分RPM

    // 位置环（同DEMO结构）
    float pos_setpoint;       // 位置目标 (rev)
    float pos_gain;           // 位置环P增益 (rev/s / rev)
    float pos_accum;          // 累积位置 (rev) — 来自PLL的pos_est/CPR
    float vel_ff;             // 速度前馈 (rev/s)
    float curr_ff;            // 电流前馈 (A), 来自加速度×J/Kt
    float last_vel_ff;        // 上周期vel_ff，用于算加速度
    float inertia;            // J/Kt (A/(rev/s²)), 前馈系数

    // S型轨迹规划
    float traj_Xi;            // 起始位置 (rev)
    float traj_Xf;            // 最终目标位置 (rev)
    float traj_v_max;         // 最大速度 (rev/s)
    float traj_a_max;         // 最大加速度 (rev/s²)
    float traj_j_max;         // 最大加加速度 (rev/s³)
    float traj_Tf;            // 总时间 (s)
    float traj_t;             // 当前时间 (s)
    uint8_t traj_active;      // 轨迹运行中

    // 速度环 PI 控制器
    float vel_setpoint;        // 速度目标值 (rev/s)
    float vel_integrator;      // 速度环积分累加 (A)
    float vel_gain;            // 速度环比例增益 (A/(rev/s))
    float vel_integrator_gain; // 速度环积分增益 (A/(rev/s)²)
    float current_limit;       // 速度环输出电流限幅 (A)

    // 运行状态
    uint8_t run_state;        // 运行状态
    uint8_t error_code;       // 故障代码

    // 开环控制参数
    float open_loop_speed;    // 当前电频率 (Hz)，斜坡控制下会被自动改变
    float open_loop_amp;      // 当前电压幅值 (0.0~1.0)，V/f 控制下会被自动改变

    // 开环频率斜坡与 V/f 曲线
    float target_speed;       // 斜坡目标电频率 (Hz)
    float ramp_rate;          // 频率变化率 (Hz/s)，设为 0 禁用斜坡
    float v_boost;            // V/f 低速 boost 幅值（归一化，0.0~1.0）
    float v_per_hz;           // V/f 斜率（幅值/Hz）

    // 电流环 PI 控制器
    PI_Inc_t* pi_id;
    PI_Inc_t* pi_iq;

    // 运行时零点校准（硬件触发采样，volatile 防编译器优化）
    volatile uint8_t  zero_calib_enabled;
    volatile uint32_t zero_calib_sum0;
    volatile uint32_t zero_calib_sum1;
    volatile uint16_t zero_calib_count;

    // ADC 通道增益修正系数
    float gain_v1;      // ADC1 V相增益修正 (ADC0_V/ADC1_V)
    float gain_vw1;     // ADC1 W相增益修正 (ADC0_V/ADC1_W) = gain_v1 × amp_wv
    float gain_uv0;     // ADC0 U/V通道增益比 (ADC0_V/ADC0_U)

    // RL 校准累积（ISR 中累加 Iu）
    volatile uint8_t  rl_accum_enabled;
    volatile float    rl_accum_sum;
    volatile uint32_t rl_accum_count;

    // 电感交替法（ISR 中交替电压 + 累积 Iα）
    volatile uint8_t  l_alt_enabled;
    volatile uint8_t  l_alt_phase;       // 0=负电压, 1=正电压
    volatile float    l_alt_sum_pos;     // 正半周 Iα 累加
    volatile float    l_alt_sum_neg;     // 负半周 Iα 累加
    volatile uint32_t l_alt_count;       // 总 ISR 调用次数

    // 编码器校准
    int16_t  encoder_offset;    // 编码器偏移量（直流分量）
    int8_t   encoder_dir;       // 方向 +1/-1
    uint8_t  change_dir;        // B/C 相交换标志（DEMO 兼容：ENCODER_DIR_NOCHANGE / ENCODER_MODE_CHANGE）
    uint8_t  pole_pairs;        // 极对数
    int16_t  offset_lut[128];   // 128 点补偿 LUT（有符号误差，匹配 DEMO）

    // 编码器运行时修正
    volatile uint16_t enc_raw_cache;      // ISR 缓存的编码器原始值
    volatile uint16_t enc_corrected;      // LUT 修正后的编码器值
    volatile uint8_t  lut_valid;          // LUT 已生成

    // 速度
    volatile float    motor_speed_rpm;    // ISR 实时更新的转速

    // 校准结果
    float motor_r;      // 相电阻 (ohm)
    float motor_l;      // 相电感 (H)

} g_stMotorData, *g_pstMotorDataPtr;

// 全局指针
extern g_pstMotorDataPtr g_pstMotorData;

typedef struct PI_Inc {
    float kp;
    float ki;
    float Ts;
    float out_min;
    float out_max;
    float u;
    float e_prev;
} PI_Inc_t;

void PI_Inc_Init(PI_Inc_t* pi, float kp, float ki, float Ts, float out_min, float out_max);
float PI_Inc_Update(PI_Inc_t* pi, float ref, float fb);
void PI_Inc_Reset(PI_Inc_t* pi, float u0);

#endif // __MOTOR_H
