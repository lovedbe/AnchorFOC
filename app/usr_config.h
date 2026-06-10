#ifndef USR_CONFIG_H
#define USR_CONFIG_H

/* ============================================================
 * 用户配置文件 — 移植到新板/新电机时只需要改这里
 * ============================================================
 * 所有硬件参数、控制默认值集中管理，方便用户快速适配。
 * 各模块头文件中保留内部宏定义不冲突。
 * ============================================================ */

/* ==================== 系统时钟 ==================== */
#define SYS_CLOCK_FREQ          120000000UL     /* 系统主频 120MHz */

/* ==================== PWM 定时器 ==================== */
#define PWM_FREQ_HZ             20000           /* PWM 载波频率 (Hz) */
#define PWM_DEAD_TIME_NS        0               /* 死区时间 (ns)，0=无死区 */

/* ==================== 电流采样 ==================== */
#define CURRENT_SHUNT_R_OHM     0.010f          /* 采样电阻 (Ω) */
#define CURRENT_AMP_R_TOP       40200.0f        /* 差分放大器上臂电阻 (Ω) */
#define CURRENT_AMP_R_BOT       1100.0f         /* 差分放大器下臂电阻 (Ω) */
#define CURRENT_BIAS_V          1.18f           /* 偏置电压 (V) */
#define CURRENT_VDDA            3.3f            /* ADC 供电电压 (V) */

/* ==================== 编码器 ==================== */
#define ENCODER_POLE_PAIRS      7               /* 电机极对数 */
#define ENCODER_RESOLUTION      16384.0f        /* 编码器分辨率 (每圈码值) */

/* ==================== 串口 ==================== */
#define UART_BAUDRATE           115200

/* ==================== 开环 V/f 默认参数 ==================== */
#define OL_START_FREQ_HZ        5.0f            /* 斜坡起始频率 (Hz) */
#define OL_TARGET_FREQ_HZ       50.0f           /* 斜坡目标频率 (Hz) */
#define OL_RAMP_RATE_HZPS       50.0f           /* 频率加速率 (Hz/s) */
#define OL_V_BOOST              0.03f           /* V/f 低速 boost 幅值 */
#define OL_V_PER_HZ             0.0075f         /* V/f 斜率 (幅值/Hz) */
#define OL_INIT_AMP             0.05f           /* 初始电压幅值 */

/* ==================== 运行时初始化 ==================== */
void USR_Set_Default_Params(void);

#endif /* USR_CONFIG_H */
