#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <stdint.h>
#include <stddef.h>

/* ==================== 初始化 ==================== */
void UI_Init(void);
void UI_Clear(void);

/* ==================== 全参数数据布局 ==================== */
typedef struct {
    int     run_state;       /* 运行状态码 */
    const char *mode_str;    /* 模式文字 "POS"/"SPD"/"TRQ" */
    float   temp;            /* 温度 °C */
    float   vbus;            /* 母线电压 V */
    float   id;              /* Id 电流 A */
    float   iq;              /* Iq 电流 A */
    float   pos_setpoint;    /* 位置设定 rev */
    float   pos_actual;      /* 实际位置 rev */
    float   iu;              /* U 相电流 A */
    float   iv;              /* V 相电流 A */
    float   iw;              /* W 相电流 A */
    float   speed;           /* 速度 rpm */
    float   speed_target;    /* 目标速度 rpm */
    uint8_t  warn_level;     /* 报警级别 0=正常 1/2/3 */
    uint8_t  warn_code;      /* 报警代码 */
    uint16_t cpu_pct;        /* CPU 利用率 0~1000 = 0.0%~100.0% */
} MotorDisplayData;

/* 刷新全屏数据列表（含清屏） */
void UI_ShowDataList(const MotorDisplayData *data);

#endif /* DISPLAY_UI_H */
