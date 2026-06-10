/*
 * @Author: ntw48 2476672877@qq.com
 * @Date: 2026-03-14 15:36:04
 * @LastEditors: ntw48 2476672877@qq.com
 * @LastEditTime: 2026-03-20 18:33:10
 * @FilePath: \SVPWM_HALL\drivers\timer.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef TIMER_H
#define TIMER_H

#include "gd32f30x.h"
#include "app/usr_config.h"

/* 120MHz / (2 * PWM_PERIOD) = PWM_FREQ_HZ */
#define PWM_PERIOD          (SYS_CLOCK_FREQ / 2 / PWM_FREQ_HZ)
#define PWM_DEAD_TIME       0

/* 函数声明 */
void TIMER0_SVPWM_Init(void);

#endif /* TIMER_H */
