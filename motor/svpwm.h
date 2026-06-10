/*
 * @Author: ntw48 2476672877@qq.com
 * @Date: 2026-03-14 15:41:36
 * @LastEditors: ntw48 2476672877@qq.com
 * @LastEditTime: 2026-03-18 04:03:48
 * @FilePath: \SVPWM_HALL\drivers\svpwm.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SVPWM_H
#define __SVPWM_H

#include "gd32f30x.h"

/**
 * @brief SVPWM初始化，配置定时器等
 */
void SVPWM_Init(void);

void Us_Limit(float *U_d_alpha, float *U_q_beta, float U_max);
void SVPWM_Calc_Cartesian(float Ualpha, float Ubeta, float *mod_U_CCR, float *mod_V_CCR, float *mod_W_CCR);
void Clarke_Transform(float Ia, float Ib, float *Ialpha, float *Ibeta);
void Park_Transform(float Ialpha, float Ibeta, float angle, float *Id, float *Iq);
void InvPark_Transform(float Vd, float Vq, float angle, float *Valpha, float *Vbeta);
void fast_sin_cos(float angle, float *sin_out, float *cos_out);
void Dead_Time_Compensate(float Iu, float Iv, float Iw, float *Ualpha, float *Ubeta);

#endif // __SVPWM_H
