#include "motor/motor.h"
#include <string.h>
#include <stdlib.h>

void PI_Inc_Init(PI_Inc_t* pi, float kp, float ki, float Ts, float out_min, float out_max)
{
    if(!pi) return;
    pi->kp = kp;
    pi->ki = ki;
    pi->Ts = Ts;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->u = 0.0f;
    pi->e_prev = 0.0f;
}

float PI_Inc_Update(PI_Inc_t* pi, float ref, float fb)
{
    if(!pi) return 0.0f;
    float e = ref - fb;
    float du = pi->kp * (e - pi->e_prev) + (pi->ki * pi->Ts) * e;
    float u_new = pi->u + du;
    if(u_new > pi->out_max) u_new = pi->out_max;
    if(u_new < pi->out_min) u_new = pi->out_min;
    pi->u = u_new;
    pi->e_prev = e;
    return pi->u;
}

void PI_Inc_Reset(PI_Inc_t* pi, float u0)
{
    if(!pi) return;
    pi->u = u0;
    if(pi->u > pi->out_max) pi->u = pi->out_max;
    if(pi->u < pi->out_min) pi->u = pi->out_min;
    pi->e_prev = 0.0f;
}
