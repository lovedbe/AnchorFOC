#include "motor/trajectory.h"
#include <math.h>

/* ===================== S型轨迹规划 ===================== */
/* 7段双S: +J->A->-J->coast->-J->-A->+J */
#define SCURVE_SEG_MAX 8

typedef struct { float t0,t1,x0,v0,a0,j; } SCurveSeg_t;
static SCurveSeg_t s_segs[SCURVE_SEG_MAX];
static int s_n;

/* 单段三阶求值: x = x0 + v0.dt + 0.5.a0.dt2 + (1/6).j.dt3 */
static inline float s_eval(const SCurveSeg_t *s, float t, float *v, float *a) {
    float dt = t - s->t0, dt2 = dt*dt, dt3 = dt2*dt;
    *a = s->a0 + s->j*dt;
    *v = s->v0 + s->a0*dt + (s->j*0.5f)*dt2;
    return s->x0 + s->v0*dt + (s->a0*0.5f)*dt2 + (s->j/6.0f)*dt3;
}

void TRAJ_plan(g_pstMotorDataPtr motor)
{
    motor->traj_Xi = motor->pos_accum;
    float dX = motor->traj_Xf - motor->traj_Xi;
    if (fabsf(dX) < 0.001f) { motor->traj_active = 0; return; }
    int dir = (dX > 0) ? 1 : -1;
    dX = fabsf(dX);

    float Vm = fabsf(motor->traj_v_max);
    float Am = fabsf(motor->traj_a_max);
    float Jm = fabsf(motor->traj_j_max);
    if (Jm < 1e-6f) Jm = 1e-6f;

    float Tj = Am / Jm;
    if (Tj > Vm / Am) Tj = sqrtf(Vm / Jm);
    float A_act = Jm * Tj;
    float Ta = Vm / A_act + Tj;  if (Ta < Tj) Ta = Tj;
    float Tm = fmaxf(Ta - 2.0f*Tj, 0.0f);
    float d_ramp = A_act*(1.5f*Tj*Tm + 0.5f*Tm*Tm + Tj*Tj);
    float Tv = 0.0f;
    if (dX < 2.0f * d_ramp) {
        float Tj_r = powf(dX / (2.0f * Jm), 1.0f/3.0f);
        Vm = Jm * Tj_r * Tj_r;
        if (Vm > fabsf(motor->traj_v_max)) Vm = fabsf(motor->traj_v_max);
        if (Vm > 0) Tj = (Am/Jm < Vm/Am) ? Am/Jm : sqrtf(Vm/Jm);
        A_act = Jm * Tj;
        Ta = Vm / A_act + Tj; if (Ta < Tj) Ta = Tj;
        Tm = fmaxf(Ta - 2.0f*Tj, 0.0f);
        d_ramp = A_act*(1.5f*Tj*Tm + 0.5f*Tm*Tm + Tj*Tj);
        Am = A_act;
    }
    if (dX >= 2.0f * d_ramp) {
        Tv = (dX - 2.0f * d_ramp) / Vm;
    }

    Vm *= dir; Am *= dir; Jm *= dir;
    Ta = fabsf(Ta); Tj = fabsf(Tj); Tv = fabsf(Tv);

    int n = 0;
    float t0=0, x0=motor->traj_Xi, v0=0, a0=0, dt;

    dt = Tj;
    if (dt > 1e-8f) {
        s_segs[n].t0 = t0; s_segs[n].t1 = t0 + dt;
        s_segs[n].x0 = x0; s_segs[n].v0 = v0;
        s_segs[n].a0 = a0; s_segs[n].j = Jm;
        t0 += dt;
        x0 += v0*dt + 0.5f*a0*dt*dt + (1/6.0f)*Jm*dt*dt*dt;
        v0 += a0*dt + 0.5f*Jm*dt*dt;
        a0 += Jm*dt;
        n++;
    }
    dt = Ta - 2*Tj;
    if (dt > 1e-8f) {
        s_segs[n].t0 = t0; s_segs[n].t1 = t0 + dt;
        s_segs[n].x0 = x0; s_segs[n].v0 = v0;
        s_segs[n].a0 = a0; s_segs[n].j = 0;
        t0 += dt;
        x0 += v0*dt + 0.5f*a0*dt*dt;
        v0 += a0*dt;
        n++;
    }
    dt = Tj;
    if (dt > 1e-8f) {
        s_segs[n].t0 = t0; s_segs[n].t1 = t0 + dt;
        s_segs[n].x0 = x0; s_segs[n].v0 = v0;
        s_segs[n].a0 = a0; s_segs[n].j = -Jm;
        t0 += dt;
        x0 += v0*dt + 0.5f*a0*dt*dt - (1/6.0f)*Jm*dt*dt*dt;
        v0 += a0*dt - 0.5f*Jm*dt*dt;
        a0 -= Jm*dt;
        n++;
    }
    dt = Tv;
    if (dt > 1e-8f) {
        s_segs[n].t0 = t0; s_segs[n].t1 = t0 + dt;
        s_segs[n].x0 = x0; s_segs[n].v0 = v0;
        s_segs[n].a0 = a0; s_segs[n].j = 0;
        t0 += dt;
        x0 += v0*dt;
        n++;
    }
    dt = Tj;
    if (dt > 1e-8f) {
        s_segs[n].t0 = t0; s_segs[n].t1 = t0 + dt;
        s_segs[n].x0 = x0; s_segs[n].v0 = v0;
        s_segs[n].a0 = a0; s_segs[n].j = -Jm;
        t0 += dt;
        x0 += v0*dt + 0.5f*a0*dt*dt - (1/6.0f)*Jm*dt*dt*dt;
        v0 += a0*dt - 0.5f*Jm*dt*dt;
        a0 -= Jm*dt;
        n++;
    }
    dt = Ta - 2*Tj;
    if (dt > 1e-8f) {
        s_segs[n].t0 = t0; s_segs[n].t1 = t0 + dt;
        s_segs[n].x0 = x0; s_segs[n].v0 = v0;
        s_segs[n].a0 = a0; s_segs[n].j = 0;
        t0 += dt;
        x0 += v0*dt + 0.5f*a0*dt*dt;
        v0 += a0*dt;
        n++;
    }
    dt = Tj;
    if (dt > 1e-8f) {
        s_segs[n].t0 = t0; s_segs[n].t1 = t0 + dt;
        s_segs[n].x0 = x0; s_segs[n].v0 = v0;
        s_segs[n].a0 = a0; s_segs[n].j = Jm;
        t0 += dt;
        n++;
    }

    s_n = n;
    motor->traj_Tf = t0;
    motor->traj_t = 0.0f;
    motor->traj_active = 1;
    motor->pos_setpoint = motor->traj_Xi;
}

void TRAJ_eval(g_pstMotorDataPtr motor, float dt)
{
    if (!motor->traj_active) return;
    float t = motor->traj_t;
    if (t >= motor->traj_Tf) {
        motor->traj_active = 0;
        motor->pos_setpoint = motor->traj_Xf;
        motor->vel_ff = 0; motor->curr_ff = 0;
        return;
    }
    for (int i = 0; i < s_n; i++) {
        if (t < s_segs[i].t0) break;
        if (t < s_segs[i].t1) {
            float v, a;
            motor->pos_setpoint = s_eval(&s_segs[i], t, &v, &a);
            motor->vel_ff = v;
            motor->curr_ff = motor->inertia * a;
            if (motor->curr_ff > 0.5f) motor->curr_ff = 0.5f;
            if (motor->curr_ff < -0.5f) motor->curr_ff = -0.5f;
            motor->traj_t += dt;
            return;
        }
    }
    motor->traj_active = 0;
    motor->pos_setpoint = motor->traj_Xf;
    motor->vel_ff = 0; motor->curr_ff = 0;
}
