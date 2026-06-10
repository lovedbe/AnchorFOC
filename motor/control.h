#ifndef __CONTROL_H
#define __CONTROL_H

#include "motor/motor.h"

enum FOC_State {
    FOC_INIT = 0,
    FOC_CHECK = 1,
    FOC_CALIB = 2,
    FOC_STAND = 3,
    FOC_RUN = 4,
    FOC_STOP = 5,
    FOC_HAUFT = 6,
    FOC_RESET = 7
};

enum CALIB_State {
    CALIB_IDLE = 0,
    CALIB_ADC = 1,
    CALIB_RL = 2,
    CALIB_ENC = 3,
    CALIB_DONE = 4
};

typedef struct WarnStatus //报警结构体
{
    uint8_t level;   // 级别
    uint8_t module;   // 类别
    uint8_t code;    // 代码  
} g_stWarnStatus_t, *g_pstWarnStatusPtr;

typedef struct Foc_RunState
{
    uint8_t run_state;        // 0: 停止, 1: 闭环运行, 2: 故障, 3: 开环运行
    uint8_t CALIB_State;       // 校准状态

} g_stFoc_RunState_t, *g_pstFoc_RunStatePtr;

extern g_pstFoc_RunStatePtr g_pstFoc_RunState;

/* 位置环控制模式 */
#define POS_MODE_DIRECT  0   /* 直接用pos_gain锁位置 */
#define POS_MODE_TRAP    1   /* 梯形轨迹规划 */

void FocSystemRun(void);
void Control_Current_Init(g_pstMotorDataPtr motor, float Ts, float kp_id, float ki_id, float kp_iq, float ki_iq, float u_limit);
void Control_Current_Step(g_pstMotorDataPtr motor, float Id_ref, float Iq_ref);
void FOC_Update_Current_Gain(float bandwidth);
void Control_Speed_Init(g_pstMotorDataPtr motor, float vel_gain, float vel_ki, float current_limit);
float Control_Speed_Step(g_pstMotorDataPtr motor, float vel_setpoint_revs, float vel_feedback_revs);
void Control_OpenLoop_Step(g_pstMotorDataPtr motor);
void Control_AlphaBeta_Inject_Step(g_pstMotorDataPtr motor);
void Control_DQ_Inject_Step(g_pstMotorDataPtr motor);
float Motor_Get_RPM(void);
float Control_Position_Step(g_pstMotorDataPtr motor, float dt);

#endif // __CONTROL_H
