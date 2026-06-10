#include "app/usr_config.h"
#include "motor/motor.h"
#include "motor/control.h"
#include "bsp/bsp_adc.h"

/**
 * @brief  应用用户默认参数
 * 在 prvSetupHardware 最后调用，覆盖 g_pstMotorData 等结构体中的零值
 */
void USR_Set_Default_Params(void)
{
    /* ---- 校准启动状态 ---- */
    g_pstFoc_RunState->CALIB_State = CALIB_ADC;
    g_pstMotorData->run_state = FOC_CALIB;

    /* ---- 开环 V/f 斜坡参数 ---- */
    g_pstMotorData->open_loop_speed = OL_START_FREQ_HZ;
    g_pstMotorData->target_speed    = OL_TARGET_FREQ_HZ;
    g_pstMotorData->ramp_rate       = OL_RAMP_RATE_HZPS;
    g_pstMotorData->v_boost         = OL_V_BOOST;
    g_pstMotorData->v_per_hz        = OL_V_PER_HZ;
    g_pstMotorData->open_loop_amp   = OL_INIT_AMP;

    /* ---- ADC 通道增益修正（标定后会覆盖） ---- */
    g_pstMotorData->gain_v1   = 1.0f;
    g_pstMotorData->gain_vw1  = 1.0f;
    g_pstMotorData->gain_uv0  = 1.0f;
}
