#include "app/sys_malloc.h"
#include "bsp/bsp_usart.h"     /* g_stSystemData */
#include "bsp/bsp_adc.h"       /* g_stEncCalibData */
#include "motor/motor.h"       /* g_stMotorData */
#include "motor/control.h"     /* g_stFoc_RunState_t */
#include "motor/flag_storage.h"/* g_stFlagData */
#include "motor/calib.h"       /* EncCalib_LoadFromFlash */
#include <stdlib.h>
#include <string.h>

/* ===== 全局指针定义（各 .h 中 extern） ===== */

g_pstSystemDataPtr    g_pstSystemData;
g_pstMotorDataPtr     g_pstMotorData    = NULL;
g_pstEncCalibDataPtr  g_pstEncCalibData = NULL;
g_pstFlagDataPtr      g_pstFlagData     = NULL;
g_pstFoc_RunStatePtr  g_pstFoc_RunState = NULL;

/* ===== 内存初始化 ===== */

void System_Memory_Init(void)
{
    g_pstSystemData = (g_pstSystemDataPtr)malloc(sizeof(g_stSystemData));
    if (NULL != g_pstSystemData) {
        memset((void *)g_pstSystemData, 0, sizeof(g_stSystemData));
    }
}

void Motor_Memory_Init(void)
{
    g_pstMotorData = (g_pstMotorDataPtr)malloc(sizeof(g_stMotorData));
    if (NULL != g_pstMotorData) {
        memset((void *)g_pstMotorData, 0, sizeof(g_stMotorData));
        g_pstMotorData->sector = 1;
    }
}

void EncCalib_Memory_Init(void)
{
    g_pstEncCalibData = (g_pstEncCalibDataPtr)malloc(sizeof(g_stEncCalibData));
    if (NULL != g_pstEncCalibData) {
        memset(g_pstEncCalibData, 0, sizeof(g_stEncCalibData));
    }

    /* 启动时尝试从 Flash 加载校准数据 */
    if (EncCalib_LoadFromFlash() == 0) {
        UartPrintf("ENC: loaded from Flash, offset=%d lut=%d\r\n",
                   g_pstMotorData->encoder_offset, g_pstMotorData->offset_lut[0]);
    }
}

void Flag_Memory_Init(void)
{
    g_pstFlagData = (g_pstFlagDataPtr)malloc(sizeof(g_stFlagData));
    if (NULL == g_pstFlagData) {
        return;
    }
    memset(g_pstFlagData, 0, sizeof(g_stFlagData));

    /* 启动时加载 */
    if (Flag_Load() == 0) {
        UartPrintf("FLAG: loaded, cal=%d\r\n", g_pstFlagData->motor_calibrated);
    } else {
        g_pstFlagData->magic = FLAG_MAGIC;
        g_pstFlagData->motor_calibrated = 0;
    }
}

void Foc_State_Init(void)
{
    g_pstFoc_RunState = (g_pstFoc_RunStatePtr)malloc(sizeof(g_stFoc_RunState_t));
    if (NULL != g_pstFoc_RunState) {
        memset((void *)g_pstFoc_RunState, 0, sizeof(g_stFoc_RunState_t));
    }
}
