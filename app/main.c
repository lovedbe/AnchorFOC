
#include "gd32f30x.h"
#include "app/sys_malloc.h"
#include "app/usr_config.h"
#include "bsp/bsp_systick.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_usart.h"
#include "bsp/bsp_timer.h"
#include "bsp/bsp_adc.h"
#include "bsp/bsp_mt6816.h"
#include "motor/svpwm.h"
#include "motor/motor.h"
#include "motor/control.h"
#include "motor/flag_storage.h"
#include "bsp/bsp_dwt.h"
#include "bsp/bsp_lcd.h"
#include "display/display_ui.h"
#include "display/display_theme.h"

void prvSetupHardware( void )
{
    System_Memory_Init();
    Motor_Memory_Init();
    EncCalib_Memory_Init();
    Flag_Memory_Init();
    Foc_State_Init();
    systick_config();
    LedGpioInit();
    /* 示波器测量 ADC ISR 频率：PB9 推挽输出 */
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    MT6816_Init();
    ADC_FOC_Init();
    Usart0GpioInit();
    USART0_Init();
    TIMER0_SVPWM_Init();
    SVPWM_Init();

    USR_Set_Default_Params();

    /* DWT 周期计数器初始化（ADC中断计时 + CPU占用率） */
    DWT_Init();

    /* TFT 显示初始化 */
    UI_Init();
    UartPrintf("TFT init OK\r\n");
}

int main(void)
{
    prvSetupHardware();
    UartPrintf("open-loop SVPWM start\r\n");

    uint32_t t_disp = 0;
    static uint16_t s_cpu = 0;

    while (1)
    {
        DWT_IDLE_START();
        uint32_t now = g_pstSystemData->SystemTimes;

        /* 每 100ms 刷新一次显示（差异更新，只写变化行） */
        if (now - t_disp >= 100) {
            MotorDisplayData dd;
            const char *mode = (g_pstMotorData->pos_gain > 0.0f) ? "POS" : "SPD";

            dd.run_state    = g_pstMotorData->run_state;
            dd.mode_str     = mode;
            dd.temp         = 0.0f;
            dd.vbus         = 0.0f;
            dd.id           = g_pstMotorData->Id_filt;
            dd.iq           = g_pstMotorData->Iq_filt;
            dd.pos_setpoint = g_pstMotorData->pos_setpoint;
            dd.pos_actual   = g_pstMotorData->pos_accum;
            dd.speed        = g_pstMotorData->motor_speed_rpm;
            dd.speed_target = g_pstMotorData->vel_setpoint * 60.0f;
            dd.iu           = g_pstMotorData->Iu;
            dd.iv           = g_pstMotorData->Iv;
            dd.iw           = g_pstMotorData->Iw;
            dd.warn_level   = 0;
            dd.warn_code    = 0;
            dd.cpu_pct      = s_cpu;
            UI_ShowDataList(&dd);

            t_disp = now;
        }

        /* 每 1s 打印 + 更新 CPU */
        static uint32_t t_dbg = 0;
        if (now - t_dbg >= 1000) {
            float avg, min, max, last;
            s_cpu = DWT_CalcCPUUsage();
            DWT_GetAdcStats(&avg, &min, &max, &last);
            UartPrintf("ADC_ISR: avg=%.2fus min=%.2fus max=%.2fus last=%.2fus CPU=%d.%d%%\r\n",
                       avg, min, max, last, s_cpu / 10, s_cpu % 10);
            t_dbg = now;
        }

        /* UART 命令优先处理 */
        UartCmdHandle();
        UartPollDmaTx();

        FocSystemRun();
        DWT_IDLE_END();
    }
}
