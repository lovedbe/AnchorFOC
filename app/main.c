
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
}

int main(void)
{
    prvSetupHardware();
    UartPrintf("open-loop SVPWM start\r\n");

    while (1)
    {
        FocSystemRun();
        UartCmdHandle();
        UartPollDmaTx();
    }
}
