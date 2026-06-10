#include "bsp/bsp_gpio.h"
#include "bsp/bsp_usart.h"
#include "bsp/bsp_timer.h"
#include "bsp/bsp_mt6816.h"

void LedGpioInit( void )
{
    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    // rcu_periph_clock_enable(LED_RGBCLK);
    rcu_periph_clock_enable(LED_12CLK);
    // gpio_init(LED_RGBPORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LEDR_PIN | LEDG_PIN | LEDB_PIN );
    gpio_init(LED_12PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED1_PIN | LED2_PIN );
    LEDRCtrl(OFF);
    LEDGCtrl(ON);
    LEDBCtrl(OFF);
    LED1Ctrl(OFF);
    LED2Ctrl(OFF);
}

void Usart0GpioInit( void )
{
    gpio_pin_remap_config(GPIO_USART0_REMAP, ENABLE);
    rcu_periph_clock_enable(USART0_CLK);
    rcu_periph_clock_enable(RCU_USART0);
    gpio_init(USART0_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,USART0_TX_PIN);
    gpio_init(USART0_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,USART0_RX_PIN);
}

void gpio_bit_toggle(uint32_t gpio_periph,uint32_t pin)
{
    gpio_bit_write(gpio_periph,pin,(bit_status)(gpio_input_bit_get(gpio_periph,pin)^1));
}

uint8_t LED_Blink(void)
{
    static uint32_t TimesCount = 0;
    if(TimesCount == g_pstSystemData->SystemTimes/500)
    {
        return 0;
    }
    TimesCount = g_pstSystemData->SystemTimes/500;
    gpio_bit_toggle(LED_12PORT,LED1_PIN);
    
    // gpio_bit_toggle(LED_12PORT,LED2_PIN);
    return 0;
}


