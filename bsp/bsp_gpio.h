#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "gd32f30x_gpio.h"

#define ON 1
#define OFF 0

/*********************LED*********************/
#define     LED_RGBCLK         RCU_GPIOC 
#define     LED_12CLK         RCU_GPIOA
#define     LED_RGBPORT        GPIOC 
#define     LED_12PORT       GPIOA
#define     LEDR_PIN        GPIO_PIN_13
#define     LEDG_PIN        GPIO_PIN_15
#define     LEDB_PIN        GPIO_PIN_14
#define     LED1_PIN        GPIO_PIN_11
#define     LED2_PIN        GPIO_PIN_12


/*********************USART*********************/
#define     USART0_CLK         RCU_GPIOB
#define     USART0_PORT        GPIOB
#define     USART0_TX_PIN        GPIO_PIN_6
#define     USART0_RX_PIN        GPIO_PIN_7
#define     LEDRCtrl(x)     ( x ? gpio_bit_reset(LED_RGBPORT,LEDR_PIN) : gpio_bit_set(LED_RGBPORT,LEDR_PIN) )
#define     LEDGCtrl(x)     ( x ? gpio_bit_reset(LED_RGBPORT,LEDG_PIN) : gpio_bit_set(LED_RGBPORT,LEDG_PIN) )
#define     LEDBCtrl(x)     ( x ? gpio_bit_reset(LED_RGBPORT,LEDB_PIN) : gpio_bit_set(LED_RGBPORT,LEDB_PIN) )
#define     LED1Ctrl(x)     ( x ? gpio_bit_reset(LED_12PORT,LED1_PIN) : gpio_bit_set(LED_12PORT,LED1_PIN) )
#define     LED2Ctrl(x)     ( x ? gpio_bit_reset(LED_12PORT,LED2_PIN) : gpio_bit_set(LED_12PORT,LED2_PIN) )

void LedGpioInit( void );
void Usart0GpioInit( void );
void gpio_bit_toggle(uint32_t gpio_periph,uint32_t pin);
uint8_t LED_Blink(void);
#endif 
