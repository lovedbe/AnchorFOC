/*!
    \file  gd32f30x_it.c
    \brief interrupt service routines

    \version 2021-03-23, V2.0.0, demo for GD32F30x
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x_it.h"
#include "bsp/bsp_systick.h"
#include "bsp/bsp_usart.h"           /* g_pstSystemData */
#include "gd32f30x_timer.h"  /* TIMER_INT_FLAG_UP, timer_interrupt_flag_xxx */

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{

}
void SysTick_Handler(void)
{
    g_pstSystemData->SystemTimes++;
}

/**
  * @brief  TIMER0 更新中断服务函数 - 20kHz开环SVPWM控制
  * @param  无
  * @retval 无
  */
void TIMER0_UP_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_UP) != RESET)
    {
        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
    }
}

/**
  * @brief  TIMER0 COM中断服务函数 - 换相完成中断
  * @param  无
  * @retval 无
  */
void TIMER0_TRG_CMT_IRQHandler(void)  
{
    /* 检查COM中断标志位 */
    if(timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CMT) != RESET) 
    {
        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CMT);
    }
}

void delay_1ms(uint32_t count)
{
    uint32_t target = g_pstSystemData->SystemTimes + count;
    while (g_pstSystemData->SystemTimes < target);
}
