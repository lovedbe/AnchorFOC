#include "bsp/bsp_timer.h"
#include "gd32f30x_timer.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_gpio.h"

/**
 * @brief  初始化TIMER0用于SVPWM输出和FOC ADC触发
 * @note   配置TIMER0中心对齐模式，生成三相带死区互补PWM
 *         使用CH3比较匹配事件生成TRGO触发ADC采样
 * @param  无
 * @retval 无
 */
void TIMER0_SVPWM_Init(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    timer_break_parameter_struct timer_breakpara;

    /* 使能时钟 */
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /* 配置GPIO */
    /* PWM输出引脚 PA8/PA9/PA10 (上桥臂) */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    /* 互补PWM输出引脚 PB13/PB14/PB15 (下桥臂) */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    /* 复位定时器 */
    timer_deinit(TIMER0);

    /* 1. 定时器基本参数配置 */
    /* PWM 频率 20kHz (假设定时器时钟为 120MHz) */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP; /* 中心对齐模式 */
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = PWM_PERIOD;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0, &timer_initpara);

    /* 2. PWM 通道配置 (CH0, CH1, CH2) */
    /*
     * FD6288T: HIN/LIN 均为正相输入 (高电平导通, 低电平关断)
     * PWM0模式下 CNT<CCR 时 OC=HIGH(上管通), OCN自动互补(下管断)
     *            CNT>CCR 时 OC=LOW(上管断),  OCN自动互补(下管通)
     * OC/OCN 极性均设为 HIGH, 使内部 OC/OCN 信号直通到引脚.
     */
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;  
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocintpara);

    /* 配置为PWM0模式 */
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 0);

    /* 开启预装载 */
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);
    timer_auto_reload_shadow_enable(TIMER0);

    /* 3. 死区时间配置 */
    timer_break_struct_para_init(&timer_breakpara);
    timer_breakpara.runoffstate      = TIMER_ROS_STATE_ENABLE;
    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_ENABLE;
    timer_breakpara.deadtime         = PWM_DEAD_TIME; 
    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_LOW;
    timer_breakpara.outputautostate  = TIMER_OUTAUTO_ENABLE;
    timer_breakpara.protectmode      = TIMER_CCHP_PROT_OFF;
    timer_breakpara.breakstate       = TIMER_BREAK_ENABLE;
    timer_break_config(TIMER0, &timer_breakpara);

    /* 4. 配置主输出模式 (用于触发ADC) */
    // TIMER0_TRGO = CH3 比较匹配事件 (O3CPRE)
    // CH3 设为 PWM_PERIOD-1，在计数器接近ARR时触发。
    // 此时 CNT > 所有相位的CCR（死区保证最大占空比 < ARR），
    // 三路OC均为LOW → 上管关断，下管全通 → V0(000)矢量。
    // V0下三个分流电阻都导通，ADC可采样任意两相。
    // O3CPRE在上下计数各触发一次（20kHz×2→ADC注入组使能），
    // ADC_EOIC中断20kHz执行FOC。
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, PWM_PERIOD - 1); // 触发点设在 ARR - 1

    timer_master_slave_mode_config(TIMER0, TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_master_output_trigger_source_select(TIMER0, TIMER_TRI_OUT_SRC_O3CPRE);

    /* 5. 开启定时器和主输出 */
    timer_primary_output_config(TIMER0, ENABLE);
    timer_enable(TIMER0);

    /* 6. 使能TIMER0更新中断 (20kHz)，FOC_CALIB/OPEN_LOOP 模式使用 */
    timer_interrupt_enable(TIMER0, TIMER_INT_UP);
    nvic_irq_enable(TIMER0_UP_IRQn, 0, 0);
}
