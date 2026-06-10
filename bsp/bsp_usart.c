#include "bsp/bsp_usart.h"
#include "components/fifo.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "bsp/bsp_timer.h"
#include "bsp/bsp_adc.h"
#include "motor/motor.h"
#include "motor/svpwm.h"
#include "motor/trajectory.h"
#include "bsp/bsp_mt6816.h"
#include "motor/control.h"
#include "motor/flag_storage.h"
#include <math.h>

/* ============================================================
 *  USART0 DMA 驱动
 *  架构: FIFO + DMA 循环接收(RX) / 轮询发送(TX)
 *  RX DMA: DMA0 CH4, 循环模式, full/IDLE 中断
 *  TX DMA: DMA0 CH3, 普通模式, 主循环 uart_poll_dma_tx() 启动
 * ============================================================ */

/* -------- 缓冲区 -------- */
static uint8_t s_tx_fifo_buf[UART_TX_BUF_SIZE];
static uint8_t s_rx_fifo_buf[UART_RX_BUF_SIZE];
static g_stFifo s_tx_fifo;
static g_stFifo s_rx_fifo;

/* RX DMA 循环缓冲 */
static uint8_t s_dmarx_buf[UART_DMA_RX_BUF_SIZE];

/* TX DMA 发送缓冲（暂存从 FIFO 读出的数据） */
static uint8_t s_dmatx_buf[UART_DMA_TX_BUF_SIZE];

/* TX DMA 状态 */
static volatile uint8_t s_tx_dma_busy = 0;

/* 调试统计 */
static volatile uint32_t s_tx_cnt = 0;
static volatile uint32_t s_rx_cnt = 0;

/* RX DMA 上次处理位置 */
static volatile uint16_t s_last_dmarx_pos = 0;

/* ==================== FIFO 锁 ==================== */
static void Fifo_Lock(void)   { __disable_irq(); }
static void Fifo_Unlock(void) { __enable_irq(); }

/* ==================== RX DMA 重启 ==================== */
static void DmaRx_Start(void)
{
    dma_channel_disable(DMA0, DMA_CH4);
    dma_deinit(DMA0, DMA_CH4);

    dma_parameter_struct p;
    dma_struct_para_init(&p);
    p.direction       = DMA_PERIPHERAL_TO_MEMORY;
    p.periph_addr     = (uint32_t)&USART_DATA(USART0);
    p.memory_addr     = (uint32_t)s_dmarx_buf;
    p.number          = UART_DMA_RX_BUF_SIZE;
    p.periph_width    = DMA_PERIPHERAL_WIDTH_8BIT;
    p.memory_width    = DMA_MEMORY_WIDTH_8BIT;
    p.periph_inc      = DMA_PERIPH_INCREASE_DISABLE;
    p.memory_inc      = DMA_MEMORY_INCREASE_ENABLE;
    p.priority        = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH4, &p);

    dma_circulation_enable(DMA0, DMA_CH4);
    dma_interrupt_enable(DMA0, DMA_CH4, DMA_INT_FTF | DMA_INT_HTF);

    dma_channel_enable(DMA0, DMA_CH4);
}

/* ==================== 获取 DMA RX 剩余计数 ==================== */
static uint16_t DmaRx_GetRemain(void)
{
    return (uint16_t)dma_transfer_number_get(DMA0, DMA_CH4);
}

/* ==================== 读取 DMA RX 数据 → RX FIFO ==================== */
static void DmaRx_Drain(void)
{
    uint16_t remain  = DmaRx_GetRemain();
    uint16_t written = UART_DMA_RX_BUF_SIZE - remain;

    /* 计算新收到的字节数（环形缓冲） */
    uint16_t new_bytes;
    if (written >= s_last_dmarx_pos)
        new_bytes = written - s_last_dmarx_pos;
    else
        new_bytes = UART_DMA_RX_BUF_SIZE - s_last_dmarx_pos + written;

    if (new_bytes == 0) return;

    /* 从 DMA 缓冲读入 RX FIFO */
    uint32_t pos = s_last_dmarx_pos;
    for (uint16_t i = 0; i < new_bytes; i++) {
        uint8_t b = s_dmarx_buf[pos++];
        if (pos >= UART_DMA_RX_BUF_SIZE) pos = 0;
        FIFO_Write(&s_rx_fifo, &b, 1);
    }
    s_rx_cnt += new_bytes;
    s_last_dmarx_pos = written;
}

/* ==================== USART0 初始化 ==================== */
void USART0_Init(void)
{
    /* ---- FIFO 注册 ---- */
    FIFO_Init(&s_tx_fifo, s_tx_fifo_buf, UART_TX_BUF_SIZE,
                  Fifo_Lock, Fifo_Unlock);
    FIFO_Init(&s_rx_fifo, s_rx_fifo_buf, UART_RX_BUF_SIZE,
                  Fifo_Lock, Fifo_Unlock);

    /* ---- USART 硬件初始化（与原中断版保持一致） ---- */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    /* ---- DMA 时钟 ---- */
    rcu_periph_clock_enable(RCU_DMA0);

    /* ---- RX DMA（循环模式） ---- */
    DmaRx_Start();
    usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);

    /* ---- 中断 ---- */
    {
        volatile uint32_t tmp = USART_STAT0(USART0);
        tmp = USART_DATA(USART0);
        (void)tmp;
    }
    usart_interrupt_enable(USART0, USART_INT_IDLE);

    nvic_irq_enable(USART0_IRQn, 2, 2);
    nvic_irq_enable(DMA0_Channel3_IRQn, 2, 2);
    nvic_irq_enable(DMA0_Channel4_IRQn, 2, 2);
}

/* ============================================================
 *  API: 非阻塞读写
 * ============================================================ */
uint32_t UartWrite(const uint8_t *data, uint32_t size)
{
    return FIFO_Write(&s_tx_fifo, data, size);
}

uint32_t UartRead(uint8_t *data, uint32_t size)
{
    return FIFO_Read(&s_rx_fifo, data, size);
}

uint32_t UartReadable(void)
{
    return FIFO_GetOccupySize(&s_rx_fifo);
}

/* ============================================================
 *  UartPrintf — 格式化输出（写入 TX FIFO）
 * ============================================================ */
void UartPrintf(const char *fmt, ...)
{
    char buf[UART_DMA_TX_BUF_SIZE];

    va_list ap;
    va_start(ap, fmt);
    uint16_t len = (uint16_t)vsprintf(buf, fmt, ap);
    va_end(ap);

    if (len > sizeof(buf)) len = sizeof(buf);
    if (len == 0) return;

    /* 写入 TX FIFO，由 UartPollDmaTx 通过 DMA 发送 */
    UartWrite((uint8_t *)buf, len);
}

/* ============================================================
 *  UartPollDmaTx — 主循环调用，从 TX FIFO 启动 DMA 发送
 * ============================================================ */
void UartPollDmaTx(void)
{
    if (s_tx_dma_busy) return;

    uint32_t len = FIFO_Read(&s_tx_fifo, s_dmatx_buf, UART_DMA_TX_BUF_SIZE);
    if (len == 0) return;

    s_tx_dma_busy = 1;
    s_tx_cnt += len;

    /* 配置 DMA TX */
    dma_channel_disable(DMA0, DMA_CH3);
    dma_deinit(DMA0, DMA_CH3);

    dma_parameter_struct p;
    dma_struct_para_init(&p);
    p.direction       = DMA_MEMORY_TO_PERIPHERAL;
    p.periph_addr     = (uint32_t)&USART_DATA(USART0);
    p.memory_addr     = (uint32_t)s_dmatx_buf;
    p.number          = len;
    p.periph_width    = DMA_PERIPHERAL_WIDTH_8BIT;
    p.memory_width    = DMA_MEMORY_WIDTH_8BIT;
    p.periph_inc      = DMA_PERIPH_INCREASE_DISABLE;
    p.memory_inc      = DMA_MEMORY_INCREASE_ENABLE;
    p.priority        = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH3, &p);

    dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
    usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE);
    dma_channel_enable(DMA0, DMA_CH3);
}

/* ============================================================
 *  中断服务例程
 * ============================================================ */

/* DMA0 CH4: RX 循环接收 */
void DMA0_Channel4_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA0, DMA_CH4, DMA_INT_FLAG_FTF) != RESET) {
        dma_interrupt_flag_clear(DMA0, DMA_CH4, DMA_INT_FLAG_FTF);
        DmaRx_Drain();
    }
    if (dma_interrupt_flag_get(DMA0, DMA_CH4, DMA_INT_FLAG_HTF) != RESET) {
        dma_interrupt_flag_clear(DMA0, DMA_CH4, DMA_INT_FLAG_HTF);
        DmaRx_Drain();
    }
}

/* DMA0 CH3: TX 发送完成 */
void DMA0_Channel3_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF) != RESET) {
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);
        dma_channel_disable(DMA0, DMA_CH3);
        usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_DISABLE);
        s_tx_dma_busy = 0;
    }
}

/* USART0: IDLE 帧检测中断 */
void USART0_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE) != RESET) {
        /* 清除 IDLE 标志 */
        volatile uint32_t tmp = USART_STAT0(USART0);
        tmp = USART_DATA(USART0);
        (void)tmp;

        /* 排空 DMA RX 残留数据 */
        DmaRx_Drain();
    }
}

/* ============================================================
 *  UartCmdHandle — 兼容旧协议 (0xAA5555AA)
 *  状态机逐字节解析（不丢数据）
 * ============================================================ */

typedef struct {
    uint8_t  RxDataBuff[UART_CMD_DATA_BUFF_SIZE];
    uint8_t  RxDataCount;
    uint8_t  DebugEvent;
} g_stUartData_Compat;
static g_stUartData_Compat s_uart_data;

/* 解码状态机 */
#define DECODE_STATE_SYNC1  0   /* 等待 0xAA */
#define DECODE_STATE_SYNC2  1   /* 等待 0x55 */
#define DECODE_STATE_SYNC3  2   /* 等待 0x55 */
#define DECODE_STATE_SYNC4  3   /* 等待 0xAA */
#define DECODE_STATE_DATA   4   /* 接收载荷 */

static uint8_t s_decode_state = DECODE_STATE_SYNC1;
static uint8_t s_decode_idx   = 0;

void UartCmdHandle(void)
{
    uint8_t b;

    while (FIFO_Read(&s_rx_fifo, &b, 1) > 0) {
        switch (s_decode_state) {
        case DECODE_STATE_SYNC1:
            if (b == 0xAA) s_decode_state = DECODE_STATE_SYNC2;
            break;
        case DECODE_STATE_SYNC2:
            s_decode_state = (b == 0x55) ? DECODE_STATE_SYNC3 : DECODE_STATE_SYNC1;
            break;
        case DECODE_STATE_SYNC3:
            s_decode_state = (b == 0x55) ? DECODE_STATE_SYNC4 : DECODE_STATE_SYNC1;
            break;
        case DECODE_STATE_SYNC4:
            if (b == 0xAA) {
                s_decode_state = DECODE_STATE_DATA;
                s_decode_idx = 0;
            } else {
                s_decode_state = DECODE_STATE_SYNC1;
            }
            break;
        case DECODE_STATE_DATA:
            s_uart_data.RxDataBuff[s_decode_idx++] = b;
            if (s_decode_idx >= UART_CMD_DATA_BUFF_SIZE) {
                s_uart_data.RxDataCount = s_decode_idx;
                s_uart_data.DebugEvent = 1;
                s_decode_state = DECODE_STATE_SYNC1;
            }
            break;
        }
    }

    /* 处理命令 */
    while (s_uart_data.DebugEvent) {
        uint8_t cmd = s_uart_data.RxDataBuff[0];

        switch (cmd) {
        case 0x01: {
            uint8_t action = s_uart_data.RxDataBuff[1];
            if (action == 0x01) {
                if (g_pstMotorData) {
                    g_pstMotorData->open_loop_speed = 5.0f;
                    g_pstMotorData->run_state = 3;
                    UartPrintf("Motor started\r\n");
                }
            } else {
                if (g_pstMotorData) {
                    TIMER_CH0CV(TIMER0) = 0;
                    TIMER_CH1CV(TIMER0) = 0;
                    TIMER_CH2CV(TIMER0) = 0;
                    g_pstMotorData->open_loop_speed = 0.0f;
                    g_pstMotorData->open_loop_amp   = 0.0f;
                    g_pstMotorData->run_state = 0;
                    UartPrintf("Motor stopped\r\n");
                }
            }
        } break;

        case 0x02:
            UartPrintf("run_state=%d speed=%.1fHz amp=%.3f\r\n",
                g_pstMotorData ? g_pstMotorData->run_state : -1,
                g_pstMotorData ? g_pstMotorData->open_loop_speed : 0,
                g_pstMotorData ? g_pstMotorData->open_loop_amp : 0);
            break;

        case 0x07:
            if (g_pstMotorData) {
                g_pstMotorData->speed_fb_mode = s_uart_data.RxDataBuff[1];
                UartPrintf("SPD_FB: %s\r\n",
                    g_pstMotorData->speed_fb_mode ? "DIRECT" : "PLL");
            }
            break;

        case 0x08: {
            if (g_pstMotorData) {
                uint16_t rpm = s_uart_data.RxDataBuff[1] |
                              (s_uart_data.RxDataBuff[2] << 8);
                g_pstMotorData->vel_setpoint = (float)rpm / 60.0f;
                UartPrintf("TARGET: %u RPM\r\n", rpm);
            }
        } break;

        case 0x09: {
            if (!g_pstMotorData) break;
            uint8_t mode = s_uart_data.RxDataBuff[1];
            switch (mode) {
            case 0:
                TIMER_CH0CV(TIMER0) = 0; TIMER_CH1CV(TIMER0) = 0; TIMER_CH2CV(TIMER0) = 0;
                g_pstMotorData->run_state = FOC_STOP;
                g_pstMotorData->vel_integrator = 0.0f;
                UartPrintf("MODE: STOP\r\n"); break;
            case 1:
                g_pstMotorData->run_state = FOC_CALIB;
                g_pstFoc_RunState->CALIB_State = CALIB_ADC;
                if (g_pstFlagData) g_pstFlagData->motor_calibrated = 0;
                UartPrintf("MODE: CALIB\r\n"); break;
            case 2:
                g_pstMotorData->run_state = FOC_RUN;
                g_pstMotorData->vel_integrator = 0.0f;
                g_pstMotorData->pos_gain = 0.0f;
                UartPrintf("MODE: SPEED\r\n"); break;
            case 4:
                g_pstMotorData->run_state = FOC_RUN;
                g_pstMotorData->vel_integrator = 0.0f;
                if (g_pstMotorData->pos_gain < 0.001f) g_pstMotorData->pos_gain = 55.0f;
                UartPrintf("MODE: POSITION (gain=%.0f)\r\n", g_pstMotorData->pos_gain); break;
            default:
                UartPrintf("MODE: unknown %d\r\n", mode); break;
            }
        } break;

        case 0x0A: {
            if (g_pstMotorData) {
                float kp;
                memcpy(&kp, &s_uart_data.RxDataBuff[1], 4);
                if (kp < 0.0f) kp = 0.0f;
                if (kp > 2.0f) kp = 2.0f;
                g_pstMotorData->vel_gain = kp;
                UartPrintf("KP: %.4f\r\n", kp);
            }
        } break;

        case 0x0B: {
            if (g_pstMotorData) {
                float ki;
                memcpy(&ki, &s_uart_data.RxDataBuff[1], 4);
                if (ki < 0.0f) ki = 0.0f;
                if (ki > 5.0f) ki = 5.0f;
                g_pstMotorData->vel_integrator_gain = ki;
                UartPrintf("KI: %.4f\r\n", ki);
            }
        } break;

        case 0x0C: {
            if (g_pstMotorData) {
                float pos;
                memcpy(&pos, &s_uart_data.RxDataBuff[1], 4);
                g_pstMotorData->pos_setpoint = pos;
                if (g_pstMotorData->pos_gain > 0.0f) {
                    float dist = fabsf(pos - g_pstMotorData->pos_accum);
                    if (dist > 1.0f) {
                        g_pstMotorData->traj_Xf = pos;
                        TRAJ_plan(g_pstMotorData);
                    }
                }
                UartPrintf("POS: %.2f rev\r\n", pos);
            }
        } break;

        case 0x0D: {
            if (g_pstMotorData) {
                float gain;
                memcpy(&gain, &s_uart_data.RxDataBuff[1], 4);
                if (gain < 0.0f) gain = 0.0f;
                if (gain > 200.0f) gain = 200.0f;
                g_pstMotorData->pos_gain = gain;
                UartPrintf("POS_GAIN: %.1f\r\n", gain);
            }
        } break;

        case 0x0E: {
            if (g_pstMotorData) {
                float deg;
                memcpy(&deg, &s_uart_data.RxDataBuff[1], 4);
                float pos_rev = deg / 360.0f;
                g_pstMotorData->pos_setpoint = pos_rev;
                if (g_pstMotorData->pos_gain > 0.0f) {
                    float dist = fabsf(pos_rev - g_pstMotorData->pos_accum);
                    if (dist > 1.0f) {
                        g_pstMotorData->traj_Xf = pos_rev;
                        TRAJ_plan(g_pstMotorData);
                    }
                }
                UartPrintf("ANGLE: %.1f deg (%.3f rev)\r\n", deg, pos_rev);
            }
        } break;

        default:
            UartPrintf("Unknown Command: 0x%02X\r\n", cmd);
            break;
        }

        s_uart_data.DebugEvent = 0;
    }
}

