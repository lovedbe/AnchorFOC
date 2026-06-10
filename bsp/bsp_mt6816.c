/*
 * @Author: ntw48 2476672877@qq.com
 * @Date: 2026-05-02 18:00:23
 * @LastEditors: ntw48 2476672877@qq.com
 * @LastEditTime: 2026-05-02 18:44:07
 * @FilePath: \SVPWM_HALL\drivers\mt6816.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bsp/bsp_mt6816.h"
#include "gd32f30x.h"

#define CS_LOW()    GPIO_BC(MT6816_CS_PORT)  = MT6816_CS_PIN
#define CS_HIGH()   GPIO_BOP(MT6816_CS_PORT) = MT6816_CS_PIN


static bool parity_even(uint16_t x)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (x & (1u << i)) cnt++;
    }
    return (cnt & 1u) == 0;
}

static uint16_t transfer16(uint16_t tx)
{
    spi_i2s_data_transmit(SPI2, tx);
    while (RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE));
    return spi_i2s_data_receive(SPI2);
}

void MT6816_Init(void)
{
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI2);

    gpio_init(MT6816_CS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, MT6816_CS_PIN);
    CS_HIGH();

    // JTAG默认占用了PA15, PB3, PB4，需要关闭JTAG才能当做普通引脚或者SPI复用引脚使用
    // 保留 SWD (PA13, PA14) 以便继续下载仿真，关闭 JTAG
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

    gpio_init(MT6816_SCK_PORT,  GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, MT6816_SCK_PIN | MT6816_MOSI_PIN);
    gpio_init(MT6816_MISO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, MT6816_MISO_PIN);

    spi_parameter_struct spi_para;
    spi_struct_para_init(&spi_para);

    spi_para.device_mode          = SPI_MASTER;
    spi_para.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_para.frame_size           = SPI_FRAMESIZE_16BIT;
    spi_para.nss                  = SPI_NSS_SOFT;
    spi_para.endian               = SPI_ENDIAN_MSB;
    spi_para.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_para.prescale             = SPI_PSC_8;

    spi_init(SPI2, &spi_para);
    spi_enable(SPI2);
}

void MT6816_ReadDebug(uint16_t *rx1, uint16_t *rx2)
{
    CS_LOW();
    for (volatile uint32_t i = 0; i < 12; i++) { __NOP(); }
    *rx1 = transfer16(MT6816_CMD_READ(MT6816_REG_ANGLE_HIGH) << 8);
    while (SET == spi_i2s_flag_get(SPI2, SPI_FLAG_TRANS));
    CS_HIGH();

    for (volatile uint32_t i = 0; i < 12; i++) { __NOP(); }

    CS_LOW();
    for (volatile uint32_t i = 0; i < 12; i++) { __NOP(); }
    *rx2 = transfer16(MT6816_CMD_READ(MT6816_REG_ANGLE_LOW) << 8);
    while (SET == spi_i2s_flag_get(SPI2, SPI_FLAG_TRANS));
    CS_HIGH();
}

bool MT6816_Read(MT6816_Data_t *d)
{
    uint16_t sample, rx1, rx2;

    MT6816_ReadDebug(&rx1, &rx2);

    /* 16 位帧: 前 8 SCK 发命令, 后 8 SCK 返回寄存器数据 */
    sample = ((rx1 & 0xFF) << 8) | (rx2 & 0xFF);

    d->pos       = (sample >> 2) & 0x3FFF;
    d->no_mag    = (sample & (1u << 1)) ? true : false;
    d->parity_ok = parity_even(sample);

    return d->parity_ok;
}

/**
 * @brief  简易角度读取, 校验失败则返回上次有效值
 */
float MT6816_GetAngle(void)
{
    MT6816_Data_t enc;
    static uint16_t last_pos = 0;

    if (MT6816_Read(&enc)) {
        last_pos = enc.pos;
    }
    /* 校验失败: 沿用 last_pos, 电机 50µs 内转不了多少 */

    return MT6816_CNT_TO_DEG(last_pos);
}
