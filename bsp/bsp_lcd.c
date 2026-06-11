#include "bsp_lcd.h"
#include "display/lcdfont.h"
#include <string.h>

/* ==================== SPI 底层 ==================== */
static void spi_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_SPI0);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    LCD_CS_Set(); LCD_DC_Set(); LCD_RES_Set(); LCD_BLK_Clr();

    spi_parameter_struct p;
    spi_struct_para_init(&p);
    p.device_mode          = SPI_MASTER;
    p.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    p.frame_size           = SPI_FRAMESIZE_8BIT;
    p.nss                  = SPI_NSS_SOFT;
    p.endian               = SPI_ENDIAN_MSB;
    p.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    p.prescale             = SPI_PSC_4;            /* 30MHz */
    spi_init(SPI0, &p);
    spi_enable(SPI0);
}

static void spi_write(uint8_t dat)
{
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
    spi_i2s_data_transmit(SPI0, dat);
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
    (void)spi_i2s_data_receive(SPI0);
}

/* ==================== 基本操作 ==================== */
void LCD_Writ_Bus(uint8_t dat) { spi_write(dat); }

void LCD_WR_DATA8(uint8_t dat)
    { LCD_DC_Set(); LCD_CS_Clr(); spi_write(dat); LCD_CS_Set(); }

void LCD_WR_DATA(uint16_t dat)
    { LCD_DC_Set(); LCD_CS_Clr(); spi_write(dat>>8); spi_write(dat&0xFF); LCD_CS_Set(); }

void LCD_WR_REG(uint8_t cmd)
    { LCD_DC_Clr(); LCD_CS_Clr(); spi_write(cmd); LCD_CS_Set(); }

/* ==================== 设置窗口 ==================== */
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
    LCD_WR_REG(0x2A);
    LCD_WR_DATA(x1 + 24); LCD_WR_DATA(x2 + 24);
    LCD_WR_REG(0x2B);
    LCD_WR_DATA(y1); LCD_WR_DATA(y2);
#else
    LCD_WR_REG(0x2A);
    LCD_WR_DATA(x1); LCD_WR_DATA(x2);
    LCD_WR_REG(0x2B);
    LCD_WR_DATA(y1 + 24); LCD_WR_DATA(y2 + 24);
#endif
    LCD_WR_REG(0x2C);
}

/* ==================== 批量写模式 ==================== */
void LCD_BulkStart(void) { LCD_DC_Set(); LCD_CS_Clr(); }
void LCD_BulkWrite(uint16_t color) { spi_write(color>>8); spi_write(color&0xFF); }
void LCD_BulkEnd(void) { LCD_CS_Set(); }

/* ==================== 初始化（匹配 ST7735S demo） ==================== */
void LCD_Init(void)
{
    spi_gpio_init();
    LCD_RES_Clr(); for(volatile uint32_t i=0;i<50000;i++);
    LCD_RES_Set(); for(volatile uint32_t i=0;i<50000;i++);
    LCD_BLK_Set();

    LCD_WR_REG(0x11); for(volatile uint32_t i=0;i<50000;i++);

    LCD_WR_REG(0xB1); LCD_WR_DATA8(0x05);LCD_WR_DATA8(0x3C);LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB2); LCD_WR_DATA8(0x05);LCD_WR_DATA8(0x3C);LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB3); LCD_WR_DATA8(0x05);LCD_WR_DATA8(0x3C);LCD_WR_DATA8(0x3C);
                      LCD_WR_DATA8(0x05);LCD_WR_DATA8(0x3C);LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB4); LCD_WR_DATA8(0x03);

    LCD_WR_REG(0xC0); LCD_WR_DATA8(0x0E);LCD_WR_DATA8(0x0E);LCD_WR_DATA8(0x04);
    LCD_WR_REG(0xC1); LCD_WR_DATA8(0xC5);
    LCD_WR_REG(0xC2); LCD_WR_DATA8(0x0D);LCD_WR_DATA8(0x00);
    LCD_WR_REG(0xC3); LCD_WR_DATA8(0x8D);LCD_WR_DATA8(0x2A);
    LCD_WR_REG(0xC4); LCD_WR_DATA8(0x8D);LCD_WR_DATA8(0xEE);
    LCD_WR_REG(0xC5); LCD_WR_DATA8(0x06);

    LCD_WR_REG(0x36);
#if USE_HORIZONTAL == 0
    LCD_WR_DATA8(0x08);
#elif USE_HORIZONTAL == 1
    LCD_WR_DATA8(0xC8);
#elif USE_HORIZONTAL == 2
    LCD_WR_DATA8(0x78);
#else
    LCD_WR_DATA8(0xA8);
#endif

    LCD_WR_REG(0x3A); LCD_WR_DATA8(0x55);

    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0x0B);LCD_WR_DATA8(0x17);LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x0D);LCD_WR_DATA8(0x1A);LCD_WR_DATA8(0x19);
    LCD_WR_DATA8(0x16);LCD_WR_DATA8(0x1D);LCD_WR_DATA8(0x21);
    LCD_WR_DATA8(0x26);LCD_WR_DATA8(0x37);LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x00);LCD_WR_DATA8(0x09);LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x10);

    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x0C);LCD_WR_DATA8(0x19);LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x0D);LCD_WR_DATA8(0x1B);LCD_WR_DATA8(0x19);
    LCD_WR_DATA8(0x15);LCD_WR_DATA8(0x1D);LCD_WR_DATA8(0x21);
    LCD_WR_DATA8(0x26);LCD_WR_DATA8(0x39);LCD_WR_DATA8(0x3E);
    LCD_WR_DATA8(0x00);LCD_WR_DATA8(0x09);LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x10);

    for(volatile uint32_t i=0;i<50000;i++);
    LCD_WR_REG(0x29);
}

/* ==================== 绘制 ==================== */
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{
    uint16_t i,j;
    LCD_Address_Set(xsta,ysta,xend-1,yend-1);
    LCD_BulkStart();
    for(i=ysta;i<yend;i++)
        for(j=xsta;j<xend;j++)
            LCD_BulkWrite(color);
    LCD_BulkEnd();
}

void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
    { LCD_Address_Set(x,y,x,y); LCD_WR_DATA(color); }

void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
    uint8_t temp,sizex,t,m=0;
    uint16_t i,TypefaceNum,x0=x;
    sizex=sizey/2;
    TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
    num=num-' ';
    LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);
    LCD_BulkStart();
    for(i=0;i<TypefaceNum;i++) {
        if(sizey==12)     temp=ascii_1206[num][i];
        else if(sizey==16)temp=ascii_1608[num][i];
        else if(sizey==24)temp=ascii_2412[num][i];
        else if(sizey==32)temp=ascii_3216[num][i];
        else { LCD_BulkEnd(); return; }
        for(t=0;t<8;t++) {
            if(!mode) {
                if(temp&(0x01<<t)) LCD_BulkWrite(fc); else LCD_BulkWrite(bc);
                m++; if(m%sizex==0) break;
            } else {
                LCD_BulkEnd(); LCD_DrawPoint(x,y,fc); LCD_BulkStart();
                x++; if((x-x0)==sizex) { x=x0; y++; break; }
            }
        }
    }
    LCD_BulkEnd();
}

void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
    while(*p!='\0') { LCD_ShowChar(x,y,*p,fc,bc,sizey,mode); x+=sizey/2; p++; }
}

static uint32_t mypow(uint8_t m,uint8_t n)
    { uint32_t r=1; while(n--)r*=m; return r; }

void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{
    uint8_t t,temp,enshow=0,sizex=sizey/2;
    for(t=0;t<len;t++) {
        temp=(num/mypow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1)) { if(temp==0) { LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0); continue; } else enshow=1; }
        LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
    }
}

/* ==================== 整行渲染（消除扫描） ==================== */
void LCD_DrawRow(uint16_t y, const TextSpan *spans, uint8_t count)
{
    uint8_t len[4];
    for(uint8_t s=0;s<count;s++) len[s]=strlen(spans[s].text);

    LCD_Address_Set(0,y,LCD_W-1,y+11);
    LCD_BulkStart();
    for(uint8_t row=0;row<12;row++) {
        for(uint16_t col=0;col<LCD_W;col++) {
            uint16_t color=0x0000;
            for(uint8_t s=0;s<count;s++) {
                int16_t rel=(int16_t)col-(int16_t)spans[s].x;
                if(rel<0) continue;
                uint8_t char_idx=(uint8_t)rel/6;
                if(char_idx>=len[s]) continue;
                uint8_t ch=spans[s].text[char_idx]-0x20;
                if(ch>94) continue;
                uint8_t bits=ascii_1206[ch][row];
                if(bits&(0x01<<((uint8_t)rel%6))) { color=spans[s].color; break; }
            }
            LCD_BulkWrite(color);
        }
    }
    LCD_BulkEnd();
}
