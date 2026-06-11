#ifndef BSP_LCD_H
#define BSP_LCD_H

#include <stdint.h>
#include "gd32f30x.h"

/* ==================== 横竖屏选择 ====================
 *  0/1: 竖屏 (80×160)  2/3: 横屏 (160×80)
 * ==================================================== */
#define USE_HORIZONTAL 1

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 80
#define LCD_H 160
#else
#define LCD_W 160
#define LCD_H 80
#endif

/* ==================== 引脚定义（SPI0） ==================== */
#define LCD_SCLK_Clr()  GPIO_BC(GPIOA) = GPIO_PIN_5
#define LCD_SCLK_Set()  GPIO_BOP(GPIOA) = GPIO_PIN_5
#define LCD_MOSI_Clr()  GPIO_BC(GPIOA) = GPIO_PIN_7
#define LCD_MOSI_Set()  GPIO_BOP(GPIOA) = GPIO_PIN_7
#define LCD_RES_Clr()   GPIO_BC(GPIOC) = GPIO_PIN_15
#define LCD_RES_Set()   GPIO_BOP(GPIOC) = GPIO_PIN_15
#define LCD_DC_Clr()    GPIO_BC(GPIOC) = GPIO_PIN_14
#define LCD_DC_Set()    GPIO_BOP(GPIOC) = GPIO_PIN_14
#define LCD_CS_Clr()    GPIO_BC(GPIOC) = GPIO_PIN_13
#define LCD_CS_Set()    GPIO_BOP(GPIOC) = GPIO_PIN_13
#define LCD_BLK_Clr()   GPIO_BC(GPIOA) = GPIO_PIN_6
#define LCD_BLK_Set()   GPIO_BOP(GPIOA) = GPIO_PIN_6

/* ==================== 基本操作 ==================== */
void LCD_Writ_Bus(uint8_t dat);
void LCD_WR_DATA8(uint8_t dat);
void LCD_WR_DATA(uint16_t dat);
void LCD_WR_REG(uint8_t cmd);
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void LCD_Init(void);

/* ==================== 批量写模式（消除扫描） ==================== */
void LCD_BulkStart(void);
void LCD_BulkWrite(uint16_t color);
void LCD_BulkEnd(void);

/* ==================== 绘制 ==================== */
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);

/* 本行内的一段文字 */
typedef struct {
    uint16_t x;
    const char *text;
    uint16_t color;
} TextSpan;

/* 整行渲染（一次 Address_Set 消除扫描） */
void LCD_DrawRow(uint16_t y, const TextSpan *spans, uint8_t count);

/* 基本颜色 */
#define WHITE         0xFFFF
#define BLACK         0x0000
#define BLUE          0x001F
#define RED           0xF800
#define GREEN         0x07E0
#define CYAN          0x7FFF
#define YELLOW        0xFFE0
#define GRAY          0x8430

#endif
