#include "display/display_ui.h"
#include "display/display_theme.h"
#include "bsp/bsp_lcd.h"
#include <string.h>
#include <stdio.h>

#define FONT_H  12
#define LINE_H  (FONT_H + 2)
#define ROW_Y(r)  (12 + (r) * LINE_H + 2)
#define MAX_ROWS  12

/* 行缓存：每行上次显示的文本，空串表示未初始化 */
static char row_cache[MAX_ROWS][24] = {{0}};

/* ===================================================================
 *  内部：刷新一行（仅当内容变化时）
 *  @return 1=已刷新, 0=无变化
 * =================================================================== */
static int row_update(uint8_t row, const char *text, uint16_t color)
{
    if (row >= MAX_ROWS) return 0;

    /* 与缓存比较 */
    if (strcmp(row_cache[row], text) == 0) return 0;

    /* 更新缓存 */
    strncpy(row_cache[row], text, sizeof(row_cache[row]) - 1);
    row_cache[row][sizeof(row_cache[row]) - 1] = '\0';

    /* 刷新屏幕 */
    TextSpan s = {4, row_cache[row], color};
    LCD_DrawRow(ROW_Y(row), &s, 1);
    return 1;
}

/* ===================================================================
 *  内部：格式化浮点数到字符串（修正负零）
 * =================================================================== */
static void fmt_float(char *buf, int size, const char *fmt, float val)
{
    snprintf(buf, size, fmt, val);
    if (buf[0] == '-' && buf[1] == '0' && buf[2] == '.') {
        memmove(buf, buf + 1, strlen(buf));  /* 去掉负号 */
    }
}

/* ===================================================================
 *  UI 初始化
 * =================================================================== */
void UI_Init(void)
{
    LCD_Init();
    memset(row_cache, 0, sizeof(row_cache));
    UI_Clear();
}

void UI_Clear(void)
{
    LCD_Fill(0, 0, LCD_W, LCD_H, THEME_BG_DARK);
}

/* ===================================================================
 *  全参数数据列表刷新（差异更新）
 * =================================================================== */
void UI_ShowDataList(const MotorDisplayData *d)
{
    if (!d) return;

    static int prev_state = -1;
    char buf[24];

    /* === 状态栏（蓝底白字，仅变化时重绘） === */
    {
        const char *status = "RUN";
        switch (d->run_state) {
            case 0: status = "INIT"; break;
            case 2: status = "CAL";  break;
            case 3: status = "STOP"; break;
            case 4: status = "RUN";  break;
            case 5: status = "STOP"; break;
            default: status = "???"; break;
        }
        if (d->run_state != prev_state) {
            LCD_Fill(0, 0, LCD_W, 12, THEME_STATUS_BG);
            LCD_ShowString(4, 2, (const uint8_t *)status,
                          THEME_STATUS_TEXT, THEME_STATUS_BG, 12, 0);
            if (d->mode_str) {
                uint8_t len = strlen(d->mode_str);
                LCD_ShowString(LCD_W - 4 - len * 8, 2,
                              (const uint8_t *)d->mode_str,
                              THEME_STATUS_TEXT, THEME_STATUS_BG, 12, 0);
            }
            prev_state = d->run_state;
        }
    }

    /* === 数据行（差异更新，按行缓存比较） === */

    /* 行0: 温度 + 母线电压 */
    snprintf(buf, sizeof(buf), "%.0fC  %.1fV", d->temp, d->vbus);
    row_update(0, buf, THEME_SYS_WHITE);

    /* 行1: d 轴电流 — 青 */
    fmt_float(buf, sizeof(buf), "d:%.3fA", d->id);
    row_update(1, buf, THEME_ID_CYAN);

    /* 行2: q 轴电流 — 橙黄 */
    fmt_float(buf, sizeof(buf), "q:%.3fA", d->iq);
    row_update(2, buf, THEME_IQ_ORANGE);

    /* 行3: 位置 — 薄荷 */
    snprintf(buf, sizeof(buf), "pos:%.2f", d->pos_setpoint);
    row_update(3, buf, THEME_POS_MINT);

    /* 行4: 速度 — 亮琥珀 */
    if (d->speed >= 10000)
        snprintf(buf, sizeof(buf), "spd:%dr", (int)d->speed);
    else if (d->speed >= 1000)
        snprintf(buf, sizeof(buf), "spd:%.1fk", d->speed / 1000.0f);
    else
        snprintf(buf, sizeof(buf), "spd:%dr", (int)d->speed);
    row_update(4, buf, THEME_SPD_AMBER);

    /* 行5: 目标速度 — 浅灰 */
    snprintf(buf, sizeof(buf), "pd:%dr", (int)d->speed_target);
    row_update(5, buf, THEME_TGT_GRAY);

    /* 行6~8: 三相电流 — 淡紫 */
    fmt_float(buf, sizeof(buf), "U:%.3fA", d->iu);
    row_update(6, buf, THEME_UVW_PURPLE);
    fmt_float(buf, sizeof(buf), "V:%.3fA", d->iv);
    row_update(7, buf, THEME_UVW_PURPLE);
    fmt_float(buf, sizeof(buf), "W:%.3fA", d->iw);
    row_update(8, buf, THEME_UVW_PURPLE);

    /* 行9: CPU — 白 */
    snprintf(buf, sizeof(buf), "CPU %d.%d%%", d->cpu_pct / 10, d->cpu_pct % 10);
    row_update(9, buf, THEME_SYS_WHITE);
}
