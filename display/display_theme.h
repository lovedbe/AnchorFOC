#ifndef DISPLAY_THEME_H
#define DISPLAY_THEME_H

#include <stdint.h>
#include "bsp/bsp_lcd.h"

/* ===================================================================
 *  AnchorFOC 仪表主题配色
 *  各类数据用不同颜色区分，扫一眼就知道是什么
 * =================================================================== */

/* 全屏底层背景 — 纯黑 #000000 → 0x0000 */
#define THEME_BG_DARK        0x0000

/* 顶部状态栏背景 — 亮蓝 #0044AA → 0x0125 */
#define THEME_STATUS_BG      0x0125
#define THEME_STATUS_TEXT    0xFFFF

/* d 轴电流 — 亮青 #44EEEE → 0x0777 */
#define THEME_ID_CYAN        0x0777

/* q 轴电流 — 亮橙 #FF8800 → 0xFC10（转矩主指标） */
#define THEME_IQ_ORANGE      0xFC10

/* 位置 — 亮薄荷 #44FFBB → 0x07F6 */
#define THEME_POS_MINT       0x07F6

/* 转速 — 亮黄 #FFDD44 → 0xFEC8 */
#define THEME_SPD_AMBER      0xFEC8

/* 目标速度 — 暖灰 #CCCCCC → 0xCE59 */
#define THEME_TGT_GRAY       0xCE59

/* 三相电流 — 亮紫 #DD88FF → 0xEC7F */
#define THEME_UVW_PURPLE     0xEC7F

/* CPU/温度 — 纯白 #FFFFFF → 0xFFFF */
#define THEME_SYS_WHITE      0xFFFF

/* 报警 — 亮红 #FF2244 → 0xF811 */
#define THEME_ALARM_RED      0xF811

#endif /* DISPLAY_THEME_H */
