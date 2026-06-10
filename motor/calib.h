#ifndef CALIB_H
#define CALIB_H

#include <stdint.h>

/* Flash 存储地址：校准结果持久化 */
#define ENC_CALIB_FLASH_ADDR  0x0801F000

/* ---- 校准状态机 ---- */
int     CALIB_ADC_Run(void);
int     CALIB_RL_Run(void);
int     CALIB_ENC_Run(void);
uint8_t CALIB_Run(void);

/* ---- Flash 存取 ---- */
int     EncCalib_LoadFromFlash(void);

#endif /* CALIB_H */
