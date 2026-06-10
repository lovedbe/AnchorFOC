#ifndef SYS_MALLOC_H
#define SYS_MALLOC_H

#include <stdint.h>

/* 所有 *Memory_Init / *State_Init 集中声明 */

void System_Memory_Init(void);
void Motor_Memory_Init(void);
void EncCalib_Memory_Init(void);
void Flag_Memory_Init(void);
void Foc_State_Init(void);

#endif /* SYS_MALLOC_H */
