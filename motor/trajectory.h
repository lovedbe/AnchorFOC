#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "motor/motor.h"  /* g_pstMotorDataPtr */

void TRAJ_plan(g_pstMotorDataPtr motor);
void TRAJ_eval(g_pstMotorDataPtr motor, float dt);

#endif /* TRAJECTORY_H */
