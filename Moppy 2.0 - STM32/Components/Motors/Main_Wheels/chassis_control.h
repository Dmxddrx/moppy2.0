#ifndef CHASSIS_CONTROL_H_
#define CHASSIS_CONTROL_H_

#include "main.h"
#include "main_wheels.h"
#include <stdbool.h>

/* Global Odometry Variables */
extern float total_distance_cm;
extern float total_dist_L_cm;
extern float total_dist_R_cm;

/* Chassis Control Functions */
void Chassis_Init(void);
void Chassis_SetTargetSpeed(int16_t left_ticks_per_loop, int16_t right_ticks_per_loop);
void Chassis_ResetOdometry(void);
void Chassis_Update(void);

void Chassis_SetVelocity(float linear_cm_s,
                         float angular_deg_s);
bool Chassis_StartTurn(float angle_deg,
                       float max_rate_deg_s);

void Chassis_MotionUpdate(void);
bool Chassis_MotionIsDone(void);
void Chassis_CancelMotion(void);
bool Chassis_MotionHasFault(void);
#endif /* CHASSIS_CONTROL_H_ */
