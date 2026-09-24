#ifndef CHASSIS_CONTROL_H_
#define CHASSIS_CONTROL_H_

#include "main.h"
#include "main_wheels.h"

/* Global Odometry Variables */
extern float total_distance_cm;
extern float total_dist_L_cm;
extern float total_dist_R_cm;

/* Chassis Control Functions */
void Chassis_Init(void);
void Chassis_Update(void);

#endif /* CHASSIS_CONTROL_H_ */
