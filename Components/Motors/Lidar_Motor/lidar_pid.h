#ifndef LIDAR_PID_H
#define LIDAR_PID_H

#include "main.h"
#include "lidar_uart.h"

/* Expose the TIM4 hardware timer configured for the L298N/TB6612 */
extern TIM_HandleTypeDef htim4;

/* Standard LDS-01 target speed */
#define LIDAR_TARGET_RPM 350
#define LIDAR_KICKSTART_PWM 200

void LidarMotor_Init(void);
void LidarMotor_Stop(void);

/* Call this continuously in your main General_Run() loop */
void LidarMotor_UpdatePID(void);

/* Getter for real-time OLED GUI display */
uint16_t LidarMotor_GetPWM(void);
bool LidarMotor_IsPIDActive(void);

#endif /* LIDAR_PID_H */
