#ifndef MAIN_WHEELS_H_
#define MAIN_WHEELS_H_

#include "main.h"
#include <stdint.h>
#include <stdlib.h>

/* Expose the timers configured in main.c */
extern TIM_HandleTypeDef htim1;  // Left Wheel Encoder[cite: 12]
extern TIM_HandleTypeDef htim3;  // Wheel PWMs[cite: 12]
extern TIM_HandleTypeDef htim8;  // Right Wheel Encoder[cite: 12]

/* Initialization function */
void MainWheels_Init(void);

/* Individual wheel speed control (Input range: -999 to 999) */
void MainWheels_SetLeftSpeed(int16_t speed);
void MainWheels_SetRightSpeed(int16_t speed);

/* High-level unified motion control */
void MainWheels_Forward(uint16_t speed);
void MainWheels_Backward(uint16_t speed);
void MainWheels_Stop(void);

/* Hardware state reading functions */
uint16_t MainWheels_GetLeftSpeed(void);
uint16_t MainWheels_GetRightSpeed(void);
int8_t MainWheels_GetLeftDirection(void);
int8_t MainWheels_GetRightDirection(void);

/* Encoder reading functions */
int16_t MainWheels_GetLeftEncoder(void);
int16_t MainWheels_GetRightEncoder(void);
void MainWheels_ResetEncoders(void);

#endif /* MAIN_WHEELS_H_ */
