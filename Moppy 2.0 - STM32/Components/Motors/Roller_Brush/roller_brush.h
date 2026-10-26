#ifndef ROLLER_BRUSH_H_
#define ROLLER_BRUSH_H_

#include "main.h"
#include <stdint.h>

/* Expose the TIM4 hardware timer configured in main.c */
extern TIM_HandleTypeDef htim4;

/* Initialization function */
void RollerBrush_Init(void);

/* Speed control (Input range: 0 to 999) */
void RollerBrush_Start(uint16_t speed);
void RollerBrush_Stop(void);

void RollerBrush_Update(void);

uint16_t RollerBrush_GetSpeed(void);

#endif /* ROLLER_BRUSH_H_ */
