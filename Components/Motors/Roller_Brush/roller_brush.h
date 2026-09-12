#ifndef ROLLER_BRUSH_H_
#define ROLLER_BRUSH_H_

#include "main.h"
#include <stdint.h>

/* Expose the TIM3 hardware timer configured in main.c */
extern TIM_HandleTypeDef htim3;

/* Initialization function */
void RollerBrush_Init(void);

/* Speed control (Input range: 0 to 999) */
void RollerBrush_Start(uint16_t speed);
void RollerBrush_Stop(void);

#endif /* ROLLER_BRUSH_H_ */
