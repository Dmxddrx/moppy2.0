#ifndef SIDE_BRUSHES_H_
#define SIDE_BRUSHES_H_

#include "main.h"
#include <stdint.h>

/* Expose the TIM3 hardware timer configured in main.c */
extern TIM_HandleTypeDef htim3;

/* Initialization function */
void SideBrushes_Init(void);

/* Individual brush control (Input range: 0 to 999) */
void SideBrushes_SetLeftSpeed(uint16_t speed);
void SideBrushes_SetRightSpeed(uint16_t speed);

/* High-level unified control */
void SideBrushes_Start(uint16_t speed);
void SideBrushes_Stop(void);

void SideBrushes_Update(void);

uint16_t SideBrushes_GetLeftSpeed(void);
uint16_t SideBrushes_GetRightSpeed(void);

#endif /* SIDE_BRUSHES_H_ */
