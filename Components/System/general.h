#ifndef GENERAL_H_
#define GENERAL_H_

/* Core HAL and board definitions */
#include "main.h"

/* Include all motor subsystem headers */
#include "main_wheels.h"
#include "side_brushes.h"
#include "roller_brush.h"
#include "vacuum_blower.h"

extern IWDG_HandleTypeDef hiwdg;

/* System logic functions */
void System_Init(void);
void System_RunAllForwardMax(void);
void General_Run(void);

#endif /* GENERAL_H_ */
