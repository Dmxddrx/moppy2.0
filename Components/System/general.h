#ifndef GENERAL_H_
#define GENERAL_H_

/* Core HAL and board definitions */
#include "main.h"
#include "oled.h"
#include "oledGUI.h"

/* Include all motor subsystem headers */
#include "main_wheels.h"
#include "side_brushes.h"
#include "roller_brush.h"
#include "vacuum_blower.h"

/* Include High-Level Controllers */
#include "chassis_control.h"

extern IWDG_HandleTypeDef hiwdg;
extern I2C_HandleTypeDef hi2c2;

/* System logic functions */
void General_Init(void);
void System_RunAllForwardMax(void);
void General_Run(void);

#endif /* GENERAL_H_ */
