#ifndef GENERAL_H_
#define GENERAL_H_

/* Core HAL and board definitions */
#include "main.h"

/* Display and GUI */
#include "oled.h"
#include "oledGUI.h"

/* Inputs */
#include "btns.h"

/* Motor and Logic Subsystems */
#include "main_wheels.h"
#include "chassis_control.h"
#include "side_brushes.h"
#include "roller_brush.h"


/* Expose global handles from main.c */
extern IWDG_HandleTypeDef hiwdg;
extern I2C_HandleTypeDef hi2c1; // Updated to I2C1 based on your new OLED configuration

/* System logic functions */
void General_Init(void);
void General_Run(void);

#endif /* GENERAL_H_ */
