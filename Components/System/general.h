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

/* Include your sensor headers */
#include "hmc5883l.h"
#include "mpu6050.h"

/* Hardware Recoverbus for I2C1 */
#include "i2c1.h"

extern IWDG_HandleTypeDef hiwdg;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1;

/* Expose Sensor Data to the OLED */
extern HMC5883L_RawData current_mag;
extern MPU6050_PhysData current_mpu_phys;
extern float current_heading;

/* System logic functions */
void General_Init(void);
void System_RunAllForwardMax(void);
void General_Run(void);

#endif /* GENERAL_H_ */
