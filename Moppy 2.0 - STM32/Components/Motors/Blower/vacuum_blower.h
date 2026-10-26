#ifndef VACUUM_BLOWER_H_
#define VACUUM_BLOWER_H_

#include "main.h"
#include <stdint.h>

/* Expose the TIM4 hardware timer configured in main.c */
extern TIM_HandleTypeDef htim4;


/*
 * Number of pulses the motor outputs per revolution.
 * Standard 3-phase brushless blowers typically output 2, 3, or 6 pulses per rev.
 * Adjust this if your RPM readings seem exactly double or half of reality.
 */
#define BLOWER_PULSES_PER_REV 2
/* Add this below your #defines in vacuum_blower.h */
extern volatile uint32_t blower_pulse_count;

/* Initialization function */
void VacuumBlower_Init(void);

/* Speed control (Input range: 0 to 999) */
void VacuumBlower_SetSpeed(uint16_t speed);
void VacuumBlower_Stop(void);

/* RPM Feedback */
uint32_t VacuumBlower_GetRPM(void);

#endif /* VACUUM_BLOWER_H_ */
