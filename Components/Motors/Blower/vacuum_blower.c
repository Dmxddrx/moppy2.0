#include "vacuum_blower.h"

// Volatile counter to track interrupt pulses outside the main loop
volatile uint32_t blower_pulse_count = 0;

void VacuumBlower_Init(void)
{
    // Start the PWM signal on TIM4 Channel 1 (MSD_PWM)[cite: 9, 10]
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void VacuumBlower_SetSpeed(uint16_t speed)
{
    // Clamp speed to maximum ARR value (999)
    if (speed > 999) speed = 999;

    // The vacuum blower has its own internal driver, so we only need to update the duty cycle[cite: 8]
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
}

void VacuumBlower_Stop(void)
{
    // Drop PWM duty cycle to 0 to cut the speed[cite: 8]
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
}

/* ==================================================================== */
/* RPM CALCULATION & INTERRUPT LOGIC                                    */
/* ==================================================================== */

uint32_t VacuumBlower_GetRPM(void)
{
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - last_time;

    // Prevent division by zero if called multiple times in the same millisecond
    if (elapsed_time == 0) return 0;

    // 1. Disable interrupts briefly to safely copy and reset the pulse counter
    //    without an interrupt firing halfway through the read.
    __disable_irq();
    uint32_t current_pulses = blower_pulse_count;
    blower_pulse_count = 0;
    __enable_irq();

    // 2. Update the timestamp for the next call
    last_time = current_time;

    // 3. Calculate RPM: (Pulses / Pulses_Per_Rev) * (60000ms / Elapsed_ms)
    uint32_t rpm = (current_pulses * 60000) / (BLOWER_PULSES_PER_REV * elapsed_time);

    return rpm;
}

