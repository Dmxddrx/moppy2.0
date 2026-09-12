#include "general.h"

void System_Init(void)
{
    // Initialize all motor drivers and PWM channels
    MainWheels_Init();
    SideBrushes_Init();
    RollerBrush_Init();
    VacuumBlower_Init();
}

void System_RunAllForwardMax(void)
{
    // 999 is the 100% duty cycle based on the timer ARR settings[cite: 6]
    MainWheels_Forward(999);
    SideBrushes_Start(999);
    RollerBrush_Start(999);
    VacuumBlower_SetSpeed(999);
}

/**
  * @brief  Global EXTI callback for all physical interrupts.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Route Vacuum Blower RPM pulses[cite: 5, 7]
    if (GPIO_Pin == MSD_RPM_Pin)
    {
        blower_pulse_count++;
    }

    // Future expansion: Add bumper switches or cliff sensors here
    // else if (GPIO_Pin == BUMPER_FRONT_Pin) { ... }
}


// ... (keep your existing System_Init, System_RunAllForwardMax, and EXTI Callback here) ...

void General_Run(void)
{
    // 1. Feed the watchdog timer so the STM32 does not reset[cite: 7]
    HAL_IWDG_Refresh(&hiwdg);

    // 2. Put your continuous robot logic here
    // For now, we will just force all motors forward
    System_RunAllForwardMax();

    // Example: You can also read your sensors continuously here
    // uint32_t current_rpm = VacuumBlower_GetRPM();
}
