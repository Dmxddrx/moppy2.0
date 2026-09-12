#include "roller_brush.h"

void RollerBrush_Init(void)
{
    // 1. Enable Motor Driver 3 by pulling MD3_STBY high[cite: 8]
    HAL_GPIO_WritePin(MD3_STBY_GPIO_Port, MD3_STBY_Pin, GPIO_PIN_SET);

    // 2. Start PWM signal for the Roller Brush (CH3)[cite: 10]
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void RollerBrush_Start(uint16_t speed)
{
    // Clamp speed to maximum ARR value (999)
    if (speed > 999) speed = 999;

    if (speed > 0) {
        // Forward (Spin inward to sweep debris): AIN1 High, AIN2 Low[cite: 8]
        HAL_GPIO_WritePin(M5_AIN1_GPIO_Port, M5_AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M5_AIN2_GPIO_Port, M5_AIN2_Pin, GPIO_PIN_RESET);
    }
    else {
        // Stop logic if someone passes 0 to the start function
        RollerBrush_Stop();
        return;
    }

    // Set the PWM duty cycle on TIM3 Channel 3 (M5_PWM)[cite: 8, 10]
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
}

void RollerBrush_Stop(void)
{
    // Stop: AIN1 Low, AIN2 Low (Brake)[cite: 8]
    HAL_GPIO_WritePin(M5_AIN1_GPIO_Port, M5_AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M5_AIN2_GPIO_Port, M5_AIN2_Pin, GPIO_PIN_RESET);

    // Drop PWM duty cycle to 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
}
