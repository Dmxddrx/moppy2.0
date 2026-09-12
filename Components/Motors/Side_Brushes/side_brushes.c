#include "side_brushes.h"

void SideBrushes_Init(void)
{
    // 1. Enable Motor Driver 2 by pulling MD2_STBY high[cite: 8]
    HAL_GPIO_WritePin(MD2_STBY_GPIO_Port, MD2_STBY_Pin, GPIO_PIN_SET);

    // 2. Start PWM signals for Left (CH2) and Right (CH3) side brushes[cite: 10]
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/* ==================================================================== */
/* INDIVIDUAL BRUSH CONTROL                                             */
/* ==================================================================== */

void SideBrushes_SetLeftSpeed(uint16_t speed)
{
    // Clamp speed to maximum ARR value (999)
    if (speed > 999) speed = 999;

    if (speed > 0) {
        // Forward (Spin Inward): AIN1 High, AIN2 Low[cite: 8]
        HAL_GPIO_WritePin(M3_AIN1_GPIO_Port, M3_AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M3_AIN2_GPIO_Port, M3_AIN2_Pin, GPIO_PIN_RESET);
    }
    else {
        // Stop: AIN1 Low, AIN2 Low[cite: 8]
        HAL_GPIO_WritePin(M3_AIN1_GPIO_Port, M3_AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M3_AIN2_GPIO_Port, M3_AIN2_Pin, GPIO_PIN_RESET);
    }

    // Set the PWM duty cycle on TIM4 Channel 2 (M3_PWM)[cite: 9]
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, speed);
}

void SideBrushes_SetRightSpeed(uint16_t speed)
{
    // Clamp speed to maximum ARR value (999)
    if (speed > 999) speed = 999;

    if (speed > 0) {
        // Forward (Spin Inward): BIN1 High, BIN2 Low[cite: 8]
        // Note: If the right brush spins outward instead of inward,
        // swap the SET and RESET states here.
        HAL_GPIO_WritePin(M4_BIN1_GPIO_Port, M4_BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M4_BIN2_GPIO_Port, M4_BIN2_Pin, GPIO_PIN_RESET);
    }
    else {
        // Stop: BIN1 Low, BIN2 Low[cite: 8]
        HAL_GPIO_WritePin(M4_BIN1_GPIO_Port, M4_BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M4_BIN2_GPIO_Port, M4_BIN2_Pin, GPIO_PIN_RESET);
    }

    // Set the PWM duty cycle on TIM4 Channel 3 (M4_PWM)[cite: 9]
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);
}

/* ==================================================================== */
/* HIGH-LEVEL CONTROL                                                   */
/* ==================================================================== */

void SideBrushes_Start(uint16_t speed)
{
    SideBrushes_SetLeftSpeed(speed);
    SideBrushes_SetRightSpeed(speed);
}

void SideBrushes_Stop(void)
{
    SideBrushes_SetLeftSpeed(0);
    SideBrushes_SetRightSpeed(0);
}
