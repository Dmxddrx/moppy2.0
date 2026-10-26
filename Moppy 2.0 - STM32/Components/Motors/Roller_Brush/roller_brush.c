#include "roller_brush.h"

static uint16_t target_speed = 0;
static uint16_t current_speed = 0;
static uint32_t last_ramp_time = 0;

void RollerBrush_Init(void)
{
    // 1. Enable Motor Driver 3 by pulling MD3_STBY high
    HAL_GPIO_WritePin(MD3_STBY_GPIO_Port, MD3_STBY_Pin, GPIO_PIN_SET);

    // 2. Start PWM signal for the Roller Brush (TIM4, CH4)[cite: 70]
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void RollerBrush_Start(uint16_t speed)
{
    // Clamp speed to maximum ARR value (999)[cite: 70]
    if (speed > 999) speed = 999;

    target_speed = speed;

    if (target_speed > 0) {
        // Forward (Spin inward to sweep debris): AIN1 High, AIN2 Low[cite: 70]
        HAL_GPIO_WritePin(M5_AIN1_GPIO_Port, M5_AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M5_AIN2_GPIO_Port, M5_AIN2_Pin, GPIO_PIN_RESET);
    }
}

void RollerBrush_Stop(void)
{
    target_speed = 0;
    current_speed = 0; // Cut speed instantly for safety

    // Stop: AIN1 Low, AIN2 Low (Brake)[cite: 70]
    HAL_GPIO_WritePin(M5_AIN1_GPIO_Port, M5_AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M5_AIN2_GPIO_Port, M5_AIN2_Pin, GPIO_PIN_RESET);

    // Drop PWM duty cycle to 0[cite: 70]
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

void RollerBrush_Update(void)
{
    uint32_t current_time = HAL_GetTick();

    // Execute ramp step every 10ms
    if ((current_time - last_ramp_time) >= 10) {
        last_ramp_time = current_time;

        if (current_speed < target_speed) {
            current_speed += 5; // Ramp up by 10 per 10ms (1 sec to reach 1000)
            if (current_speed > target_speed) current_speed = target_speed;
        }
        else if (current_speed > target_speed) {
            current_speed -= 20; // Ramp down twice as fast
            if (current_speed < target_speed) current_speed = target_speed;
        }

        // Apply the smoothed PWM duty cycle on TIM4 Channel 4[cite: 70]
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, current_speed);
    }
}

uint16_t RollerBrush_GetSpeed(void) {
    return current_speed;
}
