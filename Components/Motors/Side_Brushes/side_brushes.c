#include "side_brushes.h"

static uint16_t target_speed_L = 0;
static uint16_t current_speed_L = 0;
static uint16_t target_speed_R = 0;
static uint16_t current_speed_R = 0;
static uint32_t last_ramp_time = 0;

void SideBrushes_Init(void)
{
    // 1. Enable Motor Driver 2 by pulling MD2_STBY high
    HAL_GPIO_WritePin(MD2_STBY_GPIO_Port, MD2_STBY_Pin, GPIO_PIN_SET);

    // 2. Start PWM signals for Left (CH3) and Right (CH4) on TIM3[cite: 72]
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void SideBrushes_SetLeftSpeed(uint16_t speed)
{
    // Clamp speed to maximum ARR value (999)[cite: 72]
    if (speed > 999) speed = 999;

    target_speed_L = speed;

    if (target_speed_L > 0) {
        // Forward (Spin Inward)[cite: 72]
        HAL_GPIO_WritePin(M3_AIN1_GPIO_Port, M3_AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M3_AIN2_GPIO_Port, M3_AIN2_Pin, GPIO_PIN_RESET);
    }
}

void SideBrushes_SetRightSpeed(uint16_t speed)
{
    // Clamp speed to maximum ARR value (999)[cite: 72]
    if (speed > 999) speed = 999;

    target_speed_R = speed;

    if (target_speed_R > 0) {
        // Forward (Spin Inward)[cite: 72]
        HAL_GPIO_WritePin(M4_BIN1_GPIO_Port, M4_BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M4_BIN2_GPIO_Port, M4_BIN2_Pin, GPIO_PIN_RESET);
    }
}

void SideBrushes_Start(uint16_t speed)
{
    SideBrushes_SetLeftSpeed(speed);
    SideBrushes_SetRightSpeed(speed);
}

void SideBrushes_Stop(void)
{
    target_speed_L = 0;
    current_speed_L = 0;
    target_speed_R = 0;
    current_speed_R = 0;

    HAL_GPIO_WritePin(M3_AIN1_GPIO_Port, M3_AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_AIN2_GPIO_Port, M3_AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_BIN1_GPIO_Port, M4_BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_BIN2_GPIO_Port, M4_BIN2_Pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

void SideBrushes_Update(void)
{
    uint32_t current_time = HAL_GetTick();

    if ((current_time - last_ramp_time) >= 10) {
        last_ramp_time = current_time;

        // Left Brush Ramp
        if (current_speed_L < target_speed_L) {
            current_speed_L += 10;
            if (current_speed_L > target_speed_L) current_speed_L = target_speed_L;
        } else if (current_speed_L > target_speed_L) {
            current_speed_L -= 20;
            if (current_speed_L < target_speed_L) current_speed_L = target_speed_L;
        }

        // Right Brush Ramp
        if (current_speed_R < target_speed_R) {
            current_speed_R += 5;
            if (current_speed_R > target_speed_R) current_speed_R = target_speed_R;
        } else if (current_speed_R > target_speed_R) {
            current_speed_R -= 20;
            if (current_speed_R < target_speed_R) current_speed_R = target_speed_R;
        }

        // Apply smoothed PWM duty cycle[cite: 72]
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, current_speed_L);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, current_speed_R);
    }
}

uint16_t SideBrushes_GetLeftSpeed(void) { return current_speed_L; }
uint16_t SideBrushes_GetRightSpeed(void) { return current_speed_R; }
