#include "main_wheels.h"

void MainWheels_Init(void)
{
    // 1. Enable Motor Driver 1 by pulling MD1_STBY high[cite: 11]
    HAL_GPIO_WritePin(MD1_STBY_GPIO_Port, MD1_STBY_Pin, GPIO_PIN_SET);

    // 2. Start PWM signals for Left (CH1) and Right (CH2) wheels[cite: 11]
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // 3. Start Hardware Encoders on all channels[cite: 11]
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

/* ==================================================================== */
/* INDIVIDUAL WHEEL CONTROL                                             */
/* ==================================================================== */

void MainWheels_SetLeftSpeed(int16_t speed)
{
    if (speed > 999) speed = 999;
    if (speed < -999) speed = -999;

    if (speed > 0) {
        HAL_GPIO_WritePin(M1_AIN1_GPIO_Port, M1_AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1_AIN2_GPIO_Port, M1_AIN2_Pin, GPIO_PIN_RESET);
    }
    else if (speed < 0) {
        HAL_GPIO_WritePin(M1_AIN1_GPIO_Port, M1_AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M1_AIN2_GPIO_Port, M1_AIN2_Pin, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(M1_AIN1_GPIO_Port, M1_AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M1_AIN2_GPIO_Port, M1_AIN2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, abs(speed));
}

void MainWheels_SetRightSpeed(int16_t speed)
{
    if (speed > 999) speed = 999;
    if (speed < -999) speed = -999;

    if (speed > 0) {
        HAL_GPIO_WritePin(M2_BIN1_GPIO_Port, M2_BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M2_BIN2_GPIO_Port, M2_BIN2_Pin, GPIO_PIN_RESET);
    }
    else if (speed < 0) {
        HAL_GPIO_WritePin(M2_BIN1_GPIO_Port, M2_BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M2_BIN2_GPIO_Port, M2_BIN2_Pin, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(M2_BIN1_GPIO_Port, M2_BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M2_BIN2_GPIO_Port, M2_BIN2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(speed));
}

/* ==================================================================== */
/* HIGH-LEVEL MOTION CONTROL                                            */
/* ==================================================================== */

void MainWheels_Forward(uint16_t speed)
{
    MainWheels_SetLeftSpeed((int16_t)speed);
    MainWheels_SetRightSpeed((int16_t)speed);
}

void MainWheels_Backward(uint16_t speed)
{
    // Negate the speed to reverse both wheels automatically
    MainWheels_SetLeftSpeed(-(int16_t)speed);
    MainWheels_SetRightSpeed(-(int16_t)speed);
}

void MainWheels_Stop(void)
{
    MainWheels_SetLeftSpeed(0);
    MainWheels_SetRightSpeed(0);
}

/* ==================================================================== */
/* STATE READING FUNCTIONS                                              */
/* ==================================================================== */

uint16_t MainWheels_GetLeftSpeed(void)
{
    // Read the current PWM duty cycle directly from the timer register
    return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
}

uint16_t MainWheels_GetRightSpeed(void)
{
    return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
}

int8_t MainWheels_GetLeftDirection(void)
{
    // 1 = Forward, -1 = Backward, 0 = Stopped
    if (HAL_GPIO_ReadPin(M1_AIN1_GPIO_Port, M1_AIN1_Pin) == GPIO_PIN_SET) return 1;
    if (HAL_GPIO_ReadPin(M1_AIN2_GPIO_Port, M1_AIN2_Pin) == GPIO_PIN_SET) return -1;
    return 0;
}

int8_t MainWheels_GetRightDirection(void)
{
    if (HAL_GPIO_ReadPin(M2_BIN1_GPIO_Port, M2_BIN1_Pin) == GPIO_PIN_SET) return 1;
    if (HAL_GPIO_ReadPin(M2_BIN2_GPIO_Port, M2_BIN2_Pin) == GPIO_PIN_SET) return -1;
    return 0;
}

/* ==================================================================== */
/* ENCODER FUNCTIONS                                                    */
/* ==================================================================== */

int16_t MainWheels_GetLeftEncoder(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

int16_t MainWheels_GetRightEncoder(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
}

void MainWheels_ResetEncoders(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
}
