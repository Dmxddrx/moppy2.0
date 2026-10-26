#include "main_wheels.h"

void MainWheels_Init(void)
{
    // 1. Enable Motor Driver 1 by pulling MD1_STBY high
    HAL_GPIO_WritePin(MD1_STBY_GPIO_Port, MD1_STBY_Pin, GPIO_PIN_SET);

    // 2. Start PWM signals for Right/M2 (CH1) and Left/M1 (CH2)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // 3. Start Hardware Encoders in Interrupt Mode for 32/64-bit odometry tracking
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

    // Left motor (M1) is mapped to TIM3 Channel 2
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(speed));
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

    // Right motor (M2) is mapped to TIM3 Channel 1
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, abs(speed));
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
    return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
}

uint16_t MainWheels_GetRightSpeed(void)
{
    return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
}

int8_t MainWheels_GetLeftDirection(void)
{
    static int16_t last_enc = 0;
    int16_t current_enc = MainWheels_GetLeftEncoder();

    // If the encoder hasn't changed since the last check, it's stopped
    if (current_enc == last_enc) {
        return 0; // STP
    }

    last_enc = current_enc;

    // If it is moving, check the hardware timer's internal direction flag
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) {
        return -1; // REV
    } else {
        return 1;  // FWD
    }
}

int8_t MainWheels_GetRightDirection(void)
{
    static int16_t last_enc = 0;
    int16_t current_enc = MainWheels_GetRightEncoder();

    if (current_enc == last_enc) {
        return 0; // STP
    }

    last_enc = current_enc;

    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8)) {
        return -1; // REV
    } else {
        return 1;  // FWD
    }
}

/* ==================================================================== */
/* ENCODER FUNCTIONS                                                    */
/* ==================================================================== */

int16_t MainWheels_GetLeftEncoder(void)
{
    return -(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
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

