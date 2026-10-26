#include "../I2C_RecoverBus/i2c1.h"

extern I2C_HandleTypeDef hi2c1;

void I2C1_RecoverBus(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Disable the I2C1 peripheral to release hardware control
    __HAL_I2C_DISABLE(&hi2c1);

    // 2. Configure SCL and SDA pins as general-purpose open-drain outputs
    // Note: Assuming standard PB6 (SCL) and PB7 (SDA).
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Clock out the stuck slave (send 9 clock pulses)
    // If the IMU is holding SDA low, these clocks will complete its pending transmission
    for (uint8_t i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // SCL High
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // SCL Low
        HAL_Delay(1);
    }

    // 4. Generate a manual I2C STOP condition (SCL high, then SDA transitions low to high)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // SCL Low
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // SDA Low
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // SCL High
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   // SDA High
    HAL_Delay(1);

    // 5. Reset the I2C peripheral and reinitialize
    // HAL_I2C_DeInit clears the peripheral, and HAL_I2C_Init calls HAL_I2C_MspInit()
    // to automatically reconfigure PB6 and PB7 back to their Alternate Function modes.
    HAL_I2C_DeInit(&hi2c1);

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}
