#include "i2c3.h"

extern I2C_HandleTypeDef hi2c3;

/*
 * Short delays for GPIO-based I2C recovery.
 * STM32F407 uses a Cortex-M4 DWT cycle counter.
 */
static void I2C3_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void I2C3_DelayUs(uint32_t us)
{
    uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    uint32_t cycles = cycles_per_us * us;
    uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
        /* Intentionally wait for a few microseconds. */
    }
}

bool I2C3_RecoverBus(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    bool lines_released;
    HAL_StatusTypeDef init_status;

    /*
     * This must run from the main context, not an interrupt callback.
     * Stop the old transaction and release peripheral control of the pins.
     */
    /* Abort the linked I2C3 DMA transfers, if configured. */
    if (hi2c3.hdmarx != NULL)
    {
        (void)HAL_DMA_Abort(hi2c3.hdmarx);
    }

    if (hi2c3.hdmatx != NULL)
    {
        (void)HAL_DMA_Abort(hi2c3.hdmatx);
    }

    /* Reset the I2C peripheral and release its pin configuration. */
    (void)HAL_I2C_DeInit(&hi2c3);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    I2C3_DWT_Init();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    /* I2C3 SCL: PA8 */
    GPIO_InitStruct.Pin = IMU_SCL_Pin;
    HAL_GPIO_Init(IMU_SCL_GPIO_Port, &GPIO_InitStruct);

    /* I2C3 SDA: PC9 */
    GPIO_InitStruct.Pin = IMU_SDA_Pin;
    HAL_GPIO_Init(IMU_SDA_GPIO_Port, &GPIO_InitStruct);

    /* Open-drain SET releases the lines. */
    HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_SET);
    I2C3_DelayUs(5U);

    /*
     * If SDA is stuck low, clock the bus up to nine times.
     * Stop early if SCL itself cannot rise.
     */
    for (uint8_t i = 0U;
         i < 9U &&
         HAL_GPIO_ReadPin(IMU_SDA_GPIO_Port, IMU_SDA_Pin) == GPIO_PIN_RESET;
         i++)
    {
        HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin, GPIO_PIN_RESET);
        I2C3_DelayUs(5U);

        HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin, GPIO_PIN_SET);
        I2C3_DelayUs(5U);

        if (HAL_GPIO_ReadPin(IMU_SCL_GPIO_Port, IMU_SCL_Pin) == GPIO_PIN_RESET)
        {
            /* A device or electrical fault is holding SCL low. */
            break;
        }
    }

    /* Attempt an I2C STOP: SDA rises while SCL is released high. */
    HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_RESET);
    I2C3_DelayUs(5U);

    HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin, GPIO_PIN_SET);
    I2C3_DelayUs(5U);

    HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_SET);
    I2C3_DelayUs(5U);

    lines_released =
        (HAL_GPIO_ReadPin(IMU_SCL_GPIO_Port, IMU_SCL_Pin) == GPIO_PIN_SET) &&
        (HAL_GPIO_ReadPin(IMU_SDA_GPIO_Port, IMU_SDA_Pin) == GPIO_PIN_SET);

    /*
     * Restore alternate-function configuration, DMA links and I2C settings
     * through the existing HAL MSP initialization.
     */
    init_status = HAL_I2C_Init(&hi2c3);

    return lines_released && (init_status == HAL_OK);
}
