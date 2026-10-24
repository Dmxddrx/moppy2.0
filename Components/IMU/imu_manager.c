#include "imu_manager.h"

#define IMU_TRANSFER_TIMEOUT_MS  50U
#define IMU_RETRY_DELAY_MS       250U

static uint32_t active_transfer_started_ms = 0U;
static uint32_t retry_after_ms = 0U;

extern I2C_HandleTypeDef hi2c3;

#define IMU_SAMPLE_INTERVAL_MS  10U

static IMU_Data imu_data;

static uint8_t mpu_dma_buf[14];
static uint8_t hmc_dma_buf[6];

volatile uint32_t imu_dbg_recovery_count = 0U;
volatile uint32_t imu_dbg_timeout_count = 0U;
volatile uint32_t imu_dbg_error_callback_count = 0U;
volatile uint32_t imu_dbg_dma_start_fail_count = 0U;

volatile uint32_t imu_dbg_last_dma_start_status = 0U;
volatile uint32_t imu_dbg_last_i2c_error = 0U;
volatile uint32_t imu_dbg_last_i2c_state = 0U;
volatile uint32_t imu_dbg_last_recovery_ok = 0U;

typedef enum
{
    IMU_BUS_IDLE = 0,
    IMU_BUS_MPU,
    IMU_BUS_HMC
} IMU_BusState;

static volatile IMU_BusState bus_state = IMU_BUS_IDLE;
static volatile uint8_t dma_complete_pending = 0U;
static volatile uint8_t dma_error_pending = 0U;

static uint32_t last_transfer_start = 0U;
static uint8_t next_sensor_is_mpu = 1U;


const IMU_Data *IMU_GetData(void)
{
    return &imu_data;
}


void IMU_Init(void)
{
    /* Check both sensors on I2C3. */
    imu_data.mpu_status = MPU6050_Check(&hi2c3);
    imu_data.hmc_status = HMC5883L_Check(&hi2c3);

    /*
     * Initialize only detected sensors.
     * MPU6050_Init() calibrates the gyro, so keep the robot stationary.
     */
    if (imu_data.mpu_status == MPU_OK)
    {
        MPU6050_Init();
    }

    if (imu_data.hmc_status == HMC_OK)
    {
        HMC5883L_Init();
    }

    imu_data.mpu_data_valid = false;
    imu_data.hmc_data_valid = false;

    bus_state = IMU_BUS_IDLE;
    dma_complete_pending = 0U;

    dma_error_pending = 0U;

    next_sensor_is_mpu = 1U;
    last_transfer_start = HAL_GetTick();
}


void IMU_OnMemRxCplt(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c == &hi2c3) && (bus_state != IMU_BUS_IDLE))
    {
        dma_complete_pending = 1U;
    }
}


void IMU_OnError(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c == &hi2c3) && (bus_state != IMU_BUS_IDLE))
    {
        imu_dbg_error_callback_count++;

        imu_dbg_last_i2c_error = hi2c->ErrorCode;
        imu_dbg_last_i2c_state = (uint32_t)hi2c->State;

        dma_error_pending = 1U;
    }
}


void IMU_Process(void)
{
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef status;
    bool use_mpu;

    /*
     * Detect stalled DMA transfers.
     * The start timestamp must be set for EVERY transfer.
     */
    if ((bus_state != IMU_BUS_IDLE) &&
        (dma_complete_pending == 0U) &&
        (dma_error_pending == 0U) &&
        ((uint32_t)(now - active_transfer_started_ms) >=
         IMU_TRANSFER_TIMEOUT_MS))
    {
    	imu_dbg_timeout_count++;
        dma_error_pending = 1U;
    }

    /*
     * Recover outside interrupt context.
     */
    if (dma_error_pending != 0U)
    {
        dma_error_pending = 0U;
        dma_complete_pending = 0U;
        bus_state = IMU_BUS_IDLE;

        imu_data.mpu_data_valid = false;
        imu_data.hmc_data_valid = false;

        imu_dbg_recovery_count++;

        imu_dbg_last_recovery_ok =
            I2C3_RecoverBus() ? 1U : 0U;

        now = HAL_GetTick();
        last_transfer_start = now;
        retry_after_ms = now + IMU_RETRY_DELAY_MS;

        return;
    }

    /*
     * Process completed transfers in the main loop.
     * Interrupt callbacks only set flags.
     */
    if (dma_complete_pending != 0U)
    {
        dma_complete_pending = 0U;

        if (bus_state == IMU_BUS_MPU)
        {
            MPU6050_ParseDMA(mpu_dma_buf, &imu_data.mpu_raw);

            imu_data.mpu_physical =
                MPU6050_GetPhysical(&imu_data.mpu_raw);

            imu_data.mpu_timestamp_ms = HAL_GetTick();
            imu_data.mpu_data_valid = true;
        }
        else if (bus_state == IMU_BUS_HMC)
        {
            HMC5883L_ParseDMA(hmc_dma_buf, &imu_data.hmc_raw);

            imu_data.hmc_timestamp_ms = HAL_GetTick();
            imu_data.hmc_data_valid = true;
        }

        bus_state = IMU_BUS_IDLE;
    }

    /*
     * Never start another transaction while DMA is active.
     */
    if (bus_state != IMU_BUS_IDLE)
    {
        return;
    }

    now = HAL_GetTick();

    /* Wait before retrying after an error or recovery. */
    if ((retry_after_ms != 0U) &&
        ((int32_t)(now - retry_after_ms) < 0))
    {
        return;
    }

    if ((uint32_t)(now - last_transfer_start) <
        IMU_SAMPLE_INTERVAL_MS)
    {
        return;
    }

    bool mpu_available = (imu_data.mpu_status == MPU_OK);
    bool hmc_available = (imu_data.hmc_status == HMC_OK);

    if (!mpu_available && !hmc_available)
    {
        return;
    }

    /*
     * If both sensors work, alternate between them.
     * If only one works, keep sampling that sensor.
     */
    use_mpu = mpu_available &&
              (next_sensor_is_mpu || !hmc_available);

    last_transfer_start = now;
    active_transfer_started_ms = now;

    if (use_mpu)
    {
        bus_state = IMU_BUS_MPU;
        active_transfer_started_ms = HAL_GetTick();

        status = MPU6050_ReadRaw_DMA(mpu_dma_buf);

        if (status == HAL_OK)
        {
            next_sensor_is_mpu = 0U;
        }
        else
        {
            imu_dbg_dma_start_fail_count++;

            imu_dbg_last_dma_start_status = (uint32_t)status;
            imu_dbg_last_i2c_error = hi2c3.ErrorCode;
            imu_dbg_last_i2c_state = (uint32_t)hi2c3.State;

            bus_state = IMU_BUS_IDLE;
            dma_error_pending = 1U;
        }
    }
    else
    {
        bus_state = IMU_BUS_HMC;

        status = HMC5883L_ReadRaw_DMA(hmc_dma_buf);

        if (status == HAL_OK)
        {
            next_sensor_is_mpu = 1U;
        }
        else
        {
            imu_dbg_dma_start_fail_count++;

            imu_dbg_last_dma_start_status = (uint32_t)status;
            imu_dbg_last_i2c_error = hi2c3.ErrorCode;
            imu_dbg_last_i2c_state = (uint32_t)hi2c3.State;

            bus_state = IMU_BUS_IDLE;
            dma_error_pending = 1U;
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IMU_OnMemRxCplt(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    IMU_OnError(hi2c);
}
