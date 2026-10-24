#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "mpu6050.h"
#include "hmc5883l.h"
#include "i2c3.h"
#include "main.h"

#if IMU_DIAGNOSTICS

extern volatile uint32_t imu_dbg_recovery_count;
extern volatile uint32_t imu_dbg_timeout_count;
extern volatile uint32_t imu_dbg_error_callback_count;
extern volatile uint32_t imu_dbg_dma_start_fail_count;

extern volatile uint32_t imu_dbg_last_dma_start_status;
extern volatile uint32_t imu_dbg_last_i2c_error;
extern volatile uint32_t imu_dbg_last_i2c_state;
extern volatile uint32_t imu_dbg_last_recovery_ok;

extern volatile uint32_t imu_dbg_hmc_addr_1e_seen;
extern volatile uint32_t imu_dbg_qmc_addr_0d_seen;

extern volatile uint32_t imu_dbg_mpu_samples;
extern volatile uint32_t imu_dbg_hmc_samples;
extern volatile uint32_t imu_dbg_mpu_max_gap_ms;
extern volatile uint32_t imu_dbg_hmc_max_gap_ms;

#endif

typedef struct
{
    MPU_Status mpu_status;
    HMC_Status hmc_status;

    MPU6050_RawData mpu_raw;
    MPU6050_PhysData mpu_physical;
    HMC5883L_RawData hmc_raw;

    bool mpu_data_valid;
    bool hmc_data_valid;

    uint32_t mpu_timestamp_ms;
    uint32_t hmc_timestamp_ms;
} IMU_Data;

void IMU_Init(void);
void IMU_Process(void);

const IMU_Data *IMU_GetData(void);

/* Called by the existing HAL callbacks. */
void IMU_OnMemRxCplt(I2C_HandleTypeDef *hi2c);
void IMU_OnError(I2C_HandleTypeDef *hi2c);

#endif
