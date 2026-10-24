#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "mpu6050.h"
#include "hmc5883l.h"
#include "i2c3.h"

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
