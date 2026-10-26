#include "mpu6050.h"

/* I2C1: PB6 (SCL1), PB7 (SDA1) — shared with HMC5883L          */
extern I2C_HandleTypeDef hi2c3;
extern IWDG_HandleTypeDef hiwdg;

/* ── Register map ───────────────────────────────────────────────*/
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_GYRO_XOUT_H   0x43
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

/* ── Scale ──────────────────────────────────────────────────────*/
#define ACCEL_SCALE_MS2   (9.81f / 16384.0f)  /* ±2 g range       */
#define GYRO_SCALE_DPS    (1.0f  / 131.0f)    /* ±250 °/s range   */

/* ── State ──────────────────────────────────────────────────────*/
static int16_t s_gyro_bias[3] = {0, 0, 0};
static int16_t s_accel_bias[2] = {0, 0};

/* ─────────────────────────────────────────────────────────────── */
static HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, reg, 1, &val, 1, 10);
}
static HAL_StatusTypeDef ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, reg, 1, buf, len, 20);
}

/* ─────────────────────────────────────────────────────────────── */
MPU_Status MPU6050_Check(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR,
                          REG_WHO_AM_I, 1, &id, 1, 100) != HAL_OK)
        return MPU_NO_I2C;
    /* MPU-6500 = 0x70, MPU-6050 = 0x68, MPU-9250 = 0x71         */
    if (id != 0x70 && id != 0x68 && id != 0x71)
        return MPU_WRONG_ID;
    return MPU_OK;
}

/* ─────────────────────────────────────────────────────────────── */
void MPU6050_Init(void)
{
	HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(100);

    WriteReg(REG_PWR_MGMT_1, 0x80);
    HAL_IWDG_Refresh(&hiwdg);

    HAL_Delay(100);
    WriteReg(REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);
    WriteReg(REG_SMPLRT_DIV,  0x07);
    WriteReg(REG_CONFIG,       0x03);
    WriteReg(REG_GYRO_CONFIG,  0x00);
    WriteReg(REG_ACCEL_CONFIG, 0x00);

    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(100);

    /* Calibrate gyro on power-up (keep robot stationary!)         */
    MPU6050_CalibrateGyro();
}

/* ─────────────────────────────────────────────────────────────── */
/*  MPU6050_CalibrateGyro                                           */
/*  Averages 200 gyro readings to find static bias.                 */
/*  Robot MUST be motionless during this call (~1 second).          */
/* ─────────────────────────────────────────────────────────────── */
void MPU6050_CalibrateGyro(void)
{
    int32_t sum_g[3] = {0, 0, 0};
    int32_t sum_a[2] = {0, 0}; /* X and Y */
    uint8_t buf[14];
    const int N = 200;

    for (int i = 0; i < N; i++) {
        /* Read all 14 bytes in one go! */
        if (ReadRegs(REG_ACCEL_XOUT_H, buf, 14) == HAL_OK) {
            sum_a[0] += (int16_t)((buf[0] << 8) | buf[1]);
            sum_a[1] += (int16_t)((buf[2] << 8) | buf[3]);
            /* We skip Z because Z is measuring 1G of gravity! */

            sum_g[0] += (int16_t)((buf[8] << 8) | buf[9]);
            sum_g[1] += (int16_t)((buf[10] << 8) | buf[11]);
            sum_g[2] += (int16_t)((buf[12] << 8) | buf[13]);
        }
        HAL_Delay(5);
        HAL_IWDG_Refresh(&hiwdg);
    }

    s_accel_bias[0] = (int16_t)(sum_a[0] / N);
    s_accel_bias[1] = (int16_t)(sum_a[1] / N);

    s_gyro_bias[0] = (int16_t)(sum_g[0] / N);
    s_gyro_bias[1] = (int16_t)(sum_g[1] / N);
    s_gyro_bias[2] = (int16_t)(sum_g[2] / N);
}

/* ─────────────────────────────────────────────────────────────── */
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_RawData *data)
{
    uint8_t buf[14];
    HAL_StatusTypeDef status;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    status = ReadRegs(REG_ACCEL_XOUT_H, buf, sizeof(buf));

    if (status != HAL_OK)
    {
        return status;
    }

    data->ax = (int16_t)((buf[0]  << 8) | buf[1]);
    data->ay = (int16_t)((buf[2]  << 8) | buf[3]);
    data->az = (int16_t)((buf[4]  << 8) | buf[5]);

    data->temp_raw =
        (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);

    data->gx = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gy = (int16_t)((buf[10] << 8) | buf[11]);
    data->gz = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}

/* ─────────────────────────────────────────────────────────────── */
HAL_StatusTypeDef MPU6050_ReadRaw_DMA(uint8_t *dma_buf)
{
    /* Tells the DMA hardware to fetch 14 bytes in the background */
    return HAL_I2C_Mem_Read_DMA(&hi2c3, MPU6050_ADDR, REG_ACCEL_XOUT_H, 1, dma_buf, 14);
}

void MPU6050_ParseDMA(uint8_t *dma_buf, MPU6050_RawData *data)
{
	 if ((dma_buf == NULL) || (data == NULL))
	{
		return;
	}

    /* Translates the raw bytes after the DMA transfer is complete */
    data->ax = (int16_t)((dma_buf[0]  << 8) | dma_buf[1]);
    data->ay = (int16_t)((dma_buf[2]  << 8) | dma_buf[3]);
    data->az = (int16_t)((dma_buf[4]  << 8) | dma_buf[5]);

    data->temp_raw =
        (int16_t)(((uint16_t)dma_buf[6] << 8) | dma_buf[7]);

    data->gx = (int16_t)((dma_buf[8]  << 8) | dma_buf[9]);
    data->gy = (int16_t)((dma_buf[10] << 8) | dma_buf[11]);
    data->gz = (int16_t)((dma_buf[12] << 8) | dma_buf[13]);
}

/* ─────────────────────────────────────────────────────────────── */
MPU6050_PhysData MPU6050_GetPhysical(const MPU6050_RawData *raw)
{
    MPU6050_PhysData p;

    /* Subtract Accel biases to force a flat baseline */
    p.ax = (raw->ax - s_accel_bias[0]) * ACCEL_SCALE_MS2;
    p.ay = (raw->ay - s_accel_bias[1]) * ACCEL_SCALE_MS2;
    p.az = raw->az * ACCEL_SCALE_MS2; /* Z untouched, reads 9.81 */

    p.temperature_c =
        ((float)raw->temp_raw / 340.0f) + 36.53f;

    /* Subtract Gyro biases so the 100Hz DMA loop stays accurate */
    p.gx = (raw->gx - s_gyro_bias[0]) * GYRO_SCALE_DPS;
    p.gy = (raw->gy - s_gyro_bias[1]) * GYRO_SCALE_DPS;
    p.gz = (raw->gz - s_gyro_bias[2]) * GYRO_SCALE_DPS;

    return p;
}

