#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdint.h>

#define MPU6050_ADDR   (0x68 << 1)   /* AD0 = LOW   */

typedef enum {
    MPU_OK       = 0,
    MPU_NO_I2C   = 1,
    MPU_WRONG_ID = 2,
} MPU_Status;

/* Raw 16-bit values from sensor registers */
typedef struct {
    int16_t ax, ay, az;
    int16_t temp_raw;
    int16_t gx, gy, gz;
} MPU6050_RawData;

/* Physical units
   Scale factors:  ±2 g  → 16384 LSB/g  → × (9.81/16384) = m/s²
                   ±250°/s → 131 LSB/(°/s) → × (1/131)   = °/s  */
typedef struct {
    float ax, ay, az;     /* m/s²   */
    float gx, gy, gz;     /* deg/s  */
    float temperature_c;  /* degrees Celsius */
} MPU6050_PhysData;

MPU_Status        MPU6050_Check(I2C_HandleTypeDef *hi2c);
void              MPU6050_Init(void);
void              MPU6050_CalibrateGyro(void);   /* call while stationary */
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_RawData *data);
MPU6050_PhysData  MPU6050_GetPhysical(const MPU6050_RawData *raw);

HAL_StatusTypeDef MPU6050_ReadRaw_DMA(uint8_t *dma_buf);
void              MPU6050_ParseDMA(uint8_t *dma_buf, MPU6050_RawData *data);
#endif /* MPU6050_H */
