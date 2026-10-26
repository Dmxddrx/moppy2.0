#include "telemetry.h"

#include "imu_manager.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <string.h>

/* These handles are defined in main.c and configured by CubeMX. */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1; /* Encoder 1; mapped to left wheel by default. */
extern TIM_HandleTypeDef htim8; /* Encoder 2; mapped to right wheel by default. */

#define TELEMETRY_SYNC_0       0xA5U
#define TELEMETRY_SYNC_1       0x5AU
#define TELEMETRY_VERSION      0x01U
#define TELEMETRY_PAYLOAD_LEN  37U
#define TELEMETRY_FRAME_LEN    (2U + 1U + 1U + TELEMETRY_PAYLOAD_LEN + 2U)

#define TELEMETRY_FLAG_MPU_VALID  (1U << 0)
#define TELEMETRY_FLAG_HMC_VALID  (1U << 1)

static uint8_t tx_frame[TELEMETRY_FRAME_LEN];
static volatile uint8_t tx_busy = 0U;
static uint8_t initialized = 0U;
static uint32_t sequence_number = 0U;
static uint16_t previous_enc1 = 0U;
static uint16_t previous_enc2 = 0U;
static volatile uint16_t pending_enc1 = 0U;
static volatile uint16_t pending_enc2 = 0U;

volatile uint32_t telemetry_dbg_tx_started = 0U;
volatile uint32_t telemetry_dbg_tx_completed = 0U;
volatile uint32_t telemetry_dbg_busy_skips = 0U;
volatile uint32_t telemetry_dbg_dma_start_failures = 0U;
volatile uint32_t telemetry_dbg_uart_errors = 0U;
volatile uint32_t telemetry_dbg_last_hal_status = 0U;
volatile uint32_t telemetry_dbg_last_uart_error = 0U;

static void put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void put_i16_le(uint8_t *dst, int16_t value)
{
    put_u16_le(dst, (uint16_t)value);
}

static void put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFUL);
    dst[1] = (uint8_t)((value >> 8) & 0xFFUL);
    dst[2] = (uint8_t)((value >> 16) & 0xFFUL);
    dst[3] = (uint8_t)((value >> 24) & 0xFFUL);
}

/* CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, xorout=0. */
static uint16_t telemetry_crc16_ccitt(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

/* Correctly represent 16-bit encoder wrap-around as a signed delta. */
static int16_t encoder_delta_u16(uint16_t current, uint16_t previous)
{
    int32_t delta = (int32_t)current - (int32_t)previous;

    if (delta > 32767L)
    {
        delta -= 65536L;
    }
    else if (delta < -32768L)
    {
        delta += 65536L;
    }

    return (int16_t)delta;
}

void Telemetry_Init(void)
{
    /* Call after General_Init(), once encoder timers have been started. */
    previous_enc1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    previous_enc2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
    sequence_number = 0U;
    tx_busy = 0U;
    initialized = 1U;
}

void Telemetry_TaskStep(void)
{
    const IMU_Data *live_imu;
    IMU_Data imu_snapshot;
    uint16_t enc1_now;
    uint16_t enc2_now;
    int16_t enc1_delta;
    int16_t enc2_delta;
    uint8_t flags = 0U;
    uint8_t *payload;
    uint16_t crc;
    HAL_StatusTypeDef status;
    uint32_t payload_offset = 0U;

    if (initialized == 0U)
    {
        Telemetry_Init();
    }

    if (tx_busy != 0U)
    {
        telemetry_dbg_busy_skips++;
        return;
    }

    live_imu = IMU_GetData();
    if (live_imu == NULL)
    {
        return;
    }

    /* IMU_Process() updates this structure from a task, not from the DMA ISR.
       Keep the copy atomic with respect to other FreeRTOS tasks. */
    taskENTER_CRITICAL();
    memcpy(&imu_snapshot, live_imu, sizeof(imu_snapshot));
    taskEXIT_CRITICAL();

    enc1_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    enc2_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
    enc1_delta = encoder_delta_u16(enc1_now, previous_enc1);
    enc2_delta = encoder_delta_u16(enc2_now, previous_enc2);

    if (imu_snapshot.mpu_data_valid)
    {
        flags |= TELEMETRY_FLAG_MPU_VALID;
    }
    if (imu_snapshot.hmc_data_valid)
    {
        flags |= TELEMETRY_FLAG_HMC_VALID;
    }

    tx_frame[0] = TELEMETRY_SYNC_0;
    tx_frame[1] = TELEMETRY_SYNC_1;
    tx_frame[2] = TELEMETRY_VERSION;
    tx_frame[3] = TELEMETRY_PAYLOAD_LEN;
    payload = &tx_frame[4];

    /* Payload format (all multi-byte values little-endian):
       uint32 seq, uint32 HAL_ms,
       uint16 encoder1_count, uint16 encoder2_count,
       int16 encoder1_delta, int16 encoder2_delta,
       uint8 validity_flags,
       int16 MPU ax, ay, az, temp_raw, gx, gy, gz,
       int16 HMC x, y, z. */
    put_u32_le(&payload[payload_offset], sequence_number); payload_offset += 4U;
    put_u32_le(&payload[payload_offset], HAL_GetTick()); payload_offset += 4U;
    put_u16_le(&payload[payload_offset], enc1_now); payload_offset += 2U;
    put_u16_le(&payload[payload_offset], enc2_now); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], enc1_delta); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], enc2_delta); payload_offset += 2U;
    payload[payload_offset++] = flags;

    put_i16_le(&payload[payload_offset], imu_snapshot.mpu_raw.ax); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], imu_snapshot.mpu_raw.ay); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], imu_snapshot.mpu_raw.az); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], imu_snapshot.mpu_raw.temp_raw); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], imu_snapshot.mpu_raw.gx); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], imu_snapshot.mpu_raw.gy); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], imu_snapshot.mpu_raw.gz); payload_offset += 2U;

    put_i16_le(&payload[payload_offset], imu_snapshot.hmc_raw.mx); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], imu_snapshot.hmc_raw.my); payload_offset += 2U;
    put_i16_le(&payload[payload_offset], imu_snapshot.hmc_raw.mz); payload_offset += 2U;

    /* Guard against a future protocol edit that forgets to update the length. */
    if (payload_offset != TELEMETRY_PAYLOAD_LEN)
    {
        telemetry_dbg_dma_start_failures++;
        telemetry_dbg_last_hal_status = 0xFFFFFFFFUL;
        return;
    }

    /* CRC covers version + payload length + payload, not the sync bytes. */
    crc = telemetry_crc16_ccitt(&tx_frame[2], 2U + TELEMETRY_PAYLOAD_LEN);
    tx_frame[4U + TELEMETRY_PAYLOAD_LEN] = (uint8_t)(crc & 0xFFU);
    tx_frame[5U + TELEMETRY_PAYLOAD_LEN] = (uint8_t)((crc >> 8) & 0xFFU);

    /* Set busy before starting DMA. The callback clears it after completion. */
    tx_busy = 1U;
    status = HAL_UART_Transmit_DMA(&huart2, tx_frame, TELEMETRY_FRAME_LEN);
    telemetry_dbg_last_hal_status = (uint32_t)status;

    if (status == HAL_OK)
    {
        /* Only commit the encoder baseline after the UART reports full
           transmission. If this frame errors, the next delta spans the gap. */
        pending_enc1 = enc1_now;
        pending_enc2 = enc2_now;
        sequence_number++;
        telemetry_dbg_tx_started++;
    }
    else
    {
        tx_busy = 0U;
        telemetry_dbg_dma_start_failures++;
    }
}

void Telemetry_OnTxCompleteFromISR(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART2))
    {
        previous_enc1 = pending_enc1;
        previous_enc2 = pending_enc2;
        tx_busy = 0U;
        telemetry_dbg_tx_completed++;
    }
}

void Telemetry_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART2))
    {
        telemetry_dbg_uart_errors++;
        telemetry_dbg_last_uart_error = huart->ErrorCode;
        tx_busy = 0U;
    }
}
