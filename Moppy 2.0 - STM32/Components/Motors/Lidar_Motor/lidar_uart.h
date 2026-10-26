#ifndef LIDAR_UART_H
#define LIDAR_UART_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Using USART3 based on your updated CubeMX configuration */
extern UART_HandleTypeDef huart3;

/* Circular Buffer Size (Must be large enough to hold several packets) */
#define LIDAR_RX_BUFFER_SIZE 2048

#if LIDAR_DIAGNOSTICS

extern volatile uint32_t lidar_dbg_uart_errors;
extern volatile uint32_t lidar_dbg_header_candidates;
extern volatile uint32_t lidar_dbg_command_candidates;
extern volatile uint32_t lidar_dbg_bad_format;
extern volatile uint32_t lidar_dbg_bad_checksum;
extern volatile uint32_t lidar_dbg_valid_frames;

extern volatile uint32_t lidar_dbg_dma_start_status;
extern volatile uint32_t lidar_dbg_last_uart_sr;
extern volatile uint32_t lidar_dbg_last_uart_error_flags;
extern volatile uint32_t lidar_dbg_last_uart_error_code;
extern volatile uint32_t lidar_dbg_last_dma_remaining;

extern volatile uint8_t  lidar_dbg_last_cmd;
extern volatile uint16_t lidar_dbg_last_len;

extern volatile uint32_t lidar_dbg_pclk1_hz;
extern volatile uint32_t lidar_dbg_usart_brr;

extern volatile uint32_t lidar_dbg_dma_restart_count;
extern volatile uint8_t lidar_dbg_candidate_bytes[16];
extern volatile uint8_t lidar_dbg_candidate_byte_count;
extern volatile uint8_t lidar_dbg_raw_snapshot[32];

#endif /* LIDAR_DIAGNOSTICS */



/* Initialization and Processing */
void Lidar_UART_Init(void);
void Lidar_UART_Process(void);

/* Getters for the PID Controller and GUI */
uint16_t Lidar_GetLiveRPM(void);
bool Lidar_IsDataValid(void);
uint8_t Lidar_GetPacketLength(void);
bool Lidar_IsLaserOn(void);

/* NEW HARDWARE DIAGNOSTIC GETTERS */
bool Lidar_IsRxHardwareActive(void);
uint32_t Lidar_GetTotalBytesReceived(void);

#endif /* LIDAR_UART_H */
