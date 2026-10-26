#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Send one encoder/IMU telemetry frame if USART2 TX DMA is available. */
void Telemetry_Init(void);
void Telemetry_TaskStep(void);

/* Call these from the project's existing HAL UART callbacks. */
void Telemetry_OnTxCompleteFromISR(UART_HandleTypeDef *huart);
void Telemetry_OnUartError(UART_HandleTypeDef *huart);

/* Live diagnostics: inspect in the debugger. */
extern volatile uint32_t telemetry_dbg_tx_started;
extern volatile uint32_t telemetry_dbg_tx_completed;
extern volatile uint32_t telemetry_dbg_busy_skips;
extern volatile uint32_t telemetry_dbg_dma_start_failures;
extern volatile uint32_t telemetry_dbg_uart_errors;
extern volatile uint32_t telemetry_dbg_last_hal_status;
extern volatile uint32_t telemetry_dbg_last_uart_error;

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
