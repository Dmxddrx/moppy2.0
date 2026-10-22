#include "lidar_uart.h"

#if LIDAR_DIAGNOSTICS
    #define LIDAR_DIAG_INC(counter) ((counter)++)
#else
    #define LIDAR_DIAG_INC(counter) ((void)0)
#endif

#define LIDAR_MIN_FRAME_LEN    	9U
#define LIDAR_MAX_FRAME_LEN   	100U
#define LIDAR_PROTOCOL_VERSION	0x01U
#define LIDAR_FRAME_TYPE       	0x61U
#define LIDAR_CMD_SCAN         	0xADU
#define LIDAR_CMD_SPEED        	0xAEU

#if LIDAR_DIAGNOSTICS

	volatile uint32_t lidar_dbg_dma_start_status = 0;
	volatile uint32_t lidar_dbg_last_uart_sr = 0;
	volatile uint32_t lidar_dbg_last_uart_error_flags = 0;
	volatile uint32_t lidar_dbg_last_uart_error_code = 0;
	volatile uint32_t lidar_dbg_last_dma_remaining = 0;

	volatile uint32_t lidar_dbg_uart_errors = 0;
	volatile uint32_t lidar_dbg_header_candidates = 0;
	volatile uint32_t lidar_dbg_command_candidates = 0;
	volatile uint32_t lidar_dbg_bad_format = 0;
	volatile uint32_t lidar_dbg_bad_checksum = 0;
	volatile uint32_t lidar_dbg_valid_frames = 0;

	volatile uint8_t  lidar_dbg_last_cmd = 0;
	volatile uint16_t lidar_dbg_last_len = 0;

	volatile uint32_t lidar_dbg_pclk1_hz = 0;
	volatile uint32_t lidar_dbg_usart_brr = 0;

#endif

static volatile uint8_t lidar_uart_restart_pending = 0U;

#if LIDAR_DIAGNOSTICS

	volatile uint32_t lidar_dbg_dma_restart_count = 0U;
	volatile uint8_t lidar_dbg_candidate_bytes[16] = {0};
	volatile uint8_t lidar_dbg_candidate_byte_count = 0U;
	volatile uint8_t lidar_dbg_raw_snapshot[32];

#endif

uint8_t lidar_rx_buf[LIDAR_RX_BUFFER_SIZE];

static uint16_t process_ptr = 0;
static uint16_t live_rpm = 0;
static bool is_data_valid = false;
static uint32_t last_valid_time = 0;
static uint8_t current_packet_length = 0;
static bool lidar_laser_on = false;
static uint16_t last_write_ptr = 0;
static uint32_t last_raw_rx_time = 0;
static uint32_t total_bytes_received = 0;


/* Read from the circular DMA buffer, including across its wrap boundary. */
static uint8_t lidar_buffer_peek(uint16_t start, uint16_t offset)
{
    uint16_t index = (uint16_t)(
        ((uint32_t)start + offset) % LIDAR_RX_BUFFER_SIZE
    );

    volatile uint8_t *buffer = lidar_rx_buf;
    return buffer[index];
}

/* Number of received bytes between two ring-buffer positions. */
static uint16_t lidar_bytes_available(uint16_t start, uint16_t end)
{
    if (end >= start)
    {
        return end - start;
    }

    return (uint16_t)((LIDAR_RX_BUFFER_SIZE - start) + end);
}

/*
 * Protocol checksum:
 * Sum all bytes from the AA header through the end of the payload.
 * The final two bytes contain the checksum, high byte first.
 */
static bool lidar_checksum_valid(uint16_t start, uint16_t frame_len)
{
    uint16_t checksum = 0;
    uint16_t received_checksum;
    uint16_t i;

    for (i = 0; i < frame_len; i++)
    {
        checksum = (uint16_t)(checksum + lidar_buffer_peek(start, i));
    }

    received_checksum =
        ((uint16_t)lidar_buffer_peek(start, frame_len) << 8) |
        (uint16_t)lidar_buffer_peek(start, frame_len + 1U);

    return (checksum == received_checksum);
}

void Lidar_UART_Init(void)
{
    HAL_StatusTypeDef rx_status;

    process_ptr = 0;
    last_write_ptr = 0;

    live_rpm = 0;
    is_data_valid = false;
    last_valid_time = 0;
    current_packet_length = 0;
    lidar_laser_on = false;

    last_raw_rx_time = 0;
    total_bytes_received = 0;

    lidar_uart_restart_pending = 0U;

    rx_status = HAL_UART_Receive_DMA(
        &huart3,
        lidar_rx_buf,
        LIDAR_RX_BUFFER_SIZE
    );

#if LIDAR_DIAGNOSTICS
    lidar_dbg_dma_start_status = (uint32_t)rx_status;
    lidar_dbg_pclk1_hz = HAL_RCC_GetPCLK1Freq();
    lidar_dbg_usart_brr = huart3.Instance->BRR;
#endif

    if (rx_status == HAL_OK)
    {
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void Lidar_UART_Process(void)
{
    uint16_t write_ptr;
    uint16_t dma_remaining;
    HAL_StatusTypeDef rx_status;

    if (huart3.hdmarx == NULL)
    {
        return;
    }

    /*
     * Recover only after HAL has finished handling the receive error
     * and both UART RX and DMA are ready for a new transfer.
     */
    if (lidar_uart_restart_pending != 0U)
    {
        if ((huart3.RxState != HAL_UART_STATE_READY) ||
            (huart3.hdmarx->State != HAL_DMA_STATE_READY))
        {
            return;
        }

        lidar_uart_restart_pending = 0U;

        /* Discard partial data from the interrupted transfer. */
        process_ptr = 0U;
        last_write_ptr = 0U;
        last_raw_rx_time = 0U;

        live_rpm = 0U;
        is_data_valid = false;
        lidar_laser_on = false;
        current_packet_length = 0U;

        rx_status = HAL_UART_Receive_DMA(
            &huart3,
            lidar_rx_buf,
            LIDAR_RX_BUFFER_SIZE
        );

        #if LIDAR_DIAGNOSTICS
        lidar_dbg_dma_start_status = (uint32_t)rx_status;
        #endif

        if (rx_status == HAL_OK)
        {
		#if LIDAR_DIAGNOSTICS
			lidar_dbg_dma_restart_count++;
		#endif
            return;
        }

        /* Retry on a later main-loop call if restart fails. */
        lidar_uart_restart_pending = 1U;
        return;
    }

    /* Do not parse data if reception is no longer active. */
    if (huart3.RxState != HAL_UART_STATE_BUSY_RX)
    {
        return;
    }

	/* 2. Get current DMA write position */
    dma_remaining = __HAL_DMA_GET_COUNTER(huart3.hdmarx);

    /* Record the live DMA remaining count for the debugger. */
#if LIDAR_DIAGNOSTICS
    lidar_dbg_last_dma_remaining = dma_remaining;
#endif

    write_ptr = (LIDAR_RX_BUFFER_SIZE - dma_remaining)
                % LIDAR_RX_BUFFER_SIZE;

#if LIDAR_DIAGNOSTICS
		for (uint32_t i = 0; i < 32; i++)
		{
			uint32_t index =
				(write_ptr + LIDAR_RX_BUFFER_SIZE - 32 + i)
				% LIDAR_RX_BUFFER_SIZE;

			lidar_dbg_raw_snapshot[i] = lidar_rx_buf[index];
		}
#endif

	/* 3. Count incoming bytes */
	if (write_ptr != last_write_ptr)
	{
		uint16_t new_bytes;

		if (write_ptr >= last_write_ptr)
		{
			new_bytes = write_ptr - last_write_ptr;
		}
		else
		{
			new_bytes = (uint16_t)(
				(LIDAR_RX_BUFFER_SIZE - last_write_ptr) + write_ptr
			);
		}

		total_bytes_received += new_bytes;
		last_write_ptr = write_ptr;
		last_raw_rx_time = HAL_GetTick();
	}

	/* 4. Parse received bytes */
	while (process_ptr != write_ptr)
	{
		uint16_t bytes_available;
		uint16_t frame_len;
		uint16_t total_packet_size;
		uint16_t payload_len;
		uint8_t byte_cmd;
		uint8_t speed_L;

		bytes_available = lidar_bytes_available(process_ptr, write_ptr);

		if (bytes_available == 0U)
		{
			break;
		}

		/* Find AA frame header */
		if (lidar_buffer_peek(process_ptr, 0U) != 0xAAU)
		{
			process_ptr =
				(uint16_t)((process_ptr + 1U) % LIDAR_RX_BUFFER_SIZE);
			continue;
		}

		/* Wait until at least the header and length are available */
		if (bytes_available < 3U)
		{
			break;
		}

		/* Frame length */
		frame_len =
			((uint16_t)lidar_buffer_peek(process_ptr, 1U) << 8) |
			 (uint16_t)lidar_buffer_peek(process_ptr, 2U);

		if (frame_len < LIDAR_MIN_FRAME_LEN ||
			frame_len > LIDAR_MAX_FRAME_LEN)
		{
			/* DEBUG 2: Invalid frame length */
			LIDAR_DIAG_INC(lidar_dbg_bad_format);

			process_ptr =
				(uint16_t)((process_ptr + 1U) % LIDAR_RX_BUFFER_SIZE);
			continue;
		}

		total_packet_size = (uint16_t)(frame_len + 2U);

		/* Wait for the complete frame, including both checksum bytes. */
		if (bytes_available < total_packet_size)
		{
			break;
		}


#if LIDAR_DIAGNOSTICS
		/*
		 * Save the first 16 bytes of the latest complete frame candidate.
		 * Inspect this array in STM32CubeIDE Memory or Expressions view.
		 */
		uint16_t copy_count;
		uint16_t i;

		copy_count = bytes_available;

		if (copy_count > 16U)
		{
		    copy_count = 16U;
		}

		for (i = 0U; i < 16U; i++)
		{
		    if (i < copy_count)
		    {
		        lidar_dbg_candidate_bytes[i] =
		            lidar_buffer_peek(process_ptr, i);
		    }
		    else
		    {
		        lidar_dbg_candidate_bytes[i] = 0U;
		    }
		}

		lidar_dbg_candidate_byte_count = (uint8_t)copy_count;
#endif
		/*
		 * DEBUG 3: Count a complete AA/header candidate.
		 * Placed here so the same incomplete packet is not
		 * counted repeatedly during successive parser calls.
		 */
		LIDAR_DIAG_INC(lidar_dbg_header_candidates);

		/* Validate protocol version and frame type. */
		if (lidar_buffer_peek(process_ptr, 3U) !=
				LIDAR_PROTOCOL_VERSION ||
			lidar_buffer_peek(process_ptr, 4U) != LIDAR_FRAME_TYPE)
		{
			/* DEBUG 4: Invalid protocol header */
			LIDAR_DIAG_INC(lidar_dbg_bad_format);

			process_ptr =
				(uint16_t)((process_ptr + 1U) % LIDAR_RX_BUFFER_SIZE);
			continue;
		}

		/* Extract command byte */
		byte_cmd = lidar_buffer_peek(process_ptr, 5U);

		/*
		 * DEBUG 5: Record what the parser actually sees.
		 * These values remain available in the debugger.
		 */
#if LIDAR_DIAGNOSTICS
		lidar_dbg_last_cmd = byte_cmd;
		lidar_dbg_last_len = frame_len;
#endif

		if (byte_cmd != LIDAR_CMD_SCAN &&
			byte_cmd != LIDAR_CMD_SPEED)
		{
			/* DEBUG 6: Unsupported command */
			LIDAR_DIAG_INC(lidar_dbg_bad_format);

			process_ptr =
				(uint16_t)((process_ptr + 1U) % LIDAR_RX_BUFFER_SIZE);
			continue;
		}

		/*
		 * DEBUG 7: Count recognized AD/AE commands.
		 * A command candidate has not yet passed checksum validation.
		 */
		LIDAR_DIAG_INC(lidar_dbg_command_candidates);

		/* Payload length is also big-endian. */
		payload_len =
			((uint16_t)lidar_buffer_peek(process_ptr, 6U) << 8) |
			 (uint16_t)lidar_buffer_peek(process_ptr, 7U);

		/*
		 * Frame length = 8 bytes of header/metadata + payload length.
		 */
		if ((uint32_t)payload_len + 8U != frame_len)
		{
			/* DEBUG 8: Inconsistent lengths */
			LIDAR_DIAG_INC(lidar_dbg_bad_format);

			process_ptr =
				(uint16_t)((process_ptr + 1U) % LIDAR_RX_BUFFER_SIZE);
			continue;
		}

		/* AE must have one speed byte */
		if (byte_cmd == LIDAR_CMD_SPEED && payload_len != 1U)
		{
			/* DEBUG 9: Invalid AE payload size */
			LIDAR_DIAG_INC(lidar_dbg_bad_format);

			process_ptr =
				(uint16_t)((process_ptr + 1U) % LIDAR_RX_BUFFER_SIZE);
			continue;
		}

		/*
		 * 0xAD contains the speed, angle metadata and 3-byte samples.
		 * We validate their payload size but do not decode the samples.
		 */
		if (byte_cmd == LIDAR_CMD_SCAN &&
			(payload_len < 5U || ((payload_len - 5U) % 3U) != 0U))
		{
			/* DEBUG 10: Invalid AD payload structure */
			LIDAR_DIAG_INC(lidar_dbg_bad_format);

			process_ptr =
				(uint16_t)((process_ptr + 1U) % LIDAR_RX_BUFFER_SIZE);
			continue;
		}

		/* Verify checksum before accepting the packet */
		if (!lidar_checksum_valid(process_ptr, frame_len))
		{
			/* DEBUG 11: Count checksum failures */
			LIDAR_DIAG_INC(lidar_dbg_bad_checksum);

			process_ptr =
				(uint16_t)((process_ptr + 1U) % LIDAR_RX_BUFFER_SIZE);
			continue;
		}

		/*
		 * CRITICAL FIX:
		 * Offset 8 is the single-byte speed value in BOTH frame types.
		 * RPM = speed byte * 3.
		 *
		 * Do not read offset 9 as another speed byte: it is not part
		 * of the speed field.
		 */
		speed_L = lidar_buffer_peek(process_ptr, 8U);
		live_rpm = (uint16_t)speed_L * 3U;

		current_packet_length = (uint8_t)frame_len;
		last_valid_time = HAL_GetTick();
		is_data_valid = true;

		/* AD means scan data is being produced; AE means it is not. */
		lidar_laser_on = (byte_cmd == LIDAR_CMD_SCAN);

		/*
		 * DEBUG 12: This counter increases only after
		 * a recognized frame passes format and checksum checks.
		 */
		LIDAR_DIAG_INC(lidar_dbg_valid_frames);

		/* Skip the whole validated frame, not its scan measurements. */
		process_ptr = (uint16_t)(
			(process_ptr + total_packet_size) % LIDAR_RX_BUFFER_SIZE
		);
	}

	/* Clear stale readings if no valid protocol frame arrives. */
	if ((HAL_GetTick() - last_valid_time) > 1000U)
	{
		is_data_valid = false;
		live_rpm = 0;
		lidar_laser_on = false;
		current_packet_length = 0;
	}
}

bool Lidar_IsRxHardwareActive(void) {
    return (total_bytes_received > 0U) &&
           ((HAL_GetTick() - last_raw_rx_time) < 300U);
}

uint32_t Lidar_GetTotalBytesReceived(void) {
    return total_bytes_received;
}

uint16_t Lidar_GetLiveRPM(void) { return live_rpm; }
bool Lidar_IsDataValid(void) { return is_data_valid; }
uint8_t Lidar_GetPacketLength(void) { return current_packet_length; }
bool Lidar_IsLaserOn(void) { return lidar_laser_on; }

#if LIDAR_DIAGNOSTICS

static uint32_t lidar_error_code_to_flags(uint32_t error_code)
{
    uint32_t flags = 0U;

    if ((error_code & HAL_UART_ERROR_PE) != 0U)
        flags |= UART_FLAG_PE;

    if ((error_code & HAL_UART_ERROR_NE) != 0U)
        flags |= UART_FLAG_NE;

    if ((error_code & HAL_UART_ERROR_FE) != 0U)
        flags |= UART_FLAG_FE;

    if ((error_code & HAL_UART_ERROR_ORE) != 0U)
        flags |= UART_FLAG_ORE;

    return flags;
}

#endif /* LIDAR_DIAGNOSTICS */


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if LIDAR_DIAGNOSTICS
    uint32_t error_code;
#endif

    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }

#if LIDAR_DIAGNOSTICS

    error_code = huart->ErrorCode;

    LIDAR_DIAG_INC(lidar_dbg_uart_errors);

    lidar_dbg_last_uart_error_code = error_code;

    lidar_dbg_last_uart_error_flags =
        lidar_error_code_to_flags(error_code);

    lidar_dbg_last_uart_sr = huart->Instance->SR;

#endif

    /*
     * HAL handles the UART/DMA error.
     * Request a restart from the main loop instead of restarting
     * reception inside the callback.
     */
    lidar_uart_restart_pending = 1U;
}
