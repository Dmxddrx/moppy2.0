#include "general.h"

// Define some dummy variables so oledGUI.c compiles without errors
uint16_t s_lidar_raw[4] = {0, 0, 0, 0};
float s_current_yaw = 0.0f;
int s_disp_l_pwm = 0;
int s_disp_r_pwm = 0;
typedef enum { CMD_STOP, CMD_DRIVE_FORWARD } CoverageCmd;
CoverageCmd s_current_cmd = CMD_STOP;

/* Global Sensor Variables */
HMC5883L_RawData current_mag = {0};
MPU6050_RawData current_mpu_raw = {0};
MPU6050_PhysData current_mpu_phys = {0};
float current_heading = 0.0f;

/* ═══════════════════════════════════════════════════════════════ */
/* DMA STATE MACHINE                                               */
/* ═══════════════════════════════════════════════════════════════ */
typedef enum {
    SENSOR_IDLE,
    SENSOR_MPU_WAITING,
    SENSOR_HMC_WAITING
} SensorDMAState;

volatile SensorDMAState sensor_dma_state = SENSOR_IDLE;
uint8_t mpu_rx_buf[14];
uint8_t hmc_rx_buf[6];

/* This hardware interrupt fires automatically when DMA finishes copying data! */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        if (sensor_dma_state == SENSOR_MPU_WAITING) {
            // 1. MPU data has arrived! Parse it.
            MPU6050_ParseDMA(mpu_rx_buf, &current_mpu_raw);
            current_mpu_phys = MPU6050_GetPhysical(&current_mpu_raw);

            // 2. Immediately ask DMA to fetch HMC data
            sensor_dma_state = SENSOR_HMC_WAITING;
            HMC5883L_ReadRaw_DMA(hmc_rx_buf);
        }
        else if (sensor_dma_state == SENSOR_HMC_WAITING) {
            // 3. HMC data has arrived! Parse it.
            HMC5883L_ParseDMA(hmc_rx_buf, &current_mag);
            current_heading = HMC5883L_GetHeading((float)current_mag.mx, (float)current_mag.my);

            // 4. Release the bus
            sensor_dma_state = SENSOR_IDLE;
        }
    }
}

/* Fallback: If a physical I2C wire gets bumped, reset the state machine */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
    	I2C1_RecoverBus();
        sensor_dma_state = SENSOR_IDLE;
    }
}

void General_Init(void)
{
    MainWheels_Init();
    SideBrushes_Init();
    RollerBrush_Init();
    VacuumBlower_Init();

    OLED_Init(&hi2c2);

    /* 1. Show Gyro Calibration Warning */
	OLED_Clear();
	OLED_Print(10, 20, "CALIBRATING GYRO");
	OLED_Print(10, 35, "DO NOT MOVE ROBOT!");
	OLED_Update();

	/* 2. Initialize Sensors */
	MPU6050_Init();
	HMC5883L_Init();

	/* 3. Finish Boot Sequence */
    OLED_Clear();
    OLED_Print(10, 20, "MOPPY 2.0 BOOTING");
    OLED_Update();
    for (int i = 0; i < 10; i++) {
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }
}

void System_RunAllForwardMax(void)
{
    MainWheels_Forward(999);
    SideBrushes_Start(999);
    RollerBrush_Start(999);
    VacuumBlower_SetSpeed(999);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MSD_RPM_Pin)
    {
        blower_pulse_count++;
    }
}

void General_Run(void)
{
    HAL_IWDG_Refresh(&hiwdg);
    uint32_t current_time = HAL_GetTick();

    static uint32_t last_sensor_time = 0;
    static uint32_t last_display_update = 0;
    static uint32_t last_ramp_time = 0;
    static uint16_t blower_speed = 0;

    static uint32_t i2c1_watchdog_time = 0;

    // ══════════════════════════════════════════════════════════
    // PHASE 1: ARMING
    // ══════════════════════════════════════════════════════════
    if (current_time < 3000) {
        blower_speed = 0;
        VacuumBlower_SetSpeed(0);
        MainWheels_Stop();
        SideBrushes_Stop();
        RollerBrush_Stop();
        Chassis_Init();

        if ((current_time - last_display_update) >= 100) {
            last_display_update = current_time;
            render_dashboard_page();
        }
        return;
    }

    // ══════════════════════════════════════════════════════════
    // PHASE 2: NON-BLOCKING TASK SCHEDULER
    // ══════════════════════════════════════════════════════════

    // PRIORITY 1: I2C1 DMA WATCHDOG (Catches silent hangs)
	if (sensor_dma_state != SENSOR_IDLE) {
		// If the DMA state machine has been stuck for more than 50ms...
		if ((current_time - i2c1_watchdog_time) > 50) {
			I2C1_RecoverBus();              // Force hardware reset
			sensor_dma_state = SENSOR_IDLE; // Reset software state
		}
	}

    // PRIORITY 2: Trigger DMA Sensor Read (Every 20ms)
    else if ((current_time - last_sensor_time) >= 20) {
        last_sensor_time = current_time;

        // Only trigger if the previous DMA chain has completely finished
        if (sensor_dma_state == SENSOR_IDLE && HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) {
            sensor_dma_state = SENSOR_MPU_WAITING;
            i2c1_watchdog_time = current_time;
            MPU6050_ReadRaw_DMA(mpu_rx_buf);
        }
    }

    // PRIORITY 3: Display Updates (Every 100ms)
    else if ((current_time - last_display_update) >= 100) {
        last_display_update = current_time;
        render_dashboard_page();
    }

    // PRIORITY 4: Blower Ramp-Up
    else if ((current_time - last_ramp_time) >= 50) {
        last_ramp_time = current_time;
        if (blower_speed < 800) {
            blower_speed += 10;
            VacuumBlower_SetSpeed(0);
        }
    }
}
