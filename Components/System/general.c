#include "general.h"

volatile uint32_t debug_chassis_dt_ms = 0U;
volatile uint32_t debug_chassis_dt_max_ms = 0U;
volatile uint32_t debug_chassis_late_count = 0U;

void General_Init(void)
{
	IMU_Init();
    /* 1. Activate TB6612FNG Motor Driver 1 (Main Wheels) */
	 /* Enable main wheel motor driver after IMU initialization. */
		HAL_GPIO_WritePin(
			MD1_STBY_GPIO_Port,
			MD1_STBY_Pin,
			GPIO_PIN_SET
		);

    /* 2. Initialize Subsystems */
    DEBUG_BOOT_STAGE(30);
    MainWheels_Init();

    DEBUG_BOOT_STAGE(31);
    SideBrushes_Init();

    DEBUG_BOOT_STAGE(32);
    RollerBrush_Init();

    DEBUG_BOOT_STAGE(33);
    LidarMotor_Init();

    DEBUG_BOOT_STAGE(34);
    Chassis_Init();

    DEBUG_BOOT_STAGE(35);
    BTNS_Init();

    DEBUG_BOOT_STAGE(36);
    OLED_Init(&hi2c1);

    OLED_Clear();
    OLED_Print(10, 20, "MOPPY 2.0 BOOTING");
    OLED_Print(10, 35, "WHEELS ENABLED");
    OLED_Update();

    // 1-second boot delay to allow hardware to stabilize[cite: 54]
    for (int i = 0; i < 10; i++) {
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }

    // Set a baseline forward speed for testing your PID and encoders
    Chassis_SetTargetSpeed(200, 200);
    // Set target speeds for the cleaning hardware ONCE
	SideBrushes_Start(400);
	RollerBrush_Start(400);

}

void General_Run(void)
{
    HAL_IWDG_Refresh(&hiwdg);

    uint32_t current_time = HAL_GetTick();

    static uint32_t last_display_update = 0;
    static uint32_t last_chassis_update = 0U;
    static uint8_t chassis_timer_started = 0U;

    DEBUG_BOOT_STAGE(50);

    /* --- 1. PROCESS INPUTS --- */
    BTNS_Update();

    // GUI Navigation Logic using the new debounced buttons
    if (BTNS_IsK0_Pressed()) {
        current_gui_page++;
        if (current_gui_page > 3) current_gui_page = 0;
        last_display_update = 0; // Force immediate display update
    }
    if (BTNS_IsK1_Pressed()) {
        if (current_gui_page == 0) current_gui_page = 3;
        else current_gui_page--;
        last_display_update = 0;
    }

    /* --- 2. DETERMINISTIC CHASSIS CONTROL --- */
    // Execute the PID loop strictly every 20ms to satisfy the Δt requirement

    if (chassis_timer_started == 0U)
    {
        last_chassis_update = current_time;
        chassis_timer_started = 1U;
    }
    else if ((uint32_t)(current_time - last_chassis_update) >= 20U)
    {
        uint32_t dt_ms = current_time - last_chassis_update;

        debug_chassis_dt_ms = dt_ms;

        if (dt_ms > debug_chassis_dt_max_ms)
        {
            debug_chassis_dt_max_ms = dt_ms;
        }

        if (dt_ms > 25U)
        {
            debug_chassis_late_count++;
        }

        last_chassis_update = current_time;

        Chassis_Update();
    }

    IMU_Process();
    /* --- 3. BRUSH & LIDAR SOFT START UPDATES --- */
	// Continuously step the PWM towards the targets set in initialization
    DEBUG_BOOT_STAGE(53);
    SideBrushes_Update();

    DEBUG_BOOT_STAGE(54);
    RollerBrush_Update();

    DEBUG_BOOT_STAGE(55);
    LidarMotor_UpdatePID();

    /* --- 4. PERIODIC DISPLAY UPDATES --- */
    // Refresh the OLED at 10 Hz (every 100ms) to prevent I2C bus flooding
    if ((current_time - last_display_update) >= 100U) {
        last_display_update = current_time;

        DEBUG_BOOT_STAGE(56);

        render_dashboard_page();
    }
}
