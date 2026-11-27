#include "general.h"

void General_Init(void)
{
    /*
     * ============================================================
     * STAGE 0: SAFE STARTUP
     * ============================================================
     *
     * Main drive and side-brush drivers must remain disabled.
     * The LiDAR driver is enabled later because it also drives
     * the roller-brush channel.
     */

    HAL_GPIO_WritePin(
        MD1_STBY_GPIO_Port,
        MD1_STBY_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        MD2_STBY_GPIO_Port,
        MD2_STBY_Pin,
        GPIO_PIN_RESET
    );

    // 1. Initialize IMU.
    DEBUG_BOOT_STAGE(29);
    IMU_Init();

    // 2. Initialize motor and sensor subsystems.
    DEBUG_BOOT_STAGE(30);
    MainWheels_Init();

    DEBUG_BOOT_STAGE(31);
    SideBrushes_Init();

    DEBUG_BOOT_STAGE(32);
    RollerBrush_Init();

    DEBUG_BOOT_STAGE(33);
    LidarMotor_Init();

    // 3. Initialize chassis controller with zero targets.
    DEBUG_BOOT_STAGE(34);
    Chassis_Init();

    Chassis_SetTargetSpeed(0, 0);
    MainWheels_Stop();

    // 4. Initialize user interface.
    DEBUG_BOOT_STAGE(35);
    BTNS_Init();

    DEBUG_BOOT_STAGE(36);
    OLED_Init(&hi2c1);

    OLED_Clear();

    OLED_Print(10, 20, "MOPPY 2.0 BOOTING");
    OLED_Print(10, 35, "DRIVE DISARMED");

    OLED_Update();

    // 5. Allow hardware to stabilize.
    for (int i = 0; i < 10; i++)
    {
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }

    /*
     * STAGE 0 EXPECTED STATE:
     *
     * Main wheels : OFF
     * Side brushes: OFF
     * Roller brush: OFF
     * LiDAR motor : Available
     * Vacuum      : OFF
     *
     * No cleaning or driving motors are started automatically.
     */
}

/*
 * GeneralTask responsibilities:
 * - Button processing and GUI navigation
 * - Brush ramp updates
 * - LiDAR motor PID updates
 * - OLED refresh
 *
 * IMU_Process() belongs to ImuTask.
 * Chassis_Update() belongs to WheelTask.
 */
void General_Run(void)
{
    HAL_IWDG_Refresh(&hiwdg);

    uint32_t current_time = HAL_GetTick();
    static uint32_t last_display_update = 0;
    static uint8_t vm_fault_outputs_stopped = 0U;
    static uint32_t vm_low_since_ms = 0U;
    static uint32_t vm_rearm_hold_start_ms = 0U;

    DEBUG_BOOT_STAGE(50);

    /* --- 1. PROCESS INPUTS --- */
    BTNS_Update();

    // ------------------------------------------------------------
    // VM EMERGENCY CUTOFF SAFETY GATE
    // ------------------------------------------------------------
    if (vm_fault_latched != 0U)
    {
        uint8_t vm_available =
            (HAL_GPIO_ReadPin(
                VM_FAULT_GPIO_Port,
                VM_FAULT_Pin
            ) == GPIO_PIN_RESET);

        /*
         * Require VM to remain available for at least 500 ms
         * before accepting a re-arm request.
         */
        if (vm_available != 0U)
        {
            if (vm_low_since_ms == 0U)
            {
                vm_low_since_ms = current_time;
            }
        }
        else
        {
            vm_low_since_ms = 0U;
            vm_rearm_hold_start_ms = 0U;
        }

        // Consume old navigation button events during the fault.
        (void)BTNS_IsK0_Pressed();
        (void)BTNS_IsK1_Pressed();

        /*
         * RE-ARM:
         * VM must be restored and stable.
         * Both K0 and K1 must be held for 2 seconds.
         */
        if ((vm_available != 0U) &&
            (vm_low_since_ms != 0U) &&
            ((current_time - vm_low_since_ms) >= 500U) &&
            BTNS_IsK0_Held() &&
            BTNS_IsK1_Held())
        {
            if (vm_rearm_hold_start_ms == 0U)
            {
                vm_rearm_hold_start_ms = current_time;
            }

            if ((current_time - vm_rearm_hold_start_ms) >= 2000U)
            {
                vm_fault_latched = 0U;

                vm_fault_outputs_stopped = 0U;
                vm_rearm_hold_start_ms = 0U;
                vm_low_since_ms = 0U;

                /*
                 * Restore the shared MD3 enable for the LiDAR motor.
                 * Roller brush PWM remains zero.
                 */
                HAL_GPIO_WritePin(
                    MD3_STBY_GPIO_Port,
                    MD3_STBY_Pin,
                    GPIO_PIN_SET
                );

                last_display_update = 0U;
            }
        }
        else
        {
            vm_rearm_hold_start_ms = 0U;
        }

        // --------------------------------------------------------
        // KEEP ALL MOTORS STOPPED WHILE FAULT REMAINS LATCHED
        // --------------------------------------------------------
        if (vm_fault_latched != 0U)
        {
            if (vm_fault_outputs_stopped == 0U)
            {
                MainWheels_Stop();
                SideBrushes_Stop();
                RollerBrush_Stop();
                LidarMotor_Stop();
                VacuumBlower_Stop();

                vm_fault_outputs_stopped = 1U;
            }

            // Continuously enforce hardware-driver disable states.
            HAL_GPIO_WritePin(
                MD1_STBY_GPIO_Port,
                MD1_STBY_Pin,
                GPIO_PIN_RESET
            );

            HAL_GPIO_WritePin(
                MD2_STBY_GPIO_Port,
                MD2_STBY_Pin,
                GPIO_PIN_RESET
            );

            HAL_GPIO_WritePin(
                MD3_STBY_GPIO_Port,
                MD3_STBY_Pin,
                GPIO_PIN_RESET
            );

            // Show the emergency state on the OLED.
            if ((current_time - last_display_update) >= 100U)
            {
                last_display_update = current_time;

                OLED_Clear();

                OLED_Print(18, 10, "VM POWER LOST");
                OLED_Print(10, 27, "DRIVE DISARMED");
                OLED_Print(2, 48, "HOLD K0+K1 TO REARM");

                OLED_Update();
            }

            // Do not execute the normal motor-update functions.
            return;
        }
    }

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
