#include "general.h"

// Define some dummy variables so oledGUI.c compiles without errors
uint16_t s_lidar_raw[4] = {0, 0, 0, 0};
float s_current_yaw = 0.0f;
int s_disp_l_pwm = 0;
int s_disp_r_pwm = 0;
typedef enum { CMD_STOP, CMD_DRIVE_FORWARD } CoverageCmd;
CoverageCmd s_current_cmd = CMD_STOP;

void General_Init(void)
{
    MainWheels_Init();
    SideBrushes_Init();
    RollerBrush_Init();
    VacuumBlower_Init();

    OLED_Init(&hi2c2);

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

    static uint32_t last_ramp_time = 0;
    static uint16_t blower_speed = 0;
    static uint32_t last_display_update = 0;
    static uint32_t last_pid_time = 0;

    uint32_t current_time = HAL_GetTick();

    // 1. UPDATE THE DISPLAY (10 FPS)
    if ((current_time - last_display_update) >= 100) {
        last_display_update = current_time;
        render_dashboard_page();
    }

    // 2. MOTOR LOGIC
	if (current_time < 3000) {
		// Phase 1: ARMING
		blower_speed = 0;
		VacuumBlower_SetSpeed(blower_speed);

		MainWheels_Stop();
		SideBrushes_Stop();
		RollerBrush_Stop();

		// Synchronize encoder baselines while physically stopped
		Chassis_Init();
	}
	else {
		// Phase 2: DYNAMIC RAMP-UP
		if ((current_time - last_ramp_time) >= 50) {
			last_ramp_time = current_time;
			if (blower_speed < 800) {
				blower_speed += 10;
				VacuumBlower_Stop(); // Temporarily stopped for testing
			}
		}

		// Phase 3: PID & DISTANCE CONTROL LOOP (50ms)
		if ((current_time - last_pid_time) >= 50) {
			last_pid_time = current_time;
			Chassis_Update();
		}
	}
}
