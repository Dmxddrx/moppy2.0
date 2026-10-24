#include "oledGUI.h"
#include <stdio.h>

/* Global variable to control the current page (0 to 3) */
uint8_t current_gui_page = 3;

/* PAGE 0: M1 & M2 (Main Wheels & Encoders) */
static void render_page_main_wheels(void)
{
    char buf[32];

    OLED_Clear();
    OLED_Print(10, 0, "M1(L)");
    OLED_Print(78, 0, "M2(R)");
    OLED_DrawLine(0, 10, 128, 10);
    OLED_DrawLine(64, 0, 64, 64);

    uint16_t wl_pwm = MainWheels_GetLeftSpeed();
    int8_t   wl_dir = MainWheels_GetLeftDirection();
    float dist_L = total_dist_L_cm;

    uint16_t wr_pwm = MainWheels_GetRightSpeed();
    int8_t   wr_dir = MainWheels_GetRightDirection();
    float dist_R = total_dist_R_cm;

    char *l_dir_s = (wl_dir == 1) ? "FWD" : ((wl_dir == -1) ? "REV" : "STP");
    char *r_dir_s = (wr_dir == 1) ? "FWD" : ((wr_dir == -1) ? "REV" : "STP");

    /* Left Wheel (M1) */
    snprintf(buf, sizeof(buf), "P:%-3u", wl_pwm);
    OLED_Print(2, 16, buf);
    snprintf(buf, sizeof(buf), "D:%s", l_dir_s);
    OLED_Print(2, 32, buf);
    snprintf(buf, sizeof(buf), "E:%.1fcm", dist_L);
    OLED_Print(2, 48, buf);

    /* Right Wheel (M2) */
    snprintf(buf, sizeof(buf), "P:%-3u", wr_pwm);
    OLED_Print(68, 16, buf);
    snprintf(buf, sizeof(buf), "D:%s", r_dir_s);
    OLED_Print(68, 32, buf);
    snprintf(buf, sizeof(buf), "E:%.1fcm", dist_R);
    OLED_Print(68, 48, buf);

    OLED_Update();
}

/* PAGE 1: Brushes (M3, M4, M5) */
static void render_page_brushes(void)
{
    char buf[32];

    OLED_Clear();
    OLED_Print(22, 0, "BRUSH STATUS");
    OLED_DrawLine(0, 10, 128, 10);

    // Read real-time hardware PWM values
    uint16_t m3_pwm = SideBrushes_GetLeftSpeed();
    uint16_t m4_pwm = SideBrushes_GetRightSpeed();
    uint16_t m5_pwm = RollerBrush_GetSpeed();

    // Determine basic ON/OFF string states based on PWM
    const char *m3_stat = (m3_pwm > 0) ? "ON " : "OFF";
    const char *m4_stat = (m4_pwm > 0) ? "ON " : "OFF";
    const char *m5_stat = (m5_pwm > 0) ? "ON " : "OFF";

    /* Left Side Brush (M3) */
    snprintf(buf, sizeof(buf), "M3(L): %s | PWM:%-3u", m3_stat, m3_pwm);
    OLED_Print(2, 16, buf);

    /* Right Side Brush (M4) */
    snprintf(buf, sizeof(buf), "M4(R): %s | PWM:%-3u", m4_stat, m4_pwm);
    OLED_Print(2, 32, buf);

    /* Main Roller Brush (M5) */
    snprintf(buf, sizeof(buf), "M5(M): %s | PWM:%-3u", m5_stat, m5_pwm);
    OLED_Print(2, 48, buf);

    OLED_Update();
}

/* PAGE 2: LiDAR Diagnostics */
static void render_page_lidar_diagnostics(void)
{
    char buf[32];

    OLED_Clear();
    OLED_Print(14, 0, "LIDAR DIAGNOSTICS");
    OLED_DrawLine(0, 10, 128, 10);

    // Hardware status
    // 1. Hardware Status (Layer 1: DMA byte flow)
	bool hw_rx_active = Lidar_IsRxHardwareActive();
	// 2. Protocol Status (Layer 2: 0xAA header parsed)
	bool is_valid = Lidar_IsDataValid();
	// 3. Application Status (Layer 3: 0xAD scanning phase)
	bool is_scanning = Lidar_IsLaserOn();

    // Protocol status
	uint16_t rpm = Lidar_GetLiveRPM();
	uint16_t pwm = LidarMotor_GetPWM();
	bool pid_active = LidarMotor_IsPIDActive();

	/* Line 1: Direct DMA Hardware Indicator */
	const char *hw_str = hw_rx_active ? "FLOW OK" : "NO BYTES";
	snprintf(buf, sizeof(buf), "HW  : %s", hw_str);
	OLED_Print(2, 14, buf);

	/* Line 2: Valid Protocol Indicator (Both 0xAE and 0xAD) */
	const char *proto_str = is_valid ? "PROTOCOL OK" : "NO DATA";
	snprintf(buf, sizeof(buf), "PROT: %s", proto_str);
	OLED_Print(2, 24, buf);

	/* Line 3: Scanning Phase Indicator (Strictly 0xAD state) */
	const char *scan_str = is_scanning ? "SCANNING OK" : "NOT SCANNING";
	snprintf(buf, sizeof(buf), "STAT: %s", scan_str);
	OLED_Print(2, 34, buf);

	/* Line 4: Speed */
	snprintf(buf, sizeof(buf), "RPM : %u", rpm);
	OLED_Print(2, 44, buf);

	/* Line 5: Motor PWM Output */
	snprintf(
	    buf,
	    sizeof(buf),
	    "PWM:%u %s",
	    pwm,
	    pid_active ? "PID" : "OPEN"
	);

	OLED_Print(2, 54, buf);

	OLED_Update();
}

static const char *mpu_status_text(const IMU_Data *data)
{
    if (data->mpu_status == MPU_NO_I2C)
        return "NO I2C";

    if (data->mpu_status == MPU_WRONG_ID)
        return "BAD ID";

    return data->mpu_data_valid ? "OK" : "WAIT";
}


static const char *hmc_status_text(const IMU_Data *data)
{
    if (data->hmc_status == HMC_NO_I2C)
        return "NO I2C";

    if (data->hmc_status == HMC_WRONG_ID)
        return "BAD ID";

    return data->hmc_data_valid ? "OK" : "WAIT";
}


/* PAGE 3: MPU6050 + HMC5883L live data */
static void render_page_imu_data(void)
{
    char buf[32];
    const IMU_Data *data = IMU_GetData();

    uint32_t mpu_sample_age_ms;

    if (data->mpu_timestamp_ms == 0U)
    {
        mpu_sample_age_ms = UINT32_MAX;  // No sample received yet
    }
    else
    {
        mpu_sample_age_ms =
            HAL_GetTick() - data->mpu_timestamp_ms;
    }

    OLED_Clear();

    OLED_Print(20, 0, "IMU SENSOR DATA");
    OLED_DrawLine(0, 10, 128, 10);

    /* Sensor initialization and data-acquisition status */
    snprintf(
        buf,
        sizeof(buf),
        "MPU:%s HMC:%s",
        mpu_status_text(data),
        hmc_status_text(data)
    );
    OLED_Print(2, 12, buf);

    /* Accelerometer: m/s^2 */
    if (data->mpu_data_valid)
    {
        snprintf(
            buf,
            sizeof(buf),
            "A:%+.1f,%+.1f,%+.1f",
            (double)data->mpu_physical.ax,
            (double)data->mpu_physical.ay,
            (double)data->mpu_physical.az
        );
    }
    else
    {
        snprintf(buf, sizeof(buf), "A: WAITING");
    }
    OLED_Print(2, 22, buf);

    /* Gyroscope: degrees per second */
    if (data->mpu_data_valid)
    {
        snprintf(
            buf,
            sizeof(buf),
            "G:%+.1f,%+.1f,%+.1f",
            (double)data->mpu_physical.gx,
            (double)data->mpu_physical.gy,
            (double)data->mpu_physical.gz
        );
    }
    else
    {
        snprintf(buf, sizeof(buf), "G: WAITING");
    }
    OLED_Print(2, 32, buf);

    /* Raw magnetic readings divided by 100 for screen space */
    if (data->hmc_data_valid)
    {
        snprintf(
            buf,
            sizeof(buf),
            "M/100:%d,%d,%d",
            (int)(data->hmc_raw.mx / 100),
            (int)(data->hmc_raw.my / 100),
            (int)(data->hmc_raw.mz / 100)
        );
    }
    else
    {
        snprintf(buf, sizeof(buf), "M: WAITING");
    }
    OLED_Print(2, 42, buf);


    /* MPU6050 internal temperature */
    /*if (data->mpu_data_valid)
    {
        snprintf(
            buf,
            sizeof(buf),
            "TEMP: %.1f C",
            (double)data->mpu_physical.temperature_c
        );
    }
    else
    {
        snprintf(buf, sizeof(buf), "TEMP: --.- C");
    }
    OLED_Print(2, 52, buf);*/


    if (mpu_sample_age_ms == UINT32_MAX)
    {
        snprintf(buf, sizeof(buf), "MPU AGE: NEVER");
    }
    else
    {
        snprintf(buf, sizeof(buf), "AGE:%lu ms %s",
                 (unsigned long)mpu_sample_age_ms,
                 data->mpu_data_valid ? "VALID" : "ERROR");
    }

    OLED_Print(2, 52, buf);

    OLED_Update();
}
/* MASTER ROUTER */
void render_dashboard_page(void)
{
    switch (current_gui_page) {
        case 0:
        	render_page_main_wheels();
        	break;

        case 1:
        	render_page_brushes();
			break;

		case 2:
			render_page_lidar_diagnostics();
			break;

		case 3:
			render_page_imu_data();
			break;

        default:
            current_gui_page = 0;
            render_page_lidar_diagnostics();
            break;
    }
}
