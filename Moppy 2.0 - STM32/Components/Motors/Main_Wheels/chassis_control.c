#include "chassis_control.h"
#include "imu_manager.h"
#include <math.h>

// Global Odometry Variables[cite: 16]
float total_distance_cm = 0.0f;
float total_dist_L_cm   = 0.0f;
float total_dist_R_cm   = 0.0f;

// Trajectory and Setpoint Tracking
static int16_t final_target_ticks_L = 0;
static int16_t final_target_ticks_R = 0;
static float current_target_ticks_L = 0.0f;
static float current_target_ticks_R = 0.0f;

// Encoder Tracking State[cite: 16]
static int16_t last_enc_L = 0;
static int16_t last_enc_R = 0;

// PID Controller State[cite: 16]
static float integral_L = 0.0f, integral_R = 0.0f;
static int16_t last_error_L = 0, last_error_R = 0;

// Moppy 2.0 Physical Constants[cite: 16]
const float WHEEL_CIRCUMFERENCE_CM = 21.04867f;
const float TICKS_PER_REVOLUTION   = 943.0f;

// PID & Trajectory Constants
const float Kp = 2.0f; //[cite: 16]
const float Ki = 0.05f; //[cite: 16]
const float Kd = 0.5f; //[cite: 16]
const float ACCEL_STEP = 2.0f; // Max target change per 20ms loop (Higher = steeper acceleration)

/* Initial turn-control settings; tune on the real chassis. */
#define TURN_ANGLE_TOLERANCE_DEG   1.5f
#define TURN_SETTLE_RATE_DPS       3.0f
#define TURN_CONTROL_KP             2.0f

#define TURN_MIN_RATE_DPS           5.0f
#define TURN_MAX_RATE_DPS          90.0f
#define TURN_MAX_ANGLE_DEG         360.0f

#define TURN_SENSOR_TIMEOUT_MS     100U
#define TURN_SETTLE_SAMPLES          5U
#define TURN_TIMEOUT_MARGIN_MS   3000U

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_TURNING,
    MOTION_DONE,
    MOTION_FAULT
} ChassisMotionState;

static ChassisMotionState motion_state = MOTION_IDLE;

static float turn_target_yaw_deg = 0.0f;
static float turn_max_rate_dps = 0.0f;

static uint32_t turn_started_ms = 0U;
static uint32_t turn_timeout_ms = 0U;

static uint8_t turn_settle_count = 0U;

void Chassis_Init(void)
{
    // 1. Synchronize initial encoder baselines[cite: 16]
    last_enc_L = MainWheels_GetLeftEncoder();
    last_enc_R = MainWheels_GetRightEncoder();

    // 2. Reset PID controller states[cite: 16]
    integral_L = 0.0f;
    integral_R = 0.0f;
    last_error_L = 0;
    last_error_R = 0;

    // 3. Reset setpoints and trajectory profiles
    final_target_ticks_L = 0;
    final_target_ticks_R = 0;
    current_target_ticks_L = 0.0f;
    current_target_ticks_R = 0.0f;

    motion_state = MOTION_IDLE;
    turn_settle_count = 0U;
}

void Chassis_SetTargetSpeed(int16_t left_ticks_per_loop, int16_t right_ticks_per_loop)
{
    // Register the final desired speed. The Update loop will smoothly ramp towards this.[cite: 16]
    final_target_ticks_L = left_ticks_per_loop;
    final_target_ticks_R = right_ticks_per_loop;
}


/* Measure this on Moppy 2.0: centre of left wheel
   to centre of right wheel, in centimetres. */
#define CHASSIS_TRACK_WIDTH_CM          24.0f

/* WheelTask / Chassis_Update period */
#define CHASSIS_CONTROL_PERIOD_S         0.020f

/* Initial limit; tune from measured encoder data. */
#define CHASSIS_MAX_TARGET_TICKS_PER_LOOP 200.0f

void Chassis_SetVelocity(float linear_cm_s,
                         float angular_deg_s)
{
    const float deg_to_rad = 0.01745329252f;

    const float ticks_per_cm =
        TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_CM;

    /* Convert requested turn rate into radians per second. */
    const float omega_rad_s =
        angular_deg_s * deg_to_rad;

    /* Positive angular velocity means turning left. */
    const float half_track =
        CHASSIS_TRACK_WIDTH_CM * 0.5f;

    const float left_cm_s =
        linear_cm_s - (omega_rad_s * half_track);

    const float right_cm_s =
        linear_cm_s + (omega_rad_s * half_track);

    /* Convert wheel velocities to ticks per 20 ms loop. */
    float left_ticks =
        left_cm_s * ticks_per_cm * CHASSIS_CONTROL_PERIOD_S;

    float right_ticks =
        right_cm_s * ticks_per_cm * CHASSIS_CONTROL_PERIOD_S;

    /*
     * Scale both wheels together if over the configured
     * limit. This preserves the requested turning ratio.
     */
    float max_ticks = fmaxf(fabsf(left_ticks),
                            fabsf(right_ticks));

    if (max_ticks > CHASSIS_MAX_TARGET_TICKS_PER_LOOP)
    {
        float scale =
            CHASSIS_MAX_TARGET_TICKS_PER_LOOP / max_ticks;

        left_ticks *= scale;
        right_ticks *= scale;
    }

    Chassis_SetTargetSpeed(
        (int16_t)lroundf(left_ticks),
        (int16_t)lroundf(right_ticks)
    );
}

void Chassis_ResetOdometry(void)
{
    total_distance_cm = 0.0f; //[cite: 16]
    total_dist_L_cm   = 0.0f; //[cite: 16]
    total_dist_R_cm   = 0.0f; //[cite: 16]
}

void Chassis_Update(void)
{
    // --- READ ENCODERS ---[cite: 16]
    int16_t current_enc_L = MainWheels_GetLeftEncoder();
    int16_t current_enc_R = MainWheels_GetRightEncoder();

    // 16-bit signed subtraction handles counter overflow/underflow automatically[cite: 16]
    int16_t delta_L = current_enc_L - last_enc_L;
    int16_t delta_R = current_enc_R - last_enc_R;

    last_enc_L = current_enc_L; //[cite: 16]
    last_enc_R = current_enc_R; //[cite: 16]

    // --- CALCULATE DISTANCE (ODOMETRY) ---[cite: 16]
    float dist_L_cm = ((float)delta_L / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;
    float dist_R_cm = ((float)delta_R / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;

    total_dist_L_cm   += dist_L_cm; //[cite: 16]
    total_dist_R_cm   += dist_R_cm; //[cite: 16]
    total_distance_cm += (dist_L_cm + dist_R_cm) / 2.0f; //[cite: 16]

    // --- TRAJECTORY GENERATOR (SOFT START / SOFT STOP) ---
    // Smoothly ramp Left target towards final target
    if (current_target_ticks_L < final_target_ticks_L) {
        current_target_ticks_L += ACCEL_STEP;
        if (current_target_ticks_L > final_target_ticks_L) current_target_ticks_L = final_target_ticks_L;
    } else if (current_target_ticks_L > final_target_ticks_L) {
        current_target_ticks_L -= ACCEL_STEP;
        if (current_target_ticks_L < final_target_ticks_L) current_target_ticks_L = final_target_ticks_L;
    }

    // Smoothly ramp Right target towards final target
    if (current_target_ticks_R < final_target_ticks_R) {
        current_target_ticks_R += ACCEL_STEP;
        if (current_target_ticks_R > final_target_ticks_R) current_target_ticks_R = final_target_ticks_R;
    } else if (current_target_ticks_R > final_target_ticks_R) {
        current_target_ticks_R -= ACCEL_STEP;
        if (current_target_ticks_R < final_target_ticks_R) current_target_ticks_R = final_target_ticks_R;
    }

    // --- PID SPEED CONTROLLER ---
    // Calculate error using the smoothed current target instead of the raw final target
    int16_t error_L = (int16_t)current_target_ticks_L - delta_L;
    int16_t error_R = (int16_t)current_target_ticks_R - delta_R;

    // Accumulate integral with anti-windup clamping[cite: 16]
    integral_L += error_L;
    integral_R += error_R;
    if (integral_L > 1000.0f)  integral_L = 1000.0f;  else if (integral_L < -1000.0f) integral_L = -1000.0f;
    if (integral_R > 1000.0f)  integral_R = 1000.0f;  else if (integral_R < -1000.0f) integral_R = -1000.0f;

    // Derivative calculation[cite: 16]
    int16_t deriv_L = error_L - last_error_L;
    int16_t deriv_R = error_R - last_error_R;
    last_error_L = error_L;
    last_error_R = error_R;

    // Compute PID output[cite: 16]
    int16_t pwm_L = (int16_t)((Kp * error_L) + (Ki * integral_L) + (Kd * deriv_L));
    int16_t pwm_R = (int16_t)((Kp * error_R) + (Ki * integral_R) + (Kd * deriv_R));

    // Clamp output to full signed motor driver range (-999 to +999)[cite: 16]
    if (pwm_L > 999)  pwm_L = 999;  else if (pwm_L < -999) pwm_L = -999;
    if (pwm_R > 999)  pwm_R = 999;  else if (pwm_R < -999) pwm_R = -999;

    // Apply motor speeds[cite: 16]
    MainWheels_SetLeftSpeed(pwm_L);
    MainWheels_SetRightSpeed(pwm_R);
}
static void Chassis_MotionStopNow(void)
{
    final_target_ticks_L = 0;
    final_target_ticks_R = 0;

    current_target_ticks_L = 0.0f;
    current_target_ticks_R = 0.0f;

    integral_L = 0.0f;
    integral_R = 0.0f;

    last_error_L = 0;
    last_error_R = 0;

    MainWheels_Stop();
}

bool Chassis_StartTurn(float angle_deg,
                       float max_rate_deg_s)
{
    IMU_Data imu;
    uint32_t now = HAL_GetTick();

    if (motion_state == MOTION_TURNING)
    {
        return false;
    }

    if (!isfinite(angle_deg) ||
        !isfinite(max_rate_deg_s) ||
        (fabsf(angle_deg) < 0.1f) ||
        (fabsf(angle_deg) > TURN_MAX_ANGLE_DEG) ||
        (max_rate_deg_s < TURN_MIN_RATE_DPS) ||
        (max_rate_deg_s > TURN_MAX_RATE_DPS))
    {
        return false;
    }

    IMU_GetDataSnapshot(&imu);

    /*
     * Do not start a closed-loop turn without recent,
     * valid gyroscope-derived yaw data.
     */
    if (!imu.mpu_data_valid ||
        !imu.yaw_valid ||
        ((uint32_t)(now - imu.mpu_timestamp_ms) >
         TURN_SENSOR_TIMEOUT_MS))
    {
        return false;
    }

    /* Positive angles request left turns. */
    turn_target_yaw_deg = imu.yaw_deg + angle_deg;

    turn_max_rate_dps = max_rate_deg_s;
    turn_started_ms = now;

    /*
     * Allow approximately twice the nominal turn duration
     * plus a margin for acceleration and settling.
     */
    turn_timeout_ms =
        (uint32_t)((fabsf(angle_deg) /
                    max_rate_deg_s) * 2000.0f)
        + TURN_TIMEOUT_MARGIN_MS;

    turn_settle_count = 0U;
    motion_state = MOTION_TURNING;

    return true;
}

void Chassis_MotionUpdate(void)
{
    if (motion_state != MOTION_TURNING)
    {
        return;
    }

    IMU_Data imu;
    uint32_t now = HAL_GetTick();

    IMU_GetDataSnapshot(&imu);

    /* Fail safely if the yaw feedback becomes unreliable. */
    if (!imu.mpu_data_valid ||
        !imu.yaw_valid ||
        ((uint32_t)(now - imu.mpu_timestamp_ms) >
         TURN_SENSOR_TIMEOUT_MS))
    {
        Chassis_MotionStopNow();
        motion_state = MOTION_FAULT;
        return;
    }

    /* Prevent an unsuccessful turn from running indefinitely. */
    if ((uint32_t)(now - turn_started_ms) >= turn_timeout_ms)
    {
        Chassis_MotionStopNow();
        motion_state = MOTION_FAULT;
        return;
    }

    float error_deg =
        turn_target_yaw_deg - imu.yaw_deg;

    float abs_error_deg = fabsf(error_deg);
    float gyro_z_dps = imu.mpu_physical.gz;

    /*
     * Once close to the target, request zero angular speed.
     * The existing wheel-speed ramp helps decelerate the robot.
     */
    if (abs_error_deg <= TURN_ANGLE_TOLERANCE_DEG)
    {
        Chassis_SetVelocity(0.0f, 0.0f);

        if (fabsf(gyro_z_dps) <= TURN_SETTLE_RATE_DPS)
        {
            if (turn_settle_count < TURN_SETTLE_SAMPLES)
            {
                turn_settle_count++;
            }

            if (turn_settle_count >= TURN_SETTLE_SAMPLES)
            {
                motion_state = MOTION_DONE;
            }
        }
        else
        {
            turn_settle_count = 0U;
        }

        return;
    }

    turn_settle_count = 0U;

    /*
     * Proportional angular-speed control:
     * larger angle error -> faster turn;
     * smaller angle error -> slower turn.
     */
    float angular_rate_cmd =
        TURN_CONTROL_KP * error_deg;

    if (angular_rate_cmd > turn_max_rate_dps)
    {
        angular_rate_cmd = turn_max_rate_dps;
    }
    else if (angular_rate_cmd < -turn_max_rate_dps)
    {
        angular_rate_cmd = -turn_max_rate_dps;
    }

    Chassis_SetVelocity(0.0f, angular_rate_cmd);
}

bool Chassis_MotionIsDone(void)
{
    return (motion_state != MOTION_TURNING);
}

bool Chassis_MotionHasFault(void)
{
    return (motion_state == MOTION_FAULT);
}

void Chassis_CancelMotion(void)
{
    Chassis_MotionStopNow();

    turn_settle_count = 0U;
    motion_state = MOTION_IDLE;
}
