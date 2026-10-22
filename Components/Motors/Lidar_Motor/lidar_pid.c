
#include "lidar_pid.h"

static int16_t current_pwm = 0;
static float integral = 0.0f;
static int16_t last_error = 0;
static uint32_t last_pid_time = 0;

/* Kickstart / closed-loop state. */
static bool pid_active = false;

const float Kp_lidar = 1.5f;
const float Ki_lidar = 0.02f;
const float Kd_lidar = 0.5f;


void LidarMotor_Init(void)
{
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    Lidar_UART_Init();
    LidarMotor_Stop();
}


void LidarMotor_Stop(void)
{
    current_pwm = 0;
    integral = 0.0f;
    last_error = 0;
    last_pid_time = HAL_GetTick();
    pid_active = false;

    HAL_GPIO_WritePin(
        M6_BIN1_GPIO_Port, M6_BIN1_Pin, GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        M6_BIN2_GPIO_Port, M6_BIN2_Pin, GPIO_PIN_RESET
    );

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
}


void LidarMotor_UpdatePID(void)
{
    uint32_t current_time = HAL_GetTick();

    /* Keep the DMA parser running on every call. */
    Lidar_UART_Process();

    /* Update motor control every 20 ms. */
    if ((current_time - last_pid_time) >= 20U)
    {
        int16_t actual_rpm;

        last_pid_time = current_time;

        /* Set the motor direction. */
        HAL_GPIO_WritePin(
            M6_BIN1_GPIO_Port, M6_BIN1_Pin, GPIO_PIN_SET
        );

        HAL_GPIO_WritePin(
            M6_BIN2_GPIO_Port, M6_BIN2_Pin, GPIO_PIN_RESET
        );

        actual_rpm = (int16_t)Lidar_GetLiveRPM();

        if (!pid_active)
        {
            /* STATE 1: Open-loop kickstart. */
            current_pwm = LIDAR_KICKSTART_PWM;

            /*
             * Both AE and AD frames are valid speed feedback.
             * Activate PID as soon as a plausible RPM is available.
             */
            if (Lidar_IsDataValid() && actual_rpm > 30)
            {
                pid_active = true;
                integral = 0.0f;
                last_error = LIDAR_TARGET_RPM - actual_rpm;
            }
        }
        else
        {
            if (!Lidar_IsDataValid())
            {
                /*
                 * STATE 1 recovery:
                 * Feedback has timed out. Reset the controller and
                 * return to kickstart so it can recover when data returns.
                 */
                pid_active = false;
                integral = 0.0f;
                last_error = 0;
                current_pwm = LIDAR_KICKSTART_PWM;
            }
            else
            {
                /* STATE 2: Closed-loop PID control. */
                int16_t error;
                int16_t deriv;
                int16_t pid_output;
                int16_t feed_forward_pwm = 300;
                int32_t requested_pwm;

                error = LIDAR_TARGET_RPM - actual_rpm;

                integral += (float)error;

                if (integral > 300.0f)
                {
                    integral = 300.0f;
                }
                else if (integral < -300.0f)
                {
                    integral = -300.0f;
                }

                deriv = error - last_error;
                last_error = error;

                pid_output = (int16_t)(
                    (Kp_lidar * (float)error) +
                    (Ki_lidar * integral) +
                    (Kd_lidar * (float)deriv)
                );

                /*
                 * Positional PID with feed-forward baseline.
                 * Keep the baseline and tune gains from measured results.
                 */
                requested_pwm = (int32_t)feed_forward_pwm + pid_output;

                if (requested_pwm > 999)
                {
                    requested_pwm = 999;
                }

                if (requested_pwm < 0)
                {
                    requested_pwm = 0;
                }

                current_pwm = (int16_t)requested_pwm;
            }
        }

        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, current_pwm);
    }
}


uint16_t LidarMotor_GetPWM(void)
{
    return (uint16_t)current_pwm;
}


bool LidarMotor_IsPIDActive(void)
{
    return pid_active;
}
