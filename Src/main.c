#include "main.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal_i2c.h"
#include "math.h"

/* =============================================================================
 * PERIPHERAL HANDLES
 * ============================================================================= */
TIM_HandleTypeDef htim1;   // TIM1: PWM output for SG90 servo on PA8 (CH1)
TIM_HandleTypeDef htim2;   // TIM2: 100ms periodic interrupt — main heartbeat
TIM_HandleTypeDef htim3;   // TIM3: free-running microsecond counter
UART_HandleTypeDef huart2; // USART2: serial output (no adapter available — kept for completeness)
I2C_HandleTypeDef hi2c1;   // I2C1: MPU-6050 (PB6=SCL, PB7=SDA)

/* =============================================================================
 * HC-SR04 VARIABLES (skipped — bad hardware, kept for completeness)
 * ============================================================================= */
volatile uint32_t echo_start = 0;
volatile uint32_t echo_end   = 0;
volatile uint8_t  echo_done  = 0;
volatile uint32_t dist_cm    = 0;

/* =============================================================================
 * SAMPLE FLAG — set by TIM2 ISR, consumed in main loop
 * ============================================================================= */
volatile uint8_t sample_flag = 0;

/* =============================================================================
 * MPU-6050 VARIABLES
 * ============================================================================= */
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
float ax_g,  ay_g,  az_g;
float gx_ds, gy_ds, gz_ds;
float pitch = 0.0f;
float roll  = 0.0f;

#define DT      0.1f   // 100ms time step
#define ALPHA   0.98f  // complementary filter gyro weight
uint8_t who_am_i = 0;

/* =============================================================================
 * PID CONTROLLER
 *
 * Target: hold pitch at 0 degrees (level platform).
 * Error = setpoint - measured_pitch
 * Output drives servo position.
 *
 * Kp: proportional — immediate correction proportional to error
 * Ki: integral — corrects steady-state bias that Kp alone can't fix
 * Kd: derivative — damps oscillation by resisting rapid error change
 *
 * Output is clamped to [-90, +90] degrees — maps to servo range.
 * ============================================================================= */
#define PID_SETPOINT    0.0f   // target pitch in degrees (level)
#define PID_KP          2.0f   // proportional gain — tune this first
#define PID_KI          0.05f  // integral gain — small to avoid windup
#define PID_KD          0.5f   // derivative gain — damping

#define PID_OUT_MIN    -90.0f  // minimum PID output (degrees)
#define PID_OUT_MAX     90.0f  // maximum PID output (degrees)
#define PID_INTEGRAL_MAX 50.0f // integral windup clamp

float pid_integral   = 0.0f;
float pid_prev_error = 0.0f;
float pid_output     = 0.0f;

/* =============================================================================
 * SG90 SERVO PARAMETERS
 *
 * SG90 requires 50Hz PWM (20ms period).
 * Pulse width maps to angle:
 *   1.0ms pulse = 0°   (full left)
 *   1.5ms pulse = 90°  (center)
 *   2.0ms pulse = 180° (full right)
 *
 * TIM1 config (APB2 = 100MHz):
 *   Prescaler = 99  → timer clock = 100MHz / 100 = 1MHz → 1μs per tick
 *   Period    = 19999 → 20ms period = 50Hz
 *
 * CCR (compare register) value for a given pulse width in μs:
 *   e.g. 1500μs center → CCR = 1500
 *
 * Servo center = 90° → CCR = 1500
 * PID output [-90°, +90°] maps to CCR [1000, 2000]
 * Formula: CCR = 1500 + (pid_output / 90.0f) * 500
 * ============================================================================= */
#define SERVO_CENTER_CCR  1500  // 1.5ms pulse = 90° = center
#define SERVO_MIN_CCR     1000  // 1.0ms pulse = 0°
#define SERVO_MAX_CCR     2000  // 2.0ms pulse = 180°

/* =============================================================================
 * SIMULATED PITCH (used when MPU-6050 is unavailable or returning zeros)
 *
 * Generates a sine-wave pitch signal so PID has something real to respond to.
 * sim_tick increments each 100ms cycle.
 * sim_pitch oscillates between -20° and +20° over ~6 seconds.
 * Set USE_SIM_PITCH = 0 to use real MPU data instead.
 * ============================================================================= */
#define USE_SIM_PITCH   1       // 1 = simulated pitch, 0 = real MPU pitch
uint32_t sim_tick = 0;
float    sim_pitch = 0.0f;

/* =============================================================================
 * SYSTEM STATE MACHINE
 * ============================================================================= */
typedef enum {
    STATE_IDLE         = 0,
    STATE_SAMPLING     = 1,
    STATE_TRANSMITTING = 2,
    STATE_ERROR        = 3
} SystemState;

SystemState system_state = STATE_IDLE;
uint32_t    system_tick_ms = 0;

/* =============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================= */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void MPU6050_Init(void);
void MPU6050_Read(void);
void ComplementaryFilter_Update(void);
float PID_Update(float measured_pitch);
void  Servo_SetAngle(float angle_deg);
uint32_t Servo_AngleToCCR(float angle_deg);

/* =============================================================================
 * SERVO — convert angle [0°, 180°] to CCR value
 * angle_deg = 0   → CCR = 1000 (1.0ms)
 * angle_deg = 90  → CCR = 1500 (1.5ms)
 * angle_deg = 180 → CCR = 2000 (2.0ms)
 * ============================================================================= */
uint32_t Servo_AngleToCCR(float angle_deg)
{
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;
    return (uint32_t)(SERVO_MIN_CCR + (angle_deg / 180.0f) * (SERVO_MAX_CCR - SERVO_MIN_CCR));
}

/* =============================================================================
 * SERVO — write PID output to servo
 * PID output is in [-90°, +90°] centered at 0° (level).
 * We shift it to [0°, 180°] for the servo by adding 90°.
 * So: PID=0 → servo center (90°), PID=-90 → servo left (0°), PID=+90 → right (180°)
 * ============================================================================= */
void Servo_SetAngle(float pid_out)
{
    float servo_angle = pid_out + 90.0f; // shift to [0, 180]
    uint32_t ccr = Servo_AngleToCCR(servo_angle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

/* =============================================================================
 * PID_Update
 * Call once per 100ms cycle with the current measured pitch.
 * Returns clamped output in [-90°, +90°].
 * ============================================================================= */
float PID_Update(float measured_pitch)
{
    float error = PID_SETPOINT - measured_pitch;

    // Proportional term
    float p_term = PID_KP * error;

    // Integral term with windup clamp
    pid_integral += error * DT;
    if (pid_integral >  PID_INTEGRAL_MAX) pid_integral =  PID_INTEGRAL_MAX;
    if (pid_integral < -PID_INTEGRAL_MAX) pid_integral = -PID_INTEGRAL_MAX;
    float i_term = PID_KI * pid_integral;

    // Derivative term (on error, not measurement — avoids derivative kick on setpoint change)
    float d_term = PID_KD * (error - pid_prev_error) / DT;
    pid_prev_error = error;

    // Sum and clamp
    float output = p_term + i_term + d_term;
    if (output >  PID_OUT_MAX) output =  PID_OUT_MAX;
    if (output <  PID_OUT_MIN) output =  PID_OUT_MIN;

    return output;
}

/* =============================================================================
 * MPU6050_Init — wake from sleep
 * ============================================================================= */
void MPU6050_Init(void)
{
    uint8_t data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x6B, 1, &data, 1, 10);
}

/* =============================================================================
 * MPU6050_Read — burst read 14 bytes from 0x3B
 * ============================================================================= */
void MPU6050_Read(void)
{
    uint8_t buf[14];
    HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x3B, 1, buf, 14, 10);

    ax_raw = (int16_t)(buf[0]  << 8 | buf[1]);
    ay_raw = (int16_t)(buf[2]  << 8 | buf[3]);
    az_raw = (int16_t)(buf[4]  << 8 | buf[5]);
    gx_raw = (int16_t)(buf[8]  << 8 | buf[9]);
    gy_raw = (int16_t)(buf[10] << 8 | buf[11]);
    gz_raw = (int16_t)(buf[12] << 8 | buf[13]);

    ax_g  = ax_raw / 16384.0f;
    ay_g  = ay_raw / 16384.0f;
    az_g  = az_raw / 16384.0f;
    gx_ds = gx_raw / 131.0f;
    gy_ds = gy_raw / 131.0f;
    gz_ds = gz_raw / 131.0f;
}

/* =============================================================================
 * ComplementaryFilter_Update
 * ============================================================================= */
void ComplementaryFilter_Update(void)
{
    float accel_pitch = atan2f(ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * (180.0f / M_PI);
    float accel_roll  = atan2f(-ax_g, az_g) * (180.0f / M_PI);

    pitch = ALPHA * (pitch + gx_ds * DT) + (1.0f - ALPHA) * accel_pitch;
    roll  = ALPHA * (roll  + gy_ds * DT) + (1.0f - ALPHA) * accel_roll;
}

/* =============================================================================
 * MAIN
 * ============================================================================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();   // servo PWM — init before starting
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_I2C1_Init();

    // WHO_AM_I check
    HAL_Delay(100);
    HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x75, 1, &who_am_i, 1, 10);

    MPU6050_Init();

    // Start servo PWM — servo moves to center immediately
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SERVO_CENTER_CCR); // center on boot

    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start_IT(&htim2);

    while (1)
    {
        if (sample_flag)
        {
            sample_flag = 0;

            switch (system_state)
            {
                case STATE_IDLE:
                    system_state = STATE_SAMPLING;
                    break;

                case STATE_SAMPLING:
                {
                    float measured = 0.0f;

#if USE_SIM_PITCH
                    /* Simulated pitch: sine wave ±20° over ~60 ticks (6 seconds)
                     * sim_tick increments each 100ms.
                     * sinf argument in radians: full cycle = 2π over 60 ticks */
                    sim_tick++;
                    sim_pitch = 20.0f * sinf(2.0f * M_PI * sim_tick / 60.0f);
                    measured = sim_pitch;
#else
                    /* Real MPU path */
                    MPU6050_Read();
                    if (ax_g == 0.0f && ay_g == 0.0f && az_g == 0.0f)
                    {
                        system_state = STATE_ERROR;
                        break;
                    }
                    ComplementaryFilter_Update();
                    measured = pitch;
#endif

                    // Run PID — output in [-90°, +90°]
                    pid_output = PID_Update(measured);

                    // Drive servo
                    Servo_SetAngle(pid_output);

                    system_state = STATE_TRANSMITTING;
                    break;
                }

                case STATE_TRANSMITTING:
                    // No UART adapter — nothing to transmit physically.
                    // pid_output, sim_pitch, pitch visible in Live Expressions.
                    system_state = STATE_SAMPLING;
                    break;

                case STATE_ERROR:
                    // Servo returns to center on error
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SERVO_CENTER_CCR);
                    pid_integral   = 0.0f;
                    pid_prev_error = 0.0f;
                    system_state = STATE_SAMPLING;
                    break;
            }
        }
    }
}

/* =============================================================================
 * TIM2 ISR — 100ms heartbeat
 * Only sets sample_flag. No I2C here — HAL I2C needs SysTick which can't
 * run inside a higher-priority ISR.
 * ============================================================================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        system_tick_ms += 100;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        sample_flag = 1;
    }
    // TIM1 does not use period elapsed callback — PWM runs autonomously
}

/* =============================================================================
 * EXTI CALLBACK — HC-SR04 ECHO (skipped, kept for completeness)
 * ============================================================================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_9)
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
            echo_start = __HAL_TIM_GET_COUNTER(&htim3);
        else
        {
            echo_end  = __HAL_TIM_GET_COUNTER(&htim3);
            echo_done = 1;
        }
    }
}

/* =============================================================================
 * SYSTEM CLOCK — HSI → PLL → 100MHz (unchanged from Day 1)
 * ============================================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 200;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

/* =============================================================================
 * GPIO INIT
 * PA8 repurposed from HC-SR04 TRIG to TIM1_CH1 PWM output for SG90.
 * PA8 alternate function = AF1 (TIM1_CH1 on STM32F411).
 * All other pins unchanged from Day 6.
 * ============================================================================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB6 (SCL) and PB7 (SDA) — I2C1 */
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PA8 — TIM1_CH1 PWM output for SG90 servo
     * AF1 = TIM1/TIM2 on STM32F411 (see datasheet Table 9)
     * Push-pull, no pull resistor needed for PWM */
    GPIO_InitStruct.Pin       = GPIO_PIN_8;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW; // PWM at 50Hz, low speed is fine
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA9 — HC-SR04 ECHO EXTI (skipped — no sensor, EXTI not enabled) */
    // Not configured — floating pin would crash MCU if EXTI enabled

    /* PC13 — onboard LED, active LOW */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitStruct.Pin       = GPIO_PIN_13;
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* =============================================================================
 * TIM1 INIT — 50Hz PWM for SG90 on PA8 (TIM1_CH1)
 *
 * TIM1 is on APB2 = 100MHz.
 * Prescaler = 99  → TIM1 clock = 100MHz / (99+1) = 1MHz → 1μs per tick
 * Period    = 19999 → PWM period = (19999+1) × 1μs = 20ms = 50Hz ✓
 *
 * CCR (pulse width in μs):
 *   1000 = 1.0ms = 0°
 *   1500 = 1.5ms = 90° (center)
 *   2000 = 2.0ms = 180°
 * ============================================================================= */
static void MX_TIM1_Init(void)
{
	__HAL_RCC_TIM1_CLK_ENABLE();
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 99;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 19999;  // 20ms period
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    /* PWM Mode 1: output HIGH while counter < CCR, LOW otherwise.
     * This gives active-high pulse = correct polarity for SG90. */
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = SERVO_CENTER_CCR; // start at center
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

    /* TIM1 is an advanced timer — requires MOE (Main Output Enable) bit.
     * sBreakDeadTimeConfig sets this. Without it, PWM output stays silent. */
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE; // enables MOE
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

    // No NVIC for TIM1 — PWM runs in hardware, no ISR needed
}

/* =============================================================================
 * TIM2 INIT — 100ms interrupt (unchanged)
 * ============================================================================= */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 4999;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* =============================================================================
 * TIM3 INIT — free-running 1MHz counter (unchanged)
 * ============================================================================= */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 99;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
}

/* =============================================================================
 * I2C1 INIT (unchanged)
 * ============================================================================= */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 50000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* =============================================================================
 * USART2 INIT (unchanged — no adapter, kept for completeness)
 * ============================================================================= */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* =============================================================================
 * ERROR HANDLER
 * ============================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
