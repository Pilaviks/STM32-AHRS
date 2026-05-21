# STM32-AHRS

Miniature Attitude and Heading Reference System (AHRS) on STM32F411CEU6 (Black Pill), implemented in bare-metal C using STM32 HAL.

## Goals

- Build a real-time orientation estimation system on a resource-constrained ARM Cortex-M4
- Implement sensor fusion from scratch without external filter libraries
- Drive a physical actuator (servo) from PID output derived from IMU data
- Learn embedded systems development end-to-end: toolchain, clocks, peripherals, interrupts, debugging

## Challenges and Solutions

**1. ST-Link clone — no USB-UART bridge**
No serial monitor available at any point. All verification done exclusively through STM32CubeIDE Live Expressions debugger. Required careful choice of which variables to expose globally and watch in real time.

**2. HAL I2C inside ISR — MCU crash**
Early architecture put MPU-6050 reads inside the TIM2 interrupt callback. This caused a hard fault: HAL I2C uses SysTick-based timeouts, and SysTick cannot run inside a higher-priority ISR. Solution: ISR only sets `sample_flag = 1`. All I2C work happens in the main loop consuming that flag.

**3. MPU-6050 power-up timing**
WHO_AM_I register returned 0x00 on first attempts. Root cause: I2C read issued before sensor finished booting. Fixed with `HAL_Delay(100)` before the first register access.

**4. Floating EXTI pin — MCU crash**
HC-SR04 ECHO pin (PA9) configured with EXTI but sensor not connected. Floating input caused spurious interrupts and MCU lockup. Solution: do not enable EXTI on unconnected pins under any circumstances.

**5. HC-SR04 sensor failure**
Sensor did not respond despite correct wiring and code. Skipped after diagnosis — hardware fault confirmed. Code retained in codebase for completeness.

**6. TIM1 PWM silent — CCR1 stuck at 0**
SG90 servo not moving despite correct PID output. Root cause: TIM1 RCC clock not explicitly enabled before `HAL_TIM_PWM_Init`. Added `__HAL_RCC_TIM1_CLK_ENABLE()` at the top of `MX_TIM1_Init`. CCR1 immediately showed correct values after fix.

**7. TIM1 advanced timer — MOE bit**
TIM1 is an advanced-control timer. Unlike general-purpose timers, it requires the Main Output Enable (MOE) bit set or PWM output stays electrically silent. Fixed by calling `HAL_TIMEx_ConfigBreakDeadTime` with `AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE`.

**8. PID clamp sign bug**
`pid_output` was stuck at +90° regardless of `sim_pitch`. Root cause: negative output clamp written as `if (output < -PID_OUT_MIN)` — since `PID_OUT_MIN = -90.0f`, this evaluated to `if (output < +90.0f)`, clamping everything to +90. Fixed to `if (output < PID_OUT_MIN)`.

**9. Loose GND — servo not moving**
CCR1 showed correct changing values, PWM signal confirmed present, but servo did not move. Root cause: loose GND wire on breadboard — servo brown wire not making reliable contact. Physical reseating fixed it immediately.

**10. PA8 pin repurposed**
PA8 originally allocated as HC-SR04 TRIG output. After HC-SR04 was skipped, PA8 was repurposed as TIM1_CH1 PWM output for the servo. GPIO reconfigured from `GPIO_MODE_OUTPUT_PP` to `GPIO_MODE_AF_PP` with `GPIO_AF1_TIM1`.

## Hardware

| Component | Details |
|-----------|---------|
| MCU | STM32F411CEU6 (Black Pill, ARM Cortex-M4 @ 100MHz) |
| IMU | MPU-6050 (I2C, PB6=SCL, PB7=SDA) |
| Servo | Tower Pro SG90 (PWM, PA8) |
| Debugger | ST-Link V2 (SWD) |

## Features

- **Interrupt-driven architecture** — TIM2 fires every 100ms, sets `sample_flag`; all I2C work in main loop
- **MPU-6050 sensor fusion** — 14-byte burst read, accelerometer + gyroscope complementary filter (α=0.98)
- **Complementary filter** — blends gyroscope integration with accelerometer atan2 for pitch and roll
- **4-state state machine** — IDLE → SAMPLING → TRANSMITTING → ERROR
- **PID controller** — Kp/Ki/Kd with integral windup clamp
- **SG90 servo** — TIM1 PWM at 50Hz, PID output mapped to pulse width

## Pin Assignment

| Pin | Function |
|-----|----------|
| PB6 | I2C1 SCL |
| PB7 | I2C1 SDA |
| PA8 | TIM1_CH1 PWM → SG90 signal |
| PC13 | Onboard LED (active LOW, heartbeat) |
| PA2 | USART2 TX |
| PA3 | USART2 RX |

## Clock Configuration

- HSI 16MHz → PLL → SYSCLK 100MHz
- APB1 = 50MHz (TIM2, TIM3, I2C1)
- APB2 = 100MHz (TIM1)
- TIM1: Prescaler=99, Period=19999 → 50Hz PWM
- TIM2: Prescaler=4999, Period=999 → 100ms interrupt
- TIM3: Prescaler=99 → 1MHz free-running counter

## PID Parameters

| Parameter | Value |
|-----------|-------|
| Setpoint | 0° (level) |
| Kp | 2.0 |
| Ki | 0.05 |
| Kd | 0.5 |
| Output clamp | ±90° |
| Integral windup clamp | ±50° |

## Build

STM32CubeIDE. Import project, build, flash via ST-Link V2 (SWD).

## Verification

All values verified via STM32CubeIDE Live Expressions (no USB-UART adapter available):
- `sim_pitch` — simulated sine-wave pitch input (±20°, 6s period)
- `pid_output` — PID response (±90° clamped)
- `htim1.Instance->CCR1` — live PWM pulse width (1000–2000μs)
- `system_state` — state machine position
- `system_tick_ms` — elapsed time confirmation
