/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    : agv_v3.c  (single-file build — drop into a CubeIDE project)
  * @brief   : AGV 4 Motor — ACKs + safety + encoder telemetry + non-blocking spin
  *
  *  SINGLE-FILE BUILD INSTRUCTIONS (STM32CubeIDE)
  *  ──────────────────────────────────────────────
  *  1. File → New → STM32 Project → STM32F411RE (NUCLEO-F411RE) → Finish
  *     CubeIDE auto-generates stm32f4xx_hal_conf.h, main.h, startup_stm32f411xe.s
  *     and the .ld linker script. We do NOT replace any of those.
  *  2. Delete the auto-generated Src/main.c (right-click → Delete).
  *  3. Drag this file into the project's Src/ folder. Tell CubeIDE to
  *     "Copy files" + "Overwrite without warning" if it asks.
  *  4. Ctrl+B to build. Sizes should match: text=7812 data=16 bss=1880.
  *  5. F11 to flash via ST-Link (SWD).
  *
  *  This file consolidates what was previously:
  *    - main_v3.c
  *    - main.h               (prototypes inlined at top)
  *    - stm32f4xx_hal_msp.c  (HAL_MspInit / HAL_MspDeInit / HAL_TIM_MspPostInit)
  *
  *  The HAL still needs stm32f4xx_hal_conf.h, but CubeIDE auto-generates
  *  it on project creation. If you are NOT using CubeIDE (e.g. plain arm-gcc
  *  + Makefile), copy stm32f4xx_hal_conf.h from STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Inc
  *  next to this file.
  *
  * ── Protocol summary ──
  *   Jetson → STM32   1-byte command  (F B L R S X A 1..5)
  *   STM32  → Jetson  'A'             after every command (ACK)
  *   STM32  → Jetson  'E' L0 L1 R0 R1 every 100 ms (encoders, LE int16)
  *   STM32  → Jetson  'V' '3' '\n'    once on boot
  *
  * ── Direction / speed bytes ──
  *   F=Forward B=Reverse L=TurnLeft R=TurnRight X=SpinCW A=SpinACW S=Stop
  *   1=20% 2=40% 3=60% 4=80% 5=100%
  *
  * ── Pin map ──
  *   PA0  → TIM2_CH1 PWM → SmartElex 1 (Right)
  *   PA1  → TIM2_CH2 PWM → SmartElex 2 (Left)
  *   PB0  → DIR1          → SmartElex 1 DIR
  *   PB1  → DIR2          → SmartElex 2 DIR
  *   PA9  → USART1_TX     → Jetson RX (Pin 8)
  *   PA10 → USART1_RX     → Jetson TX (Pin 10)
  *   PA6  → TIM3_CH1      → Right encoder A
  *   PA7  → TIM3_CH2      → Right encoder B
  *   PB6  → TIM4_CH1      → Left encoder A
  *   PB7  → TIM4_CH2      → Left encoder B
  *
  * ── v3 features ──
  *   1. Boot version message "V3\n" on power-up. The Jetson reads this
  *      to confirm the new firmware is loaded.
  *   2. Encoder telemetry: every 100 ms, send 'E' <left16_le> <right16_le>.
  *      The Jetson uses these to display real wheel motion on the dashboard.
  *   3. Non-blocking spin: the 2.5 s HAL_Delay inside the spin is replaced
  *      by a small state machine. The main loop services the spin in 10 ms
  *      slices, so 'S' interrupts a spin within 10 ms and the safety
  *      timeout fires correctly during a spin.
  *
  * ── v2 features retained ──
  *   - ACKs ('A' byte) after every command — keeps the Jetson watchdog
  *     happy.
  *   - 500 ms safety timeout: if no UART command in 500 ms, motors stop.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stm32f4xx_hal.h"
#include "string.h"

/* ── Prototypes (inlined from main.h) ── */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);

/* ── MSP stubs (inlined from stm32f4xx_hal_msp.c) ──
 * CubeIDE normally auto-generates these; we keep the bodies here so the
 * single-file build is self-contained. HAL_MspPostInit is the only one
 * with non-trivial work — it routes TIM2_CH1/CH2 to PA0/PA1.
 */
void HAL_MspInit(void)
{
    /* CubeMX normally places low-level MCU init here (none for this app). */
}

void HAL_MspDeInit(void)
{
    /* Nothing to undo. */
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (htim->Instance == TIM2)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* ── Handles (CubeMX would normally declare these in main.c) ── */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN 0 */

/* ── Current speed (default 60%) ── */
uint16_t current_speed = 600;

/* ── Spin tuning ── */
#define SPIN_TIME_MS       1500
#define SPIN_DELAY_MS      1000

/* ── Safety timeout ── */
#define SAFETY_TIMEOUT_MS  500

/* ── Encoder telemetry cadence ── */
#define TELEMETRY_INTERVAL_MS  100

/* ── Time of last received UART command (for safety timeout) ── */
static uint32_t last_cmd_tick = 0;

/* ── Time of last encoder telemetry frame ── */
static uint32_t last_telem_tick = 0;

/* ── Latched so we only enter Motor_Stop() once per timeout ── */
static uint8_t  safety_timed_out = 0;

/* ── v3: non-blocking spin state machine ──
 * IDLE     - no spin in progress, no movement commanded
 * ACTIVE   - motors are driving the spin, count down SPIN_TIME_MS
 * COOLDOWN - motors are stopped, count down SPIN_DELAY_MS before re-arming
 */
typedef enum { SPIN_IDLE = 0, SPIN_ACTIVE, SPIN_COOLDOWN } spin_state_t;
static volatile spin_state_t spin_state   = SPIN_IDLE;
static uint8_t  spin_dir     = 0;
static uint32_t spin_phase_start = 0;

/* ── Helpers ── */

static void send_ack(void)
{
    uint8_t ack = 'A';
    HAL_UART_Transmit(&huart1, &ack, 1, 10);
}

static void send_boot_version(void)
{
    /* "V3\n" — 3 bytes. Tells the Jetson the new firmware is alive
     * without waiting for a command. */
    HAL_UART_Transmit(&huart1, (uint8_t *)"V3\n", 3, 100);
}

static void send_encoder_telemetry(void)
{
    int16_t left_count  = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t right_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    uint8_t buf[5];
    buf[0] = 'E';
    buf[1] = (uint8_t)(left_count  & 0xFF);
    buf[2] = (uint8_t)((left_count  >> 8) & 0xFF);
    buf[3] = (uint8_t)(right_count & 0xFF);
    buf[4] = (uint8_t)((right_count >> 8) & 0xFF);
    HAL_UART_Transmit(&huart1, buf, 5, 10);
}

/* Spin helpers — operate on the state machine, no blocking delays. */
static void spin_abort(void)
{
    Motor_Stop();
    spin_state = SPIN_IDLE;
}

static void spin_start(uint8_t is_cw)
{
    if (spin_state != SPIN_IDLE) return;   /* ignore if already in motion */
    if (is_cw) {
        Motor_Right(current_speed, 1);
        Motor_Left(current_speed, 0);
    } else {
        Motor_Right(current_speed, 0);
        Motor_Left(current_speed, 1);
    }
    spin_dir         = is_cw;
    spin_phase_start = HAL_GetTick();
    spin_state       = SPIN_ACTIVE;
}

static void spin_service(void)
{
    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - spin_phase_start;
    switch (spin_state)
    {
        case SPIN_IDLE:
            break;
        case SPIN_ACTIVE:
            if (elapsed >= SPIN_TIME_MS) {
                Motor_Stop();
                spin_phase_start = now;
                spin_state       = SPIN_COOLDOWN;
            }
            break;
        case SPIN_COOLDOWN:
            if (elapsed >= SPIN_DELAY_MS) {
                spin_state = SPIN_IDLE;
            }
            break;
    }
}

/* ── Motor Control ── */
void Motor_Right(uint16_t speed, uint8_t dir)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,
        dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    TIM2->CCR1 = speed;
}

void Motor_Left(uint16_t speed, uint8_t dir)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,
        dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    TIM2->CCR2 = speed;
}

void Motor_Stop(void)
{
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
}

void AGV_Forward(void)
{
    Motor_Right(current_speed, 1);
    Motor_Left(current_speed, 1);
}

void AGV_Reverse(void)
{
    Motor_Right(current_speed, 0);
    Motor_Left(current_speed, 0);
}

void AGV_Turn_Left(void)
{
    Motor_Right(current_speed / 3, 1);   /* Right slow */
    Motor_Left(current_speed, 1);        /* Left fast  */
}

void AGV_Turn_Right(void)
{
    Motor_Right(current_speed, 1);       /* Right fast */
    Motor_Left(current_speed / 3, 1);    /* Left slow  */
}

/* v3 note: AGV_Spin_CW_Once / AGV_Spin_ACW_Once are no longer called from
 * the main loop. The non-blocking state machine in spin_service() replaces
 * them. We keep the functions in case you want to re-introduce a blocking
 * spin from a debug command later.
 */
void AGV_Spin_CW_Once(void)
{
    Motor_Right(current_speed, 1);
    Motor_Left(current_speed, 0);
    HAL_Delay(SPIN_TIME_MS);
    Motor_Stop();
    HAL_Delay(SPIN_DELAY_MS);
}

void AGV_Spin_ACW_Once(void)
{
    Motor_Right(current_speed, 0);
    Motor_Left(current_speed, 1);
    HAL_Delay(SPIN_TIME_MS);
    Motor_Stop();
    HAL_Delay(SPIN_DELAY_MS);
}

void Execute_Command(char cmd)
{
    switch (cmd)
    {
        case 'F': AGV_Forward();    break;
        case 'B': AGV_Reverse();    break;
        case 'L': AGV_Turn_Left();  break;
        case 'R': AGV_Turn_Right(); break;
        case 'S': Motor_Stop();     break;

        case '1': current_speed = 200; break;
        case '2': current_speed = 400; break;
        case '3': current_speed = 600; break;
        case '4': current_speed = 800; break;
        case '5': current_speed = 999; break;

        default: break;
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    Motor_Stop();

    /* Arm the safety timeout so the AGV is silent for 500 ms after boot,
     * giving the Jetson time to connect before we start trusting UART. */
    last_cmd_tick = HAL_GetTick();

    /* Announce ourselves on the UART. If the Jetson sees "V3\n", it
     * knows the new firmware is alive. */
    send_boot_version();
    last_telem_tick = HAL_GetTick();

    uint8_t rx_byte = 0;

    while (1)
    {
        /* ── Receive one command byte (10 ms poll) ── */
        if (HAL_UART_Receive(&huart1, &rx_byte, 1, 10) == HAL_OK)
        {
            char cmd = (char)rx_byte;

            /* Any movement command (including 'S') aborts an in-progress
             * spin. This is what makes 'S' work mid-spin. */
            if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R' ||
                cmd == 'S' || cmd == 'X' || cmd == 'A')
            {
                spin_abort();
            }

            Execute_Command(cmd);

            /* X / A: kick off the non-blocking spin. */
            if (cmd == 'X')      spin_start(1);
            else if (cmd == 'A') spin_start(0);

            send_ack();
            last_cmd_tick     = HAL_GetTick();
            safety_timed_out  = 0;
        }

        /* ── Safety timeout ── */
        if ((HAL_GetTick() - last_cmd_tick) > SAFETY_TIMEOUT_MS)
        {
            if (!safety_timed_out)
            {
                Motor_Stop();
                spin_abort();
                safety_timed_out = 1;
            }
        }

        /* ── Service the spin state machine (non-blocking) ── */
        spin_service();

        /* ── Encoder telemetry (every TELEMETRY_INTERVAL_MS) ── */
        if ((HAL_GetTick() - last_telem_tick) >= TELEMETRY_INTERVAL_MS)
        {
            send_encoder_telemetry();
            last_telem_tick = HAL_GetTick();
        }
    }
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 83;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim3.Instance               = TIM3;
    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 5;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 5;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim4.Instance               = TIM4;
    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 5;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 5;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM           = 8;
    RCC_OscInitStruct.PLL.PLLN           = 84;
    RCC_OscInitStruct.PLL.PLLP           = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ           = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
