/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    : main.c
 * @brief   : AGV 4 Motor — UART Command Control
 *            Jetson Nano UART → STM32 → Motors
 *
 * Commands: F=Forward B=Reverse L=TurnLeft R=TurnRight X=Spin S=Stop W=SlowForward
 *
 * PIN MAP:
 *   PA0  → TIM2_CH1 PWM → SmartElex 1 (Right)
 *   PA1  → TIM2_CH2 PWM → SmartElex 2 (Left)
 *   PB0  → DIR1          → SmartElex 1 DIR
 *   PB1  → DIR2          → SmartElex 2 DIR
 *   PA9  → USART1_TX     → Jetson Nano RX (Pin 10)
 *   PA10 → USART1_RX     → Jetson Nano TX (Pin 8)
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include "string.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN 0 */

/* ── Speed Definitions ── */
#define FULL_SPEED 999
#define HALF_SPEED 500
#define TURN_SPEED 600
#define SLOW_SPEED 300

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

/* ── AGV Movements ── */
void AGV_Forward(void)
{
    Motor_Right(FULL_SPEED, 1);
    Motor_Left(FULL_SPEED, 1);
}

void AGV_Slow_Forward(void)
{
    Motor_Right(SLOW_SPEED, 1);
    Motor_Left(SLOW_SPEED, 1);
}

void AGV_Reverse(void)
{
    Motor_Right(FULL_SPEED, 0);
    Motor_Left(FULL_SPEED, 0);
}

void AGV_Turn_Left(void)
{
    Motor_Right(TURN_SPEED, 1); /* Right fast */
    Motor_Left(SLOW_SPEED, 1);  /* Left slow  */
}

void AGV_Turn_Right(void)
{
    Motor_Right(SLOW_SPEED, 1); /* Right slow */
    Motor_Left(TURN_SPEED, 1);  /* Left fast  */
}

void AGV_Spin(void)
{
    Motor_Right(TURN_SPEED, 1); /* Right forward */
    Motor_Left(TURN_SPEED, 0);  /* Left reverse  */
}

/* ── Execute Command ── */
void Execute_Command(char cmd)
{
    switch (cmd)
    {
    case 'F':
        AGV_Forward();
        break;
    case 'W':
        AGV_Slow_Forward();
        break;
    case 'B':
        AGV_Reverse();
        break;
    case 'L':
        AGV_Turn_Left();
        break;
    case 'R':
        AGV_Turn_Right();
        break;
    case 'S':
        Motor_Stop();
        break;
    default:
        break;
    }
}

/* ── Acknowledge the most recent command back to the Jetson ── */
static inline void Send_Ack(void)
{
    uint8_t ack = 'A';
    HAL_UART_Transmit(&huart1, &ack, 1, 5);
}

/* ── Encoder telemetry packet:
 *     header 'E', then 4 bytes (left count, right count), big-endian int16.
 *     Lets the Jetson read both motor speeds without a second wire.
 * ── */
static inline void Send_Encoder_Telemetry(void)
{
    int16_t left  = (int16_t)TIM3->CNT;
    int16_t right = (int16_t)TIM4->CNT;
    uint8_t pkt[5];
    pkt[0] = 'E';
    pkt[1] = (uint8_t)((uint16_t)left  >> 8);
    pkt[2] = (uint8_t)((uint16_t)left  & 0xFF);
    pkt[3] = (uint8_t)((uint16_t)right >> 8);
    pkt[4] = (uint8_t)((uint16_t)right & 0xFF);
    HAL_UART_Transmit(&huart1, pkt, sizeof(pkt), 5);
}

/* USER CODE END 0 */

/* ================================================================
 * MAIN
 * ================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    Motor_Stop();

    uint8_t rx_byte = 0;
    uint32_t last_cmd_ms = HAL_GetTick();
    static uint8_t motors_stopped_by_watchdog = 0;

    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */

        /* Block up to 100 ms waiting for a command byte from the Jetson. */
        if (HAL_UART_Receive(&huart1, &rx_byte, 1, 100) == HAL_OK)
        {
            Execute_Command((char)rx_byte);
            last_cmd_ms = HAL_GetTick();
            motors_stopped_by_watchdog = 0;
            Send_Ack();

            /* Piggy-back the encoder snapshot on every ACK so the Jetson
             * gets a steady stream without us having to schedule it. */
            Send_Encoder_Telemetry();
        }

        /* Watchdog — if no command for > 1000 ms, force-stop the motors.
         * Prevents the AGV from running away if the Jetson stalls. */
        if ((HAL_GetTick() - last_cmd_ms) > 1000U && !motors_stopped_by_watchdog)
        {
            Motor_Stop();
            motors_stopped_by_watchdog = 1;
        }

        /* USER CODE END 3 */
    }
}

/* ================================================================
 * USART1 — 115200 baud, PA9=TX, PA10=RX
 * ================================================================ */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

/* ================================================================
 * TIM2 — PWM both drivers
 * ================================================================ */
static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
        Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        Error_Handler();
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
    HAL_TIM_MspPostInit(&htim2);
}

/* ================================================================
 * TIM3 — Encoder Motor 1
 * ================================================================ */
static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 5;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 5;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
        Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
        Error_Handler();
}

/* ================================================================
 * TIM4 — Encoder Motor 2
 * ================================================================ */
static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 5;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 5;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
        Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
        Error_Handler();
}

/* ================================================================
 * GPIO — PB0=DIR1, PB1=DIR2
 * ================================================================ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ================================================================
 * SYSTEM CLOCK — 84MHz HSI PLL
 * ================================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
