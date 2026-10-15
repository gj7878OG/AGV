/**
  ******************************************************************************
  * @file    main.h
  * @brief   AGV 4 Motor firmware - main header
  *
  * Created by hand to replace the auto-generated STM32CubeIDE version.
  * Pulls in the full HAL, then declares the prototypes used by main.c.
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
