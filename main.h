#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

    void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
    void Error_Handler(void);

    /* USER CODE BEGIN EFP */
    void Motor_Right(uint16_t speed, uint8_t dir);
    void Motor_Left(uint16_t speed, uint8_t dir);
    void Motor_Stop(void);
    void AGV_Forward(void);
    void AGV_Slow_Forward(void);
    void AGV_Reverse(void);
    void AGV_Turn_Left(void);
    void AGV_Turn_Right(void);
    void AGV_Spin(void);
    void Execute_Command(char cmd);
    /* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
