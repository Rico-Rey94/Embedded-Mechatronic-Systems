#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"   // HAL header for STM32F4 series

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Peripheral handles --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart3; // Use huart2 if changing to USART2

/* USER CODE BEGIN EFP */
/* Place your additional exported functions here if needed */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
