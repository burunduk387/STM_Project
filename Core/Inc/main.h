/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC_Pin GPIO_PIN_13
#define PC_GPIO_Port GPIOC
#define FR_Pin GPIO_PIN_2
#define FR_GPIO_Port GPIOA
#define FL_Pin GPIO_PIN_3
#define FL_GPIO_Port GPIOA
#define ENC2_B_Pin GPIO_PIN_4
#define ENC2_B_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_5
#define ENC1_B_GPIO_Port GPIOA
#define ENC2_CH1_Pin GPIO_PIN_6
#define ENC2_CH1_GPIO_Port GPIOA
#define ENC2_CH2_Pin GPIO_PIN_7
#define ENC2_CH2_GPIO_Port GPIOA
#define RE4_Pin GPIO_PIN_0
#define RE4_GPIO_Port GPIOB
#define LE4_Pin GPIO_PIN_1
#define LE4_GPIO_Port GPIOB
#define RE3_Pin GPIO_PIN_2
#define RE3_GPIO_Port GPIOB
#define LE3_Pin GPIO_PIN_10
#define LE3_GPIO_Port GPIOB
#define RE2_Pin GPIO_PIN_11
#define RE2_GPIO_Port GPIOB
#define LE2_Pin GPIO_PIN_12
#define LE2_GPIO_Port GPIOB
#define RE1_Pin GPIO_PIN_13
#define RE1_GPIO_Port GPIOB
#define LE1_Pin GPIO_PIN_14
#define LE1_GPIO_Port GPIOB
#define ST5_Pin GPIO_PIN_15
#define ST5_GPIO_Port GPIOB
#define DIR5_Pin GPIO_PIN_8
#define DIR5_GPIO_Port GPIOA
#define ST4_Pin GPIO_PIN_9
#define ST4_GPIO_Port GPIOA
#define DIR4_Pin GPIO_PIN_10
#define DIR4_GPIO_Port GPIOA
#define ENC1_CH1_Pin GPIO_PIN_15
#define ENC1_CH1_GPIO_Port GPIOA
#define ENC2_CH2B3_Pin GPIO_PIN_3
#define ENC2_CH2B3_GPIO_Port GPIOB
#define ST3_Pin GPIO_PIN_4
#define ST3_GPIO_Port GPIOB
#define DIR3_Pin GPIO_PIN_5
#define DIR3_GPIO_Port GPIOB
#define ST2_Pin GPIO_PIN_6
#define ST2_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_7
#define DIR2_GPIO_Port GPIOB
#define ST1_Pin GPIO_PIN_8
#define ST1_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_9
#define DIR1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
