/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

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
#define SD_CS_Pin GPIO_PIN_13
#define SD_CS_GPIO_Port GPIOC
#define TS_PENIRQ_Pin GPIO_PIN_14
#define TS_PENIRQ_GPIO_Port GPIOC
#define TS_PENIRQ_EXTI_IRQn EXTI4_15_IRQn
#define SD_TAKILDI_Pin GPIO_PIN_15
#define SD_TAKILDI_GPIO_Port GPIOC
#define TS_DOUT_Pin GPIO_PIN_0
#define TS_DOUT_GPIO_Port GPIOA
#define TS_DIN_Pin GPIO_PIN_1
#define TS_DIN_GPIO_Port GPIOA
#define TS_CS_Pin GPIO_PIN_2
#define TS_CS_GPIO_Port GPIOA
#define TS_DCLCK_Pin GPIO_PIN_3
#define TS_DCLCK_GPIO_Port GPIOA
#define W25Q_CS_Pin GPIO_PIN_4
#define W25Q_CS_GPIO_Port GPIOA
#define LCD_BACKLIGHT_Pin GPIO_PIN_8
#define LCD_BACKLIGHT_GPIO_Port GPIOA
#define LCD_RD_Pin GPIO_PIN_11
#define LCD_RD_GPIO_Port GPIOA
#define LCD_WR_Pin GPIO_PIN_12
#define LCD_WR_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_6
#define LCD_CS_GPIO_Port GPIOF
#define LCD_RES_Pin GPIO_PIN_7
#define LCD_RES_GPIO_Port GPIOF
#define LCD_CD_Pin GPIO_PIN_15
#define LCD_CD_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

void MX_SPI1_Init(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
