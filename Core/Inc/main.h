/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h7xx_hal.h"

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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POP_CYCLE_DELAY 31250
#define MICROWAVE_WIDTH 1563
#define PROBE_WIDTH 625
#define MICROWAVE_DELAY 312
#define PUMP_WIDTH 1563
#define RAMSEY_TIME 12500
#define ATT_4_Pin GPIO_PIN_2
#define ATT_4_GPIO_Port GPIOE
#define ATT_8_Pin GPIO_PIN_4
#define ATT_8_GPIO_Port GPIOE
#define ATT_16_Pin GPIO_PIN_5
#define ATT_16_GPIO_Port GPIOE
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define SCOPE_TRIG_OUT_Pin GPIO_PIN_0
#define SCOPE_TRIG_OUT_GPIO_Port GPIOG
#define SPARE_SMA_Pin GPIO_PIN_12
#define SPARE_SMA_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define USB_OTG_FS_OVCR_Pin GPIO_PIN_7
#define USB_OTG_FS_OVCR_GPIO_Port GPIOG
#define SCLK_Pin GPIO_PIN_8
#define SCLK_GPIO_Port GPIOC
#define MOSI_Pin GPIO_PIN_9
#define MOSI_GPIO_Port GPIOC
#define MISO_Pin GPIO_PIN_10
#define MISO_GPIO_Port GPIOC
#define SEN_Pin GPIO_PIN_11
#define SEN_GPIO_Port GPIOC
#define REG_EN_Pin GPIO_PIN_12
#define REG_EN_GPIO_Port GPIOC
#define ATT_2_Pin GPIO_PIN_3
#define ATT_2_GPIO_Port GPIOD
#define ATT_1_Pin GPIO_PIN_4
#define ATT_1_GPIO_Port GPIOD
#define ATT_05_Pin GPIO_PIN_5
#define ATT_05_GPIO_Port GPIOD
#define ATT_025_Pin GPIO_PIN_6
#define ATT_025_GPIO_Port GPIOD
#define ATT_LE_Pin GPIO_PIN_7
#define ATT_LE_GPIO_Port GPIOD
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
