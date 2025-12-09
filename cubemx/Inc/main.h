/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board.h"
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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C_3_8_EN_Pin GPIO_PIN_2
#define I2C_3_8_EN_GPIO_Port GPIOE
#define ADCCLK_BIT0_Pin GPIO_PIN_4
#define ADCCLK_BIT0_GPIO_Port GPIOE
#define ADCCLK_BIT1_Pin GPIO_PIN_5
#define ADCCLK_BIT1_GPIO_Port GPIOE
#define nADC_SYNC_Pin GPIO_PIN_6
#define nADC_SYNC_GPIO_Port GPIOE
#define ADC_DOUT_Pin GPIO_PIN_2
#define ADC_DOUT_GPIO_Port GPIOC
#define ADC_DIN_Pin GPIO_PIN_3
#define ADC_DIN_GPIO_Port GPIOC
#define VCC_N_15V_EN_Pin GPIO_PIN_3
#define VCC_N_15V_EN_GPIO_Port GPIOA
#define VCC_5V_EN_Pin GPIO_PIN_4
#define VCC_5V_EN_GPIO_Port GPIOA
#define VCC_P_15V_EN_Pin GPIO_PIN_11
#define VCC_P_15V_EN_GPIO_Port GPIOF
#define nCH5_ALERT_Pin GPIO_PIN_0
#define nCH5_ALERT_GPIO_Port GPIOG
#define nCH7_ALERT_Pin GPIO_PIN_1
#define nCH7_ALERT_GPIO_Port GPIOG
#define nCH0_ALERT_Pin GPIO_PIN_7
#define nCH0_ALERT_GPIO_Port GPIOE
#define TFPWR_EN_Pin GPIO_PIN_8
#define TFPWR_EN_GPIO_Port GPIOE
#define nCH1_ALERT_Pin GPIO_PIN_9
#define nCH1_ALERT_GPIO_Port GPIOE
#define nCH2_ALERT_Pin GPIO_PIN_11
#define nCH2_ALERT_GPIO_Port GPIOE
#define ETHPWR_EN_Pin GPIO_PIN_12
#define ETHPWR_EN_GPIO_Port GPIOE
#define nCH3_ALERT_Pin GPIO_PIN_13
#define nCH3_ALERT_GPIO_Port GPIOE
#define nETH_RST_Pin GPIO_PIN_14
#define nETH_RST_GPIO_Port GPIOE
#define nCH4_ALERT_Pin GPIO_PIN_15
#define nCH4_ALERT_GPIO_Port GPIOE
#define ADC_CLK_Pin GPIO_PIN_10
#define ADC_CLK_GPIO_Port GPIOB
#define GNSS_SW_Pin GPIO_PIN_3
#define GNSS_SW_GPIO_Port GPIOG
#define ADCPWR_EN_Pin GPIO_PIN_6
#define ADCPWR_EN_GPIO_Port GPIOC
#define nETH_INT_Pin GPIO_PIN_8
#define nETH_INT_GPIO_Port GPIOA
#define I2C_3_8_Bit1_Pin GPIO_PIN_1
#define I2C_3_8_Bit1_GPIO_Port GPIOD
#define SD_CS_Pin GPIO_PIN_2
#define SD_CS_GPIO_Port GPIOD
#define I2C_3_8_Bit2_Pin GPIO_PIN_4
#define I2C_3_8_Bit2_GPIO_Port GPIOD
#define RFMODPWR_EN_Pin GPIO_PIN_10
#define RFMODPWR_EN_GPIO_Port GPIOG
#define I2C_3_8_Bit3_Pin GPIO_PIN_3
#define I2C_3_8_Bit3_GPIO_Port GPIOB
#define nADC_CS_Pin GPIO_PIN_6
#define nADC_CS_GPIO_Port GPIOB
#define ANPWR_EN_Pin GPIO_PIN_7
#define ANPWR_EN_GPIO_Port GPIOB
#define ADCCLK_EN_Pin GPIO_PIN_0
#define ADCCLK_EN_GPIO_Port GPIOE
#define nADC_DRDY_Pin GPIO_PIN_1
#define nADC_DRDY_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
