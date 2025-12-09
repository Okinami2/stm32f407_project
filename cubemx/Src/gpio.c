/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, I2C_3_8_EN_Pin|ADCCLK_BIT0_Pin|ADCCLK_BIT1_Pin|nADC_SYNC_Pin
                          |TFPWR_EN_Pin|ETHPWR_EN_Pin|ADCCLK_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, VCC_N_15V_EN_Pin|VCC_5V_EN_Pin|nETH_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VCC_P_15V_EN_GPIO_Port, VCC_P_15V_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nETH_RST_GPIO_Port, nETH_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GNSS_SW_GPIO_Port, GNSS_SW_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADCPWR_EN_GPIO_Port, ADCPWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, I2C_3_8_Bit1_Pin|I2C_3_8_Bit2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFMODPWR_EN_GPIO_Port, RFMODPWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, I2C_3_8_Bit3_Pin|nADC_CS_Pin|ANPWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : I2C_3_8_EN_Pin ADCCLK_BIT0_Pin ADCCLK_BIT1_Pin nADC_SYNC_Pin
                           ADCCLK_EN_Pin */
  GPIO_InitStruct.Pin = I2C_3_8_EN_Pin|ADCCLK_BIT0_Pin|ADCCLK_BIT1_Pin|nADC_SYNC_Pin
                          |ADCCLK_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : VCC_N_15V_EN_Pin VCC_5V_EN_Pin nETH_INT_Pin */
  GPIO_InitStruct.Pin = VCC_N_15V_EN_Pin|VCC_5V_EN_Pin|nETH_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VCC_P_15V_EN_Pin */
  GPIO_InitStruct.Pin = VCC_P_15V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VCC_P_15V_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nCH5_ALERT_Pin nCH7_ALERT_Pin */
  GPIO_InitStruct.Pin = nCH5_ALERT_Pin|nCH7_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : nCH0_ALERT_Pin nCH1_ALERT_Pin nCH2_ALERT_Pin nCH3_ALERT_Pin
                           nCH4_ALERT_Pin nADC_DRDY_Pin */
  GPIO_InitStruct.Pin = nCH0_ALERT_Pin|nCH1_ALERT_Pin|nCH2_ALERT_Pin|nCH3_ALERT_Pin
                          |nCH4_ALERT_Pin|nADC_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : TFPWR_EN_Pin ETHPWR_EN_Pin */
  GPIO_InitStruct.Pin = TFPWR_EN_Pin|ETHPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : nETH_RST_Pin */
  GPIO_InitStruct.Pin = nETH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(nETH_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GNSS_SW_Pin */
  GPIO_InitStruct.Pin = GNSS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GNSS_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADCPWR_EN_Pin */
  GPIO_InitStruct.Pin = ADCPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADCPWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C_3_8_Bit1_Pin I2C_3_8_Bit2_Pin */
  GPIO_InitStruct.Pin = I2C_3_8_Bit1_Pin|I2C_3_8_Bit2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RFMODPWR_EN_Pin */
  GPIO_InitStruct.Pin = RFMODPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFMODPWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C_3_8_Bit3_Pin nADC_CS_Pin ANPWR_EN_Pin */
  GPIO_InitStruct.Pin = I2C_3_8_Bit3_Pin|nADC_CS_Pin|ANPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
