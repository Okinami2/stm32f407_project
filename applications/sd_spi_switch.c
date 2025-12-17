/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-12-16     GreatMagicianGarfiel       the first version
 */

#include "time_service.h"
#include "stm32f4xx_hal.h"
#include <rtthread.h>
#include "board.h"

extern UART_HandleTypeDef huart5;
extern SPI_HandleTypeDef hspi3;

typedef enum { MODE_SAFE_HIZ, MODE_GNSS, MODE_SD_SPI } mux_mode_t;

static void _set_mux_mode(mux_mode_t mode) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (mode == MODE_SAFE_HIZ) {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    } else if (mode == MODE_GNSS) {
        rt_pin_write(BSP_GNSS_SW_PIN, PIN_HIGH);
        // PC12 -> UART5_TX
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        // PD2 -> UART5_RX
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        __HAL_UART_CLEAR_PEFLAG(&huart5);
        __HAL_UART_CLEAR_FEFLAG(&huart5);
        __HAL_UART_CLEAR_NEFLAG(&huart5);
        __HAL_UART_CLEAR_OREFLAG(&huart5);
        __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
    } else if (mode == MODE_SD_SPI) {
        rt_pin_write(BSP_GNSS_SW_PIN, PIN_LOW);
        // PC12 -> SPI3_MOSI
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        // PD2 -> GPIO Output (CS)
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = 0;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    }
}


void ts_spi_bus_claim(void) {
    rt_base_t level = rt_hw_interrupt_disable();
    __HAL_UART_DISABLE_IT(&huart5, UART_IT_RXNE);
    _set_mux_mode(MODE_SAFE_HIZ);
    _set_mux_mode(MODE_SD_SPI);
    rt_hw_interrupt_enable(level);
}

void ts_spi_bus_release(void) {
    rt_base_t level = rt_hw_interrupt_disable();
    _set_mux_mode(MODE_SAFE_HIZ);
    _set_mux_mode(MODE_GNSS);
    rt_hw_interrupt_enable(level);
}

