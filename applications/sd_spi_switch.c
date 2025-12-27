#include "sd_spi_switch.h"
#include "time_service.h"
#include "stm32f4xx_hal.h"
#include <rtthread.h>
#include "board.h"


#define DBG_TAG "sd_spi_switch"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

extern void gnss_uart_suspend(void);
extern void gnss_uart_resume(void);

typedef enum { MODE_SAFE_HIZ, MODE_GNSS, MODE_SD_SPI } mux_mode_t;

static struct rt_mutex g_sd_spi_mux_mutex;
static rt_bool_t g_sd_spi_mux_mutex_inited = RT_FALSE;

static void _ensure_mux_mutex_inited(void)
{
    if (g_sd_spi_mux_mutex_inited) return;

    rt_base_t level = rt_hw_interrupt_disable();
    if (!g_sd_spi_mux_mutex_inited)
    {
        (void)rt_mutex_init(&g_sd_spi_mux_mutex, "spi_mux", RT_IPC_FLAG_PRIO);
        g_sd_spi_mux_mutex_inited = RT_TRUE;
    }
    rt_hw_interrupt_enable(level);
}

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
    _ensure_mux_mutex_inited();
    if (rt_mutex_take(&g_sd_spi_mux_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        rt_kprintf("[sd_spi_switch] mutex take failed\n");
        return;
    }
    gnss_uart_suspend();

    rt_base_t level = rt_hw_interrupt_disable();

    _set_mux_mode(MODE_SAFE_HIZ);
    _set_mux_mode(MODE_SD_SPI);
    rt_hw_interrupt_enable(level);
}

void ts_spi_bus_release(void) {
    _ensure_mux_mutex_inited();

    rt_base_t level = rt_hw_interrupt_disable();

    _set_mux_mode(MODE_SAFE_HIZ);
    _set_mux_mode(MODE_GNSS);
    rt_hw_interrupt_enable(level);

    gnss_uart_resume();

    if (rt_mutex_release(&g_sd_spi_mux_mutex) != RT_EOK)
    {
        rt_kprintf("[sd_spi_switch] mutex release failed\n");
    }
}
