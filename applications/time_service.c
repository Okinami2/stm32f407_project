/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-12-05     GreatMagicianGarfiel       the first version
 */
#include "time_service.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart5;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim2;

typedef struct {
    volatile time_t   base_utc_sec;      // 最近一次PPS对应的UTC秒数
    volatile uint32_t last_pps_tick;     // 上一次PPS到来时的硬件计数器值
    volatile double   ticks_per_sec;     // 实测的每秒Tick数 (理论1000000.0)
    volatile uint8_t  is_gnss_synced;    // GNSS是否已锁定 (PPS+NMEA正常)
    volatile uint32_t last_sync_ts;      // 最后一次成功对时的时间戳(用于判断当前状态下获取到的时间准确程度)
} ts_ctrl_t;

static ts_ctrl_t g_ts = {0, 0, 1000000.0, 0, 0};

typedef enum { MODE_SAFE_HIZ, MODE_GNSS, MODE_SD_SPI } mux_mode_t;

double get_ticks_per_sec(void){
    return g_ts.ticks_per_sec;
}

uint32_t get_last_pps_tick(void){
    return g_ts.last_pps_tick;
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

void ts_pps_irq_handler(void *args) {
    rt_interrupt_enter();

    uint32_t now = TS_HW_TIMER->CNT;

    if (g_ts.base_utc_sec == 0) {
        g_ts.last_pps_tick = now;
        rt_interrupt_leave();
        return;
    }

    uint32_t delta_tick;
    if (now >= g_ts.last_pps_tick) {
        delta_tick = now - g_ts.last_pps_tick;
    } else {
        delta_tick = (0xFFFFFFFF - g_ts.last_pps_tick) + now + 1;
    }

    /* 防抖 */
    if (delta_tick < 50000) {
        rt_interrupt_leave();
        return;
    }

    /* estimate the passed_sec */
    double seconds_passed_f = (double)delta_tick / g_ts.ticks_per_sec;
    uint32_t seconds_passed = (uint32_t)(seconds_passed_f + 0.5);

    if (seconds_passed > 0) {
        g_ts.base_utc_sec += seconds_passed;

        double current_freq = (double)delta_tick / (double)seconds_passed;

        if (fabs(current_freq - 1000000.0) < 5000.0) {
            g_ts.ticks_per_sec = (g_ts.ticks_per_sec * 7.0 + current_freq) / 8.0;
        }
        g_ts.is_gnss_synced = 1; // 标记 PPS 存活
    }

    g_ts.last_pps_tick = now;
    g_ts.last_sync_ts = rt_tick_get(); // 记录最后一次PPS的时间

    rt_interrupt_leave();
}

void ts_get_time(struct timeval *tv) {
    rt_base_t level = rt_hw_interrupt_disable();

    uint32_t now = TS_HW_TIMER->CNT;
    uint32_t last = g_ts.last_pps_tick;
    time_t sec = g_ts.base_utc_sec;
    double freq = g_ts.ticks_per_sec;

    rt_hw_interrupt_enable(level);

    uint32_t delta;
    if (now >= last) delta = now - last;
    else delta = (0xFFFFFFFF - last) + now + 1;

    uint64_t usec_total = (uint64_t)((double)delta * 1000000.0 / freq);

    tv->tv_sec = sec + (usec_total / 1000000);
    tv->tv_usec = usec_total % 1000000;
}

void ts_get_calendar_time(sys_calendar_time_t *cal) {
    struct timeval tv;
    ts_get_time(&tv);

    struct tm *tm_ptr = gmtime(&tv.tv_sec);
    cal->year = tm_ptr->tm_year + 1900;
    cal->month = tm_ptr->tm_mon + 1;
    cal->day = tm_ptr->tm_mday;
    cal->hour = tm_ptr->tm_hour;
    cal->minute = tm_ptr->tm_min;
    cal->second = tm_ptr->tm_sec;
    cal->microsecond = tv.tv_usec;
}


/* NMEA 校准  */
void ts_correct_time_by_nmea(time_t utc_sec) {
    rt_base_t level = rt_hw_interrupt_disable();
    if (g_ts.base_utc_sec == 0 || abs((int)(g_ts.base_utc_sec - utc_sec)) > 1) {
        g_ts.base_utc_sec = utc_sec;
    }

    rt_hw_interrupt_enable(level);
}

/* NTP 校准 */
void ts_correct_time_by_ntp(time_t ntp_sec, uint32_t ntp_ms) {
    struct timeval current_tv;
    ts_get_time(&current_tv);
    int32_t diff_sec = ntp_sec - current_tv.tv_sec;
    int32_t diff_ms = diff_sec * 1000 + (ntp_ms - (current_tv.tv_usec / 1000));

    if (abs(diff_ms) > TS_NTP_SYNC_THRESHOLD_MS) {
        rt_kprintf("[Time] NTP Fix: Drift %d ms too large, syncing...\n", diff_ms);
        rt_base_t level = rt_hw_interrupt_disable();

        g_ts.base_utc_sec = ntp_sec;
        uint32_t now = TS_HW_TIMER->CNT;
        uint32_t ticks_offset = (uint32_t)((double)ntp_ms * g_ts.ticks_per_sec / 1000.0);
        g_ts.last_pps_tick = now - ticks_offset;
        g_ts.is_gnss_synced = 0;

        rt_hw_interrupt_enable(level);

    } else {
        rt_kprintf("[Time] NTP Ignored: Drift %d ms is within tolerance.\n", diff_ms);
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

int time_service_init(void) {
    rt_pin_mode(BSP_GNSS_SW_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_GNSSPWR_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_RFMODPWR_EN_PIN, PIN_MODE_OUTPUT);

    rt_pin_write(BSP_RFMODPWR_EN_PIN,PIN_HIGH);
    rt_pin_write(BSP_GNSSPWR_EN_PIN,PIN_HIGH);

    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
            rt_kprintf("[Time] Error: TIM2 start failed!\n");
            return -RT_ERROR;
        }

        // 可选：为了确保它是 32位 自由运行，你可以再次确认一下重装载值
    __HAL_TIM_SET_AUTORELOAD(&htim2, 0xFFFFFFFF);

    ts_spi_bus_release();

    rt_pin_mode(GET_PIN(G, 2),PIN_MODE_INPUT);
    rt_pin_attach_irq(GET_PIN(G, 2), PIN_IRQ_MODE_RISING, ts_pps_irq_handler, RT_NULL);
    rt_pin_irq_enable(GET_PIN(G, 2), PIN_IRQ_ENABLE);

    return 0;
}
INIT_APP_EXPORT(time_service_init);
