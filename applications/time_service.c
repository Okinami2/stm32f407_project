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

typedef struct
{
    volatile uint32_t last_pps_hw_tick;   /* 上一次 PPS 的硬件计数器值 */
    volatile uint32_t last_pps_sw_tick;   /* 上一次 PPS 的系统 Tick (用于超时监控) */
    volatile time_t   base_utc_sec;       /* 上一次 PPS 对应的 UTC 秒数 */

    volatile double   ticks_per_sec;      /* 当前估算的每秒硬件 tick 数 */

    volatile PPS_State_t pps_state;
    volatile uint8_t  pps_valid_cnt;      /* ACQUIRING 计数器 */

    volatile ts_ref_source_t ref_source;
} ts_ctrl_t;


static rt_timer_t s_pps_timeout_timer = RT_NULL;

static ts_ctrl_t g_ts = {
    .last_pps_hw_tick = 0,
    .last_pps_sw_tick = 0,
    .base_utc_sec = 0,  // 明确初始化
    .ticks_per_sec = TS_TICKS_PER_SEC_NOMINAL,
    .pps_state = PPS_STATE_INIT,
    .pps_valid_cnt = 0,
    .ref_source = TS_REF_NONE
};

typedef enum { MODE_SAFE_HIZ, MODE_GNSS, MODE_SD_SPI } mux_mode_t;


static void _spll_update(uint32_t delta_ticks) {
    const double alpha = 0.1;
    g_ts.ticks_per_sec = (1.0 - alpha) * g_ts.ticks_per_sec + alpha * (double)delta_ticks;
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



static void _update_pps_state_by_timeout(void *parameter)
{
    rt_tick_t now = rt_tick_get();
    rt_tick_t elapsed = now - g_ts.last_pps_sw_tick;
    uint32_t elapsed_ms = elapsed * 1000 / RT_TICK_PER_SECOND;

    switch (g_ts.pps_state) {
    case PPS_STATE_ACQUIRING:
        /* 捕获超时 -> 自由运行 */
        if (elapsed_ms > 3000) {  /* 3秒无PPS */
            g_ts.pps_state = PPS_STATE_FREERUN;
            g_ts.pps_valid_cnt = 0;
        }
        break;

    case PPS_STATE_LOCKED:
        /* 锁定状态下PPS丢失 -> 保持 */
        if (elapsed_ms > 1500) {  /* 1.5秒无PPS */
            g_ts.pps_state = PPS_STATE_HOLDOVER;
            g_ts.pps_valid_cnt = 0;
        }
        break;

    case PPS_STATE_HOLDOVER:
        /* 保持超时 -> 自由运行 */
        if (elapsed_ms > 60000) {  /* 60秒无有效PPS */
            g_ts.pps_state = PPS_STATE_FREERUN;
            g_ts.ref_source = TS_REF_NONE;  /* 清除参考源 */
        }
        break;

    case PPS_STATE_FREERUN:
    case PPS_STATE_INIT:
        break;
    }
}

/**
 * @brief 初始化PPS超时检测定时器
 * @return 0成功，-1失败
 */
static int _pps_timer_init(void)
{
    if (s_pps_timeout_timer != RT_NULL) {
        return 0;  /* 已初始化 */
    }

    s_pps_timeout_timer = rt_timer_create(
        "pps_tmr",                              /* 定时器名称 */
        _update_pps_state_by_timeout,           /* 回调函数 */
        RT_NULL,                                /* 回调参数 */
        rt_tick_from_millisecond(1000),         /* 周期1秒 */
        RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER  /* 周期+软件定时器 */
    );

    if (s_pps_timeout_timer == RT_NULL) {
        rt_kprintf("[TS] Failed to create timeout timer\n");
        return -1;
    }

    rt_err_t ret = rt_timer_start(s_pps_timeout_timer);
    if (ret != RT_EOK) {
        rt_kprintf("[TS] Failed to start timeout timer\n");
        rt_timer_delete(s_pps_timeout_timer);
        s_pps_timeout_timer = RT_NULL;
        return -1;
    }

    return 0;
}

/**
 * @brief 去初始化PPS超时检测定时器
 */
static void _pps_timer_deinit(void)
{
    if (s_pps_timeout_timer != RT_NULL) {
        rt_timer_stop(s_pps_timeout_timer);
        rt_timer_delete(s_pps_timeout_timer);
        s_pps_timeout_timer = RT_NULL;
    }
}



void ts_pps_irq_handler(void *args)
{
    (void)args;

    uint32_t now_hw = TS_HW_TIMER->CNT;
    rt_tick_t now_sw = rt_tick_get();

    /* 计算与上次PPS的间隔tick数 */
    uint32_t delta_hw;
    if (now_hw >= g_ts.last_pps_hw_tick) {
        delta_hw = now_hw - g_ts.last_pps_hw_tick;
    } else {
        delta_hw = (0xFFFFFFFF - g_ts.last_pps_hw_tick) + now_hw + 1;
    }

    /* 判断PPS脉冲是否有效（间隔在标称值±10%内） */
    uint32_t nominal = (uint32_t)TS_TICKS_PER_SEC_NOMINAL;
    int is_valid_pulse = (delta_hw > (uint32_t)(nominal * 0.9)) &&
                         (delta_hw < (uint32_t)(nominal * 1.1));

    /* 更新硬件时间戳（无论有效与否都更新，用于下次计算） */
    g_ts.last_pps_hw_tick = now_hw;
    g_ts.last_pps_sw_tick = now_sw;

    /* 状态机处理 */
    switch (g_ts.pps_state) {

        case PPS_STATE_INIT:
            /* 初始状态：需要有时间参考源才能开始捕获 */
            if (g_ts.ref_source != TS_REF_NONE) {
                g_ts.pps_state = PPS_STATE_ACQUIRING;
                g_ts.pps_valid_cnt = 0;
                g_ts.ticks_per_sec = TS_TICKS_PER_SEC_NOMINAL;
            }
            /* 无参考源则保持INIT，不递增秒数 */
            break;

        case PPS_STATE_ACQUIRING:
            /* 捕获状态：验证PPS信号稳定性 */
            if (is_valid_pulse) {
                uint32_t diff_abs = (delta_hw > nominal) ?
                                    (delta_hw - nominal) : (nominal - delta_hw);

                if (diff_abs < PPS_VALID_TOLERANCE_US) {
                    g_ts.pps_valid_cnt++;
                    _spll_update(delta_hw);  /* 开始训练SPLL */

                    if (g_ts.pps_valid_cnt >= PPS_ACQ_VALID_COUNT) {
                        g_ts.pps_state = PPS_STATE_LOCKED;
                    }
                } else {
                    /* 偏差过大，重新计数 */
                    g_ts.pps_valid_cnt = 0;
                }
            } else {
                /* 无效脉冲，重新计数 */
                g_ts.pps_valid_cnt = 0;
            }

            /* ACQUIRING状态下也递增秒数（已有参考源） */
            if (g_ts.ref_source != TS_REF_NONE) {
                g_ts.base_utc_sec++;
            }
            break;

        case PPS_STATE_LOCKED:
            /* 锁定状态：正常运行 */
            if (is_valid_pulse) {
                _spll_update(delta_hw);
                g_ts.base_utc_sec++;
            } else {
                /* PPS异常，进入保持模式 */
                g_ts.pps_state = PPS_STATE_HOLDOVER;
                g_ts.pps_valid_cnt = 0;
                /* 仍然递增秒数，使用上次的ticks_per_sec */
                g_ts.base_utc_sec++;
            }
            break;

        case PPS_STATE_HOLDOVER:
            /* 保持状态：等待PPS恢复 */
            if (is_valid_pulse) {
                g_ts.pps_valid_cnt++;
                _spll_update(delta_hw);

                /* 需要连续几个有效PPS才能恢复LOCKED */
                if (g_ts.pps_valid_cnt >= 2) {
                    g_ts.pps_state = PPS_STATE_LOCKED;
                }
            } else {
                g_ts.pps_valid_cnt = 0;
            }
            /* 保持模式仍递增秒数 */
            g_ts.base_utc_sec++;
            break;

        case PPS_STATE_FREERUN:
            /* 自由运行状态：时间不可信，等待重新同步 */
            if (is_valid_pulse && g_ts.ref_source != TS_REF_NONE) {
                /* 有有效PPS且有参考源，重新开始捕获 */
                g_ts.pps_state = PPS_STATE_ACQUIRING;
                g_ts.pps_valid_cnt = 1;
                g_ts.ticks_per_sec = TS_TICKS_PER_SEC_NOMINAL;
            }
            /* FREERUN状态下仍递增秒数（但不可信） */
            g_ts.base_utc_sec++;
            break;

        default:
            /* 异常状态恢复 */
            g_ts.pps_state = PPS_STATE_INIT;
            break;
    }
}

void ts_get_time(Timestamp_t *ts) {
    rt_base_t level = rt_hw_interrupt_disable();

    uint32_t now_hw = TS_HW_TIMER->CNT;
    uint32_t delta_hw = 0;
    if (now_hw >= g_ts.last_pps_hw_tick) {
            delta_hw = now_hw - g_ts.last_pps_hw_tick;
        } else {
            delta_hw = (0xFFFFFFFF - g_ts.last_pps_hw_tick) + now_hw + 1;
        }

    uint32_t usec_offset = (uint32_t)((double)delta_hw * 1000000.0 / g_ts.ticks_per_sec);

    ts->sec = g_ts.base_utc_sec;

    if (usec_offset >= 1000000) {
        ts->sec += usec_offset / 1000000;
        usec_offset %= 1000000;
    }
    ts->usec = usec_offset;

    rt_hw_interrupt_enable(level);
}

void ts_get_calendar_time(sys_calendar_time_t *cal) {
    Timestamp_t ts;
    ts_get_time(&ts);
    time_t ts_sec = (time_t)ts.sec + (8 * 3600);// 加东八区

    struct tm tm_buf;
    struct tm *tm_ptr = gmtime_r(&ts_sec, &tm_buf);

    if (tm_ptr) {
        cal->year = tm_ptr->tm_year + 1900;
        cal->month = tm_ptr->tm_mon + 1;
        cal->day = tm_ptr->tm_mday;
        cal->hour = tm_ptr->tm_hour;
        cal->minute = tm_ptr->tm_min;
        cal->second = tm_ptr->tm_sec;
    }
    cal->microsecond = ts.usec;
}

/* NMEA 校准  */
void ts_correct_time_by_nmea(time_t utc_sec) {
    rt_base_t level = rt_hw_interrupt_disable();

    if (g_ts.pps_state == PPS_STATE_LOCKED || g_ts.pps_state == PPS_STATE_ACQUIRING) {

        time_t diff = g_ts.base_utc_sec - utc_sec;
        time_t abs_diff = (diff < 0) ? -diff : diff;
        if (abs_diff > 1) {
            g_ts.base_utc_sec = utc_sec;
            rt_kprintf("[TS] NMEA Corrected Base Sec: %d\n", utc_sec);
        }
    }

    if (g_ts.ref_source == TS_REF_NONE) {
        g_ts.ref_source = TS_REF_GNSS_PPS;
    }

    rt_hw_interrupt_enable(level);
}

/* NTP 校准  */
void ts_correct_time_by_ntp(uint32_t ntp_sec, uint32_t ntp_usec) {
    Timestamp_t now;
    ts_get_time(&now);

    int32_t diff_sec = (int32_t)(ntp_sec - now.sec);
    int32_t diff_usec = (int32_t)(ntp_usec - now.usec);
    int32_t abs_diff_usec = (diff_usec < 0) ? -diff_usec : diff_usec;

    if (diff_sec != 0 || abs_diff_usec > TS_NTP_SYNC_THRESHOLD_US) {
        rt_base_t level = rt_hw_interrupt_disable();

        if (g_ts.pps_state != PPS_STATE_LOCKED && g_ts.pps_state != PPS_STATE_HOLDOVER) {
            rt_kprintf("[TS] NTP Sync (Drift %d s, %d us). PPS State: %d\n",
                       diff_sec, diff_usec, g_ts.pps_state);

            g_ts.base_utc_sec = ntp_sec;
            g_ts.ref_source = TS_REF_NTP;

            /* 反向计算 last_pps_hw_tick */
            uint32_t now_tick = TS_HW_TIMER->CNT;
            double ticks_offset = (double)diff_usec * g_ts.ticks_per_sec / 1000000.0;

            /* 正确处理正负偏移 */
            if (ticks_offset >= 0) {
                g_ts.last_pps_hw_tick = now_tick - (uint32_t)ticks_offset;
            } else {
                g_ts.last_pps_hw_tick = now_tick + (uint32_t)(-ticks_offset);
            }

            g_ts.last_pps_sw_tick = rt_tick_get();
        } else {
            rt_kprintf("[TS] NTP Ignored: PPS Locked (Drift %d us)\n", diff_usec);
        }

        rt_hw_interrupt_enable(level);
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

    __HAL_TIM_SET_AUTORELOAD(&htim2, 0xFFFFFFFF);


    if (_pps_timer_init() != 0) {
        rt_kprintf("[TS] Timer init failed\n");
        _pps_timer_deinit();
        return -1;
    }

    ts_spi_bus_release();

    rt_pin_mode(GET_PIN(G, 2),PIN_MODE_INPUT);
    rt_pin_attach_irq(GET_PIN(G, 2), PIN_IRQ_MODE_RISING, ts_pps_irq_handler, RT_NULL);
    rt_pin_irq_enable(GET_PIN(G, 2), PIN_IRQ_ENABLE);

    return 0;
}
INIT_APP_EXPORT(time_service_init);
