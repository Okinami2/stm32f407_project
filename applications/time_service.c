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

static void _spll_update(uint32_t delta_ticks) {
    const double alpha = 0.1;
    g_ts.ticks_per_sec = (1.0 - alpha) * g_ts.ticks_per_sec + alpha * (double)delta_ticks;
}


static void _update_pps_state_by_timeout(void *parameter)
{
    rt_tick_t now = rt_tick_get();
    rt_tick_t elapsed = now - g_ts.last_pps_sw_tick;
    uint32_t elapsed_ms = elapsed * 1000 / RT_TICK_PER_SECOND;

    switch (g_ts.pps_state) {
    case PPS_STATE_ACQUIRING:
        if (elapsed_ms > 30000) {  /* 30秒无PPS */
            g_ts.pps_state = PPS_STATE_FREERUN;
            g_ts.pps_valid_cnt = 0;
        }
        break;

    case PPS_STATE_LOCKED:
        if (elapsed_ms > 1500) {  /* 1.5秒无PPS */
            g_ts.pps_state = PPS_STATE_HOLDOVER;
            g_ts.pps_valid_cnt = 0;
        }
        break;

    case PPS_STATE_HOLDOVER:
        if (elapsed_ms > 180000) {  /* 180秒无有效PPS */
            g_ts.pps_state = PPS_STATE_FREERUN;
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
        return 0;
    }

    s_pps_timeout_timer = rt_timer_create(
        "pps_tmr",
        _update_pps_state_by_timeout,
        RT_NULL,
        rt_tick_from_millisecond(1000),
        RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER
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
    uint32_t now_hw = TS_HW_TIMER->CNT;
    rt_tick_t now_sw = rt_tick_get();

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

    g_ts.last_pps_hw_tick = now_hw;
    g_ts.last_pps_sw_tick = now_sw;

    switch (g_ts.pps_state) {

        case PPS_STATE_INIT:
            g_ts.pps_valid_cnt = 0;
            g_ts.ticks_per_sec = TS_TICKS_PER_SEC_NOMINAL;
            if (g_ts.ref_source != TS_REF_NONE) {
                g_ts.pps_state = PPS_STATE_ACQUIRING;
            }
            break;

        case PPS_STATE_ACQUIRING:
            if (is_valid_pulse) {
                    g_ts.pps_valid_cnt++;

                    if (g_ts.pps_valid_cnt >= PPS_ACQ_VALID_COUNT) {
                        g_ts.pps_state = PPS_STATE_LOCKED;
                    }
            } else {
                g_ts.pps_valid_cnt = 0;
            }

            break;

        case PPS_STATE_LOCKED:
            if (is_valid_pulse) {
                _spll_update(delta_hw);
                g_ts.base_utc_sec++;
            } else {
                g_ts.pps_state = PPS_STATE_HOLDOVER;
            }
            break;

        case PPS_STATE_HOLDOVER:
            if (is_valid_pulse) {
                g_ts.pps_state = PPS_STATE_LOCKED;
                g_ts.base_utc_sec++;// 仅在locked状态下自增，然后使用nmea中的定时信息来对表
            }

            break;

        case PPS_STATE_FREERUN:
            if (is_valid_pulse && g_ts.ref_source != TS_REF_NONE) {
                g_ts.pps_state = PPS_STATE_ACQUIRING;
                g_ts.pps_valid_cnt = 1;
            }
            break;

        default:
            g_ts.pps_state = PPS_STATE_INIT;
            break;
    }
}

void ts_get_time(Timestamp_t *ts)
{
    rt_base_t level = rt_hw_interrupt_disable();

    uint32_t now_hw = TS_HW_TIMER->CNT;

    uint32_t delta_hw;
    if (now_hw >= g_ts.last_pps_hw_tick) {
        delta_hw = now_hw - g_ts.last_pps_hw_tick;
    } else {
        delta_hw = (0xFFFFFFFF - g_ts.last_pps_hw_tick) + now_hw + 1;
    }

    uint64_t elapsed_us = (uint64_t)((double)delta_hw * 1000000.0 / g_ts.ticks_per_sec);

    ts->sec  = (uint32_t)(g_ts.base_utc_sec + (time_t)(elapsed_us / 1000000ULL));
    ts->usec = (uint32_t)(elapsed_us % 1000000ULL);

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
    uint32_t now_hw = TS_HW_TIMER->CNT;

    if (now_hw - g_ts.last_pps_hw_tick > 1000000 * 1.2) {
        rt_kprintf("[TS] The NMEA timestamp seems outdated: %d\n", utc_sec);
        return;
    }

    rt_base_t level = rt_hw_interrupt_disable();

    time_t diff = g_ts.base_utc_sec - utc_sec;
    time_t abs_diff = (diff < 0) ? -diff : diff;

    if (abs_diff > 1) {
        g_ts.base_utc_sec = utc_sec;
        rt_kprintf("[TS] NMEA Corrected Base Sec: %d\n", utc_sec);
    }

    if (g_ts.ref_source == TS_REF_NONE) {
        g_ts.ref_source = TS_REF_GNSS_PPS;
    }
    rt_hw_interrupt_enable(level);
}

/* NTP 校准  */
void ts_correct_time_by_ntp_offset_us(int64_t offset_us)
{
    Timestamp_t now;
    ts_get_time(&now);

    int64_t now_us    = (int64_t)now.sec * 1000000LL + (int64_t)now.usec;
    int64_t target_us = now_us + offset_us;

    int64_t target_sec  = target_us / 1000000LL;
    int64_t target_usec = target_us % 1000000LL;
    if (target_usec < 0) { target_sec -= 1; target_usec += 1000000LL; }

    int64_t abs_off_us = offset_us < 0 ? -offset_us : offset_us;
    if (abs_off_us <= TS_NTP_SYNC_THRESHOLD_US) return;

    rt_base_t level = rt_hw_interrupt_disable();

    if (g_ts.pps_state != PPS_STATE_LOCKED && g_ts.pps_state != PPS_STATE_HOLDOVER) {
        rt_kprintf("[TS] NTP Sync (Offset %+lld us). PPS State: %d\n",
                   (long long)offset_us, g_ts.pps_state);

        uint32_t now_tick = TS_HW_TIMER->CNT;
        g_ts.base_utc_sec = (uint32_t)target_sec;
        g_ts.ref_source   = TS_REF_NTP;

        double ticks_back_d = (double)target_usec * g_ts.ticks_per_sec / 1000000.0;
        uint32_t ticks_back = (uint32_t)(ticks_back_d + 0.5);

        g_ts.last_pps_hw_tick = now_tick - ticks_back;
        g_ts.last_pps_sw_tick = rt_tick_get();
    }

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

    rt_pin_mode(GET_PIN(G, 2),PIN_MODE_INPUT);
    rt_pin_attach_irq(GET_PIN(G, 2), PIN_IRQ_MODE_RISING, ts_pps_irq_handler, RT_NULL);
    rt_pin_irq_enable(GET_PIN(G, 2), PIN_IRQ_ENABLE);

    return 0;
}

INIT_APP_EXPORT(time_service_init);
