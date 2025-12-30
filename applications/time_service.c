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


typedef struct {
    uint32_t system_base_sec;   // 最近一次确定的基准秒 (UTC)
    uint32_t last_pps_tick;     // 最近一次 PPS 触发时的硬件计数器值
    uint32_t   last_pps_ovf;         // 硬件计数器溢出次数
    double   ticks_per_sec;     // 滤波器修正后的频率
    PPS_State_t state;          // 状态机状态
} TimeEngine_t;


static rt_timer_t s_pps_timeout_timer = RT_NULL;
static rt_uint8_t acquiring_pps_cnt = 0;

static TimeEngine_t engine = {
        .system_base_sec = 0,
        .last_pps_tick = 0,
        .last_pps_ovf = 0,
        .ticks_per_sec = TS_TICKS_PER_SEC_NOMINAL,
        .state = PPS_STATE_INIT
};


extern TIM_HandleTypeDef htim2;
static volatile uint32_t tim2_ovf = 0;

// 重写中断回调函数，不能重写HAL_TIM_PeriodElapsedCallback，因为已经被bsp占用了，但是底层没适配tim2
void TIM2_IRQHandler(void)
{
    rt_interrupt_enter();

    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
            tim2_ovf++;
        }
    }

    rt_interrupt_leave();
}


static void _spll_update(uint32_t delta_ticks) {
    const double alpha = 0.1;
    engine.ticks_per_sec = (1.0 - alpha) * engine.ticks_per_sec + alpha * (double)delta_ticks;
}

static inline uint64_t _read_tim2_ext_tick(uint32_t *ovf_out, uint32_t *cnt_out)
{
    uint32_t o1, o2, c;
    do {
        o1 = tim2_ovf;
        c  = TIM2->CNT;
        o2 = tim2_ovf;
    } while (o1 != o2);

    if (ovf_out) *ovf_out = o2;
    if (cnt_out) *cnt_out = c;
    return ((uint64_t)o2 << 32) | (uint64_t)c;
}

static void _update_pps_state_by_timeout(void *parameter)
{
    rt_base_t level = rt_hw_interrupt_disable();

    uint32_t ovf_now, cnt_now;
    uint64_t now_ext  = _read_tim2_ext_tick(&ovf_now, &cnt_now);
    uint64_t last_ext = ((uint64_t)engine.last_pps_ovf << 32) | engine.last_pps_tick;

    uint64_t elapsed_ticks = now_ext - last_ext;
    uint32_t elapsed_s = (uint32_t)((double)elapsed_ticks / engine.ticks_per_sec);

    switch (engine.state)
    {
    case PPS_STATE_ACQUIRING:
        if (elapsed_s > PPS_ACQUIRING_TIMEOUT) engine.state = PPS_STATE_FREERUN;
        break;
    case PPS_STATE_HOLDOVER:
        if (elapsed_s > PPS_HOLDOVER_TIMEOUT) engine.state = PPS_STATE_FREERUN;
        break;
    case PPS_STATE_LOCKED:
        if (elapsed_s > PPS_LOCKED_TIMEOUT) engine.state = PPS_STATE_HOLDOVER;
        break;
    default:
        break;
    }

    rt_hw_interrupt_enable(level);
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
    uint32_t now = TS_HW_TIMER->CNT;

    uint32_t ovf_now, cnt_now;
    uint64_t now_ext = _read_tim2_ext_tick(&ovf_now, &cnt_now);

    uint64_t last_ext = ((uint64_t)engine.last_pps_ovf << 32) | engine.last_pps_tick;
    uint64_t delta_tick = now_ext - last_ext;

    /* 判断PPS脉冲是否有效（间隔在标称值±10%内） */
    uint32_t nominal = (uint32_t)TS_TICKS_PER_SEC_NOMINAL;
    int is_valid_pulse = (delta_tick > (uint32_t)(nominal * 0.9)) &&
                         (delta_tick < (uint32_t)(nominal * 1.1));

    engine.last_pps_tick = now;
    engine.last_pps_ovf = ovf_now;
    if (is_valid_pulse && engine.system_base_sec != 0)
    {
        engine.system_base_sec++;
    }

    switch (engine.state) {

        case PPS_STATE_INIT:
            acquiring_pps_cnt = 0;
            engine.state = PPS_STATE_ACQUIRING;
            break;

        case PPS_STATE_ACQUIRING:
            if(is_valid_pulse){
                acquiring_pps_cnt++;
            }
            else{
                acquiring_pps_cnt = 0;
            }

            if(acquiring_pps_cnt >= 5){
                engine.state = PPS_STATE_LOCKED;
            }
            break;

        case PPS_STATE_HOLDOVER:
            acquiring_pps_cnt = 0;
            engine.state = PPS_STATE_ACQUIRING;
            break;

        case PPS_STATE_LOCKED:
            if(!is_valid_pulse){
                engine.state = PPS_STATE_HOLDOVER;
            }
            else{
                _spll_update(delta_tick);
            }

            break;


        case PPS_STATE_FREERUN:
            engine.state = PPS_STATE_ACQUIRING;
            break;

        default:
            engine.state = PPS_STATE_INIT;
            break;
    }
}

void ts_get_time(Timestamp_t *ts)
{
    rt_base_t level = rt_hw_interrupt_disable();

    uint32_t ovf_now, cnt_now;
    uint64_t now_ext = _read_tim2_ext_tick(&ovf_now, &cnt_now);

    uint64_t last_ext = ((uint64_t)engine.last_pps_ovf << 32) | engine.last_pps_tick;
    uint64_t elapsed_ticks = now_ext - last_ext;

    double elapsed_s = (double)elapsed_ticks / engine.ticks_per_sec;
    uint32_t sec_add = (uint32_t)elapsed_s;
    uint32_t usec    = (uint32_t)((elapsed_s - (double)sec_add) * 1000000.0 + 0.5);

    ts->sec  = engine.system_base_sec + sec_add;
    if(usec >= 1000000){
        ts->sec += 1;
        ts->usec = usec - 1000000;
    }
    else{
        ts->usec = usec;
    }

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
    uint32_t now = TS_HW_TIMER->CNT;

    if (now - engine.last_pps_tick > 1000000 * 1.2) {
        rt_kprintf("[TS] The NMEA timestamp seems outdated: %d\n", utc_sec);
        return;
    }

    rt_base_t level = rt_hw_interrupt_disable();

    if (engine.system_base_sec != utc_sec) {
        engine.system_base_sec = utc_sec;
        rt_kprintf("[TS] NMEA Corrected Base Sec: %d\n", utc_sec);
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
    while (target_usec < 0) { target_sec -= 1; target_usec += 1000000LL; }

    if (target_sec < 0) { target_sec = 0; target_usec = 0; }

    rt_base_t level = rt_hw_interrupt_disable();

    uint32_t ovf_now, cnt_now;
    uint64_t now_ext = _read_tim2_ext_tick(&ovf_now, &cnt_now);
    uint32_t tps = (uint32_t)(engine.ticks_per_sec + 0.5);
    uint64_t ticks_back = ((uint64_t)target_usec * (uint64_t)tps + 500000ULL) / 1000000ULL;

    if (ticks_back >= (uint64_t)tps)
    {
        ticks_back = 0;
        target_sec += 1;
        target_usec = 0;
    }
    uint64_t ref_ext = (ticks_back <= now_ext) ? (now_ext - ticks_back) : 0;

    engine.system_base_sec = (uint32_t)target_sec;
    engine.last_pps_ovf    = (uint32_t)(ref_ext >> 32);
    engine.last_pps_tick   = (uint32_t)(ref_ext & 0xFFFFFFFFU);

    rt_hw_interrupt_enable(level);
}


int time_service_init(void) {
    rt_pin_mode(BSP_GNSS_SW_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_GNSSPWR_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_RFMODPWR_EN_PIN, PIN_MODE_OUTPUT);

    rt_pin_write(BSP_RFMODPWR_EN_PIN,PIN_HIGH);
    rt_pin_write(BSP_GNSSPWR_EN_PIN,PIN_HIGH);

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        rt_kprintf("[Time] Error: TIM2 start_it failed!\n");
        return -RT_ERROR;
    }

    if (_pps_timer_init() != 0) {
        rt_kprintf("[TS] Timer init failed\n");
        _pps_timer_deinit();
        return -1;
    }

    rt_pin_mode(GET_PIN(G, 2),PIN_MODE_INPUT);
    rt_pin_attach_irq(GET_PIN(G, 2), PIN_IRQ_MODE_RISING, ts_pps_irq_handler, RT_NULL);
    rt_pin_irq_enable(GET_PIN(G, 2), PIN_IRQ_ENABLE);

    rt_thread_mdelay(100);
    engine.state = PPS_STATE_INIT;
    engine.last_pps_tick = 0;

    return 0;
}

INIT_APP_EXPORT(time_service_init);

PPS_State_t get_system_state(void){
    return engine.state;
}

double get_ticks_per_sec(void){
    return engine.ticks_per_sec;
}

uint32_t get_system_base_sec(void){
    return engine.system_base_sec;
}

uint32_t get_last_pps_tick(void){
    return engine.last_pps_tick;
}

