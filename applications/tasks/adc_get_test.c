/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19     GreatMagicianGarfiel       the first version
 *
 */
#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "adc_get_test.h"
#include "config_thread.h"
#include "../services/adc_packet.h"
#include "../hardware/ads131m08_hal.h"
#include "../hardware/ads131m08_app.h"
#include "../services/digital_filtering.h"
#include "../services/time_service.h"

#define DBG_TAG "adc.get"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* RT-Thread thread configuration */
#define ADC_GET_THREAD_PRIORITY         10
#define ADC_GET_THREAD_STACK_SIZE       2048
#define ADC_GET_THREAD_TIMESLICE        10

#define ADC_LOG_THREAD_PRIORITY         18
#define ADC_LOG_THREAD_STACK_SIZE       2048
#define ADC_LOG_THREAD_TIMESLICE        10

#define ADC_EVENT_BUF_SIZE              256

rt_int32_t adc_data_buffer[ADC_NUM_CHANNELS];

DataProcessor proc;
rt_int32_t processed_buf[NUM_ADC_CHANNELS];
rt_bool_t outlier_flags[NUM_ADC_CHANNELS];

/* 每个通道：始终与上一次采样值比较 */
static rt_int32_t prev_sample_voltage[ADC_NUM_CHANNELS] = {0};
static rt_bool_t prev_sample_valid[ADC_NUM_CHANNELS] = {0};

/* 每个通道独立冷却 */
static Timestamp_t last_print_time[ADC_NUM_CHANNELS] = {0};
static rt_bool_t last_print_valid[ADC_NUM_CHANNELS] = {0};

static const rt_int32_t THRESHOLD = 83886;
static const rt_uint32_t PRINT_COOLDOWN_SEC = 3;

/* 事件缓冲：采样线程只入队，日志线程慢慢打印 */
typedef struct
{
    rt_uint8_t ch;
    rt_int32_t prev;
    rt_int32_t cur;
    rt_int32_t diff;
    Timestamp_t ts;
} adc_change_event_t;

static adc_change_event_t s_event_buf[ADC_EVENT_BUF_SIZE];
static volatile rt_uint16_t s_event_w = 0;
static volatile rt_uint16_t s_event_r = 0;
static rt_sem_t s_event_sem = RT_NULL;

/* 来自 HAL 的 DRDY 锁存时间戳接口 */
extern rt_bool_t ads131m08_get_drdy_timestamp(Timestamp_t *ts);

static rt_bool_t timestamp_elapsed_ge_sec(const Timestamp_t *now,
                                          const Timestamp_t *last,
                                          rt_uint32_t sec_threshold)
{
    if (now->sec > last->sec + sec_threshold)
    {
        return RT_TRUE;
    }

    if (now->sec == last->sec + sec_threshold && now->usec >= last->usec)
    {
        return RT_TRUE;
    }

    return RT_FALSE;
}

static inline rt_int32_t abs_i32(rt_int32_t x)
{
    return (x >= 0) ? x : -x;
}

static void adc_event_push(rt_uint8_t ch,
                           rt_int32_t prev,
                           rt_int32_t cur,
                           rt_int32_t diff,
                           const Timestamp_t *ts)
{
    rt_uint16_t next;
    rt_base_t level;

    level = rt_hw_interrupt_disable();

    next = (rt_uint16_t)((s_event_w + 1) % ADC_EVENT_BUF_SIZE);
    if (next != s_event_r)
    {
        s_event_buf[s_event_w].ch = ch;
        s_event_buf[s_event_w].prev = prev;
        s_event_buf[s_event_w].cur = cur;
        s_event_buf[s_event_w].diff = diff;
        s_event_buf[s_event_w].ts = *ts;
        s_event_w = next;
        rt_hw_interrupt_enable(level);

        if (s_event_sem != RT_NULL)
        {
            rt_sem_release(s_event_sem);
        }
        return;
    }

    rt_hw_interrupt_enable(level);
    /* 缓冲满了就丢最旧后的新事件之外的当前事件，这里不打印，避免影响实时性 */
}

static rt_bool_t adc_event_pop(adc_change_event_t *evt)
{
    rt_base_t level;

    if (evt == RT_NULL)
    {
        return RT_FALSE;
    }

    level = rt_hw_interrupt_disable();

    if (s_event_r == s_event_w)
    {
        rt_hw_interrupt_enable(level);
        return RT_FALSE;
    }

    *evt = s_event_buf[s_event_r];
    s_event_r = (rt_uint16_t)((s_event_r + 1) % ADC_EVENT_BUF_SIZE);

    rt_hw_interrupt_enable(level);
    return RT_TRUE;
}

static void adc_log_thread_entry(void *parameter)
{
    adc_change_event_t evt;

    while (1)
    {
        rt_sem_take(s_event_sem, RT_WAITING_FOREVER);

        while (adc_event_pop(&evt))
        {
            rt_kprintf("CH[%d] Change! Pre:%d, Cur:%d, Diff:%d, Time:%u.%06u\n",
                       evt.ch,
                       (int)evt.prev,
                       (int)evt.cur,
                       (int)evt.diff,
                       (unsigned int)evt.ts.sec,
                       (unsigned int)evt.ts.usec);
        }
    }
}

/**
 * @brief ADC data acquisition and processing thread entry
 * @param parameter Unused
 */
static void adc_get_thread_entry(void *parameter)
{
    if (rt_sem_take(drdy_sem, rt_tick_from_millisecond(5)) == RT_EOK)
    {
        ads131m08_read_data_frame(adc_data_buffer, RT_TRUE);
    }

    while (1)
    {
        rt_int32_t mq_timeout;
        rt_uint8_t adc_enabled;

        rt_enter_critical();
        adc_enabled = app_config.adc_enable_channel;
        rt_exit_critical();

        if (adc_enabled > 0)
        {
            mq_timeout = rt_tick_from_millisecond(10);
        }
        else
        {
            mq_timeout = RT_WAITING_FOREVER;
        }

        config_update_msg_t msg;
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 1))
        if (rt_mq_recv(config_adc_update_notify, &msg, sizeof(msg), mq_timeout) > 0)
#else
        if (rt_mq_recv(config_adc_update_notify, &msg, sizeof(msg), mq_timeout) == RT_EOK)
#endif
        {
            switch (msg.msg_name)
            {
            case CONFIG_ADC_GAIN:
                ads131m08_set_gain(app_config.adc_gain);
                break;
            case CONFIG_ADC_ENABLE_CHANNEL:
                ads131m08_enable_cannel(app_config.adc_enable_channel);
                break;
            case CONFIG_SAMPLE_RATE:
                rt_sem_control(tim3_sem, RT_IPC_CMD_RESET, 0);
                break;
            case CONFIG_OUTLIER_MAX:
            case CONFIG_OUTLIER_MIN:
            case CONFIG_GRADIENT_THRESHLOD:
            case CONFIG_N_SIGMA:
            case CONFIG_LOW_PASS_ALPHA:
                processor_init(&proc,
                               app_config.outlier_min,
                               app_config.outlier_max,
                               app_config.gradient_threshold,
                               app_config.n_sigma,
                               app_config.low_pass_alpha);
                break;
            default:
                break;
            }
        }

        if (adc_enabled > 0)
        {
            if (rt_sem_take(drdy_sem, rt_tick_from_millisecond(10)) == RT_EOK)
            {
                if (ads131m08_read_data_frame(adc_data_buffer, RT_FALSE) == RT_EOK)
                {
                    Timestamp_t sample_time;

                    /*
                     * 优先使用 DRDY 中断锁存的时间戳；
                     * 若本次没有取到，则退化为当前时刻。
                     */
                    if (ads131m08_get_drdy_timestamp(&sample_time) != RT_TRUE)
                    {
                        ts_get_time(&sample_time);
                    }

                    for (int i = 0; i < ADC_NUM_CHANNELS; ++i)
                    {
                        rt_int32_t cur = adc_data_buffer[i];

                        if (!prev_sample_valid[i])
                        {
                            prev_sample_voltage[i] = cur;
                            prev_sample_valid[i] = RT_TRUE;
                            continue;
                        }

                        rt_int32_t prev = prev_sample_voltage[i];
                        rt_int32_t diff = cur - prev;

                        if (abs_i32(diff) > THRESHOLD)
                        {
                            rt_bool_t allow_push = RT_FALSE;

                            /* 每个通道独立冷却 */
                            if (!last_print_valid[i])
                            {
                                allow_push = RT_TRUE;
                            }
                            else if (timestamp_elapsed_ge_sec(&sample_time,
                                                              &last_print_time[i],
                                                              PRINT_COOLDOWN_SEC))
                            {
                                allow_push = RT_TRUE;
                            }

                            if (allow_push)
                            {
                                adc_event_push((rt_uint8_t)i, prev, cur, diff, &sample_time);
                                last_print_time[i] = sample_time;
                                last_print_valid[i] = RT_TRUE;
                            }
                        }

                        /*
                         * 关键点：
                         * 无论是否入队打印事件，都更新“上一次采样值”
                         * 下次检测永远基于相邻采样。
                         */
                        prev_sample_voltage[i] = cur;
                    }
                }
            }
        }
    }
}

/**
 * @brief Initialize ADC and start sampling thread
 * @return RT_EOK on success, error code on failure
 */
int adc_get_test_start(void)
{
    adc_get_done_sem = rt_sem_create("adc_get_done_sem", 0, RT_IPC_FLAG_FIFO);
    if (adc_get_done_sem == RT_NULL)
    {
        rt_kprintf("[ADC-APP] Error: Failed to create semaphore adc_get_done_sem.\n");
        return -RT_ERROR;
    }

    s_event_sem = rt_sem_create("adc_evt_sem", 0, RT_IPC_FLAG_FIFO);
    if (s_event_sem == RT_NULL)
    {
        rt_kprintf("[ADC-APP] Error: Failed to create semaphore adc_evt_sem.\n");
        rt_sem_delete(adc_get_done_sem);
        return -RT_ERROR;
    }

    if (ads131m08_init() != RT_EOK)
    {
        rt_kprintf("[ADC-APP] Error: ADS131M08 initialization failed.\n");
        rt_sem_delete(s_event_sem);
        rt_sem_delete(adc_get_done_sem);
        return -RT_ERROR;
    }

    if (tim3_init() != RT_EOK)
    {
        rt_kprintf("[ADC-APP] Error: TIM3 initialization failed.\n");
        rt_sem_delete(s_event_sem);
        rt_sem_delete(adc_get_done_sem);
        return -RT_ERROR;
    }

    rt_int32_t outlier_max = app_config.outlier_max;
    rt_int32_t outlier_min = app_config.outlier_min;
    rt_int32_t gradient_threshold = app_config.gradient_threshold;
    float n_sigma = app_config.n_sigma;
    float low_pass_alpha = app_config.low_pass_alpha;

    processor_init(&proc,
                   outlier_min,
                   outlier_max,
                   gradient_threshold,
                   n_sigma,
                   low_pass_alpha);

    rt_thread_t adc_tid = rt_thread_create("adc_data",
                                           adc_get_thread_entry,
                                           RT_NULL,
                                           ADC_GET_THREAD_STACK_SIZE,
                                           ADC_GET_THREAD_PRIORITY,
                                           ADC_GET_THREAD_TIMESLICE);

    if (adc_tid == RT_NULL)
    {
        rt_kprintf("[ADC-APP] Error: Failed to create ADC data processing thread.\n");
        rt_sem_delete(s_event_sem);
        rt_sem_delete(adc_get_done_sem);
        return -RT_ERROR;
    }

    rt_thread_t log_tid = rt_thread_create("adc_evt_log",
                                           adc_log_thread_entry,
                                           RT_NULL,
                                           ADC_LOG_THREAD_STACK_SIZE,
                                           ADC_LOG_THREAD_PRIORITY,
                                           ADC_LOG_THREAD_TIMESLICE);

    if (log_tid == RT_NULL)
    {
        rt_kprintf("[ADC-APP] Error: Failed to create ADC log thread.\n");
        rt_sem_delete(s_event_sem);
        rt_sem_delete(adc_get_done_sem);
        return -RT_ERROR;
    }

    rt_thread_startup(adc_tid);
    rt_thread_startup(log_tid);

    rt_pin_irq_enable(BSP_nADC_DRDY_PIN, PIN_IRQ_ENABLE);

    LOG_I("[ADC-APP] ADC sampling system initialized successfully.\n");
    return RT_EOK;
}
