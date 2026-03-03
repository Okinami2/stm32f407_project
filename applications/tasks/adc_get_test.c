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

rt_int32_t adc_data_buffer[ADC_NUM_CHANNELS];

DataProcessor proc;
rt_int32_t processed_buf[NUM_ADC_CHANNELS];
rt_bool_t outlier_flags[NUM_ADC_CHANNELS]; /* Outlier detection flags */

// for test
static rt_int32_t pre_voltage[8] = {0};
static Timestamp_t last_log_time = {0}; // 用于限制打印频率
const rt_int32_t THRESHOLD = 83886;


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

        if (adc_enabled > 0) {
            mq_timeout = rt_tick_from_millisecond(10);
        } else {
            mq_timeout = RT_WAITING_FOREVER;
        }

        config_update_msg_t msg;
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 1))
        if (rt_mq_recv(config_adc_update_notify, &msg, sizeof(msg), mq_timeout) > 0)
#else
            if (rt_mq_recv(config_adc_update_notify, &msg, sizeof(msg), mq_timeout) == RT_EOK)
#endif
            {
                switch (msg.msg_name) {
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

            if (rt_sem_take(drdy_sem, rt_tick_from_millisecond(10)) == RT_EOK) // 适当增加超时容错
            {
                if (ads131m08_read_data_frame(adc_data_buffer, RT_FALSE) == RT_EOK)
                {
                    rt_bool_t need_log = RT_FALSE;
                    rt_bool_t need_print[8] = {RT_FALSE};
                    Timestamp_t current_time;
                    Timestamp_t channel_time[8] = {0};
                    ts_get_time(&current_time);

                    if (current_time.sec - last_log_time.sec >= 3) {
                        for (int i = 0; i < 8; ++i) {

                            rt_int32_t diff = pre_voltage[i] - adc_data_buffer[i];

                            if (diff > THRESHOLD || diff < -THRESHOLD) {
                                ts_get_time(&channel_time[i]);
                                need_print[i] = RT_TRUE;
                            }
                        }
                        for(int i =0; i < 8;i++){
                            if (need_print[i]) {
                                rt_kprintf("CH[%d] Change! Pre:%d, Cur:%d, Time:%d.%06d\n",
                                        i, (int)pre_voltage[i], (int)adc_data_buffer[i], channel_time[i].sec, channel_time[i].usec);
                                need_log = RT_TRUE;
                            }
                        }
                        for (int i = 0; i < 8; ++i) {
                            pre_voltage[i] = adc_data_buffer[i];
                        }
                        if(need_log){
                            last_log_time = current_time;
                        }
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
    /* Create semaphore */
    adc_get_done_sem = rt_sem_create("adc_get_done_sem", 0, RT_IPC_FLAG_FIFO);
    if (adc_get_done_sem == RT_NULL) {
        rt_kprintf("[ADC-APP] Error: Failed to create semaphore adc_get_done_sem.\n");
        return -RT_ERROR;
    }

    /* Initialize ADS131M08 */
    if (ads131m08_init() != RT_EOK) {
        rt_kprintf("[ADC-APP] Error: ADS131M08 initialization failed.\n");
        rt_sem_delete(adc_get_done_sem);
        return -RT_ERROR;
    }

    /* Initialize hardware timer */
    if (tim3_init() != RT_EOK) {
        rt_kprintf("[ADC-APP] Error: TIM3 initialization failed.\n");
        rt_sem_delete(adc_get_done_sem);
        return -RT_ERROR;
    }

    /* Initialize digital filter */
    rt_int32_t outlier_max = app_config.outlier_max;
    rt_int32_t outlier_min = app_config.outlier_min;
    rt_int32_t gradient_threshold = app_config.gradient_threshold;
    float n_sigma = app_config.n_sigma;
    float low_pass_alpha = app_config.low_pass_alpha;


    processor_init(&proc,outlier_min,outlier_max,gradient_threshold,n_sigma,low_pass_alpha);

    rt_thread_t tid = rt_thread_create("adc_data",
                                        adc_get_thread_entry,
                                        RT_NULL,
                                        ADC_GET_THREAD_STACK_SIZE,
                                        ADC_GET_THREAD_PRIORITY,
                                        ADC_GET_THREAD_TIMESLICE);

    if (tid == RT_NULL) {
            rt_kprintf("[ADC-APP] Error: Failed to create ADC data processing thread.\n");
            rt_sem_delete(adc_get_done_sem);
            return -RT_ERROR;
        }

        /* Start thread */
        rt_thread_startup(tid);

        /* Enable DRDY pin interrupt */
        rt_pin_irq_enable(BSP_nADC_DRDY_PIN, PIN_IRQ_ENABLE);

        LOG_I("[ADC-APP] ADC sampling system initialized successfully.\n");

        return RT_EOK;
}
