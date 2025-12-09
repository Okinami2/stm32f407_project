/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19     GreatMagicianGarfiel       the first version
 */
#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "adc_get_thread.h"

#define DBG_TAG "adc_get_thread"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// RT-Thread 线程配置
#define ADC_GET_THREAD_PRIORITY         10
#define ADC_GET_THREAD_STACK_SIZE       2048
#define ADC_GET_THREAD_TIMESLICE        10

rt_int32_t adc_data_buffer[ADC_NUM_CHANNELS];

uint16_t batch_index = 0;
utc utc_time;
ADC_Receive_Buffer adc_receive_buffer;
bool receive_buff_flag = false;
rt_sem_t adc_get_done_sem = RT_NULL;

DataProcessor proc;
rt_int32_t processed_buf[NUM_ADC_CHANNELS];
rt_bool_t outlier_flags[NUM_ADC_CHANNELS];//异常值出现标志


/**
 * @brief ADC数据处理线程入口函数
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

        if (app_config.adc_enable_channel > 0) {
            mq_timeout = 0; // 运行模式：非阻塞查询消息
        } else {
            mq_timeout = RT_WAITING_FOREVER; // 待机模式：阻塞等待消息，释放CPU
        }

        config_update_msg_t msg;
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 1))
        if (rt_mq_recv(config_update_notify, &msg, sizeof(msg), mq_timeout) > 0)
#else
            if (rt_mq_recv(config_update_notify, &msg, sizeof(msg), mq_timeout) == RT_EOK)
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

        if (app_config.adc_enable_channel > 0)
        {
            if (rt_sem_take(tim3_sem, RT_WAITING_FOREVER && app_config.adc_enable_channel > 0) == RT_EOK){
                if (rt_sem_take(drdy_sem, rt_tick_from_millisecond(5)) == RT_EOK)
                {
                    // rt_device_read(tim2_dev, 0, &timeout_s, sizeof(timeout_s));
                    // utc_time.microsecond = timeout_s.usec / 1000;
                    // adc_receive_buffer.time.year = utc_time.year;
                    // adc_receive_buffer.time.month = utc_time.month;
                    // adc_receive_buffer.time.day = utc_time.day;
                    // adc_receive_buffer.time.hour = utc_time.hour;
                    // adc_receive_buffer.time.second = utc_time.second;
                    // adc_receive_buffer.time.microsecond = timeout_s.usec / 1000;
                    if (ads131m08_read_data_frame(adc_data_buffer, RT_FALSE) == RT_EOK)
                    {
                        rt_int32_t *p_src_data; // 定义源数据指针

                        if (app_config.enable_filter) {
                            adc_data_filtering(&proc, adc_data_buffer, processed_buf, app_config.outlier_detection_method, app_config.filter_type, outlier_flags);
                            p_src_data = processed_buf;
                        } else {
                            p_src_data = adc_data_buffer;
                        }
                        // 将结构体内的数组地址放入指针数组，方便循环操作
                        float *p_dest_channels[ADC_NUM_CHANNELS] = {
                                adc_receive_buffer.ad0, adc_receive_buffer.ad1, adc_receive_buffer.ad2, adc_receive_buffer.ad3,
                                adc_receive_buffer.ad4, adc_receive_buffer.ad5, adc_receive_buffer.ad6, adc_receive_buffer.ad7
                        };
                        rt_uint16_t current_gain = app_config.adc_gain;

                        switch (app_config.adc_data_type)
                        {
                        case 1: // Voltage
                            for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
                                p_dest_channels[i][batch_index] = ads131m08_convert_to_voltage_uv(p_src_data[i], current_gain);
                            }
                            break;

                        case 2: // Strain
                            for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
                                float voltage = ads131m08_convert_to_voltage_uv(p_src_data[i], current_gain);
                                p_dest_channels[i][batch_index] = convert_to_strain_ue(voltage);
                            }
                            break;

                        case 3: // Acceleration
                            for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
                                float voltage = ads131m08_convert_to_voltage_uv(p_src_data[i], current_gain);
                                p_dest_channels[i][batch_index] = convert_to_acceleration_mg(voltage);
                            }
                            break;

                        default:
                            // rt_kprintf("unexpected adc data type\n");
                            break;
                        }

                        batch_index++;

                        if (batch_index % BATCH_SIZE == 0) {
                            rt_sem_release(adc_get_done_sem);
                            receive_buff_flag = !receive_buff_flag;
                        }

                        if (batch_index >= BATCH_SIZE * 2) {
                            batch_index = 0;
                        }
                    }
                }
            }

        }
    }
}

/**
 * @brief 初始化ADC并启动采样线程
 */
int adc_get_thread_start(void)
{
    /* 创建信号量 */
    adc_get_done_sem = rt_sem_create("adc_get_done_sem", 0, RT_IPC_FLAG_FIFO);
    if (adc_get_done_sem == RT_NULL) {
        rt_kprintf("[ADC-APP] Error: Failed to create semaphore adc_get_done_sem.\n");
        return -RT_ERROR;
    }

    /* 初始化ADS131M08 */
    if (ads131m08_init() != RT_EOK) {
        rt_kprintf("[ADC-APP] Error: ADS131M08 initialization failed.\n");
        rt_sem_delete(adc_get_done_sem);
        return -RT_ERROR;
    }

    /* 初始化定时器 */
    if (tim3_init() != RT_EOK) {
        rt_kprintf("[ADC-APP] Error: TIM3 initialization failed.\n");
        rt_sem_delete(adc_get_done_sem);
        return -RT_ERROR;
    }

    /* 初始化滤波器 */
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

        /* 启动线程 */
        rt_thread_startup(tid);

        /* 启用DRDY引脚中断 */
        rt_pin_irq_enable(BSP_nADC_DRDY_PIN, PIN_IRQ_ENABLE);

        rt_kprintf("[ADC-APP] ADC sampling system initialized successfully.\n");

        return RT_EOK;
}
