/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-13     GreatMagicianGarfiel       the first version
 */
#ifndef APPLICATIONS_DIGITAL_FILTERING_H_
#define APPLICATIONS_DIGITAL_FILTERING_H_


#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "../tasks/config_thread.h"

#define NUM_ADC_CHANNELS 8
#define HISTORY_WINDOW_SIZE 20 // 移动窗口大小
#define NUM_MAX_HISOTRY 10000 //对于3sigma准则来说，需要一个足够大的样本才行

// Welford
typedef struct {
    int64_t count;      // 样本总数
    double mean;        // 当前均值
    double m2;          // 离差平方和的累加
} OnlineStats;


typedef struct {
    rt_int32_t history[HISTORY_WINDOW_SIZE];
    uint8_t head_index;
    rt_bool_t buffer_full;
    //3sigma准则所需的状态
    OnlineStats stats;
    rt_int32_t last_raw_value;
    rt_int32_t last_valid_value;
    rt_int32_t last_filtered_value;
} ChannelProcessor;


typedef struct {
    ChannelProcessor channels[NUM_ADC_CHANNELS]; // 为每个通道创建一个处理器
    rt_int32_t limit_min;
    rt_int32_t limit_max;
    rt_int32_t gradient_threshold;
    double  n_sigma;
    float   low_pass_alpha;
} DataProcessor;


/**
 * @brief 初始化数据处理器
 * @param processor 指向要初始化的处理器实例
 * @param limit_min 限幅下限
 * @param limit_max 限幅上限
 * @param gradient_threshold 梯度阈值
 * @param n_sigma Sigma倍数
 * @param low_pass_alpha 低通滤波系数
 */
void processor_init(DataProcessor* processor, rt_int32_t limit_min, rt_int32_t limit_max,
                    rt_int32_t gradient_threshold, float n_sigma, float low_pass_alpha);

/**
 * @brief 综合数据处理函数
 * @param processor 处理器实例
 * @param raw_adc_data ADC原始数据缓冲区 (输入)
 * @param processed_data 处理后的数据缓冲区 (输出)
 * @param outlier_method 异常值检测方法
 * @param filter_type 滤波类型
 * @param outlier_detected 可选指针，用于返回一个rt_bool_t数组，标记每条通道是否检测到异常值 (可为NULL)
 */
void adc_data_filtering(DataProcessor* processor,
                      const rt_int32_t* raw_adc_data,
                      rt_int32_t* processed_data,
                      OutlierDetectionMethod outlier_method,
                      FilterType filter_type,
                      rt_bool_t* outlier_detected);


#endif /* APPLICATIONS_DIGITAL_FILTERING_H_ */
