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
#include "config_thread.h"

#define NUM_ADC_CHANNELS 8
#define HISTORY_WINDOW_SIZE 20 /* Moving window size */
#define NUM_MAX_HISOTRY 10000 /* Required sample size for 3-sigma rule */

/* Welford online statistics */
typedef struct {
    int64_t count;      /* Total sample count */
    double mean;        /* Current mean */
    double m2;          /* Sum of squared deviations */
} OnlineStats;


typedef struct {
    rt_int32_t history[HISTORY_WINDOW_SIZE];
    uint8_t head_index;
    rt_bool_t buffer_full;
    /* 3-sigma rule state */
    OnlineStats stats;
    rt_int32_t last_raw_value;
    rt_int32_t last_valid_value;
    rt_int32_t last_filtered_value;
} ChannelProcessor;


typedef struct {
    ChannelProcessor channels[NUM_ADC_CHANNELS]; /* Processor for each channel */
    rt_int32_t limit_min;
    rt_int32_t limit_max;
    rt_int32_t gradient_threshold;
    double  n_sigma;
    float   low_pass_alpha;
} DataProcessor;


/**
 * @brief Initialize data processor
 * @param processor Pointer to processor instance
 * @param limit_min Amplitude limit minimum
 * @param limit_max Amplitude limit maximum
 * @param gradient_threshold Gradient change threshold
 * @param n_sigma Sigma multiplier for outlier detection
 * @param low_pass_alpha Low-pass filter coefficient
 */
void processor_init(DataProcessor* processor, rt_int32_t limit_min, rt_int32_t limit_max,
                    rt_int32_t gradient_threshold, float n_sigma, float low_pass_alpha);

/**
 * @brief Process ADC data with filtering
 * @param processor Processor instance
 * @param raw_adc_data Raw ADC data buffer (input)
 * @param processed_data Processed data buffer (output)
 * @param outlier_method Outlier detection method
 * @param filter_type Filter type
 * @param outlier_detected Optional array to mark outliers per channel (can be NULL)
 */
void adc_data_filtering(DataProcessor* processor,
                      const rt_int32_t* raw_adc_data,
                      rt_int32_t* processed_data,
                      OutlierDetectionMethod outlier_method,
                      FilterType filter_type,
                      rt_bool_t* outlier_detected);


#endif /* APPLICATIONS_DIGITAL_FILTERING_H_ */
