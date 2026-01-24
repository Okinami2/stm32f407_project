#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "digital_filtering.h"
#include "../tasks/config_thread.h"

static int _compare_int32(const void* a, const void* b) {
    rt_int32_t val_a = *(const rt_int32_t*)a;
    rt_int32_t val_b = *(const rt_int32_t*)b;
    if (val_a < val_b) return -1;
    if (val_a > val_b) return 1;
    return 0;
}

//Welford's algorithm
static void _update_online_stats(OnlineStats* stats, rt_int32_t new_value) {
    if (NUM_MAX_HISOTRY > 0 && stats->count >= NUM_MAX_HISOTRY) {
            double delta = (double)new_value - stats->mean;
            stats->mean += delta / NUM_MAX_HISOTRY;
            double delta2 = (double)new_value - stats->mean;
            stats->m2 += delta * delta2;
        } else {
            stats->count++;
            double delta = (double)new_value - stats->mean;
            stats->mean += delta / stats->count;
            double delta2 = (double)new_value - stats->mean;
            stats->m2 += delta * delta2;
        }
}

//向历史数据缓冲区添加新数据
static void _add_to_history(ChannelProcessor* channel, rt_int32_t value) {
    channel->head_index = (channel->head_index + 1) % HISTORY_WINDOW_SIZE;
    channel->history[channel->head_index] = value;

    if (!channel->buffer_full && channel->head_index == HISTORY_WINDOW_SIZE - 1) {
        channel->buffer_full = RT_TRUE;
    }
}

void processor_init(DataProcessor* processor, rt_int32_t limit_min, rt_int32_t limit_max,
                    rt_int32_t gradient_threshold, float n_sigma, float low_pass_alpha) {
    if (!processor) return;

    memset(processor, 0, sizeof(DataProcessor));

    // 保存配置
    processor->limit_min = limit_min;
    processor->limit_max = limit_max;
    processor->gradient_threshold = gradient_threshold;
    processor->n_sigma = n_sigma;
    processor->low_pass_alpha = low_pass_alpha;

    for (int i = 0; i < NUM_ADC_CHANNELS; ++i) {
        processor->channels[i].head_index = HISTORY_WINDOW_SIZE - 1;
    }
}

void adc_data_filtering(DataProcessor* processor,
                      const rt_int32_t* raw_adc_data,
                      rt_int32_t* processed_data,
                      OutlierDetectionMethod outlier_method,
                      FilterType filter_type,
                      rt_bool_t* outlier_detected) {
    if (!processor || !raw_adc_data || !processed_data) return;

    for (int ch = 0; ch < NUM_ADC_CHANNELS; ++ch) {
        ChannelProcessor* channel = &processor->channels[ch];
        rt_int32_t current_data = raw_adc_data[ch];
        rt_bool_t is_outlier = RT_FALSE;

        // 异常值
        if (outlier_method == OUTLIER_DETECT_LIMIT || outlier_method == OUTLIER_DETECT_ALL_SEQUENTIAL) {
            if (current_data < processor->limit_min || current_data > processor->limit_max) {
                is_outlier = RT_TRUE;
            }
        }
        if (!is_outlier && (outlier_method == OUTLIER_DETECT_GRADIENT || outlier_method == OUTLIER_DETECT_ALL_SEQUENTIAL)) {
            if (channel->stats.count > 0 && llabs((long long)current_data - channel->last_raw_value) > processor->gradient_threshold) {
                is_outlier = RT_TRUE;
            }
        }
        if (!is_outlier && (outlier_method == OUTLIER_DETECT_3SIGMA || outlier_method == OUTLIER_DETECT_ALL_SEQUENTIAL)) {
            if (channel->stats.count >= 20) {
                double variance = channel->stats.m2 / channel->stats.count;
                double stddev = sqrt(variance);
                double threshold = processor->n_sigma * stddev;
                if (fabs((double)current_data - channel->stats.mean) > threshold) {
                    is_outlier = RT_TRUE;
                }
            }
        }

        if(outlier_detected) {
            outlier_detected[ch] = is_outlier;
        }

        rt_int32_t value_for_filter;
        if (is_outlier) {
            value_for_filter = channel->last_valid_value; // 使用上一个有效值
        } else {
            value_for_filter = current_data;
            channel->last_valid_value = current_data;
        }
        _update_online_stats(&channel->stats, current_data);

        _add_to_history(channel, value_for_filter);

        rt_int32_t filtered_value = value_for_filter;
        int hist_count = channel->buffer_full ? HISTORY_WINDOW_SIZE : (channel->head_index + 1);

        // 滤波
        switch (filter_type) {
            case FILTER_TYPE_MOVING_AVERAGE: {
                long long sum = 0;
                for (int i = 0; i < hist_count; ++i) sum += channel->history[i];
                filtered_value = (hist_count > 0) ? (rt_int32_t)(sum / hist_count) : value_for_filter;
                break;
            }
            case FILTER_TYPE_MEDIAN: {
                if (hist_count > 0) {
                    rt_int32_t temp_buf[HISTORY_WINDOW_SIZE];
                    memcpy(temp_buf, channel->history, hist_count * sizeof(rt_int32_t));
                    qsort(temp_buf, hist_count, sizeof(rt_int32_t), _compare_int32);
                    filtered_value = temp_buf[hist_count / 2];
                } else {
                    filtered_value = value_for_filter;
                }
                break;
            }
            case FILTER_TYPE_LOW_PASS: {
                filtered_value = (rt_int32_t)(processor->low_pass_alpha * value_for_filter +
                                           (1.0f - processor->low_pass_alpha) * channel->last_filtered_value);
                break;
            }
            case FILTER_TYPE_NONE:
            default:
                filtered_value = value_for_filter;
                break;
        }
        processed_data[ch] = filtered_value;

        channel->last_raw_value = current_data;
        channel->last_filtered_value = filtered_value;
    }
}
