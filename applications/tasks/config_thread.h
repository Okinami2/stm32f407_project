#ifndef __CONFIG_THREAD_H__
#define __CONFIG_THREAD_H__

#include <rtthread.h>
#include <stddef.h>

/* Outlier detection methods */
typedef enum {
    OUTLIER_DETECT_NONE = 0,
    OUTLIER_DETECT_LIMIT,
    OUTLIER_DETECT_GRADIENT,
    OUTLIER_DETECT_3SIGMA,
    OUTLIER_DETECT_ALL_SEQUENTIAL
} OutlierDetectionMethod;

/* Filter types */
typedef enum {
    FILTER_TYPE_NONE = 0,
    FILTER_TYPE_MOVING_AVERAGE,
    FILTER_TYPE_MEDIAN,
    FILTER_TYPE_LOW_PASS,
} FilterType;

typedef enum
{
    /* ADC related */
    CONFIG_ADC_GAIN = 0,
    CONFIG_ADC_ENABLE_CHANNEL,
    CONFIG_SAMPLE_RATE,
    /* Strain, acceleration related */
    CONFIG_STRAIN_S1,
    CONFIG_EPSILON_0,
    CONFIG_ACCELERATION_S2,
    CONFIG_A_0,
    CONFIG_ADC_DATA_TYPE,
    /* Filter related */
    CONFIG_ENABLE_FILTER,
    CONFIG_OUTLIER_DETECTION_METHOD,
    CONFIG_FILTER_TYPE,
    CONFIG_OUTLIER_MAX,
    CONFIG_OUTLIER_MIN,
    CONFIG_GRADIENT_THRESHLOD,
    CONFIG_N_SIGMA,
    CONFIG_LOW_PASS_ALPHA,

} config_update_name;

typedef struct {
    config_update_name msg_name;
} config_update_msg_t;

/* Application configuration */
typedef struct {
    rt_uint8_t sample_rate;
    rt_uint16_t adc_gain;
    rt_uint8_t adc_enable_channel;

    float strain_S1;
    float epsilon_0;
    float acceleration_S2;
    float a_0;
    rt_uint8_t adc_data_type;

    rt_uint8_t enable_filter;
    OutlierDetectionMethod outlier_detection_method;
    FilterType filter_type;
    rt_int32_t outlier_max;
    rt_int32_t outlier_min;
    rt_int32_t  gradient_threshold;
    float n_sigma;
    float low_pass_alpha;

} app_config_t;

extern app_config_t app_config;
extern rt_mq_t config_adc_update_notify;;
extern rt_mq_t config_ntp_update_notify;
/**
 * @brief Initialize and start configuration thread.
 *
 * Returns RT_EOK on success.
 */
int config_thread_init(void);

/**
 * @brief Register a configuration item into the registry.
 *
 * name: null-terminated name used by the CLI.
 * type: one of config_type_t.
 * ptr: pointer to the storage (should point to actual variable).
 * size: size of storage in bytes (for strings use buffer size).
 *
 * Returns RT_EOK on success, -RT_ERROR on fail.
 */
int config_register(const char *name,config_update_name,void *ptr, size_t size);

/**
 * @brief Set a configuration item by name using string value.
 *
 * Returns RT_EOK on success, otherwise an error code.
 */
int config_set(const char *name, const char *value);

/**
 * @brief Get a configuration item by name as string.
 *
 * out: caller-provided buffer, out_size its size. Returns RT_EOK on success.
 */
int config_get(const char *name, char *out, size_t out_size);

/**
 * @brief Print all registered configuration items to console.
 */
void config_print_all(void);

#endif // __CONFIG_THREAD_H__
