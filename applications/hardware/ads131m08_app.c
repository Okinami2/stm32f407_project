#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "ads131m08_app.h"

#define DBG_TAG "ads131m08.app"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Timer related variables */
#define HWTIMER_DEV_NAME   "timer3"
rt_sem_t tim3_sem = RT_NULL;
rt_hwtimerval_t tim3_timeout_s;
rt_uint8_t tick_count  = 0;
rt_device_t tim3_dev = RT_NULL;

static rt_err_t tim3_timeout_cb(rt_device_t dev, rt_size_t size)
{
    rt_bool_t release_sem = RT_FALSE;

    switch (app_config.sample_rate) {
        case 1:
            if (tick_count % 200 == 0) {
                release_sem = RT_TRUE;
            }
            break;

        case 10:
            if (tick_count % 20 == 0) {
                release_sem = RT_TRUE;
            }
            break;

        case 20:
            if (tick_count % 10 == 0) {
                release_sem = RT_TRUE;
            }
            break;

        case 50:
            if (tick_count % 4 == 0) {
                release_sem = RT_TRUE;
            }
            break;

        case 100:
            if (tick_count % 2 == 0) {
                release_sem = RT_TRUE;
            }
            break;

        case 200:
            release_sem = RT_TRUE;
            break;

        default:
            rt_kprintf("Unknown rate: %d\n", app_config.sample_rate);
            break;
    }

    tick_count++;
    if (tick_count >= 200) {
        tick_count = 0;
    }

    if (release_sem) {
        if (drdy_sem != RT_NULL) rt_sem_release(tim3_sem);
    }
    return RT_EOK;
}

/**
 * @brief Initialize TIM3 hardware timer for sample rate control
 */
int tim3_init(void)
{
    rt_err_t ret;
    tim3_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (tim3_dev == RT_NULL) {
        rt_kprintf("Error: TIM3 device not found\n");
        return -RT_ERROR;
    }
    ret = rt_device_open(tim3_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) {
        rt_kprintf("Error: Open TIM3 device failed: %d\n", ret);
        return ret;
    }
    rt_device_set_rx_indicate(tim3_dev, tim3_timeout_cb);

    tim3_sem = rt_sem_create("tim3_sem", 0, RT_IPC_FLAG_FIFO);
    if (tim3_sem == RT_NULL) {
        rt_kprintf("Error: Failed to create semaphore tim3_sem.\n");
        return -RT_ERROR;
    }

    rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD;
    rt_uint32_t freq = 1000000;       /* Counter frequency */
    ret = rt_device_control(tim3_dev, HWTIMER_CTRL_FREQ_SET, &freq);
        if (ret != RT_EOK)
        {
            rt_kprintf("set frequency failed! ret is :%d\n", ret);
            return ret;
        }

        ret = rt_device_control(tim3_dev, HWTIMER_CTRL_MODE_SET, &mode);
        if (ret != RT_EOK)
        {
            rt_kprintf("set mode failed! ret is :%d\n", ret);
            return ret;
        }


    tim3_timeout_s.sec = 0;
    tim3_timeout_s.usec = 5000;
    if (rt_device_write(tim3_dev, 0, &tim3_timeout_s, sizeof(tim3_timeout_s)) != sizeof(tim3_timeout_s))
        {
            rt_kprintf("set timeout value failed\n");
            return RT_ERROR;
        }
    return RT_EOK;
}



/**
 * @brief Convert raw ADC reading to voltage value in microvolts
 * @param raw_value Raw 32-bit ADC data
 * @param gain Gain setting used for acquisition
 * @return Voltage in uV, or NAN if gain is invalid
 */
float ads131m08_convert_to_voltage_uv(rt_int32_t raw_value, rt_uint16_t gain)
{
    float fsr;
    switch (app_config.adc_gain)
    {
        case 1:   fsr = 1.2f;       break;
        case 2:   fsr = 0.6f;       break; // 600 mV
        case 4:   fsr = 0.3f;       break; // 300 mV
        case 8:   fsr = 0.15f;      break; // 150 mV
        case 16:  fsr = 0.075f;     break; // 75 mV
        case 32:  fsr = 0.0375f;    break; // 37.5 mV
        case 64:  fsr = 0.01875f;   break; // 18.75 mV
        case 128: fsr = 0.009375f;  break; // 9.375 mV
        default:
            return NAN;
    }

    float voltage_uv = ((float)raw_value / ADC_FULL_SCALE_CODE / 90 / 4) * fsr * 1e6f; /* 90 is the gain from max40109,fiexd for test, 4 is the out stage gain in max40109*/

    return voltage_uv;
}

/**
 * @brief Convert voltage to strain value
 * Formula: ε = V / S1 + ε0
 * @param input_voltage_v Input voltage in uV
 * @return Strain value in microstrain (uε)
 */
float convert_to_strain_ue(float input_voltage_v)
{
    if (app_config.strain_S1 == 0.0f) return NAN;

    float strain = (input_voltage_v / 1e6f )/ app_config.strain_S1 + app_config.epsilon_0;
    return strain;
}

/**
 * @brief Convert voltage to acceleration value
 * Formula: a = V/S2 + a0
 * @param input_voltage_v Input voltage in uV
 * @return Acceleration value in mg
 */
float convert_to_acceleration_mg(float input_voltage_v)
{
    if (app_config.acceleration_S2 == 0.0f) return NAN;

    float acceleration = ((input_voltage_v / 1e6f ) / app_config.acceleration_S2) + app_config.a_0;
    return acceleration;
}
