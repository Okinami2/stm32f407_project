/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-12-05     GreatMagicianGarfiel       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "time_service.h"
#include "sd_spi_switch.h"


#define DBG_TAG "gnss_thread"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define GNSS_UART_NAME       "uart5"

static rt_sem_t rx_sem = RT_NULL;
static rt_device_t serial_dev = RT_NULL;

static rt_err_t uart_input_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(rx_sem);
    return RT_EOK;
}

static int fast_atoi(const char *str, int len)
{
    int res = 0;
    for (int i = 0; i < len; i++) {
        if (str[i] >= '0' && str[i] <= '9') {
            res = res * 10 + (str[i] - '0');
        } else {
            break;
        }
    }
    return res;
}

/**
 * @brief 从 NMEA RMC 语句中提取时间戳
 * @param nmea_line NMEA 语句字符串
 * @param out_timestamp 输出的 Unix 时间戳
 * @return 0: 成功, -1: 失败
 */
static int parse_rmc_time(const char *nmea_line, time_t *out_timestamp)
{
    // RMC 格式: $xxRMC,time,status,lat,NS,lon,EW,speed,course,date,mag,magEW,mode*checksum
    // 字段索引:    0     1     2      3   4   5   6    7      8     9    10  11    12
    const char *fields[13] = {0};
    int field_idx = 0;
    const char *p = nmea_line;
    while (*p && *p != ',') p++;
    if (*p != ',') return -1;

    while (*p && *p != '*' && *p != '\r' && *p != '\n' && field_idx < 13)
    {
        if (*p == ',') {
            p++;
            fields[field_idx++] = p;
            while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n') {
                p++;
            }
        } else {
            p++;
        }
    }

    if (field_idx < 9) {
        return -1;
    }

    const char *time_ptr = fields[0];   // HHMMSS.sss
    const char *status_ptr = fields[1]; // A=有效, V=无效
    const char *date_ptr = fields[8];   // DDMMYY

    if (!time_ptr || time_ptr[0] < '0' || time_ptr[0] > '9') {
        return -1;
    }

    if (!date_ptr || date_ptr[0] < '0' || date_ptr[0] > '9') {
        return -1;
    }

    const char *time_end = time_ptr;
    int time_digit_count = 0;
    while (*time_end >= '0' && *time_end <= '9' && time_digit_count < 6) {
        time_end++;
        time_digit_count++;
    }
    if (time_digit_count < 6) {
        return -1;
    }

    const char *date_end = date_ptr;
    int date_digit_count = 0;
    while (*date_end >= '0' && *date_end <= '9' && date_digit_count < 6) {
        date_end++;
        date_digit_count++;
    }
    if (date_digit_count < 6) {
        return -1;
    }

    struct tm t = {0};
    t.tm_hour = fast_atoi(time_ptr, 2);
    t.tm_min  = fast_atoi(time_ptr + 2, 2);
    t.tm_sec  = fast_atoi(time_ptr + 4, 2);

    if (t.tm_hour > 23 || t.tm_min > 59 || t.tm_sec > 59) {
        return -1;
    }

    t.tm_mday = fast_atoi(date_ptr, 2);
    t.tm_mon  = fast_atoi(date_ptr + 2, 2) - 1;

    if (t.tm_mday < 1 || t.tm_mday > 31 || t.tm_mon < 0 || t.tm_mon > 11) {
        return -1;
    }

    int year_2d = fast_atoi(date_ptr + 4, 2);
    if (year_2d >= 80) {
        t.tm_year = year_2d;
    } else {
        t.tm_year = year_2d + 100;
    }

    *out_timestamp = timegm(&t);

    if (status_ptr && *status_ptr == 'V') {
        //rt_kprintf("RMC: GPS fix invalid, time may be inaccurate\n");
        return -1;
    }

    return 0;
}

void gnss_uart_suspend(void)
{
    if (serial_dev)
    {
        rt_device_close(serial_dev);
    }
}

void gnss_uart_resume(void)
{
    if (serial_dev)
    {
        rt_device_open(serial_dev, RT_DEVICE_FLAG_DMA_RX);

        rt_device_set_rx_indicate(serial_dev, uart_input_callback);
    }
}


static void gnss_thread_entry(void *parameter)
{
    static char rx_buffer[1024];
    static char line_buffer[128];
    static uint8_t line_idx = 0;

    while (1)
    {
        rt_sem_take(rx_sem, RT_WAITING_FOREVER);
        rt_size_t len = 0;
        while ((len = rt_device_read(serial_dev, 0, rx_buffer, sizeof(rx_buffer))) > 0)
        {
            for (int i = 0; i < len; i++)
            {
                char ch = rx_buffer[i];
                if (ch == '$') {
                    line_idx = 0;
                }

                if (line_idx < sizeof(line_buffer) - 1) {
                    line_buffer[line_idx++] = ch;
                }
                if (ch == '\n') {
                    line_buffer[line_idx] = '\0';
                    if (rt_strstr(line_buffer, "RMC"))
                    {
                        time_t gps_utc = 0;
                        //rt_kprintf("RMC: %s \n",line_buffer);

                        if (parse_rmc_time(line_buffer, &gps_utc) == 0)
                        {
                            //rt_kprintf("timestamp for nmea: %d \n",gps_utc);
                            ts_correct_time_by_nmea(gps_utc);
                            //rt_kprintf("[GNSS] Sync OK: %d \n", gps_utc);
                        }
                        else
                        {
                            // 解析失败或 Status='V' (未定位)
                        }
                    }

                    line_idx = 0;
                }
            }
        }
    }
}

int gnss_service_init(void)
{
    rt_pin_write(BSP_GNSSPWR_EN_PIN,PIN_HIGH);
    rt_pin_write(BSP_GNSS_SW_PIN,PIN_HIGH);

    ts_spi_bus_release();

    serial_dev = rt_device_find(GNSS_UART_NAME);
    if (!serial_dev)
    {
        LOG_E("GNSS UART not found!");
        return -RT_ERROR;
    }

    rx_sem = rt_sem_create("gnss_sem", 0, RT_IPC_FLAG_FIFO);

    rt_device_open(serial_dev, RT_DEVICE_FLAG_DMA_RX);

    rt_device_set_rx_indicate(serial_dev, uart_input_callback);

    rt_thread_t thread = rt_thread_create("gnss_rx",
                                          gnss_thread_entry,
                                          RT_NULL,
                                          2048,
                                          12,
                                          10);

    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }

    return RT_EOK;
}
INIT_APP_EXPORT(gnss_service_init);
