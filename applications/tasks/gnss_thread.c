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
#include "../services/time_service.h"
#include "../services/sd_spi_switch.h"


#define DBG_TAG "gnss_thread"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define GNSS_UART_NAME       "uart5"

static rt_sem_t rx_sem = RT_NULL;
static rt_device_t serial_dev = RT_NULL;


static volatile int g_fix_quality = 0;
static volatile int g_sat_used    = 0;
static volatile float g_hdop      = 99.9f;


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
 * @brief Parse timestamp from NMEA RMC sentence
 * @param nmea_line NMEA sentence string
 * @param out_timestamp Output Unix timestamp
 * @return 0 on success, -1 on failure
 */
static int parse_rmc_time(const char *nmea_line, time_t *out_timestamp)
{
    // RMC format: $xxRMC,time,status,lat,NS,lon,EW,speed,course,date,mag,magEW,mode*checksum
    // Field index:  0     1     2      3   4   5   6    7      8     9    10  11    12
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
    const char *status_ptr = fields[1]; // A=valid, V=invalid
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

    if (!status_ptr || *status_ptr != 'A') {
        return -1;
    }

    return 0;
}

// GGA format: $GNGGA,UTC,Latitude,N/S,Longitude,E/W,Position Fix Indicator,Satellites Used,HDOP,MSL Altitude,AltUnit,GeoSep,GeoSepUnit...
// Field index:  0   1   2        3   4         5   6                      7               8    9   10       11      12
static int parse_gga_sat_used(const char *nmea_line, int *fix_quality, int *sat_used, float *hdop)
{
    const char *fields[20] = {0};
    int field_idx = 0;
    const char *p = nmea_line;

    while (*p && *p != ',') p++;
    if (*p != ',') return -1;

    while (*p && *p != '*' && *p != '\r' && *p != '\n' && field_idx < 20)
    {
        if (*p == ',') {
            p++;
            fields[field_idx++] = p;
            while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n') p++;
        } else {
            p++;
        }

        if (field_idx >= 8) break;   /* Only need up to HDOP */
    }

    if (field_idx < 8) return -1;

    *fix_quality = (fields[5] && *fields[5]) ? atoi(fields[5]) : 0;
    *sat_used    = (fields[6] && *fields[6]) ? atoi(fields[6]) : 0;
    *hdop        = (fields[7] && *fields[7]) ? (float)atof(fields[7]) : 99.9f;

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

                        if (parse_rmc_time(line_buffer, &gps_utc) == 0)
                        {
                            ts_correct_time_by_nmea(gps_utc);
                        }
                        else
                        {
                            /* Parse failed or Status='V' (no fix) */
                        }
                    }
                    else if(rt_strstr(line_buffer, "GGA")){
                        int fq, used;
                        float hdop;
                        if (parse_gga_sat_used(line_buffer, &fq, &used,&hdop) == 0) {
                            g_fix_quality = fq;
                            g_sat_used = used;
                            g_hdop = hdop;
                            // rt_kprintf("[GNSS] fix=%d, used=%d\n, hdop=%d", fq, used,hdop);
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


int gnss_get_fix_quality(void)
{
    return g_fix_quality;
}

int gnss_get_satellites_used(void)
{
    return g_sat_used;
}

float gnss_get_hdop(void)
{
    return g_hdop;
}
