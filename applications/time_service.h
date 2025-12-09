/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-12-05     SystemArch   Updated for 0.1ms precision & NTP logic
 */
#ifndef APPLICATIONS_TIME_SERVICE_H_
#define APPLICATIONS_TIME_SERVICE_H_

#include <rtthread.h>
#include <sys/time.h>
#include "board.h"

#define TS_HW_TIMER         TIM2
#define TS_HW_TIMER_CLK     84000000
#define TS_TIMER_PRESCALER  83

#define TS_NTP_SYNC_THRESHOLD_MS  50

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t microsecond;
} sys_calendar_time_t;

/**
 * @brief 系统时间服务初始化
 */
int time_service_init(void);

/**
 * @brief 获取高精度系统时间 (Unix时间戳 + 微秒)
 * 核心算法: Base_Sec + (Current_Tick - Base_Tick) / Ticks_Per_Sec
 */
void ts_get_time(struct timeval *tv);

/**
 * @brief 获取年月日时分秒格式的高精度时间
 * 内部会自动将 time_t 转换为日历格式
 */
void ts_get_calendar_time(sys_calendar_time_t *cal);

/**
 * @brief 输入NMEA报文中的UTC时间进行校准
 * @param utc_sec: 解析出的Unix时间戳 (GPRMC中的时间)
 * @note  逻辑: PPS负责"走字"(Tick累加), NMEA负责"对表"(告诉系统现在是几点)
 */
void ts_correct_time_by_nmea(time_t utc_sec);

/**
 * @brief 输入NTP时间进行校准 (最坏情况下的兜底)
 * @param ntp_sec: NTP服务器返回的秒
 * @param ntp_ms:  NTP服务器返回的毫秒部分
 */
void ts_correct_time_by_ntp(time_t ntp_sec, uint32_t ntp_ms);

void ts_spi_bus_claim(void);
void ts_spi_bus_release(void);

double get_ticks_per_sec(void);
uint32_t get_last_pps_tick(void);
#endif /* APPLICATIONS_TIME_SERVICE_H_ */
