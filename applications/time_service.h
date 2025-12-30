/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef APPLICATIONS_TIME_SERVICE_H_
#define APPLICATIONS_TIME_SERVICE_H_

#include <rtthread.h>
#include <sys/time.h>
#include "board.h"

/* 硬件定时器配置 */
#define TS_HW_TIMER         TIM2
#define TS_HW_TIMER_CLK     84000000
#define TS_TIMER_PRESCALER  83
#define TS_TICKS_PER_SEC_NOMINAL 1000000.0

#define TS_NTP_SYNC_THRESHOLD_US  100000
#define PPS_VALID_TOLERANCE_US    100

#define PPS_ACQ_VALID_COUNT              5       /* LOCKED 所需连续有效次数 */
#define PPS_LOCKED_TIMEOUT           30      /* LOCKED -> HOLDOVER*/
#define PPS_HOLDOVER_TIMEOUT            360     /* HOLDOVER -> FREERUN*/
#define PPS_ACQUIRING_TIMEOUT            360     /* ACQUIRING -> FREERUN*/

typedef struct{
    uint32_t       sec;
    uint32_t      usec;
} Timestamp_t;

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t microsecond;
} sys_calendar_time_t;

typedef enum {
    PPS_STATE_INIT = 0,
    PPS_STATE_ACQUIRING,
    PPS_STATE_LOCKED,
    PPS_STATE_HOLDOVER,
    PPS_STATE_FREERUN
} PPS_State_t;


/* API */
int time_service_init(void);
void ts_get_time(Timestamp_t *ts);
void ts_get_calendar_time(sys_calendar_time_t *cal);

void ts_correct_time_by_nmea(time_t utc_sec);
void ts_correct_time_by_ntp_offset_us(int64_t offset_us);

PPS_State_t get_system_state(void);
double get_ticks_per_sec(void);
uint32_t get_system_base_sec(void);

uint32_t get_last_pps_tick(void);

int   gnss_get_fix_quality(void);
int   gnss_get_satellites_used(void);
float gnss_get_hdop(void);

#endif /* APPLICATIONS_TIME_SERVICE_H_ */
