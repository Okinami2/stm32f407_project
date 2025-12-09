/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19     GreatMagicianGarfiel       the first version
 */
#ifndef APPLICATIONS_ADC_GET_THREAD_H_
#define APPLICATIONS_ADC_GET_THREAD_H_

#include "ads131m08_app.h"
#include "digital_filtering.h"
#include "config_thread.h"


#define BATCH_SIZE 256

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t microsecond;
} utc;

typedef struct {
    float ad0[BATCH_SIZE * 2];
    float ad1[BATCH_SIZE * 2];
    float ad2[BATCH_SIZE * 2];
    float ad3[BATCH_SIZE * 2];
    float ad4[BATCH_SIZE * 2];
    float ad5[BATCH_SIZE * 2];
    float ad6[BATCH_SIZE * 2];
    float ad7[BATCH_SIZE * 2];
    utc time[BATCH_SIZE * 2];
} ADC_Receive_Buffer;

extern rt_sem_t adc_get_done_sem;
extern ADC_Receive_Buffer adc_receive_buffer;
extern bool receive_buff_flag;

int adc_get_thread_start(void);

#endif /* APPLICATIONS_ADC_GET_THREAD_H_ */
