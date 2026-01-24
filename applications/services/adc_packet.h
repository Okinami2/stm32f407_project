/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19     GreatMagicianGarfiel       the first version
 */

#ifndef APPLICATIONS_TASKS_ADC_PACKET_H_
#define APPLICATIONS_TASKS_ADC_PACKET_H_

#include <rtthread.h>
#include <rtdef.h>
#include <stdbool.h>

#include "../services/time_service.h"


#define BATCH_SIZE 64

typedef struct {
    sys_calendar_time_t start_time[2];
    rt_uint8_t sample_rate[2];
    float ad0[BATCH_SIZE * 2];
    float ad1[BATCH_SIZE * 2];
    float ad2[BATCH_SIZE * 2];
    float ad3[BATCH_SIZE * 2];
    float ad4[BATCH_SIZE * 2];
    float ad5[BATCH_SIZE * 2];
    float ad6[BATCH_SIZE * 2];
    float ad7[BATCH_SIZE * 2];
} ADC_Receive_Buffer;

extern rt_sem_t adc_get_done_sem;
extern ADC_Receive_Buffer adc_receive_buffer;
extern bool receive_buff_flag;

/* 协议定义 */
#define PACKET_HEADER_SIZE      4   /* AA 55 + 时间戳(2B) + 采样率(1B) = 实际上是动态的 */
#define PACKET_TAIL_SIZE        2   /* 0D 0A */

/* 最大数据包大小 (用于缓冲区分配) */
#define ADC_PACKET_MAX_SIZE     (BATCH_SIZE * 8 * sizeof(float) + sizeof(sys_calendar_time_t) + sizeof(rt_uint8_t) + 4)
#define ADC_RESENT_PACKET_NUM   5 /* 一次重发的数量 */
/**
 * @brief 计算 ADC 数据包的总长度
 * @return 数据包字节数
 */
uint32_t adc_packet_get_data_size(void);

/**
 * @brief 计算 ADC 数据包的完整长度（含包头包尾）
 * @return 数据包完整字节数
 */
uint32_t adc_packet_get_full_len(void);

/**
 * @brief 将 ADC 数据打包到缓冲区
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @param start_index 数据在 adc_receive_buffer 中的起始索引 (0 或 BATCH_SIZE)
 * @return 打包后的数据长度，失败返回 0
 */
uint32_t adc_packet_pack(uint8_t *buffer, uint32_t buffer_size, uint16_t start_index);

#endif /* APPLICATIONS_TASKS_ADC_PACKET_H_ */
