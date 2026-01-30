/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19     GreatMagicianGarfiel       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>

#include "adc_packet.h"
#include "../tasks/adc_get_thread.h"

rt_uint32_t g_sequence_id = 0;

uint32_t adc_packet_pack(uint8_t *buffer, uint32_t buffer_size, uint16_t start_index)
{
    uint32_t offset = 0;
    uint32_t full_len = ADC_PACKET_SIZE;
    uint8_t ts_idx = (start_index == 0) ? 0 : 1;

    if (!buffer || buffer_size < full_len)
    {
        return 0;
    }

    /* 添加包头 */
    buffer[offset++] = PACKET_FLAG_HEADER_1;
    buffer[offset++] = PACKET_FLAG_HEADER_2;

    /* 添加状态 */
    buffer[offset++] = PACKET_FLAG_STATUS_WATTING;

    /* 添加序列号 */
    rt_uint32_t seq = g_sequence_id++;
    memcpy(&buffer[offset], &seq, sizeof(rt_uint32_t));
    offset += sizeof(rt_uint32_t);

    /* 添加时间戳 */
    memcpy(&buffer[offset], &adc_receive_buffer.start_time[ts_idx], sizeof(sys_calendar_time_t));
    offset += sizeof(sys_calendar_time_t);

    /* 添加采样率 */
    memcpy(&buffer[offset], &adc_receive_buffer.sample_rate[ts_idx], sizeof(rt_uint8_t));
    offset += sizeof(rt_uint8_t);

    /* 添加 8 通道 ADC 数据 */
    memcpy(&buffer[offset], &adc_receive_buffer.ad0[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);

    memcpy(&buffer[offset], &adc_receive_buffer.ad1[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);

    memcpy(&buffer[offset], &adc_receive_buffer.ad2[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);

    memcpy(&buffer[offset], &adc_receive_buffer.ad3[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);

    memcpy(&buffer[offset], &adc_receive_buffer.ad4[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);

    memcpy(&buffer[offset], &adc_receive_buffer.ad5[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);

    memcpy(&buffer[offset], &adc_receive_buffer.ad6[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);

    memcpy(&buffer[offset], &adc_receive_buffer.ad7[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);

    /* 添加包尾 */
    buffer[offset++] = PACKET_FLAG_TAIL_1;
    buffer[offset++] = PACKET_FLAG_TAIL_2;

    return offset;
}
