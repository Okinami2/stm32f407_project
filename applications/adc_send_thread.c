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
#include <stdbool.h>
#include <string.h>

#include "adc_get_thread.h"
#include "adc_send_thread.h"

//#define SENDER_DEV_NAME  "wlan0"     // 待定
#define TX_THREAD_PRIO   20
#define TX_THREAD_STACK  2048

// 为了使用 DMA，数据必须在内存中连续
#define PACKET_DATA_SIZE (BATCH_SIZE * (8 * sizeof(float) + sizeof(utc)))

static uint8_t dma_tx_buffer[PACKET_DATA_SIZE + 16];//留一点多余空间

//static rt_device_t send_dev = RT_NULL;

/**
 * @brief 将分散的 ADC 数据打包到连续的内存中以便 DMA 发送
 * @param start_index 数据在 buffer 中的起始索引 (0 或 BATCH_SIZE)
 * @return 打包后的数据长度
 */
static uint32_t pack_data_to_buffer(uint16_t start_index)
{
    uint32_t offset = 0;

    //添加包头
    dma_tx_buffer[offset++] = 0xAA;
    dma_tx_buffer[offset++] = 0x55;

    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad0[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad1[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad2[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad3[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad4[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad5[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad6[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad7[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.time[start_index], BATCH_SIZE * sizeof(utc));
    offset += BATCH_SIZE * sizeof(utc);

    //添加包尾
    dma_tx_buffer[offset++] = 0x0D;
    dma_tx_buffer[offset++] = 0x0A;

    return offset;
}

/**
 * @brief 发送线程入口函数
 */
static void send_to_server_thread_entry(void *parameter)
{
    //rt_err_t res;
    uint16_t start_index;
    uint32_t packet_len;

    // 打开发送设备
    while (1)
    {
        if (rt_sem_take(adc_get_done_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            if (receive_buff_flag == true) {
                start_index = 0;
            } else {
                start_index = BATCH_SIZE;
            }
            // 打包
            packet_len = pack_data_to_buffer(start_index);
            rt_kprintf("---------the packet length is %d---------\n",packet_len);
            // 发送

        }
    }
}

/**
 * @brief 启动发送线程
 */
int adc_send_to_server_start(void)
{
    rt_thread_t tid = rt_thread_create("adc_send_to_server_start",
                                       send_to_server_thread_entry,
                                       RT_NULL,
                                       TX_THREAD_STACK,
                                       TX_THREAD_PRIO,
                                       10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        return RT_EOK;
    }
    return -RT_ERROR;
}
// 自动初始化 (可选)
// INIT_APP_EXPORT(adc_send_to_server_start);
