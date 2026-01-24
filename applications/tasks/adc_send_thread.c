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

#include "adc_send_thread.h"
#include "adc_resend_thread.h"
#include "adc_get_thread.h"
#include "../services/tcp_client.h"
#include "../services/adc_packet.h"
#include "../services/data_cache.h"

/* 线程配置 */
#define TX_THREAD_PRIO   20
#define TX_THREAD_STACK  4096


static uint8_t dma_tx_buffer[ADC_PACKET_MAX_SIZE];

/**
 * @brief 主发送线程入口函数
 */
static void send_to_server_thread_entry(void *parameter)
{
    uint16_t start_index;
    uint32_t packet_len;
    bool send_success = false;

    while (1)
    {
        if (!tcp_is_connected())
        {
            tcp_connect_to_server();
        }

        if (rt_sem_take(adc_get_done_sem, rt_tick_from_millisecond(1000)) == RT_EOK)
        {
            start_index = receive_buff_flag ? 0 : BATCH_SIZE;
            packet_len = adc_packet_pack(dma_tx_buffer, sizeof(dma_tx_buffer), start_index);

            if (packet_len == 0)
            {
                rt_kprintf("[Send] Packet pack failed!\n");
                continue;
            }

            if (!tcp_is_connected())
            {
                tcp_connect_to_server();
            }

            if (tcp_is_connected())
            {
                send_success = (tcp_send_packet(dma_tx_buffer, packet_len) > 0);
            }
            else
            {
                send_success = false;
            }

            if (!send_success)
            {
                rt_kprintf("[Send] Send failed, closing socket.\n");
                tcp_close_socket();
                data_cache_write(dma_tx_buffer, packet_len);
            }
        }

        if (data_cache_get_ram_usage() > 0)
        {
            data_cache_flush();
        }
    }
}

/**
 * @brief 启动发送线程
 */
int adc_send_to_server_start(void)
{
    /* 初始化各模块 */
    tcp_client_init();
    data_cache_init();

    /* 创建主发送线程 */
    rt_thread_t tid = rt_thread_create("adc_send", send_to_server_thread_entry, RT_NULL,
                                       TX_THREAD_STACK, TX_THREAD_PRIO, 10);
    if (tid)
    {
        rt_thread_startup(tid);
    }
    else
    {
        return -RT_ERROR;
    }

    /* 创建重发线程 */
    if (adc_resend_thread_start() != RT_EOK)
    {
        rt_kprintf("[Send] Failed to start resend thread!\n");
    }

    return RT_EOK;
}
