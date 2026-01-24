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
#include <stdlib.h>

#include "adc_resend_thread.h"
#include "../services/tcp_client.h"
#include "../services/data_cache.h"
#include "../services/adc_packet.h"

#define RESEND_THREAD_STACK  4096
#define RESEND_THREAD_PRIO   21
#define RESEND_INTERVAL_MS   1000

/**
 * @brief 重发线程入口函数
 * @detail 负责从 SD NAND 缓存中读取数据并重发到服务器
 */
static void adc_resend_thread_entry(void *parameter)
{
    uint8_t *read_buf = rt_malloc(ADC_PACKET_MAX_SIZE);
    if (!read_buf)
    {
        rt_kprintf("[Resend] Memory allocation failed!\n");
        return;
    }

    while (1)
    {
        rt_thread_mdelay(RESEND_INTERVAL_MS);

        if (!tcp_is_connected() || !is_network_link_up())
        {
            continue;
        }

        if (!data_cache_has_pending())
        {
            continue;
        }

        for (int i = 0; i < ADC_RESENT_PACKET_NUM; ++i) { /* 一次多发点，防止缓存数据发送太慢 */

            int read_bytes = data_cache_read(read_buf, ADC_PACKET_MAX_SIZE);

            if (read_bytes > 0)
            {
                if (tcp_send_packet(read_buf, read_bytes) == read_bytes)
                {
                    data_cache_commit_read(read_bytes);
                }
                else
                {
                    rt_kprintf("tcp resent bytes isn't equal to expected num.\n");
                    tcp_close_socket();
                    rt_thread_mdelay(2000);
                }
            }
        }
    }
}

/**
 * @brief 启动重发线程
 */
int adc_resend_thread_start(void)
{
    rt_thread_t tid = rt_thread_create("adc_resend", adc_resend_thread_entry, RT_NULL,
                                       RESEND_THREAD_STACK, RESEND_THREAD_PRIO, 10);
    if (tid)
    {
        rt_thread_startup(tid);
        return RT_EOK;
    }

    return -RT_ERROR;
}
