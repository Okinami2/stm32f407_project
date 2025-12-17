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

#include <sys/socket.h>
#include <netdb.h>

#include "adc_get_thread.h"
#include "adc_send_thread.h"

//#define SENDER_DEV_NAME  "e0"
#define TX_THREAD_PRIO   20
#define TX_THREAD_STACK  2048


#define SERVER_IP        "192.168.137.1"  // 修改为你的服务器IP
#define SERVER_PORT      9001              // 修改为你的服务器端口

// 为了使用 DMA，数据必须在内存中连续
#define PACKET_DATA_SIZE (BATCH_SIZE * (8 * sizeof(float) + sizeof(sys_calendar_time_t)))

static uint8_t dma_tx_buffer[PACKET_DATA_SIZE + 16];//留一点多余空间

// Socket 句柄
static int sock_fd = -1;
static struct sockaddr_in server_addr;

//static rt_device_t send_dev = RT_NULL;


static int init_udp_socket(void)
{
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        rt_kprintf("[ADC Send] Failed to create socket\n");
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    rt_kprintf("[ADC Send] UDP Socket initialized (Server: %s:%d)\n",
               SERVER_IP, SERVER_PORT);

    return 0;
}

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
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.time[start_index], BATCH_SIZE * sizeof(sys_calendar_time_t));
    offset += BATCH_SIZE * sizeof(sys_calendar_time_t);

    //添加包尾
    dma_tx_buffer[offset++] = 0x0D;
    dma_tx_buffer[offset++] = 0x0A;

    return offset;
}

static int send_data_udp(uint8_t *data, uint32_t len)
{
    int sent_len = sendto(sock_fd, data, len, 0,
                          (struct sockaddr *)&server_addr,
                          sizeof(server_addr));

    if (sent_len < 0)
    {
        rt_kprintf("[ADC Send] UDP sendto failed: %d\n", sent_len);
        return -1;
    }

    return sent_len;
}

/**
 * @brief 发送线程入口函数
 */
static void send_to_server_thread_entry(void *parameter)
{
    //rt_err_t res;
    uint16_t start_index;
    uint32_t packet_len;

    int ret;
    uint32_t packet_count = 0;
    uint32_t error_count = 0;

    rt_thread_mdelay(10000);
    if (init_udp_socket() != 0)
        {
            rt_kprintf("[ADC Send] Failed to initialize UDP socket, thread exit\n");
            return;
        }
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
            ret = send_data_udp(dma_tx_buffer, packet_len);

            if (ret > 0)
            {
                packet_count++;

                // 每发送 100 个包打印一次统计信息
                if (packet_count % 5 == 0)
                {
                    rt_kprintf("[ADC Send] Sent %u packets, %u errors, last size: %u bytes\n",
                            packet_count, error_count, packet_len);
                }
            }
            else
            {
                error_count++;
                rt_kprintf("[ADC Send] Send failed! Total errors: %u\n", error_count);
            }

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
