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
#include <dfs_posix.h>
#include <sys/socket.h>
#include <netdb.h>

#include "adc_get_thread.h"
#include "adc_send_thread.h"

#define SENDER_DEV_NAME  "e0"
#define TX_THREAD_PRIO   20
#define TX_THREAD_STACK  2048

#define STORAGE_PATH     "/adc_data.bin"
#define RECOVERY_PRIO    23
#define MAX_RETRY_COUNT  3


#define SERVER_IP        "192.168.137.1"  // 修改为你的服务器IP
#define SERVER_PORT      9001              // 修改为你的服务器端口

// 为了使用 DMA，数据必须在内存中连续
#define PACKET_DATA_SIZE (BATCH_SIZE * (8 * sizeof(float) + sizeof(sys_calendar_time_t)))

rt_align(32) static uint8_t dma_tx_buffer[PACKET_DATA_SIZE + 16];

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

static rt_bool_t is_network_ready(void)
{
    struct netdev *net_dev = netdev_get_by_name(SENDER_DEV_NAME);
    if (net_dev && netdev_is_link_up(net_dev) && netdev_is_up(net_dev))
    {
        return RT_TRUE;
    }
    return RT_FALSE;
}

static void save_packet_to_sd(uint8_t *data, uint32_t len)
{
    int fd = open(STORAGE_PATH, O_WRONLY | O_CREAT | O_APPEND);
    if (fd >= 0)
    {
        write(fd, data, len);
        close(fd);
    }
    else
    {
        rt_kprintf("[ADC SD] Save failed! Check SD card.\n");
    }
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
    uint16_t start_index;
    uint32_t packet_len;
    int ret;

    rt_thread_mdelay(5000);
    init_udp_socket();

    while (1)
    {
        if (rt_sem_take(adc_get_done_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            start_index = (receive_buff_flag == true) ? 0 : BATCH_SIZE;
            packet_len = pack_data_to_buffer(start_index);

            if (is_network_ready())
            {
                ret = sendto(sock_fd, dma_tx_buffer, packet_len, 0,
                        (struct sockaddr *)&server_addr, sizeof(server_addr));

                if (ret <= 0) // 如果发送失败（例如网卡虽然up但链路故障）
                {
                    save_packet_to_sd(dma_tx_buffer, packet_len);
                }
            }
            else
            {
                save_packet_to_sd(dma_tx_buffer, packet_len);
            }
        }
    }
}

static void recovery_thread_entry(void *parameter)
{
    uint8_t *read_buf = rt_malloc(PACKET_DATA_SIZE + 16);
    if (!read_buf) return;

    while (1)
    {
        if (is_network_ready())
        {
            struct stat st;
            if (stat(STORAGE_PATH, &st) == 0 && st.st_size > 0)
            {
                int fd = open(STORAGE_PATH, O_RDONLY);
                if (fd >= 0)
                {
                    rt_kprintf("[Recovery] Network recovered, resending history data...\n");

                    while (read(fd, read_buf, PACKET_DATA_SIZE + 4) > 0)
                    {
                        sendto(sock_fd, read_buf, PACKET_DATA_SIZE + 4, 0,
                               (struct sockaddr *)&server_addr, sizeof(server_addr));

                        rt_thread_mdelay(10);
                    }
                    close(fd);

                    unlink(STORAGE_PATH);
                    rt_kprintf("[Recovery] History data sent and cleared.\n");
                }
            }
        }
        rt_thread_mdelay(2000);
    }
}

/**
 * @brief 启动发送线程
 */
int adc_send_to_server_start(void)
{

    rt_thread_t tid1 = rt_thread_create("adc_send_to_server_start",
                                       send_to_server_thread_entry,
                                       RT_NULL,
                                       TX_THREAD_STACK,
                                       TX_THREAD_PRIO,
                                       10);

    rt_thread_t tid2 = rt_thread_create("adc_send_to_server_start",
                                           send_to_server_thread_entry,
                                           RT_NULL,
                                           1024,
                                           RECOVERY_PRIO,
                                           10);
    if (tid1 != RT_NULL && tid2 != RT_NULL)
    {
        rt_thread_startup(tid1);
        rt_thread_startup(tid2);
        return RT_EOK;
    }

    return -RT_ERROR;
}
// 自动初始化 (可选)
// INIT_APP_EXPORT(adc_send_to_server_start);
