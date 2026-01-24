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
#include <stdio.h>

#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netdev.h>

#include <unistd.h>
#include <fcntl.h>

#include "tcp_client.h"

#define SERVER_IP        "192.168.137.1"
#define SERVER_PORT      9001
#define RECONNECT_INTERVAL_MS  5000

/* 静态变量 */
static int sock_fd = -1; /* Socket 文件描述符 */
static struct rt_mutex net_lock;   /* 保护 sock_fd 发送 */

bool is_network_link_up(void)
{
    struct netdev *netdev = netdev_get_by_name("e0");
    if (netdev)
    {
        return netdev_is_link_up(netdev);
    }
    return false;
}

int tcp_client_init(void)
{
    rt_mutex_init(&net_lock, "net_mtx", RT_IPC_FLAG_PRIO);
    sock_fd = -1;
    return 0;
}

void tcp_close_socket(void)
{
    rt_mutex_take(&net_lock, RT_WAITING_FOREVER);
    if (sock_fd >= 0)
    {
        closesocket(sock_fd);
        sock_fd = -1;
    }
    rt_mutex_release(&net_lock);
}

int tcp_connect_to_server(void)
{
    struct sockaddr_in server_addr;
    struct timeval timeout;
    static rt_tick_t last_try_tick = 0;
    rt_tick_t now_tick = rt_tick_get();

    /* 检查重试间隔 */
    if ((now_tick - last_try_tick) < rt_tick_from_millisecond(RECONNECT_INTERVAL_MS))
    {
        return -1;
    }
    last_try_tick = now_tick;

    if (sock_fd >= 0)
    {
        tcp_close_socket();  /* 确保连接断开 */
    }

    if (!is_network_link_up())
    {
        return -1;  /* 网络未连接，直接返回 */
    }

    sock_fd = socket(AF_INET, SOCK_STREAM, 0);  /* 创建 socket */
    if (sock_fd < 0)
    {
        rt_kprintf("[Network] Socket create failed!\n");
        return -1;
    }

    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    setsockopt(sock_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));  /* 设置发送超时 */

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

    /* 连接服务器 */
    if (connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) < 0)
    {
        rt_kprintf("[Network] Connect failed!\n");
        tcp_close_socket();  /* 连接失败后，清理 socket */
        return -1;
    }

    rt_kprintf("[Network] Connected to %s:%d\n", SERVER_IP, SERVER_PORT);
    return 0;
}

int tcp_send_packet(const uint8_t *data, uint32_t len)
{
    int total_sent = 0;
    int sent_bytes;

    if (sock_fd < 0) return -1;

    rt_mutex_take(&net_lock, RT_WAITING_FOREVER);

    while (total_sent < len)
    {
        sent_bytes = send(sock_fd, data + total_sent, len - total_sent, 0);
        if (sent_bytes <= 0)
        {
            rt_kprintf("[Network] Send failed, error: %d\n", sent_bytes);
            rt_mutex_release(&net_lock);
            return -1;
        }
        total_sent += sent_bytes;
    }

    rt_mutex_release(&net_lock);
    return total_sent;
}

bool tcp_is_connected(void)
{
    return (sock_fd >= 0);
}
