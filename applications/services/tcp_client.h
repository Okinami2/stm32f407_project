/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19     GreatMagicianGarfiel       the first version
 */

#ifndef APPLICATIONS_TASKS_TCP_CLIENT_H_
#define APPLICATIONS_TASKS_TCP_CLIENT_H_

#include <rtthread.h>
#include <stdbool.h>
#include <rtdef.h>
#include "time_service.h"

/**
 * @brief 初始化 TCP 客户端模块
 * @return 0: 成功, -1: 失败
 */
int tcp_client_init(void);

/**
 * @brief 检查网络链路是否已连接
 * @return true: 已连接, false: 未连接
 */
bool is_network_link_up(void);

/**
 * @brief 连接到服务器
 * @return 0: 成功, -1: 失败
 */
int tcp_connect_to_server(void);

/**
 * @brief 关闭 TCP 连接
 */
void tcp_close_socket(void);

/**
 * @brief 发送数据包
 * @param data 数据指针
 * @param len 数据长度
 * @return 发送成功的字节数，如果失败返回 -1
 */
int tcp_send_packet(const uint8_t *data, uint32_t len);

/**
 * @brief 检查 TCP 是否已连接
 * @return true: 已连接, false: 未连接
 */
bool tcp_is_connected(void);

#endif /* APPLICATIONS_TASKS_TCP_CLIENT_H_ */
