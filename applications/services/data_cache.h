/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19     GreatMagicianGarfiel       the first version
 */

#ifndef APPLICATIONS_TASKS_DATA_CACHE_H_
#define APPLICATIONS_TASKS_DATA_CACHE_H_

#include <rtthread.h>
#include <rtdef.h>
#include <stdbool.h>

/**
 * @brief 初始化数据缓存模块
 * @return 0: 成功, -1: 失败
 */
int data_cache_init(void);

/**
 * @brief 写入数据到缓存
 * @param data 数据指针
 * @param len 数据长度
 * @return 0: 成功, -1: 失败
 */
int data_cache_write(const uint8_t *data, uint32_t len);

/**
 * @brief 从缓存读取数据
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @return 实际读取的字节数，无数据返回 0，失败返回 -1
 */
int data_cache_read(uint8_t *buffer, uint32_t buffer_size);

/**
 * @brief 更新读取位置（确认数据已成功发送）
 * @param len 已发送的数据长度
 * @return 0: 成功, -1: 失败
 */
int data_cache_commit_read(uint32_t len);

/**
 * @brief 检查是否有待发送的缓存数据
 * @return true: 有数据, false: 无数据
 */
bool data_cache_has_pending(void);

/**
 * @brief 刷新 RAM 缓存到 SD NAND
 */
void data_cache_flush(void);

/**
 * @brief 获取当前 RAM 缓存偏移量
 * @return 当前缓冲区已使用的字节数
 */
uint32_t data_cache_get_ram_usage(void);

#endif /* APPLICATIONS_TASKS_DATA_CACHE_H_ */
