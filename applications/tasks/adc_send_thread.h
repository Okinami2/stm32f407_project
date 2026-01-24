/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19     GreatMagicianGarfiel       the first version
 */

#ifndef APPLICATIONS_TASKS_ADC_SEND_THREAD_H_
#define APPLICATIONS_TASKS_ADC_SEND_THREAD_H_

#include <rtthread.h>
#include <rtdef.h>

/**
 * @brief 启动 ADC 数据发送线程（包括主发送和重发线程）
 * @return 0: 成功, -1: 失败
 */
int adc_send_to_server_start(void);

#endif /* APPLICATIONS_TASKS_ADC_SEND_THREAD_H_ */
