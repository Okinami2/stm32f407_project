/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19     GreatMagicianGarfiel       the first version
 */

#ifndef APPLICATIONS_TASKS_ADC_RESEND_THREAD_H_
#define APPLICATIONS_TASKS_ADC_RESEND_THREAD_H_

#include <rtthread.h>
#include <rtdef.h>

/**
 * @brief 启动重发线程
 * @return 0: 成功, -1: 失败
 */
int adc_resend_thread_start(void);

#endif /* APPLICATIONS_TASKS_ADC_RESEND_THREAD_H_ */
