/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-12-09     GreatMagicianGarfiel       the first version
 */

#include <sys/time.h>
#include <stdlib.h>
#include "ntp.h"
#include "time_service.h"

#define DBG_TAG "ntp_task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define DEFAULT_SYNC_INTERVAL_MINUTES  0.1

static void ntp_sync_thread_entry(void *parameter)
{


    while (1)
    {
        time_t cur_time = ntp_get_time(NULL);
        ts_correct_time_by_ntp(cur_time,50);//暂用系统函数，待实现高精度ntp授时
        rt_tick_t delay_tick = DEFAULT_SYNC_INTERVAL_MINUTES * 60 * RT_TICK_PER_SECOND;
        //rt_tick_t delay_tick = RT_TICK_PER_SECOND * 30;
        rt_thread_delay(delay_tick);
    }
}

static int ntp_thread_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("ntp_sync",
                           ntp_sync_thread_entry,
                           RT_NULL,
                           2048,
                           25,
                           10);

    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        LOG_E("Create ntp sync thread failed!");
        return -RT_ERROR;
    }

    return RT_EOK;
}
INIT_APP_EXPORT(ntp_thread_init);
