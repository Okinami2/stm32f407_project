/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-xx-xx     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>

#include "main.h"
#include "tasks/adc_get_thread.h"
#include "tasks/adc_send_thread.h"
#include "hardware/max40109_hal.h"
#include "services/data_cache.h"
#include "services/time_service.h"

#include "tasks/adc_get_test.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


int main(void)
{
    rt_thread_mdelay(100);
    int count = 1;

    if(adc_get_test_start() != RT_EOK){
        rt_kprintf("fail start adc_get_test\n");
    }
    /*
    if(adc_get_thread_start() != RT_EOK){
        rt_kprintf("fail start adc_get_thread\n");
    }
     */

    //extern int gnss_uart_send(const char *message);


    /*
    if(adc_send_to_server_start() != RT_EOK){
        rt_kprintf("fail start adc_send_thread\n");
    }
    */
    //dump_cache_files();

    /* Test code for time display
    // struct timeval sys_time;
    sys_calendar_time_t sys_cal;
    char buf[96];
    */
    while (count++)
    {
        //gnss_uart_send("$PMTK285,4,100*38\r\n");
        //ts_get_time(&sys_time);
        /*
        ts_get_calendar_time(&sys_cal);
        rt_snprintf(buf, sizeof(buf),
                        "cur_time: %04d-%02d-%02d %02d:%02d:%02d.%06lu UTC\r\n",
                        sys_cal.year, sys_cal.month, sys_cal.day,
                        sys_cal.hour, sys_cal.minute, sys_cal.second,
                        sys_cal.microsecond);
        rt_kprintf("%s",buf);

        */
        /* Test code for reading MAX chip pressure
        int pressure = 0;
        max40109_read_pressure(0,&pressure);
        rt_kprintf("ch0: %d \n",pressure);
        */

        /* Test code for GNSS
        int quality = gnss_get_fix_quality();
        int sats    = gnss_get_satellites_used();
        float hdop  = gnss_get_hdop();

        if (quality >= 1)
        {
            rt_kprintf("[GNSS] Located | Sats: %d | HDOP: %.2f (", sats, hdop);
            if (hdop < 1.5f)      rt_kprintf("Excellent)\n");
            else if (hdop < 2.0f) rt_kprintf("Good)\n");
            else if (hdop < 4.0f) rt_kprintf("Fair)\n");
            else                  rt_kprintf("Poor)\n");
        }
        else
        {
            rt_kprintf("[GNSS] No fix (Fix Quality = 0)\n");
        }
        PPS_State_t current_state = get_system_state();
        double cur_tick_per_sec = get_ticks_per_sec();
        uint32_t sec_int = (uint32_t)cur_tick_per_sec;
        uint32_t sec_f = (uint32_t)(cur_tick_per_sec - sec_int) * 100000;
        rt_kprintf("current_pps_stats: %d, current_tick_per_sec = %d.%d \n",current_state,sec_int,sec_f);
        */

        //dump_cache_files();


        /* Test code for ADC value
        //double uncal_val = 0.0;
        //double cal_val = 0.0;
        //double ads_val = 0.0;

        //max40109_read_pressure(6, &uncal_val, 0);
        //max40109_read_pressure(6, &cal_val, 1);
        extern double get_tmp_ch0_press(void);
        ads_val = get_tmp_ch0_press();

        //int a1 = (int)(uncal_val);
        //int a2 = (int)(cal_val);
        int a3 = (int)(ads_val);
        int b1,b2,b3;

        //if(a1>0) b1 = (int)((uncal_val - a1) * 1000);
        //else b1 = (int)((a1 - uncal_val) * 1000);
        //if(a2>0) b2 = (int)((cal_val - a2) * 1000);
        //else b2 = (int)((a2 - cal_val) * 1000);
        if(a3>0) b3 = (int)((ads_val - a3) * 1000);
        else b3 = (int)((a3 - ads_val) * 1000);
        //rt_kprintf("ch6: %d.%03d, %d.%03d, %d.%03d\n", a1, b1, a2, b2, a3, b3);
        rt_kprintf("ch1: %d.%03d\n", a3, b3);
        */
        rt_thread_mdelay(5000);

    }
    return RT_EOK;
}
