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
#include <dfs_fs.h>
#include "drv_spi.h"
#include "spi_msd.h"
#include "ntp.h"

#include "main.h"
#include "config_thread.h"
#include "adc_get_thread.h"
#include "adc_send_thread.h"
#include "max40109_hal.h"
#include "time_service.h"
#include "sd_spi_switch.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


rt_err_t sdnand_init_mount(void)
{
    rt_pin_write(BSP_RFMODPWR_EN_PIN,PIN_HIGH);
    rt_pin_write(BSP_TFPWR_EN_PIN,PIN_HIGH);
    ts_spi_bus_claim();
    rt_thread_mdelay(100);
    rt_pin_mode(BSP_SD_CS_PIN,PIN_MODE_OUTPUT);
    rt_err_t res = rt_hw_spi_device_attach("spi3", "spi30", SD_CS_GPIO_Port, SD_CS_Pin);
    if(res == RT_EOK ){
        res = msd_init("sdnand0","spi30");
    }

    int ret = dfs_mount("sdnand0", "/", "elm", 0, 0);

    if (ret == 0)
    {
        LOG_I("FatFS mounted to /");
    }
    else
    {
        LOG_E("Mount failed! Error code: %d", ret);
        LOG_W("Retrying mount in 500ms...");

        rt_thread_mdelay(500);
        if (dfs_mount("sdnand0", "/", "elm", 0, 0) == 0) {
            LOG_I("Retry mount success!");
        } else {
            LOG_E("Retry mount failed.");
        }
    }
    ts_spi_bus_release();
    return RT_EOK;
}

int main(void)
{
    int count = 1;

    if(config_thread_init() != RT_EOK){//必须先初始化配置线程
        rt_kprintf("fail start config_thread\n");
    }

    if(sdnand_init_mount() != RT_EOK){
        rt_kprintf("fail start sdnand_init_mount\n");
    }

    if(max_app_init() != RT_EOK){
        rt_kprintf("fail max_app_init\n");
    }

    if(adc_get_thread_start() != RT_EOK){
        rt_kprintf("fail start adc_get_thread\n");
    }

    if(adc_send_to_server_start() != RT_EOK){
        rt_kprintf("fail start adc_send_thread\n");
    }


    //struct timeval sys_time;
    //sys_calendar_time_t sys_cal;
    while (count++)
    {
        //ts_get_time(&sys_time);
        /*
        ts_get_calendar_time(&sys_cal);
        rt_kprintf("cur_time: %04d-%02d-%02d %02d:%02d:%02d.%06lu UTC\n",
                           sys_cal.year, sys_cal.month, sys_cal.day,
                           sys_cal.hour, sys_cal.minute, sys_cal.second,
                           sys_cal.microsecond);
        */
        /* 读取max芯片测量电压值
        */
        //int pressure = 0;
        //max40109_read_pressure(0,&pressure);
        //rt_kprintf("ch0: %d \n",pressure);

        rt_thread_mdelay(1000);

    }
    return RT_EOK;
}
