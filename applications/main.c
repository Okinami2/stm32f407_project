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


#define CACHE_INDEX_PATH "/cache.idx"
#define CACHE_DATA_PATH  "/cache_000.dat"

struct cache_index
{
    rt_uint32_t magic;
    rt_uint16_t write_file_idx;
    rt_uint16_t read_file_idx;
    rt_uint32_t write_off;
    rt_uint32_t read_off;
};

static void dump_cache_files(void)
{
    int fd;
    int i;

    rt_kprintf("\n===== CACHE FILE DUMP =====\n");

    /* 1. dump cache.idx 原始字节 */
    rt_uint8_t raw[sizeof(struct cache_index)] = {0};

    fd = open(CACHE_INDEX_PATH, O_RDONLY, 0);
    if (fd < 0)
    {
        rt_kprintf("open %s failed\n", CACHE_INDEX_PATH);
        return;
    }

    int r = read(fd, raw, sizeof(raw));
    close(fd);

    rt_kprintf("cache.idx raw (%d bytes):\n", r);
    for (i = 0; i < r; i++)
    {
        rt_kprintf("%02X ", raw[i]);
    }
    rt_kprintf("\n");

    /* 2. 结构体方式解析 */
    struct cache_index idx;
    rt_memcpy(&idx, raw, sizeof(idx));

    rt_kprintf("parsed cache.idx:\n");
    rt_kprintf("  magic          = 0x%08X\n", idx.magic);
    rt_kprintf("  write_file_idx = %u\n", idx.write_file_idx);
    rt_kprintf("  read_file_idx  = %u\n", idx.read_file_idx);
    rt_kprintf("  write_off      = %u\n", idx.write_off);
    rt_kprintf("  read_off       = %u\n", idx.read_off);

    /* 3. cache_000.dat 文件大小 */
    fd = open(CACHE_DATA_PATH, O_RDONLY, 0);
    if (fd >= 0)
    {
        int size = lseek(fd, 0, SEEK_END);
        close(fd);
        rt_kprintf("cache_000.dat size = %d bytes\n", size);
    }
    else
    {
        rt_kprintf("open %s failed\n", CACHE_DATA_PATH);
    }

    rt_kprintf("===== END DUMP =====\n\n");
}


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
    /*
    if(adc_send_to_server_start() != RT_EOK){
        rt_kprintf("fail start adc_send_thread\n");
    }
     */

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

        /* 卫星测试代码

        int quality = gnss_get_fix_quality();
        int sats    = gnss_get_satellites_used();
        float hdop  = gnss_get_hdop();

        if (quality >= 1)
        {
            rt_kprintf("[GNSS] 已定位 | 卫星数: %d | HDOP: %.2f (质量", sats, hdop);
            if (hdop < 1.5f)      rt_kprintf("极好)\n");
            else if (hdop < 2.0f) rt_kprintf("优秀)\n");
            else if (hdop < 4.0f) rt_kprintf("一般)\n");
            else                  rt_kprintf("较差)\n");
        }
        else
        {
            rt_kprintf("[GNSS] 未定位（Fix Quality = 0）\n");
        }
        PPS_State_t current_state = get_system_state();
        double cur_tick_per_sec = get_ticks_per_sec();
        uint32_t sec_int = (uint32_t)cur_tick_per_sec;
        uint32_t sec_f = (uint32_t)(cur_tick_per_sec - sec_int) * 100000;
        rt_kprintf("current_pps_stats: %d, current_tick_per_sec = %d.%d \n",current_state,sec_int,sec_f);

*/
        /*
        ts_spi_bus_claim();

        ls("/");
        dump_cache_files();

        ts_spi_bus_release();
        */

        rt_thread_mdelay(5000);

    }
    return RT_EOK;
}
