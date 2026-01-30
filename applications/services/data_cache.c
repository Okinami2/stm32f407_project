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
#include <unistd.h>
#include <fcntl.h>
#include "lfs.h"

#include "data_cache.h"
#include "adc_packet.h"
#include "../services/sd_spi_switch.h"

#define DBG_TAG "data_cache"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* 缓存文件配置 */
#define CACHE_FILE_PREFIX     "/cache_"
#define CACHE_FILE_SUFFIX     ".dat"

#define CACHE_FILE_COUNT      200
#define CACHE_SEG_SIZE        (16 * 1024 * 1024UL)

/* RAM 缓存配置 */
#define RAM_CACHE_SIZE               (32U * 1024U)
#define CACHE_FLUSH_INTERVAL_TICKS   (rt_tick_from_millisecond(5000))

/* 内存维护的读写状态 */
typedef struct {
    rt_uint32_t write_file_idx;
    rt_uint32_t write_offset;
    rt_uint32_t read_file_idx;
    rt_uint32_t read_offset;
    rt_uint32_t last_write_seq;
} cache_state_t;

/* 静态变量 */
static struct rt_mutex cache_lock;
static rt_uint8_t  sd_cache_ram_buf[RAM_CACHE_SIZE];
static rt_uint32_t ram_buf_offset = 0;
static rt_tick_t   last_flush_tick = 0;
static cache_state_t cache_state;

/* 辅助函数 */
static void build_cache_path(char *path, size_t size, rt_uint32_t idx)
{
    rt_snprintf(path, size, "%s%03d%s", CACHE_FILE_PREFIX, (int)idx, CACHE_FILE_SUFFIX);
}

static int read_packet_header(int fd, rt_uint8_t *status, rt_uint32_t *seq)
{
    rt_uint8_t header[7];
    if (read(fd, header, 7) != 7) return -1;

    if (header[0] != PACKET_FLAG_HEADER_1 || header[1] != PACKET_FLAG_HEADER_2)
        return -1;

    *status = header[2];
    memcpy(seq, &header[3], sizeof(rt_uint32_t));
    return 0;
}

static int update_packet_status(const char *path, rt_uint32_t offset, rt_uint8_t status)
{
    int fd = open(path, O_RDWR, 0);
    if (fd < 0) return -1;

    lseek(fd, offset + PACKET_OFFSET_STATUS, SEEK_SET);
    int ret = write(fd, &status, 1);
    close(fd);

    return (ret == 1) ? 0 : -1;
}

/* 二分查找定位未发送数据包 */
static int binary_search_unsent(rt_uint32_t start_file_idx, rt_uint32_t start_file_offset)
{
    return 0;
}

/* 读取文件指定偏移包序列号 */
static int get_file_seq_at(rt_uint32_t file_idx, rt_uint8_t packet_offset, rt_uint32_t *seq)
{
    char path[32];
    build_cache_path(path, sizeof(path), file_idx);
    int fd = open(path, O_RDONLY, 0);
    if (fd < 0) return -1;
    if(lseek(fd,packet_offset * ADC_PACKET_SIZE, SEEK_SET) != packet_offset * ADC_PACKET_SIZE) return -1;

    rt_uint8_t status;
    int ret = read_packet_header(fd, &status, seq);
    close(fd);
    return ret;
}

/* 二分查找最后一个存在的文件 */
static rt_uint32_t find_last_existing_file(void)
{
    rt_uint32_t left = 0, right = CACHE_FILE_COUNT - 1, result = 0;

    while (left <= right) {
        rt_uint32_t mid = (left + right) / 2;
        rt_uint32_t seq;

        if (get_file_seq(mid, &seq) == 0) {
            result = mid;
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }

    return result;
}

/* 查找环形缓存的起始文件位置 */
static int find_ring_start(rt_uint32_t *start_file)
{
    rt_uint32_t last_file = find_last_existing_file();
    rt_uint32_t first_seq, last_seq;

    if (get_file_seq_at(0, 0, &first_seq) < 0) {
        *start_file = 0;
        return 0;
    }

    if (last_file == 0 || get_file_seq_at(last_file, 0, &last_seq) < 0 || first_seq <= last_seq) {
        *start_file = 0;
        return 0;
    }

    rt_uint32_t left = 0, right = last_file;

    while (right - left > 10) {
        rt_uint32_t mid = (left + right) / 2;
        rt_uint32_t mid_seq;

        if (get_file_seq_at(mid, 0, &mid_seq) < 0 || mid_seq < first_seq) {
            right = mid;
        } else {
            left = mid;
        }
    }

    for (rt_uint32_t i = left; i < right; i++) {
        rt_uint32_t curr_seq, next_seq;
        if (get_file_seq_at(i, 0, &curr_seq) == 0 && get_file_seq_at(i + 1, 0, &next_seq) == 0) {
            if (next_seq < curr_seq) {
                *start_file = i + 1;
                return 0;
            }
        }
    }

    *start_file = 0;
    return 0;
}

static int find_start_offset(rt_uint32_t idx, rt_uint32_t *start_offset)
{
    char path[32];
    build_cache_path(path, sizeof(path), idx);
    int fd = open(path, O_RDONLY, 0);
    if (fd < 0) return -1;
    off_t file_size = lseek(fd, 0, SEEK_END);
    close(fd);
    if (file_size < 0) return -1;

    uint32_t n_packets = (uint32_t)(file_size / ADC_PACKET_SIZE);
    if (n_packets == 0) { *start_offset = 0; return 0; }

    uint32_t base_seq;
    if (get_file_seq_at(idx, 0, &base_seq) != 0) return -1;

    uint32_t left = 0;
    uint32_t right = n_packets;

    while (left < right) {
        uint32_t mid = left + (right - left) / 2;

        uint32_t s;
        if (get_file_seq_at(idx, mid, &s) != 0) return -1;

        if (s < base_seq) {
            right = mid;
        } else {
            left = mid + 1;
        }
    }
    *start_offset = (left == n_packets) ? 0 : left * ADC_PACKET_SIZE;
    return 0;
}

/* 初始化时恢复读写状态 */
static int recover_cache_state(void)
{
    rt_uint32_t ring_start_file = 0, ring_start_offset = 0;
    find_ring_start(&ring_start_file);
    cache_state.write_file_idx = ring_start_file;

    find_start_offset(ring_start_file, &ring_start_offset);
    cache_state.write_offset = ring_start_offset;

    /* 二分查找未发送数据包 */
    binary_search_unsent(ring_start_file, ring_start_offset);

    cache_state.read_file_idx = 0;
    cache_state.read_offset = 0;
    cache_state.last_write_seq = 0;

    LOG_I("Cache recovered: write[%d:%d] read[%d:%d]",
          cache_state.write_file_idx, cache_state.write_offset,
          cache_state.read_file_idx, cache_state.read_offset);

    return 0;
}

/* 写入数据到SD */
static int cache_write_blob_locked(const rt_uint8_t *data, rt_uint32_t len)
{
    rt_uint32_t written = 0;

    while (written < len) {
        char path[32];
        build_cache_path(path, sizeof(path), cache_state.write_file_idx);

        rt_uint32_t remain = len - written;
        rt_uint32_t space = CACHE_SEG_SIZE - cache_state.write_offset;
        rt_uint32_t chunk = (remain < space) ? remain : space;

        int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0);
        if (fd < 0) return -1;

        int n = write(fd, data + written, chunk);
        close(fd);

        if (n <= 0) return -1;

        written += n;
        cache_state.write_offset += n;

        if (cache_state.write_offset >= CACHE_SEG_SIZE) {
            cache_state.write_file_idx = (cache_state.write_file_idx + 1) % CACHE_FILE_COUNT;
            cache_state.write_offset = 0;
        }
    }

    return 0;
}

static void flush_ram_cache_to_sd(void)
{
    if (ram_buf_offset == 0) return;

    if (cache_write_blob_locked(sd_cache_ram_buf, ram_buf_offset) == 0)
    {
        ram_buf_offset = 0;
        last_flush_tick = rt_tick_get();
    }
}

/* 公共接口实现 */
int data_cache_init(void)
{
    rt_mutex_init(&cache_lock, "cache_mtx", RT_IPC_FLAG_PRIO);
    ram_buf_offset = 0;
    last_flush_tick = 0;

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    ts_spi_bus_claim();
    recover_cache_state();
    ts_spi_bus_release();
    rt_mutex_release(&cache_lock);

    return 0;
}

int data_cache_write(const uint8_t *data, uint32_t len)
{
    if (!data || len == 0) return -1;

    if (last_flush_tick == 0)
    {
        last_flush_tick = rt_tick_get();
    }

    if (len > RAM_CACHE_SIZE)
    {
        rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
        ts_spi_bus_claim();
        int ret = cache_write_blob_locked(data, len);
        ts_spi_bus_release();
        rt_mutex_release(&cache_lock);
        return ret;
    }
    else if (ram_buf_offset + len > RAM_CACHE_SIZE)
    {
        rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
        ts_spi_bus_claim();
        flush_ram_cache_to_sd();
        ts_spi_bus_release();
        rt_mutex_release(&cache_lock);
    }

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    if (ram_buf_offset + len <= RAM_CACHE_SIZE)
    {
        memcpy(sd_cache_ram_buf + ram_buf_offset, data, len);
        ram_buf_offset += len;
    }
    else{
        LOG_W("RAM is full, some data is missing.");
    }

    if (rt_tick_get() - last_flush_tick >= CACHE_FLUSH_INTERVAL_TICKS)
    {
        ts_spi_bus_claim();
        flush_ram_cache_to_sd();
        ts_spi_bus_release();
    }

    rt_mutex_release(&cache_lock);
    return 0;
}

int data_cache_read(uint8_t *buffer, uint32_t buffer_size)
{
    rt_uint32_t total_read = 0;

    if (!buffer || buffer_size == 0) return -1;

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    ts_spi_bus_claim();

    while (total_read < buffer_size)
    {
        if (cache_state.read_file_idx == cache_state.write_file_idx &&
            cache_state.read_offset == cache_state.write_offset) {
            break;
        }

        char path[32];
        build_cache_path(path, sizeof(path), cache_state.read_file_idx);

        int fd = open(path, O_RDONLY, 0);
        if (fd < 0) break;

        lseek(fd, cache_state.read_offset, SEEK_SET);

        rt_uint32_t need = buffer_size - total_read;
        rt_uint32_t chunk = (need < ADC_PACKET_MAX_SIZE) ? need : ADC_PACKET_MAX_SIZE;

        int n = read(fd, buffer + total_read, chunk);
        close(fd);

        if (n <= 0) break;

        total_read += n;
        cache_state.read_offset += n;

        if (cache_state.read_offset >= CACHE_SEG_SIZE) {
            cache_state.read_offset = 0;
            cache_state.read_file_idx = (cache_state.read_file_idx + 1) % CACHE_FILE_COUNT;
        }
    }

    ts_spi_bus_release();
    rt_mutex_release(&cache_lock);

    return (total_read > 0) ? (int)total_read : -1;
}

int data_cache_commit_read(uint32_t len)
{
    if (len == 0) return 0;

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    ts_spi_bus_claim();

    char path[32];
    build_cache_path(path, sizeof(path), cache_state.read_file_idx);
    update_packet_status(path, cache_state.read_offset - len, PACKET_FLAG_STATUS_SENT);

    ts_spi_bus_release();
    rt_mutex_release(&cache_lock);

    return 0;
}

bool data_cache_has_pending(void)
{
    bool has_data;

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    has_data = !(cache_state.read_file_idx == cache_state.write_file_idx &&
                 cache_state.read_offset == cache_state.write_offset);
    rt_mutex_release(&cache_lock);

    return has_data;
}

void data_cache_flush(void)
{
    if (ram_buf_offset == 0) return;
    if (rt_tick_get() - last_flush_tick >= CACHE_FLUSH_INTERVAL_TICKS)
    {
        rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
        ts_spi_bus_claim();

        flush_ram_cache_to_sd();

        ts_spi_bus_release();
        rt_mutex_release(&cache_lock);
    }
}

uint32_t data_cache_get_ram_usage(void)
{
    return ram_buf_offset;
}

void dump_cache_files(void)
{
    rt_kprintf("Cache State:\n");
    rt_kprintf("  Write: file[%d] offset[%d]\n", cache_state.write_file_idx, cache_state.write_offset);
    rt_kprintf("  Read:  file[%d] offset[%d]\n", cache_state.read_file_idx, cache_state.read_offset);
}

int sdnand_init_mount(void)
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

    int ret = dfs_mount("sdnand0", "/", "lfs", 0, 0);

    if (ret == 0)
        {
            LOG_I("littleFS mounted to /");
        }
        else
        {
            LOG_E("Mount failed! Error code: %d", ret);
            LOG_W("Attempting to format 'sdnand0' with littleFS...");

            if (dfs_mkfs("lfs", "sdnand0") == 0)
            {
                LOG_I("Format success, retrying mount...");
                rt_thread_mdelay(100);
                if (dfs_mount("sdnand0", "/", "lfs", 0, 0) == 0) {
                    LOG_I("Retry mount success!");
                } else {
                    LOG_E("Retry mount failed after format.");
                }
            }
            else
            {
                LOG_E("Format failed! Please check if the device supports MTD or block access.");
            }
        }

    ts_spi_bus_release();
    return RT_EOK;
}
INIT_DEVICE_EXPORT(sdnand_init_mount);
