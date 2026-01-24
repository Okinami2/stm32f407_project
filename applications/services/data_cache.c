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

#include "data_cache.h"
#include "../services/sd_spi_switch.h"

/* 缓存文件配置 */
#define CACHE_INDEX_PATH      "/cache.idx"
#define CACHE_FILE_PREFIX     "/cache_"
#define CACHE_FILE_SUFFIX     ".dat"

#define CACHE_FILE_COUNT      200
#define CACHE_SEG_SIZE        (16 * 1024 * 1024UL)
#define CACHE_MAGIC           0xA55A1234

/* RAM 缓存配置 */
#define RAM_CACHE_SIZE               (32U * 1024U)                    /* 32KB */
#define CACHE_FLUSH_INTERVAL_TICKS   (rt_tick_from_millisecond(5000)) /* 5s */

/* 缓存索引结构 */
struct cache_index
{
    rt_uint32_t magic;
    rt_uint16_t write_file_idx; /* 当前写入的文件编号 */
    rt_uint16_t read_file_idx;  /* 当前读取重发的文件编号 */
    rt_uint32_t write_off;      /* 当前写入的偏移量 */
    rt_uint32_t read_off;       /* 当前读取的偏移量 */
};

/* 静态变量 */
static struct rt_mutex cache_lock; /* 保护索引文件和内存缓冲 */
static rt_uint8_t  sd_cache_ram_buf[RAM_CACHE_SIZE];
static rt_uint32_t ram_buf_offset = 0;
static rt_tick_t   last_flush_tick = 0;

/* 内部辅助函数 */
static int write_all(int fd, const rt_uint8_t *buf, rt_uint32_t len)
{
    rt_uint32_t off = 0;
    while (off < len)
    {
        int n = write(fd, buf + off, len - off);
        if (n <= 0) return -1;
        off += (rt_uint32_t)n;
    }
    return 0;
}

static void save_index_locked(struct cache_index *idx)
{
    int ifd = open(CACHE_INDEX_PATH, O_RDWR | O_CREAT, 0666);

    if (ifd >= 0) {
        lseek(ifd, 0, SEEK_SET);
        write_all(ifd, (const rt_uint8_t *)idx, sizeof(struct cache_index));
        close(ifd);
    }
}

static int load_index_locked(struct cache_index *idx)
{
    int ifd = open(CACHE_INDEX_PATH, O_RDWR | O_CREAT, 0666);

    if (ifd < 0) return -1;
    int res = read(ifd, idx, sizeof(struct cache_index));
    close(ifd);
    if (res == sizeof(struct cache_index) && idx->magic == CACHE_MAGIC) return 0;
    return -1;
}

static int cache_write_blob_locked(const rt_uint8_t *data, rt_uint32_t len)
{
    struct cache_index idx, new_idx;

    if (load_index_locked(&idx) < 0)
    {
        rt_memset(&idx, 0, sizeof(idx));
        idx.magic = CACHE_MAGIC;
    }

    new_idx = idx;
    rt_uint16_t cur_idx = new_idx.write_file_idx;
    rt_uint32_t cur_off = new_idx.write_off;

    const rt_uint8_t *p = data;
    rt_uint32_t remain = len;

    while (remain > 0)
    {
        rt_uint32_t space = CACHE_SEG_SIZE - cur_off;
        if (space == 0)
        {
            cur_idx = (rt_uint16_t)((cur_idx + 1) % CACHE_FILE_COUNT);
            cur_off = 0;
            space = CACHE_SEG_SIZE;
        }

        rt_uint32_t chunk = (remain <= space) ? remain : space;

        char path[32];
        rt_snprintf(path, sizeof(path), "%s%03d%s", CACHE_FILE_PREFIX, (int)cur_idx, CACHE_FILE_SUFFIX);

        int dfd = open(path, O_RDWR | O_CREAT, 0666);
        if (dfd < 0)
        {
            rt_kprintf("[Cache] open %s failed\n", path);
            return -1;
        }

        if (lseek(dfd, cur_off, SEEK_SET) < 0)
        {
            rt_kprintf("[Cache] lseek failed\n");
            close(dfd);
            return -1;
        }

        if (write_all(dfd, p, chunk) < 0)
        {
            rt_kprintf("[Cache] write_all failed\n");
            close(dfd);
            return -1;
        }

        close(dfd);

        p += chunk;
        remain -= chunk;
        cur_off += chunk;
    }

    new_idx.write_file_idx = cur_idx;
    new_idx.write_off      = cur_off;

    save_index_locked(&new_idx);
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
    return 0;
}

int data_cache_write(const uint8_t *data, uint32_t len)
{
    if (!data || len == 0) return -1;

    if (last_flush_tick == 0)
    {
        last_flush_tick = rt_tick_get();
    }

    /* 如果数据超过 RAM 缓存大小，直接写入 SD */
    if (len > RAM_CACHE_SIZE)
    {
        rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
        ts_spi_bus_claim();
        int ret = cache_write_blob_locked(data, len);
        ts_spi_bus_release();
        rt_mutex_release(&cache_lock);
        return ret;
    }
    /* 如果 RAM 缓存不够，先刷新 */
    else if (ram_buf_offset + len > RAM_CACHE_SIZE)
    {
        rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
        ts_spi_bus_claim();
        flush_ram_cache_to_sd();
        ts_spi_bus_release();
        rt_mutex_release(&cache_lock);
    }

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    if (ram_buf_offset + len > RAM_CACHE_SIZE)
    {
        /* 写入 RAM 缓存 */
        memcpy(sd_cache_ram_buf + ram_buf_offset, data, len);
        ram_buf_offset += len;
    }
    else{
        rt_kprintf("[W/data_cache]RAM is full, some data is missing.");
    }

    /* 检查是否需要定时刷新 */
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
    struct cache_index idx;
    uint32_t total_read = 0;

    if (!buffer || buffer_size == 0) return -1;

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    ts_spi_bus_claim();

    if (load_index_locked(&idx) < 0) {
        goto _exit;
    }

    while (total_read < buffer_size)
    {
        if (idx.read_file_idx == idx.write_file_idx && idx.read_off == idx.write_off) {
            break;
        }
        uint32_t file_remain = 0;
        if (idx.read_file_idx == idx.write_file_idx) {
            if (idx.write_off < idx.read_off) {
                rt_kprintf("[W/data_cache]write_off < read_off\n");
                break;
            }
            file_remain = idx.write_off - idx.read_off;
        } else {
            file_remain = CACHE_SEG_SIZE - idx.read_off;
        }
        uint32_t need_read = buffer_size - total_read;
        uint32_t chunk = (need_read < file_remain) ? need_read : file_remain;

        char path[32];
        rt_snprintf(path, sizeof(path), "%s%03d%s", CACHE_FILE_PREFIX, (int)idx.read_file_idx, CACHE_FILE_SUFFIX);

        int fd = open(path, O_RDONLY, 0);
        if (fd >= 0) {
            lseek(fd, idx.read_off, SEEK_SET);
            int n = read(fd, buffer + total_read, chunk);
            close(fd);

            if (n <= 0) break;

            total_read += n;
            idx.read_off += n;

            if (idx.read_off >= CACHE_SEG_SIZE) {
                idx.read_off = 0;
                idx.read_file_idx = (idx.read_file_idx + 1) % CACHE_FILE_COUNT;
            }
        } else {
            break;
        }
    }

_exit:
    ts_spi_bus_release();
    rt_mutex_release(&cache_lock);

    return (total_read > 0) ? (int)total_read : -1;
}

int data_cache_commit_read(uint32_t len)
{
    struct cache_index idx;

    if (len == 0) return -1;

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    ts_spi_bus_claim();

    if (load_index_locked(&idx) < 0)
    {
        ts_spi_bus_release();
        rt_mutex_release(&cache_lock);
        return -1;
    }

    idx.read_off += len;
    while (idx.read_off >= CACHE_SEG_SIZE) {
        idx.read_off -= CACHE_SEG_SIZE;
        idx.read_file_idx = (idx.read_file_idx + 1) % CACHE_FILE_COUNT;
    }

    save_index_locked(&idx);

    ts_spi_bus_release();
    rt_mutex_release(&cache_lock);

    return 0;
}

bool data_cache_has_pending(void)
{
    struct cache_index idx;
    bool has_data;

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    ts_spi_bus_claim();

    if (load_index_locked(&idx) < 0)
    {
        has_data = false;
    }
    else
    {
        has_data = !(idx.read_file_idx == idx.write_file_idx && idx.read_off == idx.write_off);
    }

    ts_spi_bus_release();
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

        flush_ram_cache_to_sd(); // 这里的内部实现会更新 last_flush_tick

        ts_spi_bus_release();
        rt_mutex_release(&cache_lock);
    }
}

uint32_t data_cache_get_ram_usage(void)
{
    return ram_buf_offset;
}
