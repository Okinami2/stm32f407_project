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

#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netdev.h>

#include <unistd.h>
#include <fcntl.h>

#include "adc_get_thread.h"
#include "adc_send_thread.h"
#include "sd_spi_switch.h"

#define TX_THREAD_PRIO   20
#define RESEND_THREAD_PRIO  21  /* 低于主发送线程 */
#define TX_THREAD_STACK  4096

#define SERVER_IP        "192.168.137.1"
#define SERVER_PORT      9001
#define RECONNECT_INTERVAL_MS  5000

#define PACKET_DATA_SIZE        (BATCH_SIZE * (8 * sizeof(float)) + sizeof(sys_calendar_time_t) + sizeof(rt_uint8_t))

static uint8_t dma_tx_buffer[PACKET_DATA_SIZE + 16];


//static rt_device_t send_dev = RT_NULL;
static int sock_fd = -1; /* Socket 文件描述符 */

#define CACHE_INDEX_PATH      "/cache.idx"
#define CACHE_FILE_PREFIX     "/cache_"
#define CACHE_FILE_SUFFIX     ".dat"

#define CACHE_FILE_COUNT      200
#define CACHE_SEG_SIZE        (16 * 1024 * 1024UL)
#define CACHE_MAGIC           0xA55A1234

struct cache_index
{
    rt_uint32_t magic;
    rt_uint16_t write_file_idx; /* 当前写入的文件编号 */
    rt_uint16_t read_file_idx;  /* 当前读取重发的文件编号 */
    rt_uint32_t write_off;      /* 当前写入的偏移量 */
    rt_uint32_t read_off;       /* 当前读取的偏移量 */
};

static struct rt_mutex cache_lock; /* 保护索引文件和内存缓冲 */
static struct rt_mutex net_lock;   /* 保护 sock_fd 发送 */

#define CACHE_FLUSH_INTERVAL_TICKS   (rt_tick_from_millisecond(5000)) /* 5s */
#define RAM_CACHE_SIZE               (32U * 1024U)                    /* 32KB */

#define PACKET_FULL_LEN   (PACKET_DATA_SIZE + 4)

static rt_uint8_t  sd_cache_ram_buf[RAM_CACHE_SIZE];
static rt_uint32_t ram_buf_offset = 0;
static rt_tick_t   last_flush_tick = 0;



bool is_network_link_up(void)
{
    struct netdev *netdev = netdev_get_by_name("e0");
    if (netdev)
    {
        return netdev_is_link_up(netdev);
    }
    return false;
}


static void tcp_close_socket(void)
{
    rt_mutex_take(&net_lock, RT_WAITING_FOREVER);
    if (sock_fd >= 0)
    {
        closesocket(sock_fd);
        sock_fd = -1;
    }
    rt_mutex_release(&net_lock);
}


/**
 * @brief 尝试连接服务器
 * @return 0: 成功, -1: 失败
 */
static int tcp_connect_to_server(void)
{

    struct sockaddr_in server_addr;
    struct timeval timeout;

    static rt_tick_t last_try_tick = 0;
    rt_tick_t now_tick = rt_tick_get();

    if ((now_tick - last_try_tick) < rt_tick_from_millisecond(RECONNECT_INTERVAL_MS))
    {
        return -1;
    }

    last_try_tick = now_tick;


    if (sock_fd >= 0)
    {
        tcp_close_socket();
    }

    if (!is_network_link_up())
    {
        return -1;
    }

    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0)
    {
        rt_kprintf("[Network] Socket create failed!\n");
        return -1;
    }

    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    setsockopt(sock_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

    if (connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) < 0)
    {
        // rt_kprintf("[Network] Connect failed!\n");
        tcp_close_socket();
        return -1;
    }

    rt_kprintf("[Network] Connected to %s:%d\n", SERVER_IP, SERVER_PORT);
    return 0;
}

/**
 * @brief 发送完整数据包
 * @param data 数据指针
 * @param len 数据长度
 * @return 发送成功的字节数，如果失败返回 -1
 */
static int tcp_send_packet(const uint8_t *data, uint32_t len)
{
    int total_sent = 0;
    int sent_bytes;


    if (sock_fd < 0) return -1;

    rt_mutex_take(&net_lock, RT_WAITING_FOREVER);

    while (total_sent < len)
    {
        sent_bytes = send(sock_fd, data + total_sent, len - total_sent, 0);
        if (sent_bytes <= 0)
        {
            rt_kprintf("[Network] Send failed, error: %d\n", sent_bytes);

            rt_mutex_release(&net_lock);
            return -1;
        }
        total_sent += sent_bytes;
    }

    rt_mutex_release(&net_lock);
    return total_sent;
}


/**
 * @brief 将分散的 ADC 数据打包到连续的内存中以便 DMA 发送
 * @param start_index 数据在 buffer 中的起始索引 (0 或 BATCH_SIZE)
 * @return 打包后的数据长度
 */
static uint32_t pack_data_to_buffer(uint16_t start_index)
{
    uint32_t offset = 0;

    //添加包头
    dma_tx_buffer[offset++] = 0xAA;
    dma_tx_buffer[offset++] = 0x55;
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.start_time, sizeof(sys_calendar_time_t));
    offset += sizeof(sys_calendar_time_t);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.sample_rate, sizeof(rt_uint8_t));
    offset += sizeof(rt_uint8_t);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad0[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad1[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad2[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad3[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad4[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad5[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad6[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);
    memcpy(&dma_tx_buffer[offset], &adc_receive_buffer.ad7[start_index], BATCH_SIZE * sizeof(float));
    offset += BATCH_SIZE * sizeof(float);

    //添加包尾
    dma_tx_buffer[offset++] = 0x0D;
    dma_tx_buffer[offset++] = 0x0A;

    return offset;
}

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

/**
 * @brief 更新并保存索引 (内部调用，需已持有 cache_lock)
 */
static void save_index_locked(struct cache_index *idx)
{
    int ifd = open(CACHE_INDEX_PATH, O_RDWR | O_CREAT, 0666);

    if (ifd >= 0) {
        lseek(ifd, 0, SEEK_SET);
        write_all(ifd, (const rt_uint8_t *)idx, sizeof(struct cache_index));
        close(ifd);
    }
}

/**
 * @brief 获取当前索引 (内部调用，需已持有 cache_lock)
 */
static int load_index_locked(struct cache_index *idx)
{
    int ifd = open(CACHE_INDEX_PATH, O_RDWR | O_CREAT, 0666);

    if (ifd < 0) return -1;
    int res = read(ifd, idx, sizeof(struct cache_index));
    close(ifd);
    if (res == sizeof(struct cache_index) && idx->magic == CACHE_MAGIC) return 0;
    return -1;
}

/**
 * @brief 将一段连续数据写入 SD NAND 缓存文件（一次性提交索引）
 * @note 该函数假设已 ts_spi_bus_claim()；写入成功后才更新索引，避免断电导致索引前移。
 * @return 0: 成功, -1: 失败
 */
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

/**
 * @brief 将 RAM 缓冲区中的数据一次性刷入 SD NAND
 */
static void flush_ram_cache_to_sd(void)
{
    if (ram_buf_offset == 0) return;

    rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
    ts_spi_bus_claim();
    if (cache_write_blob_locked(sd_cache_ram_buf, ram_buf_offset) == 0)
    {
        ram_buf_offset = 0;
        last_flush_tick = rt_tick_get();
    }
    ts_spi_bus_release();
    rt_mutex_release(&cache_lock);
}

/**
 * @brief 发送失败时：先攒到 RAM，满足“满/到时”再刷盘
 */
static void save_to_sdnand(uint8_t *data, uint32_t len)
{
    if (!data || len == 0) return;

    if (last_flush_tick == 0)
    {
        last_flush_tick = rt_tick_get();
    }

    if (len > RAM_CACHE_SIZE)
    {
        rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
        ts_spi_bus_claim();
        (void)cache_write_blob_locked(data, len);
        ts_spi_bus_release();
        rt_mutex_release(&cache_lock);
        return;
    }

    if (ram_buf_offset + len > RAM_CACHE_SIZE)
    {
        flush_ram_cache_to_sd();
    }

    memcpy(sd_cache_ram_buf + ram_buf_offset, data, len);
    ram_buf_offset += len;

    if (rt_tick_get() - last_flush_tick >= CACHE_FLUSH_INTERVAL_TICKS)
    {
        flush_ram_cache_to_sd();
    }
}


/**
 * @brief 发送线程入口函数
 */
static void send_to_server_thread_entry(void *parameter)
{
    uint16_t start_index;
    uint32_t packet_len;

    tcp_connect_to_server();

    while (1)
    {

        if (sock_fd < 0)
        {
            tcp_connect_to_server();
        }

        if (rt_sem_take(adc_get_done_sem, rt_tick_from_millisecond(1000)) == RT_EOK)
        {
            if (receive_buff_flag == true) {
                start_index = 0;
            } else {
                start_index = BATCH_SIZE;
            }

            packet_len = pack_data_to_buffer(start_index);
            // rt_kprintf("[Send] Packed len: %d\n", packet_len);

            bool send_success = false;

            if (sock_fd < 0)
            {
                tcp_connect_to_server();
            }

            if (sock_fd >= 0)
            {
                if (tcp_send_packet(dma_tx_buffer, packet_len) > 0)
                {
                    send_success = true;
                }
                else
                {
                    rt_kprintf("[Network] Send failed, closing socket.\n");
                    tcp_close_socket();
                    send_success = false;
                }
            }

            if (!send_success)
            {
                save_to_sdnand(dma_tx_buffer, packet_len);
            }
        }

        if (ram_buf_offset > 0 && last_flush_tick != 0 &&
            (rt_tick_get() - last_flush_tick >= CACHE_FLUSH_INTERVAL_TICKS))
        {
            flush_ram_cache_to_sd();
        }
    }
}

static void adc_resend_thread_entry(void *parameter)
{
    struct cache_index idx;
    uint8_t *read_buf = rt_malloc(PACKET_FULL_LEN);
    if (!read_buf) return;

    while (1)
    {
        rt_thread_mdelay(1000);

        if (sock_fd < 0 || !is_network_link_up()) continue;

        rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
        ts_spi_bus_claim();
        if (load_index_locked(&idx) < 0) {
            ts_spi_bus_release();
            rt_mutex_release(&cache_lock);
            continue;
        }
        /* 调试是否重发用
        rt_kprintf("[RESEND] Found cache: r(%d,%u) -> w(%d,%u)\n",
                    idx.read_file_idx, idx.read_off,
                    idx.write_file_idx, idx.write_off);
                    */

        if (idx.read_file_idx == idx.write_file_idx && idx.read_off == idx.write_off) {
            ts_spi_bus_release();
            rt_mutex_release(&cache_lock);
            continue;
        }

        char path[32];
        rt_snprintf(path, sizeof(path), "%s%03d%s", CACHE_FILE_PREFIX, (int)idx.read_file_idx, CACHE_FILE_SUFFIX);

        ts_spi_bus_claim();
        int fd = open(path, O_RDONLY, 0);
        int read_bytes = -1;
        if (fd >= 0) {
            lseek(fd, idx.read_off, SEEK_SET);
            read_bytes = read(fd, read_buf, PACKET_FULL_LEN);
            close(fd);
        }
        ts_spi_bus_release();
        rt_mutex_release(&cache_lock);

        if (read_bytes == PACKET_FULL_LEN)
        {
            if (tcp_send_packet(read_buf, PACKET_FULL_LEN) == PACKET_FULL_LEN)
            {
                rt_mutex_take(&cache_lock, RT_WAITING_FOREVER);
                ts_spi_bus_claim();
                idx.read_off += PACKET_FULL_LEN;
                if (idx.read_off >= CACHE_SEG_SIZE)
                {
                    idx.read_off = 0;
                    idx.read_file_idx = (idx.read_file_idx + 1) % CACHE_FILE_COUNT;
                }
                save_index_locked(&idx);

                ts_spi_bus_release();
                rt_mutex_release(&cache_lock);
            }
            else
            {
                /* 发送失败就关 socket，让主线程去重连 */
                tcp_close_socket();
                rt_thread_mdelay(2000);
            }
        }
    }
}

/**
 * @brief 启动发送线程
 */
int adc_send_to_server_start(void)
{
    rt_mutex_init(&cache_lock, "cache_mtx", RT_IPC_FLAG_PRIO);
    rt_mutex_init(&net_lock, "net_mtx", RT_IPC_FLAG_PRIO);

    rt_thread_t tid;

    // 主发送线程
    tid = rt_thread_create("adc_send", send_to_server_thread_entry, RT_NULL, 4096, 20, 10);
    if (tid) rt_thread_startup(tid);

    // 重发线程
    tid = rt_thread_create("adc_resend", adc_resend_thread_entry, RT_NULL, 4096, 21, 10);
    if (tid) rt_thread_startup(tid);

    return RT_EOK;
}
// INIT_APP_EXPORT(adc_send_to_server_start);
