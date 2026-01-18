/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-12-09     GreatMagicianGarfiel       the first version
 */

#include <rtthread.h>
#include <inttypes.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include "bsp/time_service.h"
#include "config_thread.h"

#define DBG_TAG "ntp_task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define NTP_SERVER          "ntp.aliyun.com"
#define NTP_PORT            123
#define NTP_TIMESTAMP_DELTA 2208988800ULL
#define NTP_ITERATIONS      8
#define NTP_INTERVAL_MS     100
#define NTP_TIMEOUT_MS      2000
#define DEFAULT_SYNC_INTERVAL_SECONDS 60*60

typedef struct __attribute__((packed)) {
    uint8_t  li_vn_mode;
    uint8_t  stratum;
    uint8_t  poll;
    int8_t   precision;
    uint32_t root_delay;
    uint32_t root_dispersion;
    uint32_t ref_id;
    uint32_t ref_ts_sec;
    uint32_t ref_ts_frac;
    uint32_t orig_ts_sec;
    uint32_t orig_ts_frac;
    uint32_t recv_ts_sec;
    uint32_t recv_ts_frac;
    uint32_t trans_ts_sec;
    uint32_t trans_ts_frac;
} ntp_packet_t;

static int64_t ntp_to_us(uint32_t sec, uint32_t frac)
{
    return (int64_t)(sec - NTP_TIMESTAMP_DELTA) * 1000000LL +
           ((int64_t)frac * 1000000LL >> 32);
}


static int ntp_request(int sock, struct sockaddr_in *addr,
                       int64_t *offset_us, int64_t *delay_us)
{
    ntp_packet_t pkt;
    int64_t t1, t2, t3, t4;
    int ret;

    memset(&pkt, 0, sizeof(pkt));
    pkt.li_vn_mode = (4 << 3) | 3;  /* VN=4, Mode=3(client) */

    Timestamp_t ts;
    ts_get_time(&ts);
    t1 = (int64_t)ts.sec * 1000000LL + (int64_t)ts.usec;

    ret = sendto(sock, &pkt, sizeof(pkt), 0,
                     (struct sockaddr *)addr, sizeof(*addr));
        if (ret < 0)
            return -1;

        /* recvfrom auto-returns on timeout (set via setsockopt) */
        ret = recvfrom(sock, &pkt, sizeof(pkt), 0, NULL, NULL);
        if (ret < 48)
            return -1;

    ts_get_time(&ts);
    t4 = (int64_t)ts.sec * 1000000LL + (int64_t)ts.usec;

    t2 = ntp_to_us(ntohl(pkt.recv_ts_sec), ntohl(pkt.recv_ts_frac));
    t3 = ntp_to_us(ntohl(pkt.trans_ts_sec), ntohl(pkt.trans_ts_frac));

    /* offset = ((T2-T1) + (T3-T4)) / 2 */
    *offset_us = ((t2 - t1) + (t3 - t4)) / 2;
    /* delay = (T4-T1) - (T3-T2) */
    *delay_us = (t4 - t1) - (t3 - t2);

    return 0;
}

static int cmp_delay(const void *a, const void *b)
{
    const int64_t *sa = (const int64_t *)a;
    const int64_t *sb = (const int64_t *)b;
    if (sa[1] < sb[1]) return -1;
    if (sa[1] > sb[1]) return 1;
    return 0;
}

/**
 * @brief Perform NTP time synchronization
 * @return 0 on success, -1 on failure
 */
int ntp_sync(void)
{
    int sock;
    struct sockaddr_in addr;
    struct hostent *host;
    int64_t samples[NTP_ITERATIONS][2];  /* [offset_us, delay_us] */
    int valid = 0;
    int i;
    struct timeval timeout;

    host = gethostbyname(NTP_SERVER);
    if (!host)
    {
        rt_kprintf("NTP: resolve failed\n");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(NTP_PORT);
    memcpy(&addr.sin_addr, host->h_addr, host->h_length);

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        return -1;

    timeout.tv_sec = NTP_TIMEOUT_MS / 1000;
    timeout.tv_usec = (NTP_TIMEOUT_MS % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    for (i = 0; i < NTP_ITERATIONS; i++)
    {
        if (ntp_request(sock, &addr, &samples[valid][0], &samples[valid][1]) == 0)
        {
            valid++;
        }
        if (i < NTP_ITERATIONS - 1)
            rt_thread_mdelay(NTP_INTERVAL_MS);
    }

    closesocket(sock);

    if (valid < 3)
    {
        rt_kprintf("NTP: too few samples (%d)\n", valid);
        return -1;
    }

    qsort(samples, valid, sizeof(samples[0]), cmp_delay);

    int use_count = (valid >= 3) ? 3 : valid;
    int64_t avg_offset = 0;
    for (i = 0; i < use_count; i++)
        avg_offset += samples[i][0];
    avg_offset /= use_count;

    int32_t off_sec  = (int32_t)(avg_offset / 1000000LL);
    int32_t off_usec = (int32_t)(avg_offset % 1000000LL);
    if (off_usec < 0) { off_sec -= 1; off_usec += 1000000; }

    /* Calculate standard deviation */
    double stddev = 0;
    for (i = 0; i < use_count; i++)
    {
        double diff = (double)samples[i][0] - (double)avg_offset;
        stddev += diff * diff;
    }
    stddev = sqrt(stddev / use_count);
    int stddev_i = (int)stddev;
    int stddev_d = abs((int)((stddev - stddev_i) * 1000));

    rt_kprintf("NTP: final offset = %lld us (delay=%lld us) stddev = %d.%05d us\n",
               (long long)avg_offset, (long long)samples[0][1], stddev_i,stddev_d);

    ts_correct_time_by_ntp_offset_us(avg_offset);

    return 0;
}

/**
 * @brief NTP sync thread entry
 * @param parameter Unused
 */
static void ntp_sync_thread_entry(void *parameter)
{
    rt_tick_t last_update_tick = rt_tick_get();
    rt_tick_t interval_ticks = RT_TICK_PER_SECOND * DEFAULT_SYNC_INTERVAL_SECONDS;

    rt_thread_mdelay(5000);

    while (1)
    {
        config_update_msg_t msg;
        rt_tick_t now = rt_tick_get();
        rt_int32_t wait_ticks;

        rt_tick_t elapsed = now - last_update_tick;
        if (elapsed >= interval_ticks) {
            wait_ticks = 0;
        } else {
            wait_ticks = interval_ticks - elapsed;
        }

        rt_mq_recv(config_ntp_update_notify, &msg, sizeof(msg), wait_ticks);

        if (ntp_sync() == 0) {
        } else {
            rt_kprintf("ntp sync failed. please check the network.\n");
        }
        last_update_tick = rt_tick_get();
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
