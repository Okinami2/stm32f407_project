/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-10     GreatMagicianGarfiel       the first version
 * 2025-12-01     GreatMagicianGarfiel       Implement Read/Write & Alert handling
 */

#include "max40109_hal.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

#define DBG_TAG "max40109.hal"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct rt_i2c_bus_device *i2c_bus;

/* Alert pins - only channel 0 populated, others set to 0
static const rt_base_t alert_pins[NUM_MAX_CHIPS] = {
    BSP_nCH0_ALERT_PIN,
    BSP_nCH1_ALERT_PIN,
    BSP_nCH2_ALERT_PIN,
    BSP_nCH3_ALERT_PIN,
    BSP_nCH4_ALERT_PIN,
    BSP_nCH5_ALERT_PIN,
    BSP_nCH6_ALERT_PIN,
    BSP_nCH7_ALERT_PIN
};
*/
static const rt_base_t alert_pins[NUM_MAX_CHIPS] = {
    BSP_nCH0_ALERT_PIN,
    0,
    0,
    0,
    0,
    0,
    0,
    0
};

static struct rt_mutex i2c_mux_lock; /* Prevent concurrent I2C operations */
static rt_sem_t alert_sem = RT_NULL;
static volatile rt_uint8_t g_triggered_channels_mask = 0;


/* Thread stack and handle */
#define ALERT_THREAD_STACK_SIZE 2024
#define ALERT_THREAD_PRIORITY   15


/* Interrupt Handler */
static void alert_irq_handler(void *args)
{
    rt_uint8_t channel = (rt_uint8_t)(rt_ubase_t)args;

    rt_enter_critical();
    g_triggered_channels_mask |= (1 << channel);
    rt_exit_critical();

    rt_sem_release(alert_sem);
}


/**
 * @brief Select MAX40109 chip via I2C mux
 * @param chip_index Chip index (0-7)
 */
static void _select_max_chip(rt_uint8_t chip_index)
{
    if (chip_index >= NUM_MAX_CHIPS) return;

    rt_pin_write(BSP_I2C_3_8_BIT1_PIN, (chip_index & 0x01) ? PIN_HIGH : PIN_LOW);
    rt_pin_write(BSP_I2C_3_8_BIT2_PIN, (chip_index & 0x02) ? PIN_HIGH : PIN_LOW);
    rt_pin_write(BSP_I2C_3_8_BIT3_PIN, (chip_index & 0x04) ? PIN_HIGH : PIN_LOW);

    rt_hw_us_delay(20);
}

/**
 * @brief Internal unified I2C transfer interface
 * @param chip_idx Target chip index
 * @param msgs I2C message array
 * @param num Number of messages
 * @return RT_EOK on success, error code on failure
 */
static rt_err_t _max40109_transfer(rt_uint8_t chip_idx, struct rt_i2c_msg *msgs, rt_uint32_t num)
{
    rt_err_t res;

    if (rt_mutex_take(&i2c_mux_lock, rt_tick_from_millisecond(100)) != RT_EOK)
    {
        return -RT_ETIMEOUT;
    }

    _select_max_chip(chip_idx);

    /* Multiple msgs trigger Repeated Start automatically */
    if (rt_i2c_transfer(i2c_bus, msgs, num) == num)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_EIO;
    }

    rt_mutex_release(&i2c_mux_lock);
    return res;
}

/**
 * @brief Generic register write function
 * @param chip_idx Chip index
 * @param reg Register address
 * @param val Value to write
 * @param len 1 for single byte, 2 for double byte
 * @return RT_EOK on success, error code on failure
 */
rt_err_t max40109_write_reg(rt_uint8_t chip_idx, rt_uint8_t reg, rt_uint16_t val, rt_uint8_t len)
{
    struct rt_i2c_msg msg;
    rt_uint8_t buf[3];

    buf[0] = reg;
    if (len == 2)
    {
        buf[1] = (rt_uint8_t)(val >> 8);
        buf[2] = (rt_uint8_t)(val & 0xFF);
    }
    else
    {
        buf[1] = (rt_uint8_t)(val & 0xFF);
    }

    msg.addr  = MAX40109_I2C_ADDR;
    msg.flags = RT_I2C_WR;
    msg.buf   = buf;
    msg.len   = len + 1;

    return _max40109_transfer(chip_idx, &msg, 1);
}

/**
 * @brief Generic register read function
 * @param chip_idx Chip index
 * @param reg Register address
 * @param val Pointer to store read value
 * @param len 1 for single byte, 2 for double byte
 * @return RT_EOK on success, error code on failure
 */
rt_err_t max40109_read_reg(rt_uint8_t chip_idx, rt_uint8_t reg, rt_uint16_t *val, rt_uint8_t len)
{
    struct rt_i2c_msg msgs[2];
    rt_uint8_t read_buf[2] = {0};
    rt_err_t res;

    msgs[0].addr  = MAX40109_I2C_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = MAX40109_I2C_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = read_buf;
    msgs[1].len   = len;

    res = _max40109_transfer(chip_idx, msgs, 2);

    if (res == RT_EOK)
    {
        if (len == 2)
        {
            *val = (rt_uint16_t)((read_buf[0] << 8) | read_buf[1]);
        }
        else
        {
            *val = (rt_uint16_t)read_buf[0];
        }
    }
    else
    {
        *val = 0;
    }

    return res;
}

static inline rt_err_t max40109_write_u16(rt_uint8_t idx, rt_uint8_t reg, rt_uint16_t val) {
    return max40109_write_reg(idx, reg, val, 2);
}

static inline rt_err_t max40109_write_u8(rt_uint8_t idx, rt_uint8_t reg, rt_uint8_t val) {
    return max40109_write_reg(idx, reg, (rt_uint16_t)val, 1);
}

static inline rt_err_t max40109_read_u16(rt_uint8_t idx, rt_uint8_t reg, rt_uint16_t *val) {
    return max40109_read_reg(idx, reg, val, 2);
}

static inline rt_err_t max40109_read_u8(rt_uint8_t idx, rt_uint8_t reg, rt_uint8_t *val_u8) {
    rt_uint16_t temp;
    rt_err_t res = max40109_read_reg(idx, reg, &temp, 1);
    *val_u8 = (rt_uint8_t)temp;
    return res;
}

rt_err_t global_max40109_write_reg(rt_uint8_t chip_idx, rt_uint8_t reg, rt_uint16_t val,rt_uint8_t len){
    rt_err_t res = max40109_write_reg(chip_idx,reg,val,len);
    if(res != RT_EOK){
        return -RT_ERROR;
    }
    rt_uint16_t current_val;
    max40109_read_reg(chip_idx,reg,&current_val,len);

    rt_kprintf("current value: Chip[%d] [0x%X] = Value[0x%X]\n",chip_idx, reg,current_val);
    return RT_EOK;
}

rt_err_t global_max40109_read_reg(rt_uint8_t chip_idx, rt_uint8_t reg, rt_uint16_t *val,rt_uint8_t len){
    rt_uint16_t current_val;
    max40109_read_reg(chip_idx,reg,&current_val,len);
    *val = current_val;

    return RT_EOK;
}

/**
 * @brief Handle chip alerts from status register
 * @param chip_idx Chip index
 */
static void handle_chip_alert(rt_uint8_t chip_idx)
{
    rt_uint16_t status = 0;
    rt_err_t ret;

    ret = max40109_read_u16(chip_idx, MAX40109_REG_STATUS, &status);
    if (ret != RT_EOK) return;

    if (status == 0) return;

    if (status & (MAX_STATUS_INT_FAULT | MAX_STATUS_DRV_FAULT))
    {
        LOG_W("CH%d Critical Fault: 0x%04X", chip_idx, status);
    }
    if (status & MAX_STATUS_OV_INP_POS)
    {
        LOG_W("CH%d MAX_STATUS_OV_INP_POS: 0x%04X", chip_idx, status);
    }
    if (status & MAX_STATUS_UV_INP_POS)
    {
        LOG_W("CH%d MAX_STATUS_UV_INP_POS: 0x%04X", chip_idx, status);
    }
    if (status & MAX_STATUS_OV_INP_NEG)
    {
        LOG_W("CH%d MAX_STATUS_OV_INP_NEG: 0x%04X", chip_idx, status);
    }
    if (status & MAX_STATUS_UV_INP_NEG)
    {
        LOG_W("CH%d MAX_STATUS_UV_INP_NEG: 0x%04X", chip_idx, status);
    }
    if (status & MAX_STATUS_OV_INT)
    {
        LOG_W("CH%d MAX_STATUS_OV_INT: 0x%04X", chip_idx, status);
    }
    if (status & MAX_STATUS_UV_INT)
    {
        LOG_W("CH%d MAX_STATUS_UV_INT: 0x%04X", chip_idx, status);
    }
    if (status & MAX_STATUS_OV_DRV)
    {
        LOG_W("CH%d MAX_STATUS_OV_DRV: 0x%04X", chip_idx, status);
    }
    if (status & MAX_STATUS_UV_DRV)
    {
        LOG_W("CH%d MAX_STATUS_UV_DRV: 0x%04X", chip_idx, status);
    }

    if (status & MAX_STATUS_PRESS_READY)
    {
    }

    /* Clear status */
    max40109_write_u16(chip_idx, MAX40109_REG_STATUS, status);
}

/**
 * @brief Alert processing thread entry
 * @param parameter Unused
 */
static void alert_thread_entry(void *parameter)
{
    while (1)
    {
        if (rt_sem_take(alert_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            rt_uint8_t local_mask;
            rt_base_t level = rt_hw_interrupt_disable();
            local_mask = g_triggered_channels_mask;
            g_triggered_channels_mask = 0;
            rt_hw_interrupt_enable(level);
            if (local_mask == 0) continue;
            for (int i = 0; i < NUM_MAX_CHIPS; i++)
            {
                if (local_mask & (1 << i))
                {
                    handle_chip_alert(i);
                }
            }
        }
    }
}

/**
 * @brief Initialize all MAX40109 chips
 * @return RT_EOK on success, error code on failure
 */
rt_err_t max_app_init(void)
{

    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(I2C_BUS_NAME);
    if (i2c_bus == RT_NULL)
    {
        LOG_E("Can't find %s device!", I2C_BUS_NAME);
        return -RT_ENOSYS;
    }


    rt_pin_mode(BSP_ADC_POWER_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_VCC_5V_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_ANPWR_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_VCC_P_15_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_VCC_N_15_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_I2C_3_8_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_I2C_3_8_BIT1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_I2C_3_8_BIT2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_I2C_3_8_BIT3_PIN, PIN_MODE_OUTPUT);

    rt_pin_write(BSP_ADC_POWER_EN_PIN, PIN_HIGH);
    rt_pin_write(BSP_VCC_5V_EN_PIN, PIN_HIGH);
    rt_pin_write(BSP_ANPWR_EN_PIN, PIN_HIGH);
    rt_pin_write(BSP_VCC_P_15_PIN, PIN_HIGH);
    rt_pin_write(BSP_VCC_N_15_PIN, PIN_HIGH);
    rt_pin_write(BSP_I2C_3_8_EN_PIN, PIN_HIGH);

    rt_thread_mdelay(100);

    rt_mutex_init(&i2c_mux_lock, "max_mux", RT_IPC_FLAG_FIFO);
    alert_sem = rt_sem_create("max_sem", 0, RT_IPC_FLAG_FIFO);
    rt_thread_t tid = rt_thread_create("max_thread",
                                        alert_thread_entry,
                                        RT_NULL,
                                        ALERT_THREAD_STACK_SIZE,
                                        ALERT_THREAD_PRIORITY,
                                        10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    else
        return -RT_ENOMEM;


    for (int i = 0; i < NUM_MAX_CHIPS; i++)
    {
        if (alert_pins[i] != 0)
        {
            rt_pin_mode(alert_pins[i], PIN_MODE_INPUT_PULLUP);
            rt_pin_attach_irq(alert_pins[i], PIN_IRQ_MODE_FALLING, alert_irq_handler, (void*)(rt_ubase_t)i);
        }

        rt_err_t res = RT_EOK;

        /* Shutdown to config register */
        res = max40109_write_u16(i, MAX40109_REG_CONFIG, 0x80);

        /* Set bridge drive to 4V */
        res = max40109_write_u8(i, MAX40109_REG_BRIDGE_DRIVE, 0x2);

        /* Enable interrupts (default disables temp/pressure ready) */
        res = max40109_write_u16(i, MAX40109_REG_INTERRUPT_ENABLE, 0xFF);

        /* Set sample rate to 2ksps */
        res = max40109_write_u8(i, MAX40109_REG_ADC_SAMPLE_RATE, 0x2);

        /* Set analog output stage: absolute voltage, internal resistor */
        res = max40109_write_u8(i, MAX40109_REG_ANALOG_OUTPUT_STAGE, 0x8);

        /* Set pga = 144 */
        res = max40109_write_u8(i, MAX40109_REG_PGA_PRESSURE_GAIN, 0x07);

        /* Clear any pending status bits at startup */
        rt_uint16_t dummy;
        if (max40109_read_u16(i, MAX40109_REG_STATUS, &dummy) == RT_EOK && dummy != 0)
        {
            max40109_write_u16(i, MAX40109_REG_STATUS, dummy);
        }

        /* Power on */
        res = max40109_write_u16(i, MAX40109_REG_CONFIG, 0x2001);

        if(res != RT_EOK){
            LOG_W("CH[%d] max40109 config init failed",i);
            continue;
        }

        rt_thread_mdelay(150);

        if (alert_pins[i] != 0)
        {
            rt_pin_irq_enable(alert_pins[i], PIN_IRQ_ENABLE);
        }


    }

    LOG_I("MAX40109 Init Completed", NUM_MAX_CHIPS);
    return RT_EOK;
}

rt_err_t max40109_read_pressure(rt_uint8_t chip_idx, double *pressure,uint8_t is_calibrated)
{
    rt_uint16_t raw;
    rt_err_t ret = RT_EOK;
    //rt_err_t ret = max40109_read_u16(chip_idx, MAX40109_REG_UNCAL_PRESSURE, &raw);
    if(is_calibrated){
        ret = max40109_read_u16(chip_idx, MAX40109_REG_CALIBRATED_PRESSURE, &raw);
    }
    else{
        ret = max40109_read_u16(chip_idx, MAX40109_REG_UNCAL_PRESSURE, &raw);
    }
    if (ret != RT_EOK) {
        return ret;
    }

    int16_t signed_raw = (int16_t)raw;

    *pressure = ((double)signed_raw * 1250000 / 32767.0 / 90);

    return RT_EOK;
}
