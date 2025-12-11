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
#include "drv_soft_i2c.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

#define DBG_TAG "max40109.hal"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct rt_i2c_bus_device *i2c_bus;

/* Array of alert pins for each MAX40109 chip */
static const rt_base_t alert_pins[NUM_MAX_CHIPS] = {
    BSP_nCH0_ALERT_PIN,   // CH0
    BSP_nCH1_ALERT_PIN,   // CH1
    BSP_nCH2_ALERT_PIN,   // CH2
    BSP_nCH3_ALERT_PIN,   // CH3
    BSP_nCH4_ALERT_PIN,   // CH4
    BSP_nCH5_ALERT_PIN,   // CH5
    BSP_nCH6_ALERT_PIN,   // CH6 - Placeholder or Valid
    BSP_nCH7_ALERT_PIN    // CH7
};

/* Semaphore for alert handling */
static rt_sem_t alert_sem = RT_NULL;
/* Mask to track triggered channels, protected by interrupt disable/enable */
static volatile rt_uint8_t g_triggered_channels_mask = 0;

/* Thread stack and handle */
#define ALERT_THREAD_STACK_SIZE 1024
#define ALERT_THREAD_PRIORITY   15
static rt_thread_t alert_thread = RT_NULL;
static rt_uint8_t alert_stack[ALERT_THREAD_STACK_SIZE];

// Static function to select the active MAX40109 chip
static void select_max_chip(rt_uint8_t chip_index)
{
    if (chip_index >= NUM_MAX_CHIPS) return;

    rt_pin_write(BSP_I2C_3_8_BIT1_PIN, (chip_index & 0x01) ? PIN_HIGH : PIN_LOW); // LSB
    rt_pin_write(BSP_I2C_3_8_BIT2_PIN, (chip_index & 0x02) ? PIN_HIGH : PIN_LOW);
    rt_pin_write(BSP_I2C_3_8_BIT3_PIN, (chip_index & 0x04) ? PIN_HIGH : PIN_LOW); // MSB

    rt_hw_us_delay(20);
}

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
 * @brief Reads a 16-bit register from MAX40109 (Big Endian)
 */
static rt_err_t max40109_read_reg(rt_uint8_t chip_idx, rt_uint8_t reg, rt_uint16_t *val)
{
    struct rt_i2c_msg msgs[2];
    rt_uint8_t buf[2];

    select_max_chip(chip_idx);

    msgs[0].addr = MAX40109_I2C_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = MAX40109_I2C_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = buf;
    msgs[1].len = 2;

    if (rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
    {
        *val = (buf[0] << 8) | buf[1];
        return RT_EOK;
    }

    LOG_E("I2C Read Error on Chip %d, Reg 0x%02X", chip_idx, reg);
    return -RT_EIO;
}

/**
 * @brief Writes a 16-bit register to MAX40109 (Big Endian)
 */
static rt_err_t max40109_write_reg(rt_uint8_t chip_idx, rt_uint8_t reg, rt_uint16_t val)
{
    struct rt_i2c_msg msg;
    rt_uint8_t buf[3];

    select_max_chip(chip_idx);

    buf[0] = reg;
    buf[1] = (rt_uint8_t)(val >> 8);
    buf[2] = (rt_uint8_t)(val & 0xFF);

    msg.addr = MAX40109_I2C_ADDR;
    msg.flags = RT_I2C_WR;
    msg.buf = buf;
    msg.len = 3;

    if (rt_i2c_transfer(i2c_bus, &msg, 1) == 1)
    {
        return RT_EOK;
    }

    LOG_E("I2C Write Error on Chip %d, Reg 0x%02X", chip_idx, reg);
    return -RT_EIO;
}

/**
 * @brief Handle specific alerts from the Status Register
 */
static void handle_chip_alert(rt_uint8_t chip_idx)
{
    rt_uint16_t status = 0;
    rt_err_t ret;

    ret = max40109_read_reg(chip_idx, MAX40109_REG_STATUS, &status);
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

    /*Handle Data Ready */
    if (status & MAX_STATUS_PRESS_READY)
    {
    }

    /* Clear Status bits */
    max40109_write_reg(chip_idx, MAX40109_REG_STATUS, status);
}

/**
 * @brief Alert Processing Thread
 */
static void alert_thread_entry(void *parameter)
{
    rt_uint8_t local_mask = 0;
    int i;

    while (1)
    {
        if (rt_sem_take(alert_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            rt_enter_critical();
            local_mask = g_triggered_channels_mask;
            g_triggered_channels_mask = 0;
            rt_exit_critical();

            for (i = 0; i < NUM_MAX_CHIPS; i++)
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
 */
rt_err_t max_app_init(void)
{
    int i;

    /* 1. Init I2C Bus */
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(I2C_BUS_NAME);
    if (i2c_bus == RT_NULL)
    {
        LOG_E("Can't find %s device!", I2C_BUS_NAME);
        return -RT_ENOSYS;
    }

    /* 2. Hardware Power-Up Sequence */
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

    rt_thread_mdelay(100); // Wait for power stabilization

    /* 3. Initialize Semaphore and Thread */
    alert_sem = rt_sem_create("max_alert", 0, RT_IPC_FLAG_FIFO);

    //alert_thread = rt_thread_create("max_task", alert_thread_entry, RT_NULL,
    //                                ALERT_THREAD_STACK_SIZE, ALERT_THREAD_PRIORITY, 10);
    if (alert_thread != RT_NULL)
        rt_thread_startup(alert_thread);

    /* 4. Configure each MAX40109 Chip */
    for (i = 0; i < NUM_MAX_CHIPS; i++)
    {
        /* Init Alert Pin */
        if (alert_pins[i] != 0) // Check for valid pin
        {
            rt_pin_mode(alert_pins[i], PIN_MODE_INPUT_PULLUP);
            rt_pin_attach_irq(alert_pins[i], PIN_IRQ_MODE_FALLING, alert_irq_handler, (void*)(rt_ubase_t)i);
        }

        /* I2C Configuration */
        // Set Sample Rate to default 1ksps
        max40109_write_reg(i, MAX40109_REG_ADC_SAMPLE_RATE, 0x0001);

        // Enable Interrupts default disable temp_data ready and pressure_data ready
        max40109_write_reg(i, MAX40109_REG_INTERRUPT_ENABLE, 0x0FF);

        // Clear any pending status bits at startup
        rt_uint16_t dummy_status;
        max40109_read_reg(i, MAX40109_REG_STATUS, &dummy_status);
        if (dummy_status != 0)
        {
            max40109_write_reg(i, MAX40109_REG_STATUS, dummy_status);
        }

        /* Enable IRQ after chip config is done */
        if (alert_pins[i] != 0)
        {
            rt_pin_irq_enable(alert_pins[i], PIN_IRQ_ENABLE);
        }
    }

    LOG_I("MAX40109 Init Completed for %d chips", NUM_MAX_CHIPS);
    return RT_EOK;
}


rt_err_t max40109_read_pressure(rt_uint8_t chip_idx, float *pressure)
{
    /*
    rt_uint16_t raw;
    rt_err_t ret = max40109_read_reg(chip_idx, MAX40109_REG_CALIBRATED_PRESSURE, &raw);
    if (ret != RT_EOK) return ret;

    int16_t signed_raw = (int16_t)raw;
    *pressure = (float)signed_raw / 32768.0f;

    */
    return RT_EOK;
}
