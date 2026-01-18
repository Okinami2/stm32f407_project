/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-10     GreatMagicianGarfiel       the first version
 * 2025-12-01     GreatMagicianGarfiel       Add register definitions
 */
#ifndef APPLICATIONS_MAX40109_HAL_H_
#define APPLICATIONS_MAX40109_HAL_H_

#include "drv_soft_i2c.h"
#include <rtthread.h>
#include <rtdevice.h>

#define MAX40109_I2C_ADDR    0x4D
#define I2C_BUS_NAME         "i2c1"

/* MAX40109 Register Addresses */
#define MAX40109_REG_CONFIG                 0x00
#define MAX40109_REG_STATUS                 0x02
#define MAX40109_REG_PGA_PRESSURE_GAIN      0x04
#define MAX40109_REG_CURRENT_SOURCE         0x05
#define MAX40109_REG_UNCAL_PRESSURE         0x06
#define MAX40109_REG_UNCAL_TEMP             0x08
#define MAX40109_REG_ADC_SAMPLE_RATE        0x0A
#define MAX40109_REG_INTERRUPT_ENABLE       0x0B
#define MAX40109_REG_BRIDGE_DRIVE           0x0D
#define MAX40109_REG_PGA_TEMP_GAIN          0x0E
#define MAX40109_REG_CALIBRATED_PRESSURE    0x0F
#define MAX40109_REG_CALIBRATED_TEMP        0x11
#define MAX40109_REG_TEMP_MODE              0x13
#define MAX40109_REG_SENSOR_OFFSET_CAL      0x14
#define MAX40109_REG_ANALOG_FILTER_BW       0x15
#define MAX40109_REG_ZERO_PRESSURE_OFFSET   0x1A
#define MAX40109_REG_ZERO_PRESSURE_SELECT   0x1C
#define MAX40109_REG_ANALOG_OUTPUT_STAGE    0x1E


/* Status Register & Interrupt Enable Bits (Page 7 & 24) */
#define MAX_STATUS_OV_INP_POS    (1 << 0)
#define MAX_STATUS_UV_INP_POS    (1 << 1)
#define MAX_STATUS_OV_INP_NEG    (1 << 2)
#define MAX_STATUS_UV_INP_NEG    (1 << 3)
#define MAX_STATUS_OV_INT        (1 << 4)
#define MAX_STATUS_UV_INT        (1 << 5)
#define MAX_STATUS_OV_DRV        (1 << 6)
#define MAX_STATUS_UV_DRV        (1 << 7)
#define MAX_STATUS_PRESS_READY   (1 << 8)
#define MAX_STATUS_TEMP_READY    (1 << 9)
#define MAX_STATUS_INT_FAULT     (1 << 10)
#define MAX_STATUS_DRV_FAULT     (1 << 11)

#define NUM_MAX_CHIPS 8

/* Function Prototypes */
rt_err_t max_app_init(void);
rt_err_t global_max40109_write_reg(rt_uint8_t chip_idx, rt_uint8_t reg, rt_uint16_t val, rt_uint8_t len);
rt_err_t global_max40109_read_reg(rt_uint8_t chip_idx, rt_uint8_t reg, rt_uint16_t *val, rt_uint8_t len);
rt_err_t max40109_read_pressure(rt_uint8_t chip_idx, double *pressure, uint8_t is_calibrated);
#endif/* APPLICATIONS_MAX40109_HAL_H_ */
