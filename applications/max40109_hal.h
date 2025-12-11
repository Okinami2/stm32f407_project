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

#define MAX40109_I2C_ADDR    0x4D  /* The I2C address of MAX40109 (7-bit) */
#define I2C_BUS_NAME         "i2c1" /* The name of the I2C bus device */

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
#define MAX40109_REG_SENSOR_OFFSET        0x14

/* Status Register & Interrupt Enable Bits (Page 7 & 24) */
#define MAX_STATUS_OV_INP_POS    (1 << 0) // Input Positive Over Voltage
#define MAX_STATUS_UV_INP_POS    (1 << 1) // Input Positive Under Voltage
#define MAX_STATUS_OV_INP_NEG    (1 << 2) // Input Negative Over Voltage
#define MAX_STATUS_UV_INP_NEG    (1 << 3) // Input Negative Under Voltage
#define MAX_STATUS_OV_INT        (1 << 4) // INT Pin Over Voltage
#define MAX_STATUS_UV_INT        (1 << 5) // INT Pin Under Voltage
#define MAX_STATUS_OV_DRV        (1 << 6) // DRV Pin Over Voltage
#define MAX_STATUS_UV_DRV        (1 << 7) // DRV Pin Under Voltage
#define MAX_STATUS_PRESS_READY   (1 << 8) // Pressure Data Ready
#define MAX_STATUS_TEMP_READY    (1 << 9) // Temperature Data Ready
#define MAX_STATUS_INT_FAULT     (1 << 10) // INT Fault
#define MAX_STATUS_DRV_FAULT     (1 << 11) // DRV Fault

#define NUM_MAX_CHIPS 8

/* Function Prototypes */
rt_err_t max_app_init(void);
rt_err_t max40109_read_pressure(rt_uint8_t chip_idx, float *pressure);
rt_err_t max40109_read_temperature(rt_uint8_t chip_idx, float *temp);

#endif/* APPLICATIONS_MAX40109_HAL_H_ */
