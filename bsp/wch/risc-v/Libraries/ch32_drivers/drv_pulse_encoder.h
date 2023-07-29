/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-20     MXH          the first version
 */

#ifndef __DRV_PULSE_ENCODER_H__
#define __DRV_PULSE_ENCODER_H__

#include <rtthread.h>
#ifdef BSP_USING_PULSE_ENCODER
#if defined(SOC_RISCV_SERIES_CH32V3)
#include "ch32v30x_tim.h"
#endif
#if defined(SOC_RISCV_SERIES_CH32V2)
#include "ch32v20x_tim.h"
#endif
#include <drivers/pulse_encoder.h>
#include <drivers/hwtimer.h>
#include <board.h>

#define FLAG_NOT_INIT   0xFF

struct rtdevice_encoder_device
{
    struct rt_pulse_encoder_device parent;
    TIM_TypeDef* periph;
    char* name;
};

#endif/* BSP_USING_ENCODER */

#endif/* __DRV_PULSE_ENCODER_H__ */
