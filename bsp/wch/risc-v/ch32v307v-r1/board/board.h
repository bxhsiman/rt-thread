/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-23     liYony       first version
 */

/* <<< Use Configuration Wizard in Context Menu >>> */
#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtthread.h>
#include "ch32v30x.h"
#include "drv_gpio.h"
#include "drv_pwm.h"
#include "drv_pulse_encoder.h"
#include "drv_hwtimer.h"

/*电机控制PWM PWM10 CH1-PA8 CH2-PA9 CH3-PC3 CH4-PC11*/
/*pwmname:pwm10*/
#define BSP_USING_TIM10_PWM
#define BSP_USING_TIM10_PWM_CH1
#define BSP_USING_TIM10_PWM_CH2
#define BSP_USING_TIM10_PWM_CH3
#define BSP_USING_TIM10_PWM_CH4

/*编码器TIM2 PA0*/
#define BSP_USING_PULSE_ENCODER
#define BSP_USING_TIM2_ENCODER
#define BSP_USING_TIM3_ENCODER

/*超声波*/
#define BSP_USING_HWTIMER



/* board configuration */
#define SRAM_SIZE  96
#define SRAM_END (0x20000000 + SRAM_SIZE * 1024)

extern int _ebss, _susrstack;
#define HEAP_BEGIN  ((void *)&_ebss)
#define HEAP_END    ((void *)&_susrstack)

void rt_hw_board_init(void);

#endif /* __BOARD_H__ */
