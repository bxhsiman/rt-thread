/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-7-25      siman        增加电机控制
 * 2022-08-23     liYony       first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "ch32v30x.h"
/*电机PWM*/
#define PWM_MOTOR_NAME        "pwm10" /* PWM设 备 名 称 */
#define PWM_MOTOR_CHANNEL1   1      /* PWM通 道 1 */
#define PWM_MOTOR_CHANNEL2   2      /* PWM通 道 2*/
#define PWM_MOTOR_CHANNEL3   3      /* PWM通 道 3 */
#define PWM_MOTOR_CHANNEL4   4      /* PWM通 道 4 */
#define PWM_Config_Period    50     /* PWM调整周期*/
int duty1 = 0, duty2 = 0, duty3 = 0, duty4 = 0;

struct rt_device_pwm *pwm_dev; /* PWM设 备 句 柄 */



int main(void)
{
    while (1)
    {
        rt_thread_mdelay(1000);

    }
}





struct rt_device_pwm *pwm_dev; /* PWM设 备 句 柄 */
 
static int motor_pwm_set(int argc, char *argv[])
{
    rt_uint32_t period, pulse1,pulse2,pulse3,pulse4;
    period = 83333; /* 频率为12khz， 此处为周期 单位为纳秒ns */
    pulse1 = 0;pulse2 = 0;pulse3 = 0;pulse4 = 0;
    /* 查 找 设 备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_MOTOR_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_MOTOR_NAME);
        return RT_ERROR;
    }
    /* 设 置PWM周 期 和 脉 冲 宽 度 默 认 值 */
    rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL1, period, pulse1);
    rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL2, period, pulse2);
    rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL3, period, pulse2);
    rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL4, period, pulse2);
    /* 使 能 设 备 */
    rt_pwm_enable(pwm_dev, PWM_MOTOR_CHANNEL1);
    rt_pwm_enable(pwm_dev, PWM_MOTOR_CHANNEL2);
    rt_pwm_enable(pwm_dev, PWM_MOTOR_CHANNEL3);
    rt_pwm_enable(pwm_dev, PWM_MOTOR_CHANNEL4);

    while (1)
    {
        rt_thread_mdelay(PWM_Config_Period);
        pulse1 = duty1 * period / 100;
        pulse2 = duty2 * period / 100;
        pulse3 = duty3 * period / 100;
        pulse4 = duty4 * period / 100;
        rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL1, period, pulse1);
        rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL2, period, pulse2);
        rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL3, period, pulse3);
        rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL4, period, pulse4);
    }
}
/* 导 出 到 msh 命 令 列 表 中 */
MSH_CMD_EXPORT(motor_pwm_set, four channel pwm motor control);