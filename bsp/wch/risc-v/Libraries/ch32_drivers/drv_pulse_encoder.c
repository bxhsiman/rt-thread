/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-09-23     charlown          first version
 * 2022-10-14     hg0720            the first version which add from wch
 * 2022-10-20     MXH               add the remaining timers
 */

#include "drv_pulse_encoder.h"

#ifdef BSP_USING_PULSE_ENCODER

#define LOG_TAG "drv.pulse_encoder"
#include <drv_log.h>

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])

void ch32_tim_clock_init(TIM_TypeDef* timx)
{
#ifdef BSP_USING_TIM1_ENCODER
    if (timx == TIM1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    }
#endif/* BSP_USING_TIM1_ENCODER */

#ifdef BSP_USING_TIM2_ENCODER
    if (timx == TIM2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    }
#endif/* BSP_USING_TIM2_ENCODER */

#ifdef BSP_USING_TIM3_ENCODER
    if (timx == TIM3)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    }
#endif/* BSP_USING_TIM3_ENCODER */

#ifdef BSP_USING_TIM4_ENCODER
    if (timx == TIM4)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    }
#endif/* BSP_USING_TIM4_ENCODER */

#ifdef BSP_USING_TIM5_ENCODER
    if (timx == TIM5)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    }
#endif/* BSP_USING_TIM5_ENCODER */

    /* TIM6 and TIM7 don't support ENCODER Mode. */

#ifdef BSP_USING_TIM8_ENCODER
    if (timx == TIM8)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    }
#endif/* BSP_USING_TIM8_ENCODER */

#ifdef BSP_USING_TIM9_ENCODER
    if (timx == TIM9)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    }
#endif/* BSP_USING_TIM9_ENCODER */

#ifdef BSP_USING_TIM10_ENCODER
    if (timx == TIM10)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
    }
#endif/* BSP_USING_TIM10_ENCODER */
}

rt_uint32_t ch32_tim_clock_get(TIM_TypeDef* timx)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    /*tim1~10 all in HCLK*/
    return RCC_Clocks.HCLK_Frequency;
}


void ch32_encoder_init(TIM_TypeDef* timx)
{
    GPIO_InitTypeDef GPIO_InitStructure;

#ifdef BSP_USING_TIM2_ENCODER
    if (timx == TIM2)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        
    }
#endif/* BSP_USING_TIM2_ENCODER */

#ifdef BSP_USING_TIM3_ENCODER
    if (timx == TIM3)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
#endif/* BSP_USING_TIM3_ENCODER */
}

/*
 * channel = FLAG_NOT_INIT: the channel is not use.
 */
struct rtdevice_encoder_device encoder_device_list[] =
{

#ifdef BSP_USING_TIM2_ENCODER
    {
        .periph = TIM2,
        .name = "ENCODER2",
    },
#endif /* BSP_USING_TIM2_ENCODER */

#ifdef BSP_USING_TIM3_ENCODER
    {
        .periph = TIM3,
        .name = "ENCODER3",
    },
#endif /* BSP_USING_TIM3_ENCODER */
};

static rt_err_t ch32_encoder_device_enable(struct rt_pulse_encoder_device* device)
{
    struct rtdevice_encoder_device* encoder_device;

    encoder_device = (struct rtdevice_encoder_device*)device;
    
    //时钟配置
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;  
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); //初始化Tim配置结构体
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频0 
    TIM_TimeBaseStructure.TIM_Period = 0xffff; //自动重载 最高
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
    TIM_TimeBaseInit(encoder_device->periph, &TIM_TimeBaseStructure); //时钟初始化信息载入

    TIM_EncoderInterfaceConfig(encoder_device->periph, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3
    
    TIM_ICStructInit(&TIM_ICInitStructure);                           //初始化配置结构体 其实就是下面的配置 但是仅开启一个通道
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2;    //SAUS只启动了一个通道 这里我先开俩
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //输入捕获极性设置，可用于配置编码器正反相
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;             //输入捕获预分频器设置
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   //输入捕获通道选择，编码器模式需选用此配置
    TIM_ICInitStructure.TIM_ICFilter = 6;                             //输入捕获滤波器设置
    TIM_ICInit(encoder_device->periph, &TIM_ICInitStructure);

    TIM_ClearFlag(encoder_device->periph, TIM_FLAG_Update);        //清除TIM更新标志位
    TIM_ITConfig(encoder_device->periph, TIM_IT_Update, ENABLE);   //使能开启TIM中断
    
    TIM_Set(encoder_device->periph, 0)
    TIM_Cmd(encoder_device->periph, ENABLE);

    return RT_EOK;
}

static rt_int32_t ch32_encoder_get_count(struct drv_pulse_encoder_device* device)
{
    struct rtdevice_encoder_device* encoder_device;

    encoder_device = (struct rtdevice_encoder_device*)device;
    
    return encoder_device->periph->CNT;
}

static rt_err_t ch32_encoder_clear_count(struct rt_pulse_encoder_device* device)
{
    struct rtdevice_encoder_device* encoder_device;

    encoder_device = (struct rtdevice_encoder_device*)device;
    
    encoder_device->periph->CNT = 0;

    return RT_EOK;
}

static rt_err_t drv_encoder_control(struct rt_pulse_encoder_device* device, int cmd, void* arg)
{
    switch (cmd)
    {
    case PULSE_ENCODER_CMD_ENABLE:
        return ch32_encoder_device_enable(device);
    default:
        return -RT_EINVAL;
    }
}

static struct rt_pulse_encoder_ops encoder_ops =
{
    .init = ch32_encoder_init,
    .get_count = ch32_encoder_get_count,
    .clear_count = ch32_encoder_clear_count,
    .control = drv_encoder_control
};

static int rt_hw_encoder_init(void)
{
    int result = RT_EOK;
    int index = 0;

    for (index = 0; index < ITEM_NUM(encoder_device_list); index++)
    {
        ch32_tim_clock_init(encoder_device_list[index].periph);
        ch32_encoder_init(encoder_device_list[index].periph);
        ch32_encoder_device_enable(&encoder_device_list[index].parent);

        if (rt_device_pulse_encoder_register(&encoder_device_list[index].parent, encoder_device_list[index].name, &encoder_ops, RT_NULL) == RT_EOK)
        {
            LOG_D("%s register success", encoder_device_list[index].name);
        }
        else
        {
            LOG_D("%s register failed", encoder_device_list[index].name);
            result = -RT_ERROR;
        }
    }
    

    return result;
}

INIT_BOARD_EXPORT(rt_hw_encoder_init);

#endif /* BSP_USING_ENCODER */
