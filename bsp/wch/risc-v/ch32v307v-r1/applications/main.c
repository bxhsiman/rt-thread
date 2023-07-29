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
#define PWM_Config_Period    4     /* PWM调整周期*/
static rt_mutex_t mutex_duty;        /* 互斥锁 */
static int duty1 = 0, duty2 = 0, duty3 = 0, duty4 = 0;
static int Motor1_Target = 0, Motor2_Target = 0;
static int Speed = 20;
struct rt_device_pwm *pwm_dev; /* PWM设 备 句 柄 */

/*编码器*/
#define ENCODER_NAME "encoder2"

/*超声波测距*/
#define TIMER6_NAME "timer6"  //通用定时器
#define ULTRASONIC_PIN_ECHO 4 //PA4
#define ULTRASONIC_PIN_TRIG 5 //PA5
/* PID相关参数*/
static float error = 0;
/*通信协议*/
#define START 0x7E //帧头
#define END 0x7F   //帧尾

typedef union 
{
	uint8_t U8_Buff[2];
	uint16_t U16;
	int16_t S16;
}Bint16_Union;
	
typedef union 
{
	uint8_t U8_Buff[4];
	float Float;
    unsigned long U32;
}Bint32_Union;

/*UART6 Rx-PC1 Tx-PC0*/
#define UART6_NAME "uart6"
#define UART7_NAME "uart7"
#define UART8_NAME "uart8"
// #define UART6_RX_PIN 33 //PC1
// #define UART6_TX_PIN 32 //PC0
static struct rt_semaphore uart6_rx_sem; //信号量

/*
 * 命令调用格式：uart_start uart2
 * 命令解释：命令第二个参数是要使用的串口设备名称，为空则使用默认的串口设备
 * 程序功能：
*/

#define DEFAULT_UART_NAME       "uart6"

/* 用于接收消息的信号量 */
static struct rt_semaphore rx_sem;
static rt_device_t serial;

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

/* 串口接收线程 */
static void serial_thread_entry(void *parameter)
{
    
    while (1)
    {
        char buffer[6];
        while(rt_device_read(serial, 0, buffer, 6) < 6) {rt_sem_take(&rx_sem, RT_WAITING_FOREVER);rt_thread_delay(100);}
        Bint32_Union float_data;
        if(buffer[0] == 'T'){
            float_data.U8_Buff[0] = buffer[1];
            float_data.U8_Buff[1] = buffer[2];
            float_data.U8_Buff[2] = buffer[3];
            float_data.U8_Buff[3] = buffer[4];
            //rt_kprintf("%c,%c,%c,%c,%c,%c\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5]);
            error = float_data.Float;           //error设置  
            memset(buffer, 0, sizeof(buffer));
        }else{
            memset(buffer, 0, sizeof(buffer));
        }
    }
}

static int uart_start(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];

    if (argc == 2)
    {
        rt_strncpy(uart_name, argv[1], RT_NAME_MAX);
    }
    else
    {
        rt_strncpy(uart_name,DEFAULT_UART_NAME, RT_NAME_MAX);
    }

    /* 查找系统中的串口设备 */
    serial = rt_device_find(uart_name);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }

    return ret;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(uart_start, uart device start);

/*串口发送数据*/
/*
    数据格式 char * data [n]：
    第一位为使用串口编号 6-8

*/
static rt_err_t uart_send(char * data)
{

    rt_device_t uart_dev = RT_NULL;
    rt_err_t ret = RT_EOK;
    switch (data[0])
    {
    case 6:
        uart_dev = rt_device_find(UART6_NAME);
        break;
    case 7:
        uart_dev = rt_device_find(UART7_NAME);
        break;
    case 8:
        uart_dev = rt_device_find(UART8_NAME);
        break;
    default:
        rt_kprintf("串口编号错误\n");
        ret = RT_ERROR;
        break;
    }
    ret = rt_device_write(uart_dev, 0, data, rt_strlen(data));
    if(ret != RT_EOK)rt_kprintf("发送数据出错 %d\n", ret);
    return ret;    
}

/*硬件计时*/
static rt_err_t timer6_start(int argc, char *argv[]){
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;
    rt_device_t hw_dev = RT_NULL;
    rt_hwtimer_mode_t mode;

    hw_dev = rt_device_find(TIMER6_NAME);
    if(hw_dev == RT_NULL){
        rt_kprintf("can't find %s device!\n", TIMER6_NAME);
        return RT_ERROR;
    }
    
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if(ret != RT_EOK)rt_kprintf("打开计时器 %s 出错 %d \n", TIMER6_NAME, ret);
    /* 设置模式为周期性定时器（若未设置，默认是HWTIMER_MODE_ONESHOT）*/
    mode = HWTIMER_MODE_PERIOD;
    rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    /* 设置定时器超时值为400ms并启动定时器 */
    timeout_s.sec = 5;      /* 秒 */
    timeout_s.usec = 0;     /* 微秒 */
    rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s));
    return RT_EOK;
}
static rt_hwtimerval_t timer6_stop(int argc, char *argv[]){
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t time ;
    rt_device_t hw_dev = RT_NULL;
    hw_dev = rt_device_find(TIMER6_NAME);
    if(hw_dev == RT_NULL){
        rt_kprintf("can't find %s device!\n", TIMER6_NAME);
        return time;
    }
    /* 读取定时器计数值 */
    rt_device_read(hw_dev, 0, &time, sizeof(time));
    //rt_kprintf("定时器为 %d s\n",time.usec / 1000000);
    rt_device_close(hw_dev);
    return time;
}
/*超声测距函数 返回值 float 单位为m*/
static float senor_ul_length_get(int argc, char *argv[]){
    
    rt_pin_mode(ULTRASONIC_PIN_TRIG, PIN_MODE_OUTPUT);
    rt_pin_mode(ULTRASONIC_PIN_ECHO, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_write(ULTRASONIC_PIN_TRIG, PIN_LOW);
    
    float length = 0, sum = 0;
    rt_uint32_t i = 0;
    
    while(i!=5){
        //触发信号
        rt_pin_write(ULTRASONIC_PIN_TRIG, PIN_HIGH);
        rt_thread_mdelay(1);
        rt_pin_write(ULTRASONIC_PIN_TRIG, PIN_LOW);
        //回波起始
        while(rt_pin_read(ULTRASONIC_PIN_ECHO) == PIN_LOW);
        timer6_start(0,RT_NULL);
        //回波结束
        while(rt_pin_read(ULTRASONIC_PIN_ECHO) == PIN_HIGH);
        sum += timer6_stop(0,RT_NULL).usec;
        i++;
    }

    length = sum / 1000000 / 5 * 340 / 2;
    rt_kprintf("超声测距为：%d cm\n", (int)(length*100));
    return length;
}
/* 导 出 到 msh 命 令 列 表 中 */
MSH_CMD_EXPORT(senor_ul_length_get, ultrasonic length get);
/*电机控制*/
static int motor_pwm_set(void *parameter)
{
    static rt_uint32_t period, pulse1,pulse2,pulse3,pulse4;
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
        rt_mutex_take(mutex_duty, RT_WAITING_FOREVER);
        rt_kprintf("duty set! duty1:%d duty2:%d duty3:%d duty4:%d\n",duty1,duty2,duty3,duty4);
        pulse1 = duty1 * period / 100;
        pulse2 = duty2 * period / 100;
        pulse3 = duty3 * period / 100;
        pulse4 = duty4 * period / 100;
        rt_mutex_release(mutex_duty);
        rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL1, period, pulse1);
        rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL2, period, pulse2);
        rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL3, period, pulse3);
        rt_pwm_set(pwm_dev, PWM_MOTOR_CHANNEL4, period, pulse4);

        
    }
}




//Motor
typedef enum{
    Motor1,Motor2
}Motor_enum;
static void Motor_SetSpeed(Motor_enum Motor,int duty){
    rt_mutex_take(mutex_duty, RT_WAITING_FOREVER);
    if(Motor == Motor1){
        if(duty > 0){
            if(duty > 100) duty = 100;
            duty1 = duty;
            duty2 = 0;
            
        }
        else{
            duty = -duty;
            if(duty > 100) duty = 100;
            duty1 = 0;
            duty2 = duty;
        }
    }
    if(Motor == Motor2){
        if(duty > 0){
            if(duty > 100) duty = 100;
            duty3 = duty;
            duty4 = 0;
            
        }
        else{
            duty = -duty;
            if(duty > 100) duty = 100;
            duty3 = 0;
            duty4 = duty;
        }
    }
    rt_kprintf("duty change! duty1:%d duty2:%d duty3:%d duty4:%d\n",duty1,duty2,duty3,duty4);
    rt_mutex_release(mutex_duty);
}
/*PID*/
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float error;
    float error_last;
    float error_llast;
    float error_sum;
    float cor;
    float cor_limit;
    float sum_limit;
}PID;
PID Direction_PID;
PID Speed_PID; 
static void PID_Init(PID* P){
    P->Kp = 0;
    P->Ki = 0;
    P->Kd = 0;
    P->cor_limit = 0;
    P->sum_limit = 0;
    P->error = 0;
    P->error_last = 0;
    P->error_llast = 0;
    P->error_sum = 0;
    P->cor = 0;
}
static void PID_SetPrama(PID* P, float Kp, float Ki, float Kd, float cor_limit, float sum_limit){
    P->Kp = Kp;
    P->Ki = Ki;
    P->Kd = Kd;
    P->cor_limit = cor_limit;
    P->sum_limit = sum_limit;
}
static void PID_IncrementalPID(PID* P, float target, float current){
    P->error = target - current;
    P->cor += P->Kp * (P->error - P->error_last) + P->Ki * P->error + P->Kd * (P->error - 2 * P->error_last + P->error_sum);
    P->error_llast = P->error_last;
    P->error_last = P->error;
    if(P->cor > P->cor_limit)P->cor = P->cor_limit;
    if(P->cor < -P->cor_limit)P->cor = -P->cor_limit;
}
static void PID_PositionalPID(PID* P, float error){
    P->error = error;
    P->error_sum += P->error;
    if(P->error_sum > P->sum_limit)P->error_sum = P->sum_limit;
    if(P->error_sum < -P->sum_limit)P->error_sum = -P->sum_limit;
    P->cor = P->Kp * P->error + P->Ki * P->error_sum + P->Kd * (P->error - P->error_last);
    P->error_last = P->error;
    if(P->cor > P->cor_limit)P->cor = P->cor_limit;
    if(P->cor < -P->cor_limit)P->cor = -P->cor_limit;
}
static void Direction_PIDWork(void){
    PID_PositionalPID(&Direction_PID, error);
    Motor1_Target = Speed - Direction_PID.cor;
    Motor2_Target = Speed + Direction_PID.cor;
    Motor_SetSpeed(Motor1, Motor1_Target);
    Motor_SetSpeed(Motor2, Motor2_Target);
}

static void PID_work(void){
    PID_Init(&Direction_PID);
    PID_Init(&Speed_PID);
    PID_SetPrama(&Direction_PID, 0.6, 0.01, 1.3, 40, 3);
    while(1){
        Direction_PIDWork();
        rt_thread_mdelay(4);    
    }
}

int main(void)
{
    /* 创建互斥锁 */
	mutex_duty = rt_mutex_create("mutex_duty", RT_IPC_FLAG_FIFO);


    rt_thread_t thread;

    thread = rt_thread_create("PWM", motor_pwm_set, RT_NULL, 1024, 25, 10);
    if (thread != RT_NULL) rt_thread_startup(thread);
    else rt_kprintf("PID thread create failed.\n");

    thread = rt_thread_create("PID", PID_work, RT_NULL, 1024, 25, 10);
    if (thread != RT_NULL) rt_thread_startup(thread);
    else rt_kprintf("PID thread create failed.\n");

    uart_start(0,RT_NULL);
    while (1)
    {
        rt_thread_mdelay(1000);
    }
}






 
