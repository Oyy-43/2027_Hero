/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

// #include "2_Device/BSP/BMI088/bsp_bmi088.h"
// #include "2_Device/Plotter/Vofa/dvc_vofa.h"
// #include "2_Device/BSP/W25Q64JV/bsp_w25q64jv.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
// #include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "2_Device/Motor/Motor_DJI/drv_motor_dji.h"
#include <stdbool.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// LED灯
int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

// 全局初始化完成标志位
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief CAN1回调函数
 *
 *
 */
void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
        case (0x201):
        {
            Motor_C620[0].CAN_RxCpltCallback();

            break;
        }
        case (0x202):
        {
            Motor_C620[1].CAN_RxCpltCallback();

            break;
        }
        case (0x203):
        {
            Motor_C620[2].CAN_RxCpltCallback();

            break;
        }
        case (0x204):
        {
            Motor_C620[0].CAN_RxCpltCallback();

            break;
        }
    }
}

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{
}

/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    static int mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

        if (red >= 18)
        {
            red_minus_flag = true;
        }
        else if (red == 0)
        {
            red_minus_flag = false;
        }
        if (green >= 18)
        {
            green_minus_flag = true;
        }
        else if (green == 0)
        {
            green_minus_flag = false;
        }
        if (blue >= 18)
        {
            blue_minus_flag = true;
        }
        else if (blue == 0)
        {
            blue_minus_flag = false;
        }

        if (red_minus_flag)
        {
            red--;
        }
        else
        {
            red++;
        }
        if (green_minus_flag)
        {
            green--;
        }
        else
        {
            green++;
        }
        if (blue_minus_flag)
        {
            blue--;
        }
        else
        {
            blue++;
        }

        BSP_WS2812.Set_RGB(red, green, blue);
        // BSP_WS2812.Set_RGB(0, 0, 0);

        // 发送实例
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
    }


    BSP_Key.TIM_1ms_Process_PeriodElapsedCallback();
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;

        // 处理按键状态
        BSP_Key.TIM_50ms_Read_PeriodElapsedCallback();
    }


    TIM_1ms_CAN_PeriodElapsedCallback();
    // 喂狗
    TIM_1ms_IWDG_PeriodElapsedCallback();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);
    // 串口绘图的USB

    // 陀螺仪的SPI

    // WS2812的SPI
    SPI_Init(&hspi6, nullptr);
    // 电机的CAN
    CAN_Init(&hfdcan1, CAN1_Callback);
    // 电源的ADC

    // flash的OSPI

    //电机滤波器初始化
    Filter_Init_All();

    //电机PID参数初始化
    PID_Init_All();
    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim5);

    // BSP_WS2812.Init(0, 0, 0);

    // BSP_Buzzer.Init();

    BSP_Power.Init();

    BSP_Key.Init();

    // BSP_BMI088.Init();


    BSP_Power.Set_Power(true, true, true);
    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
    Namespace_SYS_Timestamp::Delay_Millisecond(1);
}


/**
 * @brief 给整车的电机PID参数初始化
 * 
 */
void PID_Init_All()
{   
    //底盘轮向电机PID参数初始化
    PID_Init(&Motor_C620[0].PID_Omega,16384,1000,0,0.002,0.0f,0.0f,0.0f,0.0f,0,0,0,0,Integral_Limit);
    PID_Init(&Motor_C620[1].PID_Omega,16384,1000,0,0.002,0.0f,0.0f,0.0f,0.0f,0,0,0,0,Integral_Limit);
    PID_Init(&Motor_C620[2].PID_Omega,16384,1000,0,0.002,0.0f,0.0f,0.0f,0.0f,0,0,0,0,Integral_Limit);
    PID_Init(&Motor_C620[3].PID_Omega,16384,1000,0,0.002,0.0f,0.0f,0.0f,0.0f,0,0,0,0,Integral_Limit);
}

/**
 * @brief 给整车的电机滤波器初始化
 * 
 */
void Filter_Init_All()
{
    //底盘轮向电机滤波器初始化
    for(int i =0;i<4;i++){
        Motor_C620[i].Filter_Omega.Init(0.0f,0.0f,Filter_Frequency_Type_LOWPASS,50.0f,0.0f,1000.0f);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
