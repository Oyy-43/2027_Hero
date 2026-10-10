/**
 * @file App_DataTransfer.cpp
 * @author Oyyp(2577468184@qq.com)
 * @brief 通过该文件实现各模块之间的数据传输，从而减少模块之间的耦合性，降低模块之间的依赖性
 * @version 0.1
 * @date 2026-09-18 0.1 init
 *
 * @copyright Copyright HUNAU-Robot(c) 2026
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "App_DataTransfer.h"
#include <stdint.h>
#include "stdio.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "cmsis_os2.h"
#include "crsf.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
Shoot_Decision Shoot_Decision_Transfer;
Remote_Data Remote_Data_Transfer;
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 初始化遥控器按键
 * @param void
 * @return void
 */
void Remote_Data::Remote_Key_Init()
{
    if(rc_channels.ch[9]<=10)
    {
        Remote_Key_Right.Pre_Press_State = false;
    }
    else
    {
        Remote_Key_Right.Pre_Press_State = true;
    }
    Remote_Key_Right.Now_Press_State = Remote_Key_Right.Pre_Press_State;
    switch(Remote_Key_Right.Now_Press_State)
    {
        case true:
        Remote_Key_Right.Key_Status = Remote_Key_Status_PRESSED;
        break;
        case false:
        Remote_Key_Right.Key_Status = Remote_Key_Status_FREE;
        break;
    }
}

/**
 * @brief 扫描遥控器按键
 * @param void
 * @return void
 */
void Remote_Data::Remote_Key_Scan()
{
    if(rc_channels.ch[9]<=10)
    {
        Remote_Key_Right.Now_Press_State = false;
    }
    else
    {
        Remote_Key_Right.Now_Press_State = true;
    }
    switch(Remote_Key_Right.Pre_Press_State)
    {
        case true:
            if(Remote_Key_Right.Now_Press_State == true)
            {
                Remote_Key_Right.Key_Status = Remote_Key_Status_PRESSED;
            }
            else
            {
                Remote_Key_Right.Key_Status = Remote_Key_Status_TRIG_PRESSED_FREE;
            }
        break;
        case false:
            if(Remote_Key_Right.Now_Press_State == true)
            {
                Remote_Key_Right.Key_Status = Remote_Key_Status_TRIG_FREE_PRESSED;
            }
            else
            {
                Remote_Key_Right.Key_Status = Remote_Key_Status_FREE;
            }
        break;
    }
    Remote_Key_Right.Pre_Press_State = Remote_Key_Right.Now_Press_State;
}

/**
 * @brief 开火信号决策函数
 * @param void
 * @return void
 */
void Shoot_Decision::Fire_Signal_Decide()
{
    if(Remote_Data_Transfer.Get_Remote_Key_Status()==Remote_Key_Status_TRIG_PRESSED_FREE)
    {
        Fire_Signal = true;
    }
    if(rc_channels.ch[8] > 800)
    {
        Enable_Signal = true;
    }
    else
    {
        Enable_Signal = false;
    }
    return;
}

/**
 * @brief 机器人决策函数
 * @param void
 * @return void
 */
void Robo_Dec_Fuc(void *argument)
{
    Remote_Data_Transfer.Remote_Key_Init();
    for(;;)
    {   
        Remote_Data_Transfer.Remote_Key_Scan();
        Shoot_Decision_Transfer.Fire_Signal_Decide();
        osDelay(1);
    }
}


/* Function prototypes -------------------------------------------------------*/