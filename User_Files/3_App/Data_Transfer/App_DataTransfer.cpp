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
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
Shoot_Decision Shoot_Decision_Transfer;
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

void Shoot_Decision::Fire_Signal_Decide()
{
    if(BSP_Key.Get_Key_Status() == BSP_Key_Status_TRIG_PRESSED_FREE)
    {
        Fire_Signal = true;
    }
    return;
}

void Robo_Dec_Fuc(void *argument)
{
    for(;;)
    {
        Shoot_Decision_Transfer.Fire_Signal_Decide();
        osDelay(1);
    }
}


/* Function prototypes -------------------------------------------------------*/