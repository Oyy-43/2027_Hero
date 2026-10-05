/**
 * @file Ammo_Task.cpp
 * @author 
 * @brief 
 * @version 0.1
 * @date 2026-10-04 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "Ammo_Task.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
Class_Motor_DM_Normal Motor_DM_4340P;
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
/**
 * @brief 初始化拨盘模块，初始化电机，绑定信号源
 */
void Class_Shoot::Ammo_Init (bool *__Fire_Signal,bool *__Enable_Signal)
{
    Motor_DM_4340P.Init(&hfdcan2,0x20, 0x10, Motor_DM_Control_Method_NORMAL_MIT_Position,12.5f,10.0f,28.0f,0.0f,0.4f);
    PID_Init(&Motor_DM_4340P.PID_Omega,27.0f, 3.0f, 0.0f, 0.015f,3.25f,6.5f,0.0f,7.5f,0,0,0,0,Integral_Limit);
    PID_Init(&Motor_DM_4340P.PID_Angle,5.8f, 2.9f, 0.0f, 0.001f,37.5f,0.0125f,0.0,0.0f,0,0,0,0,Integral_Limit);

    Fire_Signal = __Fire_Signal;
    Enable_Signal = __Enable_Signal;
    Shoot_Status = Shoot_Disenable;
    Shoot_Event = Shoot_Event_None;
    Begin_Time = SYS_Timestamp.Get_Current_Timestamp();
}

/**
 * @brief 事件更新函数
 * @param none
 */
void Class_Shoot::Shoot_Event_update()
{
    Shoot_Event = Shoot_Event_None;
    
    if(Shoot_Status == Shoot_Disenable && *Enable_Signal == true)
    {
        Shoot_Event = Shoot_Event_Enable;
        Begin_Time = SYS_Timestamp.Get_Current_Timestamp();
    }

    if(Shoot_Status == Shoot_Enable && *Enable_Signal == false)
    {
        Shoot_Event = Shoot_Event_Disenable;
        Begin_Time = SYS_Timestamp.Get_Current_Timestamp();
    }

    if(Shoot_Status == Shoot_Enable && *Fire_Signal == true)
    {
        Shoot_Event = Shoot_Event_Fire;
        *Fire_Signal = false;
        Begin_Time = SYS_Timestamp.Get_Current_Timestamp();
    }

    if(Shoot_Status == Shoot_Fireing && (fabs(Motor_DM_4340P.PID_Angle.Err)<0.1))
    {
        Shoot_Event = Shoot_Event_FireDone;
        Begin_Time = SYS_Timestamp.Get_Current_Timestamp();
    }

    if(Shoot_Status == Shoot_Fireing && Shoot_Stuck_Check())
    {
        Shoot_Event = Shoot_Event_Stuck;
        Begin_Time = SYS_Timestamp.Get_Current_Timestamp();
    }

    if(Shoot_Status == Shoot_StuckReleasing && (fabs(Motor_DM_4340P.PID_Angle.Err)<0.1))
    {
        Shoot_Event = Shoot_Event_StuckRealseDone;
        Begin_Time = SYS_Timestamp.Get_Current_Timestamp();
    }
}

/**
 * @brief 状态机运行函数
 * @param none
 */
void Class_Shoot::Shoot_FSM_Run()
{
    Current_Time = SYS_Timestamp.Get_Current_Timestamp();
    State_time = Current_Time - Begin_Time;
    switch(Shoot_Status)
    {
        case Shoot_Disenable :
    }
}
/* Function prototypes -------------------------------------------------------*/
