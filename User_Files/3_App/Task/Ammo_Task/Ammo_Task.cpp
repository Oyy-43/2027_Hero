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
    Begin_Time = SYS_Timestamp.Get_Now_Millisecond();
    Current_Time = Begin_Time;
    State_time = 0.0f;
}

/**
 * @brief 事件更新函数
 * @param none
 */
void Class_Shoot::Shoot_Event_update()
{
    Current_Time = SYS_Timestamp.Get_Now_Millisecond();
    State_time = Current_Time - Begin_Time;
    Shoot_Event = Shoot_Event_None;

    if(Shoot_Status != Shoot_Disenable && *Enable_Signal == false)
    {
        Shoot_Event = Shoot_Event_Disenable;
    }

    if(Shoot_Status == Shoot_Disenable && *Enable_Signal == true)
    {
        Shoot_Event = Shoot_Event_Enable;
    }
    else if(Shoot_Status == Shoot_Enable && *Fire_Signal == true)
    {
        Shoot_Event = Shoot_Event_Fire;
        *Fire_Signal = false;
    }
    else if(Shoot_Status == Shoot_Fireing && Shoot_Stuck_Check())
    {
        Shoot_Event = Shoot_Event_Stuck;
    }
    else if(Shoot_Status == Shoot_Fireing && (fabs(Motor_DM_4340P.PID_Angle.Err)<0.1))
    {
        Shoot_Event = Shoot_Event_FireDone;
    }
    else if(Shoot_Status == Shoot_StuckReleasing && (fabs(Motor_DM_4340P.PID_Angle.Err)<0.1))
    {
        Shoot_Event = Shoot_Event_StuckRealseDone;
    }
}

/**
 * @brief 状态机运行函数
 * @param none
 */
void Class_Shoot::Shoot_FSM_Run()
{
    const auto previous_status = Shoot_Status;
    switch(Shoot_Status)
    {
        case Shoot_Disenable :
            switch(Shoot_Event)
            {
                case Shoot_Event_Enable :
                    Shoot_Status = Shoot_Enable;
                    break;
            }
        break;
        case Shoot_Enable :
            switch(Shoot_Event)
            {
                case Shoot_Event_Disenable :
                    Shoot_Status = Shoot_Disenable;
                    break;
                case Shoot_Event_Fire :
                    Shoot_Status = Shoot_Fireing;
                    break;
            }
        break;
        case Shoot_Fireing :
            switch(Shoot_Event)
            {
                case Shoot_Event_Disenable :
                    Shoot_Status = Shoot_Disenable;
                    break;
                case Shoot_Event_FireDone :
                    Shoot_Status = Shoot_Enable;
                    break;
                case Shoot_Event_Stuck :
                    Shoot_Status = Shoot_StuckReleasing;
                    break;
            }
        break;
        case Shoot_StuckReleasing :
            switch(Shoot_Event)
            {
                case Shoot_Event_Disenable :
                    Shoot_Status = Shoot_Disenable;
                    break;
                case Shoot_Event_StuckRealseDone :
                    Shoot_Status = Shoot_Enable;
                    break;
            }
        break;
    }

    //更新状态时间
    Current_Time = SYS_Timestamp.Get_Now_Millisecond();
    if(Shoot_Status != previous_status)
    {
        Begin_Time = Current_Time;
    }
    State_time = Current_Time - Begin_Time;
}
/* Function prototypes -------------------------------------------------------*/
