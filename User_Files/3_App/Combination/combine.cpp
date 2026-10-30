/**
 * @file combine.cpp
 * @author oyyp
 * @brief description
 * @version 0.1
 * @date 2026-10-27 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "combine.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "3_App/Data_Transfer/App_DataTransfer.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
float test_combine=0.0f;
/* Private function declarations ---------------------------------------------*/
void Combine_FSM::Combine_FSM_Init()
{
    State = Combine_State_DisSetZero;
    Event = Combine_Event_None;
    State_time = 0.0f;
    Begin_Time = SYS_Timestamp.Get_Now_Millisecond();
    Current_Time = Begin_Time;
    for(uint8_t i = 0; i < 2; i++)
    {
        Stock_Flag[i] = false;
        Stock_Time[i] = 0;
        ZeroPosition[i] = 0.0f;
    }
}

void Combine_FSM::Combine_FSM_Run()
{
    Combine_FSM_Event_Update();
    switch(State)
    {
        case(Combine_State_DisSetZero):
        {
            if(Event == Combine_Event_SetZero)
            {
                Motor_DJI_C610[0].Set_Target_Omega(3.14f);
                Motor_DJI_C610[1].Set_Target_Omega(-4.71f);
                State = Combine_State_SetingZero;
                Begin_Time = SYS_Timestamp.Get_Now_Millisecond();
            }
            break;
        }
        case(Combine_State_SetingZero):
        {
            if(Event == Combine_Event_Stock)
            {   
                State = Combine_State_Free;
                Stock_Flag[0] = false;
                Stock_Flag[1] = false;
                Begin_Time = SYS_Timestamp.Get_Now_Millisecond();
            }
            break;
        }
        case(Combine_State_Combined):
        {
            if(Event == Combine_Event_To_Free)
            {
                State = Combine_State_Freeing;
                Begin_Time = SYS_Timestamp.Get_Now_Millisecond();
            }
            break;
        }
        case (Combine_State_Freeing):
        {
            if(Event == Combine_Event_FreeDone)
            {
                Motor_DJI_C610[0].Set_Target_Angle(ZeroPosition[0]);
                Motor_DJI_C610[1].Set_Target_Angle(ZeroPosition[1]);
                State = Combine_State_Free;
                Begin_Time = SYS_Timestamp.Get_Now_Millisecond();
            }
            break;
        }
        case (Combine_State_Free):
        {
            if(Event == Combine_Event_To_Combined)
            {
                Motor_DJI_C610[0].Set_Target_Angle(ZeroPosition[0]-23.5f);
                Motor_DJI_C610[1].Set_Target_Angle(ZeroPosition[1]+23.5f);
                State = Combine_State_Combining;
                Begin_Time = SYS_Timestamp.Get_Now_Millisecond();
            }
            break;
        }
        case (Combine_State_Combining):
        {
            if(Event == Combine_Event_CombinedDone)
            {
                State = Combine_State_Combined;
                Begin_Time = SYS_Timestamp.Get_Now_Millisecond();
            }
            break;
        }
    }
}

void Combine_FSM::Combine_FSM_Event_Update()
{
    Current_Time = SYS_Timestamp.Get_Now_Millisecond();
    CurrentPosition[0] = Motor_DJI_C610[0].Get_Now_Angle();
    CurrentPosition[1] = Motor_DJI_C610[1].Get_Now_Angle();
    State_time = Current_Time - Begin_Time;
    Event = Combine_Event_None;

    if(State == Combine_State_DisSetZero && Remote_Data_Transfer.Remote_Key_CH12.Key_Status==Remote_Key_Status_TRIG_PRESSED_FREE)
    {
        Event = Combine_Event_SetZero;
    }
    if(State == Combine_State_SetingZero)
    {
        bool Left_Stock = Combine_FSM_Stock_Check(Left_Side);
        bool Right_Stock = Combine_FSM_Stock_Check(Right_Side);
        // if(Left_Stock == true && Right_Stock == true)
        if(Right_Stock == true)
        {
            Event = Combine_Event_Stock;
        }
    }
    if(State == Combine_State_Combined && Remote_Data_Transfer.Remote_Key_CH13.Key_Status==Remote_Key_Status_TRIG_PRESSED_FREE)
    {
        Event = Combine_Event_To_Free;
    }
    if(State == Combine_State_Freeing && Combine_FSM_Position_Check(Left_Side)==true && Combine_FSM_Position_Check(Right_Side)==true)
    {
        Event = Combine_Event_FreeDone;
    }
    if(State == Combine_State_Free &&  Remote_Data_Transfer.Remote_Key_CH13.Key_Status==Remote_Key_Status_TRIG_FREE_PRESSED)
    {
        Event = Combine_Event_To_Combined;
    }
    if(State == Combine_State_Combining && Combine_FSM_Position_Check(Left_Side)==true && Combine_FSM_Position_Check(Right_Side)==true)
    {
        Event = Combine_Event_CombinedDone;
    }
}

bool Combine_FSM::Combine_FSM_Stock_Check(Combine_Side __side)
{
    //已判定堵转后不再重复计数和重复改写零点, 否则零点会每周期漂移
    if (Stock_Flag[__side] == true)
    {
        Stock_Time[__side] = 0;
        return true;
    }

    if (fabs(Motor_DJI_C610[__side].Get_Now_Torque()) >= 0.5f)
    {
        //电机堵转计数
        Stock_Time[__side]++;
    }
    else
    {
        Stock_Time[__side] = 0;
    }
    if (Stock_Time[__side] > 100)
    {
        //电机堵转超过100ms
        //制动电机
        Motor_DJI_C610[__side].Set_Target_Omega(0.0f);
        Motor_DJI_C610[__side].PID_Omega.ITerm=0.0f;
        Motor_DJI_C610[__side].PID_Omega.Iout=0.0f;
        Motor_DJI_C610[__side].PID_Omega.Output=0.0f;

        switch(__side)
        {
            case Left_Side:
                ZeroPosition[__side] = Motor_DJI_C610[__side].Get_Now_Angle()-0.5f;
                break;
            case Right_Side:
                ZeroPosition[__side] = Motor_DJI_C610[__side].Get_Now_Angle()+0.5f;
                break;
        }
        Motor_DJI_C610[__side].Set_Target_Angle(ZeroPosition[__side]);
        Motor_DJI_C610[__side].Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
        Stock_Flag[__side] = true;
    }
    return Stock_Flag[__side];
}
bool Combine_FSM::Combine_FSM_Position_Check(Combine_Side __side)
{
    return (Motor_DJI_C610[__side].PID_Angle.abs_Err < 0.1f);
}
/* Function prototypes -------------------------------------------------------*/

