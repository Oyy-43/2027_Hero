/**
 * @file Chassis_LiftControl.cpp
 * @author oyyp
 * @brief description
 * @version 0.1
 * @date 2026-10-30 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "Chassis_LiftControl.h"
#include "3_App/Data_Transfer/App_DataTransfer.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

void Class_Chassis_LiftControl::Lift_Init()
{
    FsmStatus.Status = Chassis_lift_status_IDLE;
    FsmStatus.Event = Chassis_lift_event_NONE;
    FsmStatus.State_time = 0;
    FsmStatus.Begin_time = SYS_Timestamp.Get_Now_Microsecond();
    FsmStatus.Current_time = FsmStatus.Begin_time;
}

void Class_Chassis_LiftControl::Lift_Run()
{
    Lift_Event_Update();
    switch (FsmStatus.Status)
    {
        //空闲状态，可手控前后的抬升
        case Chassis_lift_status_IDLE:
        switch (FsmStatus.Event)
        {
            case Chassis_lift_event_Front_UP:
                Motor_DM_6216[0].Set_Slope_Target_Angle(-Front_Up_Angle);
                Motor_DM_6216[1].Set_Slope_Target_Angle(Front_Up_Angle);
                Motor_DM_6216[2].Set_Slope_Target_Angle(0.0f);
                FsmStatus.Status = Chassis_lift_status_Front_Uping;
                FsmStatus.Begin_time = SYS_Timestamp.Get_Now_Microsecond();
                FsmStatus.State_time = 0;
                break;
            default:
                break;    
        }
        break;
        //前抬升中状态，等待前抬升完成
        case Chassis_lift_status_Front_Uping:
        switch (FsmStatus.Event)
        {
            case Chassis_lift_event_Front_UPdone:
                Motor_DM_6216[0].Set_Slope_Target_Angle(-Front_Up_Angle);        //主要是前面两个抬起来
                Motor_DM_6216[1].Set_Slope_Target_Angle(Front_Up_Angle);
                Motor_DM_6216[2].Set_Target_Angle(0.0f);
                FsmStatus.Status = Chassis_lift_status_Front_Up;
                FsmStatus.Begin_time = SYS_Timestamp.Get_Now_Microsecond();
                FsmStatus.State_time = 0;
                break;
            default:
                break;
        }
        break;
        //前抬升完成状态，可以后伸展
        case Chassis_lift_status_Front_Up:
        switch (FsmStatus.Event)
        {
            case Chassis_lift_event_Lift_UP:
                Motor_DM_6216[0].Set_Slope_Target_Angle(-Front_Up_Angle);        //前面和后面抬起来,使pitch尽量平衡
                Motor_DM_6216[1].Set_Slope_Target_Angle(Front_Up_Angle);
                Motor_DM_6216[2].Set_Target_Angle(Back_Stretch_Angle);
                FsmStatus.Status = Chassis_lift_status_Lift_Uping;
                FsmStatus.Begin_time = SYS_Timestamp.Get_Now_Microsecond();
                FsmStatus.State_time = 0;
                break;
            default:
                break;
        }
        break;
        //整体抬升中状态，等待整体抬升完成
        case Chassis_lift_status_Lift_Uping:
        switch (FsmStatus.Event)
        {
            case Chassis_lift_event_Lift_UPdone:
                Motor_DM_6216[0].Set_Slope_Target_Angle(-Front_Up_Angle);
                Motor_DM_6216[1].Set_Slope_Target_Angle(Front_Up_Angle);
                Motor_DM_6216[2].Set_Target_Angle(Back_Stretch_Angle);
                FsmStatus.Status = Chassis_lift_status_Lift_Up;
                FsmStatus.Begin_time = SYS_Timestamp.Get_Now_Microsecond();
                FsmStatus.State_time = 0;
                break;
            default:
                break;
        }
        break;
        //整体抬升完成状态，可以后伸展
        case Chassis_lift_status_Lift_Up:
        switch (FsmStatus.Event)
        {
            case Chassis_lift_event_Back_Revoke:
                Motor_DM_6216[0].Set_Slope_Target_Angle(0.0f);
                Motor_DM_6216[1].Set_Slope_Target_Angle(0.0f);
                Motor_DM_6216[2].Set_Target_Angle(Back_Stretch_Angle);
                FsmStatus.Status = Chassis_lift_status_Back_Revoking;
                FsmStatus.Begin_time = SYS_Timestamp.Get_Now_Microsecond();
                FsmStatus.State_time = 0;
                break;
            default:
                break;
        }
        break;
        //后伸展中状态，等待后伸展完成
        case Chassis_lift_status_Back_Revoking:
        switch (FsmStatus.Event)
        {
            case Chassis_lift_event_Back_Reset:
                Motor_DM_6216[0].Set_Slope_Target_Angle(0.0f);
                Motor_DM_6216[1].Set_Slope_Target_Angle(0.0f);
                Motor_DM_6216[2].Set_Target_Angle(0.0f);
                FsmStatus.Status = Chassis_lift_status_IDLE;
                FsmStatus.Begin_time = SYS_Timestamp.Get_Now_Microsecond();
                FsmStatus.State_time = 0;
                break;
            default:
                break; 
        }
        break;

        default:
            break;
    }
}

void Class_Chassis_LiftControl::Lift_Event_Update()
{
    FsmStatus.Current_time = SYS_Timestamp.Get_Now_Microsecond();
    FsmStatus.State_time = FsmStatus.Current_time - FsmStatus.Begin_time;
    FsmStatus.Event = Chassis_lift_event_NONE;
    if(FsmStatus.Status == Chassis_lift_status_IDLE && Remote_Data_Transfer.Remote_Key_CH12.Key_Status == Remote_Key_Status_TRIG_FREE_PRESSED )
    {
        FsmStatus.Event = Chassis_lift_event_Front_UP;
    }
    if(FsmStatus.Status == Chassis_lift_status_Front_Uping)
    {
        bool left_get = Lift_Position_Check(Chassis_lift_motor_FrontLeft);
        bool right_get = Lift_Position_Check(Chassis_lift_motor_FrontRight);
        if(left_get && right_get)
        {
            FsmStatus.Event = Chassis_lift_event_Front_UPdone;
        }
    }
    if(FsmStatus.Status == Chassis_lift_status_Front_Up && Remote_Data_Transfer.Remote_Key_CH13.Key_Status == Remote_Key_Status_TRIG_FREE_PRESSED)
    {
        FsmStatus.Event = Chassis_lift_event_Lift_UP;
    }
    if(FsmStatus.Status == Chassis_lift_status_Lift_Uping)
    {
        bool left_get = Lift_Position_Check(Chassis_lift_motor_FrontLeft);
        bool right_get = Lift_Position_Check(Chassis_lift_motor_FrontRight);
        bool back_get = Lift_Position_Check(Chassis_lift_motor_Back);
        if(left_get && right_get && back_get)
        {
            FsmStatus.Event = Chassis_lift_event_Lift_UPdone;
        }
    }
    if(FsmStatus.Status == Chassis_lift_status_Lift_Up && Remote_Data_Transfer.Remote_Key_CH12.Key_Status == Remote_Key_Status_TRIG_PRESSED_FREE)
    {
        FsmStatus.Event = Chassis_lift_event_Back_Revoke;
    }
    if(FsmStatus.Status == Chassis_lift_status_Back_Revoking && Lift_Position_Check(Chassis_lift_motor_Back))
    {
        FsmStatus.Event = Chassis_lift_event_Back_Reset;
    }
}

bool Class_Chassis_LiftControl::Lift_Position_Check(ChassisLift_MotorEnum __Motor_Enum)
{
    Class_Motor_DM_Normal &tmp_motor = Motor_DM_6216[__Motor_Enum];

    float tmp_angle_err = Basic_Math_Modulus_Normalization(tmp_motor.Get_Target_Angle() - tmp_motor.Get_Now_Angle(),
                                                          2.0f * tmp_motor.Get_Angle_Max());

    return (ABS(tmp_angle_err) < Position_threshold);
}
/* Function prototypes -------------------------------------------------------*/



