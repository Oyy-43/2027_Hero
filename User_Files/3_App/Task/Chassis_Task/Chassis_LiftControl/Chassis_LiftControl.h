#pragma once
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "3_App/Task/Chassis_Task/Chassis_Task.h"

/* Exported macros -----------------------------------------------------------*/
#define Position_threshold 0.1f


/* Exported types ------------------------------------------------------------*/
enum ChassisLift_MotorEnum
{
    Chassis_lift_motor_FrontLeft = 0,
    Chassis_lift_motor_FrontRight,
    Chassis_lift_motor_Back,
};


enum ChassisLift_Status
{
    Chassis_lift_status_IDLE = 0,
    Chassis_lift_status_Front_Uping,        //前四轮抬起中
    Chassis_lift_status_Front_Up,           //前四轮抬起完毕
    Chassis_lift_status_Lift_Uping,         //整体抬升中
    Chassis_lift_status_Lift_Up,            //整体抬升完毕
    Chassis_lift_status_Back_Revoking,      //后伸收回
};

enum ChassisLift_Event
{
    Chassis_lift_event_NONE = 0,
    Chassis_lift_event_Front_UP,            //前四轮抬升
    Chassis_lift_event_Front_UPdone,        //前四轮抬升完毕
    Chassis_lift_event_Lift_UP,             //整体抬升
    Chassis_lift_event_Lift_UPdone,         //整体抬升完毕
    Chassis_lift_event_Back_Revoke,         //后伸收回
    Chassis_lift_event_Back_Reset,          //复位，强行回正
};

struct Chassis_LiftFsmStatus
{
    ChassisLift_Status Status;
    ChassisLift_Event Event;
    uint64_t State_time;
    uint64_t Begin_time;
    uint64_t Current_time;
};

class Class_Chassis_LiftControl
{
public:
    Chassis_LiftFsmStatus FsmStatus;

    void Lift_Init();

    void Lift_Run();

    void Lift_Event_Update();

    float Get_Chassis_Pitch_Err();

    bool Lift_Position_Check(ChassisLift_MotorEnum __Motor_Enum);

    bool Chassis_Pitch_Stable_Check();

private:

};

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

