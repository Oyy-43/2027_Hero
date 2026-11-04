#pragma once
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "3_App/Task/Chassis_Task/Chassis_Task.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
enum ChassisLift_Status
{
    Chassis_lift_status_IDLE = 0,
    Chassis_lift_status_Front_Uping,
    Chassis_lift_status_Front_Up,
    Chassis_lift_status_Front_Downing,
    Chassis_lift_status_Front_Down,
    Chassis_lift_status_Back_Stretching,
    Chassis_lift_status_Back_Stretch,
};

enum ChassisLift_Event
{
    Chassis_lift_event_NONE = 0,
    Chassis_lift_event_Front_UP,
    Chassis_lift_event_Front_Down,
    Chassis_lift_event_Back_Stretch,
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

    void Lift_Position_Check();

private:

};

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

