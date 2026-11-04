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


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

void Class_Chassis_LiftControl::Lift_Init()
{
    FsmStatus.Status = Chassis_lift_status_IDLE;
    FsmStatus.Event = Chassis_lift_event_NONE;
    FsmStatus.State_time = 0.0f;
    FsmStatus.Begin_time = SYS_Timestamp.Get_Now_Microsecond();
    FsmStatus.Current_time = FsmStatus.Begin_time;
}

void Class_Chassis_LiftControl::Lift_Run()
{
    Lift_Event_Update();
    switch (FsmStatus.Status)
    {
        case Chassis_lift_status_IDLE:
        switch (FsmStatus.Event)
        {
            default:
                break;
        }
        break;

        case Chassis_lift_status_Front_Uping:
        switch (FsmStatus.Event)
        {
            default:
                break;
        }
        break;

        case Chassis_lift_status_Front_Up:
        switch (FsmStatus.Event)
        {
            default:
                break;
        }
        break;

        case Chassis_lift_status_Front_Downing:
        switch (FsmStatus.Event)
        {
            default:
                break;
        }
        break;

        case Chassis_lift_status_Front_Down:
        switch (FsmStatus.Event)
        {
            default:
                break;
        }
        break;

        case Chassis_lift_status_Back_Stretching:
        switch (FsmStatus.Event)
        {
            default:
                break;
        }
        break;

        case Chassis_lift_status_Back_Stretch:
        switch (FsmStatus.Event)
        {
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
    
}

/* Function prototypes -------------------------------------------------------*/



