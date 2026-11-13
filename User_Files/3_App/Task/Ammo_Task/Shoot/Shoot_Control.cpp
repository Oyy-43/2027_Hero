/**
 * @file Shoot_Control.cpp
 * @author oyyp
 * @brief 射击任务管理
 * @version 0.1
 * @date 2026-11-13 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "Shoot_Control.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
void Class_Shoot::Shoot_Init(Class_Motor_DJI_C620 __Motor_Instances[3],const float &__Fire_Torque,const float &__Increase_Value,const float &__Decrease_Value,
    const Enum_Slope_First &__Slope_First)
{
    for(int i=0;i<3;i++)
    {
        Motor_Instances[i] = &__Motor_Instances[i];
    }
    Fire_Torque = __Fire_Torque;
    for(int i=0;i<3;i++)
    {
        Slope_Motor_Omega[i].Init(__Increase_Value,__Decrease_Value,__Slope_First);
        Slope_Motor_Omega[i].Reset();
    }
    Target_Motor_Omega = 0.0f;
    Speed_Up_Flag = false;
    Shoot_Num = 0.0f;
    Shoot_Success_Count = 0;
}

void Class_Shoot::Shoot_TIM_Calculate_PeriodElapsedCallback()
{
    if(Speed_Up_Flag == false || Target_Motor_Omega == 0.0f)
    {
        Motor_Instances[0]->Set_Target_Omega(Slope_Motor_Omega[0].TIM_Calculate_GetOut(-Target_Motor_Omega,Motor_Instances[0]->Get_Now_Omega()));
        Motor_Instances[1]->Set_Target_Omega(Slope_Motor_Omega[1].TIM_Calculate_GetOut(-Target_Motor_Omega,Motor_Instances[1]->Get_Now_Omega()));
        Motor_Instances[2]->Set_Target_Omega(Slope_Motor_Omega[2].TIM_Calculate_GetOut(Target_Motor_Omega,Motor_Instances[2]->Get_Now_Omega()));
        
        Speed_Up_Flag = (Target_Motor_Omega != 0.0f) && Check_Shoot_Speed_Up_Flag();
    }
    else
    {

        Motor_Instances[0]->Set_Target_Omega(-Target_Motor_Omega);
        Slope_Motor_Omega[0].Reset(-Target_Motor_Omega);
        Motor_Instances[1]->Set_Target_Omega(-Target_Motor_Omega);
        Slope_Motor_Omega[1].Reset(-Target_Motor_Omega);
        Motor_Instances[2]->Set_Target_Omega(Target_Motor_Omega);
        Slope_Motor_Omega[2].Reset(Target_Motor_Omega);
    }
    Check_Shoot_Success_Flag();
}

bool Class_Shoot::Check_Shoot_Speed_Up_Flag()
{
    bool Speed_Down = (Motor_Instances[0]->Get_Now_Omega() < -Target_Motor_Omega*Check_Rate);
    bool Speed_Left = (Motor_Instances[2]->Get_Now_Omega() > Target_Motor_Omega*Check_Rate);
    bool Speed_Right = (Motor_Instances[1]->Get_Now_Omega() < -Target_Motor_Omega*Check_Rate);
    if(Speed_Down && Speed_Left && Speed_Right)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Class_Shoot::Check_Shoot_Success_Flag()
{
    if(Speed_Up_Flag == true)
    {
        bool Shoot_Down = (Motor_Instances[0]->Get_Now_Torque() < -Fire_Torque);
        bool Shoot_Left = (Motor_Instances[2]->Get_Now_Torque() > Fire_Torque);
        bool Shoot_Right = (Motor_Instances[1]->Get_Now_Torque() < -Fire_Torque);
        if(Shoot_Down && Shoot_Left && Shoot_Right)
        {
            if(Shoot_Success_Count < 5)
            {
                Shoot_Success_Count++;
                if(Shoot_Success_Count == 5)
                {
                    Shoot_Num++;
                    return true;
                }
            }
        }
        else
        {
            Shoot_Success_Count = 0;
        }
    }
    else
    {
        Shoot_Success_Count = 0;
    }
    return false;
}

/* Function prototypes -------------------------------------------------------*/

