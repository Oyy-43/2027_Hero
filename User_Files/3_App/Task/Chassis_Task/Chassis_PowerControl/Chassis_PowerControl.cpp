/**
 * @file Chassis_PowerControl.cpp
 * @author oyyp
 * @brief description
 * @version 0.1
 * @date 2026-10-30 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "Chassis_PowerControl.h"
#include <math.h>
#include "cmsis_os2.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
void Class_Chassis_PowerControl::Init(const float &__Power_limit_max, const float &__buffer_max)
{
    Power_limit_max = __Power_limit_max;
    buffer_max = __buffer_max;

    for(uint8_t i = 0;i<4;i++)
    {
        Target_Input_Power[i] = 0.0f;
        Output_Power[i] = 0.0f;
        Target_Output_Torque[i] = 0.0f;
        Scale_K = 1.0f;
    }
}

void Class_Chassis_PowerControl::Power_calculate_distribution(const float(&__Target_Input_Power)[4],const float(&__Motor_Omega)[4])
{
    for(uint8_t i = 0;i<4;i++)
    {
        Target_Input_Power[i] = __Target_Input_Power[i];
    }


    Current_Total_Power = Target_Input_Power[0] + Target_Input_Power[1] + Target_Input_Power[2] + Target_Input_Power[3];

    if(Current_Total_Power > Power_limit_max)
    {
        Scale_K = Power_limit_max / Current_Total_Power;
    }
    else
    {
        Scale_K = 1.0f;
    }

    for(uint8_t i = 0;i<4;i++)
    {
        if(Target_Input_Power[i] >= 0)
        {
            Output_Power[i] = Target_Input_Power[i] * Scale_K;
        }
        else
        {
            Output_Power[i] = Target_Input_Power[i];
        }
    }
    Get_Solution_Torque(Output_Power, __Motor_Omega);
}

void Class_Chassis_PowerControl::Get_Solution_Torque(const float (&__Power_Max)[4], const float (&__Motor_Omega)[4])
{
    //方程: k2*T^2 + w*T + (k1*w^2 + a - Pmax) = 0
    //T12 = (-w ± sqrt(w^2 - 4*k2*(k1*w^2 + a - Pmax))) / (2*k2)
    for(uint8_t i = 0;i<4;i++)
    {
        const float w = __Motor_Omega[i];
        const float b = w;
        const float c = Power_K1 * w * w + Power_A - __Power_Max[i];

        float delta = b * b - 4.0f * Power_K2 * c;

        if(delta <= 0.0f)
        {
            //无实数解(该转速下无法把功率压到上限内),输出0
            Torque_PositiveSolution[i] = 0.0f;
            Torque_NegativeSolution[i] = 0.0f;
        }
        else
        {
            const float sqrt_delta = sqrtf(delta);
            const float inv_2k2 = 1.0f / (2.0f * Power_K2);

            //因为 k2 > 0,取 + 的根恒 >= 0,取 - 的根恒 <= 0
            Torque_PositiveSolution[i] = (-b + sqrt_delta) * inv_2k2;
            Torque_NegativeSolution[i] = (-b - sqrt_delta) * inv_2k2;
        }
    }
}

/* Function prototypes -------------------------------------------------------*/

