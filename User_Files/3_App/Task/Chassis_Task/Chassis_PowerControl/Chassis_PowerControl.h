#pragma once
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class Class_Chassis_PowerControl
{
public:
    void Init(const float &__Power_limit_max = 50.0f,const float &__buffer_max = 40.0f);

    //更新目标输入的功率
    void Power_Measure(const float (&__Motor_Power)[4]);

    //底盘功率分配计算
    void Power_calculate_distribution(const float (&__Target_Input_Power)[4], const float (&__Motor_Omega)[4]);

    void Get_Solution_Torque(const float (&__Power_Max)[4], const float (&__Motor_Omega)[4]);

    //预计输入的功率
    float Target_Input_Power[4] = {0.0f,0.0f,0.0f,0.0f};

    //功率限制最大值
    float Output_Power[4] = {0.0f,0.0f,0.0f,0.0f};

    //目标输出扭矩
    float Target_Output_Torque[4] = {0.0f,0.0f,0.0f,0.0f};

    //输出比例分配扭矩
    float Scale_K = 1.0f; 

    float Current_Total_Power = 0.0f; //当前总功率

    float Torque_PositiveSolution[4] = {0.0f,0.0f,0.0f,0.0f}; //功率正解

    float Torque_NegativeSolution[4] = {0.0f,0.0f,0.0f,0.0f}; //功率负解

private:

    float Power_limit_max = 50.0f; //功率限制最大值

    float buffer_max = 40.0f; //功缓冲最大值

    float Power_K1 = 0.0000308553062f;

    float Power_K2 = 0.0000013128074f;

    float Power_A = 1.315f;

};
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

