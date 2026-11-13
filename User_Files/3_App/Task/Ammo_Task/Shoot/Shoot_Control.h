#pragma once
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "2_Device/Motor/Motor_DJI/drv_motor_dji.h"
#include "1_Middleware/Algorithm/Slope/alg_slope.h"

/* Exported macros -----------------------------------------------------------*/
#define Check_Rate 0.9f

/* Exported types ------------------------------------------------------------*/
class Class_Shoot
{
    public:
    // 初始化函数,绑定三个摩擦轮电机,和检测射出的力矩阈值
    void Shoot_Init(Class_Motor_DJI_C620 __Motor_Instances[3],const float &__Fire_Torque,const float &__Increase_Value,const float &__Decrease_Value,
    const Enum_Slope_First &__Slope_First = Slope_First_REAL);

    // 定时器回调函数，用于控制
    void Shoot_TIM_Calculate_PeriodElapsedCallback();

    // 设置摩擦轮速度
    inline void Set_Target_Motor_Omega(const float &__Target_Motor_Omega);

    inline void Set_Shoot_Fire_Torque(const float &__Fire_Torque);

    bool Check_Shoot_Speed_Up_Flag();

    bool Check_Shoot_Success_Flag();


    private:
    //电机示例
    Class_Slope Slope_Motor_Omega[3];
    
    Class_Motor_DJI_C620* Motor_Instances[3];
    
    //目标的摩擦轮转速
    float Target_Motor_Omega;
    
    //射出弹丸数量
    float Shoot_Num;

    uint8_t Shoot_Success_Count = 0;

    //加速完成标志位
    bool Speed_Up_Flag;

    //发射力矩，用于判断是否成功将弹丸发射出去
    float Fire_Torque = 0.0f; 

};
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
inline void Class_Shoot::Set_Target_Motor_Omega(const float &__Target_Motor_Omega)
{
    Target_Motor_Omega = __Target_Motor_Omega;
}

inline void Class_Shoot::Set_Shoot_Fire_Torque(const float &__Fire_Torque)
{
    Fire_Torque = __Fire_Torque;
}
