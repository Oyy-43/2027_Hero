#ifndef __STEER_CONTROL_H
#define __STEER_CONTROL_H__

/* Includes ------------------------------------------------------------------*/
#include "chassis_control.h"


/* Exported macros -----------------------------------------------------------*/
#define STEER_ANGLE_PI        3.14159265f   // 舵向角半周期(π)
#define STEER_ANGLE_HALF_PI   1.57079633f   // 舵向角四分之一周期(π/2)
#define STEER_ANGLE_DEADZONE  0.01f         // 舵向角死区, 到位判定
#define STEER_ANGLE_MODULUS   6.28318531f   // 舵向角归一化模数(2π)
#define STEER_HOLD_VELOCITY_THRESHOLD  0.01f  // 静止判定阈值(m/s), 目标速度小于该值则舵向保持当前角
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/**
 * @brief 舵轮底盘控制派生类
 *
 */
class Steer_Chassis_Control : public Class_Chassis_Control
{
  public:
    void Init(float __Max_Torque_per_Wheel, float __Wheel_Radius, float __Wheel_BaseX, float __Wheel_BaseY);

    inline void Set_Target_Velocity(const float &__Target_Velocity_X, const float &__Target_Velocity_Y, const float &__Target_Velocity_W);

    inline void Set_Target_Torque(const float &__Target_Torque_Fx, const float &__Target_Torque_Fy, const float &__Target_Torque_Mz);

    inline void Set_Chassis_Control_Mode(const Enum_Chassis_Control_Mode &__Chassis_Control_Mode);

    inline float Get_Chassis_Now_Velocity_X() const;

    inline float Get_Chassis_Now_Velocity_Y() const;

    inline float Get_Chassis_Now_Velocity_W() const;

    inline const float *Get_Steer_Target_Angle() const;   //四个舵轮有效目标角(已按最小转角策略更新)

    //运动学逆解，从底盘的目标速度和旋转速度求出每个动力轮的目标速度
    void Chassis_to_Motors();

    //运动学正解，从4个轮向电机的速度和舵向电机的角度求出底盘的当前平动速度和旋转速度
    void Motors_to_Chassis();

    //动力学逆解，从底盘的目标牵引力和旋转转矩求出每个动力轮的目标扭矩
    void Chassis_to_Motors_Torque();

    void TIM_Calculate_PeriodElapsedCallback(float Motor1_Omega, float Motor2_Omega, float Motor3_Omega, float Motor4_Omega,
    float Steer1_Angle, float Steer2_Angle, float Steer3_Angle, float Steer4_Angle); //求解运动学正逆解，动力学逆解，更新底盘状态信息

    void Cal_Angle_Dir(uint8_t Steer_Num); 

  protected:
    float Steer_Target_Angle[4];     //四个舵轮运动学目标角度(连续unwrap, 由 Chassis_to_Motors 维护)

    float Steer_Effective_Angle[4];  //四个舵轮有效目标角度(已按最小转角策略更新, 供电机输出, 由 Cal_Angle_Dir 更新)

    float Steer_Current_Angle[4];    //四个舵轮当前角度(0左上 1左下 2右下 3右上)

    float Motor_Dir[4];              //四个动力轮的旋转方向(0左上 1左下 2右下 3右上)

    float Motor_Cos_Down[4];         //四个动力轮的旋转速度减速余弦值(0左上 1左下 2右下 3右上)
};

/* Exported function declarations --------------------------------------------*/

inline float Steer_Chassis_Control::Get_Chassis_Now_Velocity_X() const
{
    return Chassis_Data.Now_Velocity_X;
}

inline float Steer_Chassis_Control::Get_Chassis_Now_Velocity_Y() const
{
    return Chassis_Data.Now_Velocity_Y;
}

inline float Steer_Chassis_Control::Get_Chassis_Now_Velocity_W() const
{
    return Chassis_Data.Now_Velocity_W;
}

inline const float *Steer_Chassis_Control::Get_Steer_Target_Angle() const
{
    return (Steer_Effective_Angle);
}

inline void Steer_Chassis_Control::Set_Target_Velocity(const float &__Target_Velocity_X, const float &__Target_Velocity_Y, const float &__Target_Velocity_W)
{
    Chassis_Data.Target_Velocity_X = __Target_Velocity_X;
    Chassis_Data.Target_Velocity_Y = __Target_Velocity_Y;
    Chassis_Data.Target_Velocity_W = __Target_Velocity_W;
}

inline void Steer_Chassis_Control::Set_Target_Torque(const float &__Target_Torque_Fx, const float &__Target_Torque_Fy, const float &__Target_Torque_Mz)
{
    Chassis_Data.Target_Torque_Fx = __Target_Torque_Fx;
    Chassis_Data.Target_Torque_Fy = __Target_Torque_Fy;
    Chassis_Data.Target_Torque_Mz = __Target_Torque_Mz;
}

inline void Steer_Chassis_Control::Set_Chassis_Control_Mode(const Enum_Chassis_Control_Mode &__Chassis_Control_Mode)
{
    Chassis_Control_Mode = __Chassis_Control_Mode;
}

#endif /* __STEER_CONTROL_H */