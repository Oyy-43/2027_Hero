#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "alg_pid.h"
#include "alg_basic.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 底盘控制模式
 *
 */
enum Enum_Chassis_Control_Mode
{
    Chassis_Control_Mode_Stop = 0,
    Chassis_Control_Mode_Normal,
    Chassis_Control_Mode_Follow,
    Chassis_Control_Mode_Gyring,
};

/**
 * @brief 底盘数据结构
 *
 */
struct Struct_Chassis_Data
{
    float Max_Torque_per_Wheel;   // 单个动力轮最大输出扭矩, 单位 Nm
    float Wheel_Radius;           //轮子半径
    float Wheel_BaseX;            //前后轮距
    float Wheel_BaseY;            //左右轮距
    float Wheel_Base;             //轮子投影点距离底盘中心的距离Rsqrt((BaseX/2)^2+(BaseY/2)^2)

    //底盘目标速度，用于运动学逆解求出每个动力轮的目标速度
    float Target_Velocity_X;
    float Target_Velocity_Y;
    float Target_Velocity_W;

    //底盘目标扭矩,用于动力学逆解求出每个动力轮的目标扭矩
    float Target_Torque_Fx;
    float Target_Torque_Fy;
    float Target_Torque_Mz;

    //底盘当前状态信息,用于运动学正解求出底盘当前速度
    float Now_Velocity_X;
    float Now_Velocity_Y;
    float Now_Velocity_W;

    float Motor_Torque[4];            //四个动力轮的目标扭矩,为动力学逆解求出的结果
    float Motor_Target_Omega[4];   //四个动力轮的目标速度,为运动学逆解求出的结果
    float Motor_Now_Omega[4];      //四个动力轮的当前速度,为运动学正解求出的结果

    //云台与底盘的相对角度，用于小陀螺行进模式下的坐标转换
    float Gimbal_Delta_Angle;
};

/**
 * @brief 底盘控制类
 *
 */
class Class_Chassis_Control
{
  public:
    _PID_TypeDef PID_Moving;  //底盘平移速度PID

    _PID_TypeDef PID_Spin;    //旋转速度PID

    void Init();

    inline void Set_Target_Velocity(const float &__Target_Velocity_X, const float &__Target_Velocity_Y, const float &__Target_Velocity_W);

    inline void Set_Target_Torque(const float &__Target_Torque_Fx, const float &__Target_Torque_Fy, const float &__Target_Torque_Mz);

    inline void Set_Chassis_Control_Mode(const Enum_Chassis_Control_Mode &__Chassis_Control_Mode);

    void TIM_Calculate_PeriodElapsedCallback();   //求解运动学正逆解，动力学逆解，更新底盘状态信息
  protected:
    Enum_Chassis_Control_Mode Chassis_Control_Mode;

    Struct_Chassis_Data Chassis_Data;
};

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
    
    //运动学逆解，从底盘的目标速度和旋转速度求出每个动力轮的目标速度
    void Chassis_to_Motors();

    //运动学正解，从4个轮向电机的速度和舵向电机的角度求出底盘的当前平动速度和旋转速度
    void Motors_to_Chassis();

    //动力学逆解，从底盘的目标牵引力和旋转转矩求出每个动力轮的目标扭矩
    void Chassis_to_Motors_Torque();

    void TIM_Calculate_PeriodElapsedCallback();   //求解运动学正逆解，动力学逆解，更新底盘状态信息
  protected:
    float Steer_Target_Angle[4];     //四个舵轮目标角度(0左上 1左下 2右下 3右上)
    float Steer_Current_Angle[4];    //四个舵轮当前角度(0左上 1左下 2右下 3右上)
};


/**
 * @brief 麦轮底盘控制派生类
 * 
 */
class Mecanum_Chassis_Control : public Class_Chassis_Control
{
  public:  
    void Init(float __Max_Torque_per_Wheel, float __Wheel_Radius, float __Wheel_BaseX, float __Wheel_BaseY);

    inline void Set_Target_Velocity(const float &__Target_Velocity_X, const float &__Target_Velocity_Y, const float &__Target_Velocity_W);

    inline void Set_Target_Torque(const float &__Target_Torque_Fx, const float &__Target_Torque_Fy, const float &__Target_Torque_Mz);

    inline void Set_Chassis_Control_Mode(const Enum_Chassis_Control_Mode &__Chassis_Control_Mode);

    //运动学逆解，从地盘的目标速度和旋转速度求出每个动力轮的目标速度
    void Chassis_to_Motors(Struct_Chassis_Data *chassis_data, float *motor_torque, float *motor_velocity);

    void TIM_Calculate_PeriodElapsedCallback();   //求解运动学正逆解，动力学逆解，更新底盘状态信息
  protected:
};


/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif //__CHASSIS_CONTROL_H__