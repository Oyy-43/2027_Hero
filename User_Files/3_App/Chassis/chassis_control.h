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

    //底盘当前状态信息,为运动学正解的结果
    float Now_Velocity_X;
    float Now_Velocity_Y;
    float Now_Velocity_W;

    float Motor_Target_Torque[4];     //四个动力轮的目标扭矩,为动力学逆解求出的结果
    float Motor_Target_Omega[4];      //四个动力轮的目标速度,为运动学逆解求出的结果
    float Motor_Now_Omega[4];         //四个动力轮的当前速度,为运动学正解求出的结果

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
    _PID_TypeDef PID_Moving_X;  //底盘X向平移速度PID

    _PID_TypeDef PID_Moving_Y;  //底盘Y向平移速度PID

    _PID_TypeDef PID_Spin;    //旋转速度PID

    inline void Set_X_Target(const float &__Target_Velocity_X);

    inline void Set_Y_Target(const float &__Target_Velocity_Y);

    inline void Set_W_Target(const float &__Target_Velocity_W);

    inline float *Get_Motor_Target_Torque();

    inline float *Get_Motor_Target_Omega();

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
/**
 * @brief 获取四个动力轮的目标扭矩数组首地址, Nm
 *
 * @return float* 指向 Motor_Target_Torque[4] 的指针
 */
inline float *Class_Chassis_Control::Get_Motor_Target_Torque()
{
    return (Chassis_Data.Motor_Target_Torque);
}

/**
 * @brief 获取四个动力轮的目标速度数组首地址, rad/s
 *
 * @return float* 指向 Motor_Target_Omega[4] 的指针
 */
inline float *Class_Chassis_Control::Get_Motor_Target_Omega()
{
    return (Chassis_Data.Motor_Target_Omega);
}

#endif //__CHASSIS_CONTROL_H__