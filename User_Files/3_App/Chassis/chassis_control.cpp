/**
 * @file chassis_control.cpp
 * @author name
 * @brief 底盘控制相关源文件
 * @version 0.1
 * @date 2026-08-28 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis_control.h"
#include <math.h>
#include "dsp/fast_math_functions.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
/**
 * @brief 底盘物理信息和pid控制器初始化
 * 
 * @param __Max_Torque_per_Wheel 单位Nm, 单个动力轮最大输出扭矩
 * @param __Wheel_Radius 单位m, 轮子半径
 * @param __Wheel_BaseX  单位m, 前后轮距
 * @param __Wheel_BaseY  单位m, 左右轮距
 */
void Steer_Chassis_Control::Init(float __Max_Torque_per_Wheel, float __Wheel_Radius, float __Wheel_BaseX, float __Wheel_BaseY)
{
    Chassis_Data.Max_Torque_per_Wheel = __Max_Torque_per_Wheel;
    Chassis_Data.Wheel_Radius = __Wheel_Radius;
    Chassis_Data.Wheel_BaseX = __Wheel_BaseX;
    Chassis_Data.Wheel_BaseY = __Wheel_BaseY;

    // 尺寸单位均为 m, 直接使用
    const float radius_m = __Wheel_Radius;
    const float half_base_x = __Wheel_BaseX * 0.5f;  // 前后半轮距 a
    const float half_base_y = __Wheel_BaseY * 0.5f;  // 左右半轮距 b
    Chassis_Data.Wheel_Base = sqrtf(half_base_x * half_base_x + half_base_y * half_base_y);

    // 单轮最大驱动力: f_max = T_max / r
    const float f_max = Chassis_Data.Max_Torque_per_Wheel / radius_m;

    // 底盘层最大净牵引力(4轮同向): F_max = 4 * f_max
    const float F_max = 4.0f * f_max;

    // 底盘层最大净旋转力矩(4轮切向): M_z_max = 4 * f_max * sqrt(a^2 + b^2)
    const float M_z_max = 4.0f * f_max * sqrtf(half_base_x * half_base_x + half_base_y * half_base_y);

    // 平移速度环: 最大输出=最大净牵引力, 积分限幅同步设为物理上限
    PID_Init(&PID_Moving,F_max,0, 0.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, NONE);

    // 旋转速度环: 最大输出=最大净旋转力矩, 积分限幅同步设为物理上限
    PID_Init(&PID_Spin, M_z_max,0, 0.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, NONE);
}

/**
 * @brief 运动学逆解，从底盘的目标速度和旋转速度求出每个动力轮的目标速度
 * 
 * @param chassis_data 
 */
void Steer_Chassis_Control::Chassis_to_Motors()
{
    const float vx = Chassis_Data.Target_Velocity_X;
    const float vy = Chassis_Data.Target_Velocity_Y;
    const float w  = Chassis_Data.Target_Velocity_W;
    const float s  = Chassis_Data.Wheel_Radius;
    const float r  = Chassis_Data.Wheel_Base;

    // 旋转角速度 ω 在各轮产生的切向单位方向(前X,左Y,上Z; 0左上 1左下 2右下 3右上)
    const float tx[4] = { -SQRT_2_half, -SQRT_2_half,  SQRT_2_half,  SQRT_2_half};
    const float ty[4] = {  SQRT_2_half, -SQRT_2_half, -SQRT_2_half,  SQRT_2_half};

    // 三部分分量的角速度: vx/s、vy/s、ω·r/s
    const float vx_s = vx / s;
    const float vy_s = vy / s;
    const float wr_s = w * r / s;

    for (int i = 0; i < 4; i++)
    {
        // 矢量合成该轮的角速度矢量
        const float wx = vx_s + wr_s * tx[i];
        const float wy = vy_s + wr_s * ty[i];

        arm_sqrt_f32(wx * wx + wy * wy, &Chassis_Data.Motor_Target_Omega[i]);  // 轮角速度
        arm_atan2_f32(wy, wx, &Steer_Target_Angle[i]);                         // 舵向角 = atan2(Y, X)
    }
}

/**
 * @brief 运动学正解，从4个轮向电机的速度和舵向电机的角度求出底盘的当前平动速度和旋转速度
 * 
 * @param chassis_data 
 */
void Steer_Chassis_Control::Motors_to_Chassis()
{
    const float s = Chassis_Data.Wheel_Radius;
    const float r = Chassis_Data.Wheel_Base;
    const float a = Chassis_Data.Wheel_BaseX * 0.5f;
    const float b = Chassis_Data.Wheel_BaseY * 0.5f;

    // 各轮坐标(前X,左Y), 用于求各轮速度绕底盘中心的力矩:
    // 0左上(+a,+b) 1左下(-a,+b) 2右下(-a,-b) 3右上(+a,-b)
    // 注: 正方形构型下 a = b = √2/2·r = SQRT_2_half·r, 与逆解/动力学的 ±√2/2 一致
    const float px[4] = { a, -a, -a, a};
    const float py[4] = { b,  b, -b, -b};

    float sum_vx = 0.0f, sum_vy = 0.0f, sum_m = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        const float vx_w = Chassis_Data.Motor_Now_Omega[i] * s * arm_cos_f32(Steer_Current_Angle[i]);
        const float vy_w = Chassis_Data.Motor_Now_Omega[i] * s * arm_sin_f32(Steer_Current_Angle[i]);

        sum_vx += vx_w;
        sum_vy += vy_w;
        sum_m  += -py[i] * vx_w + px[i] * vy_w;   // 该轮速度绕底盘中心的力矩
    }

    // 平动速度 = 各轮速度均值; 旋转速度 = 力矩和 / (4·r²)
    Chassis_Data.Now_Velocity_X = sum_vx / 4.0f;
    Chassis_Data.Now_Velocity_Y = sum_vy / 4.0f;
    Chassis_Data.Now_Velocity_W = sum_m / (4.0f * r * r);
}

void Steer_Chassis_Control::Chassis_to_Motors_Torque()
{
    const float Fx = Chassis_Data.Target_Torque_Fx;
    const float Fy = Chassis_Data.Target_Torque_Fy;
    const float Mz = Chassis_Data.Target_Torque_Mz;
    const float s  = Chassis_Data.Wheel_Radius;
    const float r  = Chassis_Data.Wheel_Base;


    // Mz 在各轮产生的切向单位方向(前X,左Y,上Z; 0左上 1左下 2右下 3右上)
    const float tx[4] = { -SQRT_2_half, -SQRT_2_half,  SQRT_2_half,  SQRT_2_half};
    const float ty[4] = {  SQRT_2_half, -SQRT_2_half, -SQRT_2_half,  SQRT_2_half};

    // 各轮由 Mz 产生的切向驱动力大小 = (Mz/4) / r
    const float f_rot = Mz / (4.0f * r);

    const float tau_max = Chassis_Data.Max_Torque_per_Wheel;

    for (int i = 0; i < 4; i++)
    {
        // 矢量合成: 该轮驱动力矢量 = Fx/4 + Fy/4 + Mz切向
        const float f_x = Fx * 0.25f + f_rot * tx[i];
        const float f_y = Fy * 0.25f + f_rot * ty[i];

        // 沿该轮舵向(运动学逆解得到)的有符号驱动力 = 力矢量在舵向的投影
        const float cos_a = arm_cos_f32(Steer_Current_Angle[i]);
        const float sin_a = arm_sin_f32(Steer_Current_Angle[i]);
        const float f = f_x * cos_a + f_y * sin_a;

        // 电机输出扭矩 = 驱动力 × 轮半径, 并限幅到单轮最大扭矩
        float tau = f * s;
        if (tau >  tau_max) tau =  tau_max;
        if (tau < -tau_max) tau = -tau_max;
        Chassis_Data.Motor_Torque[i] = tau;
    }
}

void Steer_Chassis_Control::TIM_Calculate_PeriodElapsedCallback()
{

    Chassis_to_Motors();
    Motors_to_Chassis();
    Chassis_to_Motors_Torque();
}
/* Function prototypes -------------------------------------------------------*/

