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
#include "steer_control.h"

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
    PID_Init(&PID_Moving_X,F_max,0, 0.0f, 0.002f, 105.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&PID_Moving_Y,F_max,0, 0.0f, 0.002f, 105.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);

    // 旋转速度环: 最大输出=最大净旋转力矩, 积分限幅同步设为物理上限
    PID_Init(&PID_Spin, M_z_max,0, 0.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);

    // 舵向最小转角策略: 默认正向、满速
    for (int i = 0; i < 4; i++)
    {
        Motor_Dir[i] = 1.0f;
        Motor_Cos_Down[i] = 1.0f;
    }
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

    // 静止判定: 目标速度都很小(例如松开摇杆)时舵向保持当前角度, 防止舵电机回0空耗功率
    // (参照DJI舵轮底盘: 静止时把舵向目标锁在当前角)
    // 小陀螺(Gyring)模式除外: 该模式下底盘持续旋转+平移, 舵向需随运动学合成方向动态更新, 不能锁死
    if (Chassis_Control_Mode != Chassis_Control_Mode_Gyring &&
        fabsf(vx) < STEER_HOLD_VELOCITY_THRESHOLD &&
        fabsf(vy) < STEER_HOLD_VELOCITY_THRESHOLD &&
        fabsf(w)  < STEER_HOLD_VELOCITY_THRESHOLD)
    {
        for (int i = 0; i < 4; i++)
        {
            Chassis_Data.Motor_Target_Omega[i] = 0.0f;
            Steer_Target_Angle[i] = Steer_Current_Angle[i];   // 舵向锚定当前角
        }
        return;
    }

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

        arm_sqrt_f32(wx * wx + wy * wy, &Chassis_Data.Motor_Target_Omega[i]);  // 轮角速度(rad/s)

        // 舵向角 = atan2(Y, X), 输出弧度为 [-pi, pi]
        float steer_angle;
        arm_atan2_f32(wy, wx, &steer_angle);
        // 连续化(unwrap): 跟上一时刻目标角取最短弧差分后累积, 避免 atan2 在 ±π 边界跳变
        // (直接归一化到 [-π,π] 时, 接近 180° 的方向会在 +3.14/-3.14 间来回跳, 导致电机角度环疯转)
        Steer_Target_Angle[i] += Basic_Math_Modulus_Normalization(steer_angle - Steer_Target_Angle[i], STEER_ANGLE_MODULUS);

        // 应用舵向最小转角策略: 反转动力轮方向 + 依据舵向误差削弱目标转速
        Chassis_Data.Motor_Target_Omega[i] *= Motor_Dir[i] * Motor_Cos_Down[i];
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
        // 舵轮反馈角为弧度值, 先归一化到 [-pi, pi] 再用于三角分解
        const float angle = Basic_Math_Modulus_Normalization(Steer_Current_Angle[i], STEER_ANGLE_MODULUS);
        const float vx_w = Chassis_Data.Motor_Now_Omega[i] * s * arm_cos_f32(angle);
        const float vy_w = Chassis_Data.Motor_Now_Omega[i] * s * arm_sin_f32(angle);

        sum_vx += vx_w;
        sum_vy += vy_w;
        sum_m  += -py[i] * vx_w + px[i] * vy_w;   // 该轮速度绕底盘中心的力矩
    }

    // 平动速度 = 各轮速度均值; 旋转速度 = 力矩和 / (4·r²)
    // Chassis_Data.Now_Velocity_X = sum_vx /4.0f;  // 本来是要除以4的
    // Chassis_Data.Now_Velocity_Y = sum_vy / 4.0f;
    // Chassis_Data.Now_Velocity_W = sum_m / (4.0f * r * r);
    Chassis_Data.Now_Velocity_X = sum_vx ;  // 本来是要除以4的
    Chassis_Data.Now_Velocity_Y = sum_vy ;
    Chassis_Data.Now_Velocity_W = sum_m / (r * r);
}

void Steer_Chassis_Control::Chassis_to_Motors_Torque()
{
    Chassis_Data.Target_Torque_Fx = PID_Calculate(&PID_Moving_X, Chassis_Data.Now_Velocity_X, Chassis_Data.Target_Velocity_X);
    Chassis_Data.Target_Torque_Fy = PID_Calculate(&PID_Moving_Y, Chassis_Data.Now_Velocity_Y, Chassis_Data.Target_Velocity_Y);
    Chassis_Data.Target_Torque_Mz = PID_Calculate(&PID_Spin, Chassis_Data.Now_Velocity_W, Chassis_Data.Target_Velocity_W);
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
        // const float f_x = Fx * 0.25f + f_rot * tx[i];
        // const float f_y = Fy * 0.25f + f_rot * ty[i];
        const float f_x = Fx * 1.0f + f_rot * tx[i];
        const float f_y = Fy * 1.0f + f_rot * ty[i];

        // 沿该轮舵向(运动学逆解得到)的有符号驱动力 = 力矢量在舵向的投影
        const float angle = Basic_Math_Modulus_Normalization(Steer_Current_Angle[i], STEER_ANGLE_MODULUS);
        const float cos_a = arm_cos_f32(angle);
        const float sin_a = arm_sin_f32(angle);
        const float f = f_x * cos_a + f_y * sin_a;

        // 电机输出扭矩 = 驱动力 × 轮半径, 依据舵向误差削弱, 再限幅到单轮最大扭矩
        float tau = f * s;
        tau *= Motor_Cos_Down[i];
        if (tau >  tau_max) tau =  tau_max;
        if (tau < -tau_max) tau = -tau_max;
        Chassis_Data.Motor_Target_Torque[i] = tau;
    }
}

void Steer_Chassis_Control::TIM_Calculate_PeriodElapsedCallback(float Motor1_Omega, float Motor2_Omega, float Motor3_Omega, float Motor4_Omega,
    float Steer1_Angle, float Steer2_Angle, float Steer3_Angle, float Steer4_Angle)
{

    // 更新舵轮当前角度和动力轮当前速度
    Steer_Current_Angle[0] = -Steer1_Angle;
    Steer_Current_Angle[1] = -Steer2_Angle;
    Steer_Current_Angle[2] = -Steer3_Angle;
    Steer_Current_Angle[3] = -Steer4_Angle;

    Chassis_Data.Motor_Now_Omega[0] = Motor1_Omega;
    Chassis_Data.Motor_Now_Omega[1] = Motor2_Omega;
    Chassis_Data.Motor_Now_Omega[2] = Motor3_Omega;
    Chassis_Data.Motor_Now_Omega[3] = Motor4_Omega;
    
    Chassis_to_Motors();
    Motors_to_Chassis();
    Chassis_to_Motors_Torque();

    Cal_Angle_Dir(0);
    Cal_Angle_Dir(1);
    Cal_Angle_Dir(2);
    Cal_Angle_Dir(3);

}

/**
 * @brief 计算舵轮旋转的最小角，和是否需要反转动力轮旋转方向，并且可在舵轮角度没到位前减少动力轮速度
 * 
 * @param Steer_Num 
 */
void Steer_Chassis_Control::Cal_Angle_Dir(uint8_t Steer_Num)
{
    float err,angle;
    err = Steer_Target_Angle[Steer_Num] - Steer_Current_Angle[Steer_Num];
    // 归一化到 [-pi, pi]，实现转劣弧
    err = Basic_Math_Modulus_Normalization(err, STEER_ANGLE_MODULUS);

    //转最优角, 90°边界留容差: 恰好90°时固定走不反转分支, 避免浮点噪声让四个轮方向不统一
    if(err > STEER_ANGLE_HALF_PI)
    {
        err -= STEER_ANGLE_PI;
        Motor_Dir[Steer_Num] = -1.0f;
    }
    else if(err < -STEER_ANGLE_HALF_PI)
    {
        err += STEER_ANGLE_PI;
        Motor_Dir[Steer_Num] = -1.0f;
    }
    else
    {
        Motor_Dir[Steer_Num] = 1.0f;
    }

    angle = err;
    if (fabs(angle) > STEER_ANGLE_HALF_PI)
      angle = STEER_ANGLE_HALF_PI;
    else if (fabs(angle) < STEER_ANGLE_DEADZONE && Chassis_Control_Mode != Chassis_Control_Mode_Gyring)
      angle = 0.0f;

    // 送电机的有效目标角 = 当前角 + 最优小角(不折叠), 供 Get_Steer_Target_Angle() 输出;
    // 而 Steer_Target_Angle 保持为 Chassis_to_Motors 计算的连续运动学目标角, 不再被覆盖,
    // 避免最优角(反转)判断因目标角漂移而误转优弧
    Steer_Effective_Angle[Steer_Num] = Steer_Current_Angle[Steer_Num] + angle;

    const float cos_angle = arm_cos_f32(angle);
    Motor_Cos_Down[Steer_Num] = cos_angle*cos_angle*cos_angle;

}
/* Function prototypes -------------------------------------------------------*/

