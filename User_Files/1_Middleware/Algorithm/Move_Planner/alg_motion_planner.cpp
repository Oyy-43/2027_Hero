/**
 * @file alg_motion_planner.cpp
 * @author Oyyp
 * @brief 一维运动规划器, 把"阶跃的位置目标"规划成时间最优(梯形速度曲线)的参考轨迹
 * @version 0.1
 * @date 2026-11-08 0.1 初版
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_motion_planner.h"

#include <math.h>

/* Private macros ------------------------------------------------------------*/

// 加速度小于该比例(相对 Max_Acceleration)时认为处于匀速段, 仅用于状态观测
#define MOVE_PLANNER_STATUS_ACCELERATION_EPSILON 0.02f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化
 *
 * @param __Max_Velocity 速度上限
 * @param __Max_Acceleration 加速度上限
 * @param __Max_Deceleration 减速度上限, 一般可以给得比加速度大一些, 便于及时收住
 * @param __Period 计算周期, 单位秒, 必须与实际调用间隔一致
 * @param __Position_Tolerance 到位位置死区, 小于该值即认为到位
 * @param __Velocity_Tolerance 到位速度死区, 同时满足才认为到位, 用于抑制末端抖动
 * @param __Wrap_Range 周期折叠范围, 0 表示线性轴不折叠, 2*pi 表示转一圈回到原点
 */
void Class_Move_Planner::Init(const float &__Max_Velocity,
                              const float &__Max_Acceleration,
                              const float &__Max_Deceleration,
                              const float &__Period,
                              const float &__Position_Tolerance,
                              const float &__Velocity_Tolerance,
                              const float &__Wrap_Range)
{
    // 参数兜底, 任何一项非法都会让规划器静默失效甚至输出无穷大, 这里统一修正成安全值
    Max_Velocity = (__Max_Velocity > 0.0f) ? __Max_Velocity : 1.0f;
    Max_Acceleration = (__Max_Acceleration > 0.0f) ? __Max_Acceleration : 1.0f;
    Max_Deceleration = (__Max_Deceleration > 0.0f) ? __Max_Deceleration : Max_Acceleration;
    Period = (__Period > 0.0f) ? __Period : 0.002f;
    Position_Tolerance = (__Position_Tolerance > 0.0f) ? __Position_Tolerance : 0.0f;
    Velocity_Tolerance = (__Velocity_Tolerance > 0.0f) ? __Velocity_Tolerance : 0.0f;
    Wrap_Range = (__Wrap_Range > 0.0f) ? __Wrap_Range : 0.0f;

    Ref_Position = 0.0f;
    Ref_Velocity = 0.0f;
    Ref_Acceleration = 0.0f;
    Target_Position = 0.0f;
    Remain_Distance = 0.0f;
    Status = Move_Planner_Status_IDLE;
}

/**
 * @brief 设定目标位置, 运动中随时可以改, 下一周期自动重规划
 *
 * @param __Target_Position 目标位置
 */
void Class_Move_Planner::Set_Target_Position(const float &__Target_Position)
{
    Target_Position = __Target_Position;
}

/**
 * @brief 重新对齐参考位置到实际位置, 速度保留
 * 用于长时间运行后消除参考轨迹与实际位置的累计偏差
 *
 * @param __Now_Position 实际位置
 */
void Class_Move_Planner::Set_Now_Position(const float &__Now_Position)
{
    Ref_Position = __Now_Position;
}

/**
 * @brief 复位, 参考位置对齐到实际位置并清空目标与速度
 * 上电, 使能, 切换模式时必须调用一次
 *
 * @param __Now_Position 实际位置
 */
void Class_Move_Planner::Reset(const float &__Now_Position)
{
    Ref_Position = __Now_Position;
    Ref_Velocity = 0.0f;
    Ref_Acceleration = 0.0f;
    Target_Position = __Now_Position;
    Remain_Distance = 0.0f;
    Status = Move_Planner_Status_IDLE;
}

/**
 * @brief 规划计算, 计算周期取决于调用者, 必须与 Init 中的 Period 一致
 *
 */
void Class_Move_Planner::TIM_Calculate_PeriodElapsedCallback()
{
    // 1. 剩余行程, 带符号, 正负号就是运动方向
    const float remain_before = Normalize_Distance(Target_Position - Ref_Position);
    const float abs_remain = fabsf(remain_before);
    const float direction = (remain_before >= 0.0f) ? 1.0f : -1.0f;

    // 2. 位置和速度都进死区, 认为到位, 直接贴到目标并停下
    if (abs_remain <= Position_Tolerance && fabsf(Ref_Velocity) <= Velocity_Tolerance)
    {
        Snap_To_Target();
        return;
    }

    // 3. 本周期允许的速度上限
    //    v_brake = sqrt(2 * a_dec * s) 是"以当前行程恰好能刹停"的速度;
    //    这里预扣一个周期的位移, 抵消离散化带来的过冲
    const float abs_velocity = fabsf(Ref_Velocity);
    const float brake_distance = (abs_remain > abs_velocity * Period) ? (abs_remain - abs_velocity * Period) : 0.0f;
    const float velocity_brake = sqrtf(2.0f * Max_Deceleration * brake_distance);
    const float velocity_limit = (velocity_brake < Max_Velocity) ? velocity_brake : Max_Velocity;

    // 4. 把当前速度投影到运动方向, 反向运动时该值为负
    const float velocity_error = velocity_limit - direction * Ref_Velocity;

    // 5. 把"本周期要补上的速度差"折算成加速度并限幅
    //    加速/匀速/减速/反向制动 四种情况都由这一条式子统一覆盖, 不需要状态机分支
    float acceleration = velocity_error / Period;
    if (acceleration > Max_Acceleration)
    {
        acceleration = Max_Acceleration;
    }
    if (acceleration < -Max_Deceleration)
    {
        acceleration = -Max_Deceleration;
    }

    // 6. 梯形积分更新参考轨迹, 比一阶欧拉更贴合真实速度曲线
    const float velocity_last = Ref_Velocity;
    Ref_Velocity = velocity_last + acceleration * Period;
    Ref_Position = Ref_Position + (velocity_last + Ref_Velocity) * 0.5f * Period;
    Ref_Acceleration = acceleration;

    // 7. 防过冲, 越过目标点则直接贴到目标并清零速度
    Remain_Distance = Normalize_Distance(Target_Position - Ref_Position);
    if ((Remain_Distance > 0.0f) != (remain_before > 0.0f))
    {
        Snap_To_Target();
        return;
    }

    Update_Status(acceleration);
}

/**
 * @brief 求取带符号距离, 启用周期折叠时折叠到 ±Wrap_Range/2 之间, 保证走最短方向
 *
 * @param __Distance 原始带符号距离
 * @return 折叠后的带符号距离
 */
float Class_Move_Planner::Normalize_Distance(const float &__Distance) const
{
    if (Wrap_Range <= 0.0f)
    {
        // 线性轴, 不折叠
        return (__Distance);
    }

    // 折叠到 [-Wrap_Range/2, Wrap_Range/2)
    float tmp = fmodf(__Distance + Wrap_Range * 0.5f, Wrap_Range);

    if (tmp < 0.0f)
    {
        tmp += Wrap_Range;
    }

    return (tmp - Wrap_Range * 0.5f);
}

/**
 * @brief 参考轨迹贴到目标点并清零速度, 进入空闲状态
 *
 */
void Class_Move_Planner::Snap_To_Target()
{
    // 沿最短方向贴过去, 而不是直接写 Target_Position,
    // 这样在周期折叠场合参考轨迹仍然连续, 不会突然跳一个整周期
    Ref_Position = Ref_Position + Normalize_Distance(Target_Position - Ref_Position);
    Ref_Velocity = 0.0f;
    Ref_Acceleration = 0.0f;
    Remain_Distance = 0.0f;
    Status = Move_Planner_Status_IDLE;
}

/**
 * @brief 根据加速度与速度是否同号区分加速/减速, 仅用于观测
 *
 * @param __Acceleration 本周期加速度
 */
void Class_Move_Planner::Update_Status(const float &__Acceleration)
{
    if (fabsf(__Acceleration) <= MOVE_PLANNER_STATUS_ACCELERATION_EPSILON * Max_Acceleration)
    {
        // 加速度基本为零, 速度维持不变
        Status = Move_Planner_Status_CRUISE;
    }
    else if (__Acceleration * Ref_Velocity >= 0.0f)
    {
        // 加速度与速度同号, 速度绝对值在增大
        Status = Move_Planner_Status_ACCELERATE;
    }
    else
    {
        // 加速度与速度反号, 速度绝对值在减小
        Status = Move_Planner_Status_DECELERATE;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
