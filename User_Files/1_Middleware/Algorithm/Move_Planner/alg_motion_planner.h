/**
 * @file alg_motion_planner.h
 * @author Oyyp
 * @brief 一维运动规划器, 把"阶跃的位置目标"规划成时间最优(梯形速度曲线)的参考轨迹
 * @version 0.1
 * @date 2026-11-08 0.1 初版
 *
 * @copyright Copyright
 *
 * 设计原则(低耦合):
 *  1. 只依赖 <math.h>, 不引用电机/驱动/PID/CAN/任何硬件头文件, 可以脱离工程单独单元测试;
 *  2. 只做一件事 —— 生成"位置连续、速度连续、加速度有界"的参考轨迹, 不关心执行器怎么实现;
 *  3. 对外只暴露 float 标量和一个输出快照, 上层要不要用前馈、前馈加到哪一级环路, 由上层决定.
 *
 * 算法说明(每周期在线重规划, 因此目标点随时可以改, 不需要 stop 再 start):
 *  - 剩余行程 s 按最短方向计算(支持 ±pi 周期折叠);
 *  - 刹车速度 v_brake = sqrt(2 * a_dec * s), 表示"以当前行程还能刹停"的速度上限;
 *  - 本周期速度上限 v_limit = min(Max_Velocity, v_brake);
 *  - 加速度指令 a = clamp((v_limit - v) / T, -Max_Deceleration, +Max_Acceleration).
 *    这一条式子同时覆盖了 加速 / 匀速 / 减速 / 反向制动 四种情况, 不需要写状态机分支.
 *  - 用梯形积分更新位置, 比一阶欧拉更贴合真实速度曲线, 减少离散误差.
 *
 * 推荐接法(与现有 "角度环 -> 速度环 -> 力矩" 结构兼容):
 * @code
 *  // 初始化: 最大速度 15 rad/s, 加/减速 60 rad/s^2, 控制周期 2ms, 到位死区 0.005rad/0.05rad/s
 *  //         周期折叠范围取 2 * 电机角度幅值, 不折叠则填 0
 *  planner.Init(15.0f, 60.0f, 60.0f, 0.002f, 0.005f, 0.05f, 2.0f * Motor.Get_Angle_Max());
 *  planner.Reset(Motor.Get_Now_Angle());          // 上电/使能时对齐
 *  planner.Set_Target_Position(target_angle);     // 任何时候都可以改目标
 *
 *  // 每 2ms 调用一次
 *  planner.TIM_Calculate_PeriodElapsedCallback();
 *  Struct_Move_Planner_Output ref = planner.Get_Output();
 *
 *  // 用 "速度前馈 + 位置误差修正" 取代原来的纯 P 位置环, 消掉"先快后慢"
 *  speed_target = ref.Velocity + Kp_Position * (ref.Position - Motor.Get_Now_Angle());
 *
 *  // 力矩前馈: 转动惯量 * 规划加速度, 解决加速段后劲不足
 *  torque_ff = J * ref.Acceleration + Constant_Feedforward_Torque;
 * @endcode
 *
 * 若不想改动现有 PID_Angle, 也可以只把 Get_Ref_Velocity() 加到速度环目标上做前馈,
 * 规划器本身不需要任何修改.
 */

#ifndef ALG_MOTION_PLANNER_H
#define ALG_MOTION_PLANNER_H

/* Includes ------------------------------------------------------------------*/

// 无外部依赖, 只使用 float 标量

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 规划器状态, 仅用于观测和调试, 不参与控制决策
 *
 */
enum Enum_Move_Planner_Status
{
    Move_Planner_Status_IDLE = 0,    // 空闲, 已停在目标点
    Move_Planner_Status_ACCELERATE,  // 加速段, 速度绝对值在增大
    Move_Planner_Status_CRUISE,      // 匀速段, 速度维持在 Max_Velocity
    Move_Planner_Status_DECELERATE,  // 减速段, 速度绝对值在减小
};

/**
 * @brief 规划器输出快照, 一帧取全, 便于上层同时使用位置/速度/加速度
 *
 */
struct Struct_Move_Planner_Output
{
    // 参考位置, 上层位置环的跟踪目标
    float Position;
    // 参考速度, 上层速度环的前馈速度
    float Velocity;
    // 参考加速度, 上层力矩前馈用(J * Acceleration)
    float Acceleration;
};

/**
 * @brief Reusable, 一维运动规划器, 时间最优(梯形速度曲线)
 * 无零点, 无单位, 单位为 rad / rad/s / rad/s^2 或 m / m/s / m/s^2, 由调用者统一
 *
 */
class Class_Move_Planner
{
public:
    void Init(const float &__Max_Velocity,
              const float &__Max_Acceleration,
              const float &__Max_Deceleration,
              const float &__Period,
              const float &__Position_Tolerance,
              const float &__Velocity_Tolerance,
              const float &__Wrap_Range = 0.0f);

    void Set_Target_Position(const float &__Target_Position);

    void Set_Now_Position(const float &__Now_Position);

    void Reset(const float &__Now_Position);

    void TIM_Calculate_PeriodElapsedCallback();

    inline Struct_Move_Planner_Output Get_Output() const;

    inline float Get_Ref_Position() const;

    inline float Get_Ref_Velocity() const;

    inline float Get_Ref_Acceleration() const;

    inline float Get_Target_Position() const;

    inline float Get_Remain_Distance() const;

    inline Enum_Move_Planner_Status Get_Status() const;

    inline bool Is_Finished() const;

    inline void Set_Max_Velocity(const float &__Max_Velocity);

    inline void Set_Max_Acceleration(const float &__Max_Acceleration);

    inline void Set_Max_Deceleration(const float &__Max_Deceleration);

protected:
    // 初始化相关变量

    // 输出速度上限
    float Max_Velocity = 0.0f;
    // 输出加速度上限, 单位/秒^2
    float Max_Acceleration = 0.0f;
    // 输出减速度上限, 单位/秒^2
    float Max_Deceleration = 0.0f;
    // 计算周期, 单位秒, 由调用者保证与实际调用频率一致
    float Period = 0.002f;
    // 到位位置死区
    float Position_Tolerance = 0.0f;
    // 到位速度死区
    float Velocity_Tolerance = 0.0f;
    // 周期折叠范围, 0 表示线性轴不折叠, 2*pi 表示转一圈回到原点
    float Wrap_Range = 0.0f;

    // 内部变量

    // 参考位置
    float Ref_Position = 0.0f;
    // 参考速度
    float Ref_Velocity = 0.0f;
    // 参考加速度
    float Ref_Acceleration = 0.0f;
    // 目标位置, 由 Set_Target_Position 设定
    float Target_Position = 0.0f;
    // 带符号剩余行程, 正负号表示运动方向
    float Remain_Distance = 0.0f;

    // 读变量

    // 规划器状态
    Enum_Move_Planner_Status Status = Move_Planner_Status_IDLE;

    // 内部函数

    float Normalize_Distance(const float &__Distance) const;

    void Snap_To_Target();

    void Update_Status(const float &__Acceleration);
};

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出快照
 *
 * @return 参考位置/速度/加速度
 */
inline Struct_Move_Planner_Output Class_Move_Planner::Get_Output() const
{
    Struct_Move_Planner_Output output;

    output.Position = Ref_Position;
    output.Velocity = Ref_Velocity;
    output.Acceleration = Ref_Acceleration;

    return (output);
}

/**
 * @brief 获取参考位置
 *
 * @return 参考位置
 */
inline float Class_Move_Planner::Get_Ref_Position() const
{
    return (Ref_Position);
}

/**
 * @brief 获取参考速度
 *
 * @return 参考速度
 */
inline float Class_Move_Planner::Get_Ref_Velocity() const
{
    return (Ref_Velocity);
}

/**
 * @brief 获取参考加速度
 *
 * @return 参考加速度
 */
inline float Class_Move_Planner::Get_Ref_Acceleration() const
{
    return (Ref_Acceleration);
}

/**
 * @brief 获取目标位置
 *
 * @return 目标位置
 */
inline float Class_Move_Planner::Get_Target_Position() const
{
    return (Target_Position);
}

/**
 * @brief 获取带符号剩余行程
 *
 * @return 带符号剩余行程, 正负号表示运动方向
 */
inline float Class_Move_Planner::Get_Remain_Distance() const
{
    return (Remain_Distance);
}

/**
 * @brief 获取规划器状态, 仅用于观测
 *
 * @return 规划器状态
 */
inline Enum_Move_Planner_Status Class_Move_Planner::Get_Status() const
{
    return (Status);
}

/**
 * @brief 是否已到位
 *
 * @return 是否已到位
 */
inline bool Class_Move_Planner::Is_Finished() const
{
    return (Status == Move_Planner_Status_IDLE);
}

/**
 * @brief 在线修改速度上限
 *
 * @param __Max_Velocity 速度上限
 */
inline void Class_Move_Planner::Set_Max_Velocity(const float &__Max_Velocity)
{
    if (__Max_Velocity > 0.0f)
    {
        Max_Velocity = __Max_Velocity;
    }
}

/**
 * @brief 在线修改加速度上限
 *
 * @param __Max_Acceleration 加速度上限
 */
inline void Class_Move_Planner::Set_Max_Acceleration(const float &__Max_Acceleration)
{
    if (__Max_Acceleration > 0.0f)
    {
        Max_Acceleration = __Max_Acceleration;
    }
}

/**
 * @brief 在线修改减速度上限
 *
 * @param __Max_Deceleration 减速度上限
 */
inline void Class_Move_Planner::Set_Max_Deceleration(const float &__Max_Deceleration)
{
    if (__Max_Deceleration > 0.0f)
    {
        Max_Deceleration = __Max_Deceleration;
    }
}

#endif /* ALG_MOTION_PLANNER_H */
