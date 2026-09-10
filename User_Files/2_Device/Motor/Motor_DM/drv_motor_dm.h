#ifndef __DRV_MOTOR_DM_H__
#define __DRV_MOTOR_DM_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Algorithm/PID/alg_pid.h"
#include "1_Middleware/Algorithm/Basic/alg_basic.h"
#include "1_Middleware/Algorithm/Filter/Frequency/alg_filter_frequency.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 达妙电机状态
 *
 */
enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
};

/**
 * @brief 达妙电机的ID枚举类型, 一拖四模式用
 *
 */
enum Enum_Motor_DM_Motor_ID_1_To_4 : uint8_t
{
    Motor_DM_ID_0x201 = 1,
    Motor_DM_ID_0x202,
    Motor_DM_ID_0x203,
    Motor_DM_ID_0x204,
    Motor_DM_ID_0x205,
    Motor_DM_ID_0x206,
    Motor_DM_ID_0x207,
    Motor_DM_ID_0x208,
};

/**
 * @brief 达妙电机控制状态, 传统模式有效
 *
 */
enum Enum_Motor_DM_Control_Status_Normal
{
    Motor_DM_Control_Status_DISABLE = 0x0,                  //未使能
    Motor_DM_Control_Status_ENABLE,                         //已使能
    Motor_DM_Control_Status_OVERVOLTAGE = 0x8,              //过压
    Motor_DM_Control_Status_UNDERVOLTAGE,                   //欠压
    Motor_DM_Control_Status_OVERCURRENT,                    //过流
    Motor_DM_Control_Status_MOS_OVERTEMPERATURE,            //MOS过温（电驱温度过高）
    Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE,          //转子过温（电机温度过高）
    Motor_DM_Control_Status_LOSE_CONNECTION,                //失联（电机未连接）
    Motor_DM_Control_Status_MOS_OVERLOAD,                   //MOS过载（电驱过载）
};

/**
 * @brief 达妙电机控制方式
 *
 */
enum Enum_Motor_DM_Control_Method
{
    Motor_DM_Control_Method_NORMAL_MIT = 0,
    Motor_DM_Control_Method_NORMAL_MIT_Position,
    Motor_DM_Control_Method_NORMAL_MIT_Omega,
    Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
    Motor_DM_Control_Method_NORMAL_OMEGA,
    Motor_DM_Control_Method_NORMAL_EMIT,
    Motor_DM_Control_Method_1_TO_4_CURRENT,
    Motor_DM_Control_Method_1_TO_4_OMEGA,
    Motor_DM_Control_Method_1_TO_4_ANGLE,
};

/**
 * @brief 达妙电机传统模式源数据
 *
 */
struct Struct_Motor_DM_CAN_Rx_Data_Normal
{
    uint8_t CAN_ID : 4;
    uint8_t Control_Status_Enum : 4;
    uint16_t Angle_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机一拖四模式源数据
 *
 */
struct Struct_Motor_DM_CAN_Rx_Data_1_To_4
{
    uint16_t Encoder_Reverse;
    // 角速度，单位为rpm
    int16_t Omega_Reverse;
    // 电流映射值，为[-16384,16384]，映射范围[-20.5,20.5]A
    int16_t Current_Reverse;
    //电机线圈温度
    uint8_t Rotor_Temperature;
    //错误码
    uint8_t Error_Code;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, MIT控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_MIT
{
    uint16_t Control_Angle_Reverse;
    uint8_t Control_Omega_11_4;
    uint8_t Control_Omega_3_0_K_P_11_8;
    uint8_t K_P_7_0;
    uint8_t K_D_11_4;
    uint8_t K_D_3_0_Control_Torque_11_8;
    uint8_t Control_Torque_7_0;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 位置速度控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega
{
    float Control_Angle;
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 速度控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Omega
{
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, EMIT控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT
{
    float Control_Angle;
    // 限定速度用, rad/s的100倍
    uint16_t Control_Omega;
    // 限定电流用, 电流最大值的10000倍
    uint16_t Control_Current;
} __attribute__((packed));

/**
 * @brief 达妙电机经过处理的数据, 传统模式有效
 *
 */
struct Struct_Motor_DM_Rx_Data_Normal
{
    Enum_Motor_DM_Control_Status_Normal Control_Status;
    float Now_Angle;
    float Filtered_Now_Angle;
    float Now_Omega;
    float Filtered_Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
};

/**
 * @brief 达妙电机经过处理的数据, 一拖四模式有效
 *
 */
struct Struct_Motor_DM_Rx_Data_1_To_4
{
    float Now_Angle;
    float Filtered_Now_Angle;
    float Now_Omega;
    float Filtered_Now_Omega;
    float Now_Torque;
    float Now_Rotor_Temperature;
    float Now_Error_Code;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
 * @brief Reusable, 达妙电机, 传统模式
 * 没有零点, 可在上位机调零点
 * 初始化的角度, 角速度, 扭矩等参数是J4310电机默认值
 *
 */
class Class_Motor_DM_Normal
{
public:
    // PID角度环控制
    PID_TypeDef PID_Angle;
    // PID角速度环控制
    PID_TypeDef PID_Omega;

    // 滤波器
    Class_Filter_Frequency<> Filter_Angle;

    Class_Filter_Frequency<10> Filter_Omega;

    void Init(const FDCAN_HandleTypeDef *hcan, const uint8_t &__CAN_Rx_ID = 0x00, const uint8_t &__CAN_Tx_ID = 0x01, const Enum_Motor_DM_Control_Method &__Motor_DM_Control_Method = Motor_DM_Control_Method_NORMAL_MIT, const float &__Angle_Max = 12.5f, const float &__Omega_Max = 25.0f, const float &__Torque_Max = 10.0f, const float &__Current_Max = 10.261194f);

    inline float Get_Angle_Max() const;

    inline float Get_Omega_Max() const;

    inline float Get_Torque_Max() const;

    inline float Get_Current_Max() const;

    inline Enum_Motor_DM_Status Get_Status() const;

    inline Enum_Motor_DM_Control_Status_Normal Get_Control_Status() const;

    inline float Get_Now_Angle() const;

    inline float Get_Now_Omega() const;

    inline float Get_Now_Torque() const;

    inline float Get_Now_MOS_Temperature() const;

    inline float Get_Now_Rotor_Temperature() const;

    inline Enum_Motor_DM_Control_Method Get_Control_Method() const;

    inline float Get_Control_Angle() const;

    inline float Get_Control_Omega() const;

    inline float Get_Control_Torque() const;

    inline float Get_Control_Current() const;

    inline float Get_K_P() const;

    inline float Get_K_D() const;

    inline void Set_Control_Angle(const float &__Control_Angle);

    inline void Set_Target_Angle(const float &__Target_Angle);

    inline void Set_Control_Omega(const float &__Control_Omega);

    inline void Set_Target_Omega(const float &__Target_Omega);

    inline void Set_Control_Torque(const float &__Control_Torque);

    inline void Set_Control_Current(const float &__Control_Current);

    inline void Set_K_P(const float &__K_P);

    inline void Set_K_D(const float &__K_D);

    void CAN_RxCpltCallback();

    void CAN_Send_Clear_Error() const;

    void CAN_Send_Enter() const;

    void CAN_Send_Exit() const;

    void CAN_Send_Save_Zero() const;

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Send_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t CAN_Rx_ID;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t CAN_Tx_ID;
    // 最大位置, 与上位机控制幅值PMAX保持一致
    float Angle_Max;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float Omega_Max;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float Torque_Max;
    // 最大电流, 与上位机串口中上电打印电流保持一致, EMIT模式需要
    float Current_Max;

    // 常量

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    // 发送缓冲区
    uint8_t Tx_Data[8];

    // 读变量

    // 电机状态
    Enum_Motor_DM_Status Motor_DM_Status = Motor_DM_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_DM_Rx_Data_Normal Rx_Data;

    // 写变量

    // 读写变量

    // 电机控制方式
    Enum_Motor_DM_Control_Method Motor_DM_Control_Method = Motor_DM_Control_Method_NORMAL_MIT;

    // 角度, rad, 目标角度
    float Control_Angle = 0.0f;
    // mit下pid控制的目标角度,单位rad
    float Target_Angle = 0.0f;
    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
    float Control_Omega = 0.0f;
    // 角速度, rad/s, mit下pid控制的目标角速度
    float Target_Omega = 0.0f;
    // 前馈角速度，rad/s, MIT的pid输出速度模式下使用
    float Feedforward_Omega = 0.0f;
    // 前馈扭矩，Nm, MIT模式下使用
    float Feedforward_Torque = 0.0f;
    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
    float Control_Torque = 0.0f;
    // 电流, A, EMIT模式是限幅, 其余模式无效
    float Control_Current = 0.0f;
    // K_P, 0~500, MIT模式有效
    float K_P = 0.0f;
    // K_D, 0~5, MIT模式有效
    float K_D = 0.0f;

    // 内部函数

    void Data_Process();

    void Output();
};

/**
 * @brief Reusable, 达妙电机, 一拖四模式
 * 初始化的角度, 角速度, 扭矩等参数是J4310电机默认值
 */
class Class_Motor_DM_1_To_4
{
public:
    // PID角度环控制
    PID_TypeDef PID_Angle;
    // PID角速度环控制
    PID_TypeDef PID_Omega;

    Class_Filter_Frequency<> Filter_Angle;

    Class_Filter_Frequency<> Filter_Omega;

    void Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DM_Motor_ID_1_To_4 &__CAN_Rx_ID, const Enum_Motor_DM_Control_Method &__Motor_DM_Control_Method = Motor_DM_Control_Method_1_TO_4_ANGLE, const int32_t &__Encoder_Offset = 0, const float &__Nearest_Angle = 0.0f, const float &__Gearbox_Rate = 10.0f);

    inline Enum_Motor_DM_Status Get_Status() const;

    inline float Get_Now_Angle() const;

    inline float Get_Now_Omega() const;

    inline float Get_Now_Torque() const;

    inline float Get_Now_Rotor_Temperature() const;

    inline float Get_Now_Error_Code() const;

    inline Enum_Motor_DM_Control_Method Get_Control_Method() const;

    inline float Get_Target_Angle() const;

    inline float Get_Target_Omega() const;

    inline float Get_Target_Torque() const;

    inline float Get_Feedforward_Omega() const;

    inline float Get_Feedforward_Torque() const;

    inline void Set_Control_Method(const Enum_Motor_DM_Control_Method &__DM_Motor_Control_Method);

    inline void Set_Target_Angle(const float &__Target_Angle);

    inline void Set_Target_Omega(const float &__Target_Omega);

    inline void Set_Target_Torque(const float &__Target_Torque);

    inline void Set_Feedforward_Omega(const float &__Feedforward_Omega);

    inline void Set_Feedforward_Torque(const float &__Feedforward_Torque);

    void CAN_RxCpltCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 达妙系列0x301~0x308
    Enum_Motor_DM_Motor_ID_1_To_4 CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 编码器偏移
    int32_t Encoder_Offset;
    // 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
    float Nearest_Angle;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;

    // 常量

    // 一圈编码器刻度
    uint16_t ENCODER_NUM_PER_ROUND = 8192;

    // 扭矩电流常数, 由于电机手册没给, 则以额定扭矩除以额定电流计算
    const float CURRENT_TO_TORQUE = 1.2f / 10.0f;
    // 扭矩电流到输出刻度的转化系数
    const float CURRENT_TO_OUT = 16384.0f / 10.261194f;
    // 最大输出刻度
    const float OUT_MAX = 16384.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;

    // 读变量

    // 电机状态
    Enum_Motor_DM_Status Motor_DM_Status = Motor_DM_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_DM_Rx_Data_1_To_4 Rx_Data;

    // 写变量

    // 读写变量

    // 电机控制方式
    Enum_Motor_DM_Control_Method Motor_DM_Control_Method = Motor_DM_Control_Method_1_TO_4_ANGLE;
    // 目标的角度
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的扭矩, Nm
    float Target_Torque = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的扭矩, Nm
    float Feedforward_Torque = 0.0f;

    // 内部函数

    void Data_Process();

    void PID_Cal();

    void Output();
};
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
/**
 * @brief 获取角度最大值
 *
 * @return float 角度最大值
 */
inline float Class_Motor_DM_Normal::Get_Angle_Max() const
{
    return (Angle_Max);
}

/**
 * @brief 获取角速度最大值
 *
 * @return float 角速度最大值
 */
inline float Class_Motor_DM_Normal::Get_Omega_Max() const
{
    return (Omega_Max);
}

/**
 * @brief 获取扭矩最大值
 *
 * @return float 扭矩最大值
 */
inline float Class_Motor_DM_Normal::Get_Torque_Max() const
{
    return (Torque_Max);
}

/**
 * @brief 获取电流最大值
 *
 * @return float 电流最大值
 */
inline float Class_Motor_DM_Normal::Get_Current_Max() const
{
    return (Current_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DM_Status 电机状态
 */
inline Enum_Motor_DM_Status Class_Motor_DM_Normal::Get_Status() const
{
    return (Motor_DM_Status);
}

/**
 * @brief 获取电机控制状态
 *
 * @return Enum_Motor_DM_Control_Status_Normal 电机控制状态
 */
inline Enum_Motor_DM_Control_Status_Normal Class_Motor_DM_Normal::Get_Control_Status() const
{
    return (Rx_Data.Control_Status);
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度
 */
inline float Class_Motor_DM_Normal::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Motor_DM_Normal::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前扭矩
 *
 * @return float 当前扭矩
 */
inline float Class_Motor_DM_Normal::Get_Now_Torque() const
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取当前MOS温度
 *
 * @return float 当前MOS温度
 */
inline float Class_Motor_DM_Normal::Get_Now_MOS_Temperature() const
{
    return (Rx_Data.Now_MOS_Temperature);
}

/**
 * @brief 获取当前转子温度
 *
 * @return float 当前转子温度
 */
inline float Class_Motor_DM_Normal::Get_Now_Rotor_Temperature() const
{
    return (Rx_Data.Now_Rotor_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_DM_Control_Method 电机控制方式
 */
inline Enum_Motor_DM_Control_Method Class_Motor_DM_Normal::Get_Control_Method() const
{
    return (Motor_DM_Control_Method);
}

/**
 * @brief 获取角度, rad, 目标角度
 *
 * @return float 角度, rad, 目标角度
 */
inline float Class_Motor_DM_Normal::Get_Control_Angle() const
{
    return (Control_Angle);
}

/**
 * @brief 获取角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @return float 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline float Class_Motor_DM_Normal::Get_Control_Omega() const
{
    return (Control_Omega);
}

/**
 * @brief 获取扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @return float 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline float Class_Motor_DM_Normal::Get_Control_Torque() const
{
    return (Control_Torque);
}

/**
 * @brief 获取电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @return float 电流, A, EMIT模式是限幅, 其余模式无效
 */
inline float Class_Motor_DM_Normal::Get_Control_Current() const
{
    return (Control_Current);
}

/**
 * @brief 获取K_P, 0~500, MIT模式有效
 *
 * @return float K_P, 0~500, MIT模式有效
 */
inline float Class_Motor_DM_Normal::Get_K_P() const
{
    return (K_P);
}

/**
 * @brief 获取K_D, 0~5, MIT模式有效
 *
 * @return float K_D, 0~5, MIT模式有效
 */
inline float Class_Motor_DM_Normal::Get_K_D() const
{
    return (K_D);
}

/**
 * @brief 设定角度, rad, 目标角度
 *
 * @param __Control_Angle 角度, rad, 目标角度
 */
inline void Class_Motor_DM_Normal::Set_Control_Angle(const float &__Control_Angle)
{
    Control_Angle = __Control_Angle;
}

/**
 * @brief 设定目标角度,这个目标角度不会用于mit的公式，而是pid计算输出扭矩
 * 
 * @param __Target_Angle 
 */
inline void Class_Motor_DM_Normal::Set_Target_Angle(const float &__Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @param __Control_Omega 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline void Class_Motor_DM_Normal::Set_Control_Omega(const float &__Control_Omega)
{
    Control_Omega = __Control_Omega;
}

/**
 * @brief 设定目标角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @param __Target_Omega 目标角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline void Class_Motor_DM_Normal::Set_Target_Omega(const float &__Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @param __Control_Torque 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline void Class_Motor_DM_Normal::Set_Control_Torque(const float &__Control_Torque)
{
    Control_Torque = __Control_Torque;
}

/**
 * @brief 设定电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @param __Control_Current 电流, A, EMIT模式是限幅, 其余模式无效
 */
inline void Class_Motor_DM_Normal::Set_Control_Current(const float &__Control_Current)
{
    Control_Current = __Control_Current;
}

/**
 * @brief 设定K_P, 0~500, MIT模式有效
 *
 * @param __K_P K_P, 0~500, MIT模式有效
 */
inline void Class_Motor_DM_Normal::Set_K_P(const float &__K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设定K_D, 0~5, MIT模式有效
 *
 * @param __K_D K_D, 0~5, MIT模式有效
 */
inline void Class_Motor_DM_Normal::Set_K_D(const float &__K_D)
{
    K_D = __K_D;
}

//======================================================================================================================//
/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DM_Status 电机状态
 */
inline Enum_Motor_DM_Status Class_Motor_DM_1_To_4::Get_Status() const
{
    return (Motor_DM_Status);
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度
 */
inline float Class_Motor_DM_1_To_4::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Motor_DM_1_To_4::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前扭矩
 *
 * @return float 当前扭矩
 */
inline float Class_Motor_DM_1_To_4::Get_Now_Torque() const
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取当前MOS温度
 *
 * @return float 当前MOS温度
 */
inline float Class_Motor_DM_1_To_4::Get_Now_Rotor_Temperature() const
{
    return (Rx_Data.Now_Rotor_Temperature);
}

/**
 * @brief 获取当前错误码
 *
 * @return float 当前错误码
 */
inline float Class_Motor_DM_1_To_4::Get_Now_Error_Code() const
{
    return (Rx_Data.Now_Error_Code);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_DM_Control_Method 电机控制方式
 */
inline Enum_Motor_DM_Control_Method Class_Motor_DM_1_To_4::Get_Control_Method() const
{
    return (Motor_DM_Control_Method);
}

/**
 * @brief 获取目标的角度
 *
 * @return float 目标的角度
 */
inline float Class_Motor_DM_1_To_4::Get_Target_Angle() const
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float Class_Motor_DM_1_To_4::Get_Target_Omega() const
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, Nm
 *
 * @return float 目标的扭矩, Nm
 */
inline float Class_Motor_DM_1_To_4::Get_Target_Torque() const
{
    return (Target_Torque);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float Class_Motor_DM_1_To_4::Get_Feedforward_Omega() const
{
    return (Feedforward_Omega);
}

/**
 * @brief 获取前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float Class_Motor_DM_1_To_4::Get_Feedforward_Torque() const
{
    return (Feedforward_Torque);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __DM_Motor_Control_Method 电机控制方式
 */
inline void Class_Motor_DM_1_To_4::Set_Control_Method(const Enum_Motor_DM_Control_Method &__DM_Motor_Control_Method)
{
    Motor_DM_Control_Method = __DM_Motor_Control_Method;
}

/**
 * @brief 设定目标的角度
 *
 * @param __Target_Angle 目标的角度
 */
inline void Class_Motor_DM_1_To_4::Set_Target_Angle(const float &__Target_Angle)
{
    Target_Angle = __Target_Angle;
    if (Nearest_Angle != 0.0f)
    {
        float delta_angle = Basic_Math_Modulus_Normalization(Target_Angle - Rx_Data.Now_Angle, Nearest_Angle * 2.0f);

        Target_Angle = Rx_Data.Now_Angle + delta_angle;
    }
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void Class_Motor_DM_1_To_4::Set_Target_Omega(const float &__Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __Target_Torque 目标的扭矩, Nm
 */
inline void Class_Motor_DM_1_To_4::Set_Target_Torque(const float &__Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void Class_Motor_DM_1_To_4::Set_Feedforward_Omega(const float &__Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __Feedforward_Torque 前馈的扭矩, Nm
 */
inline void Class_Motor_DM_1_To_4::Set_Feedforward_Torque(const float &__Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}
#endif /* __DRV_MOTOR_DM_H__ */
