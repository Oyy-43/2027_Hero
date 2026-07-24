#ifndef __DRV_MOTOR_LK_H__
#define __DRV_MOTOR_LK_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Algorithm/PID/alg_pid.h"
#include "1_Middleware/Algorithm/Basic/alg_basic.h"
#include "1_Middleware/Algorithm/Filter/Frequency/alg_filter_frequency.h"


/* Exported macros -----------------------------------------------------------*/
#define MF_Current_Mapping 0.008056640625   //  33/4096
#define MG_Current_Mapping 0.01611328125    //  66/4096
/* Exported types ------------------------------------------------------------*/
/**
 * @brief 瓴控状态
 *
 */
enum Enum_Motor_LK_Status
{
    Motor_LK_Status_DISABLE = 0,
    Motor_LK_Status_ENABLE,
};

/**
 * @brief 瓴控电机的ID枚举类型
 *
 */
enum Enum_Motor_LK_ID
{
    Motor_LK_ID_0x141 = 0x141,
    Motor_LK_ID_0x142 = 0x142,
    Motor_LK_ID_0x143 = 0x143,
    Motor_LK_ID_0x144 = 0x144,
};

/**
 * @brief 瓴控电机的类别区分
 *
 */
enum Enum_Motor_LK_Type
{
    MS = 0,
    MF,
    MG,
};


/**
 * @brief 瓴控电机控制方式
 *
 */
enum Enum_Motor_LK_Control_Method
{
    Motor_LK_Control_Method_Current = 0,
    Motor_LK_Control_Method_OMEGA,
    Motor_LK_Control_Method_ANGLE,
};

/**
 * @brief 瓴控电机状态
 *
 */
enum Enum_Motor_LK_Motor_State
{
    Motor_LK_Motor_State_OFF = 0,
    Motor_LK_Motor_State_ON,
};

/**
 * @brief 瓴控电机错误状态
 *
 */
enum Enum_Motor_LK_Error_State
{
    Motor_LK_Error_State_UNDERVOLTAGE=0,                    //欠压
    Motor_LK_Error_State_OVERVOLTAGE,                       //过压  
    Motor_LK_Error_State_Mos_OverTemperature,               //MOS过温（电驱温度过高）
    Motor_LK_Error_State_Rotor_OverTemperature,             //转子过温（电机温度过高）
    Motor_LK_Error_State_OverCurrent,                       //过流
    Motor_LK_Error_State_Short_Circuit,                     //短路
    Motor_LK_Error_State_Stock,                             //堵转
    Motor_LK_Error_State_Control_Cmd_Timeout,               //控制指令超时
};

/**
 * @brief 瓴控电机传统模式源数据
 *
 */
struct Struct_Motor_LK_CAN_Rx_Data_Row
{
    int8_t Motor_Temperature;           //单位1°C/LSB
    float voltage;                      //单位0.01V/LSB
    float current;                      //母线电流，单位0.01A/LSB
    uint8_t Motor_State;                //电机状态，各个位代表不同的电机状态
    uint8_t Error_State;                //错误状态，各个位代表不同的电机错误状态
    float Torque_Current;               //电机转矩电流，iq分辨率：MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    int16_t Speed;                      //单位： 1dps/LSB
    uint16_t Encoder;                   //编码器计数 范围：0~65535
    uint16_t EncoderRaw;                //原始编码器计数 范围：0~65535
    uint16_t EncoderOffset;             //编码器0偏移 范围：0~65535  该点作为电机角度的零点
    float  iA;                          //电机相电流A分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    float  iB;                          //电机相电流B分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    float  iC;                          //电机相电流C分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint8_t  brake_state;               //电机刹车状态,如果为0x00则刹车启动，为0x01则刹车释放
    float  Motor_multiAngle;            //电机多圈角度值，正值表示顺时针累计角度，负值表示逆时针累计角度
    float  CircleAngle;                 //电机单圈角度值，以编码器零点为起始点，顺时针增加，再次到达零点时数值回 0，单位 0.01°/LSB，数值范围 0~36000*减速比-1。
};

/**
 * @brief 瓴控电机经过处理的数据
 *
 */
struct Struct_Motor_LK_Rx_Data
{
    int8_t Motor_Temperature;
    float Power;
    float Torque;
    float Now_Angle;
    float Filtered_Now_Angle;
    float Now_Omega;
    float Filtered_Now_Omega;
    uint16_t Encoder;
    uint8_t Motor_State;
    uint8_t Error_State;
};

/**
 * @brief 瓴控反馈报文格式1 
 * 
 */
struct Struct_Motor_LK_Data_Frame1
{
    uint8_t cmd_id;
    uint8_t Motor_Temperature;           //单位1°C/LSB
    uint16_t Voltage;                    //单位0.01V/LSB
    uint16_t Current;                    //单位MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint8_t Motor_State;                 //电机状态，各个位代表不同的电机状态
    uint8_t Error_State;                 //错误状态，各个位代表不同的电机错误状态
}__attribute__((packed));

/**
 * @brief 瓴控反馈报文格式2 
 * 
 */
struct Struct_Motor_LK_Data_Frame2
{
    uint8_t cmd_id;
    uint8_t Motor_Temperature;           //单位1°C/LSB
    uint16_t Current;                    //单位MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint16_t Speed;                      //单位： 1dps/LSB
    uint16_t Encoder;                    //编码器计数 范围：0~65535
}__attribute__((packed));

/**
 * @brief 瓴控反馈报文格式3 
 * 
 */
struct Struct_Motor_LK_Data_Frame3
{
    uint8_t cmd_id;
    uint8_t Motor_Temperature;           //单位1°C/LSB
    uint16_t  iA;                        //电机相电流A分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint16_t  iB;                        //电机相电流B分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint16_t  iC;                        //电机相电流C分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
}__attribute__((packed));

/**
 * @brief 瓴控反馈报文刹车状态数据          
 * 
 */
struct Struct_Motor_LK_Data_Breakstate
{
    uint8_t cmd_id;
    uint8_t brake_state;               //电机刹车状态,如果为0x00则刹车启动，为0x01则刹车释放
    uint8_t reserved[6];
}__attribute__((packed));

/**
 * @brief 瓴控反馈报文编码器数据          
 * 
 */
struct Struct_Motor_LK_Data_Encoder
{
    uint8_t cmd_id;
    uint8_t reserved;
    uint16_t Encoder;                   //编码器计数 范围：0~65535
    uint16_t EncoderRaw;                //原始编码器计数 范围：0~65535
    uint16_t EncoderOffset;             //编码器0偏移 范围：0~65535  该点作为电机角度的零点
}__attribute__((packed));

/**
 * @brief 瓴控反馈报文多圈角度数据          
 * 
 */
struct Struct_Motor_LK_Data_MultiAngle
{
    uint8_t cmd_id;      // 0x92
    uint8_t angle_b[7];  // DATA[1]..DATA[7], little-endian
}__attribute__((packed));

/**
 * @brief 瓴控反馈报文单圈角度数据          
 * 
 */
struct Struct_Motor_LK_Data_CircleAngle
{
    uint8_t cmd_id;      // 0x93
    uint8_t reserved[3];  // DATA[1]..DATA[7], little-endian
    uint32_t CircleAngle;  // DATA[4]..DATA[7], little-endian, 单圈角度值，以编码器零点为起始点，顺时针增加，再次到达零点时数值回0，单位0.01°/LSB，数值范围0~36000*减速比-1。
}__attribute__((packed));


/**
 * @brief 瓴控电机类
 *
 */
class Class_Motor_LK
{
public:
    // PID角度环控制
    PID_TypeDef PID_Angle;
    // PID角速度环控制
    PID_TypeDef PID_Omega;

    Class_Filter_Frequency<> Filter_Angle;

    Class_Filter_Frequency<> Filter_Omega;

    void Init(const FDCAN_HandleTypeDef *hcan,const Enum_Motor_LK_ID __Motor_LK_ID,const Enum_Motor_LK_Type &__Motor_LK_Type = MF,
        const Enum_Motor_LK_Control_Method &__Motor_LK_Control_Method = Motor_LK_Control_Method_Current, 
        const float &__Nearest_Angle = 0.0f,float Reducer_Ratio=1.0f);

    inline uint8_t Get_Now_Temperature() const;
    
    inline float Get_Now_Power() const;
    
    inline uint8_t Get_Status() const;

    inline uint8_t Get_Error_State() const; 

    inline float Get_Now_Torquet() const;

    inline float Get_Now_Omega() const;

    inline uint16_t Get_Now_Encoder() const;

    inline float Get_Now_Angle() const;

    void CAN_RxCpltCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    //初始化相关变量

    // 瓴控电机的类型
    Enum_Motor_LK_Type Motor_LK_Type;
    // 绑定的CAN   
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // B瓴控tm发送的CAN_ID和接收的CAN_ID是一样的，都是0x140+id
    Enum_Motor_LK_ID CAN_ID;
    // 减速比
    float Reducer_Ratio;
    // 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
    float Nearest_Angle;
    // 电流控制最大输出刻度
    int16_t OUT_MAX = 2048;
    
    //常量
    const float Torque_Current_Mapping = 0.0005f; //电机转矩电流与扭矩的映射关系，单位：Nm/A

    // 发送缓冲区
    uint8_t Tx_Data[8];

    //内部变量
 
    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;

    // 读变量

    // 电机状态
    Enum_Motor_LK_Status Motor_DJI_Status = Motor_LK_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_LK_Rx_Data Rx_Data;

    // 写变量

    // 读写变量
    Struct_Motor_LK_CAN_Rx_Data_Row Rx_Data_Row;
    // 电机控制方式
    Enum_Motor_LK_Control_Method Motor_LK_Control_Method = Motor_LK_Control_Method_ANGLE;
    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的扭矩, Nm
    float Target_Torque = 0.0f;
    // 目标的电流
    float Target_Current = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的扭矩, Nm
    float Feedforward_Torque = 0.0f;
    // 前馈的电流
    float Feedforward_Current = 0.0f;

    // 内部函数

    void Data_Process();

    void Data_Frame1_Process();

    void Data_Frame2_Process();

    void Data_Frame3_Process();

    void Data_BreakState_Process();

    void Data_Encoder_Process();

    void Data_MultiAngle_Process();

    void Data_CircleAngle_Process();

    void PID_Cal();

    void Output();
};
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_LK_Status 电机状态
 */
inline uint8_t Class_Motor_LK::Get_Now_Temperature() const
{
    return (Rx_Data.Motor_Temperature);
}

/**
 * @brief 获取电机功率
 *
 * @return float 电机功率
 */
inline float Class_Motor_LK::Get_Now_Power() const
{
    return (Rx_Data.Power);
}

/**
 * @brief 
 * 
 * @return Enum_Motor_LK_Status 
 */
inline uint8_t Class_Motor_LK::Get_Status() const
{
    return (Motor_DJI_Status);
}

/**
 * @brief 获取电机错误状态
 *
 * @return Enum_Motor_LK_Error_State 电机错误状态
 */
inline uint8_t Class_Motor_LK::Get_Error_State() const
{
    return (Rx_Data.Error_State);
}

/**
 * @brief 获取电机扭矩   单位: Nm
 *
 * @return float 电机扭矩
 */
inline float Class_Motor_LK::Get_Now_Torquet() const
{
    return (Rx_Data.Torque);
}

/**
 * @brief 获取电机角速度 单位: °/s
 *
 * @return float 电机角速度
 */
inline float Class_Motor_LK::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取电机编码器值
 *
 * @return uint16_t 电机编码器值
 */
inline uint16_t Class_Motor_LK::Get_Now_Encoder() const
{
    return (Rx_Data.Encoder);
}

/**
 * @brief 获取电机角度 单位：°
 *
 * @return float 电机角度
 */
inline float Class_Motor_LK::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

#endif /* __DRV_MOTOR_LK_H__ */
