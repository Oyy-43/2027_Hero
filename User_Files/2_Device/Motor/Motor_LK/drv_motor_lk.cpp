/**
 * @file drv_motor_lk.cpp
 * @author Oyyp
 * @brief 自己编写的lk驱动库
 * @version 0.1
 * @date 2026-07-22 0.1 init
 *
 * @copyright Copyright HUNAU-RM team
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_motor_lk.h"
#include "1_Middleware/Algorithm/Basic/alg_basic.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
void Class_Motor_LK::Init(const FDCAN_HandleTypeDef *hcan,const Enum_Motor_LK_ID __Motor_LK_ID,const Enum_Motor_LK_Type &__Motor_LK_Type,const Enum_Motor_LK_Control_Method &__Motor_LK_Control_Method, const float &__Nearest_Angle, float Reducer_Ratio)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }

    this->CAN_ID = __Motor_LK_ID;
    this->Motor_LK_Type = __Motor_LK_Type;
    this->Motor_LK_Control_Method = __Motor_LK_Control_Method;
    this->Nearest_Angle = __Nearest_Angle;
    this->Reducer_Ratio = Reducer_Ratio;
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void Class_Motor_LK::CAN_RxCpltCallback()
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_LK::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_LK_Status = Motor_LK_Status_DISABLE;
        this->PID_Angle.Iout=0.0f;
        this->PID_Omega.Iout=0.0f;
    }
    else
    {
        // 电机保持连接
        Motor_LK_Status = Motor_LK_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器1ms发送CAN报文
 *
 */
void Class_Motor_LK::TIM_1ms_PeriodElapsedCallback()
{
    CAN_Transmit_Data(&hfdcan1,CAN_ID,Tx_Data,8);   //底盘3508    
}

/**
 * @brief 瓴控电机CAN中断接收回调函数
 * 
 */
void Class_Motor_LK::Data_Process()
{
    switch (CAN_Manage_Object->Rx_Buffer[0])
    { 
        case 0x9A:
        case 0x9B:
            Data_Frame1_Process();
        break;
        case 0x9C:
        case 0xA0:
        case 0xA1:
        case 0xA2:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
        case 0xA7:
        case 0xA8:
            Data_Frame2_Process();
        break;
        case 0x9D:
            Data_Frame3_Process();
        break;
        case 0x8C:
            Data_BreakState_Process();
        break;
        case 0x90:
            Data_Encoder_Process();
        break;   
        case 0x92:
            Data_MultiAngle_Process();
        break;
        case 0x94:
            Data_CircleAngle_Process();
        break;
        default:
        break;
    }
    Rx_Data.Motor_Temperature = Rx_Data_Row.Motor_Temperature;
    Rx_Data.Power = Rx_Data_Row.voltage * Rx_Data_Row.current;
    switch (Motor_LK_Type)
    {
    case MF:
        Rx_Data.Torque = Rx_Data_Row.Torque_Current*MF_Current_Mapping*0.81;
        break;
    case MG:
        Rx_Data.Torque = Rx_Data_Row.Torque_Current*MG_Current_Mapping*0.15152;
        break;
    default:
        break;
    }

    Rx_Data.Now_Angle = (float)Rx_Data_Row.Encoder*Encoder_To_Rad_18Bit;
    if (Filter_Angle.Init_Flag)
    {
        Filter_Angle.Set_Now(Rx_Data.Now_Angle);
        Filter_Angle.TIM_Calculate_PeriodElapsedCallback();
        Rx_Data.Filtered_Now_Angle = Filter_Angle.Get_Out();
    }
    else
    {
        Rx_Data.Filtered_Now_Angle = Rx_Data.Now_Angle;
    }
    Rx_Data.Now_Omega = (float)Rx_Data_Row.Speed*BASIC_MATH_DEGPS_TO_RADPS;
    if (Filter_Omega.Init_Flag)
    {
        Filter_Omega.Set_Now(Rx_Data.Now_Omega);
        Filter_Omega.TIM_Calculate_PeriodElapsedCallback();
        Rx_Data.Filtered_Now_Omega = Filter_Omega.Get_Out();
    }
    else
    {
        Rx_Data.Filtered_Now_Omega = Rx_Data.Now_Omega;
    }
    Rx_Data.Encoder = Rx_Data_Row.Encoder;
    Rx_Data.Motor_State = Rx_Data_Row.Motor_State;
    Rx_Data.Error_State = Rx_Data_Row.Error_State;
}

void Class_Motor_LK::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Cal();

    switch (Motor_LK_Type)
    {
    case MF:
        Out = (Target_Torque + Feedforward_Torque)*MF_Torque_to_Current_Mapping;
        break;
    case MG:
        Out = (Target_Torque + Feedforward_Torque)*MG_Torque_to_Current_Mapping;
        break;
    }
    Basic_Math_Constrain(&Out, (float)-OUT_MAX, (float)OUT_MAX);

    Output();

    Feedforward_Omega = 0.0f;
    Feedforward_Torque = 0.0f;
    Feedforward_Current = 0.0f;

}

void Class_Motor_LK::Output()
{
    int16_t iq = (int16_t)Out;
    Tx_Data[0] = 0xA1;
    Tx_Data[1] = 0x00;
    Tx_Data[2] = 0x00;
    Tx_Data[3] = 0x00;
    Tx_Data[4] = (uint8_t)(iq);       // 低字节
    Tx_Data[5] = (uint8_t)(iq >> 8);  // 高字节
    Tx_Data[6] = 0x00;
    Tx_Data[7] = 0x00;
}

void Class_Motor_LK::PID_Cal()
{
    switch (Motor_LK_Control_Method)
    {
        case Motor_LK_Control_Method_Current:
            break;
        break;
        case Motor_LK_Control_Method_OMEGA:
            Target_Torque = PID_Calculate(&this->PID_Omega,Rx_Data.Now_Omega,(Target_Omega + Feedforward_Omega));
        break;
        case Motor_LK_Control_Method_ANGLE:
            Target_Omega = PID_Calculate(&this->PID_Angle,Rx_Data.Now_Angle,Target_Angle);
            Target_Torque = PID_Calculate(&this->PID_Omega,Rx_Data.Now_Omega,(Target_Omega + Feedforward_Omega));
        break;
    }
}

/**
 * @brief 帧类型1数据处理函数
 *
 */
void Class_Motor_LK::Data_Frame1_Process()
{
    int16_t temp_voltage,temp_current;
    Struct_Motor_LK_Data_Frame1 *tmp_buffer = (Struct_Motor_LK_Data_Frame1 *)CAN_Manage_Object->Rx_Buffer;
    Rx_Data_Row.Motor_Temperature = tmp_buffer->Motor_Temperature;
    temp_voltage = GET16(&tmp_buffer->Voltage);
    temp_current = GET16(&tmp_buffer->Current);
    Rx_Data_Row.voltage = (float)temp_voltage * 0.01f;
    Rx_Data_Row.current = (float)temp_current * 0.01f;
    Rx_Data_Row.Motor_State = tmp_buffer->Motor_State;
    Rx_Data_Row.Error_State = tmp_buffer->Error_State;
}

/**
 * @brief 帧类型2数据处理函数
 *
 */
void Class_Motor_LK::Data_Frame2_Process()
{
    int16_t temp_torque;
    Struct_Motor_LK_Data_Frame2 *tmp_buffer = (Struct_Motor_LK_Data_Frame2 *)CAN_Manage_Object->Rx_Buffer;
    Rx_Data_Row.Motor_Temperature = tmp_buffer->Motor_Temperature;
    temp_torque = (int16_t)GET16(&tmp_buffer->Current);
    switch (Motor_LK_Type)
    {
        case MS:
        Rx_Data_Row.Torque_Current = (float)temp_torque;
        break;
        case MF:
        Rx_Data_Row.Torque_Current = (float)temp_torque * MF_Current_Mapping;
        break;
        case MG:
        Rx_Data_Row.Torque_Current = (float)temp_torque * MG_Current_Mapping;
        break;
        default:
        break;
    }
    Rx_Data_Row.Speed = ((int16_t)GET16(&tmp_buffer->Speed))/Reducer_Ratio;
    Rx_Data_Row.Encoder =(uint16_t)GET16(&tmp_buffer->Encoder);
}

/**
 * @brief 帧类型3数据处理函数
 *
 */
void Class_Motor_LK::Data_Frame3_Process()
{
    int16_t temp_iA,temp_iB,temp_iC;
    Struct_Motor_LK_Data_Frame3 *tmp_buffer = (Struct_Motor_LK_Data_Frame3 *)CAN_Manage_Object->Rx_Buffer;
    Rx_Data_Row.Motor_Temperature = tmp_buffer->Motor_Temperature;
    temp_iA =(int16_t)GET16(&tmp_buffer->iA);
    temp_iB =(int16_t)GET16(&tmp_buffer->iB);
    temp_iC =(int16_t)GET16(&tmp_buffer->iC);
        switch (Motor_LK_Type)
    {
        case MS:
        break;
        case MF:
        Rx_Data_Row.iA = (float)temp_iA * MF_Current_Mapping;
        Rx_Data_Row.iB = (float)temp_iB * MF_Current_Mapping;
        Rx_Data_Row.iC = (float)temp_iC * MF_Current_Mapping;
        break;
        case MG:
        Rx_Data_Row.iA = (float)temp_iA * MG_Current_Mapping;
        Rx_Data_Row.iB = (float)temp_iB * MG_Current_Mapping;
        Rx_Data_Row.iC = (float)temp_iC * MG_Current_Mapping;
    }
}

/**
 * @brief 刹车状态数据处理函数
 *
 */
void Class_Motor_LK::Data_BreakState_Process()
{
    Struct_Motor_LK_Data_Breakstate *tmp_buffer = (Struct_Motor_LK_Data_Breakstate *)CAN_Manage_Object->Rx_Buffer;
    Rx_Data_Row.brake_state = tmp_buffer->brake_state;
}

/**
 * @brief 编码器数据处理函数
 *
 */
void Class_Motor_LK::Data_Encoder_Process()
{
    Struct_Motor_LK_Data_Encoder *tmp_buffer = (Struct_Motor_LK_Data_Encoder *)CAN_Manage_Object->Rx_Buffer;
    Rx_Data_Row.Encoder = (uint16_t)GET16(&tmp_buffer->Encoder);
    Rx_Data_Row.EncoderRaw = (uint16_t)GET16(&tmp_buffer->EncoderRaw);
    Rx_Data_Row.EncoderOffset = (uint16_t)GET16(&tmp_buffer->EncoderOffset);
}

/**
 * @brief 多圈角度数据处理函数
 *
 */
void Class_Motor_LK::Data_MultiAngle_Process()
{
    int64_t temp_multiAngle;
    Struct_Motor_LK_Data_MultiAngle *tmp_buffer = (Struct_Motor_LK_Data_MultiAngle *)CAN_Manage_Object->Rx_Buffer;
    uint64_t temp_multiAngle_raw = 0;
    temp_multiAngle_raw = ((uint64_t)tmp_buffer->angle_b[0]) |
                          ((uint64_t)tmp_buffer->angle_b[1] << 8) |
                          ((uint64_t)tmp_buffer->angle_b[2] << 16) |
                          ((uint64_t)tmp_buffer->angle_b[3] << 24) |
                          ((uint64_t)tmp_buffer->angle_b[4] << 32) |
                          ((uint64_t)tmp_buffer->angle_b[5] << 40) |
                          ((uint64_t)tmp_buffer->angle_b[6] << 48);
    if (temp_multiAngle_raw & (1ULL << 55))
    {
        temp_multiAngle_raw |= 0xFF00000000000000ULL;
    }
    temp_multiAngle = (int64_t)temp_multiAngle_raw;
    Rx_Data_Row.Motor_multiAngle = (float)(temp_multiAngle+1) * 0.01f/Reducer_Ratio;
}

/**
 * @brief 单圈角度数据处理函数
 *
 */
void Class_Motor_LK::Data_CircleAngle_Process()
{
    Struct_Motor_LK_Data_CircleAngle *tmp_buffer = (Struct_Motor_LK_Data_CircleAngle *)CAN_Manage_Object->Rx_Buffer;
    uint32_t temp_circleAngle_raw = 0;
    temp_circleAngle_raw = (uint32_t)(GET32(&tmp_buffer->CircleAngle));
    Rx_Data_Row.CircleAngle = (float)(temp_circleAngle_raw+1) *0.01f/Reducer_Ratio;
}