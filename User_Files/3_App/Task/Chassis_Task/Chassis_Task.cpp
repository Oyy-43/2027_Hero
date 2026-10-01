/**
 * @file Chassis_Task.cpp
 * @author Oyyp
 * @brief 下板底盘代码所使用的任务
 * @version 0.1
 * @date 2026-09-15 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "Chassis_Task.h"

// #include "2_Device/BSP/BMI088/bsp_bmi088.h"
// #include "2_Device/Plotter/Vofa/dvc_vofa.h"
// #include "2_Device/BSP/W25Q64JV/bsp_w25q64jv.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
// #include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "1_Middleware/Driver/CRSF/crsf.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"
#include "1_Middleware/Algorithm/SingWave/alg_sin.h"
#include "1_Middleware/Algorithm/Slope/alg_slope.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "2_Device/Motor/Motor_DJI/drv_motor_dji.h"
#include "2_Device/Motor/Motor_DM/drv_motor_dm.h"
#include <stdbool.h>
#include "steer_control.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
// Class_Motor_DJI_C620 Motor_C620[4];
Class_Motor_DM_Normal Motor_DM_6220[3];
Class_Motor_DM_Normal Motor_DM_80[3];
Triangle_Steer_Chassis_Control Steer_Chassis;
Class_Slope Slope_VX,Slope_VY,Slope_VW;
float cmd_vx,cmd_vy,cmd_vw;
float test_feedforward_torque = 0.0f;
/* Private variables ---------------------------------------------------------*/

// LED灯
int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

// 全局初始化完成标志位
bool init_finished = false;

RAM_D2_BUFFER uint16_t test_count = 0;

RAM_D2_BUFFER float Sin_Out=0.0f;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief CAN1回调函数
 *
 *
 */
void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
        case (0x21):
        {
            Motor_DM_80[0].CAN_RxCpltCallback();

            break;
        }
        case (0x22):
        {
            Motor_DM_80[1].CAN_RxCpltCallback();

            break;
        }
        case (0x23):
        {
            Motor_DM_80[2].CAN_RxCpltCallback();

            break;
        }
    }
}

void CAN2_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
        case (0x11):
        {
            Motor_DM_6220[0].CAN_RxCpltCallback();

            break;
        }
        case (0x12):
        {
            Motor_DM_6220[1].CAN_RxCpltCallback();

            break;
        }
        case (0x13):
        {
            Motor_DM_6220[2].CAN_RxCpltCallback();

            break;
        }
    }
}

void CAN3_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    return ;
}
/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每500ms调用一次
 * 
 */
void Task500ms_Callback()
{
    Tele_TIM_PeriodElapsedCallback();
}

/**
 * @brief 每100ms调用一次
 *
 */
void Task100ms_Callback()
{
    Motor_DM_6220[0].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_DM_6220[1].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_DM_6220[2].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_DM_80[0].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_DM_80[1].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_DM_80[2].TIM_100ms_Alive_PeriodElapsedCallback();
}


/**
 * @brief 每1ms调用一次电机计算函数
 *
 */
void Task1ms_Chassis_Calculate_Callback()
{   
    Chassis_Control_Task();
}


/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    //底盘数据更新函数,1ms调用一次
    Task1ms_Chassis_Calculate_Callback();
    
    // dji电机的CAN发送函数, 2ms发送一次


    BSP_WS2812.Set_RGB(0, 0, 0);

    // 发送实例
    BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();

    BSP_Key.TIM_1ms_Process_PeriodElapsedCallback();

    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;

        // 处理按键状态
        BSP_Key.TIM_50ms_Read_PeriodElapsedCallback();
    }
    static uint16_t mod100 = 0;

    mod100++;
    if (mod100 == 100)
    {
        mod100 = 0;

        // Task100ms_Callback();
    }
    
    static uint16_t mod500 = 0;
    mod500++;
    if(mod500==500)
    {
        mod500=0;
        Task500ms_Callback();
    }

    // 喂狗
    TIM_1ms_IWDG_PeriodElapsedCallback();

    // Wave_Output();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);

    // USB初始化

    // 陀螺仪的SPI

    // WS2812的SPI
    SPI_Init(&hspi6, nullptr);

    // 电机的CAN
    CAN_Init(&hfdcan1, CAN1_Callback);
    CAN_Init(&hfdcan2, CAN2_Callback);

    //电源ADC的初始化
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    
    // flash的OSPI

    //电机滤波器初始化
    Filter_Init_All();

    //斜波规划器初始化
    Slope_VX.Init(0.08f, 0.8f, Slope_First_REAL);
    Slope_VY.Init(0.08f, 0.8f, Slope_First_REAL);
    Slope_VW.Init(0.08f, 0.8f, Slope_First_REAL);
    
    //电机初始化
    Motor_Init();

    //电机PID参数初始化
    PID_Init_All();
    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim7);

    // 串口空闲中断回调初始化
    UART_Init(&huart7, crsf_rx_idle_callback);
    
    // 初始化WS2812灯珠, 默认灯灭
    BSP_WS2812.Init(0, 0, 0);

    // BSP_Buzzer.Init();

    BSP_Power.Init();

    BSP_Key.Init();

    // BSP_BMI088.Init();

    BSP_Power.Set_Power(true, true, true);

    //底盘初始化
    Steer_Chassis.Init(3.0f, 0.06f, 0.2887f, 150.0f);

    // 标记初始化完成
    init_finished = true;

}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
    Namespace_SYS_Timestamp::Delay_Millisecond(1);
}

/**
 * @brief 电机初始化
 *
 */
void Motor_Init()
{
    Motor_DM_80[0].Init(&hfdcan1, 0x21,0x10, Motor_DM_Control_Method_NORMAL_MIT_Omega, 12.5f, 45.0f, 54.0f);
    Motor_DM_80[1].Init(&hfdcan1, 0x22,0x20, Motor_DM_Control_Method_NORMAL_MIT_Omega, 12.5f, 45.0f, 54.0f);
    Motor_DM_80[2].Init(&hfdcan1, 0x23,0x30, Motor_DM_Control_Method_NORMAL_MIT_Omega, 12.5f, 45.0f, 54.0f);

    Motor_DM_6220[0].Init(&hfdcan2, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT_Position,3.141593f,45.0f,10.0f);
    Motor_DM_6220[1].Init(&hfdcan2, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT_Position,3.141593f,45.0f,10.0f);
    Motor_DM_6220[2].Init(&hfdcan2, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT_Position,3.141593f,45.0f,10.0f);
}

void Motor_Enable()
{
    Motor_DM_6220[0].CAN_Send_Enter();
    osDelay(10);
    Motor_DM_6220[1].CAN_Send_Enter(); 
    osDelay(10);
    Motor_DM_6220[2].CAN_Send_Enter();
    osDelay(10);
    Motor_DM_80[0].CAN_Send_Enter();
    osDelay(10);
    Motor_DM_80[1].CAN_Send_Enter(); 
    osDelay(10);
    Motor_DM_80[2].CAN_Send_Enter();
}

/**
 * @brief 给整车的电机PID参数初始化
 * 
 */
void PID_Init_All()
{   
    //底盘轮向电机PID参数初始化

    //舵向电机速度环PID参数初始化
    PID_Init(&Motor_DM_6220[0].PID_Omega,2.7f, 0.7f, 0.00f, 0.001f,0.025f,0.35f,0.0f,0.025f,2.0f,0.5f,0,0,Integral_Limit);
    PID_Init(&Motor_DM_6220[1].PID_Omega,2.7f, 0.7f, 0.00f, 0.001f,0.025f,0.35f,0.0f,0.025f,2.0f,0.5f,0,0,Integral_Limit);
    PID_Init(&Motor_DM_6220[2].PID_Omega,2.7f, 0.7f, 0.00f, 0.001f,0.025f,0.35f,0.0f,0.025f,2.0f,0.5f,0,0,Integral_Limit);


    //舵向电机角度环PID参数初始化                                              0.85
    PID_Init(&Motor_DM_6220[0].PID_Angle,30.0f, 0.0f, 0.0f, 0.001f,27.5f,0.0f,0.85f,6.25f,0,0,0,0,Integral_Limit|Derivative_On_Measurement);
    PID_Init(&Motor_DM_6220[1].PID_Angle,30.0f, 0.0f, 0.0f, 0.001f,27.5f,0.0f,0.85f,6.25f,0,0,0,0,Integral_Limit|Derivative_On_Measurement);
    PID_Init(&Motor_DM_6220[2].PID_Angle,30.0f, 0.0f, 0.0f, 0.001f,27.5f,0.0f,0.85f,6.25f,0,0,0,0,Integral_Limit|Derivative_On_Measurement);
}

/**
 * @brief 给整车的电机滤波器初始化
 * 
 */
void Filter_Init_All()
{
    //底盘轮向电机滤波器初始化
    // for(int i =0;i<4;i++){
    //     Motor_C620[i].Filter_Omega.Init(0.0f,0.0f,Filter_Frequency_Type_LOWPASS,50.0f,0.0f,1000.0f);
    // }
    for(int i =0;i<3;i++){
        Motor_DM_6220[i].Filter_Omega.Init(0.0f,0.0f,Filter_Frequency_Type_LOWPASS,40.0f,0.0f,1000.0f);
    }
}

/**
 * @brief 波形输出8
 * 
 */
void Wave_Output()
{
    static int8_t press_count = -1;
    if(BSP_Key.Get_Key_Status()== BSP_Key_Status_TRIG_PRESSED_FREE)
    {
        press_count++;
    }
     if(press_count%2==1)
     {
    //     ALG_Sin_Generate(&Sin_Out, 2.0f, -4.0f, 1000.0f);
    //     // Motor_DM_6220[0].Set_Target_Angle(Sin_Out);
    //     // ALG_Value_Toggle_Periodic(&Sin_Out, 1.57f,-1.57f,2.0f,1000.0f);
           // Motor_DM_6220[0].Set_Target_Angle(Sin_Out);
    //     // ALG_Sin_Generate(&Sin_Out, 2.5f, -11.0f, 1000.0f);
    //     // Motor_DM_6220[0].Set_Target_Omega(Sin_Out);
     }
    // else
    // {
    //     Sin_Out = 0.0f;
    //     // Motor_DM_6220[0].Set_Target_Angle(0.0f);
    //     // Motor_DM_6220[0].Set_Target_Omega(0.0f);
    //     Motor_DM_4340P.Set_Target_Omega(0.0f);
    // }
}

void Chassis_Control_Task()
{
    cmd_vx = Slope_VX.TIM_Calculate_GetOut((float)rc_channels.ch[1] / 1640.0f,Steer_Chassis.Get_Chassis_Now_Velocity_X());
    cmd_vy = Slope_VY.TIM_Calculate_GetOut((float)(-rc_channels.ch[0]) / 1640.0f,Steer_Chassis.Get_Chassis_Now_Velocity_Y());
    cmd_vw = Slope_VW.TIM_Calculate_GetOut((float)rc_channels.ch[3] / 1640.0f,Steer_Chassis.Get_Chassis_Now_Velocity_W());

    // Steer_Chassis.Set_Target_Velocity(cmd_vx, cmd_vy, cmd_vw);

    Steer_Chassis.TIM_Calculate_PeriodElapsedCallback(Motor_DM_80[0].Get_Now_Omega(),Motor_DM_80[1].Get_Now_Omega(),Motor_DM_80[2].Get_Now_Omega(),
    Motor_DM_6220[0].Get_Now_Angle(),Motor_DM_6220[1].Get_Now_Angle(),Motor_DM_6220[2].Get_Now_Angle());

    Motor_DM_80[0].Set_Control_Torque(Steer_Chassis.Get_Motor_Target_Torque()[0]);
    Motor_DM_80[1].Set_Control_Torque(Steer_Chassis.Get_Motor_Target_Torque()[1]);
    Motor_DM_80[2].Set_Control_Torque(Steer_Chassis.Get_Motor_Target_Torque()[2]);

    Motor_DM_80[0].Set_Target_Omega(Steer_Chassis.Get_Motor_Target_Omega()[0]);
    Motor_DM_80[1].Set_Target_Omega(Steer_Chassis.Get_Motor_Target_Omega()[1]);
    Motor_DM_80[2].Set_Target_Omega(Steer_Chassis.Get_Motor_Target_Omega()[2]);

    //舵轮的坐标系其实和底盘的坐标系Z轴是相反的，所以这里要取负号
    Motor_DM_6220[0].Set_Target_Angle(-Steer_Chassis.Get_Steer_Target_Angle()[0]);
    Motor_DM_6220[1].Set_Target_Angle(-Steer_Chassis.Get_Steer_Target_Angle()[1]);
    Motor_DM_6220[2].Set_Target_Angle(-Steer_Chassis.Get_Steer_Target_Angle()[2]);
}

void Chassis_Task_Func(void *argument)
{
    osDelay(1000);
    //初始化电机
    Motor_Enable();
    for(;;)
    {
        // dm电机的CAN发送函数,1ms发送一次
        Motor_DM_6220[0].TIM_Send_PeriodElapsedCallback();              
        Motor_DM_6220[1].TIM_Send_PeriodElapsedCallback();
        Motor_DM_6220[2].TIM_Send_PeriodElapsedCallback();
        static int mod2 = 0;
        mod2++;
        if (mod2 == 2)
        {
            mod2 = 0;
            // Motor_DM_80[0].TIM_Send_PeriodElapsedCallback();
            // Motor_DM_80[1].TIM_Send_PeriodElapsedCallback();
            // Motor_DM_80[2].TIM_Send_PeriodElapsedCallback();
            Motor_DM_80[0].Output_Torque();
            Motor_DM_80[1].Output_Torque();
            Motor_DM_80[2].Output_Torque();
        }   
        osDelay(1);
    }
}
