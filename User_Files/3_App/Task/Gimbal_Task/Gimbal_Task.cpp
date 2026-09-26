/**
 * @file Gimbal_Task.cpp
 * @author Oyyp
 * @brief 云台控制任务
 * @version 0.1
 * @date 2026-09-16 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "Gimbal_Task.h"

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
#include "2_Device/Motor/Motor_LK/drv_motor_lk.h"
#include <stdbool.h>
#include "steer_control.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
Class_Motor_LK Motor_LK[2];
Steer_Chassis_Control Steer_Chassis;
Class_Slope Slope_VX,Slope_VY,Slope_VW;
float sin_outp = 0.0f;
float cmd_vx,cmd_vy,cmd_vw;
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
        case (0x141):
        {
            Motor_LK[0].CAN_RxCpltCallback();
        }
    }
}

void CAN2_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
        return;
    }
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
 * @brief 每100ms调用一次
 *
 */
void Task100ms_Callback()
{
    Motor_LK[0].TIM_100ms_Alive_PeriodElapsedCallback();
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
 * @brief 每1ms调用一次电机计算函数
 *
 */
void Task1ms_Chassis_Calculate_Callback()
{   
    return;
}


/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{


    //底盘数据更新函数,1ms调用一次
    Task1ms_Chassis_Calculate_Callback();

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

        Task100ms_Callback();
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

    Wave_Output();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);

    // USB初始化

    //电机的初始化
    Motor_Init();

    // WS2812的SPI
    SPI_Init(&hspi6, nullptr);

    // 电机的CAN
    CAN_Init(&hfdcan1, CAN1_Callback);
    CAN_Init(&hfdcan2, CAN2_Callback);

    //电源ADC的初始化
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    
    // flash的OSPI

    

    //斜波规划器初始化
    Slope_VX.Init(0.08f, 0.8f, Slope_First_REAL);
    Slope_VY.Init(0.08f, 0.8f, Slope_First_REAL);
    Slope_VW.Init(0.08f, 0.8f, Slope_First_REAL);

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
    Motor_LK[0].Init(&hfdcan1, Motor_LK_ID_0x141, MF, Motor_LK_Control_Method_ANGLE, 0.0f, 1.0f);
}


/**
 * @brief 给整车的电机PID参数初始化
 * 
 */
void PID_Init_All()
{     
    PID_Init(&Motor_LK[0].PID_Omega,5.8f,1.4f,0.0f,0.001f,0.15f,0.45f,0.0f,0.5f,0.0f,0.0f,0,0,Integral_Limit);
    // PID_Init(&Motor_LK[0].PID_Omega,5.8f,1.4f,0.0f,0.001f,0.08f,0.50f,0.0f,0.25f,0.0f,0.0f,0,0,Integral_Limit);
    PID_Init(&Motor_LK[0].PID_Angle,24.0f,0.0f,0.00f,0.001f,12.50f,0.00f,0.00f,1000.0f,0.0f,0.0f,0,0,Integral_Limit);
}



/**
 * @brief 波形输出
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
        ALG_Sin_Generate(&Sin_Out, sin_outp, -6.28f, 1000.0f);
        Motor_LK[0].Set_Target_Angle(Sin_Out);
         // ALG_Value_Toggle_Periodic(&Sin_Out, 1.57f,-1.57f,2.0f,1000.0f);
         // ALG_Sin_Generate(&Sin_Out, 2.5f, -11.0f, 1000.0f);
        // Motor_LK[0].Set_To_Zero_Nearest_Flag();
     }
    else
    {
        Sin_Out = 0.0f;
        Motor_LK[0].Set_Target_Angle(0.0f);
    //     // Motor_DM_6220[0].Set_Target_Angle(0.0f);
    //     // Motor_DM_6220[0].Set_Target_Omega(0.0f);
    //     Motor_DM_4340P.Set_Target_Omega(0.0f);
        return;
    }
}


void Gimbal_Task_Func(void *argument)
{
    osDelay(1000);
    //初始化电机
    for(;;)
    { 
        Motor_LK[0].TIM_Calculate_PeriodElapsedCallback();
        Motor_LK[0].TIM_1ms_PeriodElapsedCallback();
        osDelay(1);
    }
}