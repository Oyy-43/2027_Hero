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
Class_Motor_DJI_C620 Motor_C620[3];
Class_Motor_DM_Normal Motor_DM_6220[3];

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
        case (0x201):
        {
            Motor_C620[0].CAN_RxCpltCallback();

            break;
        }
        case (0x202):
        {
            Motor_C620[1].CAN_RxCpltCallback();

            break;
        }
        case (0x203):
        {
            Motor_C620[2].CAN_RxCpltCallback();

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
    Motor_C620[0].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_C620[1].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_C620[2].TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 每1ms调用一次电机计算函数
 *
 */
void Task1ms_Chassis_Calculate_Callback()
{   
    Chassis_Control_Task();
}

