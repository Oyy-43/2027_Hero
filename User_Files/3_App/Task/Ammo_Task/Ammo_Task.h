#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern bool init_finished;
/* Exported function declarations --------------------------------------------*/

void Task1ms_Callback(void);

void Task3600s_Callback(void);

void Task_Init();

void Task_Loop();

void Filter_Init_All();

void PID_Init_All();

void Wave_Output();

void Motor_Init();

void Gimbal_Control_Task();

void Gimbal_Task_Func(void *argument);  

void Motor_TIM_Calculate_PeriodElapsedCallback();

#ifdef __cplusplus
};
#endif