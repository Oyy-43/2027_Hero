#pragma once

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
#include "2_Device/Motor/Motor_DM/drv_motor_dm.h"
extern Class_Motor_DM_Normal Motor_DM_6216[3];
#endif

/* Exported variables --------------------------------------------------------*/

extern bool init_finished;



#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void Task1ms_Callback(void);

void Task3600s_Callback(void);

void Task_Init();

void Task_Loop();

void Filter_Init_All();

void PID_Init_All();

void Wave_Output();

void Motor_Init();

void Chassis_Control_Task();

void Chassis_Task_Func(void *argument);  

#ifdef __cplusplus
};
#endif
