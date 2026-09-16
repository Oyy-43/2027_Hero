#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

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

void Chassis_Control_Task();

void Chassis_Task_Func(void *argument);  

#ifdef __cplusplus
};
#endif

#endif /* __CHASSIS_TASK_H__ */