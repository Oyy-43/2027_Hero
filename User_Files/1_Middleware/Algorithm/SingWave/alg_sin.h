#ifndef _ALG_SIN_H
#define _ALG_SIN_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

float ALG_Sin_Generate(float *p_var, float period_s, float amplitude, float call_freq_hz);
void ALG_Value_Toggle_Periodic(float *p_var, float max_out, float min_out, float switch_time_s, float call_freq_hz);






#endif /* _ALG_SIN_H */