#ifndef __ALG_TRANSDATA_H
#define __ALG_TRANSDATA_H
/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"



/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

uint8_t TransData_Send_Two_Float_Frame(UART_HandleTypeDef *huart, float data1, float data2, uint8_t precision);

#endif /* __ALG_TRANSDATA_H */
