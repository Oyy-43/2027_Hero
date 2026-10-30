#pragma once
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/
#define POWER_HEADER          0xAAu     // 帧头
#define POWER_TAIL            0x55u     // 帧尾 
#define POWER_FRAME_LENGTH    15u       // 整帧字节数
#define POWER_CHECKSUM_LEN    13u       // 校验和覆盖的字节数
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 功率计解析结果
 */
typedef struct
{
    float voltage;     
    float current;    
    float power;     
} power_data_t;

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

extern volatile power_data_t power_data;

/* Exported function declarations --------------------------------------------*/
/**
 * @brief 解析功率计数据（从 UART DMA 接收缓冲区调用）
 * @param buf 数据缓冲区指针
 * @param len 数据长度
 */
void power_data_analysis(uint8_t *buf, uint16_t len);
