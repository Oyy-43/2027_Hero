/**
 * @file Power_meter.cpp
 * @author oyyp
 * @brief description
 * @version 0.1
 * @date 2026-10-30 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "Power_meter.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

// 功率计数据
volatile power_data_t power_data; 

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
/**
 * @brief 解析功率计数据
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
void power_data_analysis(uint8_t *buf, uint16_t len)
{
    // 检查输入参数有效性
    if (buf == nullptr || len < POWER_FRAME_LENGTH) 
    {
        return;
    }

    // 检查帧头和帧尾
    uint8_t offset = 0;

    while(offset + POWER_FRAME_LENGTH <= len)
    {
        // 检查数据帧头是否正确，确保数据帧的起始位置有效
        if (buf[offset] != POWER_HEADER)
        {
            ++offset;
            continue;
        }

        // 校验帧尾，确保数据完整性
        if (buf[offset + POWER_FRAME_LENGTH - 1] != POWER_TAIL)
        {
            ++offset;
            continue;
        }

        // 校验和验证：前 13 字节求和（帧头 + 电压 + 电流 + 功率）
        uint8_t sum = 0;
        for (uint8_t i = 0; i < POWER_CHECKSUM_LEN; i++)
        {
            sum += buf[offset + i];
        }
        
        if (sum != buf[offset + POWER_CHECKSUM_LEN])
        {
            ++offset;
            continue;
        }

        // 用局部变量中转，直接指针转换读取
        power_data.voltage = *(const float *)&buf[offset + 1];
        power_data.current = *(const float *)&buf[offset + 5];
        power_data.power   = *(const float *)&buf[offset + 9];

        offset += POWER_FRAME_LENGTH;
    }
}

/* Function prototypes -------------------------------------------------------*/

