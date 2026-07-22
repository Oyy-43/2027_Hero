/**
 * @file alg_transData.c
 * @author Oyyp
 * @brief 通过DMA发送数据
 * @version 0.1
 * @date 2026-04-19 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_transData.h"

#include <stdio.h>


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

#define TRANS_DATA_TX_FRAME_MAX_LEN 64U

static uint8_t trans_data_tx_frame_buf[TRANS_DATA_TX_FRAME_MAX_LEN] ;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 发送两路浮点数据帧，格式为"+x.xx -y.yy"（中间一个空格）
 *
 * @param huart 串口句柄
 * @param data1 发送数据1
 * @param data2 发送数据2
 * @param precision 小数位数（2表示保留两位小数）
 * @return uint8_t HAL状态码
 */
uint8_t TransData_Send_Two_Float_Frame(UART_HandleTypeDef *huart, float data1, float data2, uint8_t precision)
{
	int frame_len;
	uint32_t scale;
	int32_t data1_scaled;
	int32_t data2_scaled;
	char sign1;
	char sign2;
	uint32_t data1_abs;
	uint32_t data2_abs;
	uint32_t data1_int;
	uint32_t data2_int;
	uint32_t data1_frac;
	uint32_t data2_frac;
	uint8_t i;

	if (huart == NULL)
	{
		return HAL_ERROR;
	}

	if (precision > 6U)
	{
		precision = 6U;
	}

	scale = 1U;
	for (i = 0U; i < precision; i++)
	{
		scale *= 10U;
	}

	if (huart->gState != HAL_UART_STATE_READY)
	{
		return HAL_BUSY;
	}

	data1_scaled = (int32_t)((data1 >= 0.0f) ? (data1 * (float)scale + 0.5f) : (data1 * (float)scale - 0.5f));
	data2_scaled = (int32_t)((data2 >= 0.0f) ? (data2 * (float)scale + 0.5f) : (data2 * (float)scale - 0.5f));

	sign1 = (data1_scaled >= 0) ? '+' : '-';
	sign2 = (data2_scaled >= 0) ? '+' : '-';
	data1_abs = (uint32_t)((data1_scaled >= 0) ? data1_scaled : -data1_scaled);
	data2_abs = (uint32_t)((data2_scaled >= 0) ? data2_scaled : -data2_scaled);

	if (precision == 0U)
	{
		frame_len = snprintf((char *)trans_data_tx_frame_buf,
							 TRANS_DATA_TX_FRAME_MAX_LEN,
							 "%c%lu %c%lu\r\n",
							 sign1,
							 (unsigned long)data1_abs,
							 sign2,
							 (unsigned long)data2_abs);
	}
	else
	{
		data1_int = data1_abs / scale;
		data2_int = data2_abs / scale;
		data1_frac = data1_abs % scale;
		data2_frac = data2_abs % scale;

		frame_len = snprintf((char *)trans_data_tx_frame_buf,
							 TRANS_DATA_TX_FRAME_MAX_LEN,
							 "%c%lu.%0*lu %c%lu.%0*lu\r\n",
							 sign1,
							 (unsigned long)data1_int,
							 (int)precision,
							 (unsigned long)data1_frac,
							 sign2,
							 (unsigned long)data2_int,
							 (int)precision,
							 (unsigned long)data2_frac);
	}

	if ((frame_len <= 0) || ((uint32_t)frame_len >= TRANS_DATA_TX_FRAME_MAX_LEN))
	{
		return HAL_ERROR;
	}

	return UART_Transmit_Data(huart, trans_data_tx_frame_buf, (uint16_t)frame_len);
}

