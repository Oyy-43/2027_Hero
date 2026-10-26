#ifndef _ALG_SIN_H
#define _ALG_SIN_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

float ALG_Sin_Generate(float *p_var, float period_s, float amplitude, float call_freq_hz);
/**
 * @brief 线性升频正弦波，首次输出为 0，到达终止频率后保持该频率。
 * @param p_var 输出指针，可为 NULL；返回值始终为当前采样值。
 * @param start_freq_hz 起始频率（Hz），必须 >= 0。
 * @param end_freq_hz 终止频率（Hz），必须 >= 起始频率。
 * @param freq_rate_hz_s 每秒频率增加值（Hz/s），必须 > 0。
 * @param amplitude 正弦波幅值。
 * @param call_freq_hz 实际调用频率（Hz），必须 > 0，建议 > 2 * end_freq_hz。
 * @note 单实例静态状态，不可重入。修改起始/终止频率或增速会从相位 0 重启；
 *       修改幅值或调用频率不重启。非法或非有限参数返回/写入 0 并复位。
 */
float ALG_Sin_Sweep_Generate(float *p_var, float start_freq_hz, float end_freq_hz,
                             float freq_rate_hz_s, float amplitude, float call_freq_hz);
void ALG_Value_Toggle_Periodic(float *p_var, float max_out, float min_out, float switch_time_s, float call_freq_hz);

#ifdef __cplusplus
}
#endif






#endif /* _ALG_SIN_H */