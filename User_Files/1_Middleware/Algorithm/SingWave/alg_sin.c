/**
 * @file alg_sin.c
 * @author Oyyp
 * @brief  输出一个正弦变化的变量
 * @version 0.1
 * @date 2026-04-18 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "alg_sin.h"
#include <math.h>


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

float ALG_Sin_Generate(float *p_var, float period_s, float amplitude, float call_freq_hz)
{
	static float s_phase_rad = 0.0f;
	const float two_pi = 6.28318530718f;
	float out = 0.0f;

	if ((period_s <= 0.0f) || (call_freq_hz <= 0.0f))
	{
		s_phase_rad = 0.0f;
		if (p_var != NULL)
		{
			*p_var = 0.0f;
		}
		return 0.0f;
	}

	/* 先按当前相位输出，确保首次调用时从 0 开始 */
	out = amplitude * sinf(s_phase_rad);

	if (p_var != NULL)
	{
		*p_var = out;
	}

	s_phase_rad += two_pi / (period_s * call_freq_hz);
	if (s_phase_rad >= two_pi)
	{
		s_phase_rad = fmodf(s_phase_rad, two_pi);
	}

	return out;
}

float ALG_Sin_Sweep_Generate(float *p_var, float start_freq_hz, float end_freq_hz,
                             float freq_rate_hz_s, float amplitude, float call_freq_hz)
{
	static float s_phase_rad = 0.0f;
	static double s_freq_hz = 0.0;
	static float s_last_start = 0.0f;
	static float s_last_end = 0.0f;
	static float s_last_rate = 0.0f;
	static uint8_t s_initialized = 0U;
	const float two_pi = 6.28318530718f;
	float dt_s;
	float remaining_hz;
	float ramp_time_s;
	float cycles;
	float out;

	if (!isfinite(start_freq_hz) || !isfinite(end_freq_hz) ||
		!isfinite(freq_rate_hz_s) || !isfinite(amplitude) || !isfinite(call_freq_hz) ||
		(start_freq_hz < 0.0f) || (end_freq_hz < start_freq_hz) ||
		(freq_rate_hz_s <= 0.0f) || (call_freq_hz <= 0.0f))
	{
		s_phase_rad = 0.0f;
		s_freq_hz = 0.0f;
		s_initialized = 0U;
		if (p_var != NULL)
		{
			*p_var = 0.0f;
		}
		return 0.0f;
	}

	if (!s_initialized || (s_last_start != start_freq_hz) ||
		(s_last_end != end_freq_hz) || (s_last_rate != freq_rate_hz_s))
	{
		s_phase_rad = 0.0f;
		s_freq_hz = start_freq_hz;
		s_last_start = start_freq_hz;
		s_last_end = end_freq_hz;
		s_last_rate = freq_rate_hz_s;
		s_initialized = 1U;
	}

	out = amplitude * sinf(s_phase_rad);
	if (p_var != NULL)
	{
		*p_var = out;
	}

	dt_s = 1.0f / call_freq_hz;
	remaining_hz = end_freq_hz - s_freq_hz;
	ramp_time_s = remaining_hz / freq_rate_hz_s;
	if (ramp_time_s <= dt_s)
	{
		/* 本次间隔内到达终点：分别积分升频段和恒频段，保持相位连续。 */
		cycles = end_freq_hz * dt_s - 0.5f * remaining_hz * ramp_time_s;
		s_freq_hz = end_freq_hz;
	}
	else
	{
		cycles = (s_freq_hz + 0.5f * freq_rate_hz_s * dt_s) * dt_s;
		s_freq_hz += freq_rate_hz_s * dt_s;
	}
	s_phase_rad = fmodf(s_phase_rad + two_pi * cycles, two_pi);

	return out;
}

void ALG_Value_Toggle_Periodic(float *p_var, float max_out, float min_out, float switch_time_s, float call_freq_hz)
{
	static float *s_last_var = NULL;
	static float s_last_max = 0.0f;
	static float s_last_min = 0.0f;
	static float s_last_switch_time = 0.0f;
	static float s_last_call_freq = 0.0f;
	static uint32_t s_elapsed_calls = 0U;
	static uint8_t s_next_to_max = 1U;
	float period_calls_f;
	uint32_t period_calls;

	if ((p_var == NULL) || (switch_time_s <= 0.0f) || (call_freq_hz <= 0.0f))
	{
		return;
	}

	period_calls_f = switch_time_s * call_freq_hz;
	if (period_calls_f < 1.0f)
	{
		period_calls = 1U;
	}
	else
	{
		period_calls = (uint32_t)(period_calls_f + 0.5f);
	}

	if ((s_last_var != p_var) ||
		(s_last_max != max_out) ||
		(s_last_min != min_out) ||
		(s_last_switch_time != switch_time_s) ||
		(s_last_call_freq != call_freq_hz))
	{
		s_last_var = p_var;
		s_last_max = max_out;
		s_last_min = min_out;
		s_last_switch_time = switch_time_s;
		s_last_call_freq = call_freq_hz;
		s_elapsed_calls = 0U;
		s_next_to_max = 1U;
	}

	s_elapsed_calls++;
	if (s_elapsed_calls >= period_calls)
	{
		s_elapsed_calls = 0U;
		if (s_next_to_max)
		{
			*p_var = max_out;
			s_next_to_max = 0U;
		}
		else
		{
			*p_var = min_out;
			s_next_to_max = 1U;
		}
	}
}

