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
const ALG_SweepPoint_t k_sweep_table[] = {
    /* freq_hz, cycles */
    { 0.50f,  6U}, { 0.63f,  6U}, { 0.80f,  7U}, { 1.00f,  8U},
    { 1.25f,  8U}, { 1.60f, 10U}, { 2.00f, 10U}, { 2.50f, 12U},
    { 3.15f, 12U}, { 4.00f, 15U}, { 5.00f, 15U}, { 6.30f, 20U},
    { 8.00f, 20U}, {10.00f, 25U}, {12.50f, 30U}, {16.00f, 40U},
    {20.00f, 50U}, {25.00f, 60U}
};
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

float ALG_Sin_Step_Sweep_Generate(float *p_var, const ALG_SweepPoint_t *table,
                                  uint32_t point_count, float amplitude, float call_freq_hz)
{
	static double s_cycles = 0.0;
	static uint32_t s_point_index = 0U;
	static const ALG_SweepPoint_t *s_last_table = NULL;
	static uint32_t s_last_count = 0U;
	const double two_pi = 6.283185307179586;
	double remaining_s;
	double time_to_step_s;
	uint8_t valid;
	float out;

	valid = (table != NULL) && (point_count > 0U) && isfinite(amplitude) &&
		isfinite(call_freq_hz) && (call_freq_hz > 0.0f);
	if (valid && ((s_last_table != table) || (s_last_count != point_count)))
	{
		for (uint32_t point_index = 0U; point_index < point_count; point_index++)
		{
			if (!isfinite(table[point_index].freq_hz) ||
				(table[point_index].freq_hz <= 0.0f) || (table[point_index].cycles == 0U))
			{
				valid = 0U;
				break;
			}
		}
	}
	if (!valid)
	{
		s_cycles = 0.0;
		s_point_index = 0U;
		s_last_table = NULL;
		s_last_count = 0U;
		if (p_var != NULL)
		{
			*p_var = 0.0f;
		}
		return 0.0f;
	}

	if ((s_last_table != table) || (s_last_count != point_count))
	{
		s_cycles = 0.0;
		s_point_index = 0U;
		s_last_table = table;
		s_last_count = point_count;
	}

	out = amplitude * sinf((float)(two_pi * fmod(s_cycles, 1.0)));
	if (p_var != NULL)
	{
		*p_var = out;
	}

	remaining_s = 1.0 / (double)call_freq_hz;
	while (s_point_index + 1U < point_count)
	{
		time_to_step_s = ((double)table[s_point_index].cycles - s_cycles) /
			(double)table[s_point_index].freq_hz;
		if (remaining_s < time_to_step_s)
		{
			s_cycles += (double)table[s_point_index].freq_hz * remaining_s;
			return out;
		}
		remaining_s -= time_to_step_s;
		s_cycles = 0.0;
		s_point_index++;
	}
	s_cycles = fmod(s_cycles + (double)table[s_point_index].freq_hz * remaining_s, 1.0);

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

