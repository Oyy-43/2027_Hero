/**
 * @file alg_basic.h
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些极其简易的数学
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-11-10 1.1 修改成cpp
 * @date 2025-09-23 2.1 引入NaN判断
 *
 * @copyright Copyright (c) 2023-2025
 *
 */

#ifndef ALG_BASIC_H
#define ALG_BASIC_H

/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"
#include <float.h>
#include <stdbool.h>

/* Exported macros -----------------------------------------------------------*/

extern const float BASIC_MATH_RPM_TO_RADPS;
extern const float BASIC_MATH_DEG_TO_RAD;
extern const float BASIC_MATH_CELSIUS_TO_KELVIN;
extern const float SQRT_2;
extern const float SQRT_2_half;

/**
 * @brief 按字节的地址读取的宏定义
 * 
 */
#define  GET16(ADDR) *((int16_t*)(ADDR))
#define  GETU16(ADDR) *((uint16_t*)(ADDR))
#define  GET32(ADDR) *((int32_t*)(ADDR))
#define  GETU32(ADDR) *((uint32_t*)(ADDR))

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

float Basic_Math_Sqr(float x);

void Basic_Math_Boolean_Logical_Not(bool *Value);

uint16_t Basic_Math_Endian_Reverse_16(void *Source, void *Destination);

uint32_t Basic_Math_Endian_Reverse_32(void *Source, void *Destination);

uint8_t Basic_Math_Sum_8(const uint8_t *Address, uint32_t Length);

uint16_t Basic_Math_Sum_16(const uint16_t *Address, uint32_t Length);

uint32_t Basic_Math_Sum_32(const uint32_t *Address, uint32_t Length);

float Basic_Math_Sinc(float x);

int32_t Basic_Math_Float_To_Int(float x, float Float_1, float Float_2, int32_t Int_1, int32_t Int_2);

float Basic_Math_Int_To_Float(int32_t x, int32_t Int_1, int32_t Int_2, float Float_1, float Float_2);

bool Basic_Math_Is_Invalid_Float(float x);

float Basic_Math_Modulus_Normalization(float x, float modulus);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 * @return 输出值
 */
template<typename Type>
Type Basic_Math_Constrain(Type x, Type Min, Type Max)
{
    if (x < Min)
    {
        x = Min;
    }
    else if (x > Max)
    {
        x = Max;
    }
    return (x);
}

/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 * @return 输出值
 */
template<typename Type>
Type Basic_Math_Constrain(Type *x, Type Min, Type Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
    return (*x);
}

/**
 * @brief 求绝对值
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template<typename Type>
Type Basic_Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/