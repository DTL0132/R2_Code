/**
 * @file drv_math.h
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些数学
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DRV_MATH_H
#define DRV_MATH_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

// rpm换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
// deg换算到rad
#define DEG_TO_RAD (PI / 180.0f)
// 摄氏度换算到开氏度
#define CELSIUS_TO_KELVIN (273.15f)
// 圆周率PI
#define PI (3.14159265358979323846f)

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void Math_Boolean_Logical_Not(bool *Value);

void Math_Endian_Reverse_16(void *Address);

uint16_t Math_Endian_Reverse_16_SrcDst(void *Source, void *Destination);

void Math_Endian_Reverse_32(void *Address);

uint32_t Math_Endian_Reverse_32_SrcDst(void *Source, void *Destination);

uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length);

uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length);

uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length);

float Math_Sinc(float x);

int32_t Math_Float_To_Int(float x, float Float_Min, float Float_Max, int32_t Int_Min, int32_t Int_Max);

float Math_Int_To_Float(int32_t x, int32_t Int_Min, int32_t Int_Max, float Float_Min, float Float_Max);

/**
 * @brief 限幅函数
 *
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
float Math_Constrain(float *x, float Min, float Max);

/**
 * @brief 求绝对值
 *
 * @param x 传入数据
 * @return float x的绝对值
 */
float Math_Abs(float x);

/**
 * @brief 求取模归化
 *
 * @param x 传入数据
 * @param modulus 模数
 * @return float 返回的归化数, 介于 ±modulus / 2 之间
 */
float Math_Modulus_Normalization(float x, float modulus);

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
