/*
 * spline_q31.h
 *
 *  Created on: Jan 17, 2024
 *      Author: Admin
 */

#ifndef INC_SPLINE_Q31_H_
#define INC_SPLINE_Q31_H_

#include "arm_math_types.h"
#include "dsp/interpolation_functions.h"

#define FRAC_BITS 15

typedef struct
{
	arm_spline_type type;      /**< Type (boundary conditions) */
	const int32_t * x;       /**< x values */
	const int32_t * y;       /**< y values */
	uint32_t n_x;              /**< Number of known data points */
	int32_t * coeffs;        /**< Coefficients buffer (b,c, and d) */
} arm_spline_instance_q31;

void arm_spline_init_q31(
        arm_spline_instance_q31 * S,
        arm_spline_type type,
  const int32_t * x,
  const int32_t * y,
        uint32_t n,
				int32_t * coeffs,
				int32_t * tempBuffer);

void arm_spline_q31(
        arm_spline_instance_q31 * S,
  const int32_t * xq,
	      int32_t * pDst,
        uint32_t blockSize);

__STATIC_INLINE int32_t q_mult(int32_t x, int32_t y)
{
	return (int32_t)(((int64_t)x * y) >> FRAC_BITS);
}

__STATIC_INLINE int32_t q_div(int32_t x, int32_t y)
{
	return (int32_t)(((int64_t)x << FRAC_BITS) / y);
}

__STATIC_INLINE int32_t q_from_int(int32_t x)
{
	return (int32_t)((int64_t)x << FRAC_BITS);
}

__STATIC_INLINE int32_t int_from_q(int32_t x)
{
	return x >> FRAC_BITS;
}

extern const int32_t XTableQ[];

#endif /* INC_SPLINE_Q31_H_ */
