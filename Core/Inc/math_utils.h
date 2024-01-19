/*
 * math_utils.h
 *
 *  Created on: Jan 17, 2024
 *      Author: Admin
 */

#ifndef INC_MATH_UTILS_H_
#define INC_MATH_UTILS_H_

#define ONE_Q29       (1<<29)
#define ONE_HALF_Q29  (ONE_Q29>>1)
#define ONE_SIXTH_Q29 0x5555558

__STATIC_INLINE int32_t i32_min(int32_t x, int32_t y)
{
	return x <= y ? x : y;
}

__STATIC_INLINE int32_t i32_max(int32_t x, int32_t y)
{
	return x >= y ? x : y;
}

__STATIC_INLINE int32_t i32_abs(int32_t x)
{
	return x >= 0 ? x : -x;
}

__STATIC_INLINE int32_t i32_lerp(int32_t x, int32_t x0, int32_t y0, int32_t slope)
{
	int32_t t = x - x0;
	t *= slope;
	t += y0;
	return t;
}

__STATIC_INLINE int32_t q29_mult(int32_t x, int32_t y)
{
	return (int32_t)(((int64_t)x * y) >> 29);
}

__STATIC_INLINE int32_t q29_exp(int32_t x)
{
	int32_t x2, x3;
	x2 = q29_mult(x, x);
	x3 = q29_mult(x2, x);
	return ONE_Q29 + x + q29_mult(x2, ONE_HALF_Q29) + q29_mult(x3, ONE_SIXTH_Q29);
}

#endif /* INC_MATH_UTILS_H_ */
