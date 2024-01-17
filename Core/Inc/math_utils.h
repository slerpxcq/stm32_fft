/*
 * math_utils.h
 *
 *  Created on: Jan 17, 2024
 *      Author: Admin
 */

#ifndef INC_MATH_UTILS_H_
#define INC_MATH_UTILS_H_

__STATIC_INLINE int32_t Min(int32_t x, int32_t y)
{
	return x <= y ? x : y;
}

__STATIC_INLINE int32_t Max(int32_t x, int32_t y)
{
	return x >= y ? x : y;
}

__STATIC_INLINE int32_t Abs(int32_t x)
{
	return x >= 0 ? x : -x;
}

__STATIC_INLINE int32_t Lerp(int32_t x, int32_t x0, int32_t y0, int32_t slope)
{
	int32_t t = x - x0;
	t *= slope;
	t += y0;
	return t;
}

#endif /* INC_MATH_UTILS_H_ */
