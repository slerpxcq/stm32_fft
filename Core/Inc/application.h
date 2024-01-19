#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "ssd1362.h"
#include "main.h"
#include "arm_math.h"
#include "tables.h"

#define BAR_FALL_SPEED 1
#define DOT_FALL_SPEED (1U << 7)
#define DOT_TTL 64
#define FFT_SIZE 1024
//#define DC_BIAS 2130
#define LOG_RESULT_OFFSET 350000000
#define LOG_RESULT_SCALE 6
#define ARR_SIZE(x) (sizeof(x)/sizeof(x[0]))
#define INTERP_COUNT ARR_SIZE(jumpPoints)
#define INTERP_END jumpPoints[INTERP_COUNT-1]
#define INTERP_SHIFT 1
#define FILL_GRADIENT 0
#define LOG_SCALE 1

#define INTERP_METHOD_TABULAR 0
#define INTERP_METHOD_SPLINE 1
#define INTERP_METHOD_LINEAR 2
#define INTERP_METHOD INTERP_METHOD_SPLINE

void DoFFTAndUpdateDisplay(int32_t* buf);

#endif // _APPLICATION_H_
