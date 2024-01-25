#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "ssd1362.h"
#include "main.h"
#include "arm_math.h"
#include "tables.h"

// Feel free to try out different values
#define BAR_FALL_SPEED 1
#define DOT_FALL_SPEED (1 << 9)
#define BAR_COLOR 12
#define DOT_COLOR 15
#define FADE_SIZE 4
#define DOT_TTL 16
#define LOG_SCALE 1

#define INTERP_METHOD_TABULAR 0
#define INTERP_METHOD_SPLINE 1
#define INTERP_METHOD_LINEAR 2
#define INTERP_METHOD INTERP_METHOD_SPLINE

// DO NOT CHANGE
#define ARR_SIZE(x) (sizeof(x)/sizeof(x[0]))

#define FFT_SIZE 1024
#define LOG_RESULT_OFFSET 350000000
#define LOG_RESULT_SCALE 6
#define INTERP_COUNT ARR_SIZE(jumpPoints)
#define INTERP_END jumpPoints[INTERP_COUNT-1]
#define INTERP_SHIFT 1
#define DOT_FLOOR (1<<8)

void DoFFTAndUpdateDisplay(int32_t* buf);

#endif // _APPLICATION_H_
