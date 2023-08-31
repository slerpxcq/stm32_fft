#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "ssd1362.h"
#include "main.h"
#include "arm_math.h"
#include "tables.h"

#define BAR_FALL_SPEED (1U << 10)
#define DOT_FALL_SPEED (1U << 7)
#define DOT_TTL 64
#define FFT_SIZE 1024	// If you change this, everything will be broken...
#define DC_BIAS 2052
#define RESULT_FLOOR 400000000U
#define RESULT_SCALE 6U
#define INTERP_COUNT 30U

void UpdateScreen(int32_t* buf);

#endif // _APPLICATION_H_
