#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "ssd1362.h"
#include "main.h"
#include "arm_math.h"
#include "tables.h"

#define BAR_FALL_SPEED (1U << 13)
#define DOT_FALL_SPEED (1U << 7)
#define DOT_TTL 64
#define FFT_SIZE 1024
#define DC_BIAS 2130
#define RESULT_BIAS 350000000U
#define RESULT_SCALE 6U
#define INTERP_COUNT 30U
#define FILL_GRADIENT 0

void UpdateDisplayBuffer(int32_t* buf);

#endif // _APPLICATION_H_
