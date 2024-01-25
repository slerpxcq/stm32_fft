#include "application.h"
#include "math_utils.h"
#include "spline_q31.h"

arm_rfft_instance_q31 rfftInstance;
uint8_t dispBuf[SSD1362_SEGS / 2][SSD1362_COMS];
static uint8_t isMemoryUsing = 0;

static void* StaticMalloc()
{
	if (isMemoryUsing)
		Error_Handler();
	isMemoryUsing = 1;
	return dispBuf;
}

static void StaticFree(void* mem)
{
	(void)mem;
	isMemoryUsing = 0;
}

static void fast_mag_q31(int32_t* pSrc, int32_t* pDst, uint32_t blockSize)
{
   for (uint32_t i = 0; i < blockSize; ++i)
   {
     int32_t absRe = abs_q31(pSrc[i<<1]);
     int32_t absIm = abs_q31(pSrc[(i<<1)+1]);

     int32_t max = max_q31(absRe, absIm);
     int32_t min = min_q31(absRe, absIm);

     pDst[i] = max + ((3*min) >> 3);
   }
}

#if LOG_SCALE
#if (INTERP_METHOD == INTERP_METHOD_LINEAR || INTERP_METHOD == INTERP_METHOD_TABULAR)
static void NaiveInterpolate(int32_t* height)
{
  for (int32_t i = 1; i < INTERP_COUNT; ++i)
  {
  	int32_t x0 = jumpPoints[i-1];
  	int32_t x1 = jumpPoints[i];
    int32_t y0 = height[x0];
		int32_t y1 = height[x1];
#if (INTERP_METHOD == INTERP_METHOD_LINEAR)
		int32_t slope = (y1 - y0) / (x1 - x0);
#endif
		for (int32_t x = x0 + 1; x < x1; ++x)
		{
			// Linear
#if (INTERP_METHOD == INTERP_METHOD_LINEAR)
			height[x] = Lerp(x, x0, y0, slope);
#else

			// Tabular
			int32_t tabIdx = ((x - x0) * 32) / (x1 - x0);
			int32_t tabVal = interpTable[tabIdx];

			int32_t tmp = y1 - y0;
			arm_mult_q31(&tmp, &tabVal, &tmp, 1);
			tmp += y0;

			height[x] = tmp;
#endif // (INTERP_METHOD == INTERP_METHOD_LINEAR)
		}
  }
}
#else
static void CubicSplineInterpolate(int32_t* height)
{
	int32_t knownX[INTERP_COUNT];
  int32_t knownY[INTERP_COUNT];

	for (int32_t i = 0; i < INTERP_COUNT; ++i)
	{
		knownX[i] = q_from_int(jumpPoints[i]);
		knownY[i] = height[jumpPoints[i]];
	}

	arm_shift_q31(knownY, -INTERP_SHIFT, knownY, INTERP_COUNT);
	arm_spline_instance_q31 splineInstance;
	static int32_t coeffs[3 * (INTERP_COUNT - 1)];
	static int32_t tmpBuf[2 * INTERP_COUNT - 1];
	arm_spline_init_q31(&splineInstance, ARM_SPLINE_NATURAL, knownX, knownY, INTERP_COUNT, coeffs, tmpBuf);
	arm_spline_q31(&splineInstance, XTableQ, height, INTERP_END);
	arm_shift_q31(height, INTERP_SHIFT, height, INTERP_END);
}
#endif // (INTERP_METHOD == INTERP_METHOD_LINEAR || INTERP_METHOD == INTERP_METHOD_TABULAR)
#endif // LOG_SCALE

static void PreprocessSamples(int32_t* fftResult)
{
	// Subtract DC bias and align to left
	int32_t mean;
	arm_mean_q31(fftResult, FFT_SIZE, &mean);
	arm_offset_q31(fftResult, -mean, fftResult, FFT_SIZE);
	arm_shift_q31(fftResult, 19, fftResult, FFT_SIZE);

	// Window function
	arm_mult_q31(fftResult, blackmanHarris1024, fftResult, FFT_SIZE);
}

static void DoFFTAndGetMagnitude(int32_t* fftResult)
{
	int32_t* fftTemp = (int32_t*)StaticMalloc();
	arm_rfft_q31(&rfftInstance, fftResult, fftTemp);

	// Take magnitude and scale up
	fast_mag_q31(fftTemp, fftResult, FFT_SIZE / 2);
	arm_shift_q31(fftResult, 6, fftResult, FFT_SIZE / 2);
	StaticFree(fftTemp);
}

static void MapToLogarithmicScale(int32_t* fftResult, int32_t* currHeight)
{
	for (int32_t i = 0; i < SSD1362_SEGS; ++i)
	{
#if LOG_SCALE
		currHeight[i] = fftResult[logScale[i]];
#else
		currHeight[i] = fftResult[i];
#endif // LOG_SCALE
	}

	// Take log, shift, scale, clamp
	arm_vlog_q31(currHeight, currHeight, SSD1362_SEGS);
	arm_offset_q31(currHeight, LOG_RESULT_OFFSET, currHeight, SSD1362_SEGS);
	arm_clip_q31(currHeight, currHeight, 0, INT32_MAX, SSD1362_SEGS);
	arm_scale_q31(currHeight, LOG_RESULT_SCALE << 28, 3, currHeight, SSD1362_SEGS);
}

static void UpdateBarAndDotHeight(int32_t* currHeight, int16_t* barHeight, int16_t* dotHeight, uint8_t* dotTTL)
{
	for (int32_t i = 0; i < SSD1362_SEGS; ++i)
	{
		currHeight[i] >>= 16;
		barHeight[i] = max_q31(barHeight[i], (int16_t)currHeight[i]);

		if (dotHeight[i] < barHeight[i])
		{
			dotHeight[i] = barHeight[i];
			dotTTL[i] = DOT_TTL;
		}
	}
}

static void UpdateDisplayFromHeights(int16_t* barHeight, int16_t* dotHeight)
{
	for (uint32_t i = 0; i < SSD1362_SEGS / 2; ++i)
	{
		// Remap bar and dot height from [0, 2^15) to [0, 64), shift right by 9 bits
		int16_t barH0 = barHeight[(i<<1)] >> 9;
		int16_t dotH0 = dotHeight[(i<<1)] >> 9;
		int16_t barH1 = barHeight[(i<<1)+1] >> 9;
		int16_t dotH1 = dotHeight[(i<<1)+1] >> 9;

		for (uint32_t j = 0; j < SSD1362_COMS; ++j, --barH0, --barH1, --dotH0, --dotH1)
		{
			uint8_t tmp = 0;

			tmp |= (barH0 > FADE_SIZE) ? (BAR_COLOR << 4) : (barH0 >= 0 && barH0 <= FADE_SIZE) ? ((BAR_COLOR-(FADE_SIZE-barH0)-1) << 4) : 0;
			tmp	|= (barH1 > FADE_SIZE) ? (BAR_COLOR) : (barH1 >= 0 && barH1 <= FADE_SIZE) ? ((BAR_COLOR-(FADE_SIZE-barH1)-1)) : 0;
			tmp |= (dotH0 == 0) ? (DOT_COLOR << 4) : 0;
			tmp |= (dotH1 == 0) ? DOT_COLOR : 0;

			dispBuf[i][j] = tmp;
		}
	}
}

static void UpdateFalling(int16_t* barHeight, int16_t* dotHeight, uint8_t* dotTTL)
{
	for (int32_t i = 0; i < SSD1362_SEGS; ++i)
	{
		int16_t barFallSpeed = BAR_FALL_SPEED * ((exp_q29(barHeight[i]) - ONE_Q29) >> 3);
		barHeight[i] = max_q31(barHeight[i] - barFallSpeed, 0);

		if (dotTTL[i] == 0)
		{
			dotHeight[i] = max_q31(dotHeight[i] - DOT_FALL_SPEED, DOT_FLOOR);
		}
		else
		{
			--dotTTL[i];
		}
	}
}

void DoFFTAndUpdateDisplay(int32_t* fftResult)
{
  static int16_t barHeight[SSD1362_SEGS];
  static int16_t dotHeight[SSD1362_SEGS];
  static uint8_t dotTTL[SSD1362_SEGS];

  PreprocessSamples(fftResult);
  DoFFTAndGetMagnitude(fftResult);

  int32_t* currHeight = (int32_t*)StaticMalloc();
  MapToLogarithmicScale(fftResult, currHeight);

#if LOG_SCALE
#if (INTERP_METHOD == INTERP_METHOD_SPLINE)
  CubicSplineInterpolate(currHeight);
#else
  NaiveInterpolate(currHeight);
#endif // (INTERP_METHOD == INTERP_METHOD_SPLINE)
#endif // LOG_SCALE

  UpdateBarAndDotHeight(currHeight, barHeight, dotHeight, dotTTL);
  UpdateDisplayFromHeights(barHeight, dotHeight);
  UpdateFalling(barHeight, dotHeight, dotTTL);

  StaticFree(currHeight);
}
