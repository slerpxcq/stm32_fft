#include "application.h"
#include "math_utils.h"

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

static void FastMagnitude(int32_t* pSrc, int32_t* pDst, uint32_t blockSize)
{
   for (uint32_t i = 0; i < blockSize; ++i)
   {
     int32_t absRe = Abs(pSrc[2*i]);
     int32_t absIm = Abs(pSrc[2*i+1]);

     int32_t max = Max(absRe, absIm);
     int32_t min = Min(absRe, absIm);

     pDst[i] = max + ((3*min) >> 3);
   }
}

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

#else // (INTERP_METHOD == INTERP_METHOD_LINEAR || INTERP_METHOD == INTERP_METHOD_TABULAR)
static void IntToFloatInPlace(int32_t* inout, uint32_t blockSize)
{
  for (uint32_t i = 0; i < blockSize; ++i)
  {
  	union {
  		int32_t i;
			float f;
  	} u;

    u.f = (float32_t)inout[i];
    inout[i] = u.i;
  }
}

static void FloatToIntInPlace(int32_t* inout, uint32_t blockSize)
{
  for (uint32_t i = 0; i < blockSize; ++i)
  {
    int32_t tmp;
    tmp = (int32_t)(*(float32_t*)&inout[i]);
    inout[i] = tmp;
  }
}

static void CubicSplineInterpolate(int32_t* height)
{
  int32_t knownX[INTERP_COUNT], knownY[INTERP_COUNT];

  // Find all jump points
  for (int32_t i = 0; i < INTERP_COUNT; ++i)
  {
  	knownX[i] = jumpPoints[i];
  	knownY[i] = height[jumpPoints[i]];
  }

  int32_t interpEnd = knownX[INTERP_COUNT - 1];

  IntToFloatInPlace(knownX, INTERP_COUNT);
  IntToFloatInPlace(knownY, INTERP_COUNT);

  static arm_spline_instance_f32 splineInstance;
  static float32_t coeffs[3 * (INTERP_COUNT - 1)];
  static float32_t tmpBuf[2 * INTERP_COUNT - 1];

  arm_spline_init_f32(&splineInstance, ARM_SPLINE_NATURAL,
      (float32_t*)knownX, (float32_t*)knownY, INTERP_COUNT, coeffs, tmpBuf);
  IntToFloatInPlace(height, interpEnd);
  arm_spline_f32(&splineInstance, linearX, (float32_t*)height, interpEnd);
  FloatToIntInPlace(height, interpEnd);
}
#endif // (INTERP_METHOD == INTERP_METHOD_SPLINE)

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
	FastMagnitude(fftTemp, fftResult, FFT_SIZE / 2);
	arm_shift_q31(fftResult, 6, fftResult, FFT_SIZE / 2);
	StaticFree(fftTemp);
}

static void MapToLogarithmicScale(int32_t* fftResult, int32_t* currHeight)
{
	// Logarithmic remapping; here fftTemp is reused, again, due to memory limitation
#if LOG_SCALE
	for (int32_t i = 0; i < SSD1362_SEGS; ++i)
		currHeight[i] = fftResult[logScale[i]];
#endif

	// Take log, shift, scale, clamp
	arm_vlog_q31(currHeight, currHeight, SSD1362_SEGS);

	for (uint32_t i = 0; i < SSD1362_SEGS; ++i)
	{
		int32_t tmp = currHeight[i];

		tmp += RESULT_BIAS;
		tmp = Max(0, tmp);
		tmp *= RESULT_SCALE;

		currHeight[i] = tmp;
	}
}

static void UpdateBarAndDotHeight(int32_t* currHeight, int16_t* barHeight, int16_t* dotHeight, uint8_t* dotTTL)
{
	for (int32_t i = 0; i < SSD1362_SEGS; ++i)
	  {
	  	currHeight[i] >>= 16;
	    barHeight[i] = Max(barHeight[i], (int16_t)currHeight[i]);
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
		int16_t barH0 = barHeight[2*i] >> 9;
		int16_t dotH0 = dotHeight[2*i] >> 9;
		int16_t barH1 = barHeight[2*i+1] >> 9;
		int16_t dotH1 = dotHeight[2*i+1] >> 9;

		for (uint32_t j = 0; j < SSD1362_COMS; ++j)
		{
			dispBuf[i][j] = 0;
			// Uniform fill
#if !FILL_GRADIENT
			dispBuf[i][j] |= (barH0 > 0) ? 0xf0 : 0;
			dispBuf[i][j]	|= (barH1 > 0) ? 0x0f : 0;

#else
			// Gradient fill
			dispBuf[i][j] |= (barH0 < 16 && barH0 > 0) ? ((16 - barH0) << 4) : 0;
			dispBuf[i][j] |= (barH1 < 16 && barH1 > 0) ? (16 - barH0) : 0;
#endif
			dispBuf[i][j] |= (dotH0 == 0) ? 0xf0 : 0;
			dispBuf[i][j] |= (dotH1 == 0) ? 0x0f : 0;
			--barH0;
			--dotH0;
			--barH1;
			--dotH1;
		}
	}
}

static void UpdateFalling(int16_t* barHeight, int16_t* dotHeight, uint8_t* dotTTL)
{
	for (int32_t i = 0; i < SSD1362_SEGS; ++i)
	{
		float barFallSpeed = BAR_FALL_SPEED * (expf(barHeight[i] * (1.f / UINT16_MAX)) - 1.f);
		barHeight[i] = Max(barHeight[i] - (int32_t)barFallSpeed, 0);

		if (dotTTL[i] == 0)
		{
			dotHeight[i] = Max(dotHeight[i] - DOT_FALL_SPEED, (1U << 8));
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
  //
#else
  NaiveInterpolate(currHeight);
#endif // (INTERP_METHOD == INTERP_METHOD_SPLINE)
#endif // LOG_SCALE

  UpdateBarAndDotHeight(currHeight, barHeight, dotHeight, dotTTL);
  UpdateDisplayFromHeights(barHeight, dotHeight);
  UpdateFalling(barHeight, dotHeight, dotTTL);

  StaticFree(currHeight);
}
