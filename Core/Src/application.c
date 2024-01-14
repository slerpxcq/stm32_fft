#include "application.h"

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

arm_rfft_instance_q31 rfftInstance;
uint8_t dispBuf[SSD1362_SEGS / 2][SSD1362_COMS];

static void FastMagnitude(int32_t* pSrc, int32_t* pDst, uint32_t blockSize)
{
   for (uint32_t i = 0; i < blockSize; ++i)
   {
     int32_t absRe = Abs(pSrc[2 * i]);
     int32_t absIm = Abs(pSrc[2 * i + 1]);

     int32_t max = Max(absRe, absIm);
     int32_t min = Min(absRe, absIm);

     pDst[i] = max + ((3 * min) >> 3);
   }
}

#if 0
static void SinInterp(int32_t* height)
{
  int32_t x0 = 0, x1 = 0;
  for (int32_t i = 1; i < SSD1362_COLS; ++i)
  {
    if (expMap[i] != expMap[i-1])
      x1 = i;

    int32_t y0 = height[x0], y1 = height[x1];

    if (x1 - x0 > 1)
    {
      // int32_t slope = (y1 - y0) / (x1 - x0);
      for (int32_t x = x0 + 1; x < x1; ++x)
      {
        // Linear
        // height[x] = Lerp(x, x0, y0, slope);

        // Tabular
        int32_t tabIdx = ((x - x0) * 32) / (x1 - x0);
        int32_t tabVal = sinInterp[tabIdx];

        int32_t tmp = y1 - y0;
        arm_mult_q31(&tmp, &tabVal, &tmp, 1);
        tmp += y0;

        height[x] = tmp;
      }
    }

    x0 = x1;
  }
}
#endif

static void I32ToF32(int32_t* inout, uint32_t blockSize)
{
  for (uint32_t i = 0; i < blockSize; ++i)
  {
    float32_t tmp;
    tmp = (float32_t)inout[i];
    inout[i] = *(int32_t*)&tmp;
  }
}

static void F32ToI32(int32_t* inout, uint32_t blockSize)
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
  uint8_t j = 0;

  // Find all jump points
  for (int32_t i = 1; i < SSD1362_SEGS; ++i)
  {
    if (expMap[i] != expMap[i-1])
    {
      knownX[j] = i;
      knownY[j] = height[i];
      ++j;

      if (j == INTERP_COUNT)
        break;
    }
  }

  int32_t interpEnd = knownX[INTERP_COUNT - 1];

  I32ToF32(knownX, INTERP_COUNT);
  I32ToF32(knownY, INTERP_COUNT);

  static arm_spline_instance_f32 splineInstance;
  static float32_t coeffs[3 * (INTERP_COUNT - 1)];
  static float32_t tmpBuf[2 * INTERP_COUNT - 1];

  arm_spline_init_f32(&splineInstance, ARM_SPLINE_NATURAL,
      (float32_t*)knownX, (float32_t*)knownY, INTERP_COUNT, coeffs, tmpBuf);
  I32ToF32(height, interpEnd);
  arm_spline_f32(&splineInstance, linearX, (float32_t*)height, interpEnd);
  F32ToI32(height, interpEnd);
}

void FFTResultToHeight(int32_t* fftResult, int32_t* currHeight)
{
	// Subtract DC bias and align to left
	int32_t mean;
	arm_mean_q31(fftResult, FFT_SIZE, &mean);
	arm_offset_q31(fftResult, -mean, fftResult, FFT_SIZE);
	arm_shift_q31(fftResult, 19, fftResult, FFT_SIZE);

	// Window function
	arm_mult_q31(fftResult, blackmanHarris1024, fftResult, FFT_SIZE);

	// Do FFT
	// Reuse display buffer due to memory limitation
	// Note that display buffer happened to have the same size required by fftTemp, what a coincidence...
	int32_t* fftTemp = (int32_t*)dispBuf;
	arm_rfft_q31(&rfftInstance, fftResult, fftTemp);

	// Take magnitude and scale up
	FastMagnitude(fftTemp, fftResult, FFT_SIZE / 2);
	arm_shift_q31(fftResult, 5, fftResult, FFT_SIZE / 2);

	// Logarithmic remapping; here fftTemp is reused, again, due to memory limitation
	for (int32_t i = 0; i < SSD1362_SEGS; ++i)
		currHeight[i] = fftResult[expMap[i]];

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

void UpdateDisplayBuffer(int32_t* fftResult)
{
  static int16_t barHeight[SSD1362_SEGS];
  static int16_t dotHeight[SSD1362_SEGS];
  static uint8_t dotTTL[SSD1362_SEGS];

  int32_t* currHeight = (int32_t*)dispBuf;
  FFTResultToHeight(fftResult, currHeight);

  // Interpolation
   CubicSplineInterpolate(currHeight);
//  SinInterp(currHeight);

  // Update bar height, dot height and TTL
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

  // Update display buffer
  for (uint32_t i = 0; i < SSD1362_SEGS / 2; ++i)
  {
  	// Remap bar and dot height from [0, 2^15) to [0, 64), shift right by 9 bits
  	int32_t barH0 = barHeight[2 * i] >> 9U;
  	int32_t dotH0 = dotHeight[2 * i] >> 9U;
  	int32_t barH1 = barHeight[2 * i + 1] >> 9U;
		int32_t dotH1 = dotHeight[2 * i + 1] >> 9U;

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

  // Holding and smooth falling
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
