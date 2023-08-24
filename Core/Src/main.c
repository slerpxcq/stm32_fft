/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "tables.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BAR_FALL_SPEED (1U << 26)
#define DOT_FALL_SPEED (1U << 23)
#define DOT_TTL 64
#define FFT_SIZE 1024
#define DC_BIAS 2052
#define RESULT_FLOOR 400000000
#define RESULT_SCALE 4
#define INTERP_COUNT 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t sampleAvail;
volatile uint8_t processCplt;

static uint8_t wrIdx;
static uint8_t rdIdx;
static int32_t fftInOut[2][FFT_SIZE];
static uint8_t dispBuf[128][8];

static arm_rfft_instance_q31 rfftInstance;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void UpdateScreen();
static void FastMag(int32_t* pSrc, int32_t* pDst, uint32_t blockSize);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  LL_SPI_Enable(SPI1);
  SSD1306_Init();

  // ADC DMA
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)fftInOut[0]);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, FFT_SIZE);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  // SPI DMA
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&SPI1->DR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)dispBuf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 1024);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

  LL_ADC_Enable(ADC1);
  LL_ADC_StartCalibration(ADC1);
  while (LL_ADC_IsCalibrationOnGoing(ADC1));
  //LL_ADC_EnableIT_EOS(ADC1);

  LL_TIM_EnableCounter(TIM3);
  LL_TIM_EnableUpdateEvent(TIM3);

  arm_rfft_init_q31(&rfftInstance, FFT_SIZE, 0, 1);

  sampleAvail = 0;
  processCplt = 1;
  wrIdx = 0;
  rdIdx = 1;
  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if (sampleAvail && processCplt)
  	{
  		sampleAvail = 0;
  		processCplt = 0;

  		// Swap buffers
  		wrIdx = !wrIdx;
  		rdIdx = !rdIdx;

  		// RM0008 13.4.6
  		// This register must not be written when the channel is enabled.
  		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  		LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)fftInOut[wrIdx]);
  		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  		LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

  		LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  		int32_t* rdBuf = fftInOut[rdIdx];
			UpdateScreen(rdBuf);

			LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA1   ------> ADC1_IN1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_7CYCLES_5);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, SSD1306_RES_Pin|SSD1306_DC_Pin);

  /**/
  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  GPIO_InitStruct.Pin = SSD1306_RES_Pin|SSD1306_DC_Pin|LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void FastMag(int32_t* pSrc, int32_t* pDst, uint32_t blockSize)
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

static void SinInterp(int32_t* height)
{
	int32_t x0 = 0, x1 = 0;
	for (int32_t i = 1; i < 128; ++i)
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

static void CubicInterp(int32_t* height)
{
	int32_t knownX[INTERP_COUNT], knownY[INTERP_COUNT];
	uint8_t j = 0;

	// Find all jump points
	for (int32_t i = 1; i < 128; ++i)
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

	int32_t outX[64];
	for (uint32_t i = 0; i < interpEnd; ++i)
		outX[i] = i;

	I32ToF32(outX, interpEnd);
	I32ToF32(knownX, INTERP_COUNT);
	I32ToF32(knownY, INTERP_COUNT);

	static arm_spline_instance_f32 splineInstance;
	static float32_t coeffs[3 * (INTERP_COUNT - 1)];
	static float32_t tmpBuf[2 * INTERP_COUNT - 1];

	arm_spline_init_f32(&splineInstance, ARM_SPLINE_NATURAL,
			(float32_t*)knownX, (float32_t*)knownY, INTERP_COUNT, coeffs, tmpBuf);
	I32ToF32(height, interpEnd);
	arm_spline_f32(&splineInstance, (float32_t*)outX, (float32_t*)height, interpEnd);
	F32ToI32(height, interpEnd);
}

static void UpdateScreen(int32_t* buf)
{
	static const uint8_t barMap[] = { 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff };
	static const uint8_t dotMap[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };

	static int32_t fftTemp[FFT_SIZE * 2];
	static int32_t barHeight[128];
	static int32_t dotHeight[128];
	static uint8_t dotTTL[128];

	// Subtract DC bias and align to left
	for (uint32_t i = 0; i < FFT_SIZE; ++i)
		buf[i] = (buf[i] - DC_BIAS) << 19;

	// Apply window function
	arm_mult_q31(buf, blackmanHarris1024, buf, FFT_SIZE);

	// Do FFT
	arm_rfft_q31(&rfftInstance, buf, fftTemp);

	// Take magnitude and scale up
	FastMag(fftTemp, buf, FFT_SIZE / 2);
	for (uint32_t i = 0; i < FFT_SIZE / 2; ++i)
		buf[i] <<= 5;

	// Logarithmic remapping; here fftTemp is reused due to memory limitation
	int32_t* currHeight = fftTemp;
	for (int32_t i = 0; i < 128; ++i)
		currHeight[i] = buf[expMap[i]];

	// Take log, shift, scale
	arm_vlog_q31(currHeight, currHeight, 128);

	for (uint32_t i = 0; i < 128; ++i)
	{
		int32_t tmp = currHeight[i];

		tmp += RESULT_FLOOR;
		tmp = Max(0, tmp);
		tmp *= RESULT_SCALE;

		currHeight[i] = tmp;
	}

	// Interpolation
	 CubicInterp(currHeight);
//	SinInterp(currHeight);

	// Update bar height, dot height and TTL
	for (int32_t i = 0; i < 128; ++i)
	{
		barHeight[i] = Max(barHeight[i], currHeight[i]);
		if (dotHeight[i] < barHeight[i])
		{
			dotHeight[i] = barHeight[i];
			dotTTL[i] = DOT_TTL;
		}
	}

	// Update display buffer
	for (int32_t i = 0; i < 128; ++i)
	{
		int32_t barH = barHeight[i] >> 25;
		int32_t dotH = dotHeight[i] >> 25;

		for (int32_t j = 0; j < 8; ++j)
		{
			dispBuf[i][j] = (barH > 8) ? 0xff : (barH < 1) ? 0x00 : barMap[barH - 1];
			dispBuf[i][j] |= (dotH > 8 || dotH < 1) ? 0x00 : dotMap[dotH - 1];
			barH -= 8;
			dotH -= 8;
		}
	}

	// Holding and smooth falling
	for (int32_t i = 0; i < 128; ++i)
	{
		barHeight[i] = Max(barHeight[i] - BAR_FALL_SPEED, (1 << 25));

		if (dotTTL[i] == 0)
		{
			dotHeight[i] = Max(dotHeight[i] - DOT_FALL_SPEED, (1 << 25));
		}
		else
		{
			--dotTTL[i];
		}
	}

	LL_SPI_EnableDMAReq_TX(SPI1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
