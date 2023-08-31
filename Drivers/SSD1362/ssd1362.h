#ifndef _SSD13XX_H_
#define _SSD13XX_H_

#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_gpio.h"
#include "main.h"

#define SSD1362_SPI 	SPI1

#define SSD1362_COMS 		64U
#define SSD1362_SEGS		256U

#define SSD1362_CMD() 		LL_GPIO_ResetOutputPin(SSD1362_DC_GPIO_Port, SSD1362_DC_Pin)
#define SSD1362_DATA() 		LL_GPIO_SetOutputPin(SSD1362_DC_GPIO_Port, SSD1362_DC_Pin)
#define SSD1362_DISABLE() LL_GPIO_ResetOutputPin(SSD1362_RES_GPIO_Port, SSD1362_RES_Pin)
#define SSD1362_ENABLE() 	LL_GPIO_SetOutputPin(SSD1362_RES_GPIO_Port, SSD1362_RES_Pin)

#define SSD1362_TRANSMIT_BYTE(x) do { \
	LL_SPI_TransmitData8(SSD1362_SPI, (x)); \
	while (!LL_SPI_IsActiveFlag_TXE(SSD1362_SPI)); \
} while (0)

void SSD1362_Transmit(uint8_t* buffer, uint32_t size);
void SSD1362_Clear();
void SSD1362_Init();

#endif // _SSD13XX_H_
