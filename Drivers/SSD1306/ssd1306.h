#ifndef _SSD1306_H_
#define _SSD1306_H_

#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_gpio.h"
#include "main.h"

#define SSD1306_SPI SPI1

#define SSD1306_CMD() LL_GPIO_ResetOutputPin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin)
#define SSD1306_DATA() LL_GPIO_SetOutputPin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin)
#define SSD1306_DISABLE() LL_GPIO_ResetOutputPin(SSD1306_RES_GPIO_Port, SSD1306_RES_Pin)
#define SSD1306_ENABLE() LL_GPIO_SetOutputPin(SSD1306_RES_GPIO_Port, SSD1306_RES_Pin)

#define SSD1306_TRANSMIT_BYTE(x) do { \
	LL_SPI_TransmitData8(SSD1306_SPI, (x)); \
	while (!LL_SPI_IsActiveFlag_TXE(SSD1306_SPI)); \
} while (0)

void SSD1306_Transmit(uint8_t* buffer, uint32_t size);
void SSD1306_Clear();
void SSD1306_Init();

#endif // _SSD1306_H_
