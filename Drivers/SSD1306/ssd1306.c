#include "ssd1306.h"
#include <stdint.h>

static const uint8_t ssd1306InitCmd[] =
{
    0xae,
    0x00, 0x10,
    0x40,
    0x20, 0x01,	// Vertical addressing
    0xb0,
    0x81, 0xcf,
    0xa1,
    0xa6,
    0xa8, 0x3f,
    0xc0,				// Vertical flip
    0xd3, 0x00,
    0xd5, 0x80,
    0xd9, 0xf1,
    0xda, 0x12,
    0xdb, 0x30,
    0x8d, 0x14,
    0xaf
};

void SSD1306_Transmit(uint8_t* buffer, uint32_t size)
{
	for (uint32_t i = 0; i < size; ++i)
		SSD1306_TRANSMIT_BYTE(buffer[i]);
}

void SSD1306_Clear()
{
    SSD1306_DATA();
    for (uint16_t i = 0; i < 1024; ++i)
    	SSD1306_TRANSMIT_BYTE(0x0);
}

void SSD1306_Init()
{
    SSD1306_DISABLE();
    LL_mDelay(1);
    SSD1306_ENABLE();
    LL_mDelay(1);

    SSD1306_CMD();
    SSD1306_Transmit((void*)ssd1306InitCmd, sizeof(ssd1306InitCmd));

    LL_mDelay(1);
    SSD1306_Clear();
}
