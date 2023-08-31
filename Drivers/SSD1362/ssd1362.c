#include "ssd1362.h"

#include <stdint.h>

static const uint8_t ssd1362InitCmd[] =
{
		0xfd, 0x12,					// Command Lock
		0xae,								// Set Display Off
		0xb3, 0xc0,					// Set Display Clock Divide Ratio/Oscillator Frequency
		0xa8, 0x3f,					// Set Multiplex Ratio
		0xab, 0x01,					// Set internal VDD regulator On
		0x15, 0x00, 0x7f,		// Set Column address
		0x75, 0x00, 0x3F,		// Set Row address
		0xA0, 0x55,					// Set Re-Map
		0xA1, 0x00,					// Set Display Start Line
		0xA2, 0x00,					// Set Display Offset
		0xAD, 0x8E,					// Set external IREF
		0x81, 0x66,					// Set Contrast Current
		0xB8,								// Select Gray Scale Table
		0x02,0x09,0x0D,0x14,0x1b,
		0x24,0x2d,0x38,0x45,0x54,
		0x64,0x75,0x88,0x9E,0xb4,
		0xB1, 0xD2,					// Set Phase Length
		0xB5, 0x02,					// Set GPIO
		0xB6, 0x0D,					// Set Second Pre-Charge Period
		0xBC, 0x08,					// Set Pre-Charge Voltage
		0xBD, 0x01,					// Set Pre-Charge Voltage capacitor
		0xBE, 0x07,					// Set VCOMH Deselect Level
		0xA4,								// Set Normal Display Mode
		0xAF								// Set Display On
};

void SSD1362_Transmit(uint8_t* buffer, uint32_t size)
{
	for (uint32_t i = 0; i < size; ++i)
		SSD1362_TRANSMIT_BYTE(buffer[i]);
}

void SSD1362_Clear()
{
    SSD1362_DATA();
    for (uint16_t i = 0; i < SSD1362_COMS * SSD1362_SEGS / 2; ++i)
    	SSD1362_TRANSMIT_BYTE(0x0);
}

void SSD1362_Init()
{
    SSD1362_DISABLE();
    LL_mDelay(1);
    SSD1362_ENABLE();
    LL_mDelay(1);

    SSD1362_CMD();
    SSD1362_Transmit((void*)ssd1362InitCmd, sizeof(ssd1362InitCmd));

    LL_mDelay(1);
    SSD1362_Clear();
}
