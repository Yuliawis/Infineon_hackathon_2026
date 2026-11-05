#ifndef LEDCTRL_H
#define LEDCTRL_H

#include <stdint.h>
#include "cy_scb_spi.h"

/* *****************************************************************************************************
    For this WS2812 LED driver to work, a clever hack was by people of the Internet.
    We can use SPI clocked at 2.5 MBit per second to emulate 1.2 uS period (see WS2812 datasheet).
    This allows us send either 110h as HIGH level bit or 011h as low level.
    This is a tradeoff (3 actual SPI clocks for 1 real color bit), but it is a fast and simple solution.
    You can also use PWM + DMA, but this would involve heavy work with double buffering and Interrupts.
***************************************************************************************************** */

// Car has 12 WS2812 LEDs connected
#define WS2812_LED_QTY 12u

// RGB leds operate with 3 colors (obviously)
#define WS2812_COLORS_QTY 3u

// Size of buffer is LED amount times 24. Simulates 24 bits but with bytes because we use SPI hacks
#define WS2812_BUFFER_SIZE (WS2812_LED_QTY * 24)

// This structure must be aligned by 1 byte.
// [Unused] But you can be creative if you want
#pragma pack(push, 1)
struct ledColor {
    // This is an order of colors in WS2812 (and some other LED/Displays etc.)
    uint8_t g;
    uint8_t r;
    uint8_t b;
};
#pragma pack(pop)

// This is a buffer with raw data prepared for SPI transmission to WS2812 LEDs
// Not made static in case you want to control it manually (not recommended, but you can modify the code for that)
extern uint8_t Leds_rawColorBuffer[WS2812_BUFFER_SIZE];

/*******************************************************************************
* Function Name: Leds_Init()
********************************************************************************
* Summary:
*    Initialize and enable Master SPI to send data to WS2812 LED.
*
*******************************************************************************/
cy_en_scb_spi_status_t Leds_Init(void);

/*******************************************************************************
* Function Name: Leds_PutPixel()
********************************************************************************
* Summary:
*    Set the color of one single pixel.
*
* Parameters:
*   pix_no: WS2812 LED idx from 0 to WS2812_LED_QTY - 1
*   g, r, b: 8 bit colors
*
*******************************************************************************/
void Leds_PutPixel(uint8_t pix_no, uint8_t g, uint8_t r, uint8_t b);


/*******************************************************************************
* Function Name: Leds_FillSolidColor()
********************************************************************************
* Summary:
*    Set the color of whole LED strip.
*
* Parameters:
*   g, r, b: 8 bit colors
*
*******************************************************************************/
void Leds_FillSolidColor(uint8_t g, uint8_t r, uint8_t b);

/*******************************************************************************
* Function Name: Leds_Update()
********************************************************************************
* Summary:
*    Display WS2812 buffer contents on an actual string.
*
*******************************************************************************/
void Leds_Update(void);

#endif /* LEDCTRL_H */

/* [] END OF FILE */
