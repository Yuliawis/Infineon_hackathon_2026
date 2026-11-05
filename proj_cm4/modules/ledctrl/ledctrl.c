
#include "ledctrl.h"
#include "cybsp.h"


// Blanking frame (all zeros) because of how SCB SPI works.
static uint8_t blankingFrame = 0x00;

static void Leds_SendData(void* data, uint16_t len);

// This is a buffer with raw data prepared for SPI transmission to WS2812 LEDs
// Not made static in case you want to control it manually (not recommended, but you can modify the code for that)
uint8_t Leds_rawColorBuffer[WS2812_BUFFER_SIZE] = {0,};

cy_en_scb_spi_status_t Leds_Init(void)
{
    cy_en_scb_spi_status_t initStatus;
    
    /* Configure component */
    initStatus = Cy_SCB_SPI_Init(SPI_LEDCTRL_HW, &SPI_LEDCTRL_config, NULL);
    if(initStatus == CY_SCB_SPI_SUCCESS)
    {
        /* Enable SPI master hardware. */
        Cy_SCB_SPI_Enable(SPI_LEDCTRL_HW);
    }

    Leds_FillSolidColor(0, 0, 0);
    Leds_Update();
    
    return initStatus;
}

// Send data array to LED strip.
static void Leds_SendData(void* data, uint16_t len)
{
    // Perform blanking so WS2812 would think that RESET state occured
    Cy_SCB_SPI_WriteArrayBlocking(SPI_LEDCTRL_HW, &blankingFrame, 1);
    // Send actual data
    Cy_SCB_SPI_WriteArrayBlocking(SPI_LEDCTRL_HW, data, len);
    // Perform blanking so WS2812 would think that RESET state occured (lock colors) 
    Cy_SCB_SPI_WriteArrayBlocking(SPI_LEDCTRL_HW, &blankingFrame, 1);
}

// Convert GRB 8 bit components to SPI-friendly data for transmission
static void Leds_ColorToRawBitstream(uint8_t g, uint8_t r, uint8_t b, uint8_t* out_data)
{
	uint32_t color = (g << 16) | (r << 8) | b;
	uint16_t idx = 0;

	for (int16_t i = 23; i >= 0; i--)
	{
        // If original color is 1...
		if (((color >> i) & 0x01) == 1)
        {
            out_data[idx++] = 0b110;  // ...store 1
        }
		else
        {
            out_data[idx++] = 0b100;  // ...store 0
        }
	}
}

void Leds_PutPixel(uint8_t pix_no, uint8_t g, uint8_t r, uint8_t b)
{    
    if (pix_no < WS2812_LED_QTY)
    {
        Leds_ColorToRawBitstream(g, r, b, &Leds_rawColorBuffer[24 * pix_no]);
    }
}

void Leds_FillSolidColor(uint8_t g, uint8_t r, uint8_t b)
{    
    for (uint16_t i = 0; i < WS2812_LED_QTY; i++)
    {
        Leds_ColorToRawBitstream(g, r, b, &Leds_rawColorBuffer[24 * i]);
    }
}

void Leds_Update(void)
{
    Leds_SendData(&Leds_rawColorBuffer[0], WS2812_BUFFER_SIZE);
}


/* [] END OF FILE */
