/**
 * SSD1306 OLED driver
 * Adaptación para STM32F411CEU6 por Tadeo Mendelevich
 *
 * Este driver proporciona:
 *  - un API agnóstico para inicializar y dibujar en pantalla
 *  - un buffer en memoria para el mapa de bits
 *  - funciones de dibujo de píxeles y relleno
 *  - un refresco no bloqueante por máquina de estados
 *  - abstracción del hardware mediante SSD1306_Platform_t
 *
 * Licencia: GNU GPLv3
 */

#include "ssd1306.h"
#include "main.h"
#include <string.h>
#include <stdarg.h>

//extern void USB_Debug(const char *fmt, ...);

static const SSD1306_Platform_t *pPlat = NULL;
#define WRITE_CMD(c)      pPlat->write_cmd(pPlat->ctx, (c))
#define WRITE_DATA(b,n)   pPlat->write_data(pPlat->ctx, (b), (n))
//#define WRITE_DATA_ASYNC(b,n) pPlat->write_data_async(pPlat->ctx, (b), (n))
#define WRITE_DATA_ASYNC(buf, len)   \
SSD1306_Platform_WriteDataAsync((buf), (len))
#define IS_BUSY()         pPlat->is_busy(pPlat->ctx)
#define DELAY_MS(ms)      pPlat->delay_ms(pPlat->ctx, (ms))

/* Absolute value */
#define ABS(x)   ((x) > 0 ? (x) : -(x))

/* SSD1306 data buffer */
uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static uint8_t ssd_update_done = 1;
static uint8_t ssd_start_update = 0;
static uint8_t ssd_pending_update = 0;

/* Private SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

/* Private variable */
static SSD1306_t SSD1306;


#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define SSD1306_DEACTIVATE_SCROLL                    0x2E // Stop scroll
#define SSD1306_ACTIVATE_SCROLL                      0x2F // Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3 // Set scroll range

#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7

#define SSD1306_COLUMNADDR  0x21  // Set column address
#define SSD1306_PAGEADDR    0x22  // Set page address


void SSD1306_RegisterPlatform(const SSD1306_Platform_t *plat) {
    pPlat = plat;
}

void SSD1306_Platform_Init(void) {
    pPlat->init(pPlat->ctx);
}

void SSD1306_Platform_DelayMs(uint32_t ms) {
    pPlat->delay_ms(pPlat->ctx, ms);
}

void SSD1306_Platform_WriteCommand(uint8_t cmd) {
    pPlat->write_cmd(pPlat->ctx, cmd);
}

void SSD1306_Platform_WriteData(const uint8_t *data, uint16_t len) {
    pPlat->write_data(pPlat->ctx, data, len);
}

uint8_t SSD1306_Platform_IsBusy(void) {
    return pPlat->is_busy(pPlat->ctx);
}

void SSD1306_Platform_WriteDataAsync(const uint8_t* data, uint16_t len) {
    pPlat->write_data_async(pPlat->ctx, data, len);
}


void SSD1306_ScrollRight(uint8_t start_row, uint8_t end_row)
{
  WRITE_CMD (SSD1306_RIGHT_HORIZONTAL_SCROLL);  // send 0x26
  WRITE_CMD (0x00);  // send dummy
  WRITE_CMD(start_row);  // start page address
  WRITE_CMD(0X00);  // time interval 5 frames
  WRITE_CMD(end_row);  // end page address
  WRITE_CMD(0X00);
  WRITE_CMD(0XFF);
  WRITE_CMD (SSD1306_ACTIVATE_SCROLL); // start scroll
}


void SSD1306_ScrollLeft(uint8_t start_row, uint8_t end_row)
{
  WRITE_CMD (SSD1306_LEFT_HORIZONTAL_SCROLL);  // send 0x26
  WRITE_CMD (0x00);  // send dummy
  WRITE_CMD(start_row);  // start page address
  WRITE_CMD(0X00);  // time interval 5 frames
  WRITE_CMD(end_row);  // end page address
  WRITE_CMD(0X00);
  WRITE_CMD(0XFF);
  WRITE_CMD (SSD1306_ACTIVATE_SCROLL); // start scroll
}


void SSD1306_Scrolldiagright(uint8_t start_row, uint8_t end_row)
{
  WRITE_CMD(SSD1306_SET_VERTICAL_SCROLL_AREA);  // sect the area
  WRITE_CMD (0x00);   // write dummy
  WRITE_CMD(SSD1306_HEIGHT);

  WRITE_CMD(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
  WRITE_CMD (0x00);
  WRITE_CMD(start_row);
  WRITE_CMD(0X00);
  WRITE_CMD(end_row);
  WRITE_CMD (0x01);
  WRITE_CMD (SSD1306_ACTIVATE_SCROLL);
}


void SSD1306_Scrolldiagleft(uint8_t start_row, uint8_t end_row)
{
  WRITE_CMD(SSD1306_SET_VERTICAL_SCROLL_AREA);  // sect the area
  WRITE_CMD (0x00);   // write dummy
  WRITE_CMD(SSD1306_HEIGHT);

  WRITE_CMD(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
  WRITE_CMD (0x00);
  WRITE_CMD(start_row);
  WRITE_CMD(0X00);
  WRITE_CMD(end_row);
  WRITE_CMD (0x01);
  WRITE_CMD (SSD1306_ACTIVATE_SCROLL);
}


void SSD1306_Stopscroll(void)
{
	WRITE_CMD(SSD1306_DEACTIVATE_SCROLL);
}



void SSD1306_InvertDisplay (int i)
{
  if (i) WRITE_CMD (SSD1306_INVERTDISPLAY);

  else WRITE_CMD (SSD1306_NORMALDISPLAY);

}


void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color)
{

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++)
    {
        for(int16_t i=0; i<w; i++)
        {
            if(i & 7)
            {
               byte <<= 1;
            }
            else
            {
               byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }
            if(byte & 0x80) SSD1306_DrawPixel(x+i, y, color);
        }
    }
}

uint8_t SSD1306_Init(void) {
	//USB_Debug("SSD1306_Init: Platform init...\r\n");
    SSD1306_Platform_Init();
    SSD1306_Platform_DelayMs(10);

    /* Init LCD */
	WRITE_CMD(0xAE); //display off
	WRITE_CMD(0x20); //Set Memory Addressing Mode
	WRITE_CMD(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WRITE_CMD(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	WRITE_CMD(0xC8); //Set COM Output Scan Direction
	WRITE_CMD(0x00); //---set low column address
	WRITE_CMD(0x10); //---set high column address
	WRITE_CMD(0x40); //--set start line address
	WRITE_CMD(0x81); //--set contrast control register
	WRITE_CMD(0xFF);
	WRITE_CMD(0xA1); //--set segment re-map 0 to 127
	WRITE_CMD(0xA6); //--set normal display
	WRITE_CMD(0xA8); //--set multiplex ratio(1 to 64)
	WRITE_CMD(0x3F); //
	WRITE_CMD(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WRITE_CMD(0xD3); //-set display offset
	WRITE_CMD(0x00); //-not offset
	WRITE_CMD(0xD5); //--set display clock divide ratio/oscillator frequency
	WRITE_CMD(0xF0); //--set divide ratio
	WRITE_CMD(0xD9); //--set pre-charge period
	WRITE_CMD(0x22); //
	WRITE_CMD(0xDA); //--set com pins hardware configuration
	WRITE_CMD(0x12);
	WRITE_CMD(0xDB); //--set vcomh
	WRITE_CMD(0x20); //0x20,0.77xVcc
	WRITE_CMD(0x8D); //--set DC-DC enable
	WRITE_CMD(0x14); //
	WRITE_CMD(0xAF); //--turn on SSD1306 panel

    WRITE_CMD(SSD1306_DEACTIVATE_SCROLL);

    // Limpia el buffer y refresca la pantalla
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_UpdateScreen_Blocking();

    // Inicializa punteros internos
    SSD1306.CurrentX    = 0;
    SSD1306.CurrentY    = 0;
    SSD1306.Inverted    = 0;
    SSD1306.Initialized = 1;


   // USB_Debug("SSD1306_Init: Platform init...\r\n");
    return 1;
}

void SSD1306_UpdateScreen_Blocking(void) {
    for (uint8_t m = 0; m < 8; m++) {
        WRITE_CMD(0xB0 + m);  // seleccionar página m
        WRITE_CMD(0x00);      // columna baja = 0
        WRITE_CMD(0x10);      // columna alta = 0

        WRITE_DATA(&SSD1306_Buffer[SSD1306_WIDTH * m],
                   SSD1306_WIDTH);
    }
}



void SSD1306_UpdateScreen(void) {
    static uint8_t page    = 0;
    static uint8_t state   = 0;  // 0=idle, 1=prep envío, 2=esperar fin DMA
    static uint8_t counter = 0;  // para timeouts

    if (ssd_pending_update && ssd_update_done && !ssd_start_update) {
		ssd_pending_update = 0;
		ssd_start_update   = 1;
	}

    switch (state) {
        case 0:
            if (!ssd_start_update) {
                break;  // sigo idle
            }
            // Inicio refresco
            page            = 0;
            ssd_update_done = 0;
            state           = 1;
            counter         = 0;
            // caída intencional a state=1

        case 1:
            // Estado de preparación: envío comandos + arranque DMA
            if (!SSD1306_Platform_IsBusy()) {
                // Comandos de posición
                WRITE_CMD(0xB0 + page);
                WRITE_CMD(0x00);
                WRITE_CMD(0x10);

                // Arranco la DMA de datos
                uint8_t *buf = &SSD1306_Buffer[SSD1306_WIDTH * page];
                WRITE_DATA_ASYNC(buf, SSD1306_WIDTH);

                state   = 2;
                counter = 0;
            }
            // Timeout si no arranca la transferencia en X ciclos
            else if (++counter >= 700) {
                pPlat->onError(pPlat->ctx, -2);  // -2 = TIMEOUT en preparación
                // abortar refresco
                ssd_update_done  = 1;
                ssd_start_update = 0;
                state            = 0;
            }
            break;

        case 2:
            // Estado de espera: fin de la DMA
        	if (!SSD1306_Platform_IsBusy()) {
				page++;
				if (page < 8) {
					state   = 1;
					counter = 0;
				} else {
					// refresco completo
					ssd_update_done  = 1;
					ssd_start_update = 0;
					state            = 0;
					// si había solicitud pendiente, la arranco
					if (ssd_pending_update) {
						ssd_pending_update = 0;
						ssd_start_update   = 1;
						// caerá en state=1 en la siguiente llamada
					}
				}
			}
            // Timeout si la DMA no completa en X ciclos
            else if (++counter >= 700) {
                pPlat->onError(pPlat->ctx, -3);  // -3 = TIMEOUT DMA
                // abortar refresco
                ssd_update_done  = 1;
                ssd_start_update = 0;
                state            = 0;
            }
            break;
    }
}




void SSD1306_ToggleInvert(void) {
	uint16_t i;

	/* Toggle invert */
	SSD1306.Inverted = !SSD1306.Inverted;

	/* Do memory toggle */
	for (i = 0; i < sizeof(SSD1306_Buffer); i++) {
		SSD1306_Buffer[i] = ~SSD1306_Buffer[i];
	}
}

void SSD1306_Fill(SSD1306_COLOR_t color) {
	/* Set memory */
	memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

void SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}

	/* Check if pixels are inverted */
	if (SSD1306.Inverted) {
		color = (SSD1306_COLOR_t)!color;
	}

	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

void SSD1306_GotoXY(uint16_t x, uint16_t y) {
	/* Set write pointers */
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

char SSD1306_Putc(char ch, const FontDef_t* Font, SSD1306_COLOR_t color) {
	uint32_t i, b, j;

	/* Check available space in LCD */
	if (
		SSD1306_WIDTH <= (SSD1306.CurrentX + Font->FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font->FontHeight)
	) {
		/* Error */
		return 0;
	}

	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t) color);
			} else {
				SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t)!color);
			}
		}
	}

	/* Increase pointer */
	SSD1306.CurrentX += Font->FontWidth;

	/* Return character written */
	return ch;
}

char SSD1306_Puts(const char* str, const FontDef_t* Font, SSD1306_COLOR_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (SSD1306_Putc(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}

		/* Increase string pointer */
		str++;
	}

	/* Everything OK, zero should be returned */
	return *str;
}


void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c) {
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if (x0 >= SSD1306_WIDTH) {
		x0 = SSD1306_WIDTH - 1;
	}
	if (x1 >= SSD1306_WIDTH) {
		x1 = SSD1306_WIDTH - 1;
	}
	if (y0 >= SSD1306_HEIGHT) {
		y0 = SSD1306_HEIGHT - 1;
	}
	if (y1 >= SSD1306_HEIGHT) {
		y1 = SSD1306_HEIGHT - 1;
	}

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			SSD1306_DrawPixel(x0, i, c);
		}

		/* Return from function */
		return;
	}

	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			SSD1306_DrawPixel(i, y0, c);
		}

		/* Return from function */
		return;
	}

	while (1) {
		SSD1306_DrawPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}

void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c) {
	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}

	/* Draw 4 lines */
	SSD1306_DrawLine(x, y, x + w, y, c);         /* Top line */
	SSD1306_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	SSD1306_DrawLine(x, y, x, y + h, c);         /* Left line */
	SSD1306_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

void SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c) {
	uint8_t i;

	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		SSD1306_DrawLine(x, y + i, x + w, y + i, c);
	}
}

void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color) {
	/* Draw lines */
	SSD1306_DrawLine(x1, y1, x2, y2, color);
	SSD1306_DrawLine(x2, y2, x3, y3, color);
	SSD1306_DrawLine(x3, y3, x1, y1, color);
}


void SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color) {
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		SSD1306_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, c);
    SSD1306_DrawPixel(x0, y0 - r, c);
    SSD1306_DrawPixel(x0 + r, y0, c);
    SSD1306_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawPixel(x0 + x, y0 + y, c);
        SSD1306_DrawPixel(x0 - x, y0 + y, c);
        SSD1306_DrawPixel(x0 + x, y0 - y, c);
        SSD1306_DrawPixel(x0 - x, y0 - y, c);

        SSD1306_DrawPixel(x0 + y, y0 + x, c);
        SSD1306_DrawPixel(x0 - y, y0 + x, c);
        SSD1306_DrawPixel(x0 + y, y0 - x, c);
        SSD1306_DrawPixel(x0 - y, y0 - x, c);
    }
}

void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, c);
    SSD1306_DrawPixel(x0, y0 - r, c);
    SSD1306_DrawPixel(x0 + r, y0, c);
    SSD1306_DrawPixel(x0 - r, y0, c);
    SSD1306_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        SSD1306_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        SSD1306_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        SSD1306_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}

void SSD1306_Clear (void)
{
	SSD1306_Fill (0);
	SSD1306_UpdateScreen_Blocking();
}
void SSD1306_ON(void) {
	WRITE_CMD(0x8D);
	WRITE_CMD(0x14);
	WRITE_CMD(0xAF);
}
void SSD1306_OFF(void) {
	WRITE_CMD(0x8D);
	WRITE_CMD(0x10);
	WRITE_CMD(0xAE);
}

/**
 * @brief Solicita un refresco no bloqueante de pantalla.
 */
void SSD1306_RequestUpdate(void) {
    if (ssd_update_done && !ssd_start_update) {
        ssd_start_update = 1;
    } else {
        ssd_pending_update = 1;
    }
}


/**
 * @brief Indica si el refresco NB ha finalizado.
 * @retval 1 si completado, 0 si en curso.
 */
uint8_t SSD1306_IsUpdateDone(void) {
    return ssd_update_done;
}

static int32_t ssd_last_error = 0;

void reportError(int err) {
    ssd_last_error = err;
    if (pPlat->onError) {
        pPlat->onError(pPlat->ctx, err);
    }
}

int32_t SSD1306_GetLastError(void) {
    return ssd_last_error;
}

void SSD1306_ClearError(void) {
    ssd_last_error = 0;
}

/**
 *  @brief  Fuerza el estado “idle” de la máquina de refresco
 */
void SSD1306_ResetUpdateState(void) {
    ssd_update_done    = 1;
    ssd_start_update   = 0;
    ssd_pending_update = 0;
}

void SSD1306_DrawDigit5x7(uint8_t digit, uint16_t x, uint16_t y) {
    if (digit > 9) return;
    // Cada dígito ocupa Font_5x7.FontWidth bytes consecutivos
    const uint8_t *pat = &Font5x7[digit * Font_5x7.FontWidth];
    for (uint8_t col = 0; col < Font_5x7.FontWidth; col++) {
        uint8_t bits = pat[col];
        for (uint8_t row = 0; row < Font_5x7.FontHeight; row++) {
            if (bits & (1 << row)) {
                SSD1306_DrawPixel(x + col, y + row, SSD1306_COLOR_WHITE);
            }
        }
    }
}

void SSD1306_DrawChar5x7(char c, uint16_t x, uint16_t y) {
    uint8_t idx;
    if      (c >= '0' && c <= '9') idx = c - '0';
    else if (c >= 'A' && c <= 'Z') idx = 10 + (c - 'A');
    else return;

    const uint8_t *pat = &Font5x7[idx * Font_5x7.FontWidth];
    for (uint8_t col = 0; col < Font_5x7.FontWidth; col++) {
        uint8_t bits = pat[col];
        for (uint8_t row = 0; row < Font_5x7.FontHeight; row++) {
            if (bits & (1 << row)) {
                SSD1306_DrawPixel(x + col, y + row, SSD1306_COLOR_WHITE);
            }
        }
    }
}
