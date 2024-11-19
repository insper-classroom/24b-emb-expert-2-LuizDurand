#ifndef SSD1306_H
#define SSD1306_H

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdlib.h>

#define SSD1306_CMD_COL_ADD_SET_LSB(column) (0x00 | (column))
#define SSD1306_CMD_COL_ADD_SET_MSB(column) (0x10 | (column))
#define SSD1306_CMD_SET_MEMORY_ADDRESSING_MODE 0x20
#define SSD1306_CMD_SET_COLUMN_ADDRESS 0x21
#define SSD1306_CMD_SET_PAGE_ADDRESS 0x22
#define SSD1306_CMD_SET_DISPLAY_START_LINE(line) (0x40 | (line))
#define SSD1306_CMD_SET_CONTRAST_CONTROL_FOR_BANK0 0x81
#define SSD1306_CMD_SET_CHARGE_PUMP_SETTING 0x8D
#define SSD1306_CMD_SET_SEGMENT_RE_MAP_COL0_SEG0 0xA0
#define SSD1306_CMD_SET_SEGMENT_RE_MAP_COL127_SEG0 0xA1
#define SSD1306_CMD_ENTIRE_DISPLAY_AND_GDDRAM_ON 0xA4
#define SSD1306_CMD_ENTIRE_DISPLAY_ON 0xA5
#define SSD1306_CMD_SET_NORMAL_DISPLAY 0xA6
#define SSD1306_CMD_SET_INVERSE_DISPLAY 0xA7
#define SSD1306_CMD_SET_MULTIPLEX_RATIO 0xA8
#define SSD1306_CMD_SET_DISPLAY_ON 0xAF
#define SSD1306_CMD_SET_DISPLAY_OFF 0xAE
#define SSD1306_CMD_SET_PAGE_START_ADDRESS(page) (0xB0 | (page))
#define SSD1306_CMD_SET_COM_OUTPUT_SCAN_UP 0xC0
#define SSD1306_CMD_SET_COM_OUTPUT_SCAN_DOWN 0xC8
#define SSD1306_CMD_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_CMD_SET_DISPLAY_CLOCK_DIVIDE_RATIO 0xD5
#define SSD1306_CMD_SET_PRE_CHARGE_PERIOD 0xD9
#define SSD1306_CMD_SET_COM_PINS 0xDA
#define SSD1306_CMD_SET_VCOMH_DESELECT_LEVEL 0xDB
#define SSD1306_CMD_NOP 0xE3

#define GFX_MONO_LCD_WIDTH 128
#ifndef GFX_MONO_LCD_HEIGHT
#define GFX_MONO_LCD_HEIGHT 32
#endif
#define GFX_MONO_LCD_PIXELS_PER_BYTE 8
#define GFX_MONO_LCD_PAGES (GFX_MONO_LCD_HEIGHT / GFX_MONO_LCD_PIXELS_PER_BYTE)
#define GFX_MONO_LCD_FRAMEBUFFER_SIZE \
    ((GFX_MONO_LCD_WIDTH * GFX_MONO_LCD_HEIGHT) / GFX_MONO_LCD_PIXELS_PER_BYTE)

#define SSD1306_RST_PIN 14
#define SSD1306_DATA_CMD_SEL 15

#define PIN_SCK 10
#define PIN_TX 11
#define PIN_CS 9
#define SPI_PORT spi1
#define SSD1306_LATENCY 10

inline void spi_cs_select(void);
inline void spi_cs_deselect(void);
inline void ssd1306_set_display_start_line_address(uint8_t address);
inline void ssd1306_set_column_address(uint8_t address);
inline void ssd1306_set_page_address(uint8_t address);
inline void ssd1306_display_on(void);
inline void ssd1306_display_off(void);
inline uint8_t ssd1306_set_contrast(uint8_t contrast);
inline void ssd1306_display_invert_enable(void);
inline void ssd1306_display_invert_disable(void);

void gfx_mono_ssd1306_put_byte(uint8_t page, uint8_t column, uint8_t data,
                               bool force);
void ssd1306_interface_init(void);
void ssd1306_hard_reset(void);
void ssd1306_write_command(uint8_t command);
void ssd1306_write_data(uint8_t data);
void ssd1306_init(void);

#endif // SSD1306_H
