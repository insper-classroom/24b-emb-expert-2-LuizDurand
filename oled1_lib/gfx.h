#ifndef gfx_H
#define gfx_H

#include "ssd1306.h"
#include <string.h>

typedef struct {
    uint8_t width;     /**< width of display */
    uint8_t height;    /**< height of display */
    uint8_t pages;     /**< stores pages of display (calculated on initialization*/
    bool external_vcc; /**< whether display uses external vcc */
    uint8_t *buffer;   /**< display buffer */
    size_t bufsize;    /**< buffer size */
} ssd1306_t;

char gfx_init(ssd1306_t *p, uint16_t width, uint16_t height);
void gfx_clear_buffer(ssd1306_t *p);
void gfx_show(ssd1306_t *p);
void gfx_draw_line(ssd1306_t *p, int32_t x1, int32_t y1, int32_t x2,
                   int32_t y2);
void gfx_draw_pixel(ssd1306_t *p, uint32_t x, uint32_t y);
void gfx_draw_string(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t scale,
                     const char *s);
void gfx_draw_string_with_font(ssd1306_t *p, uint32_t x, uint32_t y,
                               uint32_t scale, const uint8_t *font,
                               const char *s);

#endif // gfx_H
