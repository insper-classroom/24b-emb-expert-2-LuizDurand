#include "gfx.h"
#include "font.h"

inline static void swap(int32_t *a, int32_t *b) {
    int32_t *t = a;
    *a = *b;
    *b = *t;
}

char gfx_init(ssd1306_t *p, uint16_t width, uint16_t height) {
    p->width = width;
    p->height = height;
    p->pages = height / 8;
    p->bufsize = (p->pages) * (p->width);

    // trocar remover malloc por alocação estática
    if ((p->buffer = malloc(p->bufsize + 1)) == NULL) {
        p->bufsize = 0;
        return false;
    }

    memset(p->buffer, 0, p->bufsize + 1);

    ++(p->buffer);

    return true;
}

inline void gfx_deinit(ssd1306_t *p) { free(p->buffer - 1); }

void gfx_clear_buffer(ssd1306_t *p) {
    memset(p->buffer, 0, p->bufsize + 1);
}

void gfx_clear_pixel(ssd1306_t *p, uint32_t x, uint32_t y) {
    if (x >= p->width || y >= p->height)
        return;

    p->buffer[x + p->width * (y >> 3)] &= ~(0x1 << (y & 0x07));
}

void gfx_draw_pixel(ssd1306_t *p, uint32_t x, uint32_t y) {
    if (x >= p->width || y >= p->height)
        return;

    p->buffer[x + p->width * (y >> 3)] |=
        0x1 << (y & 0x07); // y>>3==y/8 && y&0x7==y%8
}

void gfx_draw_line(ssd1306_t *p, int32_t x1, int32_t y1, int32_t x2,
                   int32_t y2) {
    if (x1 > x2) {
        swap(&x1, &x2);
        swap(&y1, &y2);
    }

    if (x1 == x2) {
        if (y1 > y2)
            swap(&y1, &y2);
        for (int32_t i = y1; i <= y2; ++i)
            gfx_draw_pixel(p, x1, i);
        return;
    }

    float m = (float)(y2 - y1) / (float)(x2 - x1);

    for (int32_t i = x1; i <= x2; ++i) {
        float y = m * (float)(i - x1) + (float)y1;
        gfx_draw_pixel(p, i, (uint32_t)y);
    }
}

void gfx_draw_square(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t width,
                     uint32_t height) {
    for (uint32_t i = 0; i < width; ++i)
        for (uint32_t j = 0; j < height; ++j)
            gfx_draw_pixel(p, x + i, y + j);
}

void gfx_draw_empty_square(ssd1306_t *p, uint32_t x, uint32_t y,
                           uint32_t width, uint32_t height) {
    gfx_draw_line(p, x, y, x + width, y);
    gfx_draw_line(p, x, y + height, x + width, y + height);
    gfx_draw_line(p, x, y, x, y + height);
    gfx_draw_line(p, x + width, y, x + width, y + height);
}

void gfx_draw_char_with_font(ssd1306_t *p, uint32_t x, uint32_t y,
                             uint32_t scale, const uint8_t *font, char c) {
    if (c < font[3] || c > font[4])
        return;

    uint32_t parts_per_line = (font[0] >> 3) + ((font[0] & 7) > 0);
    for (uint8_t w = 0; w < font[1]; ++w) { // width
        uint32_t pp =
            (c - font[3]) * font[1] * parts_per_line + w * parts_per_line + 5;
        for (uint32_t lp = 0; lp < parts_per_line; ++lp) {
            uint8_t line = font[pp];

            for (int8_t j = 0; j < 8; ++j, line >>= 1) {
                if (line & 1)
                    gfx_draw_square(p, x + w * scale, y + ((lp << 3) + j) * scale,
                                    scale, scale);
            }

            ++pp;
        }
    }
}

void gfx_draw_string_with_font(ssd1306_t *p, uint32_t x, uint32_t y,
                               uint32_t scale, const uint8_t *font,
                               const char *s) {
    for (int32_t x_n = x; *s; x_n += (font[1] + font[2]) * scale) {
        gfx_draw_char_with_font(p, x_n, y, scale, font, *(s++));
    }
}

void gfx_draw_char(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t scale,
                   char c) {
    gfx_draw_char_with_font(p, x, y, scale, font_8x5, c);
}

void gfx_draw_string(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t scale,
                     const char *s) {
    gfx_draw_string_with_font(p, x, y, scale, font_8x5, s);
}

void gfx_show(ssd1306_t *p) {
    for (uint8_t page = 0; page < p->pages; page++) {
        ssd1306_set_page_address(page);
        ssd1306_set_column_address(0);
        ssd1306_put_page(p->buffer + (page * p->width), page, 0, p->width);
    }
}
