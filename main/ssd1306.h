#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Compile-time display geometry.
 * Override via compiler flags (-DOLED_WIDTH=128) or before including this header. */
#ifndef OLED_WIDTH
#define OLED_WIDTH      72
#endif
#ifndef OLED_HEIGHT
#define OLED_HEIGHT     40
#endif
/* Column start offset within the 128-wide SSD1306 GDDRAM.
 * 72-pixel displays are typically wired at column 28 (128-72)/2 = 28. */
#ifndef OLED_COL_OFFSET
#define OLED_COL_OFFSET 28
#endif

#define OLED_COLS  (OLED_WIDTH  / 8)
#define OLED_ROWS  (OLED_HEIGHT / 8)

/* Initialise I2C and the SSD1306.
 * sda_pin / scl_pin: GPIO numbers.
 * i2c_addr: usually 0x3C or 0x3D. */
void ssd1306_init(int sda_pin, int scl_pin, uint8_t i2c_addr);

/* Clear the internal framebuffer (does NOT flush to display). */
void ssd1306_clear(void);

/* Push the framebuffer to the display. */
void ssd1306_flush(void);

/* Write one character at character cell (col 0-15, row 0-7). */
void ssd1306_putchar(int col, int row, char c);

/* Write a NUL-terminated string starting at (col, row).
 * Clips at right edge; does NOT wrap. */
void ssd1306_puts(int col, int row, const char *s);

/* Clear one character row (0-7). */
void ssd1306_clear_row(int row);

/* Turn the display panel on (true) or off (false). */
void ssd1306_power(bool on);
