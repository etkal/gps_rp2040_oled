/*
 * Pico SSD1306 OLED driver class
 *
 * Copyright (c) 2023 Erik Tkal
 *
 * Based on a MicroPython implementation, combined with pico sample code
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "ssd1306.h"


SSD1306::SSD1306(uint nWidth, uint nHeight, bool bExternalVcc)
    : m_dispWidth(nWidth),
      m_dispHeight(nHeight),
      m_bExternalVcc(bExternalVcc),
      m_nPages(nHeight / 8)
{
}

void SSD1306::Reset()
{
}

void SSD1306::Initialize()
{
    Framebuf::Initialize(m_dispWidth, m_dispHeight, MVLSB);

    initInternal();

    write_cmd(OLED_SET_DISP); // display off

    /* memory mapping */
    write_cmd(OLED_SET_MEM_ADDR); // set memory address mode
    write_cmd(0x00);              // horizontal addressing mode

    /* resolution and layout */
    write_cmd(OLED_SET_DISP_START_LINE); // set display start line to 0

    write_cmd(OLED_SET_SEG_REMAP | 0x01); // set segment re-map
    // column address 127 is mapped to SEG0

    write_cmd(OLED_SET_MUX_RATIO); // set multiplex ratio
    write_cmd(m_dispHeight - 1);   // display height

    write_cmd(OLED_SET_COM_OUT_DIR | 0x08); // set COM (common) output scan direction
    // scan from bottom up, COM[N-1] to COM0

    write_cmd(OLED_SET_DISP_OFFSET); // set display offset
    write_cmd(0x00);                 // no offset

    write_cmd(OLED_SET_COM_PIN_CFG);                           // set COM (common) pins hardware configuration
    write_cmd((m_dispWidth > 2 * m_dispHeight) ? 0x02 : 0x12); // manufacturer magic number
                                                               // 0x02 if self.width > 2 * self.height else 0x12,

    /* timing and driving scheme */
    write_cmd(OLED_SET_DISP_CLK_DIV); // set display clock divide ratio
    write_cmd(0x80);                  // div ratio of 1, standard freq

    write_cmd(OLED_SET_PRECHARGE);           // set pre-charge period
    write_cmd(m_bExternalVcc ? 0x22 : 0xF1); // Vcc

    write_cmd(OLED_SET_VCOM_DESEL); // set VCOMH deselect level
    write_cmd(0x30);                // 0.83xVcc

    /* display */
    write_cmd(OLED_SET_CONTRAST); // set contrast control
    write_cmd(0xFF);

    write_cmd(OLED_SET_ENTIRE_ON); // set entire display on to follow RAM content

    write_cmd(OLED_SET_NORM_INV); // set normal (not inverted) display

    write_cmd(OLED_SET_CHARGE_PUMP);         // set charge pump
    write_cmd(m_bExternalVcc ? 0x10 : 0x14); // Vcc

    write_cmd(OLED_SET_SCROLL | 0x00); // deactivate horizontal scrolling if set
    // this is necessary as memory writes will corrupt if scrolling was enabled

    write_cmd(OLED_SET_DISP | 0x01); // turn display on
}

void SSD1306::DisplayOff()
{
    write_cmd(OLED_SET_DISP); // display off
}

void SSD1306::DisplayOn()
{
    write_cmd(OLED_SET_DISP | 0x01); // turn display on
}

void SSD1306::SetContrast(uint8_t contrast)
{
    write_cmd(OLED_SET_CONTRAST); // set contrast control
    write_cmd(contrast);
}

void SSD1306::Invert(bool bInvert)
{
    write_cmd(OLED_SET_NORM_INV | bInvert ? 0x01 : 0x00);
}

void SSD1306::Rotate(bool bRotate)
{
    write_cmd(OLED_SET_COM_OUT_DIR | (bRotate ? 0x08 : 0x00)); // set COM (common) output scan direction
    write_cmd(OLED_SET_SEG_REMAP | (bRotate ? 0x01 : 0x00));   // set segment re-map
}

void SSD1306::Show()
{
    uint8_t x0 = 0;
    uint8_t x1 = m_dispWidth - 1;
    if (m_dispWidth != 128)
    {
        // narrow displays use centred columns
        uint col_offset = (128 - m_dispWidth) / 2;
        x0 += col_offset;
        x1 += col_offset;
    }
    write_cmd(OLED_SET_COL_ADDR);
    write_cmd(x0);
    write_cmd(x1);
    write_cmd(OLED_SET_PAGE_ADDR);
    write_cmd(0);
    write_cmd(m_nPages - 1);
    uint buflen = (x1 - x0 + 1) * m_nPages;
    write_data(reinterpret_cast<uint8_t*>(buffer()), buflen);
}

void SSD1306::SetPixel(int x, int y, uint16_t color)
{
    return Framebuf::setpixel(x, y, color);
}

uint16_t SSD1306::GetPixel(int x, int y)
{
    return Framebuf::getpixel(x, y);
}

void SSD1306::FillRect(int x, int y, int w, int h, uint16_t color)
{
    return Framebuf::fillrect(x, y, w, h, color);
}

void SSD1306::Fill(uint16_t color)
{
    return Framebuf::fill(color);
}

void SSD1306::HLine(int x, int y, int w, uint16_t color)
{
    return Framebuf::hline(x, y, w, color);
}

void SSD1306::VLine(int x, int y, int h, uint16_t color)
{
    return Framebuf::vline(x, y, h, color);
}

void SSD1306::Rect(int x, int y, int w, int h, uint16_t color, bool bFill)
{
    return Framebuf::rect(x, y, w, h, color, bFill);
}

void SSD1306::Line(int x1, int y1, int x2, int y2, uint16_t color)
{
    return Framebuf::line(x1, y1, x2, y2, color);
}

void SSD1306::Ellipse(int cx, int cy, int xradius, int yradius, uint16_t color, bool bFill, uint8_t mask)
{
    return Framebuf::ellipse(cx, cy, xradius, yradius, color, bFill, mask);
}

void SSD1306::Text(const char* str, int x, int y, uint16_t color)
{
    return Framebuf::text(str, x, y, color);
}

void SSD1306::Text(const char* str, int x, int y, uint16_t color, int scale)
{
    return text(str, x, y, color, scale);
}

void SSD1306::Text(const char* str, int x, int y, uint16_t color, const BitmapFont& font, int scale)
{
    return text(str, x, y, color, font, scale);
}

//
// SSD1306_I2C
//

SSD1306_I2C::SSD1306_I2C(uint nWidth, uint nHeight, i2c_inst_t* i2c, uint sdaPin, uint sclPin, uint baudrate, uint8_t addr, bool bExternalVcc)
    : SSD1306(nWidth, nHeight, bExternalVcc),
      m_i2c(i2c),
      m_sdaPin(sdaPin),
      m_sclPin(sclPin),
      m_baudrate(baudrate),
      m_addr(addr)
{
}

void SSD1306_I2C::initInternal()
{
    i2c_init(m_i2c, m_baudrate);
    gpio_set_function(m_sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(m_sclPin, GPIO_FUNC_I2C);
    gpio_pull_up(m_sdaPin);
    gpio_pull_up(m_sclPin);
}

void SSD1306_I2C::write_cmd(uint8_t cmd)
{
    // I2C write process expects a control byte followed by data
    // this "data" can be a command or data to follow up a command

    // Co = 1, D/C = 0 => the driver expects a command
    uint8_t buf[2] = {0x80, cmd};
    i2c_write_blocking(m_i2c, (OLED_ADDR & OLED_WRITE_MODE), buf, 2, false);
}

void SSD1306_I2C::write_data(uint8_t* buf, uint nLen)
{
    // in horizontal addressing mode, the column address pointer auto-increments
    // and then wraps around to the next page, so we can send the entire frame
    // buffer in one gooooooo!

    // copy our frame buffer into a new buffer because we need to add the control byte
    // to the beginning

    // TODO find a more memory-efficient way to do this..
    // maybe break the data transfer into pages?
    uint8_t* temp_buf = new uint8_t[nLen + 1]; // reinterpret_cast<uint8_t*>(malloc(nLen + 1));

    for (uint i = 1; i < nLen + 1; i++)
    {
        temp_buf[i] = buf[i - 1];
    }
    // Co = 0, D/C = 1 => the driver expects data to be written to RAM
    temp_buf[0] = 0x40;
    i2c_write_blocking(m_i2c, (OLED_ADDR & OLED_WRITE_MODE), temp_buf, nLen + 1, false);

    delete[] temp_buf; // free(temp_buf);
}
