// Simple bitmap font descriptor
#pragma once

#include <cstdint>

struct BitmapFont
{
    const uint8_t* data;      // pointer to glyph data
    uint8_t width;            // glyph width in pixels
    uint8_t height;           // glyph height in pixels
    uint8_t firstChar;        // ASCII code of first glyph in data
    uint8_t charCount;        // number of glyphs stored
    uint8_t colMajor = 0;     // 1 if data is column-major (bytes = columns), 0 if row-major (bytes = rows)
    uint8_t lineAdvance = 0;  // vertical spacing between lines; 0 means use height
    uint8_t rowBytes() const { return (width + 7) / 8; }
    uint8_t effectiveLineAdvance() const { return lineAdvance ? lineAdvance : height; }
};
