#pragma once
#include "font.h"

// Get a font by exact pixel height.
// Available heights: 8 (PetMe), 12 (Terminus 6x12), 14 (Terminus 8x14), 
//                   18 (Terminus 10x18), 24 (Terminus 12x24), 32 (Terminus 16x32)
// Returns nullptr if the requested height is not available.
const BitmapFont* get_terminus_font(int height);

// Get the best-fit font for a desired pixel height.
// Picks the smallest font that is >= desired_size, or the largest if none fits.
inline const BitmapFont* get_recommended_font(int desired_size)
{
    if (desired_size < 12)  return get_terminus_font(8);   // PetMe 8x8
    if (desired_size < 14) return get_terminus_font(12);  // Terminus 6x12
    if (desired_size < 18) return get_terminus_font(14);  // Terminus 8x14
    if (desired_size < 24) return get_terminus_font(18);  // Terminus 10x18
    if (desired_size < 32) return get_terminus_font(24);  // Terminus 12x24
    return get_terminus_font(32);                          // Terminus 16x32
}
