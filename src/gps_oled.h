/*
 * GPS using OLED display
 *
 * (c) 2023 Erik Tkal
 *
 */

#pragma once

#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/critical_section.h>
#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <queue>
#include <memory>

#include "ssd1306.h"
#include "gps.h"
#include "led.h"
#include "font.h"
#include "timemgr.h"

// GPS_OLED class
//
// This combines an OLED display, GPS module and LED.
// The devices are initialized here and then callbacks are set up in order
// to receive data from the GPS, and the resulting data is displayed
// and the LED can be used to indicate a position lock and/or other
// status.
//
class GPS_OLED
{
public:
    typedef std::shared_ptr<GPS_OLED> Shared;

    GPS_OLED(SSD1306::Shared spDisplay, GPS::Shared spGPS, LED::Shared spLED, TimeMgr::Shared spTimeMgr);
    ~GPS_OLED();

    void Initialize();
    void Run();

private:
    static void sentenceCB(void* pCtx, std::string strSentence);
    static void gpsDataCB(void* pCtx, GPSData::Shared spGPSData);

    void updateUI(GPSData::Shared spGPSData);
    void drawSatGrid(uint xCenter, uint yCenter, uint radius, uint nRings = 3);
    // void drawBarGraph(uint x, uint y, uint width, uint height);
    // void drawClock(uint x, uint y, uint radius, std::string strTime);
    void drawCircleSat(uint gridCenterX,
                       uint gridCenterY,
                       uint nGridRadius,
                       float elrad,
                       float azrad,
                       uint satRadius,
                       uint16_t color     = COLOUR_WHITE,
                       uint16_t fillColor = COLOUR_WHITE);
    int linePos(int nLine);
    void drawText(int nLine, std::string strText, uint16_t color = COLOUR_WHITE, bool bRightAlign = true, uint nPadding = 0);

    // Font management - delegates to m_spDisplay
    void SetFont(const BitmapFont* pFont)
    {
        if (m_spDisplay)
            m_spDisplay->SetFont(pFont);
    }
    const BitmapFont* GetFont() const
    {
        return m_spDisplay ? m_spDisplay->GetFont() : nullptr;
    }

    // Get current font dimensions dynamically from display's font
    inline uint getCharWidth() const
    {
        const BitmapFont* pFont = GetFont();
        return pFont ? pFont->width : 8;
    }
    inline uint getCharHeight() const
    {
        const BitmapFont* pFont = GetFont();
        return pFont ? pFont->height : 8;
    }
    inline uint getLineAdvance() const
    {
        const BitmapFont* pFont = GetFont();
        return pFont ? pFont->effectiveLineAdvance() : 8;
    }

    SSD1306::Shared m_spDisplay;
    GPS::Shared m_spGPS;
    LED::Shared m_spLED;
    GPSData::Shared m_spGPSData;
    std::queue<GPSData::Shared> m_qGPSData; // Queue of GPS data to be processed by the display loop
    TimeMgr::Shared m_spTimeMgr;
    uint64_t m_nLastTimeSyncAttemptSec;
    critical_section m_GpsDataCallbackCS; // Protects access to GPS data queue
    uint64_t m_nLastUpdateUISecond;
};
