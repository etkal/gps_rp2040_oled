/*
 * GPS using OLED display
 *
 * (c) 2023-2026 Erik Tkal
 *
 */

#pragma once

#include <stdio.h>
#include <memory>

#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

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

    GPS_OLED(SSD1306::Shared spDisplay, GPS::Shared spGPS, LED::Shared spLED);
    ~GPS_OLED();

    void Initialize();
    void Run();

private:
    static void gpsDataCB(void* pCtx, GPSData::Shared spGPSData);
    static void messageCB(void* pCtx, std::string strMessage);

    // Hand a GPSData::Shared across a pico queue_t via a heap-allocated shared_ptr wrapper,
    // so the underlying object's lifetime is managed safely (and only) via reference counting.
    static bool enqueueGPSData(queue_t& q, const GPSData::Shared& spData);
    // Drain a queue of heap-allocated shared_ptr wrappers, keeping only the most recent GPSData.
    static GPSData::Shared dequeueLatestGPSData(queue_t& q);

    void showScreenMessage(std::string strMessage);
    void blinkLED(bool bHasPosition, bool bExternalAntenna);
    void updateTime(std::string strGPSTimeRaw, std::string strGPSDateRaw);
    std::string getVsysVoltage();
    void updateUI(GPSData::Shared spGPSData);
    void drawSatGrid(const GPSData::Shared& spGPSData, uint xCenter, uint yCenter, uint radius, uint nRings = 3);
    void drawBarGraph(const GPSData::Shared& spGPSData, uint x, uint y, uint width, uint height);
    void drawClock(uint x, uint y, uint radius, std::string strTime);
    void drawCircleSat(uint gridCenterX,
                       uint gridCenterY,
                       uint nGridRadius,
                       float elrad,
                       float azrad,
                       uint satRadius,
                       uint16_t color = COLOUR_WHITE,
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
    uint64_t m_nLastTimeSyncAttemptSec;
    queue_t m_qIncomingGPSData; // Queue of GPS data from the source
    queue_t m_qDisplayGPSData; // Queue of GPS data to be displayed
    AlarmTimer::Shared m_spIdleTimer;     // Timer to detect lack of GPS data
    bool m_bShowWaitingForGPS {false};
};
