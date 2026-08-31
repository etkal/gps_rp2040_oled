/*
 * GPS using OLED display
 *
 * Copyright (c) 2025-2026 Erik Tkal
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

#include <stdio.h>
#include <string>
#include <iostream>
#include <pico/double.h>
#include <math.h>
#include <iomanip>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#include "ssd1306.h"
#include "gps_oled.h"
#include "power_status.h"
#include "font_factory.h"

#if !defined(NDEBUG)
#include <malloc.h>
static uint32_t getTotalHeap()
{
    extern char __StackLimit, __bss_end__;
    return &__StackLimit - &__bss_end__;
}
static uint32_t getFreeHeap()
{
    struct mallinfo m = mallinfo();
    return getTotalHeap() - m.uordblks;
}
#endif

#define SAT_ICON_RADIUS 2

namespace
{
    constexpr uint64_t timeSyncRetryIntervalSec = 5 * 60;
    constexpr double pi = 3.14159265359;
} // namespace

GPS_OLED::GPS_OLED(SSD1306::Shared spDisplay, GPS::Shared spGPS, LED::Shared spLED)
    : m_spDisplay(spDisplay),
      m_spGPS(spGPS),
      m_spLED(spLED),
      m_nLastTimeSyncAttemptSec(std::numeric_limits<uint64_t>::max())
{
    critical_section_init(&m_GpsDataCallbackCS);
}

GPS_OLED::~GPS_OLED()
{
}

void GPS_OLED::Initialize()
{
    m_spDisplay->Reset();
    m_spDisplay->Initialize();

    // Initialize display with desired font (best is Terminus 12, anything larger is not recommended)
    m_spDisplay->SetFont(get_terminus_font(12));

    m_spDisplay->SetContrast(0x10);
    showWaitingForGPS();

    m_spGPS->SetSentenceCallback(this, sentenceCB);
    m_spGPS->SetGpsDataCallback(this, gpsDataCB);
}

void GPS_OLED::Run()
{
    // Start GPS processing loop on processor core 1
    static auto sm_spGPS = m_spGPS; // Capture shared pointer for use in lambda
    multicore_launch_core1([]() {
        GPS::Shared spGPS = sm_spGPS;
        spGPS->Initialize();
        spGPS->Run();
    });

    m_spIdleTimer = std::make_shared<AlarmTimer>([this]() {
        LogInfo("GPS_OLED - No GPS data received, showing waiting message");
        showWaitingForGPS();
    });

    // Main loop for updating the display
    while (true)
    {
        busy_wait_ms(10); // Sleep for 10ms to avoid busy waiting

        bool bHasQueuedGpsData = false;
        GPSData::Shared spGPSData;
        // Check if we have new GPS data to display, just take the most recent one and discard the rest to avoid UI lag
        critical_section_enter_blocking(&m_GpsDataCallbackCS);
        if (!m_qGPSData.empty())
        {
            bHasQueuedGpsData = true;
            while (!m_qGPSData.empty())
            {
                spGPSData = m_qGPSData.front();
                m_qGPSData.pop();
            }
        }
        critical_section_exit(&m_GpsDataCallbackCS);

        if (bHasQueuedGpsData && spGPSData)
        {
            LogInfo("GPS_OLED - Updating UI");
            updateUI(spGPSData);
            spGPSData.reset();           // Free the data
            m_spIdleTimer->Start(10000); // Reset the idle timer to 10 seconds
        }
    }
}

void GPS_OLED::sentenceCB(void* pCtx, std::string strSentence)
{
    // printf("sentenceCB received: %s\n", strSentence.c_str());
}

void GPS_OLED::gpsDataCB(void* pCtx, GPSData::Shared spGPSData)
{
    LogInfo("GPS_OLED - received GPS data");
    // This callback is called from the GPS processing loop when new GPS data is available.
    // It most likely runs on a different thread/core than the main display loop, so we need
    // to ensure thread safety.  We will perform a deep copy of the GPSData and then call
    // updateUI() on the main thread in the run loop as soon as queued data is available.
    GPS_OLED* pThis = reinterpret_cast<GPS_OLED*>(pCtx);
    if (nullptr == pThis)
    {
        LogInfo("gpsDataCB: pCtx is null");
        return;
    }

    // Make a deep copy of the GPSData to avoid issues with shared ownership and data races
    critical_section_enter_blocking(&pThis->m_GpsDataCallbackCS);
    GPSData::Shared spGPSDataCopy = std::make_shared<GPSData>(*spGPSData);
    pThis->m_qGPSData.push(spGPSDataCopy);
    critical_section_exit(&pThis->m_GpsDataCallbackCS);
}

void GPS_OLED::showWaitingForGPS()
{
    m_spDisplay->Fill(COLOUR_BLACK);
    drawText(0, "Waiting for GPS", COLOUR_WHITE, false, 0);
    m_spDisplay->Show();
}

void GPS_OLED::updateUI(GPSData::Shared spGPSData)
{
    m_spGPSData = spGPSData;
    if (m_spLED)
    {
        if (m_spGPSData->bHasPosition)
        {
            m_spLED->SetPixel(0, m_spGPSData->bExternalAntenna ? led_blue : led_green);
        }
        else
        {
            m_spLED->SetPixel(0, m_spGPSData->bExternalAntenna ? led_magenta : led_red);
        }
        m_spLED->Blink_ms(20);
    }

    // Update the time if necessary
    if (!m_spGPSData->strGPSTimeRaw.empty() && !m_spGPSData->strGPSDateRaw.empty())
    {
        const uint64_t uptimeSec = time_us_64() / 1000000;
        const bool bNeverRetried = (m_nLastTimeSyncAttemptSec == std::numeric_limits<uint64_t>::max());
        const bool bUpdateDue = !TimeMgr::IsWallClockValid() || bNeverRetried ||
                                (uptimeSec - m_nLastTimeSyncAttemptSec >= timeSyncRetryIntervalSec) ||
                                !TimeMgr::IsGpsTimeDateWithinOneSecond(m_spGPSData->strGPSTimeRaw, m_spGPSData->strGPSDateRaw);
        if (bUpdateDue)
        {
            m_nLastTimeSyncAttemptSec = uptimeSec;
            LogInfo("Attempting GPS time sync");
            if (TimeMgr::SetTimeFromGps(m_spGPSData->strGPSTimeRaw, m_spGPSData->strGPSDateRaw))
            {
                LogInfo("GPS time synchronized");
            }
            else
            {
                LogInfo("GPS time sync failed");
            }
        }
    }

    uint16_t nWidth = m_spDisplay->Width();
    uint16_t nHeight = m_spDisplay->Height();

    // Compute padding dynamically from font dimensions
    constexpr uint PAD_CHARS_X = 0;
    // constexpr uint PAD_CHARS_Y = 0;
    uint X_PAD = PAD_CHARS_X * getCharWidth();
    // uint Y_PAD = PAD_CHARS_Y * (getCharHeight() + 1);

#if defined(PLATFORM_PICO)
    float vsys = 0.0;
    bool bBattery = false;
    std::string strVsys;
    if (PICO_OK == power_voltage(&vsys))
    {
        power_source(&bBattery);
        vsys = floorf(vsys * 100) / 100;
        std::stringstream oss;
        oss << (bBattery ? "b:" : "") << std::fixed << std::setfill(' ') << std::setprecision(1) << vsys << "V";
        strVsys = oss.str();
    }
#endif

    m_spDisplay->Fill(COLOUR_BLACK);

    // Draw satellite grid
    drawSatGrid(nWidth / 4, nHeight / 2, nHeight / 2 - getCharHeight() / 2, 2);

    // Draw fix and #sats text
    drawText(0, spGPSData->strMode3D + (m_spGPSData->bExternalAntenna ? "*" : ""), COLOUR_WHITE, false, X_PAD);
    drawText(3, spGPSData->strNumSats, COLOUR_WHITE, true, X_PAD);

    if (!spGPSData->strLatitude.empty())
    {
        drawText(0, spGPSData->strLatitude, COLOUR_WHITE, true, X_PAD);
        drawText(1, spGPSData->strLongitude, COLOUR_WHITE, true, X_PAD);
        drawText(2, spGPSData->strAltitude, COLOUR_WHITE, true, X_PAD);
        if (getCharHeight() <= 12) // only if room
        {
            drawText(4, spGPSData->strSpeed, COLOUR_WHITE, true, X_PAD);
        }
    }
    if (!spGPSData->strGPSTime.empty())
    {
        drawText(-1, spGPSData->strGPSTime, COLOUR_WHITE, true, X_PAD);
    }

#if defined(PLATFORM_PICO)
    if (!strVsys.empty() && getCharHeight() <= 8) // only if room
    {
        drawText(-2, strVsys, COLOUR_WHITE, true, X_PAD);
    }
#endif

    // blit the framebuf to the display
    m_spDisplay->Show();

    m_spGPSData.reset();

#if !defined(NDEBUG)
    LogInfo("Total Heap: " + std::to_string(getTotalHeap()) + "  Free Heap: " + std::to_string(getFreeHeap()));
#endif
}

void GPS_OLED::drawSatGrid(uint xCenter, uint yCenter, uint radius, uint nRings)
{
    for (uint i = 1; i <= nRings; ++i)
    {
        m_spDisplay->Ellipse(xCenter, yCenter, radius * i / nRings, radius * i / nRings, COLOUR_WHITE);
    }

    m_spDisplay->VLine(xCenter, yCenter - radius - 2, 2 * radius + 5, COLOUR_WHITE);
    m_spDisplay->HLine(xCenter - radius - 2, yCenter, 2 * radius + 5, COLOUR_WHITE);
    // m_spDisplay->Text("N", xCenter - getCharWidth() / 2, yCenter - radius - getCharHeight(), COLOUR_RED);
    m_spDisplay->Text("^", xCenter - getCharWidth() / 2 + 1, yCenter - radius - getCharHeight() / 2, COLOUR_RED);
    // m_spDisplay->Text("'", xCenter - 6, yCenter - radius - getCharHeight() / 2, COLOUR_RED);
    // m_spDisplay->Text("`", xCenter - 2, yCenter - radius - getCharHeight() / 2, COLOUR_RED);

    int satRadius = SAT_ICON_RADIUS / 2;
    if (!m_spGPSData->strLatitude.empty())
    {
        satRadius = SAT_ICON_RADIUS;
    }
    for (auto oEntry : m_spGPSData->mSatList)
    {
        auto oSat = oEntry.second;
        double elrad = oSat.m_el * pi / 180;
        double azrad = oSat.m_az * pi / 180;
        drawCircleSat(xCenter, yCenter, radius, elrad, azrad, satRadius, COLOUR_WHITE, COLOUR_BLACK);
        for (auto nSat : m_spGPSData->vUsedList)
        {
            if (oSat.m_num == nSat)
            {
                drawCircleSat(xCenter, yCenter, radius, elrad, azrad, satRadius, COLOUR_WHITE, COLOUR_BLUE);
                break;
            }
        }
    }
}

void GPS_OLED::drawCircleSat(uint gridCenterX,
                             uint gridCenterY,
                             uint nGridRadius,
                             float elrad,
                             float azrad,
                             uint satRadius,
                             uint16_t color,
                             uint16_t fillColor)
{
    // Draw satellite (fill first, then draw open circle)
    int dx = (nGridRadius - SAT_ICON_RADIUS) * cos(elrad) * sin(azrad);
    int dy = (nGridRadius - SAT_ICON_RADIUS) * cos(elrad) * -cos(azrad);
    int x = gridCenterX + dx;
    int y = gridCenterY + dy;
    m_spDisplay->Ellipse(x, y, satRadius, satRadius, fillColor, true); // Clear area with fill
    m_spDisplay->Ellipse(x, y, satRadius, satRadius, color);           // Draw circle without fill
}

int GPS_OLED::linePos(int nLine)
{
    if (nLine >= 0)
        return nLine * getLineAdvance();
    else
        return m_spDisplay->Height() + (nLine * getLineAdvance());
}

void GPS_OLED::drawText(int nLine, std::string strText, uint16_t color, bool bRightAlign, uint nRightPad)
{
    int x = (!bRightAlign) ? 0 : m_spDisplay->Width() - (strText.length() * getCharWidth());
    int y = linePos(nLine);
    x = x - nRightPad;
    m_spDisplay->Text(strText.c_str(), x, y, color);
}
