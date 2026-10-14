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

#include "gps_oled.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>
#include <iomanip>

#include "pico/stdlib.h"
#include "pico/double.h"
#include "pico/multicore.h"

#include "ssd1306.h"
#include "font_factory.h"
#include "log.h"
#include "timemgr.h"
#include "powermgr.h"

namespace
{
    constexpr double pi = 3.14159265359;
} // namespace

GPS_OLED::GPS_OLED(Display::Shared spDisplay, GPS::Shared spGPS, Button::Shared spButton)
    : GPSDevice(spDisplay, spGPS, spButton)
{
    // Custom display attributes to be used by common code the GPSDevice implementation
    m_satIconRadius = 2;
    m_bClockThickHands = true;
}

GPS_OLED::~GPS_OLED()
{
}

bool GPS_OLED::handleButtonEvent()
{
    ButtonEvent eLastButtonEvent = getLastButtonEvent();
    if (ButtonEvent::None == eLastButtonEvent)
    {
        return false;
    }
    if (ButtonEvent::Tap == eLastButtonEvent)
    {
        LogInfoD("Button tap detected");
    }
    else if (ButtonEvent::Press == eLastButtonEvent)
    {
        LogInfoD("Button press detected");
    }
    else if (ButtonEvent::LongPress == eLastButtonEvent)
    {
        LogInfoD("Long button press detected");
    }

    switch (eLastButtonEvent)
    {
    case ButtonEvent::Tap:
        switch (m_eDisplayMode)
        {
        case DisplayMode::ModeFull:
            m_eDisplayMode = DisplayMode::ModeSpeed;
            break;
        case DisplayMode::ModeSpeed:
            m_eDisplayMode = DisplayMode::ModeTime;
            break;
        case DisplayMode::ModeTime:
            m_eDisplayMode = DisplayMode::ModePosition;
            break;
        case DisplayMode::ModePosition:
            m_eDisplayMode = DisplayMode::ModeFull;
            break;
        }
        break;
    case ButtonEvent::Press:
        if (DisplayMode::ModeTime == m_eDisplayMode)
        {
            m_bTextTime = !m_bTextTime;
        }
        else if (DisplayMode::ModePosition == m_eDisplayMode || DisplayMode::ModeFull == m_eDisplayMode)
        {
            m_bAltitudeFeet = !m_bAltitudeFeet;
        }
        else if (DisplayMode::ModeSpeed == m_eDisplayMode)
        {
            switch (m_eSpeedUnit)
            {
            case SpeedUnit::Mph:
                m_eSpeedUnit = SpeedUnit::Kph;
                break;
            case SpeedUnit::Kph:
                m_eSpeedUnit = SpeedUnit::Knots;
                break;
            case SpeedUnit::Knots:
                m_eSpeedUnit = SpeedUnit::Mph;
                break;
            }
        }
        break;
    case ButtonEvent::LongPress:
        m_eDisplayMode = DisplayMode::ModeFull;
        break;
    default:
        break;
    }
    return true; // trigger a display update on state change
}

void GPS_OLED::showScreenMessage(std::string strMessage)
{
    Clear();
    SetFont(get_terminus_font(12));
    m_spDisplay->DoQuadrants([this, &strMessage]() {
        drawText(2, strMessage, COLOUR_RED, false, 0);
    });
    Show();
}

// Update the UI with the latest GPS data.
void GPS_OLED::updateUI(GPSData::Shared spGPSData)
{
    LogInfoD("GPS_OLED - updateUI() called");

    auto eDisplayMode = m_eDisplayMode;

    switch (eDisplayMode)
    {
    case DisplayMode::ModeFull:
        drawFullUI(spGPSData);
        return;
    case DisplayMode::ModeTime:
        showTime(spGPSData);
        return;
    case DisplayMode::ModePosition:
        showLatLon(spGPSData);
        return;
    case DisplayMode::ModeSpeed:
        showSpeed(spGPSData);
        return;
    default:
        LogInfo("Unknown display mode");
        showScreenMessage("Unknown display mode");
        break;
    }
}

void GPS_OLED::drawFullUI(GPSData::Shared spGPSData)
{
    m_spDisplay->DoQuadrants([this, &spGPSData]() {
        // Handle the full display
        uint16_t nWidth = Width();
        uint16_t nHeight = Height();

        // Compute padding dynamically from font dimensions
        constexpr uint PAD_CHARS_X = 0;
        // constexpr uint PAD_CHARS_Y = 0;
        uint X_PAD = PAD_CHARS_X * getCharWidth();
        // uint Y_PAD = PAD_CHARS_Y * (getCharHeight() + 1);

        Fill(COLOUR_BLACK);
        SetFont(get_terminus_font(12));

        // Get battery voltage if available
        std::string strVoltage;
        auto spPowerMgr = PowerMgr::GetInstance();
        if (spPowerMgr)
        {
            static std::vector<uint32_t> s_vVoltageSamples;
            // Integer formatting: float/double via std::to_string yields 0 when run on core 1
            uint32_t uVoltage_mV = spPowerMgr->GetVoltage_mV();
            if (uVoltage_mV != 0)
            {
                // Use a rolling average of the last 10 voltage samples
                if (s_vVoltageSamples.size() >= 10)
                {
                    s_vVoltageSamples.erase(s_vVoltageSamples.begin());
                }
                s_vVoltageSamples.push_back(uVoltage_mV);
                uVoltage_mV = 0; // Reset before summing the samples
                for (auto v : s_vVoltageSamples)
                {
                    uVoltage_mV += v;
                }
                uVoltage_mV /= s_vVoltageSamples.size();

                auto szUsingBattery = spPowerMgr->UsingBattery() ? "b" : "v";
                char szVoltage[16];
                snprintf(szVoltage,
                         sizeof(szVoltage),
                         "%s: %lu.%02lu",
                         szUsingBattery,
                         (unsigned long)(uVoltage_mV / 1000),
                         (unsigned long)((uVoltage_mV % 1000) / 10));
                strVoltage = szVoltage;
            }
        }

        // Draw satellite grid
        drawSatGrid(spGPSData, nWidth / 4 + getCharWidth(), nHeight / 2, nHeight / 2 - getCharHeight() / 2, 2);

        // Draw fix and #sats text
        drawText(0, spGPSData->strMode3D + (spGPSData->bExternalAntenna ? "*" : ""), COLOUR_WHITE, false, X_PAD);
        drawText(3, spGPSData->strNumSats, COLOUR_WHITE, true, X_PAD);
        if (!spGPSData->strLatitude.empty())
        {
            drawText(0, spGPSData->strLatitude, COLOUR_WHITE, true, X_PAD);
            drawText(1, spGPSData->strLongitude, COLOUR_WHITE, true, X_PAD);
            if (m_bAltitudeFeet)
                drawText(2, spGPSData->strAltitudeFeet, COLOUR_WHITE, true, X_PAD);
            else
                drawText(2, spGPSData->strAltitude, COLOUR_WHITE, true, X_PAD);
        }
        if (!spGPSData->strGPSTime.empty())
        {
            drawText(-1, spGPSData->strGPSTime, COLOUR_WHITE, true, X_PAD);
        }

        // Draw battery voltage if available
        drawText(4, strVoltage.c_str(), COLOUR_WHITE, true, X_PAD);

        // blit the framebuf to the display
        Show();
    });
}

void GPS_OLED::showTime(const GPSData::Shared& spGPSData)
{
    m_spDisplay->DoQuadrants([this, &spGPSData]() {
        Fill(COLOUR_BLACK);
        SetFont(get_terminus_font(18));
        if (spGPSData->strGPSTime.empty())
        {
            SetFont(get_terminus_font(12));
            drawTextCentered(2, "No GPS Time", COLOUR_WHITE);
            Show();
            return;
        }

        const bool bTextTime = m_bTextTime;

        if (!bTextTime)
        {
            drawClock(Width() / 2 - Height() / 2, 0, Height() / 2 - 1, spGPSData->strGPSTime);
        }
        else
        {
            drawTextCentered(0, TimeMgr::FormatCurrentDate(), COLOUR_WHITE);
            drawTextCentered(1, TimeMgr::FormatCurrentTimeHMS(), COLOUR_WHITE);
            drawTextCentered(-2, TimeMgr::FormatCurrentDateUTC(), COLOUR_WHITE);
            drawTextCentered(-1, TimeMgr::FormatCurrentTimeUTC() + " UTC", COLOUR_WHITE);
        }
        Show();
    });
}

void GPS_OLED::showLatLon(const GPSData::Shared& spGPSData)
{
    m_spDisplay->DoQuadrants([this, &spGPSData]() {
        Fill(COLOUR_BLACK);
        SetFont(get_terminus_font(18));
        if (!spGPSData->bHasPosition)
        {
            SetFont(get_terminus_font(12));
            drawTextCentered(2, "No GPS Position", COLOUR_WHITE);
            Show();
            return;
        }

        drawText(0, spGPSData->strLatitude, COLOUR_WHITE, true, 1);
        drawText(1, spGPSData->strLongitude, COLOUR_WHITE, true, 1);
        const bool bAltitudeFeet = m_bAltitudeFeet;

        if (bAltitudeFeet)
        {
            drawText(2, spGPSData->strAltitudeFeet, COLOUR_WHITE, true, 1);
        }
        else
        {
            drawText(2, spGPSData->strAltitude, COLOUR_WHITE, true, 1);
        }
        Show();
    });
}

void GPS_OLED::showSpeed(const GPSData::Shared& spGPSData)
{
    m_spDisplay->DoQuadrants([this, &spGPSData]() {
        Fill(COLOUR_BLACK);
        const SpeedUnit eSpeedUnit = m_eSpeedUnit;
        std::string strSpeed;
        switch (eSpeedUnit)
        {
        case SpeedUnit::Mph:
            strSpeed = spGPSData->strSpeedMph;
            break;
        case SpeedUnit::Kph:
            strSpeed = spGPSData->strSpeedKph;
            break;
        case SpeedUnit::Knots:
            strSpeed = spGPSData->strSpeedKts;
            break;
        default:
            break;
        }
        if (strSpeed.empty())
        {
            SetFont(get_terminus_font(12));
            drawTextCentered(2, "No GPS Speed", COLOUR_WHITE);
        }
        else
        {
            size_t space_pos = strSpeed.find(' ');
            if (space_pos != std::string::npos)
            {
                SetFont(get_terminus_font(32));
                Text(strSpeed.substr(0, space_pos).c_str(), 0, 0, COLOUR_WHITE, 2);
                SetFont(get_terminus_font(14));
                drawText(-1, strSpeed.substr(space_pos + 1), COLOUR_WHITE);
            }
        }
        Show();
    });
}

void GPS_OLED::showSplashScreen()
{
    SetFont(get_recommended_font(16));
    showScreenMessage("ET GPS");
    sleep_ms(2000);
}
