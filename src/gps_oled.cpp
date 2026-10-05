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

#define SAT_ICON_RADIUS 2

namespace
{
    constexpr double pi = 3.14159265359;
} // namespace

GPS_OLED::GPS_OLED(SSD1306::Shared spDisplay, GPS::Shared spGPS, Button::Shared spButton)
    : m_spDisplay(spDisplay),
      m_spGPS(spGPS),
      m_spButton(spButton)
{
    critical_section_init(&m_CallbackCs);
}

GPS_OLED::~GPS_OLED()
{
}

void GPS_OLED::Initialize()
{
    m_bInitialized = true;
    queue_init(&m_qIncomingGPSData, sizeof(GPSData::Shared*), 10); // Initialize the queue with a capacity of 10

    m_spDisplay->Initialize();

    // Initialize display with desired font (best is Terminus 12, anything larger is not recommended)
    m_spDisplay->SetFont(get_terminus_font(12));

    m_spDisplay->SetContrast(0x10);
    m_bShowWaitingForGPS = true;

    m_spGPS->SetGpsDataCallback(this, gpsDataCB);
    m_spGPS->SetMessageCallback(this, messageCB);
    if (m_spButton)
    {
        m_spButton->SetEventCallback(this, buttonEventCB);
    }

    m_spIdleTimer = std::make_shared<AlarmTimer>([this]() {
        m_bShowWaitingForGPS = true;
    });
}

void GPS_OLED::Start()
{
#if defined(DISPLAY_ON_CORE_1)
    static auto sm_spThis = shared_from_this(); // Capture pointer for use in lambda
    // Default core 1 stack is only 4KB; iostream/ostringstream logging overflows it
    static uint32_t sm_core1Stack[16 * 1024 / sizeof(uint32_t)];
    multicore_launch_core1_with_stack(
        []() {
            GPS_OLED::Shared spThis = sm_spThis;
            // Initialize and run the GPS processing loop on core 1
            LogInfo("Starting GPS_OLED processing on core 1");
            spThis->Initialize();
            spThis->Run();
        },
        sm_core1Stack,
        sizeof(sm_core1Stack));
#else
    LogInfo("Initializing GPS_OLED processing on core 0");
    Initialize();
#endif
}

void GPS_OLED::Run()
{
#if defined(DISPLAY_ON_CORE_1)
    while (!m_bExit)
    {
        DoWork();
    }
#else
    LogInfo("GPS_OLED::Run() should not be called on core 0");
    return;
#endif
}

void GPS_OLED::DoWork()
{
#if defined(DISPLAY_ON_CORE_1)
    if (0 == get_core_num())
    {
        return; // Skip processing on core 0 if GPS_OLED is running on core 1
    }
#endif
    if (!m_bInitialized)
    {
        return;
    }
    if (!m_bExit)
    {
        if (handleButtonEvent() && m_spLastGPSData)
        {
            // Redraw immediately with the last known data rather than waiting for the next GPS update
            updateUI(m_spLastGPSData);
        }

        GPSData::Shared spGPSData = dequeueLatestGPSData(m_qIncomingGPSData);
        if (spGPSData)
        {
            LogInfoD("GPS_OLED - Processing new GPS data");
            m_bShowWaitingForGPS = false;
            critical_section_enter_blocking(&m_CallbackCs);
            m_bStatusChanged = true;
            m_bHasPosition = spGPSData->bHasPosition;
            m_bExternalAntenna = spGPSData->bExternalAntenna;
            m_strGpsTimeRaw = spGPSData->strGPSTimeRaw;
            m_strGpsDateRaw = spGPSData->strGPSDateRaw;
            critical_section_exit(&m_CallbackCs);
            m_spLastGPSData = spGPSData;
            updateUI(spGPSData);
            m_spIdleTimer->Start(5000); // Reset the idle timer
        }
        if (m_bShowWaitingForGPS)
        {
            LogInfo("GPS_OLED - No GPS data received showing waiting message");
            critical_section_enter_blocking(&m_CallbackCs);
            m_bStatusChanged = true;
            m_bHasPosition = false;
            m_bExternalAntenna = false;
            m_strGpsTimeRaw.clear();
            m_strGpsDateRaw.clear();
            critical_section_exit(&m_CallbackCs);
            showScreenMessage("Waiting for GPS data");
            m_bShowWaitingForGPS = false;
        }
    }
}

// Stop the GPS_OLED processing loop. This will cause Run() to return.
void GPS_OLED::Stop()
{
    m_bExit = true;
}

bool GPS_OLED::GetStatus(GPS_Status& status)
{
    critical_section_enter_blocking(&m_CallbackCs);
    if (!m_bStatusChanged)
    {
        critical_section_exit(&m_CallbackCs);
        return false;
    }
    m_bStatusChanged = false;
    status.bHasPosition = m_bHasPosition;
    status.bExternalAntenna = m_bExternalAntenna;
    status.strGpsTimeRaw = m_strGpsTimeRaw;
    status.strGpsDateRaw = m_strGpsDateRaw;
    critical_section_exit(&m_CallbackCs);
    return true;
}

bool GPS_OLED::enqueueGPSData(queue_t& q, const GPSData::Shared& spData)
{
    GPSData::Shared* pspData = new GPSData::Shared(spData);
    if (!queue_try_add(&q, &pspData))
    {
        delete pspData;
        return false;
    }
    return true;
}

GPSData::Shared GPS_OLED::dequeueLatestGPSData(queue_t& q)
{
    GPSData::Shared spLatest;
    GPSData::Shared* pspData = nullptr;
    while (queue_try_remove(&q, &pspData))
    {
        if (pspData)
        {
            spLatest = *pspData; // shares ownership of the underlying GPSData with the queued copy
            delete pspData;      // only deletes the heap-allocated wrapper, not the GPSData itself
        }
        pspData = nullptr;
    }
    return spLatest;
}

bool GPS_OLED::handleButtonEvent()
{
    critical_section_enter_blocking(&m_CallbackCs);
    if (ButtonEvent::None == m_eLastButtonEvent)
    {
        critical_section_exit(&m_CallbackCs);
        return false;
    }
    if (ButtonEvent::Tap == m_eLastButtonEvent)
    {
        LogInfoD("GPS_OLED - Button tap detected");
    }
    else if (ButtonEvent::Press == m_eLastButtonEvent)
    {
        LogInfoD("GPS_OLED - Button press detected");
    }
    else if (ButtonEvent::LongPress == m_eLastButtonEvent)
    {
        LogInfoD("GPS_OLED - Long button press detected");
    }

    switch (m_eLastButtonEvent)
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

    // Reset the last button event after handling
    m_eLastButtonEvent = ButtonEvent::None;
    critical_section_exit(&m_CallbackCs);
    return true; // trigger a display update on state change
}

void GPS_OLED::gpsDataCB(void* pCtx, GPSData::Shared spGPSData)
{
    LogInfoD("GPS_OLED - received GPS data");
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

    // Deep copy the GPSData once here; all subsequent hand-offs share ownership of this copy.
    LogInfoD("GPS_OLED - Enqueueing GPS data");
    enqueueGPSData(pThis->m_qIncomingGPSData, std::make_shared<GPSData>(*spGPSData));
}

void GPS_OLED::messageCB(void* pCtx, std::string strMessage)
{
    LogInfo("GPS_OLED - received message: " + strMessage);
    GPS_OLED* pThis = reinterpret_cast<GPS_OLED*>(pCtx);
    if (nullptr == pThis)
    {
        LogInfo("messageCB: pCtx is null");
        return;
    }

    pThis->showScreenMessage(strMessage);
}

void GPS_OLED::buttonEventCB(void* pCtx, ButtonEvent eType)
{
    GPS_OLED* pThis = reinterpret_cast<GPS_OLED*>(pCtx);
    if (nullptr == pThis)
    {
        LogInfo("buttonEventCB: pCtx is null");
        return;
    }

    // Handle button events here
    critical_section_enter_blocking(&pThis->m_CallbackCs);
    switch (eType)
    {
    case ButtonEvent::Tap:
        pThis->m_eLastButtonEvent = ButtonEvent::Tap;
        break;
    case ButtonEvent::Press:
        pThis->m_eLastButtonEvent = ButtonEvent::Press;
        break;
    case ButtonEvent::LongPress:
        pThis->m_eLastButtonEvent = ButtonEvent::LongPress;
        break;
    default:
        break;
    }
    critical_section_exit(&pThis->m_CallbackCs);
}

void GPS_OLED::showScreenMessage(std::string strMessage)
{
    m_spDisplay->Fill(COLOUR_BLACK);
    m_spDisplay->SetFont(get_terminus_font(12));
    drawText(0, strMessage, COLOUR_WHITE, false, 0);
    m_spDisplay->Show();
}

// Update the UI with the latest GPS data.
void GPS_OLED::updateUI(GPSData::Shared spGPSData)
{
    LogInfoD("GPS_OLED - updateUI() called");

    critical_section_enter_blocking(&m_CallbackCs);
    auto eDisplayMode = m_eDisplayMode;
    critical_section_exit(&m_CallbackCs);

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
    // Handle the full display
    uint16_t nWidth = m_spDisplay->Width();
    uint16_t nHeight = m_spDisplay->Height();

    // Compute padding dynamically from font dimensions
    constexpr uint PAD_CHARS_X = 0;
    // constexpr uint PAD_CHARS_Y = 0;
    uint X_PAD = PAD_CHARS_X * getCharWidth();
    // uint Y_PAD = PAD_CHARS_Y * (getCharHeight() + 1);

    m_spDisplay->Fill(COLOUR_BLACK);
    m_spDisplay->SetFont(get_terminus_font(12));

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

    // blit the framebuf to the display
    m_spDisplay->Show();
}

void GPS_OLED::drawSatGrid(const GPSData::Shared& spGPSData, uint xCenter, uint yCenter, uint radius, uint nRings)
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
    if (!spGPSData->strLatitude.empty())
    {
        satRadius = SAT_ICON_RADIUS;
    }
    for (auto oEntry : spGPSData->mSatList)
    {
        auto oSat = oEntry.second;
        double elrad = oSat.m_el * pi / 180;
        double azrad = oSat.m_az * pi / 180;
        drawCircleSat(xCenter, yCenter, radius, elrad, azrad, satRadius, COLOUR_WHITE, COLOUR_BLACK);
        for (auto nSat : spGPSData->vUsedList)
        {
            if (oSat.m_num == nSat)
            {
                drawCircleSat(xCenter, yCenter, radius, elrad, azrad, satRadius, COLOUR_WHITE, COLOUR_BLUE);
                break;
            }
        }
    }
}

void GPS_OLED::drawClock(uint x, uint y, uint radius, std::string strTime)
{
    uint xCenter = x + radius;
    uint yCenter = y + radius;
    uint nHour = atoi(strTime.substr(0, 2).c_str());
    const float gmtOffset = TimeMgr::TimeZoneOffsetHours();
    float hour = (float)(nHour % 12) + gmtOffset;
    hour = (hour < 0) ? hour + 12 : hour;
    hour = (hour >= 12) ? hour - 12 : hour;
    float minute = (float)atoi(strTime.substr(3, 2).c_str());
    float second = (float)atoi(strTime.substr(6, 2).c_str());
    uint16_t ringColor = COLOUR_WHITE;
    uint16_t faceColor = COLOUR_BLACK;
    uint16_t handColor = COLOUR_WHITE;
    uint16_t secondHandColor = COLOUR_RED;
    double handLenHour = radius * 0.4;
    double handLenMinute = radius * 0.7;
    double handLenSecond = radius * 0.8;
    double radiansHour = 2 * pi * (((hour * 3600.0) + (minute * 60.0) + second) / (12.0 * 60.0 * 60.0));
    double radiansMinute = 2 * pi * (((minute * 60.0) + second) / (60.0 * 60.0));
    double radiansSecond = 2 * pi * (second / 60.0);
    int dxh = int(handLenHour * sin(radiansHour));
    int dyh = int(handLenHour * -cos(radiansHour));
    int dxm = int(handLenMinute * sin(radiansMinute));
    int dym = int(handLenMinute * -cos(radiansMinute));
    int dxs = int(handLenSecond * sin(radiansSecond));
    int dys = int(handLenSecond * -cos(radiansSecond));

    // Draw the face
    m_spDisplay->Ellipse(xCenter, yCenter, radius, radius, ringColor, false);
    m_spDisplay->Ellipse(xCenter, yCenter, radius - 1, radius - 1, faceColor, true);
    // Draw quarter dots
    for (uint degDot = 0; degDot < 360; degDot += 30)
    {
        uint16_t colDot = COLOUR_WHITE;
        uint16_t sizDot = (degDot % 90 == 0) ? 2 : 1;
        uint dxDot = int((radius - sizDot) * sin(degDot * pi / 180));
        uint dyDot = int((radius - sizDot) * -cos(degDot * pi / 180));
        m_spDisplay->Ellipse(xCenter + dxDot, yCenter + dyDot, sizDot, sizDot, colDot, true);
    }
    // Draw the hands
    m_spDisplay->Line(xCenter, yCenter, xCenter + dxs, yCenter + dys, secondHandColor);
    auto drawThickHand = [this, xCenter, yCenter, handColor](int dx, int dy)
    {
        int xTip = xCenter + dx;
        int yTip = yCenter + dy;
        int x = xCenter;
        int y = yCenter;
        int deltaX = abs(xTip - x);
        int stepX = (x < xTip) ? 1 : -1;
        int deltaY = -abs(yTip - y);
        int stepY = (y < yTip) ? 1 : -1;
        int error = deltaX + deltaY;

        while (true)
        {
            if (x == xTip && y == yTip)
            {
                m_spDisplay->SetPixel(x, y, handColor);
                break;
            }

            m_spDisplay->FillRect(x - 1, y - 1, 3, 3, handColor);

            int doubledError = 2 * error;
            if (doubledError >= deltaY)
            {
                error += deltaY;
                x += stepX;
            }
            if (doubledError <= deltaX)
            {
                error += deltaX;
                y += stepY;
            }
        }
    };
    drawThickHand(dxh, dyh);
    drawThickHand(dxm, dym);
    // m_spDisplay->ellipse(xCenter, yCenter, 1, 1, faceColor, true);
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
    {
        return nLine * getLineAdvance();
    }
    else
    {
        // Anchor the last line (-1) fully on screen so descenders aren't clipped,
        // then step upward by the line advance for -2, -3, ...
        return m_spDisplay->Height() - getCharHeight() + ((nLine + 1) * getLineAdvance());
    }
}

void GPS_OLED::drawText(int nLine, std::string strText, uint16_t color, bool bRightAlign, uint nRightPad)
{
    int x = (!bRightAlign) ? 0 : m_spDisplay->Width() - (strText.length() * getCharWidth());
    int y = linePos(nLine);
    x = x - nRightPad;
    m_spDisplay->Text(strText.c_str(), x, y, color);
}

void GPS_OLED::drawTextCentered(int nLine, std::string strText, uint16_t color)
{
    int x = (m_spDisplay->Width() - (strText.length() * getCharWidth())) / 2;
    int y = linePos(nLine);
    m_spDisplay->Text(strText.c_str(), x, y, color);
}

void GPS_OLED::showTime(const GPSData::Shared& spGPSData)
{
    m_spDisplay->Fill(COLOUR_BLACK);
    m_spDisplay->SetFont(get_terminus_font(18));
    if (spGPSData->strGPSTime.empty())
    {
        m_spDisplay->SetFont(get_terminus_font(12));
        drawTextCentered(2, "No GPS Time", COLOUR_WHITE);
        m_spDisplay->Show();
        return;
    }

    critical_section_enter_blocking(&m_CallbackCs);
    const bool bTextTime = m_bTextTime;
    critical_section_exit(&m_CallbackCs);

    if (!bTextTime)
    {
        drawClock(m_spDisplay->Width() / 2 - m_spDisplay->Height() / 2, 0, m_spDisplay->Height() / 2 - 1, spGPSData->strGPSTime);
    }
    else
    {
        drawTextCentered(0, TimeMgr::FormatCurrentDate(), COLOUR_WHITE);
        drawTextCentered(1, TimeMgr::FormatCurrentTimeHMS(), COLOUR_WHITE);
        drawTextCentered(-2, TimeMgr::FormatCurrentDateUTC(), COLOUR_WHITE);
        drawTextCentered(-1, TimeMgr::FormatCurrentTimeUTC() + " UTC", COLOUR_WHITE);
    }
    m_spDisplay->Show();
}

void GPS_OLED::showLatLon(const GPSData::Shared& spGPSData)
{
    m_spDisplay->Fill(COLOUR_BLACK);
    m_spDisplay->SetFont(get_terminus_font(18));
    if (!spGPSData->bHasPosition)
    {
        m_spDisplay->SetFont(get_terminus_font(12));
        drawTextCentered(2, "No GPS Position", COLOUR_WHITE);
        m_spDisplay->Show();
        return;
    }

    drawText(0, spGPSData->strLatitude, COLOUR_WHITE, true, 1);
    drawText(1, spGPSData->strLongitude, COLOUR_WHITE, true, 1);
    critical_section_enter_blocking(&m_CallbackCs);
    const bool bAltitudeFeet = m_bAltitudeFeet;
    critical_section_exit(&m_CallbackCs);

    if (bAltitudeFeet)
    {
        drawText(2, spGPSData->strAltitudeFeet, COLOUR_WHITE, true, 1);
    }
    else
    {
        drawText(2, spGPSData->strAltitude, COLOUR_WHITE, true, 1);
    }
    m_spDisplay->Show();
}

void GPS_OLED::showSpeed(const GPSData::Shared& spGPSData)
{
    m_spDisplay->Fill(COLOUR_BLACK);
    critical_section_enter_blocking(&m_CallbackCs);
    const SpeedUnit eSpeedUnit = m_eSpeedUnit;
    critical_section_exit(&m_CallbackCs);

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
        m_spDisplay->SetFont(get_terminus_font(12));
        drawTextCentered(2, "No GPS Speed", COLOUR_WHITE);
    }
    else
    {
        size_t space_pos = strSpeed.find(' ');
        if (space_pos != std::string::npos)
        {
            m_spDisplay->SetFont(get_terminus_font(32));
            m_spDisplay->Text(strSpeed.substr(0, space_pos).c_str(), 0, 0, COLOUR_WHITE, 2);
            m_spDisplay->SetFont(get_terminus_font(14));
            drawText(-1, strSpeed.substr(space_pos + 1), COLOUR_WHITE);
        }
    }
    m_spDisplay->Show();
}
