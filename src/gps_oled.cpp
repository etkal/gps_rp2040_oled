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
#include <vector>

#include "pico/stdlib.h"
#include "pico/double.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#include "ssd1306.h"
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

struct DisplayModeStateMachine
{
    DisplayMode m_currentMode;
    ButtonEvent m_lastEvent;
    DisplayMode m_nextMode;
};

std::vector<DisplayModeStateMachine> g_displayModeStateMachine = {
    {DisplayMode::ModeFull,          ButtonEvent::Tap,       DisplayMode::ModeTimeClock    },
    {DisplayMode::ModeTimeClock,     ButtonEvent::Press,     DisplayMode::ModeTimeText     },
    {DisplayMode::ModeTimeText,      ButtonEvent::Press,     DisplayMode::ModeTimeClock    },
    {DisplayMode::ModeTimeClock,     ButtonEvent::Tap,       DisplayMode::ModeLatLonAlt    },
    {DisplayMode::ModeTimeText,      ButtonEvent::Tap,       DisplayMode::ModeLatLonAlt    },
    {DisplayMode::ModeLatLonAlt,     ButtonEvent::Press,     DisplayMode::ModeLatLonAltFeet},
    {DisplayMode::ModeLatLonAltFeet, ButtonEvent::Press,     DisplayMode::ModeLatLonAlt    },
    {DisplayMode::ModeLatLonAlt,     ButtonEvent::Tap,       DisplayMode::ModeSpeedMph     },
    {DisplayMode::ModeLatLonAltFeet, ButtonEvent::Tap,       DisplayMode::ModeSpeedMph     },
    {DisplayMode::ModeSpeedMph,      ButtonEvent::Press,     DisplayMode::ModeSpeedKph     },
    {DisplayMode::ModeSpeedKph,      ButtonEvent::Press,     DisplayMode::ModeSpeedKts     },
    {DisplayMode::ModeSpeedKts,      ButtonEvent::Press,     DisplayMode::ModeSpeedMph     },
    {DisplayMode::ModeSpeedMph,      ButtonEvent::Tap,       DisplayMode::ModeFull         },
    {DisplayMode::ModeSpeedKph,      ButtonEvent::Tap,       DisplayMode::ModeFull         },
    {DisplayMode::ModeSpeedKts,      ButtonEvent::Tap,       DisplayMode::ModeFull         },
    {DisplayMode::ModeAny,           ButtonEvent::LongPress, DisplayMode::ModeFull         },
};

GPS_OLED::GPS_OLED(SSD1306::Shared spDisplay, GPS::Shared spGPS, LED::Shared spLED, Button::Shared spButton)
    : m_spDisplay(spDisplay),
      m_spGPS(spGPS),
      m_spLED(spLED),
      m_spButton(spButton),
      m_nLastTimeSyncAttemptSec(std::numeric_limits<uint64_t>::max())
{
    queue_init(&m_qIncomingGPSData, sizeof(GPSData::Shared*), 10); // Initialize the queue with a capacity of 10
    queue_init(&m_qDisplayGPSData, sizeof(GPSData::Shared*), 10);  // Initialize the queue with a capacity of 10
}

GPS_OLED::~GPS_OLED()
{
}

void GPS_OLED::Initialize()
{
    m_spDisplay->Initialize();

    // Initialize display with desired font (best is Terminus 12, anything larger is not recommended)
    m_spDisplay->SetFont(get_terminus_font(12));

    m_spDisplay->SetContrast(0x10);
    showScreenMessage("Waiting for GPS data");

    m_spGPS->SetGpsDataCallback(this, gpsDataCB);
    m_spGPS->SetMessageCallback(this, messageCB);
    m_spButton->SetEventCallback(this, buttonEventCB);

    m_spIdleTimer = std::make_shared<AlarmTimer>([this]() {
        m_bShowWaitingForGPS = true;
    });
}

void GPS_OLED::Run()
{
#if defined(GPS_ON_CORE_1)
    // Start GPS processing loop on processor core 1
    static auto sm_spGPS = m_spGPS; // Capture shared pointer for use in lambda
    multicore_launch_core1([]() {
        GPS::Shared spGPS = sm_spGPS;
        // Initialize and run the GPS processing loop on core 1
        LogInfo("Starting GPS processing on core 1");
        spGPS->Initialize();
        spGPS->Run();
    });
#else
    // If we are not using multicore, we can run the GPS processing from the display loop
    LogInfo("Starting GPS processing on core 0");
    m_spGPS->Initialize();
#endif // GPS_ON_CORE_1

#if defined(DISPLAY_ON_CORE_1)
    static auto sm_pThis = this; // Capture pointer for use in lambda
    multicore_launch_core1([]() {
        GPS_OLED* pThis = sm_pThis;
        while (true)
        {
            static GPSData::Shared spLastGPSData;
            multicore_fifo_pop_blocking(); // Wait for signal from core 0
            GPSData::Shared spGPSData = dequeueLatestGPSData(pThis->m_qDisplayGPSData);
            if (spGPSData)
            {
                pThis->updateUI(spGPSData);
                spLastGPSData = spGPSData;
            }
            else if (spLastGPSData)
            {
                pThis->updateUI(spLastGPSData);
            }
        }
    });
#endif

    // Main loop for updating the display
    while (true)
    {
        if (handleButtonEvent() && m_spLastGPSData)
        {
            // Redraw immediately with the last known data rather than waiting for the next GPS update
#if defined(DISPLAY_ON_CORE_1)
            multicore_fifo_push_blocking(0); // Signal core 1 to redraw using its last received data
#else
            updateUI(m_spLastGPSData);
#endif
        }
        m_spLED->DoWork();
#if !defined(GPS_ON_CORE_1)
        m_spGPS->RunOnce();
#endif
        GPSData::Shared spGPSData = dequeueLatestGPSData(m_qIncomingGPSData);

        if (spGPSData)
        {
            LogInfo("GPS_OLED - Processing new GPS data");
            // Perform operations that need to run on core 0 (main core)
            updateTime(spGPSData->strGPSTimeRaw, spGPSData->strGPSDateRaw);
            // LogInfo("GPS: " + spGPSData->strGPSTimeRaw + " Clock: " + TimeMgr::FormatCurrentTimeUTC());
            blinkLED(spGPSData->bHasPosition, spGPSData->bExternalAntenna);
            spGPSData->strVsys = getVsysVoltage();
            m_spLastGPSData = spGPSData; // Kept on core 0 only, for immediate button-triggered redraws
#if defined(DISPLAY_ON_CORE_1)
            // Hand ownership across cores; queue holds a heap-allocated shared_ptr wrapper only
            if (enqueueGPSData(m_qDisplayGPSData, spGPSData))
            {
                multicore_fifo_push_blocking(0);
            }
#else
            updateUI(spGPSData);
#endif
            m_spIdleTimer->Start(10000); // Reset the idle timer to 10 seconds
        }
        if (m_bShowWaitingForGPS)
        {
            LogInfo("GPS_OLED - No GPS data received showing waiting message");
            showScreenMessage("Waiting for GPS data");
            m_bShowWaitingForGPS = false;
        }
    }
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
    if (ButtonEvent::None == m_eLastButtonEvent)
    {
        return false;
    }
    if (ButtonEvent::Tap == m_eLastButtonEvent)
    {
        LogInfo("GPS_OLED - Button tap detected");
    }
    else if (ButtonEvent::Press == m_eLastButtonEvent)
    {
        LogInfo("GPS_OLED - Button press detected");
    }
    else if (ButtonEvent::LongPress == m_eLastButtonEvent)
    {
        LogInfo("GPS_OLED - Long button press detected");
    }

    for (auto entry : g_displayModeStateMachine)
    {
        if ((entry.m_currentMode == m_eDisplayMode && entry.m_lastEvent == m_eLastButtonEvent) ||
            (entry.m_currentMode == DisplayMode::ModeAny && entry.m_lastEvent == m_eLastButtonEvent))
        {
            m_eDisplayMode = entry.m_nextMode;
            break;
        }
    }

    // Reset the last button event after handling
    m_eLastButtonEvent = ButtonEvent::None;
    return true; // trigger a display update on state change
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

    // Deep copy the GPSData once here; all subsequent hand-offs share ownership of this copy.
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
}

void GPS_OLED::showScreenMessage(std::string strMessage)
{
    m_spDisplay->Fill(COLOUR_BLACK);
    m_spDisplay->SetFont(get_terminus_font(12));
    drawText(0, strMessage, COLOUR_WHITE, false, 0);
    m_spDisplay->Show();
}

void GPS_OLED::blinkLED(bool bHasPosition, bool bExternalAntenna)
{
    if (m_spLED)
    {
        if (bHasPosition)
        {
            m_spLED->SetPixel(0, bExternalAntenna ? led_blue : led_green);
        }
        else
        {
            m_spLED->SetPixel(0, bExternalAntenna ? led_magenta : led_red);
        }
        m_spLED->Blink_ms(20);
    }
}

void GPS_OLED::updateTime(std::string strGPSTimeRaw, std::string strGPSDateRaw)
{
    // Update the system time if necessary
    if (!strGPSTimeRaw.empty() && !strGPSDateRaw.empty())
    {
        const uint64_t uptimeSec = time_us_64() / 1000000;
        const bool bNeverRetried = (m_nLastTimeSyncAttemptSec == std::numeric_limits<uint64_t>::max());
        const bool bUpdateDue = !TimeMgr::IsWallClockValid() || bNeverRetried ||
                                (uptimeSec - m_nLastTimeSyncAttemptSec >= timeSyncRetryIntervalSec) ||
                                !TimeMgr::IsGpsTimeDateWithinOneSecond(strGPSTimeRaw, strGPSDateRaw);
        if (bUpdateDue)
        {
            m_nLastTimeSyncAttemptSec = uptimeSec;
            LogInfo("Attempting GPS time sync");
            if (TimeMgr::SetTimeFromGps(strGPSTimeRaw, strGPSDateRaw))
            {
                LogInfo("GPS time synchronized");
            }
        }
    }
}

std::string GPS_OLED::getVsysVoltage()
{
    std::string strVsys;
#if defined(PLATFORM_PICO) && defined(DISPLAY_VSYS_VOLTAGE) // Only the Raspberry Pi Pico series have a VSYS voltage monitor
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
    LogInfo("getVsysVoltage: " + strVsys);
#endif
    return strVsys;
}

// Update the UI with the latest GPS data.
void GPS_OLED::updateUI(GPSData::Shared spGPSData)
{
    LogInfo("GPS_OLED - updateUI() called");

    switch (m_eDisplayMode)
    {
    case DisplayMode::ModeFull:
        drawFullUI(spGPSData);
        return;
    case DisplayMode::ModeTimeClock:
    case DisplayMode::ModeTimeText:
        showTime(spGPSData);
        return;
    case DisplayMode::ModeLatLonAlt:
    case DisplayMode::ModeLatLonAltFeet:
        showLatLon(spGPSData);
        return;
    case DisplayMode::ModeSpeedKts:
    case DisplayMode::ModeSpeedKph:
    case DisplayMode::ModeSpeedMph:
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
        drawText(2, spGPSData->strAltitude, COLOUR_WHITE, true, X_PAD);
    }
    if (!spGPSData->strGPSTime.empty())
    {
        drawText(-1, spGPSData->strGPSTime, COLOUR_WHITE, true, X_PAD);
    }

    // blit the framebuf to the display
    m_spDisplay->Show();

#if !defined(NDEBUG)
    LogInfo("Total Heap: " + std::to_string(getTotalHeap()) + "  Free Heap: " + std::to_string(getFreeHeap()));
#endif
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
    m_spDisplay->Line(xCenter, yCenter, xCenter + dxh, yCenter + dyh, handColor);
    m_spDisplay->Line(xCenter, yCenter, xCenter + dxm, yCenter + dym, handColor);
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
        return m_spDisplay->Height() + (nLine * getLineAdvance());
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

    switch (m_eDisplayMode)
    {
    case DisplayMode::ModeTimeClock:
        drawClock(m_spDisplay->Width() / 2 - m_spDisplay->Height() / 2, 0, m_spDisplay->Height() / 2 - 1, spGPSData->strGPSTime);
        break;
    case DisplayMode::ModeTimeText:
    default:
        drawTextCentered(0, TimeMgr::FormatCurrentDate(), COLOUR_WHITE);
        drawTextCentered(1, TimeMgr::FormatCurrentTimeHMS(), COLOUR_WHITE);
        drawTextCentered(-2, TimeMgr::FormatCurrentDateUTC(), COLOUR_WHITE);
        drawTextCentered(-1, TimeMgr::FormatCurrentTimeUTC() + " UTC", COLOUR_WHITE);
        break;
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
    switch (m_eDisplayMode)
    {
    case DisplayMode::ModeLatLonAlt:
        drawText(2, spGPSData->strAltitude, COLOUR_WHITE, true, 1);
        break;
    case DisplayMode::ModeLatLonAltFeet:
        drawText(2, spGPSData->strAltitudeFeet, COLOUR_WHITE, true, 1);
        break;
    default:
        break;
    }
    m_spDisplay->Show();
}

void GPS_OLED::showSpeed(const GPSData::Shared& spGPSData)
{
    m_spDisplay->Fill(COLOUR_BLACK);
    m_spDisplay->SetFont(get_terminus_font(24));
    std::string strSpeed;
    switch (m_eDisplayMode)
    {
    case DisplayMode::ModeSpeedMph:
        strSpeed = spGPSData->strSpeedMph;
        break;
    case DisplayMode::ModeSpeedKph:
        // Convert
        strSpeed = spGPSData->strSpeedKph;
        break;
    case DisplayMode::ModeSpeedKts:
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
        drawTextCentered(1, strSpeed, COLOUR_WHITE);
    }
    m_spDisplay->Show();
}
