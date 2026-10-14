/*
 * GPS using OLED display
 *
 * (c) 2023-2026 Erik Tkal
 *
 */

#pragma once

#include <stdio.h>
#include <memory>

#include "display.h"
#include "gpsdevice.h"

enum class DisplayMode
{
    ModeFull,
    ModeTime,
    ModePosition,
    ModeSpeed
};

// GPS_OLED class
//
// This represents an OLED display showing information from a GPS module.
// The device is initialized here and then callbacks are set up in order
// to receive data from the GPS, and the resulting data is displayed.
//
class GPS_OLED : public GPSDevice
{
public:
    typedef std::shared_ptr<GPS_OLED> Shared;

    GPS_OLED(Display::Shared spDisplay, GPS::Shared spGPS, Button::Shared spButton);
    ~GPS_OLED();

private:
    bool handleButtonEvent() override;
    void showScreenMessage(std::string strMessage) override;

    void updateUI(GPSData::Shared spGPSData);
    void drawFullUI(GPSData::Shared spGPSData);
    void showTime(const GPSData::Shared& spGPSData);
    void showLatLon(const GPSData::Shared& spGPSData);
    void showSpeed(const GPSData::Shared& spGPSData);

    void showSplashScreen() override;

    DisplayMode m_eDisplayMode {DisplayMode::ModeFull};
    SpeedUnit m_eSpeedUnit {SpeedUnit::Mph};
    bool m_bTextTime {false};
    bool m_bAltitudeFeet {true};
};
