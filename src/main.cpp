/*
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

#include <iostream>
#include <limits>

#include "pico/stdlib.h"

#if defined(PLATFORM_PICO_W)
#include "pico/cyw43_arch.h"
#endif

#include "gps_oled.h"
#include "gps_uart.h"
#include "timemgr.h"
#include "log.h"

#if defined(GPS_ON_CORE_1) && defined(DISPLAY_ON_CORE_1)
#error "GPS_ON_CORE_1 and DISPLAY_ON_CORE_1 cannot both be defined"
#endif

#define UART0_DEVICE uart0 // Default is uart0
#define PIN_UART0_TX 0     // Default is 0
#define PIN_UART0_RX 1     // Default is 1

#if defined(ECHO_TO_UART1) && !defined(PICO_DEBUGPROBE)
#if defined(WAVESHARE_RP2040_ZERO)
#define UART1_DEVICE uart1 // uart1 for echo
#define PIN_UART1_TX 4
#define PIN_UART1_RX 5
#elif defined(PLATFORM_PICO)
#define UART1_DEVICE uart1 // uart1 for echo
#define PIN_UART1_TX 8
#define PIN_UART1_RX 9
#endif
#endif

#define UART_BAUD_RATE 9600
#define DATA_BITS      8
#define STOP_BITS      1
#define PARITY         UART_PARITY_NONE

#if defined(WAVESHARE_RP2040_ZERO)
#define I2C_DEVICE i2c1
#define PIN_SDA    2
#define PIN_SCL    3
#else
#define I2C_DEVICE i2c_default
#define PIN_SDA    PICO_DEFAULT_I2C_SDA_PIN
#define PIN_SCL    PICO_DEFAULT_I2C_SCL_PIN
#endif

#define USE_WS2812_PIN 16 // Override
// #define USE_LED_PIN 16    // Override

// GPIO pin for a button
#define PIN_BUTTON 6

namespace
{
    constexpr uint64_t timeSyncRetryIntervalSec = 5 * 60;
} // namespace

#if !defined(NDEBUG)
// Used in debug builds to check for memory leaks
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

extern "C"
{
    int _getentropy(void* buffer, size_t length)
    {
        (void)buffer;
        (void)length;
        return ENOSYS;
    }
}

int main()
{
#if !defined(PICO_DEBUGPROBE)
    stdio_usb_init();
#else
    stdio_init_all(); // Use this for debugprobe
#endif

#if !defined(NDEBUG)
    timer_hw->dbgpause = 0;
    sleep_ms(5000);
#else
    sleep_ms(1000);
#endif

#if defined(PLATFORM_PICO_W)
    if (cyw43_arch_init())
    {
        std::cout << "Failed to initialize cyw43 hardware" << std::endl;
        return 1;
    }
#endif

    TimeMgr::InitializeSingleton(TIME_ZONE); // Needed for logging timestamps
    LogInfo("Starting GPS OLED application...");

#if defined(SEEED_XIAO_RP2040)
    // Clear LED(s) on XIAO (default on)
    LED_pico ledBlue(25);  // blue
    LED_pico ledGreen(16); // green
    LED_pico ledRed(17);   // red
#endif

    // Create the LED object
    LED::Shared spLED;
#if defined(USE_WS2812_PIN)
    spLED = std::make_shared<LED_neo>(1, USE_WS2812_PIN);
    spLED->Initialize();
    spLED->SetPixel(0, led_green);
#elif defined(PICO_DEFAULT_WS2812_PIN) && !defined(USE_LED_PIN)
    spLED = std::make_shared<LED_neo>(1, PICO_DEFAULT_WS2812_PIN);
    spLED->Initialize();
    spLED->SetPixel(0, led_green);
#elif defined(USE_LED_PIN)
    spLED = std::make_shared<LED_pico>(USE_LED_PIN);
    spLED->Initialize();
    spLED->SetIgnore({led_red, led_magenta});
#elif defined(PICO_DEFAULT_LED_PIN)
    spLED = std::make_shared<LED_pico>(PICO_DEFAULT_LED_PIN);
    spLED->Initialize();
    spLED->SetIgnore({led_red, led_magenta});
#elif defined(PLATFORM_PICO_W)
    spLED = std::make_shared<LED_pico_w>(CYW43_WL_GPIO_LED_PIN);
    spLED->Initialize();
    spLED->SetIgnore({led_red, led_magenta});
#endif

    // Create the button object
    Button::Shared spButton;
#if defined(PIN_BUTTON)
    spButton = std::make_shared<Button>(PIN_BUTTON);
    spButton->Initialize();
#endif

    LogInfo("Creating GPS objects...");

    // Create the GPS object
    GPS_UART::Shared spGPS = std::make_shared<GPS_UART>();
    spGPS->SetInputUART(UART0_DEVICE, PIN_UART0_TX, PIN_UART0_RX, DATA_BITS, STOP_BITS, PARITY, UART_BAUD_RATE);
#if defined(UART1_DEVICE)
    spGPS->SetOutputUART(UART1_DEVICE, PIN_UART1_TX, PIN_UART1_RX, DATA_BITS, STOP_BITS, PARITY, UART_BAUD_RATE);
#endif

    LogInfo("Creating display objects...");
    // Create the display
    SSD1306::Shared spDisplay = std::make_shared<SSD1306_I2C>(128, 64, I2C_DEVICE, PIN_SDA, PIN_SCL);
    // Create the GPS_OLED display object
    GPS_OLED::Shared spDevice = std::make_shared<GPS_OLED>(spDisplay, spGPS, spButton);

    // Start the GPS acquisition, might be local or on core 1
    spGPS->Start();
    // Start the GPS_OLED device
    spDevice->Start();

    uint64_t nLastTimeSyncAttemptSec = std::numeric_limits<uint64_t>::max();
    GPS_OLED_Status deviceStatus;
    uint64_t prevNowSecond = TimeMgr::CurrentEpochSeconds();

    while (true)
    {
        // Set the LED state based on GPS position or other criteria
        if (spLED)
        {
            spLED->DoWork(); // Handle any outstanding work (e.g. turn off blink)
        }

        spGPS->DoWork(); // Process the GPS

        spDevice->DoWork(); // Process the GPS_OLED and display

        // Check if the device has received new data, limits the frequency of time synchronization attempts, etc.
        if (spDevice->GetStatus(deviceStatus))
        {
            // Update the system time if necessary
            if (!deviceStatus.strGpsTimeRaw.empty() && !deviceStatus.strGpsDateRaw.empty())
            {
                const uint64_t uptimeSec = time_us_64() / 1000000;
                const bool bNeverRetried = (nLastTimeSyncAttemptSec == std::numeric_limits<uint64_t>::max());
                const bool bUpdateDue = !TimeMgr::IsWallClockValid() || bNeverRetried ||
                                        (uptimeSec - nLastTimeSyncAttemptSec >= timeSyncRetryIntervalSec); // ||
                // !TimeMgr::IsGpsTimeDateWithinOneSecond(strGPSTimeRaw, strGPSDateRaw);
                if (bUpdateDue)
                {
                    nLastTimeSyncAttemptSec = uptimeSec;
                    LogInfo("Attempting GPS time sync");
                    if (TimeMgr::SetTimeFromGps(deviceStatus.strGpsTimeRaw, deviceStatus.strGpsDateRaw))
                    {
                        LogInfo("GPS time synchronized");
                    }
                }
            }
        }

        // Blink the LED here based on the device status.
        if (spLED)
        {
            uint64_t nowSecond = TimeMgr::CurrentEpochSeconds();
            if (nowSecond != prevNowSecond)
            {
                prevNowSecond = nowSecond;

                if (deviceStatus.strGpsTimeRaw.empty())
                {
                    spLED->SetPixel(0, led_red);
                    spLED->Blink_ms(0, 500);
                }
                else
                {
                    if (deviceStatus.bHasPosition)
                    {
                        spLED->SetPixel(0, deviceStatus.bExternalAntenna ? led_blue : led_green);
                    }
                    else
                    {
                        spLED->SetPixel(0, deviceStatus.bExternalAntenna ? led_magenta : led_red);
                    }
                    spLED->Blink_ms(0, 50);
                }

#if !defined(NDEBUG)
                LogInfo("Total Heap: " + std::to_string(getTotalHeap()) + "  Free Heap: " + std::to_string(getFreeHeap()));
#endif
            }
        }

        tight_loop_contents();
    }


#if defined(PLATFORM_PICO_W)
    cyw43_arch_deinit();
#endif

    LogInfo("Exiting...");
    return 0;
}
