/*
 * Pico LED class
 *
 * (c) 2025 Erik Tkal
 *
 */

#include "pico/stdlib.h"
#if defined(PLATFORM_PICO_W)
#include "pico/cyw43_arch.h"
#endif
#include "led.h"
#include "ws2812.pio.h"

static inline void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

// Instead of sleeping, set an alarm to turn off the LED
static int64_t offAlarmCallback(alarm_id_t id, void* user_data)
{
    LED* pThis = reinterpret_cast<LED*>(user_data);
    if (nullptr != pThis)
    {
        pThis->Off();
    }
    return false; // don't restart the timer.
}

void LED::Blink_ms(uint duration)
{
    On();
    add_alarm_in_ms(duration, offAlarmCallback, reinterpret_cast<void*>(this), true);
}


LED_pico::LED_pico(uint pin)
    : m_nPin(pin),
      m_nColor(led_white)
{
    gpio_init(m_nPin);
    gpio_set_dir(m_nPin, GPIO_OUT);
    Off();
}

LED_pico::~LED_pico()
{
    Off();
    gpio_deinit(m_nPin);
}

void LED_pico::On()
{
    for (auto i : m_vIgnore)
    {
        if (i == m_nColor)
        {
            return;
        }
    }
    gpio_put(m_nPin, LED_ON);
}

void LED_pico::Off()
{
    gpio_put(m_nPin, LED_OFF);
}

void LED_pico::SetPixel(uint idx, uint32_t color)
{
    m_nColor = color;
}

void LED_pico::SetIgnore(std::vector<uint32_t> vIgnore)
{
    m_vIgnore = vIgnore;
}


LED_neo::LED_neo(uint numLEDs, uint pin, uint powerPin, bool bIsRGBW)
    : m_nPin(pin),
      m_nPowerPin(powerPin),
      m_nNumLEDs(numLEDs),
      m_bIsRGBW(bIsRGBW)
{
    Off();
}

LED_neo::~LED_neo()
{
    Off();
    if (0 != m_nPowerPin)
    {
        gpio_put(m_nPowerPin, 0);
        gpio_deinit(m_nPowerPin);
    }
}

void LED_neo::Initialize()
{
    PIO pio     = pio0;
    int sm      = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, m_nPin, 800000, m_bIsRGBW);

    if (0 != m_nPowerPin)
    {
        gpio_init(m_nPowerPin);
        gpio_set_dir(m_nPowerPin, GPIO_OUT);
        gpio_put(m_nPowerPin, 1);
    }

    m_vPixels.resize(m_nNumLEDs);
}

void LED_neo::On()
{
    for (size_t i = 0; i < m_nNumLEDs; ++i)
    {
        put_pixel(m_vPixels[i]);
    }
}

void LED_neo::Off()
{
    for (size_t i = 0; i < m_nNumLEDs; ++i)
    {
        put_pixel(0);
    }
}

void LED_neo::SetPixel(uint idx, uint32_t color)
{
    m_vPixels[idx] = color;
}


#if defined(PLATFORM_PICO_W)
LED_pico_w::LED_pico_w(uint pin)
    : m_nPin(pin),
      m_nColor(led_white)
{
    Off();
}

LED_pico_w::~LED_pico_w()
{
    Off();
}

void LED_pico_w::On()
{
    for (auto i : m_vIgnore)
    {
        if (i == m_nColor)
        {
            return;
        }
    }
    cyw43_arch_gpio_put(m_nPin, 1);
}

void LED_pico_w::Off()
{
    cyw43_arch_gpio_put(m_nPin, 0);
}

void LED_pico_w::SetPixel(uint idx, uint32_t color)
{
    m_nColor = color;
}

void LED_pico_w::SetIgnore(std::vector<uint32_t> vIgnore)
{
    m_vIgnore = vIgnore;
}
#endif
