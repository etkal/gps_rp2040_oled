/*
 * Time manager for wall-clock validity and time-zone offset state.
 *
 * (c) 2026 Erik Tkal
 *
 */

#pragma once

#include <cstdint>
#include <ctime>
#include <memory>
#include <string>

class TimeMgr
{
public:
    typedef std::shared_ptr<TimeMgr> Shared;

    explicit TimeMgr(std::string timeZoneName = "UTC");

    static bool ResolveTimeZoneOffset(const std::string& timeZoneName, std::time_t whenUtc, float& offsetHours, bool* pIsDst = nullptr);
    static bool IsWallClockValid();
    static uint64_t CurrentEpochSeconds();
    static bool IsGpsTimeDateWithinOneSecond(const std::string& gpsTime, const std::string& gpsDate);
    static std::string FormatCurrentTimestamp();
    static std::string FormatCurrentTimeHMS();
    static void LogInfo(const std::string& message);

    bool SetTimeFromNtp(uint32_t timeoutMs = 10000);
    bool SetTimeFromGps(const std::string& gpsTime, const std::string& gpsDate);
    bool RefreshTimeZoneOffset(std::time_t whenUtc = 0);
    bool IsValid() const;
    bool HasTimeZoneOffset() const;
    float TimeZoneOffsetHours() const;
    bool IsDst() const;
    const std::string& TimeZoneName() const;
    void SetTimeZoneName(std::string timeZoneName);

private:
    std::string m_timeZoneName;
    float m_timeZoneOffsetHours;
    bool m_isDst;
    bool m_hasTimeZoneOffset;
};

// Helper function to log messages with TimeMgr context
inline void LogInfo(const std::string& message)
{
    TimeMgr::LogInfo(message);
}
