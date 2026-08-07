/*
 * timeutil.h  --  wall-clock helpers.
 *
 * The ESP32-C6 system clock runs off the RTC timer, which keeps counting through
 * deep sleep -- so once settimeofday() has been called (from modem NITZ time or
 * SNTP), time() stays valid across every sleep until the next power loss.
 */
#pragma once

#include <time.h>
#include <sys/time.h>
#include "config.h"

/* Anything before 2026 means "never synced" (cold boot starts at epoch 0). */
static constexpr time_t CLOCK_MIN_VALID = 1767225600;   /* 2026-01-01 */

static inline bool clockValid() { return time(nullptr) >= CLOCK_MIN_VALID; }

static inline int64_t nowMs()
{
    if (!clockValid()) return 0;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static inline void setClockMs(int64_t unix_ms)
{
    if (unix_ms < (int64_t)CLOCK_MIN_VALID * 1000) return;
    struct timeval tv;
    tv.tv_sec  = (time_t)(unix_ms / 1000);
    tv.tv_usec = (suseconds_t)((unix_ms % 1000) * 1000);
    settimeofday(&tv, nullptr);
}

/* Local calendar day number (for "how much sun did we harvest today"). -1 if the
 * clock has never been synced. */
static inline int32_t localDayNum()
{
    if (!clockValid()) return -1;
    return (int32_t)((time(nullptr) + (time_t)TZ_OFFSET_MIN * 60) / 86400);
}
