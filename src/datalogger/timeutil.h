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

/* Anything outside [2026, 2036) means "never synced" or garbage. The upper
 * bound matters: an A7672 without network time (NITZ) reports its default
 * 1970 epoch, which the 2-digit-year CCLK parse turns into 2070 -- trusting
 * that once stamped a whole day of records with 2070 timestamps. */
static constexpr time_t CLOCK_MIN_VALID = 1767225600;   /* 2026-01-01 */
static constexpr time_t CLOCK_MAX_VALID = 2082758400;   /* 2036-01-01 */

static inline bool clockValid()
{
    time_t t = time(nullptr);
    return t >= CLOCK_MIN_VALID && t < CLOCK_MAX_VALID;
}

static inline int64_t nowMs()
{
    if (!clockValid()) return 0;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static inline void setClockMs(int64_t unix_ms)
{
    if (unix_ms <  (int64_t)CLOCK_MIN_VALID * 1000) return;
    if (unix_ms >= (int64_t)CLOCK_MAX_VALID * 1000) return;   /* 2070 bug guard */
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
