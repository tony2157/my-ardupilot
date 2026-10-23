#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// ARRC: caps a stream at one message per interval on average (token bucket). A message that comes up to an
// eighth of an interval early still goes when the previous ones were on time. Zero-initialised it lets the
// first message through
struct AP_ARRC_RateCap {
    float tokens;
    uint32_t last_ms;

    bool take(uint16_t interval_ms) {
        const uint32_t now_ms = AP_HAL::millis();
        if (last_ms == 0) {
            tokens = 1.0f;
        } else {
            tokens = MIN(tokens + float(now_ms - last_ms) / MAX(interval_ms, 1U), 1.5f);
        }
        last_ms = MAX(now_ms, 1U);
        if (tokens < 0.875f) {
            return false;
        }
        tokens -= 1.0f;
        return true;
    }
};
