#pragma once

#include <chrono>

namespace puppet::runtime {

    using TimeStampType = std::chrono::steady_clock::time_point;

    // Delay until [tStart + dtSeconds] if current time has not reached it yet.
    void sysPreciseDelay(TimeStampType tStart, double dtSeconds);

}  // namespace puppet::runtime
