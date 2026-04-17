#include "puppet/runtime/time_utils.hpp"

#include <thread>

namespace puppet::runtime {

    void sysPreciseDelay(TimeStampType tStart, double dtSeconds) {
        const auto tNow     = std::chrono::steady_clock::now();
        const auto elapsed  = std::chrono::duration<double>(tNow - tStart).count();
        const auto remained = dtSeconds - elapsed;
        if (remained > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(remained));
        }
    }

}  // namespace puppet::runtime
