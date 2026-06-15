#pragma once

#include <string>
#include <string_view>
#include <utility>

#include <glog/logging.h>

#define PUPPET_LOG(level, event, module, op) LOG(level) << "event=" << event << " module=" << module << " op=" << op

#define PUPPET_LOG_EVERY_N(level, n, event, module, op) \
    LOG_EVERY_N(level, n) << "event=" << event << " module=" << module << " op=" << op

#define PUPPET_VLOG(level, event, module, op) VLOG(level) << "event=" << event << " module=" << module << " op=" << op

namespace puppet::common {

    inline bool Fail(std::string& error, std::string message) {
        error = std::move(message);
        return false;
    }

    inline bool WrapError(std::string& error, std::string_view prefix) {
        error = std::string(prefix) + error;
        return false;
    }

    inline void ClearError(std::string& error) {
        error.clear();
    }

}  // namespace puppet::common
