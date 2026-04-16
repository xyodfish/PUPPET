#pragma once

#include <string>
#include <vector>

namespace puppet::target {

    struct GroupTarget {
        std::string bodyGroup;
        std::vector<std::string> jointNames;
        std::vector<double> jointPosition;
        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;
    };

    struct FinalTarget {
        uint64_t sequenceId = 0;
        std::vector<GroupTarget> groups;
    };

}  // namespace puppet::target
