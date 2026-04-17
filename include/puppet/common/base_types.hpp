#pragma once

#include <cstdint>
#include <string>

namespace puppet::model {

    struct Timestamp {
        int64_t sec      = 0;
        uint32_t nanosec = 0;
    };

    struct Duration {
        double sec      = 0.0;
        int64_t nanosec = 0;
    };

    struct Header {
        Timestamp timestamp;
        std::string frameId;
    };

    struct Point {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Vector3 {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Quaternion {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 1.0;
    };

    struct Pose {
        Point position;
        Quaternion orientation;
    };

    struct Twist {
        Vector3 linear;
        Vector3 angular;
    };

}  // namespace puppet::model
