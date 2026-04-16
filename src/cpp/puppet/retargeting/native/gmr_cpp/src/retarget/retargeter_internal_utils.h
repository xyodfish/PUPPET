#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "gmr/retarget/retargeter.h"

namespace gmr {
    namespace retarget_internal {

        inline std::string toLower(std::string value) {
            std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return value;
        }

        inline Eigen::Vector3d computeOrientationErrorWorld(const Eigen::Quaterniond& current, const Eigen::Quaterniond& target) {
            Eigen::Quaterniond qErr = target * current.conjugate();
            if (qErr.w() < 0.0) {
                qErr.coeffs() *= -1.0;
            }

            const Eigen::Vector3d vec = qErr.vec();
            const double vecNorm      = vec.norm();
            if (vecNorm < 1e-12) {
                return Eigen::Vector3d::Zero();
            }

            const double angle = 2.0 * std::atan2(vecNorm, qErr.w());
            return vec / vecNorm * angle;
        }

        inline HumanFrame scaleAndOffsetHumanFrameImpl(const HumanFrame& frame, const IkConfig& ikConfig,
                                                       const std::unordered_map<std::string, Eigen::Vector3d>& table1PosOffsets,
                                                       const std::unordered_map<std::string, Eigen::Quaterniond>& table1RotOffsets,
                                                       bool offsetToGround) {
            auto rootIt = frame.find(ikConfig.humanRootName);
            if (rootIt == frame.end()) {
                throw std::runtime_error("Human frame misses root body: " + ikConfig.humanRootName);
            }

            HumanFrame result;
            const Eigen::Vector3d rootPos    = rootIt->second.position;
            const Eigen::Quaterniond rootRot = rootIt->second.orientation;

            const auto rootScaleIt              = ikConfig.humanScaleTable.find(ikConfig.humanRootName);
            const double rootScale              = rootScaleIt == ikConfig.humanScaleTable.end() ? 1.0 : rootScaleIt->second;
            const Eigen::Vector3d scaledRootPos = rootScale * rootPos;
            result[ikConfig.humanRootName]      = HumanBodyState{scaledRootPos, rootRot};

            for (const auto& [bodyName, scale] : ikConfig.humanScaleTable) {
                if (bodyName == ikConfig.humanRootName) {
                    continue;
                }

                auto bodyIt = frame.find(bodyName);
                if (bodyIt == frame.end()) {
                    continue;
                }

                const Eigen::Vector3d localPos = (bodyIt->second.position - rootPos) * scale;
                result[bodyName]               = HumanBodyState{localPos + scaledRootPos, bodyIt->second.orientation};
            }

            for (auto& [bodyName, body] : result) {
                auto posIt = table1PosOffsets.find(bodyName);
                auto rotIt = table1RotOffsets.find(bodyName);
                if (posIt == table1PosOffsets.end() || rotIt == table1RotOffsets.end()) {
                    continue;
                }

                body.orientation = body.orientation * rotIt->second;
                body.position += body.orientation * posIt->second;
            }

            if (offsetToGround) {
                double lowest = std::numeric_limits<double>::infinity();
                for (const auto& [bodyName, body] : result) {
                    if (bodyName.find("Foot") == std::string::npos && bodyName.find("foot") == std::string::npos) {
                        continue;
                    }
                    lowest = std::min(lowest, body.position[2]);
                }

                if (std::isfinite(lowest)) {
                    for (auto& [bodyName, body] : result) {
                        (void)bodyName;
                        body.position[2] = body.position[2] - lowest + 0.1;
                    }
                }
            }

            return result;
        }

    }  // namespace retarget_internal
}  // namespace gmr
