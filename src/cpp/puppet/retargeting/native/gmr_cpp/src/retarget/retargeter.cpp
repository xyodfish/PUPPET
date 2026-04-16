#include "gmr/retarget/retargeter.h"

#include <memory>
#include <stdexcept>

#include "retargeter_internal_utils.h"

namespace gmr {

    RetargetBackend parseRetargetBackend(const std::string& backendName) {
        const std::string lowered = retarget_internal::toLower(backendName);
        if (lowered == "pin_ik" || lowered == "pinocchio_ik" || lowered == "pinocchio") {
            return RetargetBackend::kPinocchio;
        }
        if (lowered == "pin_ik_jacobian_legacy" || lowered == "pinocchio_jacobian_legacy" || lowered == "pinocchio_legacy" ||
            lowered == "pin_legacy") {
            return RetargetBackend::kPinocchioLegacy;
        }
        if (lowered == "mujoco_se3" || lowered == "mujoco" || lowered == "se3") {
            return RetargetBackend::kMujoco;
        }
        if (lowered == "mujoco_jacobian_legacy" || lowered == "mujoco_legacy" || lowered == "legacy" || lowered == "mujoco-old" ||
            lowered == "mujoco_old") {
            return RetargetBackend::kMujocoLegacy;
        }
        throw std::runtime_error("Unsupported backend: " + backendName +
                                 ". Expected pin_ik, pin_ik_jacobian_legacy, mujoco_se3, or mujoco_jacobian_legacy.");
    }

    const char* toString(RetargetBackend backend) {
        switch (backend) {
            case RetargetBackend::kPinocchio:
                return "pin_ik";
            case RetargetBackend::kPinocchioLegacy:
                return "pin_ik_jacobian_legacy";
            case RetargetBackend::kMujoco:
                return "mujoco_se3";
            case RetargetBackend::kMujocoLegacy:
                return "mujoco_jacobian_legacy";
        }
        return "unknown";
    }

    std::unique_ptr<Retargeter> createRetargeter(RetargetBackend backend, const std::filesystem::path& robotModelPath, IkConfig ikConfig,
                                                 RetargetOptions options) {
        if (backend == RetargetBackend::kPinocchio) {
            return std::make_unique<PinocchioRetargetBackend>(robotModelPath, std::move(ikConfig), options);
        }
        if (backend == RetargetBackend::kPinocchioLegacy) {
            return std::make_unique<PinocchioLegacyRetargetBackend>(robotModelPath, std::move(ikConfig), options);
        }
        if (backend == RetargetBackend::kMujoco) {
            return std::make_unique<MujocoRetargetBackend>(robotModelPath, std::move(ikConfig), options);
        }
        if (backend == RetargetBackend::kMujocoLegacy) {
            return std::make_unique<MujocoLegacyRetargetBackend>(robotModelPath, std::move(ikConfig), options);
        }
        throw std::runtime_error("Unsupported retarget backend.");
    }

}  // namespace gmr
