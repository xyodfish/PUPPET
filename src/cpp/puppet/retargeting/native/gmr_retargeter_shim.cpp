#include <stdexcept>
#include <string>

#include <gmr/retarget/retargeter.h>

namespace gmr {

    RetargetBackend parseRetargetBackend(const std::string& backendName) {
        if (backendName == "pin_ik" || backendName == "pinocchio") {
            return RetargetBackend::kPinocchio;
        }
        if (backendName == "pin_ik_jacobian_legacy" || backendName == "pinocchio_legacy") {
            return RetargetBackend::kPinocchioLegacy;
        }
        throw std::invalid_argument("Unsupported backend in PUPPET shim: " + backendName);
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
            default:
                return "unknown";
        }
    }

    std::unique_ptr<Retargeter> createRetargeter(RetargetBackend backend, const std::filesystem::path& robotModelPath, IkConfig ikConfig,
                                                 RetargetOptions options) {
        switch (backend) {
            case RetargetBackend::kPinocchio:
                return std::make_unique<PinocchioRetargetBackend>(robotModelPath, std::move(ikConfig), options);
            case RetargetBackend::kPinocchioLegacy:
                return std::make_unique<PinocchioLegacyRetargetBackend>(robotModelPath, std::move(ikConfig), options);
            default:
                throw std::runtime_error("Only pinocchio backends are enabled in PUPPET GMR source integration");
        }
    }

}  // namespace gmr
