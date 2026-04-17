#pragma once

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include "gmr/retarget/ik_config.h"

namespace gmr {

    struct HumanBodyState {
        Eigen::Vector3d position       = Eigen::Vector3d::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // world frame, wxyz semantics
    };

    using HumanFrame = std::unordered_map<std::string, HumanBodyState>;

    struct RetargetOptions {
        std::string solverName     = "qpoases";
        double damping             = 5e-1;
        double integrationTimestep = 1e-2;
        int maxIterations          = 100;
        bool useVelocityLimit      = false;
        double velocityLimit       = 3.0 * M_PI;
        double progressThreshold   = 1e-3;
    };

    struct ScalarJointCoordinate {
        int qIndex = -1;
        int vIndex = -1;
        std::string jointName;
    };

    enum class RetargetBackend {
        kPinocchio,
        kPinocchioLegacy,
        kMujoco,
        kMujocoLegacy,
    };

    RetargetBackend parseRetargetBackend(const std::string& backendName);
    const char* toString(RetargetBackend backend);

    class Retargeter {
       public:
        virtual ~Retargeter() = default;

        virtual Eigen::VectorXd retargetFrame(const HumanFrame& humanFrame, bool offsetToGround = false)      = 0;
        virtual HumanFrame prepareHumanFrame(const HumanFrame& humanFrame, bool offsetToGround = false) const = 0;
        virtual void setQpos(const Eigen::VectorXd& qpos)                                                     = 0;

        virtual const Eigen::VectorXd& currentQpos() const                               = 0;
        virtual bool hasRootFreeFlyer() const                                            = 0;
        virtual const std::vector<ScalarJointCoordinate>& scalarJointCoordinates() const = 0;
    };

    class PinocchioRetargetBackend final : public Retargeter {
       public:
        PinocchioRetargetBackend(const std::filesystem::path& robotModelPath, IkConfig ikConfig, RetargetOptions options = {});
        ~PinocchioRetargetBackend() override;

        PinocchioRetargetBackend(const PinocchioRetargetBackend&) = delete;
        PinocchioRetargetBackend& operator=(const PinocchioRetargetBackend&) = delete;

        Eigen::VectorXd retargetFrame(const HumanFrame& humanFrame, bool offsetToGround = false) override;
        HumanFrame prepareHumanFrame(const HumanFrame& humanFrame, bool offsetToGround = false) const override;
        void setQpos(const Eigen::VectorXd& qpos) override;

        const Eigen::VectorXd& currentQpos() const override;
        bool hasRootFreeFlyer() const override;
        const std::vector<ScalarJointCoordinate>& scalarJointCoordinates() const override;

       private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

    class MujocoRetargetBackend final : public Retargeter {
       public:
        MujocoRetargetBackend(const std::filesystem::path& robotModelPath, IkConfig ikConfig, RetargetOptions options = {});
        ~MujocoRetargetBackend() override;

        MujocoRetargetBackend(const MujocoRetargetBackend&) = delete;
        MujocoRetargetBackend& operator=(const MujocoRetargetBackend&) = delete;

        Eigen::VectorXd retargetFrame(const HumanFrame& humanFrame, bool offsetToGround = false) override;
        HumanFrame prepareHumanFrame(const HumanFrame& humanFrame, bool offsetToGround = false) const override;
        void setQpos(const Eigen::VectorXd& qpos) override;

        const Eigen::VectorXd& currentQpos() const override;
        bool hasRootFreeFlyer() const override;
        const std::vector<ScalarJointCoordinate>& scalarJointCoordinates() const override;

       private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

    class PinocchioLegacyRetargetBackend final : public Retargeter {
       public:
        PinocchioLegacyRetargetBackend(const std::filesystem::path& robotModelPath, IkConfig ikConfig, RetargetOptions options = {});
        ~PinocchioLegacyRetargetBackend() override;

        PinocchioLegacyRetargetBackend(const PinocchioLegacyRetargetBackend&) = delete;
        PinocchioLegacyRetargetBackend& operator=(const PinocchioLegacyRetargetBackend&) = delete;

        Eigen::VectorXd retargetFrame(const HumanFrame& humanFrame, bool offsetToGround = false) override;
        HumanFrame prepareHumanFrame(const HumanFrame& humanFrame, bool offsetToGround = false) const override;
        void setQpos(const Eigen::VectorXd& qpos) override;

        const Eigen::VectorXd& currentQpos() const override;
        bool hasRootFreeFlyer() const override;
        const std::vector<ScalarJointCoordinate>& scalarJointCoordinates() const override;

       private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

    class MujocoLegacyRetargetBackend final : public Retargeter {
       public:
        MujocoLegacyRetargetBackend(const std::filesystem::path& robotModelPath, IkConfig ikConfig, RetargetOptions options = {});
        ~MujocoLegacyRetargetBackend() override;

        MujocoLegacyRetargetBackend(const MujocoLegacyRetargetBackend&) = delete;
        MujocoLegacyRetargetBackend& operator=(const MujocoLegacyRetargetBackend&) = delete;

        Eigen::VectorXd retargetFrame(const HumanFrame& humanFrame, bool offsetToGround = false) override;
        HumanFrame prepareHumanFrame(const HumanFrame& humanFrame, bool offsetToGround = false) const override;
        void setQpos(const Eigen::VectorXd& qpos) override;

        const Eigen::VectorXd& currentQpos() const override;
        bool hasRootFreeFlyer() const override;
        const std::vector<ScalarJointCoordinate>& scalarJointCoordinates() const override;

       private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

    std::unique_ptr<Retargeter> createRetargeter(RetargetBackend backend, const std::filesystem::path& robotModelPath, IkConfig ikConfig,
                                                 RetargetOptions options = {});

}  // namespace gmr
