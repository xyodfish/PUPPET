#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "puppet/device/device_provider.hpp"

#include "GalbotRemoteOperate/RemoteOperate.h"
#include "GalbotRemoteOperate/System.h"

namespace puppet::device {

    class ScaledDeviceProvider : public IDeviceProvider {
       public:
        bool initialize(const YAML::Node& configNode, const std::string& frameId, const std::string& sourceId, std::string* error) override;

        bool nextFrame(uint64_t sequenceId, model::PrimitiveFrame* frame, std::string* error) override;

        int suggestedLoopHz() const override;

       private:
        static constexpr size_t kArmMotorCount = static_cast<size_t>(RemoteOperate::M_NUM);

        struct ArmRuntimeState {
            std::string armName;
            std::string stateName;
            model::BodyGroup bodyGroup = model::BodyGroup::kWholeBody;
            std::vector<std::string> jointNames;
            std::vector<double> jointInit;
            std::vector<double> jointScale;
            std::vector<double> positions;
            std::vector<double> velocities;
            std::vector<double> efforts;
            std::vector<uint32_t> errors;
            bool enabled = true;
            bool updated = false;
        };

        enum class ArmSyncMode {
            None                      = 0,
            DirectSync                = 1,
            JointSpaceIncremental     = 2,
            CartesianSpaceIncremental = 3,
        };

        struct ArmControlState {
            bool directSync           = false;
            bool inTakeover           = false;
            bool useCartesianTakeover = false;
            ArmSyncMode effectiveMode = ArmSyncMode::None;
            bool modeUpdated          = true;
        };

        bool parseArmConfig(const YAML::Node& armNode, const std::string& armName, ArmRuntimeState* state, std::string* error);
        static model::BodyGroup parseBodyGroup(const std::string& value);
        static double wrapJointAngle(double value);
        static double applyDeadZone(double value, double deadZone);
        static double mapJoyAxis(double value, double threshold);
        static double amplifyAndClamp(double value, double amplify, double upperBound);
        void updateRightPole(const RemoteOperate::Pole& poleInfo);
        void updateLeftPole(const RemoteOperate::Pole& poleInfo);
        void updateButtonAlias(const std::string& aliasName, uint8_t statusValue);
        void updateArmSyncMode(bool forceEmit = false);

        void remoteCallback(uint8_t cmdValue, uint8_t functionValue, std::vector<uint8_t> payload);
        void systemCallback(uint8_t cmdValue, uint8_t functionValue, std::vector<uint8_t> payload);
        void updateArmState(const RemoteOperate::armInfo& armInfo, ArmRuntimeState* state);
        void updateGripperFromTrigger(const RemoteOperate::Pole& triggerInfo, bool isLeft);
        void tryReconnect();

        std::string frameId_    = "torso_link";
        std::string sourceId_   = "scaled_device";
        std::string semantic_   = "scaled_device_joint_state";
        std::string mode_       = "joint_state_sync";
        std::string pipelineId_ = "scaled_device_pipeline";

        std::string busName_                  = "can0";
        int loopHz_                           = 100;
        double triggerThreshold_              = 8.0;
        double offlineTimeoutSec_             = 0.5;
        double reconnectIntervalSec_          = 1.0;
        std::vector<double> joyAxisThreshold_ = std::vector<double>(6, 0.2);
        std::vector<double> joyAxisDeadZone_  = std::vector<double>(6, 0.05);
        std::vector<double> chassisVelAmpl_   = {0.5, 0.5, 0.5};
        std::vector<double> chassisVelUb_     = {0.6, 0.6, 0.6};
        bool leftComboLongPressed_            = false;
        bool rightComboLongPressed_           = false;
        std::vector<double> joyAxesState_     = std::vector<double>(8, 0.0);
        bool chassisMotionUpdated_            = false;
        bool joyScalarUpdated_                = false;
        std::map<std::string, std::string> buttonAliasByRawKey_;
        bool leftSoftEmergencyStop_    = false;
        bool rightSoftEmergencyStop_   = false;
        bool softEmergencyStopUpdated_ = false;
        ArmControlState leftArmControl_;
        ArmControlState rightArmControl_;

        bool leftGripperEnabled_           = true;
        bool rightGripperEnabled_          = true;
        std::string leftGripperJointName_  = "left_gripper_joint1";
        std::string rightGripperJointName_ = "right_gripper_joint1";
        double leftGripperPos_             = 0.0;
        double rightGripperPos_            = 0.0;
        bool leftGripperUpdated_           = false;
        bool rightGripperUpdated_          = false;

        ArmRuntimeState leftArm_;
        ArmRuntimeState rightArm_;

        mutable std::mutex dataMutex_;
        std::chrono::steady_clock::time_point lastDataTime_;
        std::chrono::steady_clock::time_point lastReconnectTryTime_;

        std::unique_ptr<System> system_;
        std::unique_ptr<RemoteOperate> remote_;
    };

}  // namespace puppet::device
