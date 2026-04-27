#include "puppet/device/scaled/scaled_device_provider.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace puppet::device {

    namespace scaled_device_provider_detail {

        constexpr size_t kJoyAxisLeftX        = 0;
        constexpr size_t kJoyAxisLeftY        = 1;
        constexpr size_t kJoyAxisRightX       = 2;
        constexpr size_t kJoyAxisRightY       = 3;
        constexpr size_t kJoyAxisLeftTrigger  = 4;
        constexpr size_t kJoyAxisRightTrigger = 5;

        constexpr size_t kJoyStateLegUd       = 0;
        constexpr size_t kJoyStateWaistLrRot  = 1;
        constexpr size_t kJoyStateWaistFbTran = 2;
        constexpr size_t kJoyStateChassisFb   = 3;
        constexpr size_t kJoyStateChassisLr   = 4;
        constexpr size_t kJoyStateChassisRot  = 5;
        constexpr size_t kJoyStateHeadYaw     = 6;
        constexpr size_t kJoyStateHeadPitch   = 7;

        constexpr std::array<const char*, 18> kRawButtonKeys = {
            "key1L", "key2L", "key3L", "key4L", "key5L", "key6L", "key7L", "key8L", "key9L",
            "key1R", "key2R", "key3R", "key4R", "key5R", "key6R", "key7R", "key8R", "key9R",
        };

        std::vector<std::string> readStringVector(const YAML::Node& node, const std::string& key,
                                                  const std::vector<std::string>& fallback) {
            if (!node[key] || !node[key].IsSequence()) {
                return fallback;
            }
            std::vector<std::string> values;
            for (const auto& item : node[key]) {
                values.push_back(item.as<std::string>());
            }
            return values;
        }

        bool readOptionalDoubleVector(const YAML::Node& node, const std::string& key, std::vector<double>* values) {
            if (!node[key] || !node[key].IsSequence()) {
                return false;
            }
            values->clear();
            for (const auto& item : node[key]) {
                values->push_back(item.as<double>());
            }
            return true;
        }

    }  // namespace scaled_device_provider_detail

    bool ScaledDeviceProvider::parseArmConfig(const YAML::Node& armNode, const std::string& armName, ArmRuntimeState* state,
                                              std::string* error) {
        state->armName   = armName;
        state->stateName = armName + "_joint_state";
        if (armNode["state_name"]) {
            state->stateName = armNode["state_name"].as<std::string>();
        }
        if (armNode["enabled"]) {
            state->enabled = armNode["enabled"].as<bool>();
        }

        std::string bodyGroup = "whole_body";
        if (armNode["body_group"]) {
            bodyGroup = armNode["body_group"].as<std::string>();
        }
        state->bodyGroup = parseBodyGroup(bodyGroup);

        if (armNode["joint_names"] && armNode["joint_names"].IsSequence()) {
            state->jointNames = scaled_device_provider_detail::readStringVector(armNode, "joint_names", {});
        }
        if (state->jointNames.empty()) {
            if (armName == "left_arm") {
                state->jointNames = {
                    "left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4",
                    "left_arm_joint5", "left_arm_joint6", "left_arm_joint7",
                };
            } else {
                state->jointNames = {
                    "right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4",
                    "right_arm_joint5", "right_arm_joint6", "right_arm_joint7",
                };
            }
        }

        state->jointInit  = std::vector<double>(state->jointNames.size(), 0.0);
        state->jointScale = std::vector<double>(state->jointNames.size(), 1.0);
        if (!scaled_device_provider_detail::readOptionalDoubleVector(armNode, "joint_init", &state->jointInit)) {
            scaled_device_provider_detail::readOptionalDoubleVector(armNode, "init_angle", &state->jointInit);
        }
        if (!scaled_device_provider_detail::readOptionalDoubleVector(armNode, "joint_scale", &state->jointScale)) {
            scaled_device_provider_detail::readOptionalDoubleVector(armNode, "scale_value", &state->jointScale);
        }

        if (state->jointInit.size() != state->jointNames.size()) {
            *error = armName + ".joint_init size mismatch";
            return false;
        }
        if (state->jointScale.size() != state->jointNames.size()) {
            *error = armName + ".joint_scale size mismatch";
            return false;
        }

        state->positions  = std::vector<double>(state->jointNames.size(), 0.0);
        state->velocities = std::vector<double>(state->jointNames.size(), 0.0);
        state->efforts    = std::vector<double>(state->jointNames.size(), 0.0);
        state->errors     = std::vector<uint32_t>(state->jointNames.size(), 0);
        state->updated    = false;
        return true;
    }

    model::BodyGroup ScaledDeviceProvider::parseBodyGroup(const std::string& value) {
        if (value == "left_arm") {
            return model::BodyGroup::kLeftArm;
        }
        if (value == "right_arm") {
            return model::BodyGroup::kRightArm;
        }
        if (value == "left_hand" || value == "left_gripper") {
            return model::BodyGroup::kLeftGripper;
        }
        if (value == "right_hand" || value == "right_gripper") {
            return model::BodyGroup::kRightGripper;
        }
        return model::BodyGroup::kWholeBody;
    }

    double ScaledDeviceProvider::wrapJointAngle(double value) {
        constexpr double kPi = 3.14159265358979323846;
        while (value > kPi) {
            value -= 2.0 * kPi;
        }
        while (value < -kPi) {
            value += 2.0 * kPi;
        }
        return value;
    }

    double ScaledDeviceProvider::applyDeadZone(double value, double deadZone) {
        if (std::abs(value) < std::abs(deadZone)) {
            return 0.0;
        }
        return value;
    }

    double ScaledDeviceProvider::mapJoyAxis(double value, double threshold) {
        const double maxAmpl = (1.0 - threshold) * (1.0 - threshold);
        if (value >= threshold) {
            return (value - threshold) * (value - threshold) / maxAmpl;
        }
        if (value <= -threshold) {
            return -(value + threshold) * (value + threshold) / maxAmpl;
        }
        return 0.0;
    }

    double ScaledDeviceProvider::amplifyAndClamp(double value, double amplify, double upperBound) {
        const double absBound = std::abs(upperBound);
        return std::clamp(value * amplify, -absBound, absBound);
    }

    void ScaledDeviceProvider::updateArmSyncMode(bool forceEmit) {
        auto calcMode = [](const ArmControlState& state) {
            if (state.directSync) {
                return ArmSyncMode::DirectSync;
            }
            if (state.inTakeover) {
                return state.useCartesianTakeover ? ArmSyncMode::CartesianSpaceIncremental : ArmSyncMode::JointSpaceIncremental;
            }
            return ArmSyncMode::None;
        };

        const ArmSyncMode leftMode  = calcMode(leftArmControl_);
        const ArmSyncMode rightMode = calcMode(rightArmControl_);

        if (forceEmit || leftMode != leftArmControl_.effectiveMode) {
            leftArmControl_.effectiveMode = leftMode;
            leftArmControl_.modeUpdated   = true;
        }
        if (forceEmit || rightMode != rightArmControl_.effectiveMode) {
            rightArmControl_.effectiveMode = rightMode;
            rightArmControl_.modeUpdated   = true;
        }
    }

    void ScaledDeviceProvider::updateRightPole(const RemoteOperate::Pole& poleInfo) {
        const double lr = (50.0 - static_cast<double>(poleInfo.y)) / 50.0;
        const double fb = (static_cast<double>(poleInfo.x) - 50.0) / 50.0;

        const double rXThreshold = joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisRightX];
        const double rYThreshold = joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisRightY];
        const double rXDeadZone  = joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisRightX];
        const double rYDeadZone  = joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisRightY];

        if (!leftComboLongPressed_) {
            if (std::fabs(fb) >= rXThreshold && std::fabs(lr) >= rYThreshold) {
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisFb] =
                    amplifyAndClamp(applyDeadZone(mapJoyAxis(fb, rXThreshold), rXDeadZone), chassisVelAmpl_[0], chassisVelUb_[0]);
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisLr] =
                    amplifyAndClamp(applyDeadZone(mapJoyAxis(lr, rYThreshold), rYDeadZone), chassisVelAmpl_[1], chassisVelUb_[1]);
            } else if (std::fabs(fb) >= rXThreshold) {
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisFb] =
                    amplifyAndClamp(applyDeadZone(mapJoyAxis(fb, rXThreshold), rXDeadZone), chassisVelAmpl_[0], chassisVelUb_[0]);
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisLr] = 0.0;
            } else if (std::fabs(lr) >= rYThreshold) {
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisFb] = 0.0;
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisLr] =
                    amplifyAndClamp(applyDeadZone(mapJoyAxis(lr, rYThreshold), rYDeadZone), chassisVelAmpl_[1], chassisVelUb_[1]);
            } else {
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisFb] = 0.0;
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisLr] = 0.0;
            }

            joyAxesState_[scaled_device_provider_detail::kJoyStateHeadYaw]   = 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateHeadPitch] = 0.0;
        } else {
            joyAxesState_[scaled_device_provider_detail::kJoyStateHeadYaw] =
                (std::abs(lr) < rYThreshold) ? 0.0 : mapJoyAxis(lr, rYThreshold);
            joyAxesState_[scaled_device_provider_detail::kJoyStateHeadPitch] =
                (std::abs(fb) < rXThreshold) ? 0.0 : mapJoyAxis(-fb, rXThreshold);
        }

        if (leftComboLongPressed_ || rightComboLongPressed_) {
            joyAxesState_[scaled_device_provider_detail::kJoyStateChassisFb]  = 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateChassisLr]  = 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateChassisRot] = 0.0;
        }

        chassisMotionUpdated_ = true;
        joyScalarUpdated_     = true;
        lastDataTime_         = std::chrono::steady_clock::now();
    }

    void ScaledDeviceProvider::updateLeftPole(const RemoteOperate::Pole& poleInfo) {
        const double lr = (50.0 - static_cast<double>(poleInfo.y)) / 50.0 * -1.0;
        const double fb = (static_cast<double>(poleInfo.x) - 50.0) / 50.0 * -1.0;

        const double lXThreshold = joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisLeftX];
        const double lYThreshold = joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisLeftY];
        const double lXDeadZone  = joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisLeftX];
        const double lYDeadZone  = joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisLeftY];

        if (!rightComboLongPressed_) {
            if (std::abs(lr) < lYThreshold) {
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisRot] = 0.0;
            } else {
                joyAxesState_[scaled_device_provider_detail::kJoyStateChassisRot] =
                    amplifyAndClamp(applyDeadZone(mapJoyAxis(lr, lYThreshold), lYDeadZone), chassisVelAmpl_[2], chassisVelUb_[2]);
            }

            joyAxesState_[scaled_device_provider_detail::kJoyStateLegUd] =
                (std::fabs(fb) >= lXThreshold) ? applyDeadZone(mapJoyAxis(fb, lXThreshold), lXDeadZone) : 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateWaistLrRot]  = 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateWaistFbTran] = 0.0;
        } else {
            joyAxesState_[scaled_device_provider_detail::kJoyStateWaistLrRot] =
                (std::fabs(lr) >= lYThreshold) ? applyDeadZone(mapJoyAxis(lr, lYThreshold), lYDeadZone) : 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateWaistFbTran] =
                (std::fabs(fb) >= lXThreshold) ? applyDeadZone(mapJoyAxis(fb, lXThreshold), lXDeadZone) : 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateLegUd] = 0.0;
        }

        if (leftComboLongPressed_ || rightComboLongPressed_) {
            joyAxesState_[scaled_device_provider_detail::kJoyStateChassisFb]  = 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateChassisLr]  = 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateChassisRot] = 0.0;
        }

        chassisMotionUpdated_ = true;
        joyScalarUpdated_     = true;
        lastDataTime_         = std::chrono::steady_clock::now();
    }

    bool ScaledDeviceProvider::initialize(const YAML::Node& configNode, const std::string& frameId, const std::string& sourceId,
                                          std::string* error) {
        frameId_  = frameId;
        sourceId_ = sourceId;

        if (configNode["bus_name"]) {
            busName_ = configNode["bus_name"].as<std::string>();
        }
        if (configNode["device_name"]) {
            busName_ = configNode["device_name"].as<std::string>();
        }
        if (configNode["loop_hz"]) {
            loopHz_ = configNode["loop_hz"].as<int>();
        }
        if (configNode["trigger_threshold"]) {
            triggerThreshold_ = configNode["trigger_threshold"].as<double>();
        }
        if (configNode["offline_timeout_sec"]) {
            offlineTimeoutSec_ = configNode["offline_timeout_sec"].as<double>();
        }
        if (configNode["reconnect_interval_sec"]) {
            reconnectIntervalSec_ = configNode["reconnect_interval_sec"].as<double>();
        }
        if (configNode["semantic_context"]) {
            semantic_ = configNode["semantic_context"].as<std::string>();
        }
        if (configNode["mode"]) {
            mode_ = configNode["mode"].as<std::string>();
        }
        if (configNode["pipeline_id"]) {
            pipelineId_ = configNode["pipeline_id"].as<std::string>();
        }

        if (configNode["threshold"]) {
            const YAML::Node thresholdNode = configNode["threshold"];
            if (thresholdNode["joy_left_x_axis_threshold"])
                joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisLeftX] = thresholdNode["joy_left_x_axis_threshold"].as<double>();
            if (thresholdNode["joy_left_y_axis_threshold"])
                joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisLeftY] = thresholdNode["joy_left_y_axis_threshold"].as<double>();
            if (thresholdNode["joy_right_x_axis_threshold"])
                joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisRightX] = thresholdNode["joy_right_x_axis_threshold"].as<double>();
            if (thresholdNode["joy_right_y_axis_threshold"])
                joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisRightY] = thresholdNode["joy_right_y_axis_threshold"].as<double>();
            if (thresholdNode["joy_left_trigger_axis_threshold"])
                joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisLeftTrigger] =
                    thresholdNode["joy_left_trigger_axis_threshold"].as<double>();
            if (thresholdNode["joy_right_trigger_axis_threshold"])
                joyAxisThreshold_[scaled_device_provider_detail::kJoyAxisRightTrigger] =
                    thresholdNode["joy_right_trigger_axis_threshold"].as<double>();
        }

        if (configNode["dead_zone"]) {
            const YAML::Node deadZoneNode = configNode["dead_zone"];
            if (deadZoneNode["joy_left_x_axis_dead_zone"])
                joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisLeftX] = deadZoneNode["joy_left_x_axis_dead_zone"].as<double>();
            if (deadZoneNode["joy_left_y_axis_dead_zone"])
                joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisLeftY] = deadZoneNode["joy_left_y_axis_dead_zone"].as<double>();
            if (deadZoneNode["joy_right_x_axis_dead_zone"])
                joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisRightX] = deadZoneNode["joy_right_x_axis_dead_zone"].as<double>();
            if (deadZoneNode["joy_right_y_axis_dead_zone"])
                joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisRightY] = deadZoneNode["joy_right_y_axis_dead_zone"].as<double>();
            if (deadZoneNode["joy_left_trigger_axis_dead_zone"])
                joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisLeftTrigger] =
                    deadZoneNode["joy_left_trigger_axis_dead_zone"].as<double>();
            if (deadZoneNode["joy_right_trigger_axis_dead_zone"])
                joyAxisDeadZone_[scaled_device_provider_detail::kJoyAxisRightTrigger] =
                    deadZoneNode["joy_right_trigger_axis_dead_zone"].as<double>();
        }

        if (configNode["command_params"] && configNode["command_params"]["chassis_vel"]) {
            const YAML::Node chassisNode = configNode["command_params"]["chassis_vel"];
            if (chassisNode["apml"] && chassisNode["apml"].IsSequence()) {
                chassisVelAmpl_.clear();
                for (const auto& value : chassisNode["apml"])
                    chassisVelAmpl_.push_back(value.as<double>());
            } else if (chassisNode["ampl"] && chassisNode["ampl"].IsSequence()) {
                chassisVelAmpl_.clear();
                for (const auto& value : chassisNode["ampl"])
                    chassisVelAmpl_.push_back(value.as<double>());
            }
            if (chassisNode["ub"] && chassisNode["ub"].IsSequence()) {
                chassisVelUb_.clear();
                for (const auto& value : chassisNode["ub"])
                    chassisVelUb_.push_back(value.as<double>());
            }
        }
        if (chassisVelAmpl_.size() < 3)
            chassisVelAmpl_ = {0.5, 0.5, 0.5};
        if (chassisVelUb_.size() < 3)
            chassisVelUb_ = {0.6, 0.6, 0.6};

        if (configNode["buttons"] && configNode["buttons"].IsMap()) {
            for (const auto& item : configNode["buttons"]) {
                const std::string rawKey = item.first.as<std::string>();
                if (item.second["alias"]) {
                    buttonAliasByRawKey_[rawKey] = item.second["alias"].as<std::string>();
                }
            }
        }

        const YAML::Node leftArmNode  = configNode["left_arm"] ? configNode["left_arm"] : YAML::Node(YAML::NodeType::Map);
        const YAML::Node rightArmNode = configNode["right_arm"] ? configNode["right_arm"] : YAML::Node(YAML::NodeType::Map);
        if (!parseArmConfig(leftArmNode, "left_arm", &leftArm_, error)) {
            return false;
        }
        if (!parseArmConfig(rightArmNode, "right_arm", &rightArm_, error)) {
            return false;
        }

        if (configNode["left_gripper"]) {
            const YAML::Node node = configNode["left_gripper"];
            if (node["enabled"]) {
                leftGripperEnabled_ = node["enabled"].as<bool>();
            }
            if (node["joint_name"]) {
                leftGripperJointName_ = node["joint_name"].as<std::string>();
            }
        }
        if (configNode["right_gripper"]) {
            const YAML::Node node = configNode["right_gripper"];
            if (node["enabled"]) {
                rightGripperEnabled_ = node["enabled"].as<bool>();
            }
            if (node["joint_name"]) {
                rightGripperJointName_ = node["joint_name"].as<std::string>();
            }
        }

        updateArmSyncMode(true);

        system_ = std::make_unique<System>(busName_);
        remote_ = std::make_unique<RemoteOperate>(busName_);
        if (system_ == nullptr || remote_ == nullptr) {
            *error = "create scaled device sdk objects failed";
            return false;
        }

        system_->setRevCallBack(*this, &ScaledDeviceProvider::systemCallback);
        remote_->setRevCallBack(*this, &ScaledDeviceProvider::remoteCallback);

        const auto now        = std::chrono::steady_clock::now();
        lastDataTime_         = now;
        lastReconnectTryTime_ = now;
        return true;
    }

    void ScaledDeviceProvider::systemCallback(uint8_t cmdValue, uint8_t functionValue, std::vector<uint8_t> payload) {
        (void)cmdValue;
        (void)functionValue;
        (void)payload;
    }

    void ScaledDeviceProvider::updateArmState(const RemoteOperate::armInfo& armInfo, ArmRuntimeState* state) {
        const size_t jointCount = std::min(state->jointNames.size(), kArmMotorCount);
        for (size_t index = 0; index < jointCount; ++index) {
            const double mappedPos   = state->jointScale[index] * static_cast<double>(armInfo.m[index].curPos) - state->jointInit[index];
            state->positions[index]  = wrapJointAngle(mappedPos);
            state->velocities[index] = static_cast<double>(armInfo.m[index].curSpeed);
            state->efforts[index]    = static_cast<double>(armInfo.m[index].curTorque);
            state->errors[index]     = armInfo.m[index].error;
        }
        state->updated = true;
        lastDataTime_  = std::chrono::steady_clock::now();
    }

    void ScaledDeviceProvider::updateGripperFromTrigger(const RemoteOperate::Pole& triggerInfo, bool isLeft) {
        const double raw = static_cast<double>(triggerInfo.x);
        double position  = 0.0;
        if (raw > triggerThreshold_) {
            const double total = 100.0 - triggerThreshold_;
            position           = std::clamp((total - (raw - triggerThreshold_)) / total, 0.0, 1.0);
        }
        if (isLeft) {
            leftGripperPos_     = position;
            leftGripperUpdated_ = true;
        } else {
            rightGripperPos_     = position;
            rightGripperUpdated_ = true;
        }
        lastDataTime_ = std::chrono::steady_clock::now();
    }

    void ScaledDeviceProvider::updateButtonAlias(const std::string& aliasName, uint8_t statusValue) {
        const bool isDown = (statusValue == KEY_DOWN);
        const bool isUp   = (statusValue == KEY_UP);
        const bool isLong = (statusValue == KEY_LONG);

        if (aliasName == "left_combo") {
            if (isLong)
                leftComboLongPressed_ = true;
            if (isUp)
                leftComboLongPressed_ = false;
        } else if (aliasName == "right_combo") {
            if (isLong)
                rightComboLongPressed_ = true;
            if (isUp)
                rightComboLongPressed_ = false;
        } else if (aliasName == "left_arm_sync" && isDown) {
            leftArmControl_.directSync = !leftArmControl_.directSync;
            leftArmControl_.inTakeover = false;
        } else if (aliasName == "right_arm_sync" && isDown) {
            rightArmControl_.directSync = !rightArmControl_.directSync;
            rightArmControl_.inTakeover = false;
        } else if (aliasName == "left_takeover") {
            leftArmControl_.inTakeover = !isUp;
            if (!leftArmControl_.inTakeover) {
                leftArmControl_.directSync = false;
            }
        } else if (aliasName == "right_takeover") {
            rightArmControl_.inTakeover = !isUp;
            if (!rightArmControl_.inTakeover) {
                rightArmControl_.directSync = false;
            }
        } else if (aliasName == "left_takeover_switch" && isUp) {
            leftArmControl_.useCartesianTakeover = !leftArmControl_.useCartesianTakeover;
        } else if (aliasName == "right_takeover_switch" && isUp) {
            rightArmControl_.useCartesianTakeover = !rightArmControl_.useCartesianTakeover;
        } else if (aliasName == "left_stop") {
            leftSoftEmergencyStop_    = !isUp;
            softEmergencyStopUpdated_ = true;
        } else if (aliasName == "right_stop") {
            rightSoftEmergencyStop_   = !isUp;
            softEmergencyStopUpdated_ = true;
        }

        const bool hasSoftStop = leftSoftEmergencyStop_ || rightSoftEmergencyStop_;
        if (hasSoftStop) {
            leftArmControl_.directSync  = false;
            leftArmControl_.inTakeover  = false;
            rightArmControl_.directSync = false;
            rightArmControl_.inTakeover = false;
        }

        updateArmSyncMode();

        if (leftComboLongPressed_ || rightComboLongPressed_) {
            joyAxesState_[scaled_device_provider_detail::kJoyStateChassisFb]  = 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateChassisLr]  = 0.0;
            joyAxesState_[scaled_device_provider_detail::kJoyStateChassisRot] = 0.0;
            chassisMotionUpdated_                                             = true;
        }
    }

    void ScaledDeviceProvider::remoteCallback(uint8_t cmdValue, uint8_t functionValue, std::vector<uint8_t> payload) {
        std::lock_guard<std::mutex> lock(dataMutex_);
        if (cmdValue != cmd.at("autoReport")) {
            return;
        }

        if (functionValue == remote_->getFun("armL") && payload.size() >= sizeof(RemoteOperate::armInfo)) {
            RemoteOperate::armInfo info;
            std::memcpy(static_cast<void*>(&info), payload.data(), sizeof(RemoteOperate::armInfo));
            updateArmState(info, &leftArm_);
            return;
        }
        if (functionValue == remote_->getFun("armR") && payload.size() >= sizeof(RemoteOperate::armInfo)) {
            RemoteOperate::armInfo info;
            std::memcpy(static_cast<void*>(&info), payload.data(), sizeof(RemoteOperate::armInfo));
            updateArmState(info, &rightArm_);
            return;
        }

        if (functionValue == remote_->getFun("triggerL") && payload.size() >= sizeof(RemoteOperate::Pole)) {
            RemoteOperate::Pole info;
            std::memcpy(static_cast<void*>(&info), payload.data(), sizeof(RemoteOperate::Pole));
            updateGripperFromTrigger(info, true);
            return;
        }
        if (functionValue == remote_->getFun("triggerR") && payload.size() >= sizeof(RemoteOperate::Pole)) {
            RemoteOperate::Pole info;
            std::memcpy(static_cast<void*>(&info), payload.data(), sizeof(RemoteOperate::Pole));
            updateGripperFromTrigger(info, false);
            return;
        }

        if (functionValue == remote_->getFun("poleL") && payload.size() >= sizeof(RemoteOperate::Pole)) {
            RemoteOperate::Pole info;
            std::memcpy(static_cast<void*>(&info), payload.data(), sizeof(RemoteOperate::Pole));
            updateLeftPole(info);
            return;
        }
        if (functionValue == remote_->getFun("poleR") && payload.size() >= sizeof(RemoteOperate::Pole)) {
            RemoteOperate::Pole info;
            std::memcpy(static_cast<void*>(&info), payload.data(), sizeof(RemoteOperate::Pole));
            updateRightPole(info);
            return;
        }

        for (const char* rawKey : scaled_device_provider_detail::kRawButtonKeys) {
            if (functionValue == remote_->getFun(rawKey) && payload.size() >= 2) {
                const auto aliasIt = buttonAliasByRawKey_.find(rawKey);
                if (aliasIt != buttonAliasByRawKey_.end()) {
                    updateButtonAlias(aliasIt->second, payload[1]);
                }
                lastDataTime_ = std::chrono::steady_clock::now();
                return;
            }
        }
    }

    void ScaledDeviceProvider::tryReconnect() {
        const auto now       = std::chrono::steady_clock::now();
        const double idleSec = std::chrono::duration<double>(now - lastDataTime_).count();
        if (idleSec < offlineTimeoutSec_) {
            return;
        }

        const double reconnectElapsed = std::chrono::duration<double>(now - lastReconnectTryTime_).count();
        if (reconnectElapsed < reconnectIntervalSec_) {
            return;
        }

        lastReconnectTryTime_ = now;
        if (system_ != nullptr) {
            system_->reOpen();
        }
        if (remote_ != nullptr) {
            remote_->reOpen();
        }
    }

    bool ScaledDeviceProvider::nextFrame(uint64_t sequenceId, model::PrimitiveFrame* frame, std::string* error) {
        std::lock_guard<std::mutex> lock(dataMutex_);
        tryReconnect();

        const bool hasArmUpdate     = (leftArm_.enabled && leftArm_.updated) || (rightArm_.enabled && rightArm_.updated);
        const bool hasGripperUpdate = (leftGripperEnabled_ && leftGripperUpdated_) || (rightGripperEnabled_ && rightGripperUpdated_);
        const bool hasModeUpdate    = leftArmControl_.modeUpdated || rightArmControl_.modeUpdated || softEmergencyStopUpdated_;
        if (!hasArmUpdate && !hasGripperUpdate && !chassisMotionUpdated_ && !joyScalarUpdated_ && !hasModeUpdate) {
            error->clear();
            return false;
        }
        const auto now  = std::chrono::system_clock::now().time_since_epoch();
        const auto sec  = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - sec * 1000000000LL;

        frame->header.timestamp.sec     = sec;
        frame->header.timestamp.nanosec = static_cast<uint32_t>(nsec);
        frame->header.frameId           = frameId_;
        frame->sequenceId               = sequenceId;
        frame->context.sourceId         = sourceId_;
        frame->context.sourceType       = model::SourceType::kExternal;
        frame->context.semanticContext  = semantic_;
        frame->context.mode             = mode_;
        frame->context.pipelineId       = pipelineId_;
        frame->poses.clear();
        frame->twists.clear();
        frame->jointStates.clear();
        frame->jointCommands.clear();
        frame->scalars.clear();
        frame->booleans.clear();
        frame->modes.clear();
        frame->planarMotions.clear();
        frame->skeletons.clear();
        frame->landmarkSets.clear();
        frame->tags.clear();

        auto appendArmJointState = [&](const ArmRuntimeState& armState) {
            if (!armState.enabled || !armState.updated) {
                return;
            }
            model::JointStatePrimitive jointState;
            jointState.meta.name             = armState.stateName;
            jointState.meta.entity           = armState.armName;
            jointState.meta.bodyGroup        = armState.bodyGroup;
            jointState.meta.frameId          = frameId_;
            jointState.meta.referenceFrameId = frameId_;
            jointState.meta.confidence       = 1.0F;
            jointState.meta.valid            = true;
            jointState.jointNames.reserve(armState.jointNames.size());
            jointState.position.reserve(armState.jointNames.size());
            jointState.velocity.reserve(armState.jointNames.size());
            jointState.effort.reserve(armState.jointNames.size());

            for (size_t index = 0; index < armState.jointNames.size(); ++index) {
                jointState.jointNames.push_back(armState.jointNames[index]);
                jointState.position.push_back(armState.positions[index]);
                jointState.velocity.push_back(armState.velocities[index]);
                jointState.effort.push_back(armState.efforts[index]);
            }
            frame->jointStates.push_back(std::move(jointState));
        };
        appendArmJointState(leftArm_);
        appendArmJointState(rightArm_);

        if (leftGripperEnabled_ && leftGripperUpdated_) {
            model::JointStatePrimitive jointState;
            jointState.meta.name             = "left_gripper_state";
            jointState.meta.entity           = "left_gripper";
            jointState.meta.bodyGroup        = model::BodyGroup::kLeftGripper;
            jointState.meta.frameId          = frameId_;
            jointState.meta.referenceFrameId = frameId_;
            jointState.meta.confidence       = 1.0F;
            jointState.meta.valid            = true;
            jointState.jointNames.push_back(leftGripperJointName_);
            jointState.position.push_back(leftGripperPos_);
            frame->jointStates.push_back(std::move(jointState));
        }

        if (rightGripperEnabled_ && rightGripperUpdated_) {
            model::JointStatePrimitive jointState;
            jointState.meta.name             = "right_gripper_state";
            jointState.meta.entity           = "right_gripper";
            jointState.meta.bodyGroup        = model::BodyGroup::kRightGripper;
            jointState.meta.frameId          = frameId_;
            jointState.meta.referenceFrameId = frameId_;
            jointState.meta.confidence       = 1.0F;
            jointState.meta.valid            = true;
            jointState.jointNames.push_back(rightGripperJointName_);
            jointState.position.push_back(rightGripperPos_);
            frame->jointStates.push_back(std::move(jointState));
        }

        if (chassisMotionUpdated_) {
            model::PlanarMotionPrimitive planar;
            planar.meta.name             = "scaled_device_chassis_motion";
            planar.meta.entity           = "chassis";
            planar.meta.bodyGroup        = model::BodyGroup::kBase;
            planar.meta.frameId          = frameId_;
            planar.meta.referenceFrameId = frameId_;
            planar.meta.confidence       = 1.0F;
            planar.meta.valid            = true;
            planar.vx                    = joyAxesState_[scaled_device_provider_detail::kJoyStateChassisFb];
            planar.vy                    = joyAxesState_[scaled_device_provider_detail::kJoyStateChassisLr];
            planar.wz                    = joyAxesState_[scaled_device_provider_detail::kJoyStateChassisRot];
            planar.referenceFrameId      = frameId_;
            frame->planarMotions.push_back(std::move(planar));
        }

        if (joyScalarUpdated_) {
            auto addScalar = [&](const std::string& name, model::BodyGroup group, double value) {
                model::ScalarPrimitive scalar;
                scalar.meta.name             = name;
                scalar.meta.entity           = name;
                scalar.meta.bodyGroup        = group;
                scalar.meta.frameId          = frameId_;
                scalar.meta.referenceFrameId = frameId_;
                scalar.meta.confidence       = 1.0F;
                scalar.meta.valid            = true;
                scalar.value                 = value;
                scalar.minValue              = -1.0;
                scalar.maxValue              = 1.0;
                frame->scalars.push_back(std::move(scalar));
            };

            addScalar("leg_ud", model::BodyGroup::kLowerBody, joyAxesState_[scaled_device_provider_detail::kJoyStateLegUd]);
            addScalar("waist_lr_rot", model::BodyGroup::kTorso, joyAxesState_[scaled_device_provider_detail::kJoyStateWaistLrRot]);
            addScalar("waist_fb_tran", model::BodyGroup::kTorso, joyAxesState_[scaled_device_provider_detail::kJoyStateWaistFbTran]);
            addScalar("head_yaw", model::BodyGroup::kHead, joyAxesState_[scaled_device_provider_detail::kJoyStateHeadYaw]);
            addScalar("head_pitch", model::BodyGroup::kHead, joyAxesState_[scaled_device_provider_detail::kJoyStateHeadPitch]);
        }

        auto appendMode = [&](const std::string& name, const std::string& entity, ArmSyncMode modeValue, bool modeUpdated) {
            if (!modeUpdated) {
                return;
            }
            model::ModePrimitive mode;
            mode.meta.name             = name;
            mode.meta.entity           = entity;
            mode.meta.bodyGroup        = entity == "left_arm" ? model::BodyGroup::kLeftArm : model::BodyGroup::kRightArm;
            mode.meta.frameId          = frameId_;
            mode.meta.referenceFrameId = frameId_;
            mode.meta.confidence       = 1.0F;
            mode.meta.valid            = true;
            mode.modeId                = static_cast<int32_t>(modeValue);
            switch (modeValue) {
                case ArmSyncMode::DirectSync:
                    mode.modeName = "direct_sync";
                    break;
                case ArmSyncMode::JointSpaceIncremental:
                    mode.modeName = "joint_space_incremental_sync";
                    break;
                case ArmSyncMode::CartesianSpaceIncremental:
                    mode.modeName = "cartesian_space_incremental_sync";
                    break;
                default:
                    mode.modeName = "none";
                    break;
            }
            frame->modes.push_back(std::move(mode));
        };

        appendMode("left_arm_sync_mode", "left_arm", leftArmControl_.effectiveMode, leftArmControl_.modeUpdated);
        appendMode("right_arm_sync_mode", "right_arm", rightArmControl_.effectiveMode, rightArmControl_.modeUpdated);

        if (softEmergencyStopUpdated_) {
            model::BooleanPrimitive stopFlag;
            stopFlag.meta.name             = "soft_emergency_stop";
            stopFlag.meta.entity           = "scaled_device";
            stopFlag.meta.bodyGroup        = model::BodyGroup::kWholeBody;
            stopFlag.meta.frameId          = frameId_;
            stopFlag.meta.referenceFrameId = frameId_;
            stopFlag.meta.confidence       = 1.0F;
            stopFlag.meta.valid            = true;
            stopFlag.value                 = leftSoftEmergencyStop_ || rightSoftEmergencyStop_;
            frame->booleans.push_back(std::move(stopFlag));
        }

        leftArm_.updated             = false;
        rightArm_.updated            = false;
        leftGripperUpdated_          = false;
        rightGripperUpdated_         = false;
        chassisMotionUpdated_        = false;
        joyScalarUpdated_            = false;
        leftArmControl_.modeUpdated  = false;
        rightArmControl_.modeUpdated = false;
        softEmergencyStopUpdated_    = false;
        return true;
    }

    int ScaledDeviceProvider::suggestedLoopHz() const {
        return loopHz_;
    }

}  // namespace puppet::device
