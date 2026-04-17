#include "puppet/retargeting/native/gmr_retargeting_plugin.hpp"

#include <gmr/retarget/ik_config.h>
#include <gmr/retarget/retargeter.h>
#include <Eigen/Dense>

#include <iostream>

namespace puppet::retargeting {
    namespace gmr_retargeting_detail {
        constexpr const char* kRootPosXKey  = "__root_pos_x";
        constexpr const char* kRootPosYKey  = "__root_pos_y";
        constexpr const char* kRootPosZKey  = "__root_pos_z";
        constexpr const char* kRootQuatWKey = "__root_quat_w";
        constexpr const char* kRootQuatXKey = "__root_quat_x";
        constexpr const char* kRootQuatYKey = "__root_quat_y";
        constexpr const char* kRootQuatZKey = "__root_quat_z";
    }  // namespace gmr_retargeting_detail

    namespace gmr_wrap {

        class RetargeterHolder {
           public:
            std::unique_ptr<gmr::Retargeter> retargeter;
        };

    }  // namespace gmr_wrap

    bool GmrRetargetingPlugin::configure(const runtime::RuntimeConfig& config, std::string* error) {
        enabled_ = config.gmr.enabled;
        if (!enabled_) {
            if (error != nullptr) {
                error->clear();
            }
            return true;
        }
        const gmr::RetargetBackend backend = gmr::parseRetargetBackend(config.gmr.backendName);
        const bool isMujocoBackend         = backend == gmr::RetargetBackend::kMujoco || backend == gmr::RetargetBackend::kMujocoLegacy;
        const std::string& resolvedRobotModelPath =
            isMujocoBackend && !config.gmr.robotModelXmlPath.empty() ? config.gmr.robotModelXmlPath : config.gmr.robotModelPath;

        if (resolvedRobotModelPath.empty() || config.gmr.ikConfigPath.empty()) {
            if (error != nullptr) {
                *error = "gmr enabled but resolved robot model path or ik_config_path is empty";
            }
            return false;
        }

        gmr::RetargetOptions options;
        options.damping             = config.gmr.damping;
        options.maxIterations       = config.gmr.maxIterations;
        options.useVelocityLimit    = config.gmr.useVelocityLimit;
        options.integrationTimestep = config.gmr.integrationTimestep;
        options.progressThreshold   = config.gmr.progressThreshold;

        gmr::IkConfig ikConfig = gmr::loadIkConfig(config.gmr.ikConfigPath, config.gmr.actualHumanHeight);

        auto holder        = std::make_shared<gmr_wrap::RetargeterHolder>();
        holder->retargeter = gmr::createRetargeter(backend, resolvedRobotModelPath, std::move(ikConfig), options);
        holder_            = std::move(holder);

        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    bool GmrRetargetingPlugin::process(const model::PrimitiveFrame& input, const std::string& bodyGroup, model::GroupControlIntent* output,
                                       std::string* error) {
        static bool firstRun = true;
        if (!enabled_) {
            if (error != nullptr) {
                *error = "GMR plugin is not enabled in runtime config";
            }
            return false;
        }

        if (output == nullptr) {
            if (error != nullptr) {
                *error = "output is null";
            }
            return false;
        }

        if (holder_ == nullptr || holder_->retargeter == nullptr) {
            if (error != nullptr) {
                *error = "GMR retargeter is not initialized";
            }
            return false;
        }

        gmr::HumanFrame humanFrame;
        for (const auto& posePrimitive : input.poses) {
            if (posePrimitive.meta.entity.empty()) {
                continue;
            }
            gmr::HumanBodyState state;
            state.position = Eigen::Vector3d(posePrimitive.pose.position.x, posePrimitive.pose.position.y, posePrimitive.pose.position.z);
            state.orientation                     = Eigen::Quaterniond(posePrimitive.pose.orientation.w, posePrimitive.pose.orientation.x,
                                                                       posePrimitive.pose.orientation.y, posePrimitive.pose.orientation.z);
            humanFrame[posePrimitive.meta.entity] = state;
        }

        if (humanFrame.empty()) {
            if (error != nullptr) {
                *error = "GMR input human frame is empty, requires pose primitives with entity";
            }
            return false;
        }

        const Eigen::VectorXd qpos = holder_->retargeter->retargetFrame(humanFrame, false);

        model::JointCommandIntent jointIntent;
        jointIntent.bodyGroup = model::BodyGroup::kCustom;
        jointIntent.weight    = 1.0;

        const auto& scalarCoords = holder_->retargeter->scalarJointCoordinates();
        jointIntent.jointNames.reserve(scalarCoords.size());
        jointIntent.position.reserve(scalarCoords.size());

        for (const auto& coord : scalarCoords) {
            if ((coord.qIndex < 0) || (coord.jointName.empty()) || (coord.qIndex >= qpos.size())) {
                continue;
            }

            jointIntent.jointNames.push_back(coord.jointName);
            jointIntent.position.push_back(qpos[coord.qIndex]);
        }

        if (holder_->retargeter->hasRootFreeFlyer() && qpos.size() >= 7) {
            jointIntent.jointNames.push_back(gmr_retargeting_detail::kRootPosXKey);
            jointIntent.position.push_back(qpos[0]);
            jointIntent.jointNames.push_back(gmr_retargeting_detail::kRootPosYKey);
            jointIntent.position.push_back(qpos[1]);
            jointIntent.jointNames.push_back(gmr_retargeting_detail::kRootPosZKey);
            jointIntent.position.push_back(qpos[2]);
            jointIntent.jointNames.push_back(gmr_retargeting_detail::kRootQuatWKey);
            jointIntent.position.push_back(qpos[3]);
            jointIntent.jointNames.push_back(gmr_retargeting_detail::kRootQuatXKey);
            jointIntent.position.push_back(qpos[4]);
            jointIntent.jointNames.push_back(gmr_retargeting_detail::kRootQuatYKey);
            jointIntent.position.push_back(qpos[5]);
            jointIntent.jointNames.push_back(gmr_retargeting_detail::kRootQuatZKey);
            jointIntent.position.push_back(qpos[6]);
        }

        output->ownerSourceId = input.context.sourceId;
        output->mode          = "gmr";
        output->enabled       = true;
        output->jointCommandIntents.push_back(std::move(jointIntent));

        if (error != nullptr) {
            error->clear();
        }
        (void)bodyGroup;
        return true;
    }

}  // namespace puppet::retargeting
