#include "puppet/retargeting/native/gmr_retargeting_plugin.hpp"

#include <gmr/retarget/ik_config.h>
#include <gmr/retarget/retargeter.h>
#include <Eigen/Dense>

namespace puppet::retargeting {
    namespace {
    }  // namespace

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
        if (config.gmr.robotModelPath.empty() || config.gmr.ikConfigPath.empty()) {
            if (error != nullptr) {
                *error = "gmr enabled but robot_model_path or ik_config_path is empty";
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
        holder->retargeter = gmr::createRetargeter(gmr::parseRetargetBackend(config.gmr.backendName), config.gmr.robotModelPath,
                                                   std::move(ikConfig), options);
        holder_            = std::move(holder);

        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    bool GmrRetargetingPlugin::process(const model::PrimitiveFrame& input, const std::string& bodyGroup, model::GroupControlIntent* output,
                                       std::string* error) {
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
            if (coord.qIndex < 0) {
                continue;
            }
            if (coord.jointName.empty()) {
                continue;
            }
            if (coord.qIndex >= qpos.size()) {
                continue;
            }
            jointIntent.jointNames.push_back(coord.jointName);
            jointIntent.position.push_back(qpos[coord.qIndex]);
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
