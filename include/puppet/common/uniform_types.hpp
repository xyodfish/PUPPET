#pragma once

#include <string>
#include <string_view>
#include <unordered_map>

#include "puppet/common/base_types.hpp"

namespace puppet::model {

    using TagMap = std::unordered_map<std::string, std::string>;

    enum class BodyGroup {
        kUnspecified  = 0,
        kHead         = 1,
        kLeftArm      = 2,
        kRightArm     = 3,
        kBiManual     = 4,
        kTorso        = 5,
        kBase         = 6,
        kLowerBody    = 7,
        kWholeBody    = 8,
        kLeftGripper  = 9,
        kRightGripper = 10,
        kCustom       = 100,
    };

    inline std::string bodyGroupToString(BodyGroup bodyGroup) {
        switch (bodyGroup) {
            case BodyGroup::kUnspecified:
                return "unspecified";
            case BodyGroup::kHead:
                return "head";
            case BodyGroup::kLeftArm:
                return "left_arm";
            case BodyGroup::kRightArm:
                return "right_arm";
            case BodyGroup::kBiManual:
                return "bi_manual";
            case BodyGroup::kTorso:
                return "torso";
            case BodyGroup::kBase:
                return "base";
            case BodyGroup::kLowerBody:
                return "lower_body";
            case BodyGroup::kWholeBody:
                return "whole_body";
            case BodyGroup::kLeftGripper:
                return "left_gripper";
            case BodyGroup::kRightGripper:
                return "right_gripper";
            case BodyGroup::kCustom:
                return "custom";
        }
        return "custom";
    }

    inline BodyGroup bodyGroupFromString(std::string_view bodyGroup) {
        if (bodyGroup.empty() || bodyGroup == "unspecified") {
            return BodyGroup::kUnspecified;
        }
        if (bodyGroup == "head") {
            return BodyGroup::kHead;
        }
        if (bodyGroup == "left_arm") {
            return BodyGroup::kLeftArm;
        }
        if (bodyGroup == "right_arm") {
            return BodyGroup::kRightArm;
        }
        if (bodyGroup == "bi_manual") {
            return BodyGroup::kBiManual;
        }
        if (bodyGroup == "torso") {
            return BodyGroup::kTorso;
        }
        if (bodyGroup == "base") {
            return BodyGroup::kBase;
        }
        if (bodyGroup == "lower_body") {
            return BodyGroup::kLowerBody;
        }
        if (bodyGroup == "whole_body") {
            return BodyGroup::kWholeBody;
        }
        if (bodyGroup == "left_gripper") {
            return BodyGroup::kLeftGripper;
        }
        if (bodyGroup == "right_gripper") {
            return BodyGroup::kRightGripper;
        }
        return BodyGroup::kCustom;
    }

    enum class SourceType {
        kUnspecified = 0,
        kVr          = 1,
        kMasterArm   = 2,
        kMocap       = 3,
        kVision      = 4,
        kJoystick    = 5,
        kExternal    = 100,
    };

    struct FrameContext {
        std::string sourceId;
        SourceType sourceType = SourceType::kUnspecified;
        std::string semanticContext;
        std::string mode;
        std::string robotId;
        std::string pluginId;
        std::unordered_map<std::string, std::string> groupPluginIds;  // group_name -> plugin_id
        TagMap tags;
    };

    struct PrimitiveMeta {
        std::string name;
        std::string entity;
        std::string bodyGroup;
        std::string frameId;
        std::string referenceFrameId;
        Timestamp timestamp;
        float confidence = 0.0F;
        bool valid       = false;
        TagMap tags;
    };

}  // namespace puppet::model
