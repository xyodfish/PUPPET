#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "puppet/primitive/primitive_types.hpp"

namespace puppet::device {

    class IDeviceProvider {
       public:
        virtual ~IDeviceProvider() = default;

        virtual bool initialize(const YAML::Node& configNode, const std::string& frameId, const std::string& sourceId,
                                std::string* error) = 0;

        virtual bool nextFrame(uint64_t sequenceId, model::PrimitiveFrame* frame, std::string* error) = 0;

        virtual int suggestedLoopHz() const { return 50; }
    };

    std::unique_ptr<IDeviceProvider> createDeviceProvider(const std::string& deviceType, std::string* error);

}  // namespace puppet::device
