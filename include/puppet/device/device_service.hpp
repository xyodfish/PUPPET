#pragma once

#include <memory>
#include <string>

#include "puppet/device/device_provider.hpp"
#include "puppet/device/device_service_config.hpp"
#include "puppet/transport/device_output_channel.hpp"

namespace puppet::device {

    class DeviceService {
       public:
        bool initialize(const DeviceServiceConfig& config, std::string* error);
        int run();

       private:
        DeviceServiceConfig config_;
        std::unique_ptr<IDeviceProvider> provider_;
        std::unique_ptr<transport::IDeviceOutputChannel> channel_;
    };

}  // namespace puppet::device
