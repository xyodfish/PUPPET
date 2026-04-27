#include "puppet/device/device_provider.hpp"

#include "puppet/device/scaled/scaled_device_provider.hpp"
#include "puppet/device/single_chain_ik/single_chain_ik_sender_device_provider.hpp"
#include "puppet/device/static_file_replay/static_file_replay_device_provider.hpp"

namespace puppet::device {

    std::unique_ptr<IDeviceProvider> createDeviceProvider(const std::string& deviceType, std::string* error) {
        if (deviceType == "static_file_replay") {
            return std::make_unique<StaticFileReplayDeviceProvider>();
        }
        if (deviceType == "single_chain_ik_sender") {
            return std::make_unique<SingleChainIkSenderDeviceProvider>();
        }
        if (deviceType == "scaled_device") {
            return std::make_unique<ScaledDeviceProvider>();
        }

        *error = "unsupported device.type: " + deviceType;
        return nullptr;
    }

}  // namespace puppet::device
