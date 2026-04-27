#include "puppet/transport/device_output_channel.hpp"

#include "puppet/transport/embosa/embosa_device_output_channel.hpp"
#include "puppet/transport/zmq/zmq_device_output_channel.hpp"

namespace puppet::transport {

    std::unique_ptr<IDeviceOutputChannel> createDeviceOutputChannel(const std::string& channelType, std::string* error) {
        if (channelType == "embosa") {
            return std::make_unique<EmbosaDeviceOutputChannel>();
        }
        if (channelType == "zmq") {
            return std::make_unique<ZmqDeviceOutputChannel>();
        }

        *error = "unsupported channel.type: " + channelType;
        return nullptr;
    }

}  // namespace puppet::transport
