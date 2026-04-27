#include "puppet/device/device_service_config.hpp"

#include <exception>

namespace puppet::device {

    namespace device_config_detail {

        int readInt(const YAML::Node& node, const char* key, int fallback) {
            if (node[key]) {
                return node[key].as<int>();
            }
            return fallback;
        }

        std::string readString(const YAML::Node& node, const char* key, const std::string& fallback) {
            if (node[key]) {
                return node[key].as<std::string>();
            }
            return fallback;
        }

    }  // namespace device_config_detail

    bool loadDeviceServiceConfig(const std::string& path, DeviceServiceConfig* config, std::string* error) {
        try {
            const YAML::Node root = YAML::LoadFile(path);

            config->nodeName       = device_config_detail::readString(root, "node_name", config->nodeName);
            config->topicName      = device_config_detail::readString(root, "topic_name", config->topicName);
            config->sourceId       = device_config_detail::readString(root, "source_id", config->sourceId);
            config->frameId        = device_config_detail::readString(root, "frame_id", config->frameId);
            config->loopHz         = device_config_detail::readInt(root, "loop_hz", config->loopHz);
            config->outputEndpoint = device_config_detail::readString(root, "output_endpoint", config->outputEndpoint);

            if (root["channel"] && root["channel"]["type"]) {
                config->channelType = root["channel"]["type"].as<std::string>();
            }
            if (root["device"] && root["device"]["type"]) {
                config->deviceType = root["device"]["type"].as<std::string>();
            }

            config->channelNode = root["channel"] ? root["channel"] : YAML::Node(YAML::NodeType::Map);
            config->deviceNode  = root["device"] ? root["device"] : YAML::Node(YAML::NodeType::Map);

            if (config->deviceType.empty()) {
                *error = "device.type is empty";
                return false;
            }
            if (config->channelType.empty()) {
                *error = "channel.type is empty";
                return false;
            }
            return true;
        } catch (const std::exception& ex) {
            *error = ex.what();
            return false;
        }
    }

}  // namespace puppet::device
