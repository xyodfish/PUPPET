#include "gmr/retarget/human_frame_io.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace gmr {
namespace {

Eigen::Vector3d parseVec3(const nlohmann::json& json) {
  if (!json.is_array() || json.size() != 3) {
    throw std::runtime_error("Expected position vec3 [x,y,z].");
  }
  return Eigen::Vector3d(json[0].get<double>(), json[1].get<double>(), json[2].get<double>());
}

Eigen::Quaterniond parseQuatWxyz(const nlohmann::json& json) {
  if (!json.is_array() || json.size() != 4) {
    throw std::runtime_error("Expected orientation quaternion [w,x,y,z].");
  }
  return Eigen::Quaterniond(json[0].get<double>(), json[1].get<double>(), json[2].get<double>(), json[3].get<double>());
}

bool isBodyState(const nlohmann::json& json) {
  if (json.is_object()) {
    return json.contains("position") && json.contains("orientation");
  }
  return json.is_array() && json.size() == 2;
}

bool isFrameDict(const nlohmann::json& json) {
  if (!json.is_object() || json.empty()) {
    return false;
  }
  for (auto it = json.begin(); it != json.end(); ++it) {
    if (!isBodyState(it.value())) {
      return false;
    }
  }
  return true;
}

HumanFrame parseFrame(const nlohmann::json& frameJson) {
  if (!frameJson.is_object()) {
    throw std::runtime_error("Each frame must be an object mapping body_name -> pose.");
  }

  HumanFrame frame;
  for (auto it = frameJson.begin(); it != frameJson.end(); ++it) {
    HumanBodyState state;
    if (it.value().is_object()) {
      state.position = parseVec3(it.value().at("position"));
      state.orientation = parseQuatWxyz(it.value().at("orientation"));
    } else if (it.value().is_array() && it.value().size() == 2) {
      state.position = parseVec3(it.value()[0]);
      state.orientation = parseQuatWxyz(it.value()[1]);
    } else {
      throw std::runtime_error("Invalid body pose format for body: " + it.key());
    }
    frame[it.key()] = state;
  }
  return frame;
}

}  // namespace

HumanFrameSequence loadHumanFrameSequence(const std::filesystem::path& filePath) {
  std::ifstream ifs(filePath);
  if (!ifs.is_open()) {
    throw std::runtime_error("Failed to open human frame json: " + filePath.string());
  }

  nlohmann::json root;
  ifs >> root;

  HumanFrameSequence sequence;
  if (root.is_object() && root.contains("fps")) {
    sequence.fps = root.at("fps").get<int>();
  }

  nlohmann::json framesNode = root;
  if (root.is_object() && root.contains("frames")) {
    framesNode = root.at("frames");
  }

  if (framesNode.is_array()) {
    sequence.frames.reserve(framesNode.size());
    for (const auto& frameJson : framesNode) {
      sequence.frames.push_back(parseFrame(frameJson));
    }
  } else if (isFrameDict(framesNode)) {
    sequence.frames.push_back(parseFrame(framesNode));
  } else if (framesNode.is_object()) {
    std::vector<std::string> keys;
    keys.reserve(framesNode.size());
    for (auto it = framesNode.begin(); it != framesNode.end(); ++it) {
      keys.push_back(it.key());
    }

    std::sort(keys.begin(), keys.end(), [](const std::string& a, const std::string& b) {
      try {
        return std::stoi(a) < std::stoi(b);
      } catch (...) {
        return a < b;
      }
    });

    sequence.frames.reserve(keys.size());
    for (const auto& key : keys) {
      sequence.frames.push_back(parseFrame(framesNode.at(key)));
    }
  } else {
    throw std::runtime_error("Unsupported JSON structure for human frames.");
  }

  if (sequence.frames.empty()) {
    throw std::runtime_error("No frames parsed from JSON.");
  }
  if (sequence.fps <= 0) {
    sequence.fps = 30;
  }

  return sequence;
}

}  // namespace gmr
