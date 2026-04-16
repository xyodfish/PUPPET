#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

namespace gmr {

struct IkTaskEntry {
  std::string robotBodyName;
  std::string humanBodyName;
  double posWeight = 0.0;
  double rotWeight = 0.0;
  Eigen::Vector3d posOffset = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotOffset = Eigen::Quaterniond::Identity();
};

struct IkConfig {
  std::string robotRootName;
  std::string humanRootName;
  double groundHeight = 0.0;
  double humanHeightAssumption = 1.0;
  bool useTable1 = true;
  bool useTable2 = true;
  std::unordered_map<std::string, double> humanScaleTable;
  std::vector<IkTaskEntry> tasksTable1;
  std::vector<IkTaskEntry> tasksTable2;
};

IkConfig loadIkConfig(const std::filesystem::path& configPath, double actualHumanHeight);

}  // namespace gmr
