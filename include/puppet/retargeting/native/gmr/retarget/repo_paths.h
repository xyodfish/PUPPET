#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace gmr {

    inline const std::unordered_map<std::string, std::string>& robotXmlMap() {
        static const std::unordered_map<std::string, std::string> kMap = {
            {"unitree_g1", "assets/unitree_g1/g1_mocap_29dof.xml"},
            {"unitree_g1_with_hands", "assets/unitree_g1/g1_mocap_29dof_with_hands.xml"},
            {"unitree_h1", "assets/unitree_h1/h1.xml"},
            {"unitree_h1_2", "assets/unitree_h1_2/h1_2_handless.xml"},
            {"booster_t1", "assets/booster_t1/T1_serial.xml"},
            {"booster_t1_29dof", "assets/booster_t1_29dof/t1_mocap.xml"},
            {"stanford_toddy", "assets/stanford_toddy/toddy_mocap.xml"},
            {"fourier_n1", "assets/fourier_n1/n1_mocap.xml"},
            {"engineai_pm01", "assets/engineai_pm01/pm_v2.xml"},
            {"kuavo_s45", "assets/kuavo_s45/biped_s45_collision.xml"},
            {"hightorque_hi", "assets/hightorque_hi/hi_25dof.xml"},
            {"galaxea_r1pro", "assets/galaxea_r1pro/r1_pro.xml"},
            {"berkeley_humanoid_lite", "assets/berkeley_humanoid_lite/bhl_scene.xml"},
            {"booster_k1", "assets/booster_k1/K1_serial.xml"},
            {"pnd_adam_lite", "assets/pnd_adam_lite/scene.xml"},
            {"tienkung", "assets/tienkung/mjcf/tienkung.xml"},
            {"pal_talos", "assets/pal_talos/talos.xml"},
            {"fourier_gr3", "assets/fourier_gr3v2_1_1/mjcf/gr3v2_1_1_dummy_hand.xml"},
        };
        return kMap;
    }

    inline const std::unordered_map<std::string, std::string>& robotUrdfMap() {
        static const std::unordered_map<std::string, std::string> kMap = {
            {"unitree_g1", "assets/unitree_g1/g1_custom_collision_29dof.urdf"},
            {"unitree_g1_with_hands", "assets/unitree_g1/g1_custom_collision_29dof.urdf"},
            {"unitree_h1", "assets/unitree_h1/h1.urdf"},
            {"unitree_h1_2", "assets/unitree_h1_2/h1_2_handless.urdf"},
            {"booster_t1", "assets/booster_t1/T1_serial.urdf"},
            {"stanford_toddy", "assets/stanford_toddy/assemblies/toddlerbot_active/toddlerbot_active.urdf"},
            {"kuavo_s45", "assets/kuavo_s45/biped_s45.urdf"},
            {"hightorque_hi", "assets/hightorque_hi/hi_25dof.urdf"},
            {"galaxea_r1pro", "assets/galaxea_r1pro/r1_pro_with_gripper.urdf"},
            {"berkeley_humanoid_lite", "assets/berkeley_humanoid_lite/berkeley_humanoid_lite_biped.urdf"},
            {"booster_k1", "assets/booster_k1/K1_serial.urdf"},
            {"pnd_adam_lite", "assets/pnd_adam_lite/adam_lite.urdf"},
            {"tienkung", "assets/tienkung/urdf/tienkung2_lite.urdf"},
            {"fourier_gr3", "assets/fourier_gr3v2_1_1/basic_urdf/gr3v2_1_1_dummy_hand.urdf"},
        };
        return kMap;
    }

    inline const std::unordered_map<std::string, std::string>& smplxIkConfigMap() {
        static const std::unordered_map<std::string, std::string> kMap = {
            {"unitree_g1", "general_motion_retargeting/ik_configs/smplx_to_g1.json"},
            {"unitree_g1_with_hands", "general_motion_retargeting/ik_configs/smplx_to_g1.json"},
            {"unitree_h1", "general_motion_retargeting/ik_configs/smplx_to_h1.json"},
            {"unitree_h1_2", "general_motion_retargeting/ik_configs/smplx_to_h1_2.json"},
            {"booster_t1", "general_motion_retargeting/ik_configs/smplx_to_t1.json"},
            {"booster_t1_29dof", "general_motion_retargeting/ik_configs/smplx_to_t1_29dof.json"},
            {"stanford_toddy", "general_motion_retargeting/ik_configs/smplx_to_toddy.json"},
            {"fourier_n1", "general_motion_retargeting/ik_configs/smplx_to_n1.json"},
            {"engineai_pm01", "general_motion_retargeting/ik_configs/smplx_to_pm01.json"},
            {"kuavo_s45", "general_motion_retargeting/ik_configs/smplx_to_kuavo.json"},
            {"hightorque_hi", "general_motion_retargeting/ik_configs/smplx_to_hi.json"},
            {"galaxea_r1pro", "general_motion_retargeting/ik_configs/smplx_to_r1pro.json"},
            {"berkeley_humanoid_lite", "general_motion_retargeting/ik_configs/smplx_to_bhl.json"},
            {"booster_k1", "general_motion_retargeting/ik_configs/smplx_to_k1.json"},
            {"pnd_adam_lite", "general_motion_retargeting/ik_configs/smplx_to_adam.json"},
            {"tienkung", "general_motion_retargeting/ik_configs/smplx_to_tienkung.json"},
            {"fourier_gr3", "general_motion_retargeting/ik_configs/smplx_to_gr3.json"},
        };
        return kMap;
    }

    inline const std::unordered_map<std::string, std::string>& bvhLafan1IkConfigMap() {
        static const std::unordered_map<std::string, std::string> kMap = {
            {"unitree_g1", "general_motion_retargeting/ik_configs/bvh_lafan1_to_g1.json"},
            {"unitree_g1_with_hands", "general_motion_retargeting/ik_configs/bvh_lafan1_to_g1.json"},
            {"booster_t1_29dof", "general_motion_retargeting/ik_configs/bvh_lafan1_to_t1_29dof.json"},
            {"fourier_n1", "general_motion_retargeting/ik_configs/bvh_lafan1_to_n1.json"},
            {"stanford_toddy", "general_motion_retargeting/ik_configs/bvh_lafan1_to_toddy.json"},
            {"engineai_pm01", "general_motion_retargeting/ik_configs/bvh_lafan1_to_pm01.json"},
            {"pal_talos", "general_motion_retargeting/ik_configs/bvh_to_talos.json"},
        };
        return kMap;
    }

    inline const std::unordered_map<std::string, std::string>& bvhNokovIkConfigMap() {
        static const std::unordered_map<std::string, std::string> kMap = {
            {"unitree_g1", "general_motion_retargeting/ik_configs/bvh_nokov_to_g1.json"},
        };
        return kMap;
    }

    inline std::filesystem::path resolveRobotXml(const std::filesystem::path& gmrRoot, const std::string& robot) {
        const auto& map = robotXmlMap();
        auto it         = map.find(robot);
        if (it == map.end()) {
            throw std::runtime_error("Unsupported robot: " + robot);
        }
        return gmrRoot / it->second;
    }

    inline std::filesystem::path resolveRobotUrdf(const std::filesystem::path& gmrRoot, const std::string& robot) {
        const auto& map = robotUrdfMap();
        auto it         = map.find(robot);
        if (it == map.end()) {
            throw std::runtime_error("Unsupported robot for Pinocchio URDF map: " + robot +
                                     ". Add it to robotUrdfMap() in cpp/include/gmr/retarget/repo_paths.h");
        }
        return gmrRoot / it->second;
    }

    inline std::filesystem::path resolveSmplxIkConfig(const std::filesystem::path& gmrRoot, const std::string& robot) {
        const auto& map = smplxIkConfigMap();
        auto it         = map.find(robot);
        if (it == map.end()) {
            throw std::runtime_error("Unsupported robot for SMPL-X IK config: " + robot);
        }
        return gmrRoot / it->second;
    }

    inline std::filesystem::path resolveIkConfig(const std::filesystem::path& gmrRoot, const std::string& srcHuman,
                                                 const std::string& robot) {
        const std::unordered_map<std::string, std::string>* map = nullptr;

        if (srcHuman == "smplx") {
            map = &smplxIkConfigMap();
        } else if (srcHuman == "bvh_lafan1") {
            map = &bvhLafan1IkConfigMap();
        } else if (srcHuman == "bvh_nokov") {
            map = &bvhNokovIkConfigMap();
        } else {
            throw std::runtime_error("Unsupported src_human for C++ retargeting: " + srcHuman);
        }

        auto it = map->find(robot);
        if (it == map->end()) {
            throw std::runtime_error("Unsupported robot '" + robot + "' for src_human '" + srcHuman + "'.");
        }
        return gmrRoot / it->second;
    }

}  // namespace gmr
