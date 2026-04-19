set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(pinocchio REQUIRED)
find_package(glog REQUIRED)


set(PUPPET_CORE_SOURCES
  puppet/runtime/message_snapshot.cpp
  puppet/runtime/robot_state_sync.cpp
  puppet/runtime/runtime_config.cpp
  puppet/runtime/teleop_runtime.cpp
  puppet/source/source_manager.cpp
  puppet/orchestrator/orchestrator.cpp
  puppet/retargeting/core/direct_pass_through_plugin.cpp
  puppet/retargeting/core/retargeting_pipeline.cpp
  puppet/retargeting/native/single_chain_ik/single_chain_ik_retargeting_plugin.cpp
  puppet/retargeting/native/gmr/gmr_retargeting_plugin.cpp
  puppet/retargeting/native/gmr/gmr_retargeter_shim.cpp
  puppet/robot/backend/direct_mapping_backend.cpp)

set(PUPPET_GMR_CPP_ROOT "${CMAKE_SOURCE_DIR}/src/cpp/puppet/retargeting/native/gmr")

add_library(puppet_cpp_core SHARED ${PUPPET_CORE_SOURCES}
  ${PUPPET_GMR_CPP_ROOT}/solver/qp_solver.cpp
  ${PUPPET_GMR_CPP_ROOT}/solver/hqp_solver.cpp
  ${PUPPET_GMR_CPP_ROOT}/retarget/ik_config.cpp
  ${PUPPET_GMR_CPP_ROOT}/retarget/human_frame_io.cpp
  ${PUPPET_GMR_CPP_ROOT}/retarget/retargeter_mujoco.cpp
  ${PUPPET_GMR_CPP_ROOT}/retarget/retargeter_mujoco_legacy.cpp
  ${PUPPET_GMR_CPP_ROOT}/retarget/retargeter_pinocchio.cpp
  ${PUPPET_GMR_CPP_ROOT}/retarget/retargeter_pinocchio_legacy.cpp)

find_library(OROCOS_KDL_LIBRARY NAMES orocos-kdl REQUIRED)
find_library(TRAC_IK_LIBRARY NAMES trac_ik REQUIRED)
# find_library(MUJOCO_LIBRARY NAMES mujoco REQUIRED)

target_include_directories(puppet_cpp_core
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/puppet/retargeting/native)

target_link_libraries(puppet_cpp_core
  PUBLIC
    yaml-cpp
    Eigen3::Eigen
    qpOASES
    pinocchio::pinocchio
    glog::glog
    ${OROCOS_KDL_LIBRARY}
    ${TRAC_IK_LIBRARY}
    ${MUJOCO_LIBRARY})
