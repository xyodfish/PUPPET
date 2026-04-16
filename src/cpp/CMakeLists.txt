set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(pinocchio REQUIRED)

set(PUPPET_CORE_SOURCES
  puppet/runtime/message_snapshot.cpp
  puppet/runtime/runtime_config.cpp
  puppet/runtime/teleop_runtime.cpp
  puppet/source/source_manager.cpp
  puppet/orchestrator/orchestrator.cpp
  puppet/retargeting/core/direct_pass_through_plugin.cpp
  puppet/retargeting/core/retargeting_pipeline.cpp
  puppet/retargeting/native/gmr_retargeting_plugin.cpp
  puppet/retargeting/native/gmr_retargeter_shim.cpp
  puppet/robot/backend/direct_mapping_backend.cpp)

set(PUPPET_GMR_CPP_ROOT "${CMAKE_SOURCE_DIR}/src/cpp/puppet/retargeting/native/gmr_cpp")

add_library(puppet_cpp_core STATIC ${PUPPET_CORE_SOURCES}
  ${PUPPET_GMR_CPP_ROOT}/src/solver/qp_solver.cpp
  ${PUPPET_GMR_CPP_ROOT}/src/solver/hqp_solver.cpp
  ${PUPPET_GMR_CPP_ROOT}/src/retarget/ik_config.cpp
  ${PUPPET_GMR_CPP_ROOT}/src/retarget/human_frame_io.cpp
  ${PUPPET_GMR_CPP_ROOT}/src/retarget/retargeter_pinocchio.cpp
  ${PUPPET_GMR_CPP_ROOT}/src/retarget/retargeter_pinocchio_legacy.cpp)

target_include_directories(puppet_cpp_core
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${PUPPET_GMR_CPP_ROOT}/include)

target_link_libraries(puppet_cpp_core
  PUBLIC
    yaml-cpp
    Eigen3::Eigen
    qpOASES
    pinocchio::pinocchio)
