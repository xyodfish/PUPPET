set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(pinocchio REQUIRED)
find_package(glog REQUIRED)
find_package(puppet_proto CONFIG QUIET)


set(PUPPET_REPO_ROOT ${CMAKE_SOURCE_DIR})
set(PUPPET_DEFAULT_DEVEL_PREFIX
    "${PUPPET_REPO_ROOT}/devel/${CMAKE_SYSTEM_PROCESSOR}-Linux-GNU-${CMAKE_CXX_COMPILER_VERSION}")
list(APPEND CMAKE_PREFIX_PATH "${PUPPET_DEFAULT_DEVEL_PREFIX}")

if (NOT ARCH_FREFIX)
  set(ARCH_FREFIX "${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}-${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}")
endif()

if (NOT ROOT_DEVEL_ARCH_PATH)
  set(ROOT_DEVEL_ARCH_PATH "/opt/robot/devel/${ARCH_FREFIX}")
endif()

set(EMBOSA_INCLUDE_DIR "${ROOT_DEVEL_ARCH_PATH}/include/embosa")
set(EMBOSA_LIBRARY_DIR "${ROOT_DEVEL_ARCH_PATH}/lib")
set(ZMQ_INCLUDE_DIR "${ROOT_DEVEL_ARCH_PATH}/include")
set(ZMQ_LIBRARY_DIR "${ROOT_DEVEL_ARCH_PATH}/lib")
set(ZMQ_LIBRARIES
    ${ZMQ_LIBRARY_DIR}/libzmq.so)
set(SCALED_DEVICE_SDK_ROOT "${CMAKE_SOURCE_DIR}/third_party/galbot_remote_operate"
    CACHE PATH "Path to scaled device SDK root that contains include/ and lib/")
set(SCALED_DEVICE_SDK_INCLUDE_DIR "${SCALED_DEVICE_SDK_ROOT}/include")
set(SCALED_DEVICE_SDK_LIBRARY_DIR "${SCALED_DEVICE_SDK_ROOT}/lib")
if (NOT EXISTS "${SCALED_DEVICE_SDK_INCLUDE_DIR}/GalbotRemoteOperate/RemoteOperate.h")
  message(FATAL_ERROR "Scaled device SDK header not found: ${SCALED_DEVICE_SDK_INCLUDE_DIR}/GalbotRemoteOperate/RemoteOperate.h")
endif()
if (NOT EXISTS "${SCALED_DEVICE_SDK_LIBRARY_DIR}/libgalbotRemoteOperate.so")
  message(FATAL_ERROR "Scaled device SDK library not found: ${SCALED_DEVICE_SDK_LIBRARY_DIR}/libgalbotRemoteOperate.so")
endif()

message(STATUS "EMBOSA_INCLUDE_DIR: ${EMBOSA_INCLUDE_DIR}")
message(STATUS "EMBOSA_LIBRARY_DIR: ${EMBOSA_LIBRARY_DIR}")
message(STATUS "ZMQ_INCLUDE_DIR: ${ZMQ_INCLUDE_DIR}")
message(STATUS "ZMQ_LIBRARY_DIR: ${ZMQ_LIBRARY_DIR}")
message(STATUS "SCALED_DEVICE_SDK_ROOT: ${SCALED_DEVICE_SDK_ROOT}")

# 使用完整路径确保链接正确版本的 embosa 核心库
# 其他依赖库使用名称,由链接器在 EMBOSA_LIBRARY_DIR 中查找
set(EMBOSA_LIBRARIES
    ${EMBOSA_LIBRARY_DIR}/libembosa.so
    ${EMBOSA_LIBRARY_DIR}/libfastcdr.so
    ${EMBOSA_LIBRARY_DIR}/libfastrtps.so
    ${EMBOSA_LIBRARY_DIR}/libfoonathan_memory-0.7.3.so
    ssl
    crypto
    tinyxml2
    spdlog
    protobuf
    z
    rt
    dl
    boost_thread)


set(PUPPET_CORE_SOURCES
  puppet/device/device_service_config.cpp
  puppet/device/device_service.cpp
  puppet/device/device_provider_factory.cpp
  puppet/device/static_file_replay/static_file_replay_device_provider.cpp
  puppet/device/single_chain_ik/single_chain_ik_sender_device_provider.cpp
  puppet/device/scaled/scaled_device_provider.cpp
  puppet/runtime/robot_state_sync.cpp
  puppet/runtime/runtime_config.cpp
  puppet/runtime/teleop_runtime.cpp
  puppet/runtime/runtime_state_report.cpp
  puppet/source/source_manager.cpp
  puppet/transport/device_output_channel_factory.cpp
  puppet/transport/embosa_device_output_channel.cpp
  puppet/transport/embosa_runtime_channel.cpp
  puppet/transport/embosa_runtime_config.cpp
  puppet/transport/zmq_device_output_channel.cpp
  puppet/transport/zmq_runtime_channel.cpp
  puppet/transport/zmq_runtime_config.cpp
  puppet/transport/message_snapshot.cpp
  puppet/transport/proto_copy.cpp
  puppet/orchestrator/orchestrator.cpp
  puppet/retargeting/core/direct_pass_through_plugin.cpp
  puppet/retargeting/core/retargeting_pipeline.cpp
  puppet/retargeting/native/single_chain_ik/single_chain_ik_retargeting_plugin.cpp
  puppet/retargeting/native/gmr/gmr_retargeting_plugin.cpp
  puppet/retargeting/native/gmr/gmr_retargeter_shim.cpp
  puppet/robot/backend/direct_mapping_backend.cpp
  puppet/common/time_utils.cpp
  puppet/config/puppet_config.cpp
  puppet/core/puppet_manager.cpp
  )

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
    ${CMAKE_SOURCE_DIR}/include/puppet/retargeting/native
    ${EMBOSA_INCLUDE_DIR}
    ${ZMQ_INCLUDE_DIR}
    ${PUPPET_DEFAULT_DEVEL_PREFIX}
)

target_include_directories(puppet_cpp_core PUBLIC ${SCALED_DEVICE_SDK_INCLUDE_DIR})
target_link_directories(puppet_cpp_core PUBLIC ${SCALED_DEVICE_SDK_LIBRARY_DIR})

# 添加库搜索路径,让链接器能找到其他依赖库
# 使用 target_link_directories 并设置为 PUBLIC,确保依赖 puppet_cpp_core 的目标也能找到这些库
target_link_directories(puppet_cpp_core PUBLIC ${EMBOSA_LIBRARY_DIR})
target_link_directories(puppet_cpp_core PUBLIC ${ZMQ_LIBRARY_DIR})

target_link_libraries(puppet_cpp_core
  PUBLIC
    yaml-cpp
    Eigen3::Eigen
    qpOASES
    pinocchio::pinocchio
    glog::glog
    puppet_proto::puppet_proto
    ${OROCOS_KDL_LIBRARY}
    ${TRAC_IK_LIBRARY}
    ${MUJOCO_LIBRARY}
    ${EMBOSA_LIBRARIES}
    ${ZMQ_LIBRARIES}
    )

target_link_libraries(puppet_cpp_core
  PUBLIC
    ${SCALED_DEVICE_SDK_LIBRARY_DIR}/libgalbotRemoteOperate.so
    ${SCALED_DEVICE_SDK_LIBRARY_DIR}/libgalbotSystem.so
    ${SCALED_DEVICE_SDK_LIBRARY_DIR}/libgalbotSerialPort.so
    ${SCALED_DEVICE_SDK_LIBRARY_DIR}/libgalbotDataProtocol.so
    ${SCALED_DEVICE_SDK_LIBRARY_DIR}/libgalbotLog.so)
