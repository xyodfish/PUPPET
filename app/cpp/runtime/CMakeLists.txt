add_executable(teleop_runtime_main
  teleop_runtime_main.cpp)

target_include_directories(teleop_runtime_main
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(teleop_runtime_main
  PRIVATE
    puppet_cpp_core)

# embosa-based runtime reader: subscribe PrimitiveFrame proto and run runtime pipeline.
if (NOT ARCH_FREFIX)
  set(ARCH_FREFIX "${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}-${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}")
endif()
if (NOT ROOT_DEVEL_ARCH_PATH)
  set(ROOT_DEVEL_ARCH_PATH "/opt/robot/devel/${ARCH_FREFIX}")
endif()

set(PUPPET_REPO_ROOT ${CMAKE_SOURCE_DIR})
set(PUPPET_DEFAULT_DEVEL_PREFIX
    "${PUPPET_REPO_ROOT}/devel/${CMAKE_SYSTEM_PROCESSOR}-Linux-GNU-${CMAKE_CXX_COMPILER_VERSION}")
list(APPEND CMAKE_PREFIX_PATH "${PUPPET_DEFAULT_DEVEL_PREFIX}")

find_package(puppet_proto CONFIG QUIET)
find_package(glog REQUIRED)
set(EMBOSA_INCLUDE_DIR "${ROOT_DEVEL_ARCH_PATH}/include/embosa")
set(EMBOSA_LIBRARY_DIR "${ROOT_DEVEL_ARCH_PATH}/lib")
set(EMBOSA_LIBRARIES
    embosa
    fastcdr
    fastrtps
    foonathan_memory-0.7.3
    ssl
    crypto
    tinyxml2
    spdlog
    protobuf
    z
    rt
    dl
    boost_thread)

if(puppet_proto_FOUND AND EXISTS "${EMBOSA_INCLUDE_DIR}/embosa.hpp" AND EXISTS "${EMBOSA_LIBRARY_DIR}")
  add_executable(teleop_runtime_embosa_main
    teleop_runtime_embosa_main.cpp
    ${CMAKE_SOURCE_DIR}/src/cpp/puppet/runtime/embosa_runtime_config.cpp
    ${CMAKE_SOURCE_DIR}/src/cpp/puppet/runtime/embosa_runtime_channel.cpp
    ${CMAKE_SOURCE_DIR}/src/puppet/config/puppet_config.cpp
    ${CMAKE_SOURCE_DIR}/src/cpp/puppet/runtime/time_utils.cpp
    ${CMAKE_SOURCE_DIR}/src/cpp/puppet/runtime/runtime_state_report.cpp
    ${CMAKE_SOURCE_DIR}/src/cpp/puppet/runtime/puppet_manager.cpp
    ${CMAKE_SOURCE_DIR}/src/cpp/puppet/transport/proto_copy.cpp)

  target_include_directories(teleop_runtime_embosa_main
    PRIVATE
      ${CMAKE_SOURCE_DIR}/include
      ${EMBOSA_INCLUDE_DIR})

  target_link_directories(teleop_runtime_embosa_main
    PRIVATE
      ${EMBOSA_LIBRARY_DIR})

  target_link_libraries(teleop_runtime_embosa_main
    PRIVATE
      puppet_cpp_core
      puppet_proto::puppet_proto
      yaml-cpp
      glog::glog
      ${EMBOSA_LIBRARIES}
      pthread)

  set_target_properties(teleop_runtime_embosa_main
    PROPERTIES
      BUILD_RPATH "${EMBOSA_LIBRARY_DIR}"
      INSTALL_RPATH "${EMBOSA_LIBRARY_DIR}")
else()
  message(WARNING "Skip teleop_runtime_embosa_main: puppet_proto or embosa dependency not found")
endif()
