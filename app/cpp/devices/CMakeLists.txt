# Device-specific executable targets.
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
set(EMBOSA_INCLUDE_DIR "${ROOT_DEVEL_ARCH_PATH}/include/embosa")
set(EMBOSA_LIBRARY_DIR "${ROOT_DEVEL_ARCH_PATH}/lib")
# Use full paths for EMBOSA libraries to avoid version conflicts
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

add_executable(static_file_replay_device
  static_file_replay_device.cpp)
target_include_directories(static_file_replay_device
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${EMBOSA_INCLUDE_DIR})
target_link_directories(static_file_replay_device
  PRIVATE
    ${EMBOSA_LIBRARY_DIR})
target_link_libraries(static_file_replay_device
  PRIVATE
    puppet_cpp_core)
set_target_properties(static_file_replay_device
  PROPERTIES
    BUILD_RPATH "${EMBOSA_LIBRARY_DIR}"
    INSTALL_RPATH "${EMBOSA_LIBRARY_DIR}")

