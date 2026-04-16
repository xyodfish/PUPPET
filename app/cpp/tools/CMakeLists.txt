# Internal tooling executable targets.

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

# Follow gmr_cpp dependency discovery style.
set(GMR_THIRDPARTY_PREFIX "${ROOT_DEVEL_ARCH_PATH}" CACHE PATH "Third-party install prefix")
set(GMR_MUJOCO_PREFIX "/opt/galbot/devel_control/${ARCH_FREFIX}" CACHE PATH "MuJoCo install prefix")
list(PREPEND CMAKE_PREFIX_PATH "${GMR_THIRDPARTY_PREFIX}")
list(PREPEND CMAKE_PREFIX_PATH "${GMR_MUJOCO_PREFIX}")

find_package(puppet_proto CONFIG QUIET)
find_package(glfw3 QUIET)

set(EMBOSA_INCLUDE_DIR "${ROOT_DEVEL_ARCH_PATH}/include/embosa")
set(EMBOSA_LIBRARY_DIR "${ROOT_DEVEL_ARCH_PATH}/lib")
if(EXISTS "${EMBOSA_INCLUDE_DIR}/embosa.hpp")
  set(EMBOSA_INCLUDE_OK TRUE)
else()
  set(EMBOSA_INCLUDE_OK FALSE)
endif()
if(EXISTS "${EMBOSA_LIBRARY_DIR}")
  set(EMBOSA_LIBDIR_OK TRUE)
else()
  set(EMBOSA_LIBDIR_OK FALSE)
endif()

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  PATHS "${GMR_THIRDPARTY_PREFIX}/include" "${GMR_MUJOCO_PREFIX}/include")
find_library(MUJOCO_LIBRARY
  NAMES mujoco libmujoco
  PATHS "${GMR_THIRDPARTY_PREFIX}/lib" "${GMR_MUJOCO_PREFIX}/lib")
if(NOT MUJOCO_LIBRARY OR MUJOCO_LIBRARY STREQUAL "MUJOCO_LIBRARY-NOTFOUND")
  find_file(MUJOCO_LIBRARY_FILE
    NAMES libmujoco.so.3.3.0 libmujoco.so
    PATHS "${GMR_THIRDPARTY_PREFIX}/lib" "${GMR_MUJOCO_PREFIX}/lib")
  if(MUJOCO_LIBRARY_FILE)
    set(MUJOCO_LIBRARY "${MUJOCO_LIBRARY_FILE}")
  endif()
endif()

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

if(puppet_proto_FOUND AND MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY AND EMBOSA_INCLUDE_OK AND EMBOSA_LIBDIR_OK)
  add_executable(retargeting_mujoco_visualizer
    retargeting_mujoco_visualizer.cpp)

  target_include_directories(retargeting_mujoco_visualizer
    PRIVATE
      ${CMAKE_SOURCE_DIR}/include
      ${EMBOSA_INCLUDE_DIR}
      ${MUJOCO_INCLUDE_DIR})

  target_link_directories(retargeting_mujoco_visualizer
    PRIVATE
      ${EMBOSA_LIBRARY_DIR})

  target_link_libraries(retargeting_mujoco_visualizer
    PRIVATE
      puppet_cpp_core
      puppet_proto::puppet_proto
      yaml-cpp
      ${MUJOCO_LIBRARY}
      ${EMBOSA_LIBRARIES}
      pthread)

  if(glfw3_FOUND)
    target_link_libraries(retargeting_mujoco_visualizer PRIVATE glfw)
  else()
    find_library(GLFW_LIBRARY NAMES glfw REQUIRED)
    target_link_libraries(retargeting_mujoco_visualizer PRIVATE ${GLFW_LIBRARY})
  endif()

  set_target_properties(retargeting_mujoco_visualizer
    PROPERTIES
      BUILD_RPATH "${EMBOSA_LIBRARY_DIR};${GMR_THIRDPARTY_PREFIX}/lib;${GMR_MUJOCO_PREFIX}/lib"
      INSTALL_RPATH "${EMBOSA_LIBRARY_DIR};${GMR_THIRDPARTY_PREFIX}/lib;${GMR_MUJOCO_PREFIX}/lib")
else()
  message(WARNING
    "Skip retargeting_mujoco_visualizer: "
    "puppet_proto_FOUND=${puppet_proto_FOUND}, "
    "MUJOCO_INCLUDE_DIR=${MUJOCO_INCLUDE_DIR}, "
    "MUJOCO_LIBRARY=${MUJOCO_LIBRARY}, "
    "EMBOSA_INCLUDE_OK=${EMBOSA_INCLUDE_OK}, "
    "EMBOSA_LIBDIR_OK=${EMBOSA_LIBDIR_OK}")
endif()
