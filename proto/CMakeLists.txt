cmake_minimum_required(VERSION 3.20)
project(puppet_proto VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(CMakePackageConfigHelpers)

# Default install prefix:
# <repo_root>/devel/x86_64-Linux-GNU-9.4.0
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  get_filename_component(PUPPET_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
  set(CMAKE_INSTALL_PREFIX
      "${PUPPET_REPO_ROOT}/devel/x86_64-Linux-GNU-9.4.0"
      CACHE PATH "Install path prefix." FORCE)
endif()

find_program(PUPPET_PROTOC_EXECUTABLE protoc REQUIRED)
get_filename_component(PUPPET_PROTOC_BIN_DIR "${PUPPET_PROTOC_EXECUTABLE}" DIRECTORY)
get_filename_component(PUPPET_PROTOBUF_PREFIX "${PUPPET_PROTOC_BIN_DIR}/.." ABSOLUTE)

# Prefer the protobuf include/lib that matches current protoc.
if(EXISTS "${PUPPET_PROTOBUF_PREFIX}/include/google/protobuf/port_def.inc")
  set(Protobuf_PROTOC_EXECUTABLE "${PUPPET_PROTOC_EXECUTABLE}" CACHE FILEPATH "" FORCE)
  set(Protobuf_INCLUDE_DIR "${PUPPET_PROTOBUF_PREFIX}/include" CACHE PATH "" FORCE)

  if(EXISTS "${PUPPET_PROTOBUF_PREFIX}/lib/libprotobuf.so")
    set(Protobuf_LIBRARY "${PUPPET_PROTOBUF_PREFIX}/lib/libprotobuf.so" CACHE FILEPATH "" FORCE)
  elseif(EXISTS "${PUPPET_PROTOBUF_PREFIX}/lib/libprotobuf.a")
    set(Protobuf_LIBRARY "${PUPPET_PROTOBUF_PREFIX}/lib/libprotobuf.a" CACHE FILEPATH "" FORCE)
  endif()
endif()

find_package(Protobuf REQUIRED)

set(PUPPET_PROTO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(PUPPET_PROTO_FILES
    "${PUPPET_PROTO_ROOT}/puppet/common.proto"
    "${PUPPET_PROTO_ROOT}/puppet/uniform.proto"
    "${PUPPET_PROTO_ROOT}/puppet/primitive_frame.proto"
    "${PUPPET_PROTO_ROOT}/puppet/control_intent.proto"
    "${PUPPET_PROTO_ROOT}/puppet/primitive_trajectory.proto"
)

set(PUPPET_CPP_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/cpp")
set(PUPPET_PY_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/python")

file(MAKE_DIRECTORY "${PUPPET_CPP_GEN_DIR}")
file(MAKE_DIRECTORY "${PUPPET_PY_GEN_DIR}")

set(PUPPET_PROTO_CPP_SRCS)
set(PUPPET_PROTO_CPP_HDRS)
set(PUPPET_PROTO_PY_FILES)

foreach(PROTO_FILE IN LISTS PUPPET_PROTO_FILES)
  get_filename_component(PROTO_NAME "${PROTO_FILE}" NAME_WE)

  set(CPP_SRC "${PUPPET_CPP_GEN_DIR}/puppet/${PROTO_NAME}.pb.cc")
  set(CPP_HDR "${PUPPET_CPP_GEN_DIR}/puppet/${PROTO_NAME}.pb.h")
  set(PY_OUT "${PUPPET_PY_GEN_DIR}/puppet/${PROTO_NAME}_pb2.py")

  add_custom_command(
      OUTPUT "${CPP_SRC}" "${CPP_HDR}" "${PY_OUT}"
      COMMAND "${Protobuf_PROTOC_EXECUTABLE}"
      ARGS
        --proto_path=${PUPPET_PROTO_ROOT}
        --cpp_out=${PUPPET_CPP_GEN_DIR}
        --python_out=${PUPPET_PY_GEN_DIR}
        "${PROTO_FILE}"
      DEPENDS "${PROTO_FILE}"
      COMMENT "Generating protobuf for ${PROTO_NAME}"
      VERBATIM
  )

  list(APPEND PUPPET_PROTO_CPP_SRCS "${CPP_SRC}")
  list(APPEND PUPPET_PROTO_CPP_HDRS "${CPP_HDR}")
  list(APPEND PUPPET_PROTO_PY_FILES "${PY_OUT}")
endforeach()

set(PUPPET_SYNC_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/puppet")
set(PUPPET_SYNC_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/puppet")
set(PUPPET_SYNC_PY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/python/puppet")

file(MAKE_DIRECTORY "${PUPPET_SYNC_SRC_DIR}")
file(MAKE_DIRECTORY "${PUPPET_SYNC_INC_DIR}")
file(MAKE_DIRECTORY "${PUPPET_SYNC_PY_DIR}")

set(PUPPET_SYNC_OUTPUTS)
foreach(GEN_SRC IN LISTS PUPPET_PROTO_CPP_SRCS)
  get_filename_component(GEN_BASENAME "${GEN_SRC}" NAME)
  set(SYNC_DST "${PUPPET_SYNC_SRC_DIR}/${GEN_BASENAME}")
  add_custom_command(
      OUTPUT "${SYNC_DST}"
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${GEN_SRC}" "${SYNC_DST}"
      DEPENDS "${GEN_SRC}"
      VERBATIM
  )
  list(APPEND PUPPET_SYNC_OUTPUTS "${SYNC_DST}")
endforeach()

foreach(GEN_HDR IN LISTS PUPPET_PROTO_CPP_HDRS)
  get_filename_component(GEN_BASENAME "${GEN_HDR}" NAME)
  set(SYNC_DST "${PUPPET_SYNC_INC_DIR}/${GEN_BASENAME}")
  add_custom_command(
      OUTPUT "${SYNC_DST}"
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${GEN_HDR}" "${SYNC_DST}"
      DEPENDS "${GEN_HDR}"
      VERBATIM
  )
  list(APPEND PUPPET_SYNC_OUTPUTS "${SYNC_DST}")
endforeach()

foreach(GEN_PY IN LISTS PUPPET_PROTO_PY_FILES)
  get_filename_component(GEN_BASENAME "${GEN_PY}" NAME)
  set(SYNC_DST "${PUPPET_SYNC_PY_DIR}/${GEN_BASENAME}")
  add_custom_command(
      OUTPUT "${SYNC_DST}"
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${GEN_PY}" "${SYNC_DST}"
      DEPENDS "${GEN_PY}"
      VERBATIM
  )
  list(APPEND PUPPET_SYNC_OUTPUTS "${SYNC_DST}")
endforeach()

set(PUPPET_PY_INIT "${PUPPET_SYNC_PY_DIR}/__init__.py")
add_custom_command(
    OUTPUT "${PUPPET_PY_INIT}"
    COMMAND "${CMAKE_COMMAND}" -E touch "${PUPPET_PY_INIT}"
    DEPENDS ${PUPPET_SYNC_OUTPUTS}
    VERBATIM
)
list(APPEND PUPPET_SYNC_OUTPUTS "${PUPPET_PY_INIT}")

add_custom_target(generate_puppet_proto ALL
    DEPENDS ${PUPPET_PROTO_CPP_SRCS} ${PUPPET_PROTO_CPP_HDRS} ${PUPPET_PROTO_PY_FILES}
)

add_custom_target(sync_generated_puppet_proto ALL
    DEPENDS ${PUPPET_SYNC_OUTPUTS}
)
add_dependencies(sync_generated_puppet_proto generate_puppet_proto)

add_library(puppet_proto SHARED ${PUPPET_PROTO_CPP_SRCS})
add_dependencies(puppet_proto generate_puppet_proto)

target_include_directories(puppet_proto
    PUBLIC
      $<BUILD_INTERFACE:${PUPPET_CPP_GEN_DIR}>
      $<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}>
      $<INSTALL_INTERFACE:include>
)
target_link_libraries(puppet_proto PUBLIC protobuf::libprotobuf)

set_target_properties(puppet_proto PROPERTIES
    VERSION "${PROJECT_VERSION}"
    SOVERSION "${PROJECT_VERSION_MAJOR}"
)

install(TARGETS puppet_proto
    EXPORT puppet_proto-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(FILES ${PUPPET_PROTO_CPP_HDRS}
    DESTINATION include/puppet
)
install(DIRECTORY "${PUPPET_PY_GEN_DIR}/"
    DESTINATION lib/python3/site-packages
)
install(DIRECTORY "${PUPPET_PROTO_ROOT}/"
    DESTINATION share/puppet/proto
)

set(PUPPET_PROTO_CMAKE_INSTALL_DIR
    "lib/cmake/puppet_proto"
    CACHE STRING "Install directory for puppet_proto CMake package files.")

install(EXPORT puppet_proto-targets
    FILE puppet_proto-targets.cmake
    NAMESPACE puppet_proto::
    DESTINATION "${PUPPET_PROTO_CMAKE_INSTALL_DIR}"
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/puppet_proto-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/puppet_proto-config.cmake"
    INSTALL_DESTINATION "${PUPPET_PROTO_CMAKE_INSTALL_DIR}"
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/puppet_proto-config-version.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/puppet_proto-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/puppet_proto-config-version.cmake"
    DESTINATION "${PUPPET_PROTO_CMAKE_INSTALL_DIR}"
)
