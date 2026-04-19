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

set(EMBOSA_INCLUDE_DIR "${ROOT_DEVEL_ARCH_PATH}/include/embosa")
set(EMBOSA_LIBRARY_DIR "${ROOT_DEVEL_ARCH_PATH}/lib")


find_package(puppet_proto CONFIG QUIET)
find_package(glog REQUIRED)


if(puppet_proto_FOUND AND EXISTS "${EMBOSA_INCLUDE_DIR}/embosa.hpp" AND EXISTS "${EMBOSA_LIBRARY_DIR}")
  add_executable(teleop_runtime_embosa_main
    teleop_runtime_embosa_main.cpp)

  target_include_directories(teleop_runtime_embosa_main
    PRIVATE
      ${CMAKE_SOURCE_DIR}/include
      ${EMBOSA_INCLUDE_DIR})

  target_link_directories(teleop_runtime_embosa_main
    PRIVATE
      ${EMBOSA_LIBRARY_DIR})

  target_link_libraries(teleop_runtime_embosa_main
    PRIVATE
      puppet_cpp_core)

  set_target_properties(teleop_runtime_embosa_main
    PROPERTIES
      BUILD_RPATH "${EMBOSA_LIBRARY_DIR}"
      INSTALL_RPATH "${EMBOSA_LIBRARY_DIR}")
else()
  message(WARNING "Skip teleop_runtime_embosa_main: puppet_proto or embosa dependency not found")
endif()
