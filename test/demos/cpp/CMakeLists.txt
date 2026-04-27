set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(PUPPET_ENABLE_EMBOSA_DEMOS "Build embosa-based proto communication demos." ON)

if(NOT PUPPET_ENABLE_EMBOSA_DEMOS)
  message(STATUS "PUPPET_ENABLE_EMBOSA_DEMOS=OFF, skip embosa demos.")
  return()
endif()

get_filename_component(PUPPET_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(PUPPET_DEFAULT_DEVEL_PREFIX
    "${PUPPET_REPO_ROOT}/devel/${CMAKE_SYSTEM_PROCESSOR}-Linux-GNU-${CMAKE_CXX_COMPILER_VERSION}")
list(APPEND CMAKE_PREFIX_PATH "${PUPPET_DEFAULT_DEVEL_PREFIX}")

find_package(puppet_proto CONFIG QUIET)
if(NOT puppet_proto_FOUND)
  message(WARNING "puppet_proto package not found. Skip embosa demos. "
                  "Expected prefix example: ${PUPPET_DEFAULT_DEVEL_PREFIX}")
  return()
endif()

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

if(NOT EXISTS "${EMBOSA_INCLUDE_DIR}/embosa.hpp" OR NOT EXISTS "${EMBOSA_LIBRARY_DIR}")
  message(WARNING "embosa include/lib not found under ${ROOT_DEVEL_ARCH_PATH}. Skip embosa demos.")
  return()
endif()

include_directories("${EMBOSA_INCLUDE_DIR}")
link_directories("${EMBOSA_LIBRARY_DIR}")

add_library(puppet_proto_copy_demo STATIC
  "${PUPPET_REPO_ROOT}/src/cpp/puppet/transport/proto_copy.cpp")
target_include_directories(puppet_proto_copy_demo
  PUBLIC
    "${PUPPET_REPO_ROOT}/include")
target_link_libraries(puppet_proto_copy_demo
  PUBLIC
    puppet_proto::puppet_proto)

add_executable(test_embosa_puppet_proto_sender
  "${CMAKE_CURRENT_LIST_DIR}/embosa_puppet_proto_sender.cpp")
target_include_directories(test_embosa_puppet_proto_sender
  PRIVATE
    "${EMBOSA_INCLUDE_DIR}")
target_link_libraries(test_embosa_puppet_proto_sender
  PRIVATE
    puppet_proto_copy_demo
    ${EMBOSA_LIBRARIES}
    pthread)

add_executable(test_embosa_single_chain_ik_sender
  "${CMAKE_CURRENT_LIST_DIR}/embosa_single_chain_ik_sender.cpp")
target_include_directories(test_embosa_single_chain_ik_sender
  PRIVATE
    "${EMBOSA_INCLUDE_DIR}")
target_link_libraries(test_embosa_single_chain_ik_sender
  PRIVATE
    puppet_proto_copy_demo
    ${EMBOSA_LIBRARIES}
    pthread)

add_executable(test_zmq_single_chain_ik_sender
  "${CMAKE_CURRENT_LIST_DIR}/zmq_single_chain_ik_sender.cpp")
target_include_directories(test_zmq_single_chain_ik_sender
  PRIVATE
    "${ZMQ_INCLUDE_DIR}")
target_link_directories(test_zmq_single_chain_ik_sender
  PRIVATE
    "${ZMQ_LIBRARY_DIR}")
target_link_libraries(test_zmq_single_chain_ik_sender
  PRIVATE
    puppet_proto_copy_demo
    zmq
    pthread)

add_executable(test_tcp_single_chain_ik_sender
  "${CMAKE_CURRENT_LIST_DIR}/tcp_single_chain_ik_sender.cpp")
target_link_libraries(test_tcp_single_chain_ik_sender
  PRIVATE
    puppet_proto_copy_demo
    pthread)

add_executable(test_tcp_control_intent_receiver
  "${CMAKE_CURRENT_LIST_DIR}/tcp_control_intent_receiver.cpp")
target_link_libraries(test_tcp_control_intent_receiver
  PRIVATE
    puppet_proto_copy_demo
    pthread)

add_executable(test_udp_single_chain_ik_sender
  "${CMAKE_CURRENT_LIST_DIR}/udp_single_chain_ik_sender.cpp")
target_link_libraries(test_udp_single_chain_ik_sender
  PRIVATE
    puppet_proto_copy_demo
    pthread)

add_executable(test_udp_control_intent_receiver
  "${CMAKE_CURRENT_LIST_DIR}/udp_control_intent_receiver.cpp")
target_link_libraries(test_udp_control_intent_receiver
  PRIVATE
    puppet_proto_copy_demo
    pthread)

add_executable(test_embosa_puppet_proto_receiver
  "${CMAKE_CURRENT_LIST_DIR}/embosa_puppet_proto_receiver.cpp")
target_include_directories(test_embosa_puppet_proto_receiver
  PRIVATE
    "${EMBOSA_INCLUDE_DIR}")
target_link_libraries(test_embosa_puppet_proto_receiver
  PRIVATE
    puppet_proto_copy_demo
    ${EMBOSA_LIBRARIES}
    pthread)

# Ensure runtime can find embosa shared libraries directly.
set_target_properties(
  test_embosa_puppet_proto_sender
  test_embosa_single_chain_ik_sender
  test_embosa_puppet_proto_receiver
  test_zmq_single_chain_ik_sender
  test_tcp_single_chain_ik_sender
  test_tcp_control_intent_receiver
  test_udp_single_chain_ik_sender
  test_udp_control_intent_receiver
  PROPERTIES
    BUILD_RPATH "${EMBOSA_LIBRARY_DIR};${ZMQ_LIBRARY_DIR}"
    INSTALL_RPATH "${EMBOSA_LIBRARY_DIR};${ZMQ_LIBRARY_DIR}")
