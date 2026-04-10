
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was puppet_proto-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)

set(_puppetProtoProtobufPrefix "/opt/galbot/devel/x86_64-Linux-GNU-9.4.0")
if(EXISTS "${_puppetProtoProtobufPrefix}/include/google/protobuf/port_def.inc")
  set(Protobuf_INCLUDE_DIR "${_puppetProtoProtobufPrefix}/include")
  if(EXISTS "${_puppetProtoProtobufPrefix}/lib/libprotobuf.so")
    set(Protobuf_LIBRARY "${_puppetProtoProtobufPrefix}/lib/libprotobuf.so")
  elseif(EXISTS "${_puppetProtoProtobufPrefix}/lib/libprotobuf.a")
    set(Protobuf_LIBRARY "${_puppetProtoProtobufPrefix}/lib/libprotobuf.a")
  endif()
  if(EXISTS "${_puppetProtoProtobufPrefix}/bin/protoc")
    set(Protobuf_PROTOC_EXECUTABLE "${_puppetProtoProtobufPrefix}/bin/protoc")
  endif()
endif()

find_dependency(Protobuf REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/puppet_proto-targets.cmake")
