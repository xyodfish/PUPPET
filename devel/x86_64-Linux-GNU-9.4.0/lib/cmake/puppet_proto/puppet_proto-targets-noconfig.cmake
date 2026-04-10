#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "puppet_proto::puppet_proto" for configuration ""
set_property(TARGET puppet_proto::puppet_proto APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(puppet_proto::puppet_proto PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpuppet_proto.so.0.1.0"
  IMPORTED_SONAME_NOCONFIG "libpuppet_proto.so.0"
  )

list(APPEND _cmake_import_check_targets puppet_proto::puppet_proto )
list(APPEND _cmake_import_check_files_for_puppet_proto::puppet_proto "${_IMPORT_PREFIX}/lib/libpuppet_proto.so.0.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
