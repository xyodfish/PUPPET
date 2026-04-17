cmake_minimum_required(VERSION 3.20)
project(PUPPET VERSION 0.1.0 LANGUAGES CXX)

option(PUPPET_BUILD_CPP_APPS "Build C++ runtime/tools/demos apps." ON)
option(PUPPET_BUILD_TESTS "Build tests." ON)


add_subdirectory(src/cpp)

if(PUPPET_BUILD_CPP_APPS)
  add_subdirectory(app/cpp)
endif()

if(PUPPET_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
