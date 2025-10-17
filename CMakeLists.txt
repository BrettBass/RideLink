cmake_minimum_required(VERSION 3.22)

set(EXTRA_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#idf_build_set_property(MINIMAL_BUILD ON)
project(RideLink)
