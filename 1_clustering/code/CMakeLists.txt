cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_extraction)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

# ------------------------------------------------------------------------------
# 2. Ensure debug symbols are always included
#    (Even if someone builds with Release or RelWithDebInfo)
# ------------------------------------------------------------------------------
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")          # keep symbols even in release
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -g")


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set (CMAKE_CXX_STANDARD 11)


add_executable (cluster_extraction src/cluster_extraction.cpp src/Renderer.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
