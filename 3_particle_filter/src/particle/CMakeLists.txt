cmake_minimum_required(VERSION 3.5)
project(pf)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Debug)
# Required for ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Ceres REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.14 REQUIRED)
find_package(pcl_conversions REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})
include_directories(include)
# Include dirs
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
# Create executable
 add_executable(${PROJECT_NAME}_node src/Renderer.cpp src/particle_filter.cpp src/main.cpp)


ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  std_msgs
  PCL
  sensor_msgs
  pcl_conversions
)

target_link_libraries(pf_node ${PCL_LIBRARIES} Ceres::ceres gflags ${VTK_LIBRARIES}
)


install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()