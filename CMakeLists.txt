cmake_minimum_required(VERSION 3.5)
project(air_vo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set Release build type by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Add custom CMake modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
add_definitions(-w)

# Add TensorRTBuffer subdirectory
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/TensorRTBuffer)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(G2O REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${GFLAGS_INCLUDE_DIRS} 
  ${GLOG_INCLUDE_DIRS}
  # Include ROS 2 headers
  ${rclcpp_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}_lib SHARED
  src/g2o_optimization/vertex_line3d.cc
  src/g2o_optimization/edge_project_line.cc
  src/g2o_optimization/edge_project_stereo_line.cc
  src/g2o_optimization/g2o_optimization.cc
  src/super_point.cpp
  src/super_glue.cpp
  src/utils.cc
  src/camera.cc
  src/dataset.cc
  src/frame.cc
  src/point_matching.cc
  src/mappoint.cc
  src/mapline.cc
  src/line_processor.cc
  src/ros_publisher.cc
  src/map.cc
  src/map_builder.cc
  src/timer.cc
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib
  nvinfer
  nvonnxparser
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${Boost_LIBRARIES}
  ${G2O_LIBRARIES}
  ${GFLAGS_LIBRARIES} 
  ${GLOG_LIBRARIES}
  yaml-cpp
  TensorRTBuffer
)

# Declare C++ executables
add_executable(${PROJECT_NAME} main.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  std_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME}_ros ros_main.cpp)
ament_target_dependencies(${PROJECT_NAME}_ros
  rclcpp
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  std_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME}_lib)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_ros
  ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Export information to downstream packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp cv_bridge geometry_msgs image_transport nav_msgs std_msgs sensor_msgs)

# Must be the last command
ament_package()
