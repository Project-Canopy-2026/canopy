cmake_minimum_required(VERSION 3.8)
project(manipulation_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pick_and_place_node src/pick_and_place_node.cpp)

target_include_directories(pick_and_place_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(pick_and_place_node PUBLIC cxx_std_17)

ament_target_dependencies(pick_and_place_node
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  geometry_msgs
  std_msgs
  std_srvs
  moveit_msgs
  shape_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(pick_and_place_node Eigen3::Eigen)

install(TARGETS pick_and_place_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
