cmake_minimum_required(VERSION 3.8)
project(vlp16_logger_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# -------------------------------
# Add executable 
add_executable(vlp16_republisher src/vlp16_republisher.cpp)
ament_target_dependencies(vlp16_republisher rclcpp sensor_msgs std_msgs)

# install the executable files under lib/${PROJECT_NAME} path
install(TARGETS
  vlp16_republisher
  DESTINATION lib/${PROJECT_NAME})

# Declare package
ament_package()
