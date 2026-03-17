cmake_minimum_required(VERSION 3.8)
project(stem_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(stem_processing src/stem_processing_node.cpp)
ament_target_dependencies(stem_processing rclcpp sensor_msgs pcl_conversions)
target_link_libraries(stem_processing ${PCL_LIBRARIES})


install(TARGETS
  stem_processing
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =========================
# Install Python code
# =========================

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install entry points (scripts)
install(PROGRAMS
  stem_processing/stem_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources (like launch files, weights, etc.)
install(DIRECTORY
  launch
  resource
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
