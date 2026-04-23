cmake_minimum_required(VERSION 3.5)
project(d435i_xarm_setup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(object_recognition_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_api REQUIRED)

set(dependencies "rclcpp" "std_msgs" "tf2_ros" "tf2_geometry_msgs" "object_recognition_msgs" "xarm_msgs" "xarm_api")

add_executable(findobj_grasp_moveit_planner src/findobj_grasp_moveit_planner.cpp)
ament_target_dependencies(findobj_grasp_moveit_planner
  ${dependencies}
)

add_executable(findobj_grasp_xarm_api src/findobj_grasp_xarm_api.cpp)
ament_target_dependencies(findobj_grasp_xarm_api
  ${dependencies}
)

add_executable(tf_object_to_base src/tf_object_to_base.cpp)
ament_target_dependencies(tf_object_to_base
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  objects
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS
  findobj_grasp_moveit_planner 
  findobj_grasp_xarm_api 
  tf_object_to_base
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
