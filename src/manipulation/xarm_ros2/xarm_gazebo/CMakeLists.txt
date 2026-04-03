cmake_minimum_required(VERSION 3.5)
project(xarm_gazebo)

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
find_package(gz-sim8 REQUIRED)
find_package(sdformat14 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_library(gazebo_mimic_joint_plugin SHARED
  src/mimic_joint_plugin.cpp
)

# Set C++ standard
target_compile_features(gazebo_mimic_joint_plugin PRIVATE cxx_std_17)

# Link dependencies
target_link_libraries(gazebo_mimic_joint_plugin
  gz-sim8::gz-sim8
)

# Install the plugin library
install(TARGETS gazebo_mimic_joint_plugin
  DESTINATION lib
)

install(DIRECTORY
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()