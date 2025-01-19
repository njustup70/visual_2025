cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# find_package(sensor_msgs REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_ros REQUIRED)
include_directories(./include
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(tf_trans src/tf_trans.cpp)
target_link_libraries(tf_trans fmt::fmt yaml-cpp)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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

ament_target_dependencies(tf_trans
  "rclcpp"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "tf2_geometry_msgs"
)
install(
  TARGETS tf_trans
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
