cmake_minimum_required(VERSION 3.8)
project(pcl_handle)
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

# 设置 LLD 为链接器
add_link_options("-fuse-ld=lld")

# 设置构建系统生成器（Ninja）
# set(CMAKE_GENERATOR "Ninja" CACHE STRING "Build system generator" FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# set(CMAKE_C_COMPILER_LAUNCHER ccache)
# set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

  # 编译选项
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )

  # 链接优化
  add_link_options(

    -Wl,--threads=16 # 并行链接线程数

    # -Wl,--lto=thin # 启用 ThinLTO 优化
  )
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL 1.13.1 REQUIRED)
ament_auto_find_build_dependencies()
set(PclHandle "PclHandle")
set(PclIcp "PclIcp")
set(common_lib "common_lib")
file(GLOB PclCommonSrc "src/pcl_common/*.cpp")

# add_library(${common_lib} SHARED src/pcl_implementation.cpp src/pcl_implementation_ros.cpp)
ament_auto_add_library(${common_lib} SHARED ${PclCommonSrc})
ament_auto_add_library(${PclHandle} SHARED src/pcl_handle.cpp)
ament_auto_add_library(${PclIcp} SHARED src/pcl_icp.cpp)
target_include_directories(${common_lib} PUBLIC include/3rd_packages include/pcl_common)
target_link_directories(${PclHandle} PRIVATE ${common_lib})
target_link_directories(${PclIcp} PRIVATE ${common_lib})

# target_include_directories(${PclHandle} PRIVATE ${common_lib})
# target_include_directories(${PclIcp} PRIVATE ${common_lib})

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# add_executable(pcl_node src/pcl_node.cpp src/pcl_implementation.cpp)
# target_link_libraries(pcl_node ${PCL_LIBRARIES} fmt::fmt)
# ament_target_dependencies(pcl_node
# rclcpp sensor_msgs pcl_conversions
# )
rclcpp_components_register_node(${PclHandle}
  PLUGIN "PclHandle"
  EXECUTABLE ${PclHandle}_node
)
rclcpp_components_register_node(${PclIcp}
  PLUGIN "PclIcp"
  EXECUTABLE ${PclIcp}_node
)

# target_precompile_headers(${PclHandle} PRIVATE include/pch.hpp)

# 需要分开安装
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_auto_package()
