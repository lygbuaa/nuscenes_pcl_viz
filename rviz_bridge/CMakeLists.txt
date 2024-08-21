cmake_minimum_required(VERSION 3.10.0)
project(pcl_rviz_bridge)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
# set(CMAKE_CXX_FLAGS "-g3 -O0 -fPIC -rdynamic -export-dynamic")
set(CMAKE_CXX_FLAGS "-O3 -Wall -fPIC -rdynamic -export-dynamic")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters visualization)

pkg_check_modules(jsoncpp REQUIRED jsoncpp)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/inc/
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
MESSAGE(STATUS "cpu architecture: ${ARCHITECTURE}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../lidar_encoder/
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_rviz_bridge_bin src/main.cc)
ament_target_dependencies(pcl_rviz_bridge_bin 
  pcl_conversions
  rclcpp 
  std_msgs
  cv_bridge
  image_transport
  geometry_msgs
  nav_msgs
  visualization_msgs
)

target_link_libraries(pcl_rviz_bridge_bin
  ${PCL_LIBRARIES} 
  ${jsoncpp_LIBRARIES} 
)

# install binary files
install(TARGETS
  pcl_rviz_bridge_bin
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()