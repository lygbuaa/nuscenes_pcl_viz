cmake_minimum_required(VERSION 3.1.0)
project(nuscenes_pcl_viz)

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

add_compile_options("-Wl,--no-undefined")
set(CMAKE_C_FLAGS "-fPIC -Wall -D_GNU_SOURCE -rdynamic -export-dynamic -Werror=return-type")
set(CMAKE_CXX_FLAGS "-std=c++17 -fexceptions -frtti -fPIC -Wall -D_GNU_SOURCE -rdynamic -export-dynamic -Werror=return-type")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ")

find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)
pkg_check_modules(yamlcpp REQUIRED yaml-cpp)

find_package(PCL REQUIRED COMPONENTS common filters visualization)

include_directories(
    include
)

add_executable(nuscenes_pcl_viz src/nuscenes_pcl_viz.cpp)
target_include_directories(nuscenes_pcl_viz PUBLIC ${PCL_INCLUDE_DIRS} ${jsoncpp_INCLUDE_DIRS} ${yamlcpp_INCLUDEDIR})
target_link_libraries(nuscenes_pcl_viz PUBLIC ${PCL_LIBRARIES} ${jsoncpp_LIBRARIES} ${yamlcpp_LIBRARIES})

set(LIDAR_ENCODER_BIN_NAME lidar_encoder_bin)
add_executable(
    ${LIDAR_ENCODER_BIN_NAME}
    lidar_encoder/main.cc
    lidar_encoder/signal_handlers.cc
)
target_link_libraries(
    ${LIDAR_ENCODER_BIN_NAME}
    ${PCL_LIBRARIES} 
    ${jsoncpp_LIBRARIES} 
    ${yamlcpp_LIBRARIES}
    -ldl
    -lrt
    -pthread
)
