cmake_minimum_required(VERSION 3.1.0)
project(nuscenes_pcl_viz)

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# add_compile_options("-Wl,--no-undefined")
# set(CMAKE_C_FLAGS "-fPIC -Wall -D_GNU_SOURCE -rdynamic -export-dynamic -Werror=return-type")
# set(CMAKE_CXX_FLAGS "-std=c++17 -fexceptions -frtti -fPIC -Wall -D_GNU_SOURCE -rdynamic -export-dynamic -Werror=return-type")
# set(CMAKE_CXX_FLAGS_RELEASE "-O2")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ")

set(CMAKE_BUILD_PYTHON $ENV{USE_Python})
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

find_package(CUDA REQUIRED)

set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -fexceptions -frtti -fPIC -Wall -D_GNU_SOURCE -rdynamic -export-dynamic -Werror=return-type")
set(CMAKE_CXX_FLAGS_DEBUG   "-std=c++17 -fexceptions -frtti -fPIC -Wall -D_GNU_SOURCE -rdynamic -export-dynamic -Werror=return-type -O0 -g")

set(CUDA_NVCC_FLAGS_RELEASE "-std=c++17 -fexceptions -frtti -fPIC -Wall -D_GNU_SOURCE -rdynamic -export-dynamic -Werror=return-type")
set(CUDA_NVCC_FLAGS_DEBUG   "-std=c++17 -fexceptions -frtti -fPIC -Wall -D_GNU_SOURCE -rdynamic -export-dynamic -Werror=return-type -O0 -g")

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} 
    -gencode arch=compute_$ENV{CUDASM},code=compute_$ENV{CUDASM}
)
find_package(CUDA REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)
pkg_check_modules(yamlcpp REQUIRED yaml-cpp)

find_package(PCL REQUIRED COMPONENTS common filters visualization)

include_directories(
    $ENV{CUDA_Inc}
    $ENV{TensorRT_Inc}
    lidar_encoder
    centerpoint/inc
    centerpoint/common
)

# add_executable(nuscenes_pcl_viz src/nuscenes_pcl_viz.cpp)
# target_include_directories(nuscenes_pcl_viz PUBLIC ${PCL_INCLUDE_DIRS} ${jsoncpp_INCLUDE_DIRS} ${yamlcpp_INCLUDEDIR})
# target_link_libraries(nuscenes_pcl_viz PUBLIC ${PCL_LIBRARIES} ${jsoncpp_LIBRARIES} ${yamlcpp_LIBRARIES})

link_directories(
    $ENV{CUDA_Lib}
    $ENV{TensorRT_Lib}
)

set(CUDA_PLUGIN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/centerpoint/model/libcenterpoint_trt_plugin.so)

file(GLOB COMMON_SRC_FILES
    centerpoint/common/*.cpp
)

set(LIDAR_ENCODER_BIN_NAME lidar_encoder_bin)
cuda_add_executable(
    ${LIDAR_ENCODER_BIN_NAME}
    lidar_encoder/main.cc
    lidar_encoder/signal_handlers.cc
    centerpoint/src/centerpoint_wrapper.cpp
    centerpoint/src/postprocess.cpp
    centerpoint/src/preprocess.cpp
    ${COMMON_SRC_FILES}
)
target_link_libraries(
    ${LIDAR_ENCODER_BIN_NAME}
    ${PCL_LIBRARIES} 
    ${jsoncpp_LIBRARIES} 
    ${yamlcpp_LIBRARIES}
    ${CUDA_PLUGIN_PATH}
    libnvinfer.so
    -lnvonnxparser
    -lnvparsers
    -lnvinfer_plugin
    -ldl
    -lrt
    -pthread
)
