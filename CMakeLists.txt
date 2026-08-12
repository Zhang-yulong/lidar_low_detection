cmake_minimum_required(VERSION 3.0.2)
project(lidar_low_detection)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall -fopenmp ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fopenmp ")


#换种写法：方法二
option(ENABLE_PCAP "Enable pcap support" ON)

# set(3rdparty /home/zyl/3rdparty/echiev/linux-x86-64)
set(3rdparty /home/zyl/3rdparty/echiev/windows/3rdparty)

set(OpenCV_DIR ${3rdparty}/opencv/opencv_3.3/share/OpenCV)
find_package(OpenCV REQUIRED)


# set(Eigen3_DIR "/home/zyl/3rdparty/eigen-3.3.7/cmake")
# find_package(Eigen3 REQUIRED)

find_package(VTK)
find_package(PCL 1.8 COMPONENTS common visualization io surface segmentation REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


INCLUDE_DIRECTORIES(
# 现代 CMake 推荐做法：为目标精确指定头文件路径
# target_include_directories( low_detection PRIVATE

    # ${3rdparty}/comm_2.7.1/include

    ${3rdparty}/comm/comm_2.6.2/include
    ${3rdparty}/libconfig/include
    ${3rdparty}/ulog-old/include
    ${3rdparty}/eigen/include/eigen3
    ${3rdparty}/opencv/opencv_3.3/include
    # ${3rdparty}/opencv/opencv_3.3/include/opencv #不需要
    # ${3rdparty}/opencv/opencv_3.3/include/opencv2 #不需要，cpp或h文件中标准写法#include <opencv2/>

    # ${3rdparty}/vtk/vtk-6.3.0/release/include/vtk-6.3
    # ${3rdparty}/pcl/pcl-1.8.0/release/include/pcl-1.8
    ./include
    ./common
    ./ls_driver/Include
    ./driver
    # ${EIGEN3_INCLUDE_DIR}
)

# LINK_DIRECTORIES("${3rdparty}/comm_2.7.1/lib")
LINK_DIRECTORIES("${3rdparty}/comm/comm_2.6.2/lib")
LINK_DIRECTORIES("${3rdparty}/libconfig/lib")
LINK_DIRECTORIES("${3rdparty}/ulog-old/lib")
LINK_DIRECTORIES("${3rdparty}/opencv/opencv_3.3/lib")
LINK_LIBRARIES("${3rdparty}/opencv/opencv_3.3/lib/libopencv_core.so" )

# LINK_DIRECTORIES("${3rdparty}/pcl/pcl-1.8.0/release/lib")
# LINK_DIRECTORIES("${3rdparty}/vtk/vtk-6.3.0/release/lib")

AUX_SOURCE_DIRECTORY(./ls_driver/Include LS)
AUX_SOURCE_DIRECTORY(./common COMMON)    
AUX_SOURCE_DIRECTORY(./include SRC)

SET(COMM ${3rdparty}/comm/comm_2.6.2/lib/libcomm.so)

add_executable(low_detection src/main.cpp ${LS} ${SRC} ${COMMON})

#方法二
if(NOT ENABLE_PCAP)
    target_compile_definitions(low_detection PRIVATE NO_USE_PCAP)
endif()

# # 方法一============ pcap 编译开关（NO_USE_PCAP 是否不使用 pcap） ============
# # input_pcap.hpp / input_pcap_jumbo.hpp 中 pcap 相关代码被 #ifndef NO_USE_PCAP 包裹。

# # 默认 NO_USE_PCAP=ON  -> 不定义 NO_USE_PCAP，pcap 代码不会被正常编译。
# # 如需启动pcap    -> 将 NO_USE_PCAP 设为 OFF，
# #                      会全局定义 NO_USE_PCAP，pcap 相关代码将被编译。
# # 注意：pcap 库的链接请自行在 target_link_libraries 中处理。
# option(NO_USE_PCAP "不使用 pcap 支持" OFF) #正确的
# if(NO_USE_PCAP)
#     target_compile_definitions(low_detection PRIVATE NO_USE_PCAP)
# endif()

target_link_libraries(low_detection 
    
    pthread 
    config 
    ulog 
    # pcap
    ${COMM} 

    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} 
    
    # # PCL库 - 使用绝对路径
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_2d.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_common.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_features.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_filters.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_io.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_io_ply.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_kdtree.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_keypoints.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_ml.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_octree.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_outofcore.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_people.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_recognition.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_registration.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_sample_consensus.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_search.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_segmentation.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_stereo.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_surface.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_tracking.so"
    # "${3rdparty}/pcl/pcl-1.8.0/release/lib/libpcl_visualization.so"
    
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkIOPLY-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkIOImage-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkImagingCore-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkIOLegacy-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkIOGeometry-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkIOCore-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkCommonExecutionModel-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkCommonDataModel-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkCommonCore-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkCommonMisc-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkCommonSystem-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkCommonTransforms-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkCommonMath-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtksys-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkpng-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtktiff-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkmetaio-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkDICOMParser-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkzlib-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkjpeg-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkRenderingCore-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkRenderingFreeType-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkInteractionStyle-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkRenderingOpenGL-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkRenderingLOD-6.3.so"
    # "${3rdparty}/vtk/vtk-6.3.0/release/lib/libvtkFiltersSources-6.3.so"
)

# # 方法一 配套
# if(NOT NO_USE_PCAP)
#     target_link_libraries(low_detection pcap)
# endif()

#方法二
if(ENABLE_PCAP)
    target_link_libraries(low_detection pcap)
endif()