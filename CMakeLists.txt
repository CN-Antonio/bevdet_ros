cmake_minimum_required(VERSION 3.5)
project(bevdet_ros LANGUAGES CXX)# CUDA)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================

# set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_BUILD_TYPE "DEBUG")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-w) # only output error, ignore warining
endif()

option(CUDA_VERBOSE "Verbose output of CUDA modules" ON)

#========================
# Dependencies Setup
#========================
# find dependencies

# GCC/G++/GNU

# ROS2 #
find_package(rclcpp REQUIRED)

if(rclcpp_FOUND)
  message(=============================================================)
  message("-- ROS2 Found. ROS2 Support is turned On.")
  message(=============================================================)

  add_definitions(-DROS2_FOUND)
  include_directories(${rclcpp_INCLUDE_DIRS})
  set(CMAKE_CXX_STANDARD 14)

  ### all ros2 function package ###
  # find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_auto REQUIRED)
  # find_package(ament_cmake_ros REQUIRED)

  set(DEPENDENCIES
    rclcpp
    message_filters
    sensor_msgs
    geometry_msgs
    visualization_msgs

    cv_bridge
    pcl_conversions
    pcl_ros 
  )

  foreach (dependency IN LISTS DEPENDENCIES)
    find_package(${dependency} REQUIRED)
  endforeach()

  ament_auto_find_build_dependencies(REQUIRED
    rclcpp
    ${DEPENDENCIES}
  )
endif()

find_package(yaml-cpp REQUIRED)

# Eigen #
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Nvidia BEGIN ##############################################################################
# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  # Note: cublas_device was depreciated in CUDA version 9.2
  #       https://forums.developer.nvidia.com/t/where-can-i-find-libcublas-device-so-or-libcublas-device-a/67251/4
  #       In LibTorch, CUDA_cublas_device_LIBRARY is used.
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# TODO: find out OR Remove ##
# set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
# set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)

# set flags for TensorRT availability
## TODO: auto find TRT
if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL  "aarch64")
  set(TENSORRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu/)
  set(TENSORRT_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu/)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(TENSORRT_INCLUDE_DIRS /home/antonio/Downloads/TensorRT-8.5.3.1/include)
  set(TENSORRT_LIBRARY_DIRS /home/antonio/Downloads/TensorRT-8.5.3.1/lib)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
PATH_SUFFIXES lib lib64 bin
DOC "CUDNN library."
)
if(CUDNN_LIBRARY)
  if(CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# release
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}
#   -ccbin ${CMAKE_CXX_COMPILER}"
# )
# # set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O0 -Xcompiler -fPIC -g -w -gencode=arch=compute_61,code=sm_61")
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O0 -Xcompiler -fPIC -g -w -gencode=arch=compute_61,code=sm_61")

# debug
# if(${CMAKE_BUILD_TYPE} STREQUAL "DEBUG")
#   message("Using Debug Mode")
#   set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
# endif()

# set(CUDA_LIB_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/lib)

# Nvidia END ##############################################################################

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/bevdet

  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
  # ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)

link_directories(
	${OpenCV_LIBRARY_DIRS}
  # ${CMAKE_CUDA_INCLUDE_DIRS}
  ${TENSORRT_LIBRARY_DIRS}
)

### ?
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
### ?


#========================
# Build Setup
#========================

# Build non-CUDA bevdet dependent parts
ament_auto_add_library(bevdet_lib SHARED
  src/bevdet/bevdet.cpp
  src/bevdet/data.cpp
  src/bevdet/cpu_jpegdecoder.cpp
  src/bevdet/nvjpegdecoder.cpp

  src/test.cpp
)

# Build CUDA bevdet dependent parts
cuda_add_library(bevdet_cuda_lib SHARED
  src/bevdet/preprocess.cu 
  src/bevdet/bevpool.cu 
  src/bevdet/iou3d_nms.cu
  src/bevdet/postprocess.cu
  src/bevdet/grid_sampler.cu
)

# Build bevdet ros parts
ament_auto_add_library(bevdet_node SHARED
  src/bevdet_node.cpp
)

# cuda_add_executable(${PROJECT_NAME}_node   
# )

target_link_libraries(bevdet_node
  yaml-cpp 
  libnvinfer.so
  libjpeg.so
  ${OpenCV_LIBS}
  # ${PCL_LIBRARIES}

  bevdet_lib
  bevdet_cuda_lib
  # bevdet_node
)

rclcpp_components_register_node(bevdet_node
  PLUGIN "bevdet::bevdet_ros::ROS_Node"
  EXECUTABLE ${PROJECT_NAME}_node
)


# none ROS standalone demo #
cuda_add_executable(${PROJECT_NAME}_demo
  src/bevdet_demo.cpp

  # confirm to keep
  src/bevdet/bevdet.cpp   # 包含class bevdet

  # unsure to remove
  src/bevdet/cpu_jpegdecoder.cpp
  src/bevdet/nvjpegdecoder.cpp
  src/bevdet/preprocess.cu 
  src/bevdet/bevpool.cu 
  src/bevdet/iou3d_nms.cu
  src/bevdet/postprocess.cu
  src/bevdet/grid_sampler.cu

  # TODO:remove
  src/bevdet/data.cpp # 类DataLoader，加载数据，cpu/nvjpegdec解码，
)

target_link_libraries(${PROJECT_NAME}_demo
  yaml-cpp
  libnvinfer.so
  libjpeg.so
  ${OpenCV_LIBS}
)


# onnx->engine export tool #
# TODO: add_exe()

### ROS2 ###
if(rclcpp_FOUND)
  ament_target_dependencies(
    ${PROJECT_NAME}_node 

    rclcpp
    ${DEPENDENCIES}
  )

  install(TARGETS
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_demo

    DESTINATION lib/${PROJECT_NAME}
  )

  # install(DIRECTORY
  #   launch
  #   # rviz
  #   DESTINATION share/${PROJECT_NAME}
  # )

  ament_auto_package(INSTALL_TO_SHARE
    config
    launch
    model
  )
endif()
