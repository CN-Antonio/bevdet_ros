cmake_minimum_required(VERSION 3.5)
SET(PROJECT_NAME bevdet_ros)
project(${PROJECT_NAME} LANGUAGES CXX CUDA)

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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#========================
# Dependencies Setup
#========================
# find dependencies
find_package(ament_cmake REQUIRED)

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

# Nvidia #
find_package(CUDA REQUIRED)

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)

# release
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}
  -ccbin ${CMAKE_CXX_COMPILER}"
)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O0 -Xcompiler -fPIC -g -w -gencode=arch=compute_61,code=sm_61")

# debug
if(${CMAKE_BUILD_TYPE} STREQUAL "DEBUG")
  message("Using Debug Mode")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
endif()

set(CUDA_LIB_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/lib)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  # ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  # ${TENSORRT_INCLUDE_DIRS}
  
  ${PROJECT_SOURCE_DIR}/include
)

link_directories(
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

cuda_add_executable(${PROJECT_NAME}_node
  src/bevdet_node.cpp

  src/bevdet/bevdet.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  yaml-cpp  
  # libnvinfer.so
)

# none ROS demo
# cuda_add_executable(${PROJECT_NAME}_demo
#   src/bevdet_demo.cpp

#   # src/flashocc/flashocc.cpp
# )

# target_link_libraries(${PROJECT_NAME}_demo
#   # libnvinfer.so
# )

### ROS2 ###
if(rclcpp_FOUND)
  ament_target_dependencies(${PROJECT_NAME}_node 
    rclcpp
    ${DEPENDENCIES}
  )

  install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    launch
    # rviz
    DESTINATION share/${PROJECT_NAME}
  )

  ament_auto_package()
endif()
