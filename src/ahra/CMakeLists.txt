cmake_minimum_required(VERSION 3.16)

# # CUDA Toolkit 경로를 명시적으로 설정
# set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-11.8")
# find_package(CUDA REQUIRED)

# # CUDA 아키텍처 설정
# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_86,code=sm_86 -Wno-deprecated-gpu-targets -Wno-deprecated-declarations -w")


# #jetson CUDA Toolkit 경로
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-11.4")
find_package(CUDA REQUIRED)

# Jetson 아키텍처
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_87,code=sm_87 -Wno-deprecated-gpu-targets -Wno-deprecated-declarations -w")



project(ahra LANGUAGES CXX CUDA)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamixel_sdk
  dynamixel_workbench_msgs
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CUDA REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_service_files(
  FILES
  Select_Motion.srv
  Turn_Angle.srv
  UD_NeckAngle.srv
  RL_NeckAngle.srv
  Emergency.srv
  SendMotion.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ahra
  CATKIN_DEPENDS
    cmake_modules 
    dynamixel_sdk 
    dynamixel_workbench_msgs 
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
  DEPENDS 
    Boost
    OpenCV
)

################################################################################
# Build
################################################################################

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${dynamixel_workbench_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ./include
)

# Main node
add_executable(
  Main_node
  src/main.cpp
  src/dynamixel.cpp include/dynamixel.hpp
  src/callback.cpp include/callback.hpp
  src/dynamixel_controller.cpp include/dynamixel_controller.hpp
  src/Walkingpattern_generator.cc include/Walkingpattern_generator.hpp
)
add_dependencies(Main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Main_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})

# img_proc_lib library
add_library(img_proc_lib
  src/img_proc.cu
)
set_target_properties(img_proc_lib PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)
target_link_libraries(img_proc_lib
  ${OpenCV_LIBRARIES}
  ${realsense2_LIBRARY}
  ${CUDA_LIBRARIES}
)

# Move_decision node
add_executable(
  Move_decision_node
  src/Move_decision_node.cpp
  src/Move_decision2.cpp
)
add_dependencies(Move_decision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Move_decision_node
  img_proc_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${realsense2_LIBRARY}
  yaml-cpp
  ${CUDA_LIBRARIES}
)

# Sensor node
add_executable(
  Sensor_node
  src/sensor.cpp include/sensor.hpp
)
add_dependencies(Sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Sensor_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY} yaml-cpp)

install(TARGETS Main_node Move_decision_node Sensor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)