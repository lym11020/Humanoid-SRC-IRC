cmake_minimum_required(VERSION 3.0.2)
project(ahra)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

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
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED)

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
  ${realsense2_INCLUDE_DIRS}
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

# Move_decision node
add_executable(
  Move_decision_node
  src/Move_decision_node.cpp
  src/Move_decision2.cpp include/Move_decision.hpp
  src/img_proc2.cpp include/img_proc.hpp
)
add_dependencies(Move_decision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Move_decision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY} yaml-cpp)

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
