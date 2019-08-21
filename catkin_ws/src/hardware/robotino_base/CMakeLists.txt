cmake_minimum_required(VERSION 2.8.3)
project(robotino_base)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  nav_msgs
  robotino_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

set(CMAKE_MODULE_PATH /usr/local/robotino/cmake)
find_package(RobotinoAPI2)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotino_base
CATKIN_DEPENDS image_transport nav_msgs robotino_msgs tf
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

list(APPEND CMAKE_MODULE_PATH "/usr/local/robotino/api2/cmake") #this is where your robotino API2 is installed
find_package(RobotinoAPI2 REQUIRED)

add_executable(
  robotino_node
  src/robotino_node.cpp
  src/AnalogInputArrayROS.cpp
  src/BumperROS.cpp
  src/ComROS.cpp
  src/DigitalInputArrayROS.cpp
  src/DigitalOutputArrayROS.cpp
  src/DistanceSensorArrayROS.cpp
  src/ElectricalGripperROS.cpp
  src/EncoderInputROS.cpp
  src/MotorArrayROS.cpp
  src/NorthStarROS.cpp
  src/OmniDriveROS.cpp
  src/PowerManagementROS.cpp
  src/RobotinoNode.cpp
  )

add_dependencies(robotino_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  robotino_node
  ${REC_ROBOTINO_API2_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(
  robotino_odometry_node
  src/robotino_odometry_node.cpp
  src/ComROS.cpp
  src/OdometryROS.cpp
  src/RobotinoOdometryNode.cpp
)

target_link_libraries(
  robotino_odometry_node
  ${REC_ROBOTINO_API2_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(
  robotino_laserrangefinder_node
  src/robotino_laserrangefinder_node.cpp
  src/ComROS.cpp
  src/LaserRangeFinderROS.cpp
  src/RobotinoLaserRangeFinderNode.cpp
)

target_link_libraries(
  robotino_laserrangefinder_node
  ${REC_ROBOTINO_API2_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(
  robotino_camera_node
  src/robotino_camera_node.cpp
  src/ComROS.cpp
  src/CameraROS.cpp
  src/RobotinoCameraNode.cpp
)

target_link_libraries(
  robotino_camera_node
  ${REC_ROBOTINO_API2_LIBRARY}
  ${catkin_LIBRARIES}
)

add_dependencies(robotino_node robotino_msgs_generate_messages_cpp)
add_dependencies(robotino_odometry_node robotino_msgs_generate_messages_cpp)
