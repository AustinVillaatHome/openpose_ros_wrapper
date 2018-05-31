cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_wrapper_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  BodyPartDetection.msg
  BodyPartDetection3d.msg
  EyeDetection.msg
  PersonDetection.msg
  PersonDetection3d.msg
  Persons.msg
  PersonsArray.msg
  Persons3d.msg
)

add_service_files(
  FILES
  GetPersons.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation message_runtime geometry_msgs sensor_msgs
)

