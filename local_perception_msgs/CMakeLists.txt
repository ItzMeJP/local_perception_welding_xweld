cmake_minimum_required(VERSION 3.0.2)
project(local_perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
  geometry_msgs
)

#Custom Msgs
add_message_files(
        FILES
        WeldingArr.msg
        PointArr.msg
        QualityMsg.msg
        QualityArr.msg
)

add_action_files(
  FILES
  LocalPerception.action
  LocalQualityPerception.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
