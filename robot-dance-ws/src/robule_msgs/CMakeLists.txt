cmake_minimum_required(VERSION 3.0.2)
project(robule_msgs)

find_package(catkin 
  REQUIRED COMPONENTS 
  roscpp 
  actionlib 
  actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Route.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})