cmake_minimum_required(VERSION 3.0.2)
project(robule)

find_package(
  catkin REQUIRED COMPONENTS 
  roscpp 
  robule_msgs
  std_msgs
  actionlib 
  actionlib_msgs  
)
find_package(Boost 
  REQUIRED COMPONENTS 
  system)

catkin_package(
  CATKIN_DEPENDS 
  robule_msgs
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(robule src/motion_server.cpp)
target_link_libraries(robule
  ${catkin_LIBRARIES}
)

add_executable(robule2 src/motion_client.cpp)
target_link_libraries(robule2
  ${catkin_LIBRARIES}
)

add_executable(music src/music_manager.cpp)
target_link_libraries(music
  ${catkin_LIBRARIES}
)

add_executable(nav_client src/nav_client.cpp)
target_link_libraries(nav_client
  ${catkin_LIBRARIES}
)