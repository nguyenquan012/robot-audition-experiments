cmake_minimum_required(VERSION 2.8.3)
project(prepath_follow)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package()

 include_directories( ${catkin_INCLUDE_DIRS})

 add_executable(prepath_follow src/prepath_follow.cpp)

 target_link_libraries(prepath_follow ${catkin_LIBRARIES} )

