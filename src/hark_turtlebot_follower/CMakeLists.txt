cmake_minimum_required(VERSION 2.8.3)
project(hark_turtlebot_follower)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
## Declare a catkin package
catkin_package()


#include_directories (${hark_INCLUDE_DIRS} ${ros_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(hark_turtlebot_follower src/hark_turtlebot_follower.cpp)
target_link_libraries(hark_turtlebot_follower ${catkin_LIBRARIES})
#add_dependencies(hark_turtlebot_follower ${catkin_EXPORTED_TARGETS})
