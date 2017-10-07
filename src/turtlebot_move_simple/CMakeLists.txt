cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_move_simple)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package()

 include_directories( ${catkin_INCLUDE_DIRS})

 add_executable(turtlebot_move_simple src/turtlebot_move_simple)

 target_link_libraries(turtlebot_move_simple ${catkin_LIBRARIES} )

