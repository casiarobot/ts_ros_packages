cmake_minimum_required(VERSION 2.8.3)
project(ts_map_saver)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ts_map_saver
#  CATKIN_DEPENDS nav_msgs geomtry_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
 add_executable(ts_map_saver src/ts_map_saver.cpp)

# Specify libraries to link a library or executable target against
 target_link_libraries(ts_map_saver
   ${catkin_LIBRARIES}
 )
