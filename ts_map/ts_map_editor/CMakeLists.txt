cmake_minimum_required(VERSION 2.8.3)
project(ts_map_editor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  OpenCV
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(ts_map_editor src/ts_map_editor.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(ts_map_editor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

