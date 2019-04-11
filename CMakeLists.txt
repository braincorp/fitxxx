cmake_minimum_required(VERSION 2.8.3)
project(fitxxx)

# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(FITXXX src/FITXXX.cpp src/lim.cpp)
target_link_libraries(FITXXX ${console_bridge_LIBRARIES})


# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(FITXXX_ros_node src/FITXXX_ros.cpp)
target_link_libraries(FITXXX_ros_node FITXXX ${catkin_LIBRARIES})


install(TARGETS FITXXX FITXXX_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
