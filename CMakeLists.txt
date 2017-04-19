################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(hls_lfcd_lds_driver)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES hls_lfcd_lds_driver
   CATKIN_DEPENDS roscpp sensor_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(hlds_laser_publisher	src/hlds_laser_publisher.cpp)
add_dependencies(hlds_laser_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hlds_laser_publisher ${catkin_LIBRARIES})

add_executable(hlds_laser_segment_publisher	src/hlds_laser_segment_publisher.cpp)
add_dependencies(hlds_laser_segment_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hlds_laser_segment_publisher ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS hlds_laser_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS hlds_laser_segment_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

################################################################################
# Test
################################################################################
