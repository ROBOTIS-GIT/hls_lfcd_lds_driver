################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(hls_lfcd_lds_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED system)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(hlds_laser_publisher src/hlds_laser_publisher.cpp)
target_link_libraries(hlds_laser_publisher
  ${Boost_LIBRARIES}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)


################################################################################
# Install
################################################################################
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS hlds_laser_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
)
ament_export_include_directories(include)
ament_package()
