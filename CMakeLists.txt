cmake_minimum_required(VERSION 3.8)
project(MirLocalize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # add -O3 when you really have to run
endif()

add_definitions(-DINFO_DUMP)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(sick_interfaces REQUIRED)


find_package(Eigen3 REQUIRED)

include_directories(
  kalman/include
  /usr/include/eigen3/
)

# Add include directories

# add_library(data_publisher SHARED src/main.cpp)

add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  sick_interfaces
  nav_msgs sensor_msgs
  tf2_ros
  rclcpp
  std_msgs
  geometry_msgs
  rclcpp_components
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

