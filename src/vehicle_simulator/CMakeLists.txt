cmake_minimum_required(VERSION 3.5)
project(vehicle_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include
  /usr/include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(jsoncpp REQUIRED)

add_executable(vehicleSimulator src/vehicleSimulator.cpp)
target_link_libraries(vehicleSimulator rclcpp::rclcpp std_msgs::std_msgs sensor_msgs::sensor_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs message_filters::message_filters pcl_conversions::pcl_conversions)
target_link_libraries(vehicleSimulator ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${GZ-MSGS_LIBRARIES} ${GZ-TRANSPORT_LIBRARIES})

install(TARGETS
  vehicleSimulator
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  log
  mesh
  rviz
  urdf
  world
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
