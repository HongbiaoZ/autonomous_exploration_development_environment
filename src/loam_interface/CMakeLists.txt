cmake_minimum_required(VERSION 3.5)
project(loam_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(loamInterface src/loamInterface.cpp)
target_link_libraries(loamInterface rclcpp::rclcpp std_msgs::std_msgs sensor_msgs::sensor_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs message_filters::message_filters pcl_conversions::pcl_conversions)

install(TARGETS
  loamInterface
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
