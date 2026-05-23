#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string waypoint_file_dir;
string boundary_file_dir;
double waypointXYRadius = 0.5;
double waypointZBound = 5.0;
double waitTime = 0;
double waitTimeStart = 0;
bool isWaiting = false;
double frameRate = 5.0;
double speed = 1.0;
bool sendSpeed = true;
bool sendBoundary = true;

pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;

rclcpp::Node::SharedPtr nh;

// reading waypoints from file function
void readWaypointFile()
{
  FILE* waypoint_file = fopen(waypoint_file_dir.c_str(), "r");
  if (waypoint_file == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(waypoint_file, "%s", str);
    if (val != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(waypoint_file, "%d", &pointNum);
      if (val != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
      }
    }
  }

  waypoints->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(waypoint_file, "%f", &point.x);
    val2 = fscanf(waypoint_file, "%f", &point.y);
    val3 = fscanf(waypoint_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    waypoints->push_back(point);
  }

  fclose(waypoint_file);
}

// reading boundary from file function
void readBoundaryFile()
{
  FILE* boundary_file = fopen(boundary_file_dir.c_str(), "r");
  if (boundary_file == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(boundary_file, "%s", str);
    if (val != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(boundary_file, "%d", &pointNum);
      if (val != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
      }
    }
  }

  boundary->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(boundary_file, "%f", &point.x);
    val2 = fscanf(boundary_file, "%f", &point.y);
    val3 = fscanf(boundary_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    boundary->push_back(point);
  }

  fclose(boundary_file);
}

// vehicle pose callback function
void poseHandler(const nav_msgs::msg::Odometry::SharedPtr pose)
{
  curTime = rclcpp::Time(pose->header.stamp).seconds(); 
  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("waypointExample");

  nh->declare_parameter<std::string>("waypoint_file_dir", waypoint_file_dir);
  nh->declare_parameter<std::string>("boundary_file_dir", boundary_file_dir);
  nh->declare_parameter<double>("waypointXYRadius", waypointXYRadius);
  nh->declare_parameter<double>("waypointZBound", waypointZBound);
  nh->declare_parameter<double>("waitTime", waitTime);
  nh->declare_parameter<double>("frameRate", frameRate);
  nh->declare_parameter<double>("speed", speed);
  nh->declare_parameter<bool>("sendSpeed", sendSpeed);
  nh->declare_parameter<bool>("sendBoundary", sendBoundary);

  nh->get_parameter("waypoint_file_dir", waypoint_file_dir);
  nh->get_parameter("boundary_file_dir", boundary_file_dir);
  nh->get_parameter("waypointXYRadius", waypointXYRadius);
  nh->get_parameter("waypointZBound", waypointZBound);
  nh->get_parameter("waitTime", waitTime);
  nh->get_parameter("frameRate", frameRate);
  nh->get_parameter("speed", speed);
  nh->get_parameter("sendSpeed", sendSpeed);
  nh->get_parameter("sendBoundary", sendBoundary);
  
  auto subPose = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, poseHandler);
  
  auto pubWaypoint = nh->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 5);
  geometry_msgs::msg::PointStamped waypointMsgs;
  waypointMsgs.header.frame_id = "map";
  
  auto pubSpeed = nh->create_publisher<std_msgs::msg::Float32>("/speed", 5);
  std_msgs::msg::Float32 speedMsgs;
  
  auto pubBoundary = nh->create_publisher<geometry_msgs::msg::PolygonStamped>("/navigation_boundary", 5);
  geometry_msgs::msg::PolygonStamped boundaryMsgs;
  boundaryMsgs.header.frame_id = "map";

  // read waypoints from file
  readWaypointFile();

  // read boundary from file
  if (sendBoundary) {
    readBoundaryFile();

    int boundarySize = boundary->points.size();
    boundaryMsgs.polygon.points.resize(boundarySize);
    for (int i = 0; i < boundarySize; i++) {
      boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
      boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
      boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
    }
  }

  int wayPointID = 0;
  int waypointSize = waypoints->points.size();

  if (waypointSize == 0) {
    RCLCPP_INFO(nh->get_logger(), "No waypoint available, exit.");
    exit(1);
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    float disX = vehicleX - waypoints->points[wayPointID].x;
    float disY = vehicleY - waypoints->points[wayPointID].y;
    float disZ = vehicleZ - waypoints->points[wayPointID].z;

    // start waiting if the current waypoint is reached
    if (sqrt(disX * disX + disY * disY) < waypointXYRadius && fabs(disZ) < waypointZBound && !isWaiting) {
      waitTimeStart = curTime;
      isWaiting = true;
    }

    // move to the next waypoint after waiting is over
    if (isWaiting && waitTimeStart + waitTime < curTime && wayPointID < waypointSize - 1) {
      wayPointID++;
      isWaiting = false;
    }

    // publish waypoint, speed, and boundary messages at certain frame rate
    if (curTime - waypointTime > 1.0 / frameRate) {
      if (!isWaiting) {
        waypointMsgs.header.stamp = rclcpp::Time(static_cast<uint64_t>(curTime * 1e9));
        waypointMsgs.point.x = waypoints->points[wayPointID].x;
        waypointMsgs.point.y = waypoints->points[wayPointID].y;
        waypointMsgs.point.z = waypoints->points[wayPointID].z;
        pubWaypoint->publish(waypointMsgs);
      }

      if (sendSpeed) {
        speedMsgs.data = speed;
        pubSpeed->publish(speedMsgs);
      }

      if (sendBoundary) {
        boundaryMsgs.header.stamp = rclcpp::Time(static_cast<uint64_t>(curTime * 1e9));
        pubBoundary->publish(boundaryMsgs);
      }

      waypointTime = curTime;
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
