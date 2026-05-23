#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZ>());

double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

bool newTransformToMap = false;

nav_msgs::msg::Odometry odometryIn;
shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pubOdometryPointer;
tf2::Stamped<tf2::Transform> transformToMap;
geometry_msgs::msg::TransformStamped transformTfGeom ; 

unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPointer;
shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubLaserCloud;

void laserCloudAndOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odometry,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
  laserCloudIn->clear();
  laserCLoudInSensorFrame->clear();

  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  odometryIn = *odometry;

  transformToMap.setOrigin(
      tf2::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf2::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  int laserCloudInNum = laserCloudIn->points.size();

  pcl::PointXYZ p1;
  tf2::Vector3 vec;

  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);

    vec = transformToMap.inverse() * vec;

    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();

    laserCLoudInSensorFrame->points.push_back(p1);
  }

  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "map";
  odometryIn.child_frame_id = "sensor_at_scan";
  pubOdometryPointer->publish(odometryIn);

  transformToMap.frame_id_ = "map";
  transformTfGeom = tf2::toMsg(transformToMap);
  transformTfGeom.header.stamp = laserCloud2->header.stamp;
  transformTfGeom.child_frame_id = "sensor_at_scan";
  tfBroadcasterPointer->sendTransform(transformTfGeom);

  sensor_msgs::msg::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "sensor_at_scan";
  pubLaserCloud->publish(scan_data);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("sensor_scan");

  // ROS message filters
  message_filters::Subscriber<nav_msgs::msg::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subLaserCloud;

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  std::shared_ptr<Sync> sync_;
  
  subOdometry.subscribe(nh, "/state_estimation", 5);
  subLaserCloud.subscribe(nh, "/registered_scan", 5);
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(std::bind(laserCloudAndOdometryHandler, placeholders::_1, placeholders::_2));
  pubOdometryPointer = nh->create_publisher<nav_msgs::msg::Odometry>("/state_estimation_at_scan", 5);

  tfBroadcasterPointer = std::make_unique<tf2_ros::TransformBroadcaster>(*nh);

  pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor_scan", 2);

  rclcpp::spin(nh);

  return 0;
}
