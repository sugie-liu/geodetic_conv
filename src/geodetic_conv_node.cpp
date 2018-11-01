#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "geodetic_conv.hpp"

typedef double float64_t;
typedef float float32_t;

ros::Publisher posePublisher;
ros::Publisher goalPublisher;

ros::Subscriber poseSubscriber;
ros::Subscriber goalSubscriber;

geodetic_converter::GeodeticConverter g_geodetic_converter;

float64_t lat_ref = 0.0;
float64_t lon_ref = 0.0;
float64_t alt_ref = 0.0;
bool is_init = false;

void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if (!is_init)
  {
    lat_ref = msg->pose.position.x;
    lon_ref = msg->pose.position.y;
    alt_ref = msg->pose.position.z;

    g_geodetic_converter.initialiseReference(lat_ref, lon_ref, alt_ref);

    ros::NodeHandle n;

    n.setParam("/lat_ref", lat_ref);
    n.setParam("/lon_ref", lon_ref);
    n.setParam("/alt_ref", alt_ref);

    is_init = true;
  }
  else
  {
    float64_t uav_lat = msg->pose.position.x;
    float64_t uav_lon = msg->pose.position.y;
    float64_t uav_alt = msg->pose.position.z;
    float64_t uav_x = 0.0; /* ENU */
    float64_t uav_y = 0.0; /* ENU */
    float64_t uav_z = 0.0; /* ENU */

    g_geodetic_converter.geodetic2Enu(uav_lat, uav_lon, uav_alt, &uav_x, &uav_y,
                                      &uav_z);

    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = "map";
    poseMsg.pose.position.x = uav_x;
    poseMsg.pose.position.y = uav_y;
    poseMsg.pose.position.z = uav_z;
    poseMsg.pose.orientation.x = msg->pose.orientation.x;
    poseMsg.pose.orientation.y = msg->pose.orientation.y;
    poseMsg.pose.orientation.z = msg->pose.orientation.z;
    poseMsg.pose.orientation.w = msg->pose.orientation.w;
    posePublisher.publish(poseMsg);
  }
}

void uavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if (is_init)
  {
    float64_t target_lat = msg->pose.position.x;
    float64_t target_lon = msg->pose.position.y;
    float64_t target_alt = msg->pose.position.z;
    float64_t target_x = 0.0; /* ENU */
    float64_t target_y = 0.0; /* ENU */
    float64_t target_z = 0.0; /* ENU */

    g_geodetic_converter.geodetic2Enu(target_lat, target_lon, target_alt,
                                      &target_x, &target_y, &target_z);

    geometry_msgs::PoseStamped goalMsg;
    goalMsg.header.stamp = ros::Time::now();
    goalMsg.header.frame_id = "map";
    goalMsg.pose.position.x = target_x;
    goalMsg.pose.position.y = target_y;
    goalMsg.pose.position.z = target_z;
    goalMsg.pose.orientation.x = msg->pose.orientation.x;
    goalMsg.pose.orientation.y = msg->pose.orientation.y;
    goalMsg.pose.orientation.z = msg->pose.orientation.z;
    goalMsg.pose.orientation.w = msg->pose.orientation.w;
    goalPublisher.publish(goalMsg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "geodetic_conv");

  ros::NodeHandle n;

  posePublisher = n.advertise<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 1000);
  goalPublisher =
      n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

  poseSubscriber = n.subscribe("/oa/uav_pose", 1000, uavPoseCallback);
  goalSubscriber = n.subscribe("/oa/uav_goal", 1000, uavGoalCallback);

  ros::spin();

  return 0;
}
