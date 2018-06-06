/**
 * @file GamsParser.cpp
 * @author Jakob Auer <jakob.auer@gmail.com>
 *
 * This file contains parsing functionality for ROS topics
 **/
#include "GamsParser.h"


gams::utility::ros::GamsParser::GamsParser (knowledge::KnowledgeBase * kb) : 
  eval_settings_(true, true, false, false, false)
{
  knowledge_ = kb;
}


void gams::utility::ros::GamsParser::parse_message (std::string container_name,
  std::string topic_name, std::string topic_type)
{
  if (topic_type == "nav_msgs/Odometry")
  {
    parse_odometry(container_name, topic_name);
  }
}

void gams::utility::ros::GamsParser::parse_odometry(std::string container_name,
  std::string topic_name)
{
  global_ros::Publisher pub = node_.advertise<nav_msgs::Odometry> (topic_name, 100);

  containers::NativeDoubleVector cont (container_name + ".pose", *knowledge_, 6,
    eval_settings_);
  gams::pose::Pose pose;
  pose.from_container(cont);

  nav_msgs::Odometry odom;
  odom.header.stamp = global_ros::Time::now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x =177.0;
  odom.pose.pose.position.y = 2.0;
  odom.pose.pose.position.z = 3.0;
  odom.pose.pose.orientation.x = 0.707;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.707;

  odom.twist.twist.linear.x = 0.5;
  odom.twist.twist.linear.y = 0.6;

  pub.publish(odom);
}