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
  gams::pose::Quaternion quat(pose.as_orientation_vec());

  nav_msgs::Odometry odom;
  odom.header.stamp = global_ros::Time::now();

  containers::String frame_id (container_name + ".frame_id", *knowledge_,
    eval_settings_);
  containers::String child_frame_id (container_name + ".child_frame_id",
    *knowledge_, eval_settings_);
  odom.header.frame_id = *frame_id;
  odom.child_frame_id = *child_frame_id;

  odom.pose.pose.position.x = pose.x();
  odom.pose.pose.position.y = pose.y();
  odom.pose.pose.position.z = pose.z();
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();


  containers::NativeDoubleVector ang_twist (container_name + ".twist.angular",
    *knowledge_, 3, eval_settings_);
  odom.twist.twist.angular.x = ang_twist[0];
  odom.twist.twist.angular.y = ang_twist[1];
  odom.twist.twist.angular.z = ang_twist[2];

  containers::NativeDoubleVector lin_twist (container_name + ".twist.linear",
    *knowledge_, 3, eval_settings_);
  odom.twist.twist.linear.x = lin_twist[0];
  odom.twist.twist.linear.y = lin_twist[1];
  odom.twist.twist.linear.z = lin_twist[2];

  containers::NativeDoubleVector odom_covariance (
    container_name + ".pose.covariance", *knowledge_, 36, eval_settings_);
  containers::NativeDoubleVector twist_covariance (
    container_name + ".twist.covariance", *knowledge_, 36, eval_settings_);
  
  parse_float64_array(&odom.pose.covariance, &odom_covariance);
  parse_float64_array(&odom.twist.covariance, &twist_covariance);

  pub.publish(odom);
}

template <size_t N>
void gams::utility::ros::GamsParser::parse_float64_array (boost::array<double, N> *array,
  containers::NativeDoubleVector *origin)
{
  
  for (unsigned int i = 0; i < array->size(); ++i)
  {
    (*array)[i] = (*origin)[i];
  }
}