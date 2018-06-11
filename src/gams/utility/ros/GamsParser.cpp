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
    parse_odometry (container_name, topic_name);
  }
  else if (topic_type == "sensor_msgs/Imu")
  {
    parse_imu (container_name, topic_name);
  }
  else if (topic_type == "sensor_msgs/LaserScan")
  {
    parse_laserscan (container_name, topic_name);
  }
}

void gams::utility::ros::GamsParser::parse_odometry(std::string container_name,
  std::string topic_name)
{
  global_ros::Publisher pub = node_.advertise<nav_msgs::Odometry> (topic_name, 100);

  containers::NativeDoubleVector cont (container_name + ".pose", *knowledge_, 6,
    eval_settings_);
  gams::pose::Pose pose;
  pose.from_container (cont);
  gams::pose::Quaternion quat (pose.as_orientation_vec());

  nav_msgs::Odometry odom;
  odom.header.stamp = global_ros::Time::now ();

  containers::String frame_id (container_name + ".frame_id", *knowledge_,
    eval_settings_);
  containers::String child_frame_id (container_name + ".child_frame_id",
    *knowledge_, eval_settings_);
  odom.header.frame_id = *frame_id;
  odom.child_frame_id = *child_frame_id;

  odom.pose.pose.position.x = pose.x ();
  odom.pose.pose.position.y = pose.y ();
  odom.pose.pose.position.z = pose.z ();
  odom.pose.pose.orientation.x = quat.x ();
  odom.pose.pose.orientation.y = quat.y ();
  odom.pose.pose.orientation.z = quat.z ();
  odom.pose.pose.orientation.w = quat.w ();


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
void gams::utility::ros::GamsParser::parse_float64_array (
  boost::array<double, N> *array,
  containers::NativeDoubleVector *origin)
{
  for (unsigned int i = 0; i < array->size(); ++i)
  {
    (*array)[i] = (*origin)[i];
  }
}

void gams::utility::ros::GamsParser::parse_float64_array (
  std::vector<float> *array,
  containers::NativeDoubleVector *origin)
{
  for (unsigned int i = 0; i < array->size(); ++i)
  {
    array->push_back ((*origin)[i]);
  }
}

void gams::utility::ros::GamsParser::parse_vector3 (geometry_msgs::Vector3 *vec,
  containers::NativeDoubleVector *origin)
{
  vec->x = (*origin)[0];
  vec->y = (*origin)[1];
  vec->z = (*origin)[2];
}

void gams::utility::ros::GamsParser::parse_quaternion (
  geometry_msgs::Quaternion *quat,
  containers::NativeDoubleVector *origin)
{
  gams::pose::Quaternion q((*origin)[0], (*origin)[1], (*origin)[2]);
  quat->x = q.x ();
  quat->y = q.y ();
  quat->z = q.z ();
  quat->w = q.w ();
}

void gams::utility::ros::GamsParser::parse_imu(std::string container_name,
  std::string topic_name)
{
  global_ros::Publisher pub = node_.advertise<sensor_msgs::Imu> (topic_name, 100);
  sensor_msgs::Imu imu;
  imu.header.stamp = global_ros::Time::now ();

  containers::String frame_id (container_name + ".frame_id", *knowledge_,
    eval_settings_);
  imu.header.frame_id = *frame_id;

  containers::NativeDoubleVector orientation (
    container_name + ".orientation", *knowledge_, -1, eval_settings_); 
  containers::NativeDoubleVector orientation_covar (
    container_name + ".orientation_covariance", *knowledge_, -1, eval_settings_);
  containers::NativeDoubleVector ang_vel (
    container_name + ".angular_velocity", *knowledge_, -1, eval_settings_); 
  containers::NativeDoubleVector ang_vel_covar (
    container_name + ".angular_velocity_covariance", *knowledge_, -1, eval_settings_);
  containers::NativeDoubleVector lin_acc (
    container_name + ".linear_acceleration", *knowledge_, -1, eval_settings_); 
  containers::NativeDoubleVector lin_acc_covar (
    container_name + ".linear_acceleration_covariance", *knowledge_, -1, eval_settings_);
  parse_quaternion(&imu.orientation, &orientation);
  parse_float64_array(&imu.orientation_covariance, &orientation_covar);
  parse_vector3(&imu.angular_velocity, &ang_vel);
  parse_float64_array(&imu.angular_velocity_covariance, &ang_vel_covar);
  parse_vector3(&imu.linear_acceleration, &lin_acc);
  parse_float64_array(&imu.linear_acceleration_covariance, &lin_acc_covar);

  pub.publish(imu);
}

void gams::utility::ros::GamsParser::parse_laserscan (std::string container_name,
  std::string topic_name)
{
  global_ros::Publisher pub = node_.advertise<sensor_msgs::LaserScan> (topic_name, 100);
  sensor_msgs::LaserScan scan;
  scan.header.stamp = global_ros::Time::now ();
  //Ranges
  containers::NativeDoubleVector ranges (container_name + ".ranges",
    *knowledge_, -1);
  parse_float64_array (&scan.ranges, &ranges);
  //Intensities
  containers::NativeDoubleVector intensities (container_name + ".intensities",
    *knowledge_, -1, eval_settings_);
  parse_float64_array (&scan.intensities, &intensities);
  //Parameters
  containers::Double angle_min (container_name + ".angle_min", *knowledge_,
    eval_settings_);
  scan.angle_min = *angle_min;
  containers::Double angle_max (container_name + ".angle_max", *knowledge_,
    eval_settings_);
  scan.angle_max = *angle_max;
  containers::Double angle_increment (
    container_name + ".angle_increment", *knowledge_, eval_settings_);
  scan.angle_increment = *angle_increment;
  containers::Double time_increment (container_name + ".time_increment",
    *knowledge_, eval_settings_);
   scan.time_increment = *time_increment;
  containers::Double scan_time (container_name + ".scan_time", *knowledge_,
    eval_settings_);
  scan.scan_time = *scan_time;
  containers::Double range_min (container_name + ".range_min", *knowledge_,
    eval_settings_);
  scan.range_min = *range_min;
  containers::Double range_max (container_name + ".range_max", *knowledge_,
    eval_settings_);
   scan.range_max = *range_max;
}