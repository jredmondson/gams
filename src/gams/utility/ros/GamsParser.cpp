/**
 * @file GamsParser.cpp
 * @author Jakob Auer <jakob.auer@gmail.com>
 *
 * This file contains parsing functionality for ROS topics
 **/
#include "GamsParser.h"


gams::utility::ros::GamsParser::GamsParser(knowledge::KnowledgeBase * kb) : 
  eval_settings_(true, true, false, false, false)
{
  knowledge_ = kb;
}


void gams::utility::ros::GamsParser::parse_message(std::string container_name,
  std::string topic_name, std::string topic_type)
{
  if (topic_type == "nav_msgs/Odometry")
  {
    publish_odometry(container_name, topic_name);
  }
  else if (topic_type == "sensor_msgs/Imu")
  {
    publish_imu(container_name, topic_name);
  }
  else if (topic_type == "sensor_msgs/LaserScan")
  {
    publish_laserscan(container_name, topic_name);
  }
  else if (topic_type == "sensor_msgs/PointCloud2")
  {
    publish_laserscan(container_name, topic_name);
  }
  else if (topic_type == "sensor_msgs/CompressedImage")
  {
    publish_compressed_image(container_name, topic_name);
  }
  else if (topic_type == "sensor_msgs/Range")
  {
    publish_range(container_name, topic_name);
  }
  else if (topic_type == "sensor_msgs/FluidPressure")
  {
    publish_fluidpressure(container_name, topic_name);
  }
  else if (topic_type == "geometry_msgs/Pose")
  {
    global_ros::Publisher pub =
      node_.advertise<geometry_msgs::Pose>(topic_name, 100);
    geometry_msgs::Pose p;
    containers::NativeDoubleVector cont(container_name, *knowledge_, -1,
      eval_settings_);
    parse_pose(&p, &cont);
    pub.publish(p);
  }
  else if (topic_type == "geometry_msgs/PoseStamped")
  {
    global_ros::Publisher pub =
      node_.advertise<geometry_msgs::PoseStamped>(topic_name, 100);
    geometry_msgs::PoseStamped p;
    containers::NativeDoubleVector cont(container_name, *knowledge_, -1,
      eval_settings_);
    parse_pose(&p.pose, &cont);
    p.header.stamp = global_ros::Time::now();
    pub.publish(p);
  }
}

void gams::utility::ros::GamsParser::publish_odometry(std::string container_name,
  std::string topic_name)
{
  global_ros::Publisher pub =
    node_.advertise<nav_msgs::Odometry>(topic_name, 100);

  containers::NativeDoubleVector cont(container_name + ".pose", *knowledge_, 6,
    eval_settings_);

  nav_msgs::Odometry odom;
  odom.header.stamp = global_ros::Time::now();

  containers::String frame_id(container_name + ".frame_id", *knowledge_,
    eval_settings_);
  containers::String child_frame_id(container_name + ".child_frame_id",
    *knowledge_, eval_settings_);
  odom.header.frame_id = *frame_id;
  odom.child_frame_id = *child_frame_id;


  containers::NativeDoubleVector ang_twist(container_name + ".twist.angular",
    *knowledge_, 3, eval_settings_);
  odom.twist.twist.angular.x = ang_twist[0];
  odom.twist.twist.angular.y = ang_twist[1];
  odom.twist.twist.angular.z = ang_twist[2];

  containers::NativeDoubleVector lin_twist(container_name + ".twist.linear",
    *knowledge_, 3, eval_settings_);
  odom.twist.twist.linear.x = lin_twist[0];
  odom.twist.twist.linear.y = lin_twist[1];
  odom.twist.twist.linear.z = lin_twist[2];

  containers::NativeDoubleVector odom_covariance(
    container_name + ".pose.covariance", *knowledge_, 36, eval_settings_);
  containers::NativeDoubleVector twist_covariance(
    container_name + ".twist.covariance", *knowledge_, 36, eval_settings_);
  
  parse_float64_array(&odom.pose.covariance, &odom_covariance);
  parse_float64_array(&odom.twist.covariance, &twist_covariance);

  pub.publish(odom);
}


void gams::utility::ros::GamsParser::publish_imu(std::string container_name,
  std::string topic_name)
{
  global_ros::Publisher pub = node_.advertise<sensor_msgs::Imu>(topic_name, 100);
  sensor_msgs::Imu imu;
  imu.header.stamp = global_ros::Time::now();

  containers::String frame_id(container_name + ".frame_id", *knowledge_,
    eval_settings_);
  imu.header.frame_id = *frame_id;

  containers::NativeDoubleVector orientation(
    container_name + ".orientation", *knowledge_, -1, eval_settings_); 
  containers::NativeDoubleVector orientation_covar(
    container_name + ".orientation_covariance", *knowledge_, -1, eval_settings_);
  containers::NativeDoubleVector ang_vel(
    container_name + ".angular_velocity", *knowledge_, -1, eval_settings_); 
  containers::NativeDoubleVector ang_vel_covar(
    container_name + ".angular_velocity_covariance", *knowledge_, -1,
    eval_settings_);
  containers::NativeDoubleVector lin_acc(
    container_name + ".linear_acceleration", *knowledge_, -1, eval_settings_); 
  containers::NativeDoubleVector lin_acc_covar(
    container_name + ".linear_acceleration_covariance", *knowledge_, -1,
    eval_settings_);
  parse_quaternion(&imu.orientation, &orientation);
  parse_float64_array(&imu.orientation_covariance, &orientation_covar);
  parse_vector3(&imu.angular_velocity, &ang_vel);
  parse_float64_array(&imu.angular_velocity_covariance, &ang_vel_covar);
  parse_vector3(&imu.linear_acceleration, &lin_acc);
  parse_float64_array(&imu.linear_acceleration_covariance, &lin_acc_covar);

  pub.publish(imu);
}

void gams::utility::ros::GamsParser::publish_laserscan(
  std::string container_name, std::string topic_name)
{
  global_ros::Publisher pub =
    node_.advertise<sensor_msgs::LaserScan>(topic_name, 100);
  sensor_msgs::LaserScan scan;
  scan.header.stamp = global_ros::Time::now();
  //Ranges
  containers::NativeDoubleVector ranges(container_name + ".ranges",
    *knowledge_, -1);
  parse_float64_array(&scan.ranges, &ranges);
  //Intensities
  containers::NativeDoubleVector intensities(container_name + ".intensities",
    *knowledge_, -1, eval_settings_);
  parse_float64_array(&scan.intensities, &intensities);
  //Parameters
  containers::Double angle_min(container_name + ".angle_min", *knowledge_,
    eval_settings_);
  scan.angle_min = *angle_min;
  containers::Double angle_max(container_name + ".angle_max", *knowledge_,
    eval_settings_);
  scan.angle_max = *angle_max;
  containers::Double angle_increment(
    container_name + ".angle_increment", *knowledge_, eval_settings_);
  scan.angle_increment = *angle_increment;
  containers::Double time_increment(container_name + ".time_increment",
    *knowledge_, eval_settings_);
   scan.time_increment = *time_increment;
  containers::Double scan_time(container_name + ".scan_time", *knowledge_,
    eval_settings_);
  scan.scan_time = *scan_time;
  containers::Double range_min(container_name + ".range_min", *knowledge_,
    eval_settings_);
  scan.range_min = *range_min;
  containers::Double range_max(container_name + ".range_max", *knowledge_,
    eval_settings_);
   scan.range_max = *range_max;
  
  pub.publish(scan);
}

void gams::utility::ros::GamsParser::publish_pointcloud2(
  std::string container_name, std::string topic_name)
{
  global_ros::Publisher pub =
    node_.advertise<sensor_msgs::PointCloud2>(topic_name, 100);
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = global_ros::Time::now();
  containers::Integer height(container_name + ".height", *knowledge_,
    eval_settings_);
  cloud.height = *height;
  containers::Integer width(container_name + ".width", *knowledge_,
    eval_settings_);
  cloud.width = *width;
  containers::Integer point_step(container_name + ".point_step", *knowledge_,
    eval_settings_);
  cloud.point_step = *point_step;
  containers::Integer row_step(container_name + ".row_step", *knowledge_,
    eval_settings_);
  cloud.row_step = *row_step;

  containers::NativeIntegerVector data(container_name + ".data",
    *knowledge_, -1, eval_settings_);
  parse_int_array(&cloud.data, &data);

  containers::Integer is_bigendian(container_name + ".is_bigendian",
    *knowledge_, eval_settings_);
  cloud.is_bigendian = *is_bigendian;
  containers::Integer is_dense(container_name + ".is_dense", *knowledge_,
    eval_settings_);
  cloud.is_dense = *is_dense;
}

void gams::utility::ros::GamsParser::publish_compressed_image(
  std::string container_name, std::string topic_name)
{
  global_ros::Publisher pub =
    node_.advertise<sensor_msgs::CompressedImage>(topic_name, 100);

  sensor_msgs::CompressedImage img;
  img.header.stamp = global_ros::Time::now();
  containers::String format(container_name + ".format", *knowledge_,
    eval_settings_);
  img.format = *format;

  containers::NativeIntegerVector data(container_name + ".data",
    *knowledge_, -1, eval_settings_);
  parse_int_array(&img.data, &data);

  pub.publish(img);
}

void gams::utility::ros::GamsParser::publish_range( std::string container_name,
  std::string topic_name)
{
  global_ros::Publisher pub =
    node_.advertise<sensor_msgs::CompressedImage>(topic_name, 100);
  sensor_msgs::Range range;
  range.header.stamp = global_ros::Time::now();

  containers::Double field_of_view(
    container_name + ".field_of_view", *knowledge_, eval_settings_);
  range.field_of_view = *field_of_view;
  containers::Double min_range(container_name + ".min_range", *knowledge_,
    eval_settings_);
  range.min_range = *min_range;
  containers::Double max_range(container_name + ".max_range", *knowledge_,
    eval_settings_);
  range.max_range = *max_range;
  containers::Double range_val(container_name + ".range", *knowledge_,
    eval_settings_);
  range.range = *range_val;
  containers::Integer radiation_type(container_name + ".radiation_type",
    *knowledge_, eval_settings_);
  range.radiation_type = *radiation_type;

  pub.publish(range);
}

void gams::utility::ros::GamsParser::publish_fluidpressure(
  std::string container_name, std::string topic_name)
{
  global_ros::Publisher pub =
    node_.advertise<sensor_msgs::FluidPressure>(topic_name, 100);
  sensor_msgs::FluidPressure press;
  press.header.stamp = global_ros::Time::now();

  containers::Double fluid_pressure(
    container_name + ".fluid_pressure", *knowledge_, eval_settings_);
   press.fluid_pressure = *fluid_pressure;
  containers::Double variance(container_name + ".variance", *knowledge_,
    eval_settings_);
  press.variance = *variance;

  pub.publish(press);
}

template <size_t N>
void gams::utility::ros::GamsParser::parse_float64_array(
  boost::array<double, N> *array,
  containers::NativeDoubleVector *origin)
{
  for(unsigned int i = 0; i < array->size(); ++i)
  {
   (*array)[i] =(*origin)[i];
  }
}

void gams::utility::ros::GamsParser::parse_float64_array(
  std::vector<float> *array,
  containers::NativeDoubleVector *origin)
{
  for(unsigned int i = 0; i < array->size(); ++i)
  {
    array->push_back((*origin)[i]);
  }
}

void gams::utility::ros::GamsParser::parse_vector3(geometry_msgs::Vector3 *vec,
  containers::NativeDoubleVector *origin)
{
  vec->x =(*origin)[0];
  vec->y =(*origin)[1];
  vec->z =(*origin)[2];
}

void gams::utility::ros::GamsParser::parse_quaternion(
  geometry_msgs::Quaternion *quat,
  containers::NativeDoubleVector *origin)
{
  gams::pose::Quaternion q((*origin)[0],(*origin)[1],(*origin)[2]);
  quat->x = q.x();
  quat->y = q.y();
  quat->z = q.z();
  quat->w = q.w();
}

template <class T>
void gams::utility::ros::GamsParser::parse_int_array(std::vector<T> *array,
  containers::NativeIntegerVector *origin)
{
  for(unsigned int i = 0; i < array->size(); ++i)
  {
    array->push_back((*origin)[i]);
  }
}

template <size_t N>
void gams::utility::ros::GamsParser::parse_int_array(boost::array<int, N> *array,
  containers::NativeIntegerVector *origin)
{
  for(unsigned int i = 0; i < array->size(); ++i)
  {
   (*array)[i] =(*origin)[i];
  }
}

void gams::utility::ros::GamsParser::parse_pose(geometry_msgs::Pose *pose,
  containers::NativeDoubleVector *origin)
{
  gams::pose::Pose p;
  p.from_container(*origin);
  gams::pose::Quaternion q(p.as_orientation_vec());

  pose->position.x = p.x();
  pose->position.y = p.y();
  pose->position.z = p.z();
  pose->orientation.x = q.x();
  pose->orientation.y = q.y();
  pose->orientation.z = q.z();
  pose->orientation.w = q.w();
}
 
int gams::utility::ros::GamsParser::publish_transform(std::string frame_id,
  std::string frame_prefix)
{
  gams::pose::ReferenceFrame ref_frame =
    gams::pose::ReferenceFrame::load(*knowledge_, frame_id, -1, frame_prefix);
  if (ref_frame.valid())
  {
    if (ref_frame.origin_frame().valid())
    {
      /*std::cout << "Publishing transform " << frame_id << std::endl;
      std::cout << "Origin: " << ref_frame.origin_frame().id() << std::endl;
      std::cout << "Transform: " << ref_frame.origin() << std::endl;*/

      gams::pose::Pose origin = ref_frame.origin();
      gams::pose::Orientation orientation(origin);
      gams::pose::Quaternion quat(orientation);
      tf2_ros::TransformBroadcaster tf_brdcaster;

      geometry_msgs::Transform transform;
      geometry_msgs::Vector3 t;
      t.x = origin.x();
      t.y = origin.y();
      t.z = origin.z();
      transform.translation = t;
      geometry_msgs::Quaternion q;
      q.x = quat.x();
      q.y = quat.y();
      q.z = quat.z();
      q.w = quat.w();
      transform.rotation = q;
      geometry_msgs::TransformStamped st;
      st.transform = transform;
      st.child_frame_id = ref_frame.id();

      std_msgs::Header h;
      h.seq = 0;
      h.stamp = global_ros::Time::now();
      h.frame_id = ref_frame.origin_frame().id();
      st.header = h;

      tf_brdcaster.sendTransform(st);
      return 1;
    }
  }
  return 0;
}
