/**
 * @file RosParser.cpp
 * @author Jakob Auer <jakob.auer@gmail.com>
 *
 * This file contains parsing functionality for ROS topics
 **/
#include "RosParser.h"
#include <cmath>
#include <dlfcn.h>

gams::utility::ros::RosParser::RosParser (knowledge::KnowledgeBase * kb,
  std::string world_frame, std::string base_frame,
  std::map<std::string, std::string> capnp_types,
  std::map<std::string, int> circular_containers,
  knowledge::EvalSettings eval_settings, std::string frame_prefix) : 
  eval_settings_(eval_settings), frame_prefix_(frame_prefix)
{
  world_frame_ = world_frame;
  base_frame_ = base_frame;
  knowledge_ = kb;
  capnp_types_ = capnp_types;
  circular_container_stats_ = circular_containers;

  if ( world_frame != "" )
  {
    gams::pose::ReferenceFrame frame (world_frame,
      gams::pose::Pose (gams::pose::ReferenceFrame (), 0, 0));
    frame.save (*knowledge_,
      gams::pose::FrameEvalSettings(frame_prefix_, eval_settings_));
  }
}


void gams::utility::ros::RosParser::parse_message (
  const rosbag::MessageInstance m, std::string container_name)
{
  //Update sim time before parsing
  set_sim_time (m.getTime ());
  //Parse the message
  std::string datatype = m.getDataType ();

  auto search = capnp_types_.find(datatype);
  if (search != capnp_types_.end())
  {
    // If the message type is in the mapfile we have to parse this message
    // into a capnproto schema

    if (m.isType<sensor_msgs::PointCloud2> () &&
      capnp_types_["sensor_msgs/PointCloud2"] == "pcl")
    {
      /*parse_pointcloud2_pclschema (
        m.instantiate<sensor_msgs::PointCloud2> ().get (),
        container_name);
        */
      // open the library
      std::cout << "Opening r2g_pcl_plugin.so...\n";
      void* handle = dlopen ("./bin/r2g_pcl_plugin.so", RTLD_NOW);
      if (!handle) {
        std::cerr << "Cannot open library: " << dlerror () << '\n';
        return;
      }

      // load the symbol
      std::cout << "Loading symbol parse...\n";
      typedef void (*plugin_t) (const rosbag::MessageInstance*,
        madara::knowledge::KnowledgeBase*, 
        std::string);

      // reset errors
      dlerror();
      plugin_t parse = (plugin_t) dlsym (handle, "parse");
      const char *dlsym_error = dlerror ();
      if (dlsym_error) {
        std::cerr << "Cannot load symbol 'parse': " << dlsym_error << std::endl;
        dlclose(handle);
        return;
      }

      // use it to do the calculation
      std::cout << "Calling parse...\n";
      parse (&m, knowledge_, container_name);

      // close the library
      std::cout << "Closing library...\n";
      dlclose (handle);
    }
    else
    {
      parse_any(m, container_name);
    }
  }
  else if (m.isType<nav_msgs::Odometry> ())
  {
    parse_odometry (m.instantiate<nav_msgs::Odometry> ().get (),
        container_name);
  }
  else if (m.isType<sensor_msgs::Imu> ())
  {
    parse_imu (m.instantiate<sensor_msgs::Imu> ().get (), container_name);
  }
  else if (m.isType<sensor_msgs::LaserScan> ())
  {
    parse_laserscan (m.instantiate<sensor_msgs::LaserScan> ().get (),
        container_name);
  }
  else if (m.isType<geometry_msgs::Pose> ())
  {
    parse_pose (m.instantiate<geometry_msgs::Pose> ().get (),
        container_name);
  }
  else if (m.isType<geometry_msgs::PoseStamped> ())
  {
    parse_pose (&m.instantiate<geometry_msgs::PoseStamped> ().get ()->pose,
        container_name);
  }
  else if (m.isType<sensor_msgs::CompressedImage> ())
  {
    parse_compressed_image (
        m.instantiate<sensor_msgs::CompressedImage> ().get (),
        container_name);
  }
  else if (m.isType<sensor_msgs::PointCloud2> ())
  {
    parse_pointcloud2 (m.instantiate<sensor_msgs::PointCloud2> ().get (),
        container_name);
  }
  else if (m.isType<sensor_msgs::Range> ())
  {
    parse_range (m.instantiate<sensor_msgs::Range> ().get (),
        container_name);
  }
  else if (m.isType<sensor_msgs::FluidPressure> ())
  {
    parse_fluidpressure (m.instantiate<sensor_msgs::FluidPressure> ().get (),
        container_name);
  }
  else if (m.isType<tf2_msgs::TFMessage> ())
  {
    parse_tf_message (m.instantiate<tf2_msgs::TFMessage> ().get ());
  }
  else
  {
    parse_unknown (m, container_name);
  }
}



void gams::utility::ros::RosParser::parse_message (
  const topic_tools::ShapeShifter::ConstPtr& m,
  std::string container_name)
{
  // TODO: HANDLE CAPNP ANY TYPES!!!

  if (m->getDataType () == "std_msgs/Int32")
  {
    int value = m->instantiate<std_msgs::Int32>()->data;
    knowledge_->set (container_name, value, eval_settings_);
  }
  else if (m->getDataType () == "nav_msgs/Odometry")
  {
    parse_odometry (m->instantiate<nav_msgs::Odometry> ().get (),
        container_name);
  }
  else if (m->getDataType () == "sensor_msgs/Imu")
  {
    parse_imu (m->instantiate<sensor_msgs::Imu> ().get (), container_name);
  }
  else if (m->getDataType () == "sensor_msgs/LaserScan")
  {
    parse_laserscan (m->instantiate<sensor_msgs::LaserScan> ().get (),
        container_name);
  }
  else if (m->getDataType () == "geometry_msgs/Pose")
  {
    parse_pose (m->instantiate<geometry_msgs::Pose> ().get (),
        container_name);
  }
  else if (m->getDataType () == "geometry_msgs/PoseStamped")
  {
    parse_pose (&m->instantiate<geometry_msgs::PoseStamped> ().get ()->pose,
        container_name);
  }
  else if (m->getDataType () == "sensor_msgs/CompressedImage")
  {
    parse_compressed_image (
        m->instantiate<sensor_msgs::CompressedImage> ().get (),
        container_name);
  }
  else if (m->getDataType () == "sensor_msgs/PointCloud2")
  {
    parse_pointcloud2 (m->instantiate<sensor_msgs::PointCloud2> ().get (),
        container_name);
  }
  else if (m->getDataType () == "sensor_msgs/Range")
  {
    parse_range (m->instantiate<sensor_msgs::Range> ().get (),
        container_name);
  }
  else if (m->getDataType () == "sensor_msgs/FluidPressure")
  {
    parse_fluidpressure (m->instantiate<sensor_msgs::FluidPressure> ().get (),
        container_name);
  }
  else if (m->getDataType () == "tf2_msgs/TFMessage")
  {
    parse_tf_message (m->instantiate<tf2_msgs::TFMessage> ().get ());
  }
}

/**
* Parses unknown messages using ros_type_introspection
* DO NOT USE THIS FOR TYPES WITH LARGE ARRAYS
**/
void gams::utility::ros::RosParser::parse_unknown (const rosbag::MessageInstance m,
  std::string container_name)
{
  // see https://github.com/facontidavide/type_introspection_tests/blob/master/example/rosbag_example.cpp
  // see http://wiki.ros.org/ros_type_introspection chapter "The Deserializer"
  // this is not recommend to be done to known types because of it's huge
  // memory usage!
  
  const std::string& topic_name  = m.getTopic ();

  // write the message into the buffer
  const size_t msg_size  = m.size ();
  parser_buffer_.resize (msg_size);
  global_ros::serialization::OStream stream (parser_buffer_.data (),
    parser_buffer_.size ());
  m.write (stream);

  RosIntrospection::FlatMessage& flat_container =
    flat_containers_[topic_name];
  RosIntrospection::RenamedValues& renamed_values =
    renamed_vectors_[topic_name];

  // deserialize and rename the vectors
  bool success = parser_.deserializeIntoFlatContainer ( topic_name,
  absl::Span<uint8_t> (parser_buffer_),
  &flat_container, 500 );

  if (!success)
  {
    std::cout << "Topic " << topic_name <<
      " could not be parsed successfully due to large array sizes!" <<
      std::endl;
  }

  parser_.applyNameTransform ( topic_name,
    flat_container, &renamed_values );

  // Save the content of the message to the knowledgebase
  int topic_len = topic_name.length ();
  for (auto it: renamed_values)
  {
    const std::string& key = it.first;
    const RosIntrospection::Variant& value   = it.second;

    std::string var_name = key.substr (topic_len);
    std::replace ( var_name.begin (), var_name.end (), '/', '.');
    if (value.getTypeID () == RosIntrospection::BuiltinType::BOOL ||
      value.getTypeID () == RosIntrospection::BuiltinType::BYTE ||
      value.getTypeID () == RosIntrospection::BuiltinType::CHAR ||
      value.getTypeID () == RosIntrospection::BuiltinType::UINT8 ||
      value.getTypeID () == RosIntrospection::BuiltinType::UINT16 ||
      value.getTypeID () == RosIntrospection::BuiltinType::UINT32 ||
      value.getTypeID () == RosIntrospection::BuiltinType::UINT64 ||
      value.getTypeID () == RosIntrospection::BuiltinType::INT8 ||
      value.getTypeID () == RosIntrospection::BuiltinType::INT16 ||
      value.getTypeID () == RosIntrospection::BuiltinType::INT32 ||
      value.getTypeID () == RosIntrospection::BuiltinType::INT64)
    {
      // Use Integer container for these types
      containers::Integer value_container (container_name + var_name,
        *knowledge_, eval_settings_);
      value_container = value.convert<double> ();
    }
    else
    {
      // otherwise convert to double
      containers::Double value_container (container_name + var_name,
        *knowledge_, eval_settings_);
      value_container = value.convert<double> ();
    }
  }
  for (auto it: flat_container.name)
  {
    const std::string& key    = it.first.toStdString ();
    const std::string& value  = it.second;

    std::string var_name = key.substr (topic_len);
    std::replace ( var_name.begin (), var_name.end (), '/', '.');
    containers::String value_container (container_name + var_name, *knowledge_,
      eval_settings_);
    value_container = value;
  }
}


void gams::utility::ros::RosParser::registerMessageDefinition(
  std::string topic_name, RosIntrospection::ROSType type,
  std::string definition)
{
  parser_.registerMessageDefinition(topic_name, type, definition);
}

void gams::utility::ros::RosParser::registerRenamingRules(
  RosIntrospection::ROSType type,
  std::vector<RosIntrospection::SubstitutionRule> rules)
{
  parser_.registerRenamingRules(type, rules);
}

/**
* Parses a ROS Odometry Message into two Madara Containers. One for the
* location and a second one for the orientation.
* @param  odom       the nav_msgs::Odometry message
* @param  knowledge   Knowledbase
**/
void gams::utility::ros::RosParser::parse_odometry (nav_msgs::Odometry * odom,
  std::string container_name)
{

  containers::NativeDoubleVector odom_covariance (
    container_name + ".pose.covariance", *knowledge_, 36, eval_settings_);
  containers::NativeDoubleVector twist_covariance (
    container_name + ".twist.covariance", *knowledge_, 36, eval_settings_);

  parse_pose (&odom->pose.pose, container_name + ".pose");
  parse_float64_array (&odom->pose.covariance, &odom_covariance);
  parse_float64_array (&odom->twist.covariance, &twist_covariance);
  parse_twist (&odom->twist.twist, container_name + ".twist");

  containers::String frame_id (container_name + ".frame_id", *knowledge_,
    eval_settings_);
  frame_id = odom->header.frame_id;
  containers::String child_frame_id (container_name + ".child_frame_id",
    *knowledge_, eval_settings_);
  child_frame_id = odom->child_frame_id;
}

/**
* Parses a ROS IMU Message
* @param  imu       the sensor_msgs::Imu message
* @param  knowledge   Knowledgbase
**/
void gams::utility::ros::RosParser::parse_imu (sensor_msgs::Imu *imu,
  std::string container_name)
{
  containers::NativeDoubleVector orientation (
    container_name + ".orientation", *knowledge_, 3, eval_settings_);
  containers::NativeDoubleVector angular_velocity (container_name +
    ".angular_velocity", *knowledge_, 3, eval_settings_);
  containers::NativeDoubleVector linear_acceleration (container_name +
    ".linear_acceleration", *knowledge_, 3, eval_settings_);
  containers::NativeDoubleVector orientation_covariance (container_name +
    ".orientation_covariance", *knowledge_, 9, eval_settings_);
  containers::NativeDoubleVector angular_velocity_covariance (container_name +
    ".angular_velocity_covariance", *knowledge_, 9, eval_settings_);
  containers::NativeDoubleVector linear_acceleration_covariance (
    container_name + ".linear_acceleration_covariance", *knowledge_, 9,
    eval_settings_);
  containers::String frame_id(container_name + ".frame_id", *knowledge_,
    eval_settings_);

  frame_id = imu->header.frame_id;


  parse_quaternion (&imu->orientation, &orientation);
  parse_vector3 (&imu->angular_velocity, &angular_velocity);
  parse_vector3 (&imu->linear_acceleration, &linear_acceleration);
  parse_float64_array (&imu->orientation_covariance, &orientation_covariance);
  parse_float64_array (&imu->angular_velocity_covariance,
    &angular_velocity_covariance);
  parse_float64_array (&imu->linear_acceleration_covariance,
    &linear_acceleration_covariance);
}

/**
* Parses a ROS LaserScan Message
* @param  laser         the sensor_msgs::LaserScan message
* @param  knowledge     Knowledgbase
* @param  container_name    container namespace (e.g. "laser")
**/
void gams::utility::ros::RosParser::parse_laserscan (
  sensor_msgs::LaserScan * laser, std::string container_name)
{
  //Ranges
  int ranges_size = laser->ranges.size ();
  containers::NativeDoubleVector ranges (container_name + ".ranges",
    *knowledge_, ranges_size, eval_settings_);
  parse_float64_array (&laser->ranges, &ranges);
  //Intensities
  int intensities_size = laser->intensities.size ();
  containers::NativeDoubleVector intensities (container_name + ".intensities",
    *knowledge_, intensities_size, eval_settings_);
  parse_float64_array (&laser->intensities, &intensities);
  //Parameters
  containers::Double angle_min (container_name + ".angle_min", *knowledge_,
    eval_settings_);
  angle_min = laser->angle_min;
  containers::Double angle_max (container_name + ".angle_max", *knowledge_,
    eval_settings_);
  angle_max = laser->angle_max;
  containers::Double angle_increment (
    container_name + ".angle_increment", *knowledge_, eval_settings_);
  angle_increment = laser->angle_increment;
  containers::Double time_increment (container_name + ".time_increment",
    *knowledge_, eval_settings_);
  time_increment = laser->time_increment;
  containers::Double scan_time (container_name + ".scan_time", *knowledge_,
    eval_settings_);
  scan_time = laser->scan_time;
  containers::Double range_min (container_name + ".range_min", *knowledge_,
    eval_settings_);
  range_min = laser->range_min;
  containers::Double range_max (container_name + ".range_max", *knowledge_,
    eval_settings_);
  range_max = laser->range_max;

}

/**
* Parses a ROS Range Message
* @param  range         the sensor_msgs::Range message
* @param  knowledge     Knowledgbase
* @param  container_name    container namespace (e.g. "range")
**/
void gams::utility::ros::RosParser::parse_range (sensor_msgs::Range * range,
  std::string container_name)
{
  containers::Double field_of_view (
    container_name + ".field_of_view", *knowledge_, eval_settings_);
  field_of_view = range->field_of_view;
  containers::Double min_range (container_name + ".min_range", *knowledge_,
    eval_settings_);
  min_range = range->min_range;
  containers::Double max_range (container_name + ".max_range", *knowledge_,
    eval_settings_);
  max_range = range->max_range;
  containers::Double range_val (container_name + ".range", *knowledge_,
    eval_settings_);
  range_val = range->range;
  containers::Integer radiation_type (container_name + ".radiation_type",
    *knowledge_, eval_settings_);
  radiation_type = range->radiation_type;
}

/**
* Parses a ROS FluidPressure Message
* @param  press         the sensor_msgs::FluidPressure message
* @param  knowledge     Knowledgbase
* @param  container_name    container namespace
**/
void gams::utility::ros::RosParser::parse_fluidpressure (
  sensor_msgs::FluidPressure * press, std::string container_name)
{
  containers::Double fluid_pressure (
    container_name + ".fluid_pressure", *knowledge_, eval_settings_);
  fluid_pressure = press->fluid_pressure;
  containers::Double variance (container_name + ".variance", *knowledge_,
    eval_settings_);
  variance = press->variance;
}


/**
* Parses a ROS CompressedImage Message
* @param  laser         the sensor_msgs::CompressedImage message
* @param  knowledge     Knowledgbase
* @param  container_name    container namespace (e.g. "image")
**/
void gams::utility::ros::RosParser::parse_compressed_image (
  sensor_msgs::CompressedImage * img, std::string container_name)
{
  containers::String format (container_name + ".format", *knowledge_,
    eval_settings_);
  format = img->format;
  int len = img->data.size ();
  //TODO: data is a vector of int8 which is parsed into an
  // NativeIntegerVector -> change to NativeCharVector etc???
  containers::NativeIntegerVector data (container_name + ".data",
    *knowledge_, len, eval_settings_);
  parse_int_array (&img->data, &data);
}

/**
* Parses a ROS TF Message
* @param  tf           the tf2_msgs::TFMessage message
* @param  knowledge     Knowledgbase
**/
void gams::utility::ros::RosParser::parse_tf_message (tf2_msgs::TFMessage * tf)
{
  // Expire frames after 0.1 seconds
  gams::pose::ReferenceFrame::default_expiry (10000000000);
  uint64_t max_timestamp = 0;
  for (tf2_msgs::TFMessage::_transforms_type::iterator iter =
    tf->transforms.begin (); iter != tf->transforms.end (); ++iter)
  {
    // read frame names_ 
    std::string frame_id = iter->header.frame_id;
    std::string child_frame_id = iter->child_frame_id;
    std::replace ( frame_id.begin (), frame_id.end (), '/', '_');
    std::replace ( child_frame_id.begin (), child_frame_id.end (), '/', '_');

    // parse the rotation and orientation
    gams::pose::ReferenceFrame parent;
    try
    {
      parent = gams::pose::ReferenceFrame::load (
        *knowledge_, frame_id);
    }
    catch (gams::exceptions::ReferenceFrameException)
    {
      parent = gams::pose::ReferenceFrame (frame_id,
        gams::pose::Pose (gams::pose::ReferenceFrame (), 0, 0));
    }

    gams::pose::Quaternion quat (iter->transform.rotation.x,
                  iter->transform.rotation.y,
                  iter->transform.rotation.z,
                  iter->transform.rotation.w);
    gams::pose::PositionVector position (
                    iter->transform.translation.x,
                    iter->transform.translation.y,
                    iter->transform.translation.z);

    gams::pose::Pose pose (parent, position,
        gams::pose::OrientationVector (quat));
    uint64_t timestamp = iter->header.stamp.sec;
    timestamp = timestamp*1000000000 + iter->header.stamp.nsec;
    gams::pose::ReferenceFrame child_frame (child_frame_id, pose, timestamp);

    child_frame.save (*knowledge_,
      gams::pose::FrameEvalSettings(frame_prefix_, eval_settings_));
    if (timestamp > max_timestamp)
      max_timestamp = timestamp;
  }
  if (base_frame_ != "" && world_frame_ != "")
  {
    // World and base frames are defined so we can calculate the agent
    // location and orientation
    gams::pose::ReferenceFrame world;
    gams::pose::ReferenceFrame base;
    try
    {
      std::vector<std::string> ids = {base_frame_, world_frame_};
      std::vector<gams::pose::ReferenceFrame> frames = 
        gams::pose::ReferenceFrame::load_tree (*knowledge_, ids);
      for (gams::pose::ReferenceFrame frame : frames)
      {
        if (frame.id() == base_frame_)
        {
          base = frame;
        }
        else if (frame.id() == world_frame_)
        {
          world = frame;
        }
      }
    }
    catch (gams::exceptions::ReferenceFrameException ex)
    {
      return;
    }
    try
    {
      auto base_origin = base.origin ();
      gams::pose::Pose base_pose = base_origin.transform_to (world);
      containers::NativeDoubleVector location ("agent.0.location",
        *knowledge_, 3, eval_settings_);
      containers::NativeDoubleVector orientation ("agent.0.orientation",
        *knowledge_, 3, eval_settings_);
      location.set (0, base_pose.as_location_vec ().get (0), eval_settings_);
      location.set (1, base_pose.as_location_vec ().get (1), eval_settings_);
      location.set (2, base_pose.as_location_vec ().get (2), eval_settings_);
      orientation.set (0, base_pose.as_orientation_vec ().get (0),
        eval_settings_);
      orientation.set (1, base_pose.as_orientation_vec ().get (1),
        eval_settings_);
      orientation.set (2, base_pose.as_orientation_vec ().get (2),
        eval_settings_);
    }
    catch (gams::pose::unrelated_frames ex)
    {}
  }
}


/**
* Parses a ROS PointCloud2 Message
* @param  laser         the sensor_msgs::PointCloud2 message
* @param  knowledge     Knowledgbase
* @param  container_name    container namespace (e.g. "pointcloud")
**/
void gams::utility::ros::RosParser::parse_pointcloud2 (sensor_msgs::PointCloud2 * pointcloud,
  std::string container_name)
{
  containers::Integer height (container_name + ".height", *knowledge_,
    eval_settings_);
  height = pointcloud->height;
  containers::Integer width (container_name + ".width", *knowledge_,
    eval_settings_);
  width = pointcloud->width;
  containers::Integer point_step (container_name + ".point_step", *knowledge_,
    eval_settings_);
  point_step = pointcloud->point_step;
  containers::Integer row_step (container_name + ".row_step", *knowledge_,
    eval_settings_);
  row_step = pointcloud->row_step;

  //TODO: data is a vector of int8 which is parsed into an NativeIntegerVector
  //-> change to NativeCharVector etc???
  int len = pointcloud->data.size ();
  containers::NativeIntegerVector data (container_name + ".data",
    *knowledge_, len, eval_settings_);
  parse_int_array (&pointcloud->data, &data);

  int field_index = 0;
  for (sensor_msgs::PointCloud2::_fields_type::iterator iter =
        pointcloud->fields.begin (); iter != pointcloud->fields.end (); ++iter)
  {
    std::string name = container_name +
      ".fields." + std::to_string (field_index);
    containers::Integer offset (name + ".offset", *knowledge_, eval_settings_);
    offset = iter->offset;
    containers::Integer datatype (name + ".datatype", *knowledge_,
      eval_settings_);
    datatype = iter->datatype;
    containers::Integer count (name + ".count", *knowledge_, eval_settings_);
    count = iter->count;
    containers::String fieldname (name + ".name", *knowledge_, eval_settings_);
    fieldname = iter->name;
    ++field_index;
  }

  containers::Integer is_bigendian (container_name + ".is_bigendian",
    *knowledge_, eval_settings_);
  is_bigendian = pointcloud->is_bigendian;
  containers::Integer is_dense (container_name + ".is_dense", *knowledge_,
    eval_settings_);
  is_dense = pointcloud->is_dense;
}

/**
* Parses a ROS Twist Message
* @param  twist         the geometry_msgs::Twist message
* @param  knowledge     Knowledgbase
* @param  container_name    container namespace (e.g. "laser")
**/
void gams::utility::ros::RosParser::parse_twist (geometry_msgs::Twist *twist,
  std::string container_name)
{
  containers::NativeDoubleVector linear (container_name + ".linear",
    *knowledge_, 3, eval_settings_);
  containers::NativeDoubleVector angular (container_name + ".angular",
    *knowledge_, 3, eval_settings_);
  parse_vector3 (&twist->linear, &linear);
  parse_vector3 (&twist->angular, &angular);
}


/**
* Parses a ROS Pose Message
* @param  pose         the geometry_msgs::Pose message
* @param  knowledge     Knowledgbase
* @param  container_name    container namespace (e.g. "pose")
**/
void gams::utility::ros::RosParser::parse_pose (geometry_msgs::Pose *pose,
  std::string container_name)
{
  containers::NativeDoubleVector cont (container_name, *knowledge_, 6,
    eval_settings_);


  gams::pose::Quaternion quat (pose->orientation.x,
                pose->orientation.y,
                pose->orientation.z,
                pose->orientation.w);
  gams::pose::Position position (pose->position.x,
                  pose->position.y,
                  pose->position.z);
  gams::pose::Pose p (position, gams::pose::Orientation (quat));
  p.to_container (cont);
}

void gams::utility::ros::RosParser::parse_vector3 (geometry_msgs::Vector3 *vec,
  containers::NativeDoubleVector *target)
{
  target->set (0, vec->x, eval_settings_);
  target->set (1, vec->y, eval_settings_);
  target->set (2, vec->z, eval_settings_);
}
/**
* Parses a ROS geometry_msgs::Point message
* @param  point_msg    the geometry_msgs::Point message
* @param  point      the container
**/
void gams::utility::ros::RosParser::parse_point (geometry_msgs::Point *point_msg,
  containers::NativeDoubleVector *point)
{
  point->set (0, point_msg->x, eval_settings_);
  point->set (1, point_msg->y, eval_settings_);
  point->set (2, point_msg->z, eval_settings_);
}

/**
* Parses a ROS geometry_msgs::Quaternion message
* @param  quat      the geometry_msgs::Quaternion message
* @param  orientatiom   the container 
**/
void gams::utility::ros::RosParser::parse_quaternion (
  geometry_msgs::Quaternion *quat,
  containers::NativeDoubleVector *orientation)
{
  tf::Quaternion tfquat (quat->x, quat->y, quat->z, quat->w);
  tf::Matrix3x3 m (tfquat);
  double roll, pitch, yaw;
  m.getRPY (roll, pitch, yaw);

  orientation->set (0, roll, eval_settings_);
  orientation->set (1, pitch, eval_settings_);
  orientation->set (2, yaw, eval_settings_);
}



template <size_t N>
void gams::utility::ros::RosParser::parse_float64_array (
  boost::array<double, N> *array,
  containers::NativeDoubleVector *target)
{
  int i = 0;
  for (typename boost::array<double, N>::iterator iter (array->begin ());
    iter != array->end (); ++iter)
  {
    target->set (i, *iter, eval_settings_);
    i++;
  }
}

void gams::utility::ros::RosParser::parse_float64_array (
  std::vector<float> *array,
  containers::NativeDoubleVector *target)
{
  int i = 0;
  for (std::vector<float>::iterator iter = array->begin ();
    iter != array->end (); ++iter)
  {
    target->set (i, *iter, eval_settings_);
    i++;
  }
}

template <class T>
void gams::utility::ros::RosParser::parse_int_array (std::vector<T> *array,
  containers::NativeIntegerVector *target)
{
  int i = 0;
  for (typename std::vector<T>::iterator iter = array->begin ();
    iter != array->end (); ++iter)
  {
    target->set (i, *iter, eval_settings_);
    i++;
  }
}

template <size_t N>
void gams::utility::ros::RosParser::parse_int_array (
  boost::array<int, N> *array,
  containers::NativeIntegerVector *target)
{
  int i = 0;
  for (typename boost::array<int, N>::iterator iter (array->begin ());
    iter != array->end (); ++iter)
  {
    target->set (i, *iter, eval_settings_);
    i++;
  }
}

std::string gams::utility::ros::ros_to_gams_name (std::string ros_topic_name)
{
  // Convert ros_topic_name to lower case
  std::transform (ros_topic_name.begin (),
    ros_topic_name.end (), ros_topic_name.begin (), ::tolower);
  std::string name = ros_topic_name.substr (1);
  std::string rosbag_robot_prefix = "robot_";
   
   std::string topic = ros_topic_name;
  if (name.find (rosbag_robot_prefix) == 0)
  {
    //remove the robot prefix
    int namespace_end = name.find ("/") + 1;
    //cut the prefix
    topic = name.substr (namespace_end);
  }
  std::replace (topic.begin (), topic.end (), '/', '.');


  return topic;
}

/**
 * Loads a capnp schema from a specified path
 **/
void gams::utility::ros::RosParser::load_capn_schema(std::string path)
{
  try{
    int fd = open (path.c_str (), 0, O_RDONLY);
    capnp::StreamFdMessageReader schema_message_reader (fd);
    auto schema_reader = 
      schema_message_reader.getRoot<capnp::schema::CodeGeneratorRequest> ();
  
    for (auto schema : schema_reader.getNodes ()) {
      std::string schema_name = cleanCapnpSchemaName (schema.getDisplayName ());
      schemas_[schema_name] = capnp_loader_.load (schema);
    }
  }
  catch(...)
  {
    std::cout << "Could not load schema " << path << "!" << std::endl;
  }
}

/**
 * Helper method to print all available schemas
 **/
void gams::utility::ros::RosParser::print_schemas()
{
  std::cout << "Available schemas: " << std::endl;
  for (auto schema : schemas_)
  {
    std::cout << "- " << schema.first << std::endl;
  }
}

/*
Searches for a given name in the schema builder and returns the new builder
*/
capnp::DynamicStruct::Builder gams::utility::ros::RosParser::get_dyn_capnp_struct(
  capnp::DynamicStruct::Builder builder,
  std::string name)
{
  if (name == "")
  {
    return builder;
  }
  // remove the leading slash
  std::string n = name.substr (1);
  std::istringstream f (n);
  std::string s;
  capnp::DynamicStruct::Builder dyn = builder;

  // Iterate through the dynamic struct
  while (getline (f, s, '/')) {
    dyn = dyn.get(s).as<capnp::DynamicStruct>();
  }
  return dyn;
}


template <class T>
void gams::utility::ros::RosParser::set_dyn_capnp_enum_value (
  capnp::DynamicStruct::Builder dynvalue,
  std::string var_name, T val)
{ 
  dynvalue.set(var_name, val);
}

// The template specialication has to be defined in these namespace structure
// because of a gcc bug. 
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=56480
namespace gams
{
  namespace utility
  {
    namespace ros
    {
      void set_dyn_capnp_enum_value(
        capnp::DynamicStruct::Builder dynvalue,
        std::string var_name, char* val)
      {
        // We need to check for the enumerants
        auto enum_schema =
          dynvalue.get(var_name).as<capnp::DynamicEnum>().getSchema();
        auto enumerant = enum_schema.getEnumerantByName(kj::StringPtr(val));
        dynvalue.set(var_name, enumerant.getOrdinal());
      }
    }
  }
}

/*
Sets the value of a specifiec schema member
*/
template <class T>
void gams::utility::ros::RosParser::set_dyn_capnp_value(
  capnp::DynamicStruct::Builder builder,
  std::string name,
  T val,
  unsigned int array_size)
{
    //remove _ values from the name string and change it to camelcase
    std::stringstream camelcase;
    for (unsigned int i = 0; i < name.size(); i++)
    {
      if (name[i] == '_')
      {
        camelcase.put(toupper(name[i + 1]));
        i++;
      }
      else
      {
        if ( i == 0 )
        {
          // first character has to be lowercase
          camelcase.put(tolower(name[i]));
        }
        else
        {
          camelcase.put(name[i]);
        }
      }
    }
    name = camelcase.str();

    int struct_end = name.find_last_of("/");
    std::string struct_name = name.substr(0,struct_end);
    std::string var_name = name.substr(struct_end+1);

    //Members start with lower case so set this for the varname
    var_name[0] = tolower(var_name[0]);

    auto dynvalue = get_dyn_capnp_struct(builder, struct_name);

    std::size_t dot_pos = var_name.find(".");
    if (dot_pos != std::string::npos)
    {
      //This is a list
      std::string var = var_name.substr(0, dot_pos);
      int index = std::stoi(var_name.substr(dot_pos+1));
      if (index == 0)
      {
        // First element so init
        dynvalue.init(var, array_size);
      }
      auto lst = dynvalue.get(var).as<capnp::DynamicList>();
      lst.set(index, val);
    }
    else
    {
        if (dynvalue.asReader().get(var_name).getType() ==
          capnp::DynamicValue::BOOL)
        {
          // We need to specifically cast bool values before asignment
          bool bool_val = (bool) val;
          dynvalue.set(var_name, bool_val);
        }
        else if (dynvalue.asReader().get(var_name).getType() ==
          capnp::DynamicValue::ENUM)
        {
          set_dyn_capnp_enum_value(dynvalue, var_name, val);
        }
        else
        {
          dynvalue.set(var_name, val);
        }
    }
}


/*
  Determines the size of an ros introspection array
*/
unsigned int gams::utility::ros::RosParser::get_array_size(std::string var_name,
  RosIntrospection::RenamedValues* array)
{
  // This method caches list sizes into the private member ros_array_sizes_.
  // IMPORTANT: Keep in mind to clear ros_array_sizes_ after each message!!
  unsigned int len = 0;
  std::size_t dot_pos = var_name.find(".");
  std::string array_name = var_name.substr(0, dot_pos);
  if (dot_pos == std::string::npos)
  {
    // This is no array
    len = 1;
  }
  else
  {
    auto cached = ros_array_sizes_.find(array_name);
    if (cached != ros_array_sizes_.end())
    {
      // We already determined the arraylength in this message
      len = cached->second;
    }
    else
    {
      // Count the members to determine the length of the array
      for ( auto it : *array)
      {
        std::string key = it.first;
        if (key.rfind(array_name, 0) == 0)
        {
          len++;
        }
      }
    }
  }
  ros_array_sizes_[array_name] = len;
  return len;
}

void gams::utility::ros::RosParser::parse_any ( std::string datatype,
  std::string topic_name,
  std::vector<uint8_t> & parser_buffer,
  std::string container_name)
{
  // This method uses ros_type_introspection to dynamically map values from
  // ros messages to capnproto schemas

  std::string schema_name = capnp_types_[datatype];
  capnp::MallocMessageBuilder buffer;
  capnp::DynamicStruct::Builder capnp_builder;
  try
  {
    auto schema = schemas_.at(schema_name).asStruct();
    capnp_builder = buffer.initRoot<capnp::DynamicStruct>(schema);
    madara::knowledge::Any::register_schema(schema_name.c_str(), schema);
  }
  catch(...)
  {
    std::cout << "Schema with name " << schema_name << "not found!"
      << std::endl;
    exit(1);
  }

  RosIntrospection::FlatMessage& flat_container =
    flat_containers_[topic_name];
  RosIntrospection::RenamedValues& renamed_values =
    renamed_vectors_[topic_name];

  // deserialize and rename the vectors
  bool success = parser_.deserializeIntoFlatContainer ( topic_name,
    absl::Span<uint8_t> (parser_buffer),
    &flat_container, 10240000 );

  if (!success)
  {
    std::cout << "Topic " << topic_name <<
      " could not be parsed successfully due to large array sizes!" <<
      std::endl;
  }

  parser_.applyNameTransform ( topic_name,
    flat_container, &renamed_values );


  // Save the content of the message to the knowledgebase
  int topic_len = topic_name.length ();
  for (auto it: renamed_values)
  {
    const std::string& key = it.first;
    const RosIntrospection::Variant& value   = it.second;

    int array_size = get_array_size(key, &renamed_values);

    std::string name = key.substr (topic_len);


    double val = NAN;
    try
    {
      val = value.convert<double> ();
    }
    catch ( const std::exception& e )
    {
      // Value is NAN if it is not readable
    }
    // Apply name substitution rules from the mapfile
    name = substitute_name(datatype, name);
    if (name == IGNORE_MARKER)
    {
      // This member is marked as to be ignored
      continue;
    } else if (
      strncmp(name.c_str(), NSEC_MARKER.c_str(), NSEC_MARKER.size()) == 0)
    {
      // This member has to be converted to nsec
      val = (unsigned long) (val * 1e9);
      // remove NSEC marker
      name = name.substr(NSEC_MARKER.size());
    }
    try
    {
      set_dyn_capnp_value<double>(capnp_builder, name, val, array_size);
    }
    catch(kj::Exception ex)
    {
      std::cout << "Failed to set " << name << " from topic " <<
        topic_name << " (" << datatype << ")! (" <<
        std::string(ex.getDescription()) << ")" << std::endl;
    }
    
  }
  for (auto it: flat_container.name)
  {
    const std::string& key    = it.first.toStdString ();
    const std::string& value  = it.second;

    int array_size = get_array_size(key, &renamed_values);

    auto val = strdup(value.c_str());
    std::string name = key.substr (topic_len);

    // Apply name substitution rules from the mapfile
    name = substitute_name(datatype, name);
    if (name == IGNORE_MARKER)
    {
      // This member is marked as to be ignored
      continue;
    }
    try
    {
      set_dyn_capnp_value<char*>(capnp_builder, name, val, array_size);
    }
    catch(kj::Exception ex)
    {
      std::cout << "Failed to set " << name << " from topic " <<
        topic_name << " (" << datatype << ")! (" <<
        std::string(ex.getDescription()) << ")" << std::endl;
    }
  }
  // Clear the array size cache
  ros_array_sizes_.clear();

  // Write to the Any object
  madara::knowledge::GenericCapnObject any(schema_name.c_str(), buffer);

  // Check if this has to be stored in CircularBuffers
  // TODO: remove this once the NativeCircularBuffers ar ready for use
  auto search = circular_container_stats_.find(container_name);
  if (search != circular_container_stats_.end())
  {
    //This is var has to be a circular buffer
    auto prod_search = circular_producers_.find(container_name);
    madara::knowledge::containers::CircularBuffer c_buffer;
    if (prod_search != circular_producers_.end())
    {
      // we already have a circular buffer producer
      c_buffer = prod_search->second;
    }
    else
    {
      // we have to create a new producer
      int buffer_size = search->second;
      c_buffer = madara::knowledge::containers::CircularBuffer(container_name,
        *knowledge_, buffer_size);
    }
    // add the value to the buffer
    c_buffer.add(any);
  }
  else
  {
    knowledge_->set_any(container_name, any);
  }
}

/**
 * Wrapper for parse_any so that it can be called from ShapeShifter objects
 **/

void gams::utility::ros::RosParser::parse_any (std::string topic,
  const topic_tools::ShapeShifter & m,
  std::string container_name)
{
  // write the message into the buffer
  const size_t msg_size  = m.size ();
  std::vector<uint8_t> parser_buffer;

  parser_buffer.resize (msg_size);
  global_ros::serialization::OStream stream (parser_buffer.data (),
    parser_buffer.size ());
  m.write (stream);

  parse_any(m.getDataType(), topic, parser_buffer, container_name);
}

/**
 * Wrapper for parse_any so that it can be called from rosbag MessageInstance
 * objects
 **/
void gams::utility::ros::RosParser::parse_any (const rosbag::MessageInstance & m,
  std::string container_name)
{
  // write the message into the buffer
  const std::string& topic  = m.getTopic ();

  const size_t msg_size  = m.size ();
  std::vector<uint8_t> parser_buffer;

  parser_buffer.resize (msg_size);
  global_ros::serialization::OStream stream (parser_buffer.data (),
    parser_buffer.size ());
  m.write (stream);

  parse_any(m.getDataType(), topic, parser_buffer, container_name);
}

void gams::utility::ros::RosParser::parse_pointcloud2_pclschema (
  sensor_msgs::PointCloud2 * pointcloud,
  std::string container_name)
{
  // THIS METHOD IS HARDCODED TO RUN WITH PCL BASED SCHEMAS

  // Load PCL schema
  std::string pointcloud_schema_name = "PointCloudXYZ";
  capnp::MallocMessageBuilder buffer;
  capnp::DynamicStruct::Builder pointcloud_builder;
  try
  {
    auto pointcloud_schema = schemas_.at(pointcloud_schema_name).asStruct();
    pointcloud_builder = buffer.initRoot<capnp::DynamicStruct>(pointcloud_schema);
    madara::knowledge::Any::register_schema(pointcloud_schema_name.c_str(),
      pointcloud_schema);
  }
  catch(...)
  {
    std::cout << "Schema with name " << pointcloud_schema_name <<
      "not found!" << std::endl;
    exit(1);
  }

  // Convert from ROS PointCloud2 to a PCL PointCloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL (*pointcloud, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2 (pcl_pc2, *temp_cloud);

  //Fill the list of points with data
  auto point_list = pointcloud_builder.init ("points",
    (*temp_cloud).size ()).as<capnp::DynamicList>();
  int point_counter = 0;
  for (pcl::PointXYZ point : *temp_cloud)
  {
    auto capnp_point = point_list[point_counter].as<capnp::DynamicStruct> ();
    capnp_point.set("x", point.x);
    capnp_point.set("y", point.y);
    capnp_point.set("z", point.z);
    point_counter++;
  }

  // Now fill the other fields
  pointcloud_builder.set ("tov",
    ((unsigned long)pointcloud->header.stamp.sec * 1e9) +
    (unsigned long)pointcloud->header.stamp.nsec);
  pointcloud_builder.set ("frameId", pointcloud->header.frame_id.c_str());
  pointcloud_builder.set ("width", pointcloud->width);
  pointcloud_builder.set ("height", pointcloud->height);
  pointcloud_builder.set ("isDense", (bool)pointcloud->is_dense);

  // Store in the knowledgebase
  madara::knowledge::GenericCapnObject any(pointcloud_schema_name.c_str(),
    buffer);
  knowledge_->set_any(container_name, any);
}

/**
 * Update the simtime from a ros Time
 **/
void gams::utility::ros::RosParser::set_sim_time (global_ros::Time rostime)
{
  #ifdef MADARA_FEATURE_SIMTIME
  uint64_t sim_time = (unsigned long)rostime.sec * 1e9 +
    (unsigned long)rostime.nsec;
  madara::utility::sim_time_notify(sim_time, 0.0);
  #endif
}

/**
 * Renaming rules for any type conversion
 **/
void gams::utility::ros::RosParser::register_rename_rules (std::map<std::string,
  std::map<std::string, std::string>> name_substitution_map)
{
  name_substitution_map_ = std::move(name_substitution_map);
}

/**
 * Name substitusion for any type conversion
 **/
std::string gams::utility::ros::RosParser::substitute_name (std::string type,
  std::string name)
{
  //Check if this is an array
  std::size_t dot_pos = name.find(".");
  std::string array_index = "";
  std::string search_name = name;
  if (dot_pos != std::string::npos)
  {
    // This is no array
    array_index = search_name.substr(dot_pos);
    search_name = search_name.substr(0, dot_pos);
  }
  std::vector<std::string> values = {type, "general"};
  for (std::string it : values)
  {
    auto search = name_substitution_map_.find (it);
    if (search != name_substitution_map_.end ())
    {
      std::map <std::string, std::string> name_map = search->second;
      
      auto name_search = name_map.find (search_name);
      if (name_search != name_map.end ())
      {
        if (name_search->second == IGNORE_MARKER)
        {
          return name_search->second;
        }
        else
        {
          return name_search->second + array_index;
        }

      }
    }
  }
  return name;
}




