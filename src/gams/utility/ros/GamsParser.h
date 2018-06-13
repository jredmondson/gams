#ifndef   _GAMS_UTILITY_ROS_GAMS_PARSER_
#define   _GAMS_UTILITY_ROS_GAMS_PARSER_

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "madara/knowledge/containers/NativeIntegerVector.h"
#include "madara/knowledge/containers/Double.h"
#include "madara/knowledge/containers/String.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/StringVector.h"
#include "gams/pose/ReferenceFrame.h"
#include "gams/pose/Pose.h"
#include "gams/pose/Quaternion.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wreorder"
#endif

#include "ros/ros.h"
#include "ros/serialization.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include "tf2_ros/transform_broadcaster.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace logger = madara::logger;
namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;
namespace global_ros = ros;

namespace gams
{
  namespace utility
  {
    namespace ros
    {
      class GamsParser
      {
        public:
          GamsParser (knowledge::KnowledgeBase * kb);
          void parse_message (std::string container_name,
            std::string topic_name, std::string topic_type);
          
          //known types
          void publish_odometry (std::string container_name,
            std::string topic_name);

          template <size_t N>
          void parse_float64_array (boost::array<double, N> *array,
            containers::NativeDoubleVector *origin);
          void parse_float64_array (std::vector<float> *array,
            containers::NativeDoubleVector *origin);
          void parse_vector3 (
            geometry_msgs::Vector3 *vec,
            containers::NativeDoubleVector *origin);
          void parse_quaternion (geometry_msgs::Quaternion *quat,
            containers::NativeDoubleVector *origin);
          void publish_imu(std::string container_name, std::string topic_name);
          void publish_laserscan (std::string container_name,
            std::string topic_name);
          void publish_pointcloud2 (std::string container_name,
            std::string topic_name);
          void parse_pose (geometry_msgs::Pose *pose,
            containers::NativeDoubleVector *origin);
          void publish_compressed_image (std::string container_name,
            std::string topic_name);
          void publish_range ( std::string container_name,
            std::string topic_name);
          void publish_fluidpressure (std::string container_name,
            std::string topic_name);

          template <class T>
          void parse_int_array (std::vector<T> *array,
            containers::NativeIntegerVector *origin);
          template <size_t N>
          void parse_int_array (boost::array<int, N> *array,
            containers::NativeIntegerVector *origin);
          
          int publish_transform (std::string frame_id,
            std::string frame_prefix);
        protected:
          // The knowledgebase
          knowledge::KnowledgeBase * knowledge_;
          knowledge::EvalSettings eval_settings_;
          global_ros::NodeHandle node_;
      };
    }
  }
}
#endif
