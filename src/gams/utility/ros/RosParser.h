/**
 * @file TopicParser.cpp
 * @author Jakob Auer <jakob.auer@gmail.com>
 *
 * This file contains parsing functionality for ROS topics
 **/

#ifndef   _GAMS_UTILITY_ROS_ROS_PARSER_
#define   _GAMS_UTILITY_ROS_ROS_PARSER_

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "madara/knowledge/containers/NativeIntegerVector.h"
#include "madara/knowledge/containers/Double.h"
#include "madara/knowledge/Any.h"
#include "madara/knowledge/CapnObject.h"
#include "madara/knowledge/containers/String.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/StringVector.h"
#include "madara/knowledge/containers/CircularBuffer.h"
#include "madara/utility/SimTime.h"
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

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <string>
#include <regex>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "capnp/serialize.h"
#include "capnp/schema-loader.h"
#include "capnp/schema-parser.h"
#include "kj/io.h"

namespace global_ros = ros;
namespace logger = madara::logger;
namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

namespace gams
{
  namespace utility
  {
    namespace ros
    {
      static std::string cleanCapnpSchemaName(const std::string& full_name)
      {
        std::regex re("[^\\/:]+");
        std::sregex_token_iterator begin(full_name.begin(), full_name.end(), re), end;
        std::vector<std::string> tokens;
        std::copy(begin, end, std::back_inserter(tokens));
        return tokens.back();
      }
      class RosParser
      {
        public:

          const std::string IGNORE_MARKER = "_IGNORE_";

          RosParser (knowledge::KnowledgeBase * kb, std::string world_frame,
            std::string base_frame,
            std::map<std::string, std::string> capnp_types,
            std::map<std::string, int> circular_containers=std::map<std::string, int>(),
            knowledge::EvalSettings eval_settings=knowledge::EvalSettings(),
            std::string frame_prefix=gams::pose::ReferenceFrame::default_prefix());
          void parse_message (const rosbag::MessageInstance m,
            std::string container_name);
          void parse_message (const topic_tools::ShapeShifter::ConstPtr& m,
            std::string container_name);

          // Parsing for unknown types
          void registerMessageDefinition(std::string topic_name,
            RosIntrospection::ROSType type, std::string definition);
          void registerRenamingRules(RosIntrospection::ROSType type,
            std::vector<RosIntrospection::SubstitutionRule> rules);
          void parse_unknown (const rosbag::MessageInstance m,
            std::string container_name);
          

          /**
           * Registers renaming rules for ros types
           * @param name_substitution_map   map<rostype, map<name, newname>>
           **/
          void register_rename_rules( std::map<std::string, std::map<std::string, std::string>> name_substitution_map);

          /**
           * Parse defined Any types
           * @param  m               rosbag::MessageInstance to parse
           * @param  container_name  name of the madara variable
           **/
          void parse_any (const rosbag::MessageInstance & m,
            std::string container_name);


          /**
           * Parse defined Any types
           * @param  topic           string with the topic name
           * @param  m               topic_tools::ShapeShifter to parse
           * @param  container_name  name of the madara variable
           **/
          void parse_any (std::string topic, const topic_tools::ShapeShifter & m,
            std::string container_name);
          
          /**
           * Parse defined Any types
           * @param  datatype        string with the ros datatype name
           * @param  topic_name      string with the topic name
           * @param  parse_buffer    byte vector with the message
           * @param  container_name  name of the madara variable
           **/
          void parse_any ( std::string datatype, std::string topic_name,
            std::vector<uint8_t> & parser_buffer,
            std::string container_name);

          /**
           * Loads a Capnproto schema from disk for later usage
           * Only Capnproto binary schemas are supported
           * @param  path        string with the path to the schema file
           **/
          void load_capn_schema(std::string path);
          
          //known types
          void parse_odometry (nav_msgs::Odometry * odom,
            std::string container_name);
          void parse_imu (sensor_msgs::Imu * imu,
            std::string container_name);
          void parse_laserscan (sensor_msgs::LaserScan * laser,
            std::string container_name);
          void parse_quaternion (geometry_msgs::Quaternion *quat,
            containers::NativeDoubleVector *orientation);
          void parse_point (geometry_msgs::Point *point_msg,
            containers::NativeDoubleVector *point);
          void parse_twist (geometry_msgs::Twist *twist,
            std::string container_name);
          void parse_vector3 (geometry_msgs::Vector3 *vec,
            containers::NativeDoubleVector *target);
          void parse_pose (geometry_msgs::Pose *pose,
            std::string container_name);
          void parse_compressed_image (sensor_msgs::CompressedImage * img,
            std::string container_name);
          void parse_pointcloud2 (sensor_msgs::PointCloud2 * pointcloud,
            std::string container_name);
          void parse_range (sensor_msgs::Range * range,
            std::string container_name);
          void parse_tf_message (tf2_msgs::TFMessage * tf);
          void parse_fluidpressure (sensor_msgs::FluidPressure * press,
            std::string container_name);


          template <size_t N>
          void parse_float64_array (boost::array<double, N> *array,
            containers::NativeDoubleVector *target);
          void parse_float64_array (std::vector<float> *array,
            containers::NativeDoubleVector *target);


          template <size_t N>
          void parse_int_array (boost::array<int, N> *array,
            containers::NativeIntegerVector *target);
          template <class T>
          void parse_int_array (std::vector<T> *array,
            containers::NativeIntegerVector *target);

          /**
           * Prints the available schema names to std out
           */
          void print_schemas();

        protected:
          // ros introspection parser for unknown types
          RosIntrospection::Parser parser_;

          //to reduce the amount of memory allocations these maps are reused for
          //all unknown topic types which are not parsed directly
          std::map<std::string, RosIntrospection::FlatMessage> flat_containers_;
          std::map<std::string, RosIntrospection::RenamedValues> renamed_vectors_;
          std::vector<uint8_t> parser_buffer_;
          std::map<std::string, unsigned int> ros_array_sizes_;

          // Transform tree frame names
          std::string world_frame_;
          std::string base_frame_;

          // Capnproto
          std::map<std::string, std::string> capnp_types_;
          std::map<std::string, capnp::Schema> schemas_;
          capnp::SchemaLoader capnp_loader_;



          // The knowledgebase
          knowledge::KnowledgeBase * knowledge_;
          knowledge::EvalSettings eval_settings_;
          std::string frame_prefix_;

          // Map for circular buffer producers (maps from var name to the producer)
          std::map<std::string, madara::knowledge::containers::CircularBuffer>
            circular_producers_;
          std::map<std::string, int> circular_container_stats_;

          //Name substitution
          std::map<std::string, std::map<std::string, std::string>>
            name_substitution_map_;




          capnp::DynamicStruct::Builder get_dyn_capnp_struct(
            capnp::DynamicStruct::Builder builder, std::string name);

          template <class T>
          void set_dyn_capnp_value(capnp::DynamicStruct::Builder builder,
            std::string name, T val, unsigned int array_size);
          
          template <class T>
          unsigned int get_array_size(std::string var_name,
            std::vector<std::pair<std::string, T>> array);

          /**
           * Substitutes the names of topic type members based on the registered
           * rules
           **/ 
          std::string substitute_name(std::string type, std::string name);

          /*
          Sets the current time to the ros header time if the simtime feature is
          activated.
          */
          void set_sim_time(global_ros::Time rostime);

      };
      std::string ros_to_gams_name (std::string ros_topic_name);
    }
  }
}


#endif // _GAMS_ROS_TOPIC_PARSER_