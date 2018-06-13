
/**
 * @file ros2madara.cpp
 * @author Jakob Auer <jakob.auer@gmail.com>
 *
 * This file converts ROSBAG files to Madara checkpoints
 **/
#include <iostream>
#include <string>
#include <fstream>

#include "madara/logger/GlobalLogger.h"
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
#include "gams/utility/ros/RosParser.h"



// ROS includes
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

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif


#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/foreach.hpp>

namespace logger = madara::logger;
namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

typedef knowledge::KnowledgeRecord::Integer  Integer;


// filename of the rosbag file
std::string rosbag_path = "";

// prefix of the agents in the rosbag
//std::string rosbag_robot_prefix = "robot_";

// prefix for the madara checkoints
std::string checkpoint_prefix = "checkpoint_";

// path to the filter definition file
std::string map_file = "";

// save as a karl or binary file
bool save_as_karl = true;

// the world and the base frame of the robot
std::string base_frame = "";
std::string world_frame = "";

//checkpoint frequency
int checkpoint_frequency = 0;

// Differential checkpointing
bool differential_checkpoints = false;




int save_checkpoint (knowledge::KnowledgeBase *knowledge,
  knowledge::CheckpointSettings *settings, std::string meta_prefix="meta");
std::string get_agent_var_prefix (std::string ros_topic_name);
std::string gams::utility::ros::ros_to_gams_name (std::string topic_name);


// handle command line arguments
void handle_arguments (int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1 (argv[i]);

    if (arg1 == "-r" || arg1 == "--rosbag")
    {
      rosbag_path = argv[i + 1];
      i++;
    }
    else if (arg1 == "-scp" || arg1 == "--save-checkpoint-prefix")
    {
      checkpoint_prefix = argv[i + 1];
      i++;
    }
    else if (arg1 == "-m" || arg1 == "--map-file")
    {
      map_file = argv[i + 1];
      i++;
    }
    else if (arg1 == "-rp" || arg1 == "--ros-robot-prefix")
    {
      //rosbag_robot_prefix = argv[i + 1];
      i++;
    }
    else if (arg1 == "-sb" || arg1 == "--save-binary")
    {
      save_as_karl = false;
    }
    else if (arg1 == "-d" || arg1 == "--differential")
    {
      differential_checkpoints = true;
    }
    else if (arg1 == "-y" || arg1 == "--frequency")
    {
      checkpoint_frequency = std::stoi(argv[i + 1]);
      i++;
    }
    else if (arg1 == "-h" || arg1 == "--help")
    {
      std::cout << "\nProgram summary for ros2gams [options] [Logic]:\n\n" \
      "Converts rosbag files to madara checkpoints.\n\noptions:\n" \
      "  [-r|--rosbag]                        Path to the rosbag file\n" \
      "  [-rp|--ros-robot-prefix prfx]        Topic prefix of each robot\n" \
      "                                       (default: '/robot_')\n" \
      "  [-scp|--save-checkpoint-prefix prfx] filname prefix to save\n"\
      "                                       for checkpoint\n" \
      "  [-sb|--save-binary]                  save the resulting knowledge \n"\
      "                                       base as a binary checkpoint\n" \
      "  [-m|--map-file file]                 File with filter information\n" \
      "  [-y|--frequency hz]                  Checkpoint frequency\n" \
      "                                       (default:checkpoint with each\n" \
      "                                        message in the bagfile)\n" \
      "  [-d|--differential]                  differential checkpoints\n" \
      "                                       only for binary checkpoints\n";
      exit (0);
    }
  }
}

#ifndef TEST_ROS2GAMS
int main (int argc, char ** argv)
{
  handle_arguments (argc, argv);
  knowledge::KnowledgeBase kb;
  knowledge::KnowledgeBase manifest;

  knowledge::CheckpointSettings settings;

  if (rosbag_path == "")
    exit (0);

  // Read the bag
  rosbag::Bag bag;
  bag.open (rosbag_path, rosbag::bagmode::Read);

  rosbag::View view;
  std::map<std::string,std::string> topic_map;

  // Use mapfile to filter topics
  if (map_file != "")
  {
    std::vector<std::string> selected_topics;
    std::string line;
    std::ifstream myfile (map_file);
    if (myfile.is_open ())
    {
      while ( getline (myfile, line) )
      {
        int ros_topic_end = line.find (" ");
        //split the line in topic name and madara var name
        std::string topic_name = line.substr (0, ros_topic_end);
        std::string var_name = line.substr (ros_topic_end+1);
        if (topic_name == "base_frame:")
        {
          // definition of the base_frame
          base_frame = var_name;
              std::replace ( base_frame.begin (), base_frame.end (), '/', '_');

          std::cout << "Base frame: " << base_frame << std::endl;
        }
        else if (topic_name == "world_frame:")
        {
          // definition of the world_frame
          world_frame = var_name;
          std::replace (
            world_frame.begin (), world_frame.end (), '/', '_');

          std::cout << "World frame: " << world_frame << std::endl;
        }
        else
        {
          selected_topics.push_back (topic_name);
          topic_map[topic_name] = var_name;
        }
      }
      myfile.close ();
    }
    rosbag::TopicQuery topics (selected_topics);
    view.addQuery (bag, topics);
  }
  else
  {
    view.addQuery (bag);
  }

  // Query the bag's stats
  std::cout   << "Size of the selected topics: " << view.size () << "\n";
  std::cout   << "Selected topics in the bagfile: \n";
  int count = view.size ();
  int id_digit_count = 0;
  do {
    count /= 10;
    id_digit_count++;
  }
  while (count != 0);

  // Log the queried topic names to the console and the manifest knowledge
  containers::StringVector ros_topic_names("ros_topic_names", manifest,
    view.getConnections().size());
  containers::StringVector gams_names("gams_names", manifest,
    view.getConnections().size());
  int i = 0;
  for (const rosbag::ConnectionInfo* c: view.getConnections ())
  {
    std::cout << "    " << c->topic << " (" << c->datatype << ")\n";
    ros_topic_names.set(i, c->topic);
    std::map<std::string, std::string>::iterator it = topic_map.find (c->topic);
    if (it != topic_map.end ())
      gams_names.set(i, it->second);
    ++i;
  }

  //Prepare the parser to be able to introspect unknown types

  gams::utility::ros::RosParser parser(&kb, world_frame, base_frame);
  for (const rosbag::ConnectionInfo* connection: view.getConnections () )
  {
    const std::string  topic_name =  connection->topic;
    const std::string  datatype   =  connection->datatype;
    const std::string  definition =  connection->msg_def;
    // register the type using the topic_name as identifier.
    parser.registerMessageDefinition (topic_name,
      RosIntrospection::ROSType (datatype), definition);
  }

  // Iterate the messages
  settings.initial_lamport_clock = 0;
  settings.last_lamport_clock = 0;
  settings.override_lamport = true;
  settings.override_timestamp = true;
  settings.buffer_size = 10240000;

  uint64_t last_checkpoint_timestamp = 0;

  // checkpoint intervall
  uint64_t checkpoint_intervall = 1000000000;
  int checkpoint_id = 0;
  if ( checkpoint_frequency > 0 )
  {
    checkpoint_intervall = checkpoint_intervall / checkpoint_frequency;
    std::cout << "\nCreating a checkpoint each " <<
      (checkpoint_intervall / 1000 / 1000) << " milliseconds." << std::endl;
  }

  // Iterate through all topics in the bagfile
  std::cout << "Converting...\n";
  for (const rosbag::MessageInstance m: view)
  {
    std::string topic = m.getTopic ();
    ros::Time time = m.getTime ();
    std::string callerid = m.getCallerId ();

    //Check if topic is in the topic mapping
    std::map<std::string, std::string>::iterator it = topic_map.find (topic);
    std::string container_name;

    if (it != topic_map.end ())
    {
      container_name = it->second;
    }
    else
    {
      container_name = get_agent_var_prefix (topic) + "." +
        gams::utility::ros::ros_to_gams_name (topic);
    }
    parser.parse_message (m, container_name);

    //kb.print ();
    std::stringstream id_ss;
    id_ss << std::setw (id_digit_count) << std::setfill ('0') <<
      checkpoint_id;
    std::string id_str = id_ss.str ();
    settings.filename = checkpoint_prefix + "_" + id_str + ".kb";

    uint64_t stamp = time.sec;
    stamp = stamp * 1000000000 + time.nsec;
    settings.last_timestamp = stamp;
    //Set time settings
    if (settings.last_lamport_clock == 0)
    {
      // this is the first message
      settings.initial_timestamp = settings.last_timestamp;
    }
    settings.last_lamport_clock += 1;

    // Save checkpoint
    int ret = 0;
    if ( checkpoint_frequency == 0 )
    {
      // Save checkpoint with each message
      ret = save_checkpoint (&kb, &settings);
      checkpoint_id++;
    }
    else if (last_checkpoint_timestamp + checkpoint_intervall < stamp ||
      last_checkpoint_timestamp == 0)
    {
      // Save checkpoint with a given frequency
      ret = save_checkpoint (&kb, &settings);
      last_checkpoint_timestamp = stamp;
      checkpoint_id++;
    }
    // Check if the checkpoint was written correctly
    if ( ret == -1 )
    {
      std::cout << "Failed to write " << settings.filename << "!" << std::endl;
      exit(-1);
    }
  }
  
  // Storing stats in the manifest knowledge
  manifest.set ("last_timestamp", (Integer) settings.last_timestamp);
  manifest.set ("initial_timestamp", (Integer) settings.initial_timestamp);
  manifest.set ("duration_ns", (Integer)
    ( settings.last_timestamp - settings.initial_timestamp));
  manifest.set ("last_lamport_clock", (Integer) settings.last_lamport_clock-1);
  manifest.set ("initial_lamport_clock",
    (Integer) settings.initial_lamport_clock);
  manifest.set ("last_checkpoint_id", checkpoint_id-1);
  manifest.save_as_karl(checkpoint_prefix + "_manifest.kb");
}
#endif


/**
* Saves the KnowlegeBase with given CheckpointSettings to a file.
* The path ,the timestamps, and lamport_clocks habe to be set in advance.
* The values are stored with a meta_prefix.
**/
int save_checkpoint (knowledge::KnowledgeBase * knowledge,
    knowledge::CheckpointSettings * settings, std::string meta_prefix)
{
  /*knowledge->set (meta_prefix + ".originator",
        settings->originator);
  knowledge->set (meta_prefix + ".version",
  settings->version);
  knowledge->set (meta_prefix + ".current_timestamp",
    (Integer) time (NULL));*/
  knowledge->set (meta_prefix + ".last_timestamp",
    (Integer) settings->last_timestamp);
  knowledge->set (meta_prefix + ".initial_timestamp",
    (Integer) settings->initial_timestamp);
  knowledge->set (meta_prefix + ".last_lamport_clock",
    (Integer) settings->last_lamport_clock);
  knowledge->set (meta_prefix + ".initial_lamport_clock",
    (Integer) settings->initial_lamport_clock);

  int ret = 0;
  if ( save_as_karl )
  {
    ret = knowledge->save_as_karl (*settings);
  }
  else
  {
    if ( differential_checkpoints == true )
    {
      ret = knowledge->save_checkpoint (*settings);
    }
    else
    {
      ret= knowledge->save_context (*settings);
    }
  }
  return ret;
}


/**
* Parses the agent id based on the rostopic name
**/
std::string get_agent_var_prefix (std::string ros_topic_name)
{
  // Convert ros_topic_name to lower case
  std::transform (ros_topic_name.begin (),
    ros_topic_name.end (), ros_topic_name.begin (), ::tolower);
  std::string name = ros_topic_name.substr (1);
  std::string rosbag_robot_prefix = "robot_";
   
  if (name.find (rosbag_robot_prefix) == 0)
  {
    int namespace_end = name.find ("/");
    //cut the prefix
    std::string prefix = name.substr (rosbag_robot_prefix.length (),
      namespace_end-rosbag_robot_prefix.length ());
    return "agent." + prefix;
  }
  else
  {
    return "agent.0";
  }
}


