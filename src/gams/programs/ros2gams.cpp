
/**
 * @file ros2madara.cpp
 * @author Jakob Auer <jakob.auer@gmail.com>
 *
 * This file converts ROSBAG files to Madara checkpoints
 **/
#include <iostream>
#include <string>


#include "madara/logger/GlobalLogger.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/NativeDoubleVector.h"
//#include "gams/pose/Quaternion.h"



#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>


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


void parse_odometry (nav_msgs::Odometry * odom, knowledge::KnowledgeBase * knowledge, std::string agent_prefix);
void parse_imu (sensor_msgs::Imu * imu, knowledge::KnowledgeBase * knowledge, std::string agent_prefix);
void parse_quaternion (geometry_msgs::Quaternion *quat, containers::NativeDoubleVector *orientation);
void parse_point (geometry_msgs::Point *point_msg, containers::NativeDoubleVector *point);



void save_checkpoint (knowledge::KnowledgeBase *knowledge,
    knowledge::CheckpointSettings *settings, std::string meta_prefix="meta");
std::string get_agent_var_prefix(std::string ros_topic_name);


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
	    else if (arg1 == "-c" || arg1 == "--checkpoint-prefix")
	    {
	      checkpoint_prefix = argv[i + 1];
	      i++;
	    }
	    else if (arg1 == "-rp" || arg1 == "--ros-robot-prefix")
	    {
	      //rosbag_robot_prefix = argv[i + 1];
	      i++;
	    }
	    else if (arg1 == "-h" || arg1 == "--help")
	    {
	      cout << "\nProgram summary for ros2madara [options] [Logic]:\n\n" \
	        "Converts rosbag files to madara checkpoints.\n\noptions:\n" \
	        "  [-r|--rosbag]                   Path to the rosbag file\n" \
	        "  [-rp|--ros-robot-prefix]        Topic prefix of each robot (default: '/robot_')\n" \
	        "  [-c|--checkpoint-prefix]        Prefix for the madara checkpoint files (default: 'checkpoint_')\n";
	      exit(0);
	    }
	}
}

int main (int argc, char ** argv)
{
	handle_arguments(argc, argv);
	knowledge::KnowledgeBase kb;
    knowledge::CheckpointSettings settings;



	// Select the ROS topics
	std::vector<std::string> selected_topics;
	selected_topics.push_back("imu/data");
	selected_topics.push_back("gps/map");

	if (rosbag_path == "")
		exit(0);

	// Read the bag
	rosbag::Bag bag;
  	bag.open(rosbag_path, rosbag::bagmode::Read);


  	// Query the bag's stats
  	rosbag::View full_view;
  	full_view.addQuery(bag);
  	std::cout << "Size of the bagfile: " << full_view.size() << "\n";
  	std::cout << "Topics in the bagfile: \n";
	for (const rosbag::ConnectionInfo* c: full_view.getConnections())
	{
    	std::cout << c->topic << "(" << c->datatype << ")\n";
    }

    // Filter topics
    rosbag::View view;
    rosbag::TopicQuery topics (selected_topics);
    view.addQuery (bag, topics);

    // Iterate the messages
  	std::cout << "\n\nMessageInstances in the bagfile: \n";
  	settings.initial_lamport_clock = 0;
  	settings.last_lamport_clock = 0;
    for (const rosbag::MessageInstance m: view)
    {
	    std::string topic   = m.getTopic();
	    ros::Time time = m.getTime();
	    std::string callerid       = m.getCallerId();
	    std::cout << time.toSec() << ": " << topic << " from " << callerid;
	    std::string agent_prefix = get_agent_var_prefix(topic);
	    if (m.isType<nav_msgs::Odometry>())
	    {
	    	parse_odometry(m.instantiate<nav_msgs::Odometry>().get(), &kb, agent_prefix);
	    }
	    else if (m.isType<sensor_msgs::Imu>())
	    {
	    	parse_imu(m.instantiate<sensor_msgs::Imu>().get(), &kb, agent_prefix);
	    }
	    else
	    {
	    	cout << topic << ": Type not supported\n";
	    	continue;
	    }

	    kb.print();
	    settings.filename = checkpoint_prefix + std::to_string(settings.last_lamport_clock) + ".kb";

	    //Set time settings
	    if (settings.last_lamport_clock == 0)
	    	// this is the first message
	    	settings.initial_timestamp = time.toSec();
	    settings.last_timestamp = time.toSec();
     	settings.last_lamport_clock += 1;

	    save_checkpoint(&kb, &settings);

	    std::cout << "\n-------- \n" << std::flush;
    }
}


/**
* Parses a ROS Odometry Message into two Madara Containers. One for the location and a
* second one for the orientation.
* @param  odom   		the nav_msgs::Odometry message
* @param  knowledge 	Knowledbase
* @param  agent_prefix  variable namespace (e.g. "agent.0"s)
**/
void parse_odometry (nav_msgs::Odometry * odom, knowledge::KnowledgeBase * knowledge, std::string agent_prefix)
{

	containers::NativeDoubleVector location(agent_prefix + ".location", *knowledge, 3);
	containers::NativeDoubleVector orientation(agent_prefix + ".orientation", *knowledge, 3);
	parse_quaternion(&odom->pose.pose.orientation, &orientation);
	parse_point(&odom->pose.pose.position, &location);
	// Todo: Parse Twist
}

/**
* Parses a ROS IMU Message
* @param  imu   		the sensor_msgs::Imu message
* @param  knowledge 	Knowledgbase
* @param  agent_prefix  variable namespace (e.g. "agent.0"s)
**/
void parse_imu (sensor_msgs::Imu *imu, knowledge::KnowledgeBase * knowledge, std::string agent_prefix)
{
	containers::NativeDoubleVector orientation(agent_prefix + ".imu.readings", *knowledge, 3);
	parse_quaternion(&imu->orientation, &orientation);

	// Todo: Parse angular and linear velocity. both geometry_msgs/Vector3
}

/**
* Parses a ROS geometry_msgs::Point message
* @param  point_msg		the geometry_msgs::Point message
* @param  point 	 	the container
**/
void parse_point (geometry_msgs::Point *point_msg, containers::NativeDoubleVector *point)
{
	point->set(0, point_msg->x);
	point->set(1, point_msg->y);
	point->set(2, point_msg->z);
}

/**
* Parses a ROS geometry_msgs::Quaternion message
* @param  quat			the geometry_msgs::Quaternion message
* @param  orientatiom 	the container 
**/
void parse_quaternion (geometry_msgs::Quaternion *quat, containers::NativeDoubleVector *orientation)
{
	std::cout << "o:[" << quat->x << ", " << quat->y << ", " << quat->z << ", " << quat->w <<"]\n";
	// Todo: Use gams Quaternion instead
	tf::Quaternion tfquat(quat->x, quat->y, quat->z, quat->w);
    tf::Matrix3x3 m(tfquat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

	orientation->set(0, (roll * 180) / M_PI);
	orientation->set(1, (pitch * 180) / M_PI);
	orientation->set(2, (yaw * 180) / M_PI);
}


/**
* Saves the KnowlegeBase with given CheckpointSettings to a file.
* The path ,the timestamps, and lamport_clocks habe to be set in advance.
* The values are stored with a meta_prefix.
**/
void save_checkpoint (knowledge::KnowledgeBase * knowledge,
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
	
	knowledge->save_as_karl(*settings);
}


/**
* Parses the agent id based on the rostopic name
**/
std::string get_agent_var_prefix(std::string ros_topic_name)
{
	// Convert ros_topic_name to lower case
	std::transform(ros_topic_name.begin(), ros_topic_name.end(), ros_topic_name.begin(), ::tolower);
	std::string name = ros_topic_name.substr(1);
	std::string rosbag_robot_prefix = "robot_";
 	
	if(name.find(rosbag_robot_prefix) == 0)
	{
		int namespace_end = name.find("/");
		//cut the prefix
		std::string prefix = name.substr(rosbag_robot_prefix.length(),
			namespace_end-rosbag_robot_prefix.length());
		return "agent." + prefix;
	}
	else
	{
		return "agent.0";
	}
}