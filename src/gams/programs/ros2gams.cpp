
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
#include "madara/knowledge/containers/Double.h"
//#include "gams/pose/ReferenceFrame.h"



#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>


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


void parse_odometry (nav_msgs::Odometry * odom, knowledge::KnowledgeBase * knowledge, std::string container_name);
void parse_imu (sensor_msgs::Imu * imu, knowledge::KnowledgeBase * knowledge, std::string container_name);
void parse_laserscan (sensor_msgs::LaserScan * laser, knowledge::KnowledgeBase * knowledge, std::string container_name);
void parse_quaternion (geometry_msgs::Quaternion *quat, containers::NativeDoubleVector *orientation);
void parse_point (geometry_msgs::Point *point_msg, containers::NativeDoubleVector *point);
void parse_twist (geometry_msgs::Twist *twist, knowledge::KnowledgeBase * knowledge, std::string container_name);
void parse_vector3 (geometry_msgs::Vector3 *vec, containers::NativeDoubleVector *target);
void parse_pose (geometry_msgs::Pose *pose, knowledge::KnowledgeBase * knowledge, std::string container_name);


template <size_t N>
void parse_float64_array(boost::array<double, N> *array, containers::NativeDoubleVector *target);
void parse_float64_array(std::vector<float> *array, containers::NativeDoubleVector *target);



void save_checkpoint (knowledge::KnowledgeBase *knowledge,
    knowledge::CheckpointSettings *settings, std::string meta_prefix="meta");
std::string get_agent_var_prefix(std::string ros_topic_name);
std::string ros_to_gams_name(std::string topic_name);


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
	    else if (arg1 == "-h" || arg1 == "--help")
	    {
	      cout << "\nProgram summary for ros2gams [options] [Logic]:\n\n" \
	        "Converts rosbag files to madara checkpoints.\n\noptions:\n" \
	        "  [-r|--rosbag]                        Path to the rosbag file\n" \
	        "  [-rp|--ros-robot-prefix prfx]        Topic prefix of each robot (default: '/robot_')\n" \
	        "  [-scp|--save-checkpoint-prefix prfx] prefix of knowledge to save in checkpoint\n" \
        	"  [-sb|--save-binary]                  save the resulting knowledge base as a\n" \
        	"                                       binary checkpoint\n" \
        	"  [-m|--map-file file]                 File with filter information";
        	exit(0);
	    }
	}
}

int main (int argc, char ** argv)
{
	handle_arguments(argc, argv);
	knowledge::KnowledgeBase kb;
    knowledge::CheckpointSettings settings;


	if (rosbag_path == "")
		exit(0);

	// Read the bag
	rosbag::Bag bag;
  	bag.open(rosbag_path, rosbag::bagmode::Read);




	rosbag::View view;
	std::map<std::string,std::string> topic_map;
    // Use mapfile to filter topics
    if (map_file != ""){
    	std::vector<std::string> selected_topics;
    	std::string line;
		ifstream myfile (map_file);
		if (myfile.is_open())
		{
			while ( getline (myfile, line) )
			{
				int ros_topic_end = line.find(" ");
				//split the line in topic name and madara var name
				std::string topic_name = line.substr(0, ros_topic_end);
				std::string var_name = line.substr(ros_topic_end+1);
				selected_topics.push_back(topic_name);
				topic_map[topic_name] = var_name;
			}
			myfile.close();
		}
	    rosbag::TopicQuery topics (selected_topics);
	    view.addQuery (bag, topics);
    }
    else
    {
    	view.addQuery(bag);
    }

  	// Query the bag's stats
    std::cout << "Size of the bagfile: " << view.size() << "\n";
  	std::cout << "Topics in the bagfile: \n";
	for (const rosbag::ConnectionInfo* c: view.getConnections())
	{
    	std::cout << c->topic << "(" << c->datatype << ")\n";
    }

    // Iterate the messages
  	//std::cout << "\n\nMessageInstances in the bagfile: \n";
  	settings.initial_lamport_clock = 0;
  	settings.last_lamport_clock = 0;
  	cout << "Converting...\n";
    for (const rosbag::MessageInstance m: view)
    {
	    std::string topic   = m.getTopic();
	    ros::Time time = m.getTime();
	    std::string callerid       = m.getCallerId();
	    //std::cout << time.toSec() << ": " << topic << " from " << callerid << "\n";

	    //Check if topic is in the topic mapping
	    std::map<std::string, std::string>::iterator it = topic_map.find(topic);
		std::string container_name;
	    if (it != topic_map.end())
	    	container_name = it->second;
	    else
	    	container_name = get_agent_var_prefix(topic) + "." + ros_to_gams_name(topic);

	    if (m.isType<nav_msgs::Odometry>())
	    {
	    	parse_odometry(m.instantiate<nav_msgs::Odometry>().get(), &kb, container_name);
	    }
	    else if (m.isType<sensor_msgs::Imu>())
	    {
	    	parse_imu(m.instantiate<sensor_msgs::Imu>().get(), &kb, container_name);
	    }
	    else if (m.isType<sensor_msgs::LaserScan>())
	    {
	    	parse_laserscan(m.instantiate<sensor_msgs::LaserScan>().get(), &kb, container_name);
	    }
	    else if (m.isType<geometry_msgs::Pose>())
	    {
	    	parse_pose(m.instantiate<geometry_msgs::Pose>().get(), &kb, container_name);
	    }
	    else if (m.isType<geometry_msgs::PoseStamped>())
	    {
	    	parse_pose(&m.instantiate<geometry_msgs::PoseStamped>().get()->pose, &kb, container_name);
	    }
	    else
	    {
	    	//cout << topic << ": Type not supported\n";
	    	continue;
	    }

	    //kb.print();
	    settings.filename = checkpoint_prefix + "_" + std::to_string(settings.last_lamport_clock) + ".kb";

	    //Set time settings
	    if (settings.last_lamport_clock == 0)
	    	// this is the first message
	    	settings.initial_timestamp = time.toSec();
	    settings.last_timestamp = time.toSec();
     	settings.last_lamport_clock += 1;

	    save_checkpoint(&kb, &settings);

	    //if (settings.last_lamport_clock > 100)
	    //	exit(0);
    }
    std::cout << "Done!\n";
	//std::cout << "Converted " + settings.last_lamport_clock << " messages.\n" << std::flush;
}


/**
* Parses a ROS Odometry Message into two Madara Containers. One for the location and a
* second one for the orientation.
* @param  odom   		the nav_msgs::Odometry message
* @param  knowledge 	Knowledbase
**/
void parse_odometry (nav_msgs::Odometry * odom, knowledge::KnowledgeBase * knowledge, std::string container_name)
{

	containers::NativeDoubleVector odom_covariance(container_name + ".pose.covariance", *knowledge, 36);
	containers::NativeDoubleVector twist_covariance(container_name + ".pose.covariance", *knowledge, 36);

	parse_pose(&odom->pose.pose, knowledge, container_name + ".pose");
	parse_float64_array(&odom->pose.covariance, &odom_covariance);
	parse_float64_array(&odom->twist.covariance, &twist_covariance);
	parse_twist(&odom->twist.twist, knowledge, container_name + ".twist");
}

/**
* Parses a ROS IMU Message
* @param  imu   		the sensor_msgs::Imu message
* @param  knowledge 	Knowledgbase
**/
void parse_imu (sensor_msgs::Imu *imu, knowledge::KnowledgeBase * knowledge, std::string container_name)
{
	containers::NativeDoubleVector orientation(container_name + ".orientation", *knowledge, 3);
	containers::NativeDoubleVector angular_velocity(container_name + ".angular_velocity", *knowledge, 3);
	containers::NativeDoubleVector linear_acceleration(container_name + ".linear_acceleration", *knowledge, 3);
	containers::NativeDoubleVector orientation_covariance(container_name + ".orientation_covariance", *knowledge, 9);
	containers::NativeDoubleVector angular_velocity_covariance(container_name + ".angular_velocity_covariance", *knowledge, 9);
	containers::NativeDoubleVector linear_acceleration_covariance(container_name + ".linear_acceleration_covariance", *knowledge, 9);


	parse_quaternion(&imu->orientation, &orientation);
	parse_vector3(&imu->angular_velocity, &angular_velocity);
	parse_vector3(&imu->linear_acceleration, &linear_acceleration);
	parse_float64_array(&imu->orientation_covariance, &orientation_covariance);
	parse_float64_array(&imu->angular_velocity_covariance, &angular_velocity_covariance);
	parse_float64_array(&imu->linear_acceleration_covariance, &linear_acceleration_covariance);
}

/**
* Parses a ROS LaserScan Message
* @param  laser   			the sensor_msgs::LaserScan message
* @param  knowledge 		Knowledgbase
* @param  container_name  	container namespace (e.g. "laser")
**/
void parse_laserscan (sensor_msgs::LaserScan * laser, knowledge::KnowledgeBase * knowledge, std::string container_name)
{
	//Ranges
	int ranges_size = laser->ranges.size();
	containers::NativeDoubleVector ranges(container_name + ".ranges", *knowledge, ranges_size);
	parse_float64_array(&laser->ranges, &ranges);
	//Intensities
	int intensities_size = laser->intensities.size();
	containers::NativeDoubleVector intensities(container_name + ".intensities", *knowledge, intensities_size);
	parse_float64_array(&laser->intensities, &intensities);
	//Parameters
	containers::Double angle_min(container_name + ".angle_min", *knowledge);
	angle_min = laser->angle_min;
	containers::Double angle_max(container_name + ".angle_max", *knowledge);
	angle_max = laser->angle_max;
	containers::Double angle_increment(container_name + ".angle_increment", *knowledge);
	angle_increment = laser->angle_increment;
	containers::Double time_increment(container_name + ".time_increment", *knowledge);
	time_increment = laser->time_increment;
	containers::Double scan_time(container_name + ".scan_time", *knowledge);
	scan_time = laser->scan_time;
	containers::Double range_min(container_name + ".range_min", *knowledge);
	range_min = laser->range_min;
	containers::Double range_max(container_name + ".range_max", *knowledge);
	range_max = laser->range_max;

}

/**
* Parses a ROS Twist Message
* @param  twist   			the geometry_msgs::Twist message
* @param  knowledge 		Knowledgbase
* @param  container_name  	container namespace (e.g. "laser")
**/
void parse_twist (geometry_msgs::Twist *twist, knowledge::KnowledgeBase * knowledge, std::string container_name)
{
	containers::NativeDoubleVector linear(container_name + ".linear", *knowledge, 3);
	containers::NativeDoubleVector angular(container_name + ".angular", *knowledge, 3);
	parse_vector3(&twist->linear, &linear);
	parse_vector3(&twist->angular, &angular);
}


/**
* Parses a ROS Pose Message
* @param  pose   			the geometry_msgs::Pose message
* @param  knowledge 		Knowledgbase
* @param  container_name  	container namespace (e.g. "pose")
**/
void parse_pose (geometry_msgs::Pose *pose, knowledge::KnowledgeBase * knowledge, std::string container_name)
{
	containers::NativeDoubleVector location(container_name + ".position", *knowledge, 3);
	containers::NativeDoubleVector orientation(container_name + ".orientation", *knowledge, 3);
	parse_quaternion(&pose->orientation, &orientation);
	parse_point(&pose->position, &location);
}

void parse_vector3 (geometry_msgs::Vector3 *vec, containers::NativeDoubleVector *target)
{
	target->set(0, vec->x);
	target->set(1, vec->y);
	target->set(2, vec->z);
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
	//std::cout << "o:[" << quat->x << ", " << quat->y << ", " << quat->z << ", " << quat->w <<"]\n";
	// Todo: Use gams Quaternion instead
	tf::Quaternion tfquat(quat->x, quat->y, quat->z, quat->w);
    tf::Matrix3x3 m(tfquat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

	orientation->set(0, roll);
	orientation->set(1, pitch);
	orientation->set(2, yaw);
}



template <size_t N>
void parse_float64_array(boost::array<double, N> *array, containers::NativeDoubleVector *target)
{
	int i = 0;
	for (boost::array<double, 36>::iterator iter(array->begin()); iter != array->end(); ++iter)
	{
		target->set(i, *iter);
		i++;
	}
}

void parse_float64_array(std::vector<float> *array, containers::NativeDoubleVector *target)
{
	int i = 0;
	for (std::vector<float>::iterator iter = array->begin(); iter != array->end(); ++iter)
	{
		target->set(i, *iter);
		i++;
	}
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
	
  	if (save_as_karl)
  		knowledge->save_as_karl(*settings);
	else
		knowledge->save_context(*settings);

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

std::string ros_to_gams_name(std::string ros_topic_name)
{
	// Convert ros_topic_name to lower case
	std::transform(ros_topic_name.begin(), ros_topic_name.end(), ros_topic_name.begin(), ::tolower);
	std::string name = ros_topic_name.substr(1);
	std::string rosbag_robot_prefix = "robot_";
 	
 	std::string topic = ros_topic_name;
	if(name.find(rosbag_robot_prefix) == 0)
	{
		//remove the robot prefix
		int namespace_end = name.find("/") + 1;
		//cut the prefix
		topic = name.substr(namespace_end);
	}
	std::replace(topic.begin(), topic.end(), '/', '.');


	return topic;
}