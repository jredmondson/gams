/**
 * ones_publisher.cpp
 * by Anton Dukeman
 */

// ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>

int main(int argc, char** argv)
{
  // Initialize and name the node
  ROS_INFO("Init node");
  ros::init(argc, argv, "talker");

  // Get node handle
  ros::NodeHandle n;

  ros::Publisher publisher = n.advertise<std_msgs::String>("chatter", 100);

  // get start time
  time_t start;
  time(&start);
  time_t end;
  time(&end);
  size_t counter = 0;

  // number of seconds to test
  const int NUM_SEC = 12;
 
  while(end - start < NUM_SEC)
  {
    std_msgs::String msg;
    msg.data = "hello";
    publisher.publish(msg);
    ros::spinOnce();
    time(&end);
    ++counter;
  }

  std::cerr << double(counter) / double(NUM_SEC) << " Hz msg publish rate" << std::endl;
  
  return 0;
}
