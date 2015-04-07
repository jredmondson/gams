/**
 * robot.cpp
 * by Anton Dukeman
 * adapted from online tutorials at ros.org
 *
 * Listens for nav goals over tcp socket
 */

// ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "madara/utility/Utility.h"

#include <iostream>

size_t msg_counter;

void callback(const std_msgs::String& event)
{
  ++msg_counter;
}

int main(int argc, char** argv)
{
  // Initialize and name the node
  ROS_INFO("Init node");
  ros::init(argc, argv, argv[1]);

  // Get node handle
  ros::NodeHandle n;

  //ros::Subscriber sub = n.subscribe("chatter", 100000, callback, ros::TransportHints().unreliable());
  ros::Subscriber sub = n.subscribe("chatter", 100000, callback);

  // number of seconds to test
  const int NUM_SEC = 10;

  // loop counter
  size_t counter = 0;
  msg_counter = 0;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  double sleep_time = 10;
  sleep_time = Madara::Utility::sleep(sleep_time);
  ros::shutdown();

  std::cerr << double(msg_counter) / sleep_time << " Hz msg rate" << std::endl;
  
  return 0;
}
