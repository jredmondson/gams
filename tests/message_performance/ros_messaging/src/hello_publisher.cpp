/**
 * ones_publisher.cpp
 * by Anton Dukeman
 *
 * See http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 for more information
 */

// ROS stuff
#include "ros/ros.h" // required for ros
#include "std_msgs/String.h" // bring in string messages

#include <iostream>
#include <sstream>

using std::cerr;
using std::endl;

size_t buffer = 1000;
double num_sec = 15;
size_t sleep_time = 0;

void handle_arguments (int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1 (argv[i]);
    bool error = true;

    if (arg1 == "-b" || arg1 == "--buffer")
    {
      if (i + 1 < argc)
      {
        std::stringstream ss;
        ss << argv[i + 1];
        ss >> buffer;
        error = false;
      }

      ++i;
    }
    else if (arg1 == "-t" || arg1 == "--time")
    {
      if (i + 1 < argc)
      {
        std::stringstream ss;
        ss << argv[i + 1];
        ss >> num_sec;
        error = false;
      }

      ++i;
    }
    else if (arg1 == "-s" || arg1 == "--sleep")
    {
      if (i + 1 < argc)
      {
        std::stringstream ss;
        ss << argv[i + 1];
        ss >> sleep_time;
        error = false;
      }

      ++i;
    }

    if(error)
    {
      cerr << "Test ROS publisher: " << argv[0] << endl;
      cerr << "  [-b | --buffer size]     size of the send buffer (default, 1000)" << endl;
      cerr << "  [-t | --time duration]   test duration (default, 15)" << endl;
      cerr << "  [-s | --sleep duration]  time to sleep in usec (default, 0)" << endl;
      exit (0);
    }
  }
}

int main(int argc, char** argv)
{
  handle_arguments(argc, argv);

  // ROS_INFO is for logging; ROS_WARN and ROS_ERROR are similar
  ROS_INFO("Init node");

  // argc and argv will not do anything here, but can be used for topic aliasing
  // "talker" is the name of this node; nodes in the ROS network must have unique names
  ros::init(argc, argv, "talker");

  // Get node handle
  ros::NodeHandle n;

  // advertise that this node will be publishing std_msgs::String messages on the "chatter" topic 
  // with some buffer size. Messages are buffered before sending out with newer messages 
  // kicking out older messages
  ros::Publisher publisher = n.advertise<std_msgs::String>("chatter", buffer);

  // timing and counter code
  time_t start;
  time(&start);
  time_t end;
  time(&end);
  size_t counter = 0;

  // publish loop
  while(end - start < num_sec)
  {
    // create message
    std_msgs::String msg;
    msg.data = "hello";

    // publish message
    publisher.publish(msg);

    // sleep a bit
    usleep(sleep_time);

    // timing and looping counter
    time(&end);
    ++counter;
  }

  // calculate publish rate
  std::cerr << double(counter) / num_sec << " Hz msg publish rate" << std::endl;
  
  return 0;
}
