/**
 * loop_rate_testing.cpp
 * by Anton Dukeman
 *
 * See http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 for more information
 */

// ROS stuff
#include "ros/ros.h" // standard ros header
#include "std_msgs/String.h" // bring in string messages

#include "madara/utility/Utility.h" // for sleep timer

#include <iostream>
#include <string>
#include <sstream>

using std::cerr;
using std::endl;

bool use_udp = false;
double num_sec = 10;
std::string node_name("");
size_t queue = 1000;

void handle_arguments(int argc, char ** argv)
{
  bool node_is_named = false;
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1(argv[i]);
    bool error = true;

    if (arg1 == "-t" || arg1 == "--transport")
    {
      if (i + 1 < argc)
      {
        int u;
        std::stringstream ss;
        ss << argv[i + 1];
        ss >> u;
        use_udp =(u == 1);
        error = false;
      }

      ++i;
    }
    else if (arg1 == "-d" || arg1 == "--duration")
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
    else if (arg1 == "-n" || arg1 == "--name")
    {
      if (i + 1 < argc)
      {
        std::stringstream ss;
        ss << argv[i + 1];
        ss >> node_name;
        error = false;
        node_is_named = true;
      }

      ++i;
    }
    else if (arg1 == "-q" || arg1 == "--queue")
    {
      if (i + 1 < argc)
      {
        std::stringstream ss;
        ss << argv[i + 1];
        ss >> queue;
        error = false;
      }

      ++i;
    }

    if (error)
    {
      cerr << "Test ROS subscriber: " << argv[0] << endl;
      cerr << "  Required:" << endl;
      cerr << "    [-n | --name <node_name>]      name for node, must be unique in ROS network" << endl;
      cerr << endl;
      cerr << "  Optional:" << endl;
      cerr << "    [-t | --transport <type>]      0 for tcp, 1 for udp(default: 0)" << endl;
      cerr << "    [-d | --duration <duration>]   number of seconds to run test(default: 10)" << endl;
      cerr << "    [-q | --queue <size>]          size of queue for message processing(default: 1000)" << endl;
      exit(0);
    }
  }

  if (!node_is_named)
  {
    cerr << "Need node name: -n | --name" << endl;
    exit(0);
  }
}

// counter for processed messages
size_t msg_counter = 0;

// callback for std_msgs::String messages
void callback(const std_msgs::String& /*msg*/)
{
  ++msg_counter;
}

int main(int argc, char** argv)
{
  handle_arguments(argc, argv);

  // Initialize and name the node
  // using argv[1] for naming since we will need many of these for performance testing
  ROS_INFO("Init node");
  ros::init(argc, argv, node_name);

  // Get node handle
  ros::NodeHandle n;

  // subscribe to topic
  ros::Subscriber sub;
  if (use_udp)
  {
    // subscribe to "chatter" topic, a queue of 100k messages, callback is the function named callback, use UDP, if publisher
    //    doesn't support UDP for some reason, it will fallback to TCP
    sub = n.subscribe("chatter", queue, callback, ros::TransportHints().udp());
  }
  else
  {
    // subscribe to "chatter" topic, a queue size, callback is the function named callback, use TCP by default
    sub = n.subscribe("chatter", queue, callback);
  }

  // launch a single separate thread to handle message callbacks...arg is number of threads
  ros::AsyncSpinner spinner(1);

  // start spinner
  spinner.start();
  
  // sleep the main thread
  num_sec = Madara::Utility::sleep(num_sec);

  // shutdown the node
  ros::shutdown();

  // display message rate
  std::cerr << double(msg_counter) / num_sec << " Hz msg rate" << std::endl;
  
  return 0;
}
