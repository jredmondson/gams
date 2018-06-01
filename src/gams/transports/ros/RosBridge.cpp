
#include <sstream>
#include "RosBridge.h"
#include "RosBridgeReadThread.h"

namespace knowledge = madara::knowledge;

gams::transports::RosBridge::RosBridge (
  const std::string & id,
  madara::transport::TransportSettings & new_settings,
  knowledge::KnowledgeBase & knowledge)
: madara::transport::Base (id, new_settings, knowledge.get_context ())
{
  // populate variables like buffer_ based on transport settings
  Base::setup ();
  
  // set the data plane for read threads
  read_threads_.set_data_plane (knowledge);
  
  // check the read hz settings to see if the user has passed something weird
  double hertz = new_settings.read_thread_hertz;
  if (hertz < 0.0)
  {
    hertz = 0.0;
  }

  // create the read threads specified in TransportSettings  
  for (uint32_t i = 0; i < new_settings.read_threads; ++i)
  {
    // construct a unique id for a new thread
    std::stringstream thread_name;
    thread_name << "read";
    thread_name << i;
    
    // start the thread at the specified hertz
    read_threads_.run (
      hertz,
      thread_name.str (),
      new RosBridgeReadThread (
        id_, new_settings, 
        send_monitor_, receive_monitor_, packet_scheduler_));
  }
}

gams::transports::RosBridge::~RosBridge ()
{
    std::cout << "TERMINATE!!!" << std::endl << std::endl << std::endl << std::endl;
    read_threads_.terminate();
}

long
gams::transports::RosBridge::send_data (
  const madara::knowledge::VariableReferenceMap & modifieds)
{
  /**
   * Return number of bytes sent or negative for error
   **/
  long result (0);
  
  /**
   * This is where you should do your custom transport sending logic/actions
   **/
  std::cout << std::endl << "SENDING UPDATE TO ROS!!!" << std::endl;
  for (
    madara::knowledge::VariableReferenceMap::const_iterator it = modifieds.begin();
    it != modifieds.end(); it++)
  {
    const char * key = it->first;
    std::cout << "Got update for " << key << std::endl;
  }
  std::cout << std::endl;
  return result;
}
