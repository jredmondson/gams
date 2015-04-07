/**
 * ContinuousMadaraReader.cpp
 * Anton Dukeman
 *
 * Listens for database updates
 */

#include "madara/knowledge_engine/Knowledge_Base.h"
#include "madara/filters/Generic_Filters.h"

#include <string>
#include <iostream>

using std::cerr;
using std::cout;
using std::endl;

class CounterFilter : public Madara::Filters::Record_Filter
{
public:
  size_t count;

  CounterFilter() : count(0) {}

  Madara::Knowledge_Record filter(Madara::Knowledge_Engine::Function_Arguments& args, Madara::Knowledge_Engine::Variables & /*vars*/)
  {
    ++count;
    return args[1];
  }
};

int main(int /*argc*/, char** /*argv*/)
{
  Madara::Transport::QoS_Transport_Settings settings;

  settings.type = Madara::Transport::BROADCAST;
  settings.hosts.push_back(std::string("192.168.0.255:1500"));
//  settings.type = Madara::Transport::MULTICAST;
//  settings.hosts.push_back(std::string("239.255.0.1:4150"));

  CounterFilter counter;
  settings.add_receive_filter(Madara::Knowledge_Record::ALL_TYPES, &counter);
  Madara::Knowledge_Engine::Knowledge_Base knowledge("", settings);

  // get start time
  time_t start;
  time(&start);
  time_t end;
  time(&end);

  // number of seconds to test
  const int NUM_SEC = 10;
  uint64_t loops = 0;

  while(end - start < NUM_SEC)
  {
    ++loops;
    time(&end);
  }

  cerr << double(loops) / double(NUM_SEC) << " Hz loop rate" << endl;
  cerr << double(counter.count) / double(NUM_SEC) << "Hz update rate" << endl;

  return 0;
}
