/**
 * ContinuousMadara.cpp
 * Anton Dukeman
 *
 * Inserts a single key/value pair into the database continuously
 */

#include "madara/knowledge_engine/Knowledge_Base.h"

#include <string>
#include <iostream>

using std::cerr;
using std::cout;
using std::endl;

int main(int /*argc*/, char** /*argv*/)
{
  Madara::Transport::Settings settings;
  settings.type = Madara::Transport::BROADCAST;
  settings.hosts.push_back(std::string("192.168.0.255:1500"));
//  settings.type = Madara::Transport::MULTICAST;
//  settings.hosts.push_back(std::string("239.255.0.1:4150"));
  Madara::Knowledge_Engine::Knowledge_Base knowledge("", settings);

  // get start time
  time_t start;
  time(&start);
  time_t end;
  time(&end);

  // number of seconds to test
  const int NUM_SEC = 12;
  double val = 1;
  uint64_t updates = 0;
  const std::string key("data");

  while(end - start < NUM_SEC)
  {
    knowledge.set(key, ++val);
    ++updates;
    time(&end);
  }

  cerr << double(updates) / double(NUM_SEC) << " Hz knowledge update rate" << endl;

  return 0;
}
