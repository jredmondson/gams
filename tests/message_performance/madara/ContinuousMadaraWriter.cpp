/**
 * ContinuousMadaraWriter.cpp
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

double num_sec = 12;

Madara::Transport::QoS_Transport_Settings settings;

void handle_arguments (int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1 (argv[i]);
    bool error = true;

    if (arg1 == "-u" || arg1 == "--udp")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = Madara::Transport::UDP;
        error = false;
      }

      ++i;
    }
    else if (arg1 == "-b" || arg1 == "--broadcast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = Madara::Transport::BROADCAST;
        error = false;
      }

      ++i;
    }
    else if (arg1 == "-m" || arg1 == "--multicast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = Madara::Transport::MULTICAST;
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

    if(error)
    {
      cerr << "Test MADARA Writier: " << argv[0] << endl;
      cerr << "  Defaults to using multicast transport on 239.255.0.1:4150" << endl;
      cerr << endl;
      cerr << "    [-u | --udp <address>]         Address for UDP transport" << endl;
      cerr << "    [-b | --broadcast <address>]   Address for broadcast transport" << endl;
      cerr << "    [-m | --multicast <address>]   Address for multicast transport (default: 239.255.0.1:4150)" << endl;
      cerr << "    [-d | --duration <duration>]   number of seconds to run test (default: 12)" << endl;
      exit (0);
    }
  }
}

int main(int argc, char** argv)
{
  handle_arguments(argc, argv);

  // set default transport
  if(settings.hosts.size() == 0)
  {
    const std::string default_multicast ("239.255.0.1:4150");
    settings.hosts.push_back(default_multicast);
    settings.type = Madara::Transport::MULTICAST;
  }

  Madara::Knowledge_Engine::Knowledge_Base knowledge("", settings);

  // get start time
  time_t start;
  time(&start);
  time_t end;
  time(&end);

  // number of seconds to test
  const std::string val("hello");
  uint64_t updates = 0;
  const std::string key("data");

  while(end - start < num_sec)
  {
    knowledge.set(key, val);
    ++updates;
    time(&end);
  }

  cerr << double(updates) / num_sec << " Hz knowledge update rate" << endl;

  return 0;
}
