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
double poll_period = 0.001;

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
    else if (arg1 == "-f" || arg1 == "--frequency")
    {
      if (i + 1 < argc)
      {
        std::stringstream ss;
        ss << argv[i + 1];
        ss >> poll_period;
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
      cerr << "    [-m | --multicast <address>]   Address for multicast transport" << endl;
      cerr << "    [-d | --duration <duration>]   number of seconds to run test (default: 12)" << endl;
      cerr << "    [-p | --period <per>]          period for publish loop (default: 0.001)" << endl;
      exit (0);
    }
  }
}

int main(int argc, char** argv)
{
  handle_arguments(argc, argv);

  Madara::Knowledge_Engine::Knowledge_Base knowledge("", settings);

  // get start time
  time_t start_time;
  time(&start_time);
  time_t end;
  time(&end);

  // number of seconds to test
  std::string val("hello");
  for(size_t i = 0; i < 5; ++i)
    val = val + val;
  uint64_t updates = 0;
  const std::string key("data");
  knowledge.set(key, val);

  ACE_Time_Value current = ACE_OS::gettimeofday();

  ACE_Time_Value publish_period;
  publish_period.set(poll_period);

  ACE_Time_Value max_runtime;
  max_runtime.set(num_sec);
  max_runtime = current + max_runtime;

  ACE_Time_Value next_epoch = current + publish_period;

  ACE_Time_Value start = ACE_OS::gettimeofday();

  while(end - start_time < num_sec)
  {
    knowledge.set(key, val);
    ++updates;

    ACE_Time_Value current = ACE_OS::gettimeofday ();
    Madara::Utility::sleep (next_epoch - current);  
      
    // setup the next 
    next_epoch += publish_period;

    time(&end);
  }

  cerr << double(updates) / num_sec << " Hz knowledge update rate" << endl;

  return 0;
}
