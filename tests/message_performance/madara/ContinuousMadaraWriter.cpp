/**
 * ContinuousMadaraWriter.cpp
 * Anton Dukeman
 *
 * Inserts a single key/value pair into the database continuously
 */

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/utility/EpochEnforcer.h"

#include <string>
#include <iostream>

using std::cerr;
using std::cout;
using std::endl;

double num_sec = 12;
double poll_period = 0.001;

typedef  madara::utility::EpochEnforcer<
  std::chrono::steady_clock> EpochEnforcer;

madara::transport::QoSTransportSettings settings;

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
        settings.type = madara::transport::UDP;
        error = false;
      }

      ++i;
    }
    else if (arg1 == "-b" || arg1 == "--broadcast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::BROADCAST;
        error = false;
      }

      ++i;
    }
    else if (arg1 == "-m" || arg1 == "--multicast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::MULTICAST;
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

  madara::knowledge::KnowledgeBase knowledge("", settings);

  // number of seconds to test
  std::string val("hello");
  for(size_t i = 0; i < 5; ++i)
    val = val + val;
  uint64_t updates = 0;
  const std::string key("data");
  knowledge.set(key, val);

  EpochEnforcer enforcer (poll_period, num_sec);

  while(!enforcer.is_done ())
  {
    knowledge.set(key, val);
    ++updates;

    enforcer.sleep_until_next ();
  }

  cerr << double(updates) / num_sec << " Hz knowledge update rate" << endl;

  return 0;
}
