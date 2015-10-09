/**
 * ContinuousMadaraReader.cpp
 * Anton Dukeman
 *
 * Counts database updates
 */

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/filters/GenericFilters.h"

#include <string>
#include <iostream>

using std::cerr;
using std::cout;
using std::endl;

double num_sec = 10;

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

    if(error)
    {
      cerr << "Test MADARA Reader: " << argv[0] << endl;
      cerr << "  Defaults to using multicast transport on 239.255.0.1:4150" << endl;
      cerr << endl;
      cerr << "    [-u | --udp <address>]         Address for UDP transport" << endl;
      cerr << "    [-b | --broadcast <address>]   Address for broadcast transport" << endl;
      cerr << "    [-m | --multicast <address>]   Address for multicast transport (default: 239.255.0.1:4150)" << endl;
      cerr << "    [-d | --duration <duration>]   number of seconds to run test (default: 10)" << endl;
      exit (0);
    }
  }
}

class CounterFilter : public madara::filters::RecordFilter
{
public:
  size_t count;

  CounterFilter() : count(0) {}

  madara::KnowledgeRecord filter(madara::knowledge::FunctionArguments& args,
    madara::knowledge::Variables & /*vars*/)
  {
    ++count;
    return args[1];
  }
};

int main(int argc, char** argv)
{
  handle_arguments(argc, argv);

  // set default transport
  if(settings.hosts.size() == 0)
  {
    const std::string default_multicast ("239.255.0.1:4150");
    settings.hosts.push_back(default_multicast);
    settings.type = madara::transport::MULTICAST;
  }

  CounterFilter counter;
  settings.add_receive_filter(madara::KnowledgeRecord::ALL_TYPES, &counter);
  madara::knowledge::KnowledgeBase knowledge("", settings);

  num_sec = madara::utility::sleep(num_sec);

  cout << double(counter.count) / num_sec << " Hz update rate" << endl;

  return 0;
}
