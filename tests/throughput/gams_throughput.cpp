

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/threads/Threader.h"
#include "gams/controllers/BaseController.h"
#include "gams/loggers/GlobalLogger.h"
#include <fstream>

#include "utility/MovingAverage.h"
#include "madara/utility/Utility.h"
#include "madara/utility/EpochEnforcer.h"
#include "madara/utility/Timer.h"

// DO NOT DELETE THIS SECTION

// begin algorithm includes
#include "algorithms/Modify.h"
// end algorithm includes

// begin platform includes
// end platform includes

// begin thread includes
// end thread includes

// begin transport includes
// end transport includes

// begin filter includes
#include "filters/RcvCount.h"
// end filter includes

// END DO NOT DELETE THIS SECTION

typedef  madara::utility::EpochEnforcer<
  std::chrono::steady_clock> EpochEnforcer;
typedef  madara::utility::Timer<
  std::chrono::steady_clock> Timer;


const std::string default_broadcast("192.168.1.255:15000");
// default transport settings
std::string host("");
const std::string default_multicast("239.255.0.1:4150");
madara::transport::QoSTransportSettings settings;

// create shortcuts to MADARA classes and namespaces
namespace controllers = gams::controllers;
typedef madara::knowledge::KnowledgeRecord   Record;
typedef Record::Integer Integer;

const std::string KNOWLEDGE_BASE_PLATFORM_KEY(".platform");
bool plat_set = false;
std::string platform("null");
std::string algorithm("modify");
std::vector <std::string> accents;
std::string output_filename;

// controller variables
double period(1.0);
double loop_time(60.0);
madara::knowledge::KnowledgeRecord::Integer payload_size(0);
bool no_apply(false);

// madara commands from a file
std::string madara_commands = "";

// for setting debug levels through command line
int madara_debug_level(-1);
int gams_debug_level(-1);

// number of agents in the swarm
Integer num_agents(-1);

// file path to save received files to
std::string file_path;

// keep track of time
uint64_t elapsed_ns(0);
Timer timer;

// the number of latencies to use in moving latency calculations
size_t max_latencies(500);

/// flag for acting as a pure data sink
bool rcv_only(false);

void print_usage(char * prog_name, char * arg)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_ALWAYS,
"\nProgram summary for %s(arg %s):\n\n" 
"     Loop controller setup for gams\n" 
" [-A |--algorithm type]        algorithm to start with\n" 
" [-a |--accent type]           accent algorithm to start with\n" 
" [-b |--broadcast ip:port]     the broadcast ip to send and listen to\n" 
" [-d |--domain domain]         the knowledge domain to send and listen to\n" 
" [--deadline time]             deadline for dropping packets in seconds\n" 
" [-e |--rebroadcasts num]      number of hops for rebroadcasting messages\n" 
" [-f |--logfile file]          log to a file\n" 
" [-i |--id id]                 the id of this agent(should be non-negative)\n" 
" [--madara-level level]        the MADARA logger level(0+)\n" 
" [--gams-level level]          the GAMS logger level(0+)\n" 
" [-L |--loop-time time]        time to execute loop\n"
" [-l |--latencies num]         number of latencies to use for calculations\n"
" [-m |--multicast ip:port]     the multicast ip to send and listen to\n" 
" [-M |--madara-file <file>]    file containing madara commands to execute\n" 
"                               multiple space-delimited files can be used\n" 
" [-n |--num_agents <number>]   the number of agents in the swarm\n" 
" [--no-apply|--no-world-update] filter out all received data\n" 
" [-o |--host hostname]         the hostname of this process(def:localhost)\n" 
" [--output   <file>]           the output file name for test statistics\n"
" [-p |--platform type]         platform for loop(vrep, dronerk)\n" 
" [-P |--period period]         time, in seconds, between loop executions\n" 
" [-q |--queue-length length]   length of transport queue in bytes\n" 
" [-r |--reduced]               use the reduced message header\n" 
" [--rcv-only]                  do not send. Act as a pure data sink.\n" 
" [-s |--size|--payload] size   the payload size to add each send\n"
" [--total-size] size           total packet size, including agent info\n"
"                              (this overrides -s/--size)\n"
" [-t |--read-threads threads]  number of read threads\n" 
" [-tz |--read-threads-hertz hz] read thread hertz\n" 
" [-u |--udp ip:port]           a udp ip to send to(first is self to bind to)\n" 
" [-z |--hertz hertz]           the hertz rate to execute algorithm at\n" 
" [--zmq proto:ip:port]         specifies a 0MQ transport endpoint\n" 
"\n",
        prog_name, arg);
  exit(0);
}

// handle command line arguments
void handle_arguments(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1(argv[i]);

    if (arg1 == "-A" || arg1 == "--algorithm")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        algorithm = argv[i + 1];
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-a" || arg1 == "--accent")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        accents.push_back(argv[i + 1]);
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-b" || arg1 == "--broadcast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back(argv[i + 1]);
        settings.type = madara::transport::BROADCAST;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-d" || arg1 == "--domain")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        settings.write_domain = argv[i + 1];
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "--deadline")
    {
      if (i + 1 < argc)
      {
        double deadline;
        std::stringstream buffer(argv[i + 1]);
        buffer >> deadline;
        
        settings.set_deadline(deadline);
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-e" || arg1 == "--rebroadcasts")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        int hops;
        std::stringstream buffer(argv[i + 1]);
        buffer >> hops;

        settings.set_rebroadcast_ttl(hops);
        settings.enable_participant_ttl(hops);
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-f" || arg1 == "--logfile")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        madara::logger::global_logger->add_file(argv[i + 1]);
        gams::loggers::global_logger->add_file(argv[i + 1]);
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }

    else if (arg1 == "-i" || arg1 == "--id")
    {
      if (i + 1 < argc && argv[i +1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> settings.id;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "--madara-level")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> madara_debug_level;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "--gams-level")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> gams_debug_level;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-L" || arg1 == "--loop-time")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> loop_time;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-l" || arg1 == "--latencies")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> max_latencies;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-m" || arg1 == "--multicast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back(argv[i + 1]);
        settings.type = madara::transport::MULTICAST;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-M" || arg1 == "--madara-file")
    {
      bool files = false;
      ++i;
      for (;i < argc && argv[i][0] != '-'; ++i)
      {
        std::string filename = argv[i];
        if (madara::utility::file_exists(filename))
        {
          madara_commands += madara::utility::file_to_string(filename);
          madara_commands += ";\n";
          files = true;
        }
      }
      --i;

      if (!files)
        print_usage(argv[0], argv[i]);
    }
    else if (arg1 == "-n" || arg1 == "--num_agents")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> num_agents;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "--no-apply" || arg1 == "--no-world-update")
    {
      no_apply = true;
    }
    else if (arg1 == "-o" || arg1 == "--host")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        host = argv[i + 1];
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "--output" || arg1 == "--output-file")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        output_filename = argv[i + 1];
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-p" || arg1 == "--platform")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        platform = argv[i + 1];
        plat_set = true;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-P" || arg1 == "--period")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> period;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-q" || arg1 == "--queue-length")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> settings.queue_length;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-r" || arg1 == "--reduced")
    {
      settings.send_reduced_message_header = true;
    }
    else if (arg1 == "--rcv-only")
    {
      rcv_only = true;
    }
    else if (arg1 == "-s" || arg1 == "--size" || arg1 == "--payload-size")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> payload_size;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "--total-size")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer(argv[i + 1]);
        buffer >> payload_size;

        if (payload_size >= 551)
        {
          payload_size -= 551;
        }
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-t" || arg1 == "--read-threads")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer;
        buffer << argv[i + 1];
        buffer >> settings.read_threads;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-tz" || arg1 == "--read-threads-hertz")
    {
      if (i + 1 < argc)
      {
        std::stringstream buffer;
        buffer << argv[i + 1];
        buffer >> settings.read_thread_hertz;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-u" || arg1 == "--udp")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back(argv[i + 1]);
        settings.type = madara::transport::UDP;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "-z" || arg1 == "--hertz")
    {
      if (i + 1 < argc)
      {
        double hertz;
        std::stringstream buffer(argv[i + 1]);
        buffer >> hertz;

        if (hertz > 0)
        {
          period = 1 / hertz;
        }
        else
        {
          period = 0.0;
        }
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else if (arg1 == "--zmq")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back(argv[i + 1]);
        settings.type = madara::transport::ZMQ;
      }
      else
        print_usage(argv[0], argv[i]);

      ++i;
    }
    else
    {
      print_usage(argv[0], argv[i]);
    }
  }
}

// perform main logic of program
int main(int argc, char ** argv)
{
  int log_count(0);

  utility::MovingAverage latencies(max_latencies);

  settings.type = madara::transport::MULTICAST;
  std::ofstream results_file;
  madara::knowledge::KnowledgeMap alg_args;

  settings.queue_length = 1000000 + payload_size * 1000;

  // handle all user arguments
  handle_arguments(argc, argv);

  if (payload_size > 0)
  {
    alg_args["payload"] = madara::knowledge::KnowledgeRecord(payload_size);
  }

  if (settings.hosts.size() == 0)
  {
    // setup default transport as multicast
    settings.hosts.resize(1);
    settings.hosts[0] = default_multicast;
  }

  // create unique originator id that is easier to remember
  if (host == "")
  {
    std::stringstream buffer;
    buffer << "agent" << settings.id;
    
    host = buffer.str();
  }

  // set this once to allow for debugging knowledge base creation
  if (madara_debug_level >= 0)
  {
    madara::logger::global_logger->set_level(madara_debug_level);
  }

  double hertz = 0.0;

  if (period != 0)
    hertz = 1.0 / period;

  if (!rcv_only)
  {
    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "%d: Test settings:\n"
      "     agent id: %d\n"
      "     originator id: %s\n"
      "     publish rate: %.2f hz\n"
      "     duration: %.2f seconds\n"
      "     transport type: %s\n"
      "     message deadline: %.2f\n"
      "     payload size: %d bytes\n"
      "     message queue length: %d bytes\n"
      "     pure data sink? %s\n"
      "     updating world model? %s\n"
      "     latency depth: %d last messages\n"
      "\n",
      ++log_count,
     (int)settings.id,
      host.c_str(),
      hertz,
      loop_time,
      madara::transport::type_name(settings).c_str(),
      settings.get_deadline(),
     (int)payload_size,
     (int)settings.queue_length,
      rcv_only ? "yes" : "no",
      no_apply ? "no" : "yes",
     (int)max_latencies
      );
  }
  else
  {
    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "%d: Transport test settings:\n"
      "     agent id: %d\n"
      "     duration: %.2f seconds\n"
      "     transport type: %s\n"
      "     message deadline: %.2f\n"
      "     message queue length: %d bytes\n"
      "     pure data sink? %s\n"
      "     updating world model? %s\n"
      "     latency depth: %d last messages\n"
      "\n",
      ++log_count,
     (int)settings.id,
      loop_time,
      madara::transport::type_name(settings).c_str(),
      settings.get_deadline(),
     (int)settings.queue_length,
      rcv_only ? "yes" : "no",
      no_apply ? "no" : "yes",
     (int)max_latencies
    );
  }

  // create knowledge base and a control loop
  madara::knowledge::KnowledgeBase knowledge;

  filters::RcvCount * filter = new filters::RcvCount(latencies, no_apply);

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Attaching transport with receive filter.\n", ++log_count);

  // begin on receive filters
  settings.add_receive_filter(filter);
  // end on receive filters

  // begin on send filters
  // end on send filters

  // if you only want to use custom transports, delete following
  knowledge.attach_transport(host, settings);

  // begin transport creation 
  // end transport creation

  // set this once to allow for debugging controller creation
  if (gams_debug_level >= 0)
  {
    gams::loggers::global_logger->set_level(gams_debug_level);
  }

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Creating controller.\n", ++log_count);

  controllers::BaseController controller(knowledge);
  madara::threads::Threader threader(knowledge);

  if (!rcv_only)
  {
    // initialize variables and function stubs
    controller.init_vars(settings.id, num_agents);

    std::vector <std::string> aliases;

    // begin adding custom algorithm factories

    // add Modify factory
    aliases.clear();
    aliases.push_back("modify");

    controller.add_algorithm_factory(aliases,
      new algorithms::ModifyFactory());
    // end adding custom algorithm factories

    // begin adding custom platform factories
    // end adding custom platform factories
  }

  // read madara initialization
  if (madara_commands != "")
  {
#ifndef _MADARA_NO_KARL_
    knowledge.evaluate(madara_commands,
      madara::knowledge::EvalSettings(false, true));
#endif
  }

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Applying debug levels to MADARA\n", ++log_count);

  // set debug levels if they have been set through command line
  if (madara_debug_level >= 0)
  {
    std::stringstream temp_buffer;
    temp_buffer << "agent." << settings.id << ".madara_debug_level = ";
    temp_buffer << madara_debug_level;

    madara::logger::global_logger->set_level(madara_debug_level);

#ifndef _MADARA_NO_KARL_
    // modify the debug level being used but don't send out to others
    knowledge.evaluate(temp_buffer.str(),
      madara::knowledge::EvalSettings(true, true));
#endif
  }

  if (!rcv_only)
  {
    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "%d: Applying debug levels to GAMS\n", ++log_count);

    if (gams_debug_level >= 0)
    {
      std::stringstream temp_buffer;
      temp_buffer << "agent." << settings.id << ".gams_debug_level = ";
      temp_buffer << gams_debug_level;

      gams::loggers::global_logger->set_level(gams_debug_level);

#ifndef _MADARA_NO_KARL_
      // modify the debug level being used but don't send out to others
      knowledge.evaluate(temp_buffer.str(),
        madara::knowledge::EvalSettings(true, true));
#endif
    }

    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "%d: Initializing algorithm and platform\n", ++log_count);

    // initialize the platform and algorithm
    // default to platform in knowledge base if platform not set in command line

    controller.init_platform(platform);
    controller.init_algorithm(algorithm, alg_args);

    // add any accents
    for (unsigned int i = 0; i < accents.size(); ++i)
    {
      controller.init_accent(accents[i]);
    }
  }

  /**
   * WARNING: the following section will be regenerated whenever new threads
   * are added via this tool. So, you can adjust hertz rates and change how
   * the thread is initialized, but the entire section will be regenerated
   * with all threads in the threads directory, whenever you use the new
   * thread option with the gpc.pl script.
   **/

  // begin thread creation
  // end thread creation
  
  /**
   * END WARNING
   **/

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Beginning experiment\n", ++log_count);

  // get timestamp for start of test. I gave up on C11 chrono in VS
  //auto start = std::chrono::high_resolution_clock::now();
  timer.start();

  if (!rcv_only)
  {
    // run a mape loop for algorithm and platform control
    controller.run(period, loop_time);

    // terminate all threads after the controller
    threader.terminate();

    // wait for all threads
    threader.wait();
  }
  else
  {
    // if only a data sink, just sleep for a time
    madara::utility::sleep(loop_time);
  }

  // get timestamp for end of test. I gave up on C11 chrono in VS
  //auto end = std::chrono::high_resolution_clock::now();

  //std::chrono::milliseconds duration(
  //  std::chrono::duration_cast<std::chrono::milliseconds>(end - start));

  timer.stop();

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Experiment stopped. Calculating time.\n", ++log_count);

  // provide convenience elapsed durations
  elapsed_ns = timer.duration_ns();
  double elapsed_seconds = timer.duration_ds();

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Printing final knowledge base.\n", ++log_count);

  // print all knowledge values
  knowledge.print();

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Printing results.\n", ++log_count);

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Experiment results:\n"
    "     elapsed time: %.2f\n"
    "     messages sent: %d\n"
    "     messages received: %d\n"
    "     transport type: %d\n"
    "     num latencies: %d\n"
    "     min latency: %.2f\n"
    "     max latency: %.2f\n"
    "     avg latency: %.2f\n"
    "\n",
    ++log_count,
    elapsed_seconds,
   (int)knowledge.get(".executions").to_integer(),
   (int)knowledge.get(".receives").to_integer(),
   (int)settings.type,
   (int)latencies.size(),
   (int)latencies.get_min(),
   (int)latencies.get_max(),
   (int)latencies.average()
    );

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Printing message originators that were received.\n", ++log_count);

  filters::Originators originators = filter->originators();

  for (filters::Originators::const_iterator i = originators.begin();
    i != originators.end(); ++i)
  {
    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "     %s: %d messages received\n", i->first.c_str(), i->second);
  }

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Saving results.\n", ++log_count);

  // save results to file
  {
    if (output_filename == "")
    {
      std::stringstream buffer;
      buffer << "agent" << settings.id << "_results.csv";
      output_filename = buffer.str();

      results_file.open(output_filename.c_str());
    }
    else
    {
      results_file.open(output_filename.c_str());
    }

    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "     printing results to %s.\n", output_filename.c_str());

    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "     creating header for CSV file.\n");

    results_file << "payload size,intended send hz,num read threads,";
    results_file << "read thread hz,test duration in seconds,";
    results_file << "actual sends,actual receives,";
    results_file << "num originators,";

    for (filters::Originators::const_iterator i = originators.begin();
      i != originators.end(); ++i)
    {
      results_file << i->first << " receives,";
    }

    results_file << "num latencies,min latency,max latency,avg latency,";
    results_file << "\n";

    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "     outputting base throughput results.\n");

    // save the send hertz rate to file
    results_file << payload_size << ",";

    // save the send hertz rate to file
    results_file << hertz << ",";

    // save the number of read threads
    results_file << settings.read_threads << ",";

    // save the read threads hertz
    results_file << settings.read_thread_hertz << ",";

    // save the duration of the test in seconds
    results_file << elapsed_seconds << ",";

    // save the loop executions to file
    results_file << knowledge.get(".executions").to_integer() << ",";

    // save the receives to file
    results_file << knowledge.get(".receives").to_integer() << ",";

    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "     outputting num originators/sources.\n");

    results_file << originators.size() << ",";

    for (filters::Originators::const_iterator i = originators.begin();
      i != originators.end(); ++i)
    {
      results_file << i->second << ",";
    }

    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "     outputting latency results.\n");

    results_file << latencies.size() << ",";
    results_file << latencies.get_min() << ",";
    results_file << latencies.get_max() << ",";

    gams::loggers::global_logger->log(
      gams::loggers::LOG_ALWAYS,
      "     outputting latency average.\n");

    results_file << latencies.average() << ",";

    results_file << "\n";

    results_file.close();
  }

  gams::loggers::global_logger->log(
    gams::loggers::LOG_ALWAYS,
    "%d: Exiting.\n", ++log_count);

  return 0;
}

