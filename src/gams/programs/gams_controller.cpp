/**
 * Copyright (c) 2014 Carnegie Mellon University. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following acknowledgments and disclaimers.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. The names "Carnegie Mellon University," "SEI" and/or "Software
 *    Engineering Institute" shall not be used to endorse or promote products
 *    derived from this software without prior written permission. For written
 *    permission, please contact permission@sei.cmu.edu.
 * 
 * 4. Products derived from this software may not be called "SEI" nor may "SEI"
 *    appear in their names without prior written permission of
 *    permission@sei.cmu.edu.
 * 
 * 5. Redistributions of any form whatsoever must retain the following
 *    acknowledgment:
 * 
 *      This material is based upon work funded and supported by the Department
 *      of Defense under Contract No. FA8721-05-C-0003 with Carnegie Mellon
 *      University for the operation of the Software Engineering Institute, a
 *      federally funded research and development center. Any opinions,
 *      findings and conclusions or recommendations expressed in this material
 *      are those of the author(s) and do not necessarily reflect the views of
 *      the United States Department of Defense.
 * 
 *      NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 *      INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 *      UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR
 *      IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF
 *      FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS
 *      OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES
 *      NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT,
 *      TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 *      This material has been approved for public release and unlimited
 *      distribution.
 **/

/**
 * @file gams_controller.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a test driver for the GAMS controller loop.
 **/

#include "madara/knowledge/KnowledgeBase.h"
#include "gams/controllers/BaseController.h"
#include "gams/loggers/GlobalLogger.h"
#include "gams/loggers/GlobalLogger.h"

const std::string default_broadcast ("192.168.1.255:15000");
// default transport settings
std::string host ("");
const std::string default_multicast ("239.255.0.1:4150");
madara::transport::QoSTransportSettings settings;

// create shortcuts to MADARA classes and namespaces
namespace engine = madara::knowledge;
namespace controllers = gams::controllers;
typedef madara::knowledge::KnowledgeRecord   Record;
typedef Record::Integer Integer;

const std::string KNOWLEDGE_BASE_PLATFORM_KEY (".platform");
bool plat_set = false;
std::string platform ("debug");
std::string algorithm ("debug");
std::vector <std::string> accents;

// controller variables
double period (1.0);
double loop_time (50.0);

// madara commands from a file
std::string madara_commands = "";

// for setting debug levels through command line
int madara_debug_level (-1);
int gams_debug_level (-1);

// number of agents in the swarm
Integer num_agents (-1);

// file path to save received files to
std::string file_path;

void print_usage (char * prog_name)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
"\nProgram summary for %s:\n\n" \
"     Loop controller setup for gams\n" \
" [-A |--algorithm type]        algorithm to start with\n" \
" [-a |--accent type]           accent algorithm to start with\n" \
" [-b |--broadcast ip:port]     the broadcast ip to send and listen to\n" \
" [-d |--domain domain]         the knowledge domain to send and listen to\n" \
" [-e |--rebroadcasts num]      number of hops for rebroadcasting messages\n" \
" [-f |--logfile file]          log to a file\n" \
" [-i |--id id]                 the id of this agent (should be non-negative)\n" \
" [--madara-level level]        the MADARA logger level (0+, higher is higher detail)\n" \
" [--gams-level level]          the GAMS logger level (0+, higher is higher detail)\n" \
" [-L |--loop-time time]        time to execute loop\n"\
" [-m |--multicast ip:port]     the multicast ip to send and listen to\n" \
" [-M |--madara-file <file>]    file containing madara commands to execute\n" \
"                               multiple space-delimited files can be used\n" \
" [-n |--num_agents <number>]   the number of agents in the swarm\n" \
" [-o |--host hostname]         the hostname of this process (def:localhost)\n" \
" [-p |--platform type]         platform for loop (vrep, dronerk)\n" \
" [-P |--period period]         time, in seconds, between control loop executions\n" \
" [-q |--queue-length length]   length of transport queue in bytes\n" \
" [-r |--reduced]               use the reduced message header\n" \
" [-t |--target path]           file system location to save received files (NYI)\n" \
" [-u |--udp ip:port]           a udp ip to send to (first is self to bind to)\n" \
"\n",
        prog_name);
  exit (0);
}

// handle command line arguments
void handle_arguments (int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1 (argv[i]);

    if (arg1 == "-A" || arg1 == "--algorithm")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        algorithm = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-a" || arg1 == "--accent")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        accents.push_back (argv[i + 1]);
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-b" || arg1 == "--broadcast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::BROADCAST;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-d" || arg1 == "--domain")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        settings.write_domain = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-e" || arg1 == "--rebroadcasts")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        int hops;
        std::stringstream buffer (argv[i + 1]);
        buffer >> hops;

        settings.set_rebroadcast_ttl (hops);
        settings.enable_participant_ttl (hops);
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-f" || arg1 == "--logfile")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        madara::logger::global_logger->add_file (argv[i + 1]);
        gams::loggers::global_logger->add_file (argv[i + 1]);
      }
      else
        print_usage (argv[0]);

      ++i;
    }

    else if (arg1 == "-i" || arg1 == "--id")
    {
      if (i + 1 < argc && argv[i +1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> settings.id;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "--madara-level")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> madara_debug_level;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "--gams-level")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> gams_debug_level;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-L" || arg1 == "--loop-time")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> loop_time;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-m" || arg1 == "--multicast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::MULTICAST;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-M" || arg1 == "--madara-file")
    {
      bool files = false;
      ++i;
      for (;i < argc && argv[i][0] != '-'; ++i)
      {
        std::string filename = argv[i];
        if (madara::utility::file_exists (filename))
        {
          madara_commands += madara::utility::file_to_string (filename);
          madara_commands += ";\n";
          files = true;
        }
      }
      --i;

      if (!files)
        print_usage (argv[0]);
    }
    else if (arg1 == "-n" || arg1 == "--num_agents")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> num_agents;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-o" || arg1 == "--host")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        host = argv[i + 1];
      else
        print_usage (argv[0]);

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
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-P" || arg1 == "--period")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> period;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-q" || arg1 == "--queue-length")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> settings.queue_length;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-r" || arg1 == "--reduced")
    {
      settings.send_reduced_message_header = true;
    }
    else if (arg1 == "-t" || arg1 == "--target")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        file_path = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-u" || arg1 == "--udp")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::UDP;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else
    {
      print_usage (argv[0]);
    }
  }
}

// perform main logic of program
int main (int argc, char ** argv)
{
  // handle all user arguments
  handle_arguments (argc, argv);

  // set this once to allow for debugging knowledge base creation
  if (madara_debug_level >= 0)
  {
    madara::logger::global_logger->set_level (madara_debug_level);
  }

  // create knowledge base and a control loop
  madara::knowledge::KnowledgeBase knowledge (host, settings);

  // set this once to allow for debugging controller creation
  if (gams_debug_level >= 0)
  {
    gams::loggers::global_logger->set_level (gams_debug_level);
  }

  controllers::BaseController loop (knowledge);

  // initialize variables and function stubs
  loop.init_vars (settings.id, num_agents);
  
  // read madara initialization
  if (madara_commands != "")
  {
    knowledge.evaluate (madara_commands,
      madara::knowledge::EvalSettings(false, true));
  }

  // command line logging levels override madara commands from files
  if (madara_debug_level >= 0)
  {
    std::stringstream temp_buffer;
    temp_buffer << "agent." << settings.id << ".madara_debug_level = ";
    temp_buffer << madara_debug_level;

    madara::logger::global_logger->set_level (madara_debug_level);

    // modify the debug level being used but don't send out to others
    knowledge.evaluate (temp_buffer.str (),
      madara::knowledge::EvalSettings (true, true));
  }

  if (gams_debug_level >= 0)
  {
    std::stringstream temp_buffer;
    temp_buffer << "agent." << settings.id << ".gams_debug_level = ";
    temp_buffer << gams_debug_level;

    gams::loggers::global_logger->set_level (gams_debug_level);

    // modify the debug level being used but don't send out to others
    knowledge.evaluate (temp_buffer.str (),
      madara::knowledge::EvalSettings (true, true));
  }

  // initialize the platform and algorithm
  // default to platform in knowledge base if platform not set in command line
  if (!plat_set && knowledge.exists (KNOWLEDGE_BASE_PLATFORM_KEY))
    platform = knowledge.get (KNOWLEDGE_BASE_PLATFORM_KEY).to_string ();
  loop.init_platform (platform);
  loop.init_algorithm (algorithm);

  // add any accents
  for (unsigned int i = 0; i < accents.size (); ++i)
  {
    loop.init_accent (accents[i]);
  }

  // run a mape loop every 1s for 50s
  loop.run (period, loop_time);

  // print all knowledge values
  knowledge.print ();

  return 0;
}
