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

#include "madara/utility/Utility.h"

#include "gams/loggers/Global_Logger.h"

#include "gams/algorithms/Controller_Algorithm_Factory.h"
#include "gams/algorithms/Land.h"
#include "gams/algorithms/Move.h"
#include "gams/algorithms/Debug_Algorithm.h"
#include "gams/algorithms/Null_Algorithm.h"
#include "gams/algorithms/Formation_Flying.h"
#include "gams/algorithms/Formation_Coverage.h"
#include "gams/algorithms/Takeoff.h"
#include "gams/algorithms/Follow.h"
#include "gams/algorithms/Message_Profiling.h"
#include "gams/algorithms/Executive.h"
#include "gams/algorithms/Wait.h"
#include "gams/algorithms/Performance_Profiling.h"

#include "gams/algorithms/area_coverage/Uniform_Random_Area_Coverage.h"
#include "gams/algorithms/area_coverage/Uniform_Random_Edge_Coverage.h"
#include "gams/algorithms/area_coverage/Priority_Weighted_Random_Area_Coverage.h"
#include "gams/algorithms/area_coverage/Local_Pheremone_Area_Coverage.h"
#include "gams/algorithms/area_coverage/Snake_Area_Coverage.h"
#include "gams/algorithms/area_coverage/Min_Time_Area_Coverage.h"
#include "gams/algorithms/area_coverage/Prioritized_Min_Time_Area_Coverage.h"
#include "gams/algorithms/area_coverage/Perimeter_Patrol.h"
#include "gams/algorithms/area_coverage/Waypoints_Coverage.h"

#include "gams/loggers/Global_Logger.h"

#include <iostream>

namespace algorithms = gams::algorithms;
namespace variables = gams::variables;
namespace platforms = gams::platforms;

algorithms::Controller_Algorithm_Factory::Controller_Algorithm_Factory (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  platforms::Base_Platform * platform,
  variables::Self * self,
  variables::Devices * devices)
: devices_ (devices), knowledge_ (knowledge), platform_ (platform),
  self_ (self), sensors_ (sensors)
{
  initialize_default_mappings ();
}

algorithms::Controller_Algorithm_Factory::~Controller_Algorithm_Factory ()
{
}

void
algorithms::Controller_Algorithm_Factory::add (
  const std::vector <std::string> & aliases,
  Algorithm_Factory * factory)
{
  for (size_t i = 0; i < aliases.size (); ++i)
  {
    std::string alias (aliases[i]);
    Madara::Utility::lower (alias);

    factory->set_devices (devices_);
    factory->set_knowledge (knowledge_);
    factory->set_platform (platform_);
    factory->set_self (self_);
    factory->set_sensors (sensors_);

    factory_map_[alias] = factory;
  }
}

void algorithms::Controller_Algorithm_Factory::initialize_default_mappings (
  void)
{
  std::vector <std::string> aliases;
  
  // the debug algorithm
  aliases.resize (3);
  aliases[0] = "debug";
  aliases[1] = "print";
  aliases[2] = "printer";

  add (aliases, new Debug_Algorithm_Factory ());

  // the follow algorithm
  aliases.resize (1);
  aliases[0] = "follow";

  add (aliases, new Follow_Factory ());

  // the formation coverage algorithm
  aliases.resize (1);
  aliases[0] = "formation coverage";

  add (aliases, new Formation_Coverage_Factory ());
  
  // the formation algorithm
  aliases.resize (1);
  aliases[0] = "formation";

  add (aliases, new Formation_Flying_Factory ());
  
  // the move algorithm
  aliases.resize (1);
  aliases[0] = "move";

  add (aliases, new Move_Factory ());
  
  // the null algorithm
  aliases.resize (1);
  aliases[0] = "null";

  add (aliases, new Null_Algorithm_Factory ());
  
  // the takeoff algorithm
  aliases.resize (1);
  aliases[0] = "takeoff";

  add (aliases, new Takeoff_Factory ());
  
  // the local pheromone coverage algorithm
  aliases.resize (1);
  aliases[0] = "local pheremone";

  add (aliases, new area_coverage::Local_Pheremone_Area_Coverage_Factory ());
  
  // the minimum time coverage algorithm
  aliases.resize (2);
  aliases[0] = "min time";
  aliases[1] = "mtac";

  add (aliases, new area_coverage::Min_Time_Area_Coverage_Factory ());
  
  // the minimum time coverage algorithm
  aliases.resize (2);
  aliases[0] = "perimeter patrol";
  aliases[1] = "ppac";

  add (aliases, new area_coverage::Perimeter_Patrol_Factory ());
  
  // the priority-weighted coverage algorithm
  aliases.resize (2);
  aliases[0] = "priority weighted random area coverage";
  aliases[1] = "pwrac";

  add (aliases, new area_coverage::Priority_Weighted_Random_Area_Coverage_Factory ());
  
  // the snake area coverage algorithm
  aliases.resize (2);
  aliases[0] = "snake";
  aliases[1] = "sac";

  add (aliases, new area_coverage::Snake_Area_Coverage_Factory ());
  
  // the uniform random area coverage algorithm
  aliases.resize (2);
  aliases[0] = "uniform random area coverage";
  aliases[1] = "urac";

  add (aliases, new area_coverage::Uniform_Random_Area_Coverage_Factory ());
  
  // the uniform random edge coverage algorithm
  aliases.resize (2);
  aliases[0] = "uniform random edge coverage";
  aliases[1] = "urec";

  add (aliases, new area_coverage::Uniform_Random_Edge_Coverage_Factory ());

  // the prioritized min time area coverage
  aliases.resize (2);
  aliases[0] = "prioritized min time area coverage";
  aliases[1] = "pmtac";

  add (aliases, new area_coverage::Prioritized_Min_Time_Area_Coverage_Factory ());

  // the message profiling algorithm
  aliases.resize (1);
  aliases[0] = "message profiling";

  add (aliases, new Message_Profiling_Factory ());

  // the waypoints coverage algorithm
  aliases.resize (1);
  aliases[0] = "waypoints";

  add (aliases, new area_coverage::Waypoints_Coverage_Factory ());

  // the executive
  aliases.resize (1);
  aliases[0] = "executive";

  add (aliases, new Executive_Factory ());

  // the wait
  aliases.resize (1);
  aliases[0] = "wait";

  add (aliases, new Wait_Factory ());

  // the performance profiler
  aliases.resize (1);
  aliases[0] = "performance profiling";

  add (aliases, new Performance_Profiling_Factory ());
}

algorithms::Base_Algorithm *
algorithms::Controller_Algorithm_Factory::create (
  const std::string & type,
  const Madara::Knowledge_Vector & args)
{
  algorithms::Base_Algorithm * result = 0;
  
  if (type != "")
  {
    Factory_Map::iterator it = factory_map_.find (type);
    if (it != factory_map_.end ())
    {
      result = it->second->create (args, knowledge_, platform_,
        sensors_, self_, devices_);
    }
    else
    {
      // the user is going to expect this kind of error to be printed immediately
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ALWAYS,
        "gams::algorithms::Controller_Algorithm_Factory::create:" \
        " could not find \"%s\" algorithm.\n", type.c_str ());
    }
  }

  return result;
}

void
algorithms::Controller_Algorithm_Factory::set_devices (
  variables::Devices * devices)
{
  devices_ = devices;
}

void
algorithms::Controller_Algorithm_Factory::set_knowledge (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge)
{
  knowledge_ = knowledge;
}

void
algorithms::Controller_Algorithm_Factory::set_platform (
  platforms::Base_Platform * platform)
{
  platform_ = platform;
}

void
algorithms::Controller_Algorithm_Factory::set_self (
  variables::Self * self)
{
  self_ = self;
}

void
algorithms::Controller_Algorithm_Factory::set_sensors (
  variables::Sensors * sensors)
{
  sensors_ = sensors;
}
