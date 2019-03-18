/**
 * Copyright (c) 2014-2016 Carnegie Mellon University. All Rights Reserved.
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

#include "gams/loggers/GlobalLogger.h"

#include "gams/algorithms/AlgorithmFactoryRepository.h"
#include "gams/algorithms/Land.h"
#include "gams/algorithms/Move.h"
#include "gams/algorithms/DebugAlgorithm.h"
#include "gams/algorithms/NullAlgorithm.h"
#include "gams/algorithms/FormationFlying.h"
#include "gams/algorithms/FormationCoverage.h"
#include "gams/algorithms/FormationSync.h"
#include "gams/algorithms/KarlEvaluator.h"
#include "gams/algorithms/ZoneCoverage.h"
#include "gams/algorithms/Takeoff.h"
#include "gams/algorithms/Greet.h"
#include "gams/algorithms/Home.h"
#include "gams/algorithms/Hold.h"
#include "gams/algorithms/Follow.h"
#include "gams/algorithms/MessageProfiling.h"
#include "gams/algorithms/PerimeterPatrol.h"
#include "gams/algorithms/Executor.h"
#include "gams/algorithms/Wait.h"
#include "gams/algorithms/PerformanceProfiling.h"
#include "gams/algorithms/GroupBarrier.h"
#include "gams/algorithms/Spell.h"

#include "gams/algorithms/area_coverage/UniformRandomAreaCoverage.h"
#include "gams/algorithms/area_coverage/UniformRandomEdgeCoverage.h"
#include "gams/algorithms/area_coverage/PriorityWeightedRandomAreaCoverage.h"
#include "gams/algorithms/area_coverage/SnakeAreaCoverage.h"
#include "gams/algorithms/area_coverage/PerimeterPatrolCoverage.h"
#include "gams/algorithms/area_coverage/WaypointsCoverage.h"

#if 0
#include "gams/algorithms/area_coverage/MinTimeAreaCoverage.h"
#include "gams/algorithms/area_coverage/PrioritizedMinTimeAreaCoverage.h"
#include "gams/algorithms/area_coverage/LocalPheremoneAreaCoverage.h"
#endif

#include <iostream>

namespace algorithms = gams::algorithms;
namespace variables = gams::variables;
namespace platforms = gams::platforms;

GAMS_EXPORT algorithms::AlgorithmFactoryRepository *
    algorithms::global_algorithm_factory() {
  static algorithms::AlgorithmFactoryRepository *algo_repo =
    new algorithms::AlgorithmFactoryRepository();
  return algo_repo;
}

algorithms::AlgorithmFactoryRepository::AlgorithmFactoryRepository (
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  platforms::BasePlatform * platform,
  variables::Self * self,
  variables::Agents * agents)
: agents_ (agents), knowledge_ (knowledge), platform_ (platform),
  self_ (self), sensors_ (sensors), init_started_(false),
  init_finished_(false)
{
}

algorithms::AlgorithmFactoryRepository::~AlgorithmFactoryRepository ()
{
}

void
algorithms::AlgorithmFactoryRepository::add (
  const std::vector <std::string> & aliases,
  AlgorithmFactory * factory)
{
  for (size_t i = 0; i < aliases.size (); ++i)
  {
    std::string alias (aliases[i]);
    madara::utility::lower (alias);

    factory->set_agents (agents_);
    factory->set_knowledge (knowledge_);
    factory->set_platform (platform_);
    factory->set_self (self_);
    factory->set_sensors (sensors_);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::AlgorithmFactoryRepository:add" \
      " Adding %s factory.\n", alias.c_str ());

    factory_map_[alias] = factory;
  }

  // if the user has added aliases, then we are technically initialized
  init_started_ = true;
  init_finished_ = true;
}

void algorithms::AlgorithmFactoryRepository::initialize_default_mappings (
  void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::AlgorithmFactoryRepository:initialize_default_mappings" \
    " Creating map of all default algorithm factories.\n");

  if (!init_started_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::AlgorithmFactoryRepository:initialize_default_mappings" \
      " Flag has not been set. Initializing.\n");

    std::vector <std::string> aliases;

    // the performance profiler
    aliases.resize (1);
    aliases[0] = "barrier";

    add (aliases, new GroupBarrierFactory ());

    // the debug algorithm
    aliases.resize (3);
    aliases[0] = "debug";
    aliases[1] = "print";
    aliases[2] = "printer";

    add (aliases, new DebugAlgorithmFactory ());

    // the executor
    aliases.resize (2);
    aliases[0] = "exec";
    aliases[1] = "executor";

    add (aliases, new ExecutorFactory ());

    // the follow algorithm
    aliases.resize (1);
    aliases[0] = "follow";

    add (aliases, new FollowFactory ());

    // the formation coverage algorithm
    aliases.resize (1);
    aliases[0] = "formation coverage";

    add (aliases, new FormationCoverageFactory ());

    // the formation algorithm
    aliases.resize (1);
    aliases[0] = "formation";

    add (aliases, new FormationFlyingFactory ());

    // the performance profiler
    aliases.resize (1);
    aliases[0] = "formation sync";

    add (aliases, new FormationSyncFactory ());

    // the performance profiler
    aliases.resize (1);
    aliases[0] = "hold";

    add (aliases, new HoldFactory ());

    // the performance profiler
    aliases.resize (2);
    aliases[0] = "home";
    aliases[1] = "return";

    add (aliases, new HomeFactory ());

    // the karl evaluator algorithm
    aliases.resize (1);
    aliases[0] = "karl";

    add (aliases, new KarlEvaluatorFactory ());

    // the message profiling algorithm
    aliases.resize (1);
    aliases[0] = "message profiling";

    add (aliases, new MessageProfilingFactory ());

    // the move algorithm
    aliases.resize (2);
    aliases[0] = "move";
    aliases[1] = "waypoints";

    add (aliases, new MoveFactory ());

    // the null algorithm
    aliases.resize (1);
    aliases[0] = "null";

    add (aliases, new NullAlgorithmFactory ());

    // the performance profiler
    aliases.resize (1);
    aliases[0] = "performance profiling";

    add (aliases, new PerformanceProfilingFactory ());

    // the perimeter patrol algorithm
    aliases.resize (3);
    aliases[0] = "patrol";
    aliases[1] = "perimeter patrol";
    aliases[2] = "pp";

    add (aliases, new PerimeterPatrolFactory ());

    // the takeoff algorithm
    aliases.resize (1);
    aliases[0] = "takeoff";

    add (aliases, new TakeoffFactory ());

    // the takeoff algorithm
    aliases.resize (1);
    aliases[0] = "land";

    add (aliases, new LandFactory ());

    // the snake area coverage algorithm
    aliases.resize (2);
    aliases[0] = "snake";
    aliases[1] = "sac";

    // the wait
    aliases.resize (1);
    aliases[0] = "wait";

    add (aliases, new WaitFactory ());

    // zone coverage
    aliases.resize (2);
    aliases[0] = "zone coverage";
    aliases[1] = "zone defense";

    add (aliases, new ZoneCoverageFactory ());

    // text spelling
    aliases.resize (2);
    aliases[0] = "text";
    aliases[1] = "spell";

    add (aliases, new SpellFactory ());

    // text spelling
    aliases.resize (2);
    aliases[0] = "greeting";
    aliases[1] = "greet";

    add (aliases, new GreetFactory ());

#if 0
    // the local pheromone coverage algorithm
    aliases.resize (1);
    aliases[0] = "local pheremone";

    add (aliases, new area_coverage::LocalPheremoneAreaCoverageFactory ());

    // the minimum time coverage algorithm
    aliases.resize (2);
    aliases[0] = "min time";
    aliases[1] = "mtac";

    add (aliases, new area_coverage::MinTimeAreaCoverageFactory ());

    // the prioritized min time area coverage
    aliases.resize (2);
    aliases[0] = "prioritized min time area coverage";
    aliases[1] = "pmtac";

    add (aliases, new area_coverage::PrioritizedMinTimeAreaCoverageFactory ());
#endif

    // the perimeter patrol algorithm
    aliases.resize (2);
    aliases[0] = "perimeter patrol area coverage";
    aliases[1] = "ppac";

    add (aliases, new area_coverage::PerimeterPatrolCoverageFactory ());

    // the priority-weighted coverage algorithm
    aliases.resize (2);
    aliases[0] = "priority weighted random area coverage";
    aliases[1] = "pwrac";

    add (aliases, new area_coverage::PriorityWeightedRandomAreaCoverageFactory ());

    add (aliases, new area_coverage::SnakeAreaCoverageFactory ());

    // the uniform random area coverage algorithm
    aliases.resize (2);
    aliases[0] = "uniform random area coverage";
    aliases[1] = "urac";

    add (aliases, new area_coverage::UniformRandomAreaCoverageFactory ());

    // the uniform random edge coverage algorithm
    aliases.resize (2);
    aliases[0] = "uniform random edge coverage";
    aliases[1] = "urec";

    add (aliases, new area_coverage::UniformRandomEdgeCoverageFactory ());

    // the waypoints coverage algorithm
    aliases.resize (2);
    aliases[0] = "waypoints coverage";
    aliases[1] = "waypoints_coverage";

    add (aliases, new area_coverage::WaypointsCoverageFactory ());

    init_finished_ = true;
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::AlgorithmFactoryRepository:initialize_default_mappings" \
      " Flag has already been set. No need to initialize.\n");
  }
}

algorithms::BaseAlgorithm *
algorithms::AlgorithmFactoryRepository::create (
  const std::string & type,
  const madara::knowledge::KnowledgeMap & args)
{
  algorithms::BaseAlgorithm * result = 0;

  // the user is going to expect this kind of error to be printed immediately
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::AlgorithmFactoryRepository::create:" \
    " creating \"%s\" algorithm.\n", type.c_str ());

  
  if (type != "" && init_finished_)
  {
    std::string lowercased_type (type);
    madara::utility::lower (lowercased_type);

    AlgorithmFactoryMap::iterator it = factory_map_.find (lowercased_type);
    if (it != factory_map_.end ())
    {
      result = it->second->create (args, knowledge_, platform_,
        sensors_, self_, agents_);
    }
    else
    {
      // the user is going to expect this kind of error to be printed immediately
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ALWAYS,
        "gams::algorithms::AlgorithmFactoryRepository::create:" \
        " could not find \"%s\" algorithm.\n", lowercased_type.c_str ());
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::AlgorithmFactoryRepository:create" \
      " ERROR: create called before initialize is complete.\n");
  }

  return result;
}

void
algorithms::AlgorithmFactoryRepository::set_agents (
  variables::Agents * agents)
{
  agents_ = agents;
}

void
algorithms::AlgorithmFactoryRepository::set_knowledge (
  madara::knowledge::KnowledgeBase * knowledge)
{
  knowledge_ = knowledge;
}

void
algorithms::AlgorithmFactoryRepository::set_platform (
  platforms::BasePlatform * platform)
{
  platform_ = platform;
}

void
algorithms::AlgorithmFactoryRepository::set_self (
  variables::Self * self)
{
  self_ = self;
}

void
algorithms::AlgorithmFactoryRepository::set_sensors (
  variables::Sensors * sensors)
{
  sensors_ = sensors;
}
