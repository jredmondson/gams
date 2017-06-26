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
 * @file PrioritizedMinTimeAreaCoverage.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Agents mark a sensor with their time when they enter a discretized cell of a 
 * region. Each agent selects a destination coordinate which provides the 
 * highest increase in sensor utility determined by time since last observation.
 *
 * NOTE: the Area Coverage algorithms currently use the deprecated
 * utility::Position classes, and should not be used as examples.
 *
 * Disabled pending repair of the variables::Sensor class
 **/

#if 0

#include "gams/loggers/GlobalLogger.h"
#include "gams/algorithms/area_coverage/PrioritizedMinTimeAreaCoverage.h"

#include <string>
using std::string;
#include <iostream>
using std::cerr;
using std::endl;
#include <cmath>
#include <set>
using std::set;

#include "gams/utility/GPSPosition.h"
#include "gams/utility/Position.h"

#include "gams/utility/ArgumentParser.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::PrioritizedMinTimeAreaCoverageFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents)
{
  BaseAlgorithm * result (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::area_coverage::PrioritizedMinTimeAreaCoverageFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    std::string search_area;
    double time = 360;

    for (KnowledgeMap::const_iterator i = args.begin (); i != args.end (); ++i)
    {
      if (i->first.size () <= 0)
        continue;

      switch (i->first[0])
      {
      case 'a':
        if (i->first == "area")
        {
          search_area = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
            " setting search_area to %s\n", search_area.c_str ());
          break;
        }
        goto unknown;
      case 's':
        if (i->first == "search_area")
        {
          search_area = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
            " setting search_area to %s\n", search_area.c_str ());
          break;
        }
        goto unknown;
      case 't':
        if (i->first == "time")
        {
          time = i->second.to_double ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
            " setting time to %f\n", time);
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::FormationSyncFactory:" \
          " argument unknown: %s -> %s\n",
          i->first.c_str (), i->second.to_string ().c_str ());
        break;
      }
    }

    // if group has not been set, use the swarm
    if (search_area == "")
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::area_coverage::PrioritizedMinTimeAreaCoverageFactory::create:" \
        " No search area specified. Returning null.\n");
    }
    else
    {
      result = new area_coverage::PrioritizedMinTimeAreaCoverage (
        search_area, ACE_Time_Value (time),
        knowledge, platform, sensors, self, agents);
    }
  }

  return result;
}

gams::algorithms::area_coverage::PrioritizedMinTimeAreaCoverage::
  PrioritizedMinTimeAreaCoverage (
  const string& search_id,
  const ACE_Time_Value& e_time, 
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents,
  const string& algo_name) :
  MinTimeAreaCoverage (search_id, e_time, knowledge, platform, sensors, self, agents, algo_name)
{
}

void
gams::algorithms::area_coverage::PrioritizedMinTimeAreaCoverage::operator= (
  const PrioritizedMinTimeAreaCoverage & rhs)
{
  this->MinTimeAreaCoverage::operator= (rhs);
}

double
gams::algorithms::area_coverage::PrioritizedMinTimeAreaCoverage::get_utility (
  const utility::Position& start, const utility::Position& end,
  set<utility::Position>& online)
{
  /**
   * check each valid position and add its value to utility if it is along
   * the possible travel path of the agent
   */
  double util = 0.0;
  const double radius =
    min_time_.get_range () / min_time_.get_discretization ();
  for (set<utility::Position>::const_iterator it = valid_positions_.begin ();
    it != valid_positions_.end (); ++it)
  {
    if (start.distance_to_2d (end, *it) < radius)
    {
      const utility::GPSPosition gps = min_time_.get_gps_from_index (*it);
      double time = min_time_.get_value (*it) * search_area_.get_priority (gps);
      double delta_util = pow (time, 3.0);
      util += delta_util;
      online.insert (*it);
    }
  }
  
  // modify the utility based on the distance that will be travelled
  util = util / sqrt (start.distance_to_2d (end) + 1);
  return util;
}

#endif
