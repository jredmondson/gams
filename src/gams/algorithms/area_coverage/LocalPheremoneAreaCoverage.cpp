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
 * @file LocalPheremoneAreaCoverage.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Agents mark a sensor with their time when they enter a discretized cell of a 
 * region. At each time step, they select the neighboring cell with the lowest 
 * pheremone reading as their next destination.
 **/

#include "gams/loggers/GlobalLogger.h"
#include "gams/algorithms/area_coverage/LocalPheremoneAreaCoverage.h"

#include "madara/utility/Utility.h"

#include "gams/utility/ArgumentParser.h"

#include <iostream>
using std::cerr;
using std::endl;
#include <vector>
using std::vector;

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::LocalPheremoneAreaCoverageFactory::create (
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
    "gams::algorithms::area_coverage::LocalPheremoneAreaCoverageFactory:" \
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
        "gams::algorithms::area_coverage::LocalPheremoneAreaCoverageFactory::create:" \
        " No search area specified. Returning null.\n");
    }
    else
    {
      result = new area_coverage::LocalPheremoneAreaCoverage (
        search_area, ACE_Time_Value (time),
        knowledge, platform, sensors, self, agents);
    }
  }

  return result;
}

gams::algorithms::area_coverage::LocalPheremoneAreaCoverage::
LocalPheremoneAreaCoverage (
  const std::string& search_id,
  const ACE_Time_Value& e_time,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents) :
  BaseAreaCoverage (knowledge, platform, sensors, self, agents, e_time),
  pheremone_ (search_id + ".pheremone", knowledge)
{
  // init status vars
  status_.init_vars (*knowledge, "lpac", self->id.to_integer ());
  status_.init_variable_values ();

  // get search area
  search_area_.from_container (*knowledge, search_id);

  // fill out pheremone sensor
  /**
   * See MinTimeAreaCoverage.cpp for why this needs to be updated later
   */
  utility::GPSPosition origin;
  madara::knowledge::containers::NativeDoubleArray origin_container;
  origin_container.set_name ("sensor.coverage.origin", *knowledge, 3);
  origin.from_container (origin_container);
  pheremone_.set_origin (origin);
  pheremone_.set_range (5.0);
  
  // generate first position to move
  generate_new_position ();
}

void
gams::algorithms::area_coverage::LocalPheremoneAreaCoverage::operator= (
  const LocalPheremoneAreaCoverage & rhs)
{
  if (this != &rhs)
  {
    this->search_area_ = rhs.search_area_;
    this->pheremone_ = rhs.pheremone_;
    this->BaseAreaCoverage::operator= (rhs);
  }
}

void
gams::algorithms::area_coverage::LocalPheremoneAreaCoverage::
  generate_new_position ()
{
  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    // get current location
    utility::GPSPosition cur_gps;
    cur_gps.from_container (self_->agent.location);

    // create possible next positions
    const int num_possible = 12;
    utility::Position possible[num_possible];
    utility::Position cur = pheremone_.get_index_from_gps (cur_gps);
    vector<unsigned int> selection;
    for (unsigned int i = 0; i < num_possible; ++i)
    {
      possible[i] = cur;
      selection.push_back (i);
    }
    ++possible[0].x; ++possible[0].y;
    ++possible[1].x;
    ++possible[2].x; --possible[2].y;
    --possible[3].y;
    --possible[4].x; --possible[4].y;
    --possible[5].x;
    --possible[6].x; ++possible[6].y;
    ++possible[7].y;

    /**
     * We consider the possibility that the agent drifts outside the actual area
     * of operation. A more robust way to do this would be to find the closest
     * cell in the area and go to that, however, this is simpler and has not
     * failed in simulation. Real-world experiments are likely to differ and may
     * require the more robust solution.
     */
    possible[8].x += 2;
    possible[9].y += 2;
    possible[10].x -= 2;
    possible[11].y -= 2;

    // find lowest pheremone concentration of possible coords in search_area
    utility::GPSPosition lowest = cur_gps;
    utility::Position s;
    double concentration = DBL_MAX;
    std::random_shuffle (selection.begin (), selection.end ());
    for (unsigned int i = 0; i < num_possible; ++i)
    {
      const int index = selection[i]; // get randomized index

      // update executions value if necessary
      const double my_concentration = pheremone_.get_value (possible[index]);
      if (my_concentration > executions_)
        executions_ = my_concentration;

      // check if new min found and update if necessary
      if (concentration > my_concentration)
      {
        utility::GPSPosition possible_gps =
          pheremone_.get_gps_from_index (possible[index]);
        if (search_area_.contains (possible_gps))
        {
          {
            concentration = my_concentration;
            lowest = possible_gps;
            s = possible[index];
          }
        }
      }
    }

    // update pheremone value
    pheremone_.set_value (lowest, executions_ + 1);

    // assign new next
    // TODO: fix with proper altitude
    lowest.altitude (self_->agent.desired_altitude.to_double ());
    next_position_ = lowest;

    initialized_ = true;
  }
}
