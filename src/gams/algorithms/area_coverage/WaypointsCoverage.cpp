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

#include "gams/algorithms/area_coverage/WaypointsCoverage.h"

#include <cmath>
#include <string>
#include <vector>

#include "gams/loggers/GlobalLogger.h"

#include "gams/utility/ArgumentParser.h"

using std::string;
using std::vector;

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::WaypointsCoverageFactory::create (
  const madara::knowledge::KnowledgeMap & map,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result (0);
  
  if (knowledge && sensors && self && map.size () >= 1)
  {
    // Use a dumb workaround for now; TODO: convert this algo to use the map
    using namespace madara::knowledge;
    KnowledgeVector args(utility::kmap2kvec(map));

    std::vector<utility::Position> waypoints;
    bool error = false;

    for (size_t i = 0; i < args.size (); ++i)
    {
      vector <double> coords = args[i].to_doubles ();
      utility::Position w;
      if (coords.size () == 2)
      {
        w.x = coords[0];
        w.y = coords[1];
        w.z = 2;
      }
      else if (coords.size () == 3)
      {
        w.x = coords[0];
        w.y = coords[1];
        w.z = coords[2];
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::area_coverage::WaypointCoverageFactory:" \
          " arg %u is of invalid size %u\n", i, coords.size ());
        error = true;
      }

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::area_coverage::WaypointCoverageFactory:" \
        " waypoint %u is \"%f,%f,%f\"\n", i, w.x, w.y, w.z);
      waypoints.push_back (w);
    }

    if (!error)
    {
      result = new area_coverage::WaypointsCoverage (waypoints, 
        knowledge, platform, sensors, self);
    }
  }

  return result;
}

/**
 * WaypointsCoverage is a precomputed area coverage algorithm. The agent
 * traverses the waypoints until reaching the end
 */
gams::algorithms::area_coverage::WaypointsCoverage::WaypointsCoverage (
  const std::vector<utility::Position>& waypoints,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents) :
  BaseAreaCoverage (knowledge, platform, sensors, self, agents),
  waypoints_(waypoints), cur_waypoint_ (0)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::WaypointCoverage:" \
    " init_vars\n");

  status_.init_vars (*knowledge, "waypoints", self->id.to_integer ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::WaypointCoverage:" \
    " init_variable_values\n");

  status_.init_variable_values ();

  next_position_ = waypoints_[cur_waypoint_];
}

gams::algorithms::area_coverage::WaypointsCoverage::~WaypointsCoverage ()
{
}

void
gams::algorithms::area_coverage::WaypointsCoverage::operator= (
  const WaypointsCoverage& rhs)
{
  if (this != &rhs)
  {
    this->waypoints_ = rhs.waypoints_;
    this->cur_waypoint_ = rhs.cur_waypoint_;
    this->BaseAreaCoverage::operator= (rhs);
  }
}

int
gams::algorithms::area_coverage::WaypointsCoverage::analyze ()
{
  int ret_val (OK);

  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    if (cur_waypoint_ >= waypoints_.size ())
    {
      ret_val = FINISHED;
      status_.finished = 1;
    }
  }

  return ret_val;
}

/**
 * The next destination is simply the next point in the list
 */
void
gams::algorithms::area_coverage::WaypointsCoverage::generate_new_position ()
{
  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    ++cur_waypoint_;
    if (cur_waypoint_ < waypoints_.size ())
      next_position_ = waypoints_[cur_waypoint_];
    else
      cur_waypoint_ = waypoints_.size (); // prevent overflow to become valid again
    initialized_ = true;
  }
}
