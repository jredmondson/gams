/**
 * Copyright(c) 2014 Carnegie Mellon University. All Rights Reserved.
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
 * @file PerimeterPatrolCoverage.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Defines a Perimeter Patrol path for agents to follow. Used as tutorial for
 * creating custom area coverage algorithms. If this file is updated, please
 * update the wiki as well.
 *
 * NOTE: the Area Coverage algorithms currently use the deprecated
 * utility::Position classes, and should not be used as examples.
 */

#include "gams/loggers/GlobalLogger.h"
#include "gams/algorithms/area_coverage/PerimeterPatrolCoverage.h"

#include <vector>

#include "gams/utility/GPSPosition.h"
#include "gams/pose/Region.h"
#include "gams/pose/SearchArea.h"
#include "gams/utility/Position.h"

#include "gams/utility/ArgumentParser.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap    KnowledgeMap;

using std::vector;
using std::string;

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::PerimeterPatrolCoverageFactory::create(
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents)
{
  BaseAlgorithm * result(0);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::area_coverage::PerimeterPatrolCoverageFactory:" \
    " entered create with %u args\n", args.size());

  if (knowledge && sensors && platform && self)
  {
    std::string search_area;
    double time = 360;

    for (KnowledgeMap::const_iterator i = args.begin(); i != args.end(); ++i)
    {
      if (i->first.size() <= 0)
        continue;

      switch (i->first[0])
      {
      case 'a':
        if (i->first == "area")
        {
          search_area = i->second.to_string();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::PerimeterPatrolCoverageFactory:" \
            " setting search_area to %s\n", search_area.c_str());
          break;
        }
        goto unknown;
      case 's':
        if (i->first == "search_area")
        {
          search_area = i->second.to_string();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::PerimeterPatrolCoverageFactory:" \
            " setting search_area to %s\n", search_area.c_str());
          break;
        }
        goto unknown;
      case 't':
        if (i->first == "time")
        {
          time = i->second.to_double();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::PerimeterPatrolCoverageFactory:" \
            " setting time to %f\n", time);
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::PerimeterPatrolCoverageFactory:" \
          " argument unknown: %s -> %s\n",
          i->first.c_str(), i->second.to_string().c_str());
        break;
      }
    }

    // if group has not been set, use the swarm
    if (search_area == "")
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::area_coverage::PerimeterPatrolCoverageFactory::create:" \
        " No search area specified. Returning null.\n");
    }
    else
    {
      result = new area_coverage::PerimeterPatrolCoverage(
        search_area, time,
        knowledge, platform, sensors, self, agents);
    }
  }

  return result;
}

/**
 * Perimeter patrol is a precomputed algorithm. The agent traverses the vertices
 * of the region in order
 */
gams::algorithms::area_coverage::PerimeterPatrolCoverage::PerimeterPatrolCoverage(
  const string & region_id, double e_time, 
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents) :
  BaseAreaCoverage(knowledge, platform, sensors, self, agents, e_time),
  region_id_(region_id)
{
  // initialize some status variables
  status_.init_vars(*knowledge, "ppac", self->agent.prefix);
  status_.init_variable_values();

  if (*platform->get_platform_status()->ok)
  {
    generate_positions();
  }
}

gams::algorithms::area_coverage::PerimeterPatrolCoverage::~PerimeterPatrolCoverage()
{
}

void
gams::algorithms::area_coverage::PerimeterPatrolCoverage::generate_positions(void)
{
  if (platform_ && *platform_->get_platform_status()->movement_available)
  {
    // get waypoints
    pose::SearchArea sa;
    sa.from_container(*knowledge_, region_id_);
    pose::Region reg = sa.get_convex_hull();
    vector<utility::GPSPosition> vertices;
    vertices.reserve(reg.vertices.size());
    for (pose::Position vert : reg.vertices)
    {
      vertices.push_back(utility::GPSPosition(
        vert.transform_to(pose::gps_frame())));
    }

    // find closest waypoint as starting point
    size_t closest = 0;
    utility::GPSPosition current;
    current.from_container(self_->agent.location);
    double min_distance = current.distance_to(vertices[0]);
    for (size_t i = 1; i < vertices.size(); ++i)
    {
      double dist = current.distance_to(vertices[i]);
      if (min_distance > dist)
      {
        dist = min_distance;
        closest = i;
      }
    }

    // add some intermediate points
    const size_t NUM_INTERMEDIATE_PTS = 1;
    for (size_t i = 0; i < vertices.size(); ++i)
    {
      utility::GPSPosition start = vertices[(i + closest) % vertices.size()];
      utility::GPSPosition end = vertices[(i + closest + 1) % vertices.size()];
      double lat_dif = start.latitude() - end.latitude();
      double lon_dif = start.longitude() - end.longitude();

      for (size_t j = 0; j < NUM_INTERMEDIATE_PTS; ++j)
      {
        utility::GPSPosition temp(
          start.latitude() - lat_dif * j / NUM_INTERMEDIATE_PTS,
          start.longitude() - lon_dif * j / NUM_INTERMEDIATE_PTS,
          self_->agent.desired_altitude.to_double());
        waypoints_.push_back(temp);
      }
    }

    // set next_position_
    cur_waypoint_ = 0;
    next_position_ = waypoints_[cur_waypoint_];

    initialized_ = true;
  }
}


void
gams::algorithms::area_coverage::PerimeterPatrolCoverage::operator=(
  const PerimeterPatrolCoverage& rhs)
{
  if (this != &rhs)
  {
    this->BaseAreaCoverage::operator=(rhs);
    this->waypoints_ = rhs.waypoints_;
    this->cur_waypoint_ = rhs.cur_waypoint_;
  }
}

/**
 * The next destination is the next vertex in the list
 */
void
gams::algorithms::area_coverage::PerimeterPatrolCoverage::generate_new_position(void)
{
  if (initialized_)
  {
    cur_waypoint_ =(cur_waypoint_ + 1) % waypoints_.size();
    next_position_ = waypoints_[cur_waypoint_];
  }
  else
  {
    generate_positions();
  }
}
