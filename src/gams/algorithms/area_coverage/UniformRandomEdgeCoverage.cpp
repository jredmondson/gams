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
 *
 * NOTE: the Area Coverage algorithms currently use the deprecated
 * utility::Position classes, and should not be used as examples.
 **/

#include "gams/algorithms/area_coverage/UniformRandomEdgeCoverage.h"
using std::string;

#include "gams/utility/ArgumentParser.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create(
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
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory:" \
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
            "gams::algorithms::FormationSyncFactory:" \
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
            "gams::algorithms::FormationSyncFactory:" \
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
            "gams::algorithms::FormationSyncFactory:" \
            " setting time to %f\n", time);
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::FormationSyncFactory:" \
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
        "gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create:" \
        " No search area specified. Returning null.\n");
    }
    else
    {
      result = new area_coverage::UniformRandomEdgeCoverage(
        search_area, time,
        knowledge, platform, sensors, self, agents);
    }
  }

  return result;
}

gams::algorithms::area_coverage::
UniformRandomEdgeCoverage::UniformRandomEdgeCoverage(
  const string& prefix,
  double e_time,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents) :
  BaseAreaCoverage(knowledge, platform, sensors, self, agents, e_time)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " entered constructor\n");

  // init status vars
  status_.init_vars(*knowledge, "urec", self->agent.prefix);
  status_.init_variable_values();

  // generate search region
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " parsing SearchArea \"%s\"\n", prefix.c_str());
  pose::SearchArea search;
  search.from_container(*knowledge, prefix);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " SearchArea \"%s\" is \"%s\"\n", prefix.c_str(), search.to_string().c_str());

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " getting convex hull of \"%s\"\n", prefix.c_str());
  
  region_ = search.get_convex_hull();

  // generate initial waypoint
  generate_new_position();

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " finished constructing algorithm\n");
}

gams::algorithms::area_coverage::UniformRandomEdgeCoverage::~UniformRandomEdgeCoverage()
{
}

void
gams::algorithms::area_coverage::UniformRandomEdgeCoverage::operator=(
  const UniformRandomEdgeCoverage & rhs)
{
  if (this != &rhs)
  {
    this->region_ = rhs.region_;
    this->BaseAreaCoverage::operator=(rhs);
  }
}

/**
 * An edge is selected at uniform random and then a point on that edge is
 * selected at uniform random.
 */
void
gams::algorithms::area_coverage::UniformRandomEdgeCoverage::generate_new_position(void)
{
  if (platform_ && *platform_->get_platform_status()->movement_available)
  {
    // select new edge
    int num_edges = int(region_.vertices.size());
    int target_edge =(int)madara::utility::rand_int(0, num_edges - 1);

    // get endpoints
    const utility::GPSPosition pos_1(region_.vertices[target_edge].transform_to(pose::gps_frame()));
    const utility::GPSPosition pos_2(region_.vertices[(target_edge + 1) % num_edges].transform_to(pose::gps_frame()));

    // get random point on line
    double delta_lat = pos_2.latitude() - pos_1.latitude();
    double delta_lon = pos_2.longitude() - pos_1.longitude();
    if (delta_lon == 0) // north/south line
    {
      const double & min = pos_1.latitude() < pos_2.latitude() ? pos_1.latitude() : pos_2.latitude();
      const double & max = pos_1.latitude() > pos_2.latitude() ? pos_1.latitude() : pos_2.latitude();
      next_position_.latitude(madara::utility::rand_double(min, max));
      next_position_.longitude(pos_1.longitude());
    }
    else if (delta_lat == 0) // east/west line
    {
      const double & min = pos_1.longitude() < pos_2.longitude() ? pos_1.longitude() : pos_2.longitude();
      const double & max = pos_1.longitude() > pos_2.longitude() ? pos_1.longitude() : pos_2.longitude();
      next_position_.longitude(madara::utility::rand_double(min, max));
      next_position_.latitude(pos_1.latitude());
    }
    else // other arbitrary line
    {
      const double slope = delta_lon / delta_lat;
      next_position_.latitude(madara::utility::rand_double(pos_1.latitude(), pos_2.latitude()));
      next_position_.longitude(pos_1.longitude() + slope *(next_position_.latitude() - pos_1.latitude()));
    }

    // fill in altitude on waypoint
    next_position_.altitude(self_->agent.desired_altitude.to_double());
  }
}
