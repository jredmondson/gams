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

#include "gams/algorithms/area_coverage/UniformRandomAreaCoverage.h"

#include "gams/utility/ArgumentParser.h"

using std::string;

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::UniformRandomAreaCoverageFactory::create (
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
    "gams::algorithms::area_coverage::UniformRandomAreaCoverageFactory:" \
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
        "gams::algorithms::area_coverage::UniformRandomAreaCoverageFactory::create:" \
        " No search area specified. Returning null.\n");
    }
    else
    {
      result = new area_coverage::UniformRandomAreaCoverage (
        search_area, ACE_Time_Value (time),
        knowledge, platform, sensors, self, agents);
    }
  }

  return result;
}

gams::algorithms::area_coverage::UniformRandomAreaCoverage::
  UniformRandomAreaCoverage (
  const string& search_area_id,
  const ACE_Time_Value& e_time,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents) :
  BaseAreaCoverage (knowledge, platform, sensors, self, agents, e_time)
{
  // init status vars
  status_.init_vars (*knowledge, "urac", self->id.to_integer ());
  status_.init_variable_values ();

  // get region to cover
  utility::SearchArea search;
  search.from_container (*knowledge, search_area_id);
  region_ = search.get_convex_hull ();

  // generate initial waypoint
  generate_new_position();
}

gams::algorithms::area_coverage::UniformRandomAreaCoverage::
  ~UniformRandomAreaCoverage ()
{
}

void
gams::algorithms::area_coverage::UniformRandomAreaCoverage::operator= (
  const UniformRandomAreaCoverage & rhs)
{
  if (this != &rhs)
  {
    this->BaseAreaCoverage::operator= (rhs);
    this->region_ = rhs.region_;
  }
}

/**
 * Random positions are generated in the bounding rectangle of the region and
 * checked if it is actually in the region. In theory this could never happen
 * when given any arbitrary region. The average selection time is 
 * area(bounding_box) / area(region). In practice, this finds a valid position
 * quickly enough to not affect the system.
 */
void
gams::algorithms::area_coverage::UniformRandomAreaCoverage::
  generate_new_position ()
{
  do
  {
    next_position_.latitude (madara::utility::rand_double (region_.min_lat_,
      region_.max_lat_));
    next_position_.longitude (madara::utility::rand_double (region_.min_lon_,
      region_.max_lon_));
    next_position_.altitude (madara::utility::rand_double (region_.min_alt_,
      region_.max_alt_));
  }
  while (!region_.contains (next_position_));

  // found an acceptable position, so set it as next
  utility::GPSPosition current;
  current.from_container (self_->agent.location);
  next_position_.altitude (current.altitude ()); // TODO: update when altitude is handled
}
