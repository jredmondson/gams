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
 * @file PriorityWeightedRandomAreaCoverage.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Prioritized Random Area Coverage prioritizes certain regions of a search area
 * based on specified priorities
 **/

#include "gams/algorithms/area_coverage/PriorityWeightedRandomAreaCoverage.h"

#include <iostream>
#include <vector>

#include "gams/utility/ArgumentParser.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverageFactory::create (
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
    "gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverageFactory:" \
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
            "gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverageFactory:" \
            " setting search_area to %s\n", search_area.c_str ());
        }
        break;
      case 's':
        if (i->first == "search_area")
        {
          search_area = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverageFactory:" \
            " setting search_area to %s\n", search_area.c_str ());
        }
        break;
      case 't':
        if (i->first == "time")
        {
          time = i->second.to_double ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverageFactory:" \
            " setting time to %f\n", time);
        }
        break;
      default:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverageFactory:" \
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
        "gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverageFactory::create:" \
        " No search area specified. Returning null.\n");
    }
    else
    {
      result = new area_coverage::PriorityWeightedRandomAreaCoverage (
        search_area, ACE_Time_Value (time),
        knowledge, platform, sensors, self, agents);
    }
  }

  return result;
}

/**
 * We precompute the total priority of the search area as this is a constant.
 * The total priority is calculated based on the priority and area of each 
 * individual region.
 */
gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverage::
PriorityWeightedRandomAreaCoverage (
  const string& search_id,
  const ACE_Time_Value& e_time,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents) :
  BaseAreaCoverage (knowledge, platform, sensors, self, agents, e_time),
  total_priority_ (0.0)
{
  // init status vars
  status_.init_vars (*knowledge, "pwrac", self->id.to_integer ());
  status_.init_variable_values ();

  // get search area
  search_area_.from_container (*knowledge, search_id);

  // calculate total priority
  const vector<utility::PrioritizedRegion>& regions =
    search_area_.get_regions ();
  for (unsigned int i = 0; i < regions.size (); ++i)
  {
    total_priority_ += regions[i].get_area () * regions[i].priority;
    priority_total_by_region_.push_back (total_priority_);
  }

  // generate first position to move
  generate_new_position ();
}

void
gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverage::
  operator= (const PriorityWeightedRandomAreaCoverage & rhs)
{
  if (this != &rhs)
  {
    this->search_area_ = rhs.search_area_;
    this->priority_total_by_region_ = rhs.priority_total_by_region_;
    this->total_priority_ = rhs.total_priority_;
    this->BaseAreaCoverage::operator= (rhs);
  }
}

/**
 * A new position is selected by selecting a random region based on the 
 * weighting of the region and the total search area priority. A random position
 * is then selected from that region.
 */
void
gams::algorithms::area_coverage::PriorityWeightedRandomAreaCoverage::
  generate_new_position ()
{
  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    // select region
    double selected_rand = madara::utility::rand_double (0.0, total_priority_);
    const utility::PrioritizedRegion* selected_region = 0;
    for (unsigned int i = 0; i < search_area_.get_regions ().size (); ++i)
    {
      if (priority_total_by_region_[i] > selected_rand)
      {
        selected_region = &((search_area_.get_regions ())[i]);
        break;
      }
    }

    // select point in region
    do
    {
      next_position_.latitude (madara::utility::rand_double (selected_region->min_lat_,
        selected_region->max_lat_));
      next_position_.longitude (madara::utility::rand_double (selected_region->min_lon_,
        selected_region->max_lon_));
      next_position_.altitude (madara::utility::rand_double (selected_region->min_alt_,
        selected_region->max_alt_));
    } while (!selected_region->contains (next_position_));

    // found an acceptable position, so set it as next
    utility::GPSPosition current;
    current.from_container (self_->agent.location);
    next_position_.altitude (current.altitude ());

    initialized_ = true;
  }
}
