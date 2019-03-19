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
 * @file MinTimeAreaCoverage.cpp
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
#include "gams/algorithms/area_coverage/MinTimeAreaCoverage.h"

#include "gams/utility/GPSPosition.h"
#include "gams/utility/Position.h"

#include <iostream>
#include <cmath>
#include <string>
#include <set>
#include <map>

#include "gams/utility/ArgumentParser.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::MinTimeAreaCoverageFactory::create(
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
    "MinTimeAreaCoverageFactory::create" \
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
            "MinTimeAreaCoverageFactory::create:" \
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
            "MinTimeAreaCoverageFactory::create:" \
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
            "MinTimeAreaCoverageFactory::create:" \
            " setting time to %f\n", time);
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "MinTimeAreaCoverageFactory::create:" \
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
        "MinTimeAreaCoverageFactory::create:" \
        " No search area specified. Returning null.\n");
    }
    else
    {
      result = new area_coverage::MinTimeAreaCoverage(
        search_area, time,
        knowledge, platform, sensors, self, agents);
    }
  }

  return result;
}

gams::algorithms::area_coverage::MinTimeAreaCoverage::
  MinTimeAreaCoverage(
  const std::string & search_id, double e_time, 
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents,
  const std::string & algo_name) :
  BaseAreaCoverage(knowledge, platform, sensors, self, agents, e_time),
  min_time_(search_id + "." + algo_name, knowledge)
{
  // init status vars
  status_.init_vars(*knowledge, algo_name, self->agent.prefix);
  status_.init_variable_values();

  // get search area
  search_area_.from_container(*knowledge, search_id);

  // fill out min_time_ sensor
  /**
   * The sensor origin and radius should be setup by the human controller and
   * not by the individual agents. As we do not currently have a human 
   * controller infrastructure yet, this will have to do. When the controller
   * is in place, the set_range, set_origin should not be called by the agents.
   */
  utility::GPSPosition origin;
  madara::knowledge::containers::NativeDoubleArray origin_container;
  origin_container.set_name("sensor.coverage.origin", *knowledge, 3);
  origin.from_container(origin_container);
  min_time_.set_origin(origin);
  min_time_.set_range(2.5); // balance this between resolution and performance

  // perform setup
  /**
   * In this algorithm, individual agents will increment their local copies of
   * the sensor map to limit the amount of communication required.
   */
  valid_positions_ = min_time_.discretize(search_area_);
  static const madara::knowledge::KnowledgeUpdateSettings
    NO_BROADCAST(true, false);
  knowledge_->lock();
  for (std::set<utility::Position>::iterator it = valid_positions_.begin();
    it != valid_positions_.end(); ++it)
  {
    min_time_.set_value(*it, min_time_.get_value(*it) + 1, NO_BROADCAST);
  }
  knowledge_->unlock();

  // find first position to go to
  generate_new_position();
}

void
gams::algorithms::area_coverage::MinTimeAreaCoverage::operator=(
  const MinTimeAreaCoverage & rhs)
{
  if (this != &rhs)
  {
    this->search_area_ = rhs.search_area_;
    this->min_time_ = rhs.min_time_;
    this->valid_positions_ = rhs.valid_positions_;
    this->BaseAreaCoverage::operator=(rhs);
  }
}

int
gams::algorithms::area_coverage::MinTimeAreaCoverage::analyze(void)
{
  ++executions_;

  // increment time since last seen for all cells
  /**
   * As with the initial setup, we don't broadcast our sensor map updates to
   * limit the communications.
   */
  static const madara::knowledge::KnowledgeUpdateSettings
    NO_BROADCAST(true, false);
  knowledge_->lock();
  for (std::set<utility::Position>::iterator it = valid_positions_.begin();
    it != valid_positions_.end(); ++it)
  {
    min_time_.set_value(*it, min_time_.get_value(*it) + 1, NO_BROADCAST);
  }
  knowledge_->unlock();

  // mark current position as seen
  utility::GPSPosition current;
  current.from_container(self_->agent.location);
  /**
   * However, we do need to communicate when we reset a time value for out
   * current location. Note that due to lack of synchronization, this value 
   * could be changed to 1 on other agents before it is actually considered for
   * utility calculations. This is inconsequential.
   */
  min_time_.set_value(current, 0);
  position_value_map_.erase(min_time_.get_index_from_gps(current));
  
  return check_if_finished(OK);
}

void
gams::algorithms::area_coverage::MinTimeAreaCoverage::
  generate_new_position(void)
{
  if (platform_ && *platform_->get_platform_status()->movement_available)
  {
    // perform check for actually hitting cells
    review_last_move();
    last_generation_ = executions_;

    // check each possible destination for max utility
    double max_util = -DBL_MAX;
    std::set<utility::Position> online;
    utility::GPSPosition current;
    current.from_container(self_->agent.location);
    next_position_ = current;
    utility::Position cur_index = min_time_.get_index_from_gps(current);
    for (std::set<utility::Position>::const_iterator it = valid_positions_.begin();
      it != valid_positions_.end(); ++it)
    {
      std::set<utility::Position> cur_online;
      double util = get_utility(cur_index, *it, cur_online);
      if (util > max_util)
      {
        max_util = util;
        next_position_ = min_time_.get_gps_from_index(*it);
        next_position_.altitude(self_->agent.desired_altitude.to_double());
        online.swap(cur_online);
      }
    }

    /**
     * Here we 0 out the cells along the line from our current cell to our
     * destination cell. Importantly, we also store the values that we are
     * clearing. Once the move is complete, we will check if we actually hit the
     * cells and update them if we did not.
     */
    for (std::set<utility::Position>::iterator it = online.begin();
      it != online.end(); ++it)
    {
      position_value_map_[*it] = min_time_.get_value(*it);
      min_time_.set_value(*it, 0.0);
    }

    initialized_ = true;
  }
}

double
gams::algorithms::area_coverage::MinTimeAreaCoverage::get_utility(
  const utility::Position& start, const utility::Position& end,
  std::set<utility::Position>& online)
{
  /**
   * check each valid position and add its value to utility if it is along
   * the possible travel path of the agent
   */
  double util = 0.0;
  const double radius =
    min_time_.get_range() / min_time_.get_discretization();
  for (std::set<utility::Position>::const_iterator it = valid_positions_.begin();
    it != valid_positions_.end(); ++it)
  {
    if (start.distance_to_2d(end, *it) < radius)
    {
      double time = min_time_.get_value(*it);
      double delta_util = pow(time, 3.0);
      util += delta_util;
      online.insert(*it);
    }
  }
  
  // modify the utility based on the distance that will be travelled
  return util / sqrt(start.distance_to_2d(end) + 1);
}

void
gams::algorithms::area_coverage::MinTimeAreaCoverage::review_last_move()
{
  /**
   * We need to check that we actually hit the cells we said we would hit. If we
   * did not, then we reset the map to what it would have been had it not been
   * reset. We also allow for the possibility that other agents coincidentally 
   * observed a cell that we were going to.
   */
  double expected = executions_ - last_generation_;
  for (std::map<utility::Position, double>::const_iterator it = 
    position_value_map_.begin(); it != position_value_map_.end();
    ++it)
  {
    if (min_time_.get_value(it->first) == expected)
      min_time_.set_value(it->first,
        min_time_.get_value(it->first) + it->second);
  }

  position_value_map_.clear();
}

#endif
