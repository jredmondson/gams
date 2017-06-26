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
#include "gams/loggers/GlobalLogger.h"
#include "PerimeterPatrol.h"

#include "ace/OS_NS_sys_time.h"

#include <string>
#include <iostream>
#include <vector>
#include <sstream>

#include "gams/utility/ArgumentParser.h"

namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

typedef knowledge::KnowledgeRecord KnowledgeRecord;
typedef KnowledgeRecord::Integer   Integer;
typedef knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::PerimeterPatrolFactory::create (
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
    "gams::algorithms::PerimeterPatrolFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    std::string search_area;
    double max_time = -1;
    bool counter = false;

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
            "gams::algorithms::PerimeterPatrolFactory:" \
            " setting search_area to %s\n", search_area.c_str ());
          break;
        }
        goto unknown;
      case 'c':
        if (i->first == "counter" || i->first == "counter_clockwise")
        {
          counter = true;

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::PerimeterPatrolFactory:" \
            " enabling counter-clockwise patrol\n");
          break;
        }
        goto unknown;
      case 'm':
        if (i->first == "max_time")
        {
          max_time = i->second.to_double ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::PerimeterPatrolFactory:" \
            " setting max time to %f\n", max_time);
          break;
        }
        goto unknown;
      case 's':
        if (i->first == "search_area")
        {
          search_area = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::PerimeterPatrolFactory:" \
            " setting search_area to %s\n", search_area.c_str ());
          break;
        }
        goto unknown;
      case 't':
        if (i->first == "time")
        {
          max_time = i->second.to_double ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::PerimeterPatrolFactory:" \
            " setting max time to %f\n", max_time);
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::PerimeterPatrolFactory:" \
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
        "gams::algorithms::PerimeterPatrolFactory::create:" \
        " No search area specified. Returning null.\n");
    }
    else
    {
      result = new PerimeterPatrol (
        search_area, max_time, counter,
        knowledge, platform, sensors, self, agents);
    }
  }

  return result;
}

gams::algorithms::PerimeterPatrol::PerimeterPatrol (
  const std::string & area,
  double max_time,
  bool counter,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents) :
  BaseAlgorithm (knowledge, platform, sensors, self, agents),
  area_ (area), max_time_ (max_time), counter_ (counter), move_index_ (0),
  initialized_ (false)
{
  status_.init_vars (*knowledge, "patrol", self->agent.prefix);
  status_.init_variable_values ();
}

gams::algorithms::PerimeterPatrol::~PerimeterPatrol ()
{
}

void
gams::algorithms::PerimeterPatrol::operator= (const PerimeterPatrol & rhs)
{
  if (this != &rhs)
  {
    this->area_ = rhs.area_;
    this->counter_ = rhs.counter_;
    this->max_time_ = rhs.max_time_;
    this->end_time_ = rhs.end_time_;
    this->locations_ = rhs.locations_;
    this->move_index_ = rhs.move_index_;

    this->BaseAlgorithm::operator=(rhs);
  }
}

int
gams::algorithms::PerimeterPatrol::analyze (void)
{
  int result (OK);

  if (initialized_ && max_time_ > 0 && ACE_OS::gettimeofday () > end_time_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "PerimeterPatrol::analyze:" \
      " patrol has timed out. Setting finished to true.\n");

    status_.finished = 1;
  }
  else if (initialized_ && status_.finished.is_false ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "PerimeterPatrol::analyze:" \
      " currently moving to location %d -> [%s].\n",
      (int)move_index_,
      locations_[move_index_].to_string ().c_str ());

    // check our distance to the next location
    pose::Position loc = platform_->get_location ();
    pose::Position next_loc = locations_[move_index_];

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "PerimeterPatrol::analyze:" \
      " distance between points is %f (need %f accuracy)\n",
      loc.distance_to (next_loc), platform_->get_accuracy ());

    if (loc.approximately_equal (next_loc, platform_->get_accuracy ()))
    {
      if (!counter_)
      {
        if (move_index_ == locations_.size () - 1)
        {
          move_index_ = 0;
        }
        else
        {
          ++move_index_;
        }
      }
      else
      {
        if (move_index_ == 0)
        {
          move_index_ = locations_.size () - 1;
        }
        else
        {
          --move_index_;
        }
      }

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "PerimeterPatrol::analyze:" \
        " reached target location. Moving to location %d -> %s\n",
        (int)move_index_,
        locations_[move_index_].to_string ().c_str ());
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "PerimeterPatrol:analyze:" \
      " locations list is not yet initialized. Nothing to do.\n");
  }

  if (status_.finished == 1)
  {
    result |= FINISHED;
  }

  return result;
}

int
gams::algorithms::PerimeterPatrol::execute (void)
{
  int result (OK);

  bool is_finished = status_.finished == 1;

  if (initialized_ && !is_finished)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "PerimeterPatrol::execute:" \
      " calling platform->move(\"%s\")\n",
      locations_[move_index_].to_string ().c_str ());

    pose::Position next = locations_[move_index_];

    platform_->move (next);
  }
  else if (is_finished)
  {
    result |= OK;
  }

  return result;
}

int
gams::algorithms::PerimeterPatrol::plan (void)
{
  int result (OK);

  if (!initialized_)
  {
    generate_locations ();
  }

  if (status_.finished.is_true ())
  {
    result |= FINISHED;
  }

  return result;
}

void
gams::algorithms::PerimeterPatrol::generate_locations (void)
{
  // if we need to generate locations from region, do so
  if (locations_.size () == 0)
  {
    // get the area from the knowledge base
    pose::SearchArea sa;
    sa.from_container (*knowledge_, area_);

    // get a bounding box around the regions
    pose::Region hull = sa.get_convex_hull ();

    locations_ = hull.vertices;

    for (pose::Position v : locations_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "PerimeterPatrol::generate_locations:" \
        " hull point: %s\n",
        v.to_string().c_str());
    }
  }

  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    // find closest location
    move_index_ = 0;

    pose::Position agent_location (platform_->get_frame());
    agent_location.from_container (self_->agent.location);

    // find distance to default position
    double min_dist = agent_location.distance_to (locations_[move_index_]);

    // find minimum distance available to a patrol location
    for (size_t i = 1; i < locations_.size (); ++i)
    {
      double cur_dist = agent_location.distance_to (locations_[i]);
      if (min_dist > cur_dist)
      {
        cur_dist = min_dist;
        move_index_ = i;
      }
    }

    if (max_time_ >= 0)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "PerimeterPatrol::generate_locations:" \
        " max_time has been set to %f. Setting timer.\n",
        max_time_);

      ACE_Time_Value delay;
      delay.set (max_time_);
      end_time_ = ACE_OS::gettimeofday () + delay;
    }

    initialized_ = true;
  }
}
