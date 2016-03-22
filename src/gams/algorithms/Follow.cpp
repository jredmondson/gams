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
 */

/**
 * @file Follow.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the definition of the Follow algorithm
 */

#include "gams/algorithms/Follow.h"

#include <sstream>
#include <iostream>
#include <limits.h>
using std::cerr;
using std::endl;

#include "gams/utility/GPSPosition.h"

#include "gams/utility/ArgumentParser.h"

using std::stringstream;

namespace knowledge = madara::knowledge;
typedef knowledge::KnowledgeRecord  KnowledgeRecord;
typedef KnowledgeRecord::Integer   Integer;
typedef knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::FollowFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result (0);

  // set defaults
  std::string target;
  Integer delay (Integer (5));

  if (knowledge && platform && self)
  {
  for (KnowledgeMap::const_iterator i = args.begin (); i != args.end (); ++i)
  {
    if (i->first.size () <= 0)
      continue;

    switch (i->first[0])
    {
    case 'd':
      if (i->first == "delay")
      {
        delay = i->second.to_double ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FollowFactory:" \
          " setting delay to %d\n", (int)delay);

        break;
      }
    case 't':
      if (i->first == "target")
      {
        target = i->second.to_string ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FollowFactory:" \
          " setting target to %s\n", target.c_str ());

        break;
      }
    default:
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::FollowFactory:" \
        " argument unknown: %s -> %s\n",
        i->first.c_str (), i->second.to_string ().c_str ());
      break;
    }
  }

  // if group has not been set, use the swarm
  if (target == "")
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::FollowFactory::create:" \
      " No target specified. Returning null.\n");
  }
  else
  {
    result = new Follow (target, delay, knowledge, platform, sensors, self);
  }
}

return result;
}

gams::algorithms::Follow::Follow (
  const std::string & id,
  madara::knowledge::KnowledgeRecord::Integer delay,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self) :
  BaseAlgorithm (knowledge, platform, sensors, self), next_position_ (DBL_MAX),
  delay_ ((size_t)delay)
{
  status_.init_vars (*knowledge, "follow", self->id.to_integer ());
  status_.init_variable_values ();

  target_location_.set_name (id + ".location", *knowledge, 3);
}

gams::algorithms::Follow::~Follow ()
{
}

void
gams::algorithms::Follow::operator= (const Follow & rhs)
{
  if (this != &rhs)
  {
    this->BaseAlgorithm::operator= (rhs);
    this->target_location_ = rhs.target_location_;
    this->next_position_ = rhs.next_position_;
    this->previous_locations_ = rhs.previous_locations_;
    this->delay_ = rhs.delay_;
  }
}

/**
 * The agent gets the target's location from the database and adds it to the
 * queue of positions being stored.
 */
int
gams::algorithms::Follow::analyze (void)
{
  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Follow::analyze:" \
      " Platform initialized. Calculating distances.\n");

    static utility::GPSPosition prev;
    utility::GPSPosition current;
    current.from_container (target_location_);

    // if target agent has moved
    if (current.distance_to (prev) > 1.0)
    {
      previous_locations_.push (current);
      prev = current;
    }

    ++executions_;
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Follow::analyze:" \
      " Platform not initialized. Unable to analyze.\n");
  }
  return OK;
}
      
/**
 * Move to next location if next_position_ is valid
 */
int
gams::algorithms::Follow::execute (void)
{
  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Follow::execute:" \
      " Platform initialized. Executing next movement.\n");

    if (next_position_.latitude () != DBL_MAX)
      platform_->move (next_position_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Follow::execute:" \
      " Platform not initialized. Unable to execute.\n");
  }
  return 0;
}

/**
 * Store locations in the queue up to the delay amount
 */
int
gams::algorithms::Follow::plan (void)
{
  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Follow::plan:" \
      " Platform initialized. Plotting next position.\n");

    if (previous_locations_.size () == delay_)
    {
      next_position_ = previous_locations_.front ();
      previous_locations_.pop ();
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Follow::plan:" \
      " Platform not initialized. Unable to plan.\n");
  }
  return 0;
}
