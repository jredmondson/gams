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
 */

/**
 * @file Follow.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the Follow algorithm
 */

#include "gams/algorithms/Follow.h"

#include <sstream>
#include <iostream>
#include <limits.h>
#include <math.h>

#include "gams/utility/ArgumentParser.h"

using std::stringstream;

namespace knowledge = madara::knowledge;
typedef knowledge::KnowledgeRecord  KnowledgeRecord;
typedef KnowledgeRecord::Integer   Integer;
typedef knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::FollowFactory::create(
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result(0);

  // set defaults
  std::string target;
  std::vector <double> offset;
  

  if (knowledge && platform && self)
  {
  for (KnowledgeMap::const_iterator i = args.begin(); i != args.end(); ++i)
  {
    if (i->first.size() <= 0)
      continue;

    switch (i->first[0])
    {
    case 'o':
      if (i->first == "offset")
      {
        offset = i->second.to_doubles();

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::FollowFactory:" \
          " %d size offset set\n",(int)offset.size());
        break;
      }
      goto unknown;
    case 't':
      if (i->first == "target")
      {
        target = i->second.to_string();

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::FollowFactory:" \
          " setting formation head/target to %s\n", target.c_str());
        break;
      }
      goto unknown;
    unknown:
    default:
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::FollowFactory:" \
        " argument unknown: %s -> %s\n",
        i->first.c_str(), i->second.to_string().c_str());
      break;
    }
  }

  // if group has not been set, use the swarm
  if (target == "")
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::FollowFactory::create:" \
      " No target specified. Returning null.\n");
  }
  else
  {
    result = new Follow(target, offset, knowledge, platform, sensors, self);
  }
}

return result;
}

gams::algorithms::Follow::Follow(
  const std::string & target,
  const std::vector <double> & offset,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self) :
  BaseAlgorithm(knowledge, platform, sensors, self), offset_(offset),
  had_valid_dest_orientation_(false)
{
  if (knowledge && platform && sensors && self)
  {
    status_.init_vars(*knowledge, "follow", self->agent.prefix);
    status_.init_variable_values();

    // initialize leader's variables
    target_.init_vars(*knowledge, target);

    // resize to provide a 
    if (offset_.size() < 3)
    {
      offset_.resize(3, 0);
    }
  }
}

gams::algorithms::Follow::~Follow()
{
}

void
gams::algorithms::Follow::operator=(const Follow & rhs)
{
  if (this != &rhs)
  {
    this->BaseAlgorithm::operator=(rhs);
    this->target_ = rhs.target_;
    this->target_location_ = rhs.target_location_;
    this->target_destination_ = rhs.target_destination_;
    this->last_location_ = rhs.last_location_;
    this->target_last_location_ = rhs.target_last_location_;
    this->offset_ = rhs.offset_;
    this->need_move_ = rhs.need_move_;
    this->had_valid_dest_orientation_ = rhs.had_valid_dest_orientation_;
  }
}

/**
 * The agent gets the target's location from the database and adds it to the
 * queue of positions being stored.
 */
int
gams::algorithms::Follow::analyze(void)
{
  if (self_)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Follow::analyze:" \
      " current pose is [%s, %s].\n",
      self_->agent.location.to_record().to_string().c_str(),
      self_->agent.orientation.to_record().to_string().c_str());
  }

  if (platform_ && *platform_->get_platform_status()->movement_available)
  {
    const pose::ReferenceFrame * platform_frame =
      &(platform_->get_location().frame());

    // initialize location and orientation frames
    last_location_.frame(platform_->get_location().frame());
    target_last_location_.frame(platform_->get_location().frame());
    target_destination_.frame(platform_->get_location().frame());
    last_target_destination_.frame(platform_->get_location().frame());
    target_location_.frame(platform_->get_location().frame());
    target_orientation_.frame(platform_->get_location().frame());

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Follow::analyze:" \
      " Platform initialized. Calculating if move is needed.\n");

    // check if target location is set correctly
    if (target_.location.to_record().to_doubles().size() >= 2)
    {
      // import target location and destination
      target_location_.from_container(
        target_.location);
      target_destination_.from_container(target_.dest);
      target_orientation_.from_container(target_.orientation);

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Follow::analyze:" \
        " Execute is going to want to move. Target at %s\n",
        target_location_.to_string().c_str());

      need_move_ = true;
    }
    else
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Follow::analyze:" \
        " Target location is invalid(%s->%s). No movement needed yet.\n",
        target_.location.get_name().c_str(),
        target_.location.to_record().to_string().c_str());

      need_move_ = false;
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
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
gams::algorithms::Follow::execute(void)
{
  if (platform_ && *platform_->get_platform_status()->movement_available)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Follow::execute:" \
      " Platform initialized. Executing next movement.\n");

    if (need_move_)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Follow::execute:" \
        " Target has location. Moving.\n");

      gams::pose::ReferenceFrame target_frame;

      if (target_.dest.to_record().size() >= 2)
      {
        // if the target destination has been set by the platform, try to adjust
        // formation according to the intended next position of the platform

        double dest_radians = atan2(
          target_destination_.y() - target_location_.y(),
          target_destination_.x() - target_location_.x());

        gams::pose::Orientation dest_orientation(0, 0, dest_radians);

        target_frame = gams::pose::ReferenceFrame(
          gams::pose::Pose(target_location_, dest_orientation));
      }
      else
      {
        // by default use a pose of the target location with a default orientation
        target_frame = gams::pose::ReferenceFrame(
          gams::pose::Pose(target_location_,
          gams::pose::Orientation(0,0,0)));
      }

      // the destination is modified by the row, column and buffer size
      gams::pose::Position destination(
        target_frame, offset_[0], offset_[1], offset_[2]);

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Follow::execute:"
        " moving to position %s.\n",
        destination.to_string().c_str());

      // move to new destination
      platform_->move(destination, platform_->get_accuracy());

      // keep track of last location seen
      last_location_.from_container(self_->agent.location);
      last_target_destination_ = target_destination_;

      ++executions_;
    }
    else
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Follow::execute:" \
        " Target location is invalid(%s->%s). No movement needed yet.\n",
        target_.location.get_name().c_str(),
        target_.location.to_record().to_string().c_str());
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::Follow::execute:" \
      " ERROR: Platform not initialized. Unable to execute.\n");
  }
  return 0;
}

/**
 * Nothing to do in planning stage
 */
int
gams::algorithms::Follow::plan(void)
{
  return 0;
}
